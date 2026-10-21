#pragma once

// C5VRX receive-chain emulation (--c5): turns the HackRF's 8-bit IQ into what
// a C5VRX-5 demodulator sees, so the 4-bit/FPGA ideas can be judged on real
// VTX signals before any hardware exists.
//
//   HackRF IQ (DC removed, analog gain held with headroom)
//   -> digital gain (dB, Direct Gain V5 decisions, sample-exact timing)
//   -> 10-bit ADC codes, rails at -512..511
//   -> "ultrafine" lane: ADC bits {9,6,5,4} as a signed 4-bit cell. Bits 8..7
//      are dropped, so |code| >= 128 wraps exactly like the C5 lane does.
//   -> cell centres (2n+1)/2, the representation Direct Gain V5 measures.
//
// Direct Gain V5 (C5VRX v5/main/gain.c) carried over:
//   - a 200 us observation cadence, each window ~102 us of IQ;
//   - P50/P90/P95 of the centred Q4 power, rail (clip) and near-origin
//     fractions, Phase8 adjacent-sample coherence;
//   - healthy band P50 13..32 = zero-write zone; weak target 17, high 27;
//   - severe excursions act on one window, moderate ones need two windows
//     and 20 ms; two direction reversals within 20 ms damp decisions to 8
//     windows for 200 ms;
//   - saturation shorter than 1 ms is an interference burst and ignored;
//     longer saturation drops gain at once (staged recovery afterwards);
//   - no carrier: listen at the maximum gain;
//   - 300 us settle guard after every write.
// Experimental, off by default (--c5-wrap; not in V5): lane-wrap detection.
// Hypothesis from a HackRF run (2026-10-12, E1): 70-100 writes a second at
// -3 dB with P50 ~9, P95 ~41, coherence ~15, healthy later at -11 dB. Codes
// beyond +-128 fold back into small, scrambled cells and could read as weak
// noise. Rule: low coherence (< 30), a large envelope (P95 >= 35) and few
// near-origin cells (noise at the listening gain has P95 3..13), lasting
// 1 ms (the burst gate's length), drops the gain 6 dB like an overload.
// Not confirmed: a clean carrier never gets trapped at any level or start
// gain (tests/test_c5_emu.cpp, C5_SWEEP=1), and that run had 200-900 burst
// windows a second on E1, a Wi-Fi channel. Sustained Wi-Fi meets the rule
// too, so compare on E1 and on F3-F5 before trusting it.
// Listening gain, auto by default (--c5-maxgain pins it): the C5's table
// maximum shows plain noise at P50 3..5 (board, VTX off: P50 3/5, coherence
// 18/25). The HackRF's noise depends on its LNA/VGA and the unit, so while
// only noise arrives at the listening gain (coherence < 30, P95 <= 20, not
// settling) the mean P50 of 50 windows (10 ms) moves that gain 1 dB toward
// P50 3..5. A carrier never adjusts it. The step is silent (no write, no
// transient): it calibrates the emulation, it is not a gain decision.
// Simplified: the vendor gain tuples become a continuous dB value. An
// emergency drop is -6 dB (one baseband halving), an adjacent physical step
// +-1 dB, and the learned gain map is not modelled. A write also replaces
// the next ~2 us of IQ with a near-origin transient, as a PHY write does.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>

namespace famidec {

struct C5EmuStats {
    std::atomic<float> gain_db{0.0f}, listen_db{0.0f};
    std::atomic<int> p50{0}, p95{0}, coherence{0}, clip_pm{0}, origin_pm{0};
    std::atomic<uint32_t> writes{0}, severe{0}, moderate{0}, overloads{0},
        bursts{0}, damp_events{0}, holds{0}, windows{0}, wraps{0};
};

class C5Emulator {
public:
    C5Emulator(double sample_rate, double max_gain_db, double min_gain_db,
               C5EmuStats* stats, bool wrap_detect = false, bool auto_listen = false)
        : fs_(sample_rate), max_db_(max_gain_db), min_db_(min_gain_db),
          stats_(stats), auto_listen_(auto_listen), wrap_detect_(wrap_detect) {
        if (stats_) stats_->listen_db.store(static_cast<float>(max_db_));
        period_ = std::max<size_t>(16, static_cast<size_t>(std::lround(fs_ * 200e-6)));
        measure_ = std::min(period_, std::max<size_t>(16, static_cast<size_t>(std::lround(fs_ * 102.4e-6))));
        transient_ = static_cast<size_t>(std::lround(fs_ * 2e-6));
        second_diff_ = fs_ < 39e6;
        gain_db_ = max_db_;
        apply_gain();
        build_tables();
    }

    // In place: x holds DC-free complex IQ scaled so +-1 is the HackRF's
    // 8-bit full scale. Output: centred 4-bit cells scaled to +-~1.
    void process(std::complex<float>* x, size_t n) {
        for (size_t k = 0; k < n; ++k) {
            const int ci = adc(x[k].real()), cq = adc(x[k].imag());
            int i4 = lane(ci), q4 = lane(cq);
            if (transient_left_) { --transient_left_; i4 = 0; q4 = 0; }
            if (pos_ < measure_) observe(i4, q4);
            x[k] = {(2 * i4 + 1) / 16.0f, (2 * q4 + 1) / 16.0f};
            if (++pos_ >= period_) {
                pos_ = 0;
                window_end();
            }
            ++sample_;
        }
    }

    float gain_db() const { return static_cast<float>(gain_db_); }

private:
    // ---- quantisation ------------------------------------------------------
    void apply_gain() { scale_ = 512.0 * std::pow(10.0, gain_db_ / 20.0); }
    int adc(float v) const {
        long c = std::lround(v * scale_);
        return static_cast<int>(std::clamp(c, -512L, 511L));
    }
    static int lane(int code) {
        const unsigned u = static_cast<unsigned>(code) & 0x3ffu;
        return ((u >> 9) & 1u ? -8 : 0) + static_cast<int>((u >> 4) & 7u);
    }

    // ---- observation (dg_measure) ------------------------------------------
    void build_tables() {
        for (int i = -8; i < 8; ++i)
            for (int q = -8; q < 8; ++q) {
                const int ci = 2 * i + 1, cq = 2 * q + 1;
                const size_t idx = static_cast<size_t>((i + 8) * 16 + (q + 8));
                power_[idx] = static_cast<uint8_t>((ci * ci + cq * cq + 2) / 4);
                double a = std::atan2(static_cast<double>(cq), static_cast<double>(ci));
                phase_[idx] = static_cast<uint8_t>(
                    static_cast<int>(std::lround(a / (2.0 * M_PI) * 256.0)) & 255);
            }
    }
    void observe(int i4, int q4) {
        const size_t idx = static_cast<size_t>((i4 + 8) * 16 + (q4 + 8));
        const uint8_t p = power_[idx], ph = phase_[idx];
        ++hist_[std::min<size_t>(p, 113)];
        origin_ += p <= 4;
        clip_ += (i4 == -8 || i4 == 7 || q4 == -8 || q4 == 7);
        // Phase8 coherence: |adjacent phase step| <= 45 deg at the C5's
        // 40 MS/s. Below that rate a carrier turns too far per sample, so the
        // step is compared with the previous step instead (second
        // difference): a carrier still scores high, noise stays at ~25 %.
        int d = ((static_cast<int>(ph) - static_cast<int>(prev_phase_) + 128) & 255) - 128;
        int test = second_diff_ ? (((d - prev_step_ + 128) & 255) - 128) : d;
        if (count_ > (second_diff_ ? 1u : 0u) && p >= 8 && std::abs(test) <= 32) ++coherent_;
        prev_step_ = d;
        prev_phase_ = ph;
        ++count_;
    }

    struct Obs {
        int p50 = 0, p90 = 0, p95 = 0, clip_pm = 0, origin_pm = 0, coherence = 0;
    };
    Obs take_obs() {
        Obs o;
        const size_t n = count_;
        size_t cum = 0;
        const size_t r50 = (n + 1) / 2, r90 = (n * 90 + 99) / 100, r95 = (n * 95 + 99) / 100;
        bool h50 = false, h90 = false;
        for (int p = 0; p < 114; ++p) {
            cum += hist_[static_cast<size_t>(p)];
            if (!h50 && cum >= r50) { o.p50 = p; h50 = true; }
            if (!h90 && cum >= r90) { o.p90 = p; h90 = true; }
            if (cum >= r95) { o.p95 = p; break; }
        }
        if (n) {
            o.clip_pm = static_cast<int>(clip_ * 1000 / n);
            o.origin_pm = static_cast<int>(origin_ * 1000 / n);
            o.coherence = n > 1 ? static_cast<int>(coherent_ * 100 / (n - 1)) : 0;
        }
        hist_.fill(0);
        count_ = origin_ = clip_ = coherent_ = 0;
        return o;
    }

    // ---- decisions (dg_tick) ------------------------------------------------
    static constexpr int kBandLo = 13, kBandHi = 32, kBandP95 = 65, kBandClipPm = 20;
    static constexpr int kWeakTarget = 17, kHighTarget = 27, kHighP90 = 53, kHighP95 = 72;
    static constexpr int kSchmittHi = 30, kSchmittP90 = 47, kSchmittP95 = 65, kSchmittLo = 14;
    static constexpr double kReversalUs = 20000, kDampUs = 200000, kModerateUs = 20000,
                            kBurstUs = 1000, kSettleUs = 300;

    static bool carrier(const Obs& o) {
        return o.coherence >= 55 && o.p50 >= 5 && o.origin_pm < 650;
    }
    static bool healthy(const Obs& o) {
        return carrier(o) && o.p50 >= kBandLo && o.p50 <= kBandHi && o.p95 <= kBandP95 &&
               o.clip_pm < kBandClipPm && o.origin_pm <= 250;
    }
    static double power_db(int p) { return 10.0 * std::log10(std::clamp(p, 1, 113)); }

    void window_end() {
        const double now = (static_cast<double>(sample_) + 1.0) / fs_ * 1e6;  // us
        const Obs o = take_obs();
        publish(o);
        const bool no_carrier = o.p50 <= 4 && o.origin_pm >= 650 && o.coherence < 20;
        const bool saturated = o.clip_pm >= 100 || o.p95 >= 95;
        if (!saturated) sat_run_ = false;
        else if (!sat_run_) { sat_run_ = true; sat_since_ = now; }
        if (saturated && now - sat_since_ < kBurstUs) { bump(stats_ ? &stats_->bursts : nullptr); return; }
        const bool settling = now - write_us_ < kSettleUs;
        if (auto_listen_) calibrate_listening(o, saturated, settling);
        if (no_carrier) {
            if (settling) return;
            if (overload_recovery_ && gain_db_ < max_db_) { write(gain_db_ + 1.0, now); return; }
            high_w_ = weak_w_ = 0;
            last_dir_ = 0;
            write(max_db_, now);
            return;
        }
        const bool wrapped = wrap_detect_ && !saturated && o.coherence < 30 && o.p95 >= 35 &&
                             o.origin_pm < 400;
        if (!wrapped) wrap_run_ = false;
        else if (!wrap_run_) { wrap_run_ = true; wrap_since_ = now; }
        if (settling && !saturated) return;
        if (wrapped && now - wrap_since_ >= kBurstUs) {
            bump(stats_ ? &stats_->wraps : nullptr);
            high_w_ = weak_w_ = 0;
            overload_recovery_ = true;
            wrap_run_ = false;
            write(gain_db_ - 6.0, now);
            return;
        }
        if (wrapped) return;  // not yet 1 ms: maybe a burst, no other decision
        if (saturated) {
            bump(stats_ ? &stats_->overloads : nullptr);
            high_w_ = weak_w_ = 0;
            overload_recovery_ = true;
            write(gain_db_ - 6.0, now);
            return;
        }
        if (healthy(o)) {
            overload_recovery_ = false;
            high_w_ = weak_w_ = 0;
            last_dir_ = 0;
            bump(stats_ ? &stats_->holds : nullptr);
            return;
        }
        bool high = o.p50 > kBandHi || o.p90 >= kHighP90 || o.p95 > kHighP95 || o.clip_pm >= kBandClipPm;
        bool weak = o.p50 < kBandLo;
        if (last_dir_ == 2 && (o.p50 > kSchmittHi || o.p90 > kSchmittP90 || o.p95 > kSchmittP95)) high = true;
        if (last_dir_ == 1 && o.p50 < kSchmittLo) weak = true;
        if (high) {
            weak_w_ = 0;
            last_dir_ = 2;
            if (!high_w_) excursion_since_ = now;
            ++high_w_;
        } else if (weak) {
            high_w_ = 0;
            last_dir_ = 1;
            if (!weak_w_) excursion_since_ = now;
            ++weak_w_;
        } else {
            high_w_ = weak_w_ = 0;
            last_dir_ = 0;
            return;
        }
        const bool damped = now < damp_until_;
        const bool severe = high ? (o.clip_pm >= kBandClipPm || o.p95 > kHighP95 || o.p50 > kBandHi + 8)
                                 : (o.p50 + 4 < kBandLo);
        const unsigned need = damped ? 8u : severe ? 1u : 2u;
        if ((high && high_w_ < need) || (weak && weak_w_ < need)) return;
        if (!severe && now - excursion_since_ < kModerateUs) return;
        const double delta = std::clamp(power_db(weak ? kWeakTarget : kHighTarget) - power_db(o.p50), -12.0, 12.0);
        high_w_ = weak_w_ = 0;
        if (write(gain_db_ + delta, now))
            bump(stats_ ? (severe ? &stats_->severe : &stats_->moderate) : nullptr);
    }

    void calibrate_listening(const Obs& o, bool saturated, bool settling) {
        // Noise-like windows at one unchanged gain. Too quiet only counts at
        // the listening gain (raise it); too loud counts at any gain, since
        // loud noise makes V5 leave the listening gain (lower it).
        if (saturated || settling || o.coherence >= 30 || std::fabs(gain_db_ - noise_gain_) > 0.05) {
            noise_windows_ = 0;
            noise_p50_sum_ = 0;
            noise_gain_ = gain_db_;
            return;
        }
        noise_p50_sum_ += o.p50;
        if (++noise_windows_ < 50) return;
        const double mean = static_cast<double>(noise_p50_sum_) / noise_windows_;
        noise_windows_ = 0;
        noise_p50_sum_ = 0;
        const bool listening = std::fabs(gain_db_ - max_db_) < 0.05;
        double step = (mean < 3.0 && listening) ? 1.0 : mean > 5.0 ? -1.0 : 0.0;
        if (step == 0.0) return;
        max_db_ = std::clamp((step > 0 ? max_db_ : std::min(max_db_, gain_db_)) + step,
                             min_db_ + 10.0, 40.0);
        if (listening || gain_db_ > max_db_) gain_db_ = max_db_;
        noise_gain_ = gain_db_;
        apply_gain();
        if (stats_) {
            stats_->listen_db.store(static_cast<float>(max_db_), std::memory_order_relaxed);
            stats_->gain_db.store(static_cast<float>(gain_db_), std::memory_order_relaxed);
        }
    }

    bool write(double target_db, double now) {
        target_db = std::clamp(target_db, min_db_, max_db_);
        if (std::fabs(target_db - gain_db_) < 0.05) return false;
        const int dir = target_db > gain_db_ ? 1 : -1;
        if (last_write_dir_ && dir != last_write_dir_ && now - dir_write_us_ < kReversalUs) {
            if (++reversals_ >= 2) {
                damp_until_ = now + kDampUs;
                reversals_ = 0;
                bump(stats_ ? &stats_->damp_events : nullptr);
            }
        } else if (now - dir_write_us_ >= kReversalUs) {
            reversals_ = 0;
        }
        last_write_dir_ = dir;
        dir_write_us_ = now;
        gain_db_ = target_db;
        apply_gain();
        write_us_ = now;
        transient_left_ = transient_;
        bump(stats_ ? &stats_->writes : nullptr);
        if (stats_) stats_->gain_db.store(static_cast<float>(gain_db_), std::memory_order_relaxed);
        return true;
    }

    void publish(const Obs& o) {
        if (!stats_) return;
        stats_->p50.store(o.p50, std::memory_order_relaxed);
        stats_->p95.store(o.p95, std::memory_order_relaxed);
        stats_->coherence.store(o.coherence, std::memory_order_relaxed);
        stats_->clip_pm.store(o.clip_pm, std::memory_order_relaxed);
        stats_->origin_pm.store(o.origin_pm, std::memory_order_relaxed);
        stats_->gain_db.store(static_cast<float>(gain_db_), std::memory_order_relaxed);
        stats_->windows.fetch_add(1, std::memory_order_relaxed);
    }
    static void bump(std::atomic<uint32_t>* c) {
        if (c) c->fetch_add(1, std::memory_order_relaxed);
    }

    double fs_, max_db_, min_db_;
    C5EmuStats* stats_;
    size_t period_ = 0, measure_ = 0, transient_ = 0, transient_left_ = 0, pos_ = 0;
    uint64_t sample_ = 0;
    double gain_db_ = 0.0, scale_ = 512.0;
    std::array<uint8_t, 256> power_{}, phase_{};
    std::array<uint32_t, 114> hist_{};
    size_t count_ = 0, origin_ = 0, clip_ = 0, coherent_ = 0;
    uint8_t prev_phase_ = 0;
    int prev_step_ = 0;
    bool second_diff_ = false;
    // decision state
    bool auto_listen_ = false;
    unsigned noise_windows_ = 0, noise_p50_sum_ = 0;
    double noise_gain_ = 1e9;
    bool wrap_detect_ = true, wrap_run_ = false;
    double wrap_since_ = 0;
    bool sat_run_ = false, overload_recovery_ = false;
    double sat_since_ = 0, write_us_ = -1e9, excursion_since_ = 0, damp_until_ = 0, dir_write_us_ = -1e9;
    unsigned high_w_ = 0, weak_w_ = 0, reversals_ = 0;
    int last_dir_ = 0, last_write_dir_ = 0;
};

}  // namespace famidec
