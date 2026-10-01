#pragma once

// "Fast" RF gain control after the C5VRX Direct Gain design
// (github.com/Twotoz/C5VRX, docs/direct-gain-v3-core.md and the README's
// Direct Gain V4/V5 notes), adapted to the HackRF's LNA/VGA and USB cadence.
//
// What carries over from C5VRX:
//   - Judge the IQ amplitude distribution (P50/P95 of |IQ|), not one peak.
//   - Zero-write hold while P50 sits in a healthy band.
//   - Leaving the band: predict the dB that brings P50 back to the target
//     and jump there in one write.
//   - Overload (rail codes / high P95): immediate emergency gain-down, never
//     damped.
//   - Anti-hunt: two direction reversals in quick succession make the next
//     decisions need several consecutive out-of-band windows.
//   - After a write, ignore observations until the new gain has settled.
//   - Without a coherent carrier, listen at high gain: noise alone never
//     pulls the gain down (only overload does).
// What changes for the HackRF:
//   - One observation per USB transfer (~13 ms at 10 MSPS) instead of the
//     C5's 200 us / 1 ms windows.
//   - Gain is LNA (8 dB steps) + VGA (2 dB steps); VGA moves first, the LNA
//     only when the VGA would leave its range (C5VRX prefers its fine stage
//     the same way).
//   - The healthy band is set for 8-bit samples, not the C5's 4-bit lanes.
//   - Settling is counted in USB transfers: libhackrf keeps several transfers
//     in flight, so the first ones after a write still carry the old gain.
// The RF amp is never touched (intermod risk), as in the peak AGC.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace famidec {

struct IqStats {
    uint64_t seq = 0;         // USB transfer number this came from
    int p50 = 0, p95 = 0;     // percentiles of |IQ| (0..181)
    float clip_frac = 0.0f;   // fraction of components at the rail
    float coherence = 0.0f;   // 0 = noise .. 1 = steady carrier
    uint32_t n = 0;           // samples examined
};

// |z| for i,q in -128..127, by i*i + q*q (0..32768).
inline uint8_t iq_radius(int i, int q) {
    static const auto table = [] {
        std::array<uint8_t, 32769> t{};
        for (int k = 0; k <= 32768; ++k)
            t[static_cast<size_t>(k)] = static_cast<uint8_t>(
                std::min(181.0, std::floor(std::sqrt(static_cast<double>(k)) + 0.5)));
        return t;
    }();
    return table[static_cast<size_t>(i * i + q * q)];
}

// Statistics of one block of interleaved signed 8-bit IQ. `stride` is in
// complex samples; each probe also reads the following sample, for the
// lag-1 coherence.
inline IqStats compute_iq_stats(const uint8_t* data, size_t bytes,
                                size_t stride = 4) {
    IqStats s;
    const size_t samples = bytes / 2;
    if (samples < 2 || stride == 0) return s;
    std::array<uint32_t, 182> hist{};
    uint32_t n = 0, clips = 0;
    // Sums for the DC-removed lag-1 correlation. The HackRF's LO leakage puts
    // a DC term on everything, and a constant vector correlates perfectly
    // with itself, so noise would read as "coherent" without removing it.
    double sa_re = 0, sa_im = 0, sb_re = 0, sb_im = 0;
    double ab_re = 0, ab_im = 0, aa = 0, bb = 0;
    for (size_t k = 0; k + 1 < samples; k += stride) {
        int ai = static_cast<int8_t>(data[2 * k]);
        int aq = static_cast<int8_t>(data[2 * k + 1]);
        int bi = static_cast<int8_t>(data[2 * k + 2]);
        int bq = static_cast<int8_t>(data[2 * k + 3]);
        ++hist[iq_radius(ai, aq)];
        ++n;
        clips += (ai >= 127 || ai <= -128) + (aq >= 127 || aq <= -128);
        // b * conj(a): rotation from sample k to k+1
        sa_re += ai; sa_im += aq; sb_re += bi; sb_im += bq;
        ab_re += static_cast<double>(bi) * ai + static_cast<double>(bq) * aq;
        ab_im += static_cast<double>(bq) * ai - static_cast<double>(bi) * aq;
        aa += static_cast<double>(ai) * ai + static_cast<double>(aq) * aq;
        bb += static_cast<double>(bi) * bi + static_cast<double>(bq) * bq;
    }
    s.n = n;
    auto pct = [&](double p) {
        uint32_t want = static_cast<uint32_t>(p * n), acc = 0;
        for (int r = 0; r < 182; ++r) {
            acc += hist[static_cast<size_t>(r)];
            if (acc > want) return r;
        }
        return 181;
    };
    s.p50 = pct(0.50);
    s.p95 = pct(0.95);
    s.clip_frac = static_cast<float>(clips) / static_cast<float>(2 * n);
    // Subtract the common mean m from both: sum (b-m)(a-m)*.
    const double mr = (sa_re + sb_re) / (2.0 * n), mi = (sa_im + sb_im) / (2.0 * n);
    double cr = ab_re - (mr * sa_re + mi * sa_im) - (sb_re * mr + sb_im * mi)
                + n * (mr * mr + mi * mi);
    double ci = ab_im - (mi * sa_re - mr * sa_im) - (sb_im * mr - sb_re * mi);
    double pa = aa - 2.0 * (mr * sa_re + mi * sa_im) + n * (mr * mr + mi * mi);
    double pb = bb - 2.0 * (mr * sb_re + mi * sb_im) + n * (mr * mr + mi * mi);
    double den = std::sqrt(std::max(pa, 0.0) * std::max(pb, 0.0));
    s.coherence = den > 0 ? static_cast<float>(std::hypot(cr, ci) / den) : 0.0f;
    return s;
}

struct GainSetting {
    int lna = 0;  // 0..40, step 8
    int vga = 0;  // 0..62, step 2
    int total() const { return lna + vga; }
    bool operator==(const GainSetting& o) const {
        return lna == o.lna && vga == o.vga;
    }
};

struct FastGainParams {
    int p50_lo = 24, p50_hi = 48;    // healthy band of the median |IQ|
    double p50_target = 34.0;        // geometric middle of the band
    int p95_max = 118;               // P95 at or above = near the rail
    float clip_frac_max = 1e-3f;     // rail components that count as overload
    float coherence_min = 0.2f;      // carrier present. FM with +-2.5 MHz
                                     // deviation reads ~0.47, white noise
                                     // (DC removed) < 0.1. Check on hardware:
                                     // the HackRF baseband filter colours noise.
    int settle_transfers = 5;        // libhackrf keeps 4 transfers in flight
    double reversal_window_s = 0.3;  // two reversals inside this -> damp
    double damp_s = 1.5;             // how long damping lasts
    int damp_windows = 4;            // consecutive out-of-band windows needed
    int min_step_db = 2;
};

class FastGainController {
public:
    explicit FastGainController(FastGainParams p = {}) : p_(p) {}

    // Feed one observation. Returns true with *out set when a gain write is
    // wanted; the caller applies it and then calls written().
    bool update(const IqStats& s, double t_s, GainSetting cur,
                GainSetting* out) {
        ++windows_;
        if (s.seq <= settle_until_seq_) return false;  // old gain still in flight

        const bool overload =
            s.clip_frac > p_.clip_frac_max || s.p95 >= p_.p95_max;
        if (overload) {
            // Emergency: predict from the median where it is still usable,
            // never less than 6 dB, 12 dB when clipping heavily. Not damped.
            double d = s.p50 > 0 ? 20.0 * std::log10(p_.p50_target / s.p50) : -12.0;
            d = std::min(d, s.clip_frac > 0.01f ? -12.0 : -6.0);
            reason_ = "overload";
            return decide(cur, d, t_s, out, /*emergency=*/true);
        }
        if (s.p50 >= p_.p50_lo && s.p50 <= p_.p50_hi) {
            ++healthy_;
            oob_run_ = 0;
            return false;  // zero-write hold
        }
        const bool carrier = s.coherence >= p_.coherence_min;
        double d = 20.0 * std::log10(p_.p50_target / std::max(s.p50, 1));
        if (!carrier && d < 0) {
            // Loud noise without a carrier: keep listening at this gain.
            ++listening_;
            oob_run_ = 0;
            return false;
        }
        ++oob_run_;
        if (t_s < damp_until_ && oob_run_ < p_.damp_windows) return false;
        reason_ = carrier ? (d > 0 ? "weak" : "strong") : "no carrier";
        return decide(cur, d, t_s, out, false);
    }

    // Call after the write requested by update() was applied.
    void written(uint64_t seq_at_write) {
        settle_until_seq_ = seq_at_write + static_cast<uint64_t>(p_.settle_transfers);
        oob_run_ = 0;
    }

    // Split a total gain into LNA + VGA: keep the LNA and move the VGA while
    // the VGA stays in range, otherwise re-centre the VGA around 30 dB.
    static GainSetting split(int total, int cur_lna) {
        total = std::clamp(total, 0, 102);
        int vga = total - cur_lna;
        if (vga >= 0 && vga <= 62) return {cur_lna, vga & ~1};
        int lna = std::clamp(((total - 30) / 8) * 8, 0, 40);
        vga = std::clamp(total - lna, 0, 62) & ~1;
        return {lna, vga};
    }

    uint64_t windows() const { return windows_; }
    uint64_t healthy() const { return healthy_; }
    uint64_t listening() const { return listening_; }
    uint64_t writes() const { return writes_; }
    uint64_t emergencies() const { return emergencies_; }
    bool damped(double t_s) const { return t_s < damp_until_; }
    const char* reason() const { return reason_; }
    const FastGainParams& params() const { return p_; }

private:
    bool decide(GainSetting cur, double delta_db, double t_s, GainSetting* out,
                bool emergency) {
        int step = static_cast<int>(std::lround(delta_db / 2.0)) * 2;
        if (step == 0) step = delta_db > 0 ? p_.min_step_db : -p_.min_step_db;
        GainSetting next = split(cur.total() + step, cur.lna);
        if (next == cur) return false;  // at a rail
        const int dir = next.total() > cur.total() ? 1 : -1;
        if (!emergency && last_dir_ != 0 && dir != last_dir_) {
            if (t_s - last_reversal_t_ <= p_.reversal_window_s)
                damp_until_ = t_s + p_.damp_s;  // second reversal in a row
            last_reversal_t_ = t_s;
        }
        last_dir_ = dir;
        ++writes_;
        if (emergency) ++emergencies_;
        *out = next;
        return true;
    }

    FastGainParams p_;
    uint64_t settle_until_seq_ = 0;
    uint64_t windows_ = 0, healthy_ = 0, listening_ = 0, writes_ = 0,
             emergencies_ = 0;
    int oob_run_ = 0;
    int last_dir_ = 0;
    double last_reversal_t_ = -1e9;
    double damp_until_ = -1e9;
    const char* reason_ = "";
};

}  // namespace famidec
