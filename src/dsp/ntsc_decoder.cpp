#include "ntsc_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "chroma.hpp"

namespace famidec {

namespace {
constexpr float kSyncSliceIre = -20.0f;   // 50% of sync amplitude
constexpr float kMinBurstAmp = 3.0f;      // IRE; below this, decode gray
}  // namespace

// Chroma bandpass half-width at this sample rate: nominally +-1 MHz around
// the subcarrier, shrunk when Nyquist gets close (low --rate). Below a
// usable width, color is off entirely (the caller sees width 0).
static double chroma_half_bw(double fs) {
    double half = std::min(1.0e6, fs / 2.0 - NtscDecoder::kFsc - 0.05e6);
    return half >= 0.2e6 ? half : 0.0;
}

NtscDecoder::NtscDecoder(const Config& cfg, TripleBuffer& out)
    : cfg_(cfg),
      tb_(out),
      fs_(cfg.sample_rate),
      nominal_period_(cfg.sample_rate / kLineRate),
      omega_sc_(2.0 * M_PI * kFsc / cfg.sample_rate),
      samples_per_us_(static_cast<int>(cfg.sample_rate / 1e6)),
      chroma_ok_(chroma_half_bw(cfg.sample_rate) > 0.0),
      chroma_bpf_(chroma_ok_
                      ? design_bandpass(kFsc - chroma_half_bw(cfg.sample_rate),
                                        kFsc + chroma_half_bw(cfg.sample_rate),
                                        cfg.sample_rate, 41)
                      : std::vector<float>{1.0f}),
      sync_lpf_(design_lowpass(1.0e6, cfg.sample_rate, 31)),
      uv_taps_(design_lowpass(0.6e6, cfg.sample_rate, 31)) {
    pll_.init(nominal_period_);
    watchdog_ = std::getenv("FPVDEC_NO_WATCHDOG") == nullptr;
    if (const char* e = std::getenv("FPVDEC_ACQ_EDGES"))
        acquire_edges_ = std::max(1, std::atoi(e));
    reseed_ = std::getenv("FPVDEC_NO_RESEED") == nullptr;
    vfly_ = std::getenv("FPVDEC_NO_VFLY") == nullptr;
    chroma_delay_ = static_cast<int64_t>(chroma_bpf_.delay());
    sync_delay_ = static_cast<int64_t>(sync_lpf_.delay());
    uv_delay_ = static_cast<int>((uv_taps_.size() - 1) / 2);
    if (!cfg.dump_composite_path.empty())
        dump_fp_ = std::fopen(cfg.dump_composite_path.c_str(), "wb");
}

NtscDecoder::~NtscDecoder() {
    if (dump_fp_) std::fclose(dump_fp_);
}

float NtscDecoder::ire_frac(double abs) const {
    int64_t i = static_cast<int64_t>(abs);
    double f = abs - static_cast<double>(i);
    float a = ire(i);
    float b = ire(i + 1);
    return a + static_cast<float>(f) * (b - a);
}

float NtscDecoder::chroma_frac(double abs) const {
    int64_t i = static_cast<int64_t>(abs);
    double f = abs - static_cast<double>(i);
    float a = chroma_at(i);
    float b = chroma_at(i + 1);
    return a + static_cast<float>(f) * (b - a);
}

void NtscDecoder::process(const float* raw, size_t n) {
    // A forced re-acquisition (user "resync") takes effect here, on the DSP
    // thread, before any decoding of this block.
    if (pending_reset_.exchange(false, std::memory_order_relaxed)) reset_state();
    // Apply any feed-forward level shift (AFC re-tune) before decoding.
    float shift = pending_shift_.exchange(0.0f, std::memory_order_relaxed);
    if (shift != 0.0f) agc_.shift(shift);
    // Store RAW envelope; conversion to IRE happens at access time with the
    // current AGC state. (Storing converted values makes the per-line AGC
    // feedback act on stale mappings a whole block late — unstable.)
    size_t old = comp_.size();
    comp_.resize(old + n);
    std::copy(raw, raw + n, comp_.begin() + static_cast<long>(old));
    if (!agc_.seeded())
        for (size_t i = 0; i < n; ++i) agc_.bootstrap(raw[i]);
    if (dump_fp_) {
        static thread_local std::vector<float> tmp;
        tmp.resize(n);
        for (size_t i = 0; i < n; ++i) tmp[i] = agc_.to_ire(raw[i]);
        std::fwrite(tmp.data(), sizeof(float), n, dump_fp_);
    }
    chromab_.resize(old + n);
    if (chroma_ok_)
        chroma_bpf_.process(comp_.data() + old, chromab_.data() + old, n);
    else
        // Chroma exceeds Nyquist at this rate: zeros make the burst
        // measurement invalid, which selects the gray decode path.
        std::fill(chromab_.begin() + static_cast<long>(old), chromab_.end(),
                  0.0f);
    syncb_.resize(old + n);
    sync_lpf_.process(comp_.data() + old, syncb_.data() + old, n);

    stats_.samples_in.store(static_cast<uint64_t>(comp_end()),
                            std::memory_order_relaxed);
    stats_.agc_seeded.store(agc_.seeded(), std::memory_order_relaxed);
    if (agc_.seeded()) decode_lines();
    stats_.agc_tip_raw.store(agc_.tip(), std::memory_order_relaxed);
    stats_.agc_blank_raw.store(agc_.blank(), std::memory_order_relaxed);
    trim_buffers();
}

// Find an hsync leading edge (sync going low in IRE terms) in [lo, hi).
// Validates pulse width so vsync broad pulses are rejected.
bool NtscDecoder::find_hsync_edge(double lo, double hi, double* edge_out,
                                  int64_t* pulse_begin, int64_t* pulse_end) const {
    int64_t start = std::max(static_cast<int64_t>(lo), base_ + 1);
    int64_t end = std::min(static_cast<int64_t>(hi), comp_end() - 8 * samples_per_us_);
    for (int64_t j = start; j < end; ++j) {
        if (ire_s(j - 1) >= kSyncSliceIre && ire_s(j) < kSyncSliceIre) {
            // Noise-tolerant pulse qualification. Fraction counts instead of
            // one contiguous run, so a noise spike inside the pulse cannot
            // split it — but anchored: chroma of bright saturated video dips
            // below the slice level every subcarrier cycle, and such a dip
            // shortly before the real sync must not qualify as the edge.
            //  - anchor: >=90% of the 2 us AFTER the candidate must be
            //    asserted. True at a real pulse start (interior); false at a
            //    chroma dip (the following samples are mostly above).
            //  - width: a 6 us window must hold 3..5.5 us of asserted
            //    samples (hsync is 4.7 us; equalizing pulses ~2.3 us fail
            //    the minimum, vsync broad pulses fail the maximum).
            // Anchor window starts 0.5 us in: the sync-path LPF slows the
            // edge, and samples still in transit sit near the slice level
            // where noise flips them.
            const int64_t anchor_skip = samples_per_us_ / 2;
            const int64_t anchor_n = 2 * samples_per_us_;
            int64_t a_cnt = 0;
            for (int64_t m = j + anchor_skip; m < j + anchor_skip + anchor_n;
                 ++m)
                if (ire_s(m) < kSyncSliceIre) ++a_cnt;
            if (10 * a_cnt < 9 * anchor_n) continue;
            int64_t maxw = j + 6 * samples_per_us_;
            int64_t w = 0;
            for (int64_t m = j; m < maxw; ++m)
                if (ire_s(m) < kSyncSliceIre) ++w;
            if (w < 3 * samples_per_us_ || w > 11 * samples_per_us_ / 2) {
#ifdef SYNC_DEBUG
                std::fprintf(stderr, "reject j=%lld w=%lld\n",
                             static_cast<long long>(j),
                             static_cast<long long>(w));
#endif
                continue;
            }
            // Trailing edge with noise gaps healed: for AGC tip averaging.
            int64_t k = j + w;
#ifdef SYNC_DEBUG
            std::fprintf(stderr, "accept j=%lld w=%lld\n",
                         static_cast<long long>(j),
                         static_cast<long long>(w));
#endif
            // sub-sample interpolation of the threshold crossing
            float a = ire_s(j - 1), b = ire_s(j);
            double frac = (a - kSyncSliceIre) / (a - b);
            *edge_out = static_cast<double>(j - 1) + frac;
            if (pulse_begin) *pulse_begin = j;
            if (pulse_end) *pulse_end = k;
            return true;
        }
    }
    return false;
}

void NtscDecoder::decode_lines() {
    const int64_t margin = 60 + chroma_delay_;
    for (;;) {
        if (state_ == State::Search) {
            if (cursor_ < base_ + 1) cursor_ = base_ + 1;
            if (cursor_ + static_cast<int64_t>(2 * nominal_period_) + margin >=
                comp_end())
                return;
            // Search has run too long without acquiring: the AGC seed is
            // probably stale (mis-scaled sync slicer). Re-bootstrap so a
            // present-but-unlocked signal gets a fresh chance.
            if (reseed_ && ++search_lines_ > kSearchReseed) {
                search_lines_ = 0;
                search_misses_ = 0;
                search_good_ = 0;
                search_interval_sum_ = 0.0;
                search_prev_edge_ = -1.0;
                agc_.reset();
                return;
            }
            // Fixed-raster fallback: draw every line at a horizontally
            // phase-snapped position (period fixed at nominal, so no skew or
            // roll), while still probing for a real lock to hand off to the
            // colour decode.
            if (cfg_.freerun_display) {
                if (raster_h_pos_ < static_cast<double>(base_ + 1) ||
                    raster_h_pos_ >
                        static_cast<double>(cursor_) + 4.0 * nominal_period_)
                    raster_h_pos_ = static_cast<double>(cursor_);
                // Narrow window while tracking, half-line while re-acquiring.
                double win = raster_h_found_ ? 2.0 * samples_per_us_
                                             : 0.5 * nominal_period_;
                double edge;
                bool found = find_hsync_edge(raster_h_pos_ - win,
                                             raster_h_pos_ + win, &edge);
                int64_t draw_start;
                if (found) {
                    raster_h_pos_ = edge;  // snap phase; period stays nominal
                    draw_start = static_cast<int64_t>(edge);
                    raster_h_found_ = true;
                    search_misses_ = 0;
                    // Consecutive line-spaced edges promote to a real lock.
                    if (search_prev_edge_ >= 0.0) {
                        double interval = edge - search_prev_edge_;
                        if (std::abs(interval - nominal_period_) <
                            0.02 * nominal_period_) {
                            search_interval_sum_ += interval;
                            if (++search_good_ >= acquire_edges_) {
                                pll_.acquire(edge, search_interval_sum_ /
                                                       search_good_);
                                state_ = State::Track;
                                search_prev_edge_ = -1.0;
                                search_good_ = 0;
                                search_interval_sum_ = 0.0;
                                search_lines_ = 0;
                                track_line_ctr_ = 0;
                                narrow_miss_ctr_ = 0;
                                bad_field_run_ = 0;
                                continue;
                            }
                        } else {
                            search_good_ = 0;
                            search_interval_sum_ = 0.0;
                        }
                    }
                    search_prev_edge_ = edge;
                } else {
                    draw_start = static_cast<int64_t>(raster_h_pos_);  // coast
                    raster_h_found_ = false;
                    search_good_ = 0;
                    search_interval_sum_ = 0.0;
                    search_prev_edge_ = -1.0;
                }
                raster_line(draw_start);
                raster_h_pos_ += nominal_period_;
                cursor_ = static_cast<int64_t>(raster_h_pos_);
                continue;
            }
            double edge;
            if (!find_hsync_edge(static_cast<double>(cursor_),
                                 static_cast<double>(cursor_) + nominal_period_,
                                 &edge)) {
                // No sync anywhere in this line's worth of samples: paint it
                // as free-running "snow" so an unlocked signal is visible.
                // If this keeps happening the AGC level estimates may be
                // stale (they only refine on found edges, so a level jump
                // can deadlock them) — re-bootstrap from the signal.
                if (++search_misses_ > 2000) {
                    search_misses_ = 0;
                    agc_.reset();
                    return;  // process() bootstraps until seeded again
                }
                // A missing edge breaks the consecutive run used to average
                // the period seed.
                search_good_ = 0;
                search_interval_sum_ = 0.0;
                search_prev_edge_ = -1.0;
                freerun_line(cursor_);
                cursor_ += static_cast<int64_t>(nominal_period_);
                continue;
            }
            search_misses_ = 0;
            cursor_ = static_cast<int64_t>(edge) + 30 * samples_per_us_;
            if (search_prev_edge_ >= 0.0) {
                double interval = edge - search_prev_edge_;
                if (std::abs(interval - nominal_period_) <
                    0.02 * nominal_period_) {
                    // Consecutive line-spaced edge: accumulate toward an
                    // averaged period before committing to a lock.
                    search_interval_sum_ += interval;
                    if (++search_good_ >= acquire_edges_) {
                        double avg = search_interval_sum_ /
                                     static_cast<double>(search_good_);
                        pll_.acquire(edge, avg);
                        state_ = State::Track;
                        search_prev_edge_ = -1.0;
                        search_good_ = 0;
                        search_interval_sum_ = 0.0;
                        search_lines_ = 0;
                        track_line_ctr_ = 0;
                        narrow_miss_ctr_ = 0;
                        bad_field_run_ = 0;
                        continue;
                    }
                } else {
                    // Interval off by more than a line's worth: restart the
                    // run from this edge.
                    search_good_ = 0;
                    search_interval_sum_ = 0.0;
                }
            }
            search_prev_edge_ = edge;
        } else {
            // Need the full line plus search window and chroma delay.
            double predicted = pll_.next_edge;
            if (static_cast<int64_t>(predicted + pll_.period) + margin >=
                comp_end())
                return;
            double measured;
            bool narrow_ok = find_hsync_edge(predicted - 2 * samples_per_us_,
                                             predicted + 2 * samples_per_us_,
                                             &measured, &pulse_begin_,
                                             &pulse_end_);
            bool ok = narrow_ok;
            double edge;
            if (!ok && pll_.coast >= 4 &&
                find_hsync_edge(predicted - 0.5 * pll_.period,
                                predicted + 0.5 * pll_.period, &measured,
                                &pulse_begin_, &pulse_end_)) {
                // Interlaced vsync shifts the hsync phase by half a line
                // every other field — the narrow window can never recover
                // from that. Snap the flywheel onto the pulse found in a
                // full-line search instead of filtering toward it.
                pll_.acquire(measured, pll_.period);
                edge = measured;
                ok = true;
            } else {
                edge = pll_.advance(ok, measured);
            }
            if (!ok) stats_.lines_coasted.fetch_add(1, std::memory_order_relaxed);
#ifdef SYNC_DEBUG
            if (!ok)
                std::fprintf(stderr, "coast line=%d pred=%.1f\n", line_no_,
                             predicted);
#endif
            stats_.line_locked.store(pll_.locked, std::memory_order_relaxed);
            stats_.line_period.store(static_cast<float>(pll_.period),
                                     std::memory_order_relaxed);
            if (!pll_.locked && pll_.coast > LinePll::kCoastLimit) {
                state_ = State::Search;
                cursor_ = static_cast<int64_t>(edge);
                search_prev_edge_ = -1.0;
                search_lines_ = 0;
                continue;
            }
            // False-lock watchdog: a lock leaning on the wide rescue keeps
            // missing the narrow window. If that dominates a field for a few
            // fields running, the period is wrong — force a re-acquisition.
            if (!narrow_ok) ++narrow_miss_ctr_;
            if (++track_line_ctr_ >= kFalseLockField) {
                bool bad = narrow_miss_ctr_ > kFalseLockMisses;
                bad_field_run_ = bad ? bad_field_run_ + 1 : 0;
                track_line_ctr_ = 0;
                narrow_miss_ctr_ = 0;
                if (watchdog_ && bad_field_run_ >= kFalseLockFields) {
                    std::fprintf(stderr,
                                 "false-lock watchdog: forcing resync\n");
                    reset_state();
                    continue;
                }
            }
            handle_line(edge, ok);
            stats_.lines.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// Abandon the current lock and AGC seed and start over from Search, as if
// freshly constructed. The sample buffers are kept (re-searched from the
// current position); the AGC re-bootstraps from the incoming signal.
void NtscDecoder::reset_state() {
    pll_.reset();
    state_ = State::Search;
    agc_.reset();
    cursor_ = base_ + 1;
    search_prev_edge_ = -1.0;
    search_good_ = 0;
    search_interval_sum_ = 0.0;
    search_misses_ = 0;
    search_lines_ = 0;
    vsync_run_ = 0;
    line_no_ = 0;
    track_line_ctr_ = 0;
    narrow_miss_ctr_ = 0;
    bad_field_run_ = 0;
    stats_.line_locked.store(false, std::memory_order_relaxed);
}

// Paint one unlocked line as grayscale snow, normalized per line so noise of
// any level is visible. Publishes a frame every kHeight/2 rows.
void NtscDecoder::freerun_line(int64_t start) {
    Frame& f = tb_.back();
    int row = freerun_row_ * 2;
    uint32_t* out0 = f.rgba.data() + static_cast<size_t>(row) * Frame::kWidth;
    uint32_t* out1 = out0 + Frame::kWidth;
    int64_t n = static_cast<int64_t>(nominal_period_);
    float lo = 1e30f, hi = -1e30f;
    for (int64_t j = start; j < start + n; ++j) {
        float v = comp_[static_cast<size_t>(j - base_)];
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    float scale = (hi > lo) ? 255.0f / (hi - lo) : 0.0f;
    double step = static_cast<double>(n) / Frame::kWidth;
    for (int px = 0; px < Frame::kWidth; ++px) {
        size_t idx = static_cast<size_t>(
            start + static_cast<int64_t>(px * step) - base_);
        auto g = static_cast<uint32_t>((comp_[idx] - lo) * scale);
        uint32_t px32 = 0xff000000u | (g << 16) | (g << 8) | g;
        out0[px] = px32;
        out1[px] = px32;
    }
    if (++freerun_row_ >= Frame::kHeight / 2) {
        freerun_row_ = 0;
        tb_.publish(++frame_seq_);
    }
}

// Fixed-raster fallback (--freerun): draw the AGC-scaled composite as luma on
// a nominal horizontal period, with a vertical flywheel (below) holding the
// field boundary, all without a horizontal sync lock. A present-but-
// unlockable signal shows up as a held picture instead of per-line snow; the
// normal locked decode takes over the moment sync is acquired. Publishes a
// field but leaves stats_.frames alone, so the OSD still honestly reports
// "not locked".
void NtscDecoder::raster_line(int64_t start) {
    int active = raster_line_no_ - kActiveStartLine;
    if (active >= 0 && active < kActiveLines) {
        int row = active * 2;
        if (row >= 0 && row + 1 < Frame::kHeight) {
            Frame& f = tb_.back();
            uint32_t* out0 =
                f.rgba.data() + static_cast<size_t>(row) * Frame::kWidth;
            uint32_t* out1 = out0 + Frame::kWidth;
            const double a0 =
                static_cast<double>(start) + 9.4 * samples_per_us_;
            const double span = 52.6 * samples_per_us_;
            const double step = span / Frame::kWidth;
            for (int px = 0; px < Frame::kWidth; ++px) {
                float y = ire_frac(a0 + px * step);
                uint32_t g = static_cast<uint32_t>(
                    std::clamp(y / 100.0f, 0.0f, 1.0f) * 255.0f + 0.5f);
                uint32_t px32 = 0xff000000u | (g << 16) | (g << 8) | g;
                out0[px] = px32;
                out1[px] = px32;
            }
        }
    }
    // Vertical flywheel: a vsync block is a line with sync asserted most of
    // its length. Anchor the field boundary to the end of that block, but
    // only when it falls near the expected field length (rejects a spurious
    // mid-field detection); free-wheel-wrap at the max length if vsync is
    // missed. This is a one-field-period flywheel, so the field holds
    // position instead of rolling.
    if (!vfly_) {  // flywheel off: plain free-run wrap (rolls)
        if (++raster_line_no_ >= 262) {
            raster_line_no_ = 0;
            tb_.publish(++frame_seq_);
        }
        return;
    }
    const int64_t period_i = static_cast<int64_t>(nominal_period_);
    int64_t asserted = 0;
    for (int64_t j = start; j < start + period_i; ++j)
        if (ire_s(j) < kSyncSliceIre) ++asserted;
    bool boundary = false;
    if (asserted > static_cast<int64_t>(0.45 * static_cast<double>(period_i))) {
        ++raster_vsync_run_;
    } else {
        if (raster_vsync_run_ >= 2 && raster_line_no_ >= kVFieldMin)
            boundary = true;
        raster_vsync_run_ = 0;
    }
    if (boundary || ++raster_line_no_ >= kVFieldMax) {
        raster_line_no_ = 0;
        tb_.publish(++frame_seq_);
    }
}

void NtscDecoder::handle_line(double edge, bool edge_measured) {
    int64_t e = static_cast<int64_t>(std::llround(edge));
    int64_t period_i = static_cast<int64_t>(pll_.period);

    // Vsync: during the broad-pulse region, sync stays asserted most of the
    // line. Famicom emits no standard equalization, so just measure the
    // asserted fraction.
    int64_t asserted = 0;
    for (int64_t j = e; j < e + period_i; ++j)
        if (ire_s(j) < kSyncSliceIre) ++asserted;
    bool is_vsync_line =
        asserted > static_cast<int64_t>(0.45 * static_cast<double>(period_i));

    if (is_vsync_line) {
        ++vsync_run_;
        return;
    }
    if (vsync_run_ >= 2) {
        tb_.publish(++frame_seq_);
        stats_.frames.fetch_add(1, std::memory_order_relaxed);
        stats_.frame_sample_pos.store(static_cast<uint64_t>(e),
                                      std::memory_order_relaxed);
        line_no_ = 0;
    } else {
        ++line_no_;
        // Free-wheeling vertical sync: if noise made us miss the vsync
        // pulses, publish on the flywheel at the nominal field length so
        // frames keep flowing (a real TV does the same). Only while the
        // line PLL is solidly locked — otherwise a fake noise-lock would
        // publish black frames instead of falling back to snow.
        if (line_no_ >= 262) {
            if (pll_.locked) {
                tb_.publish(++frame_seq_);
                stats_.frames.fetch_add(1, std::memory_order_relaxed);
                stats_.frame_sample_pos.store(static_cast<uint64_t>(e),
                                              std::memory_order_relaxed);
            }
            line_no_ -= 262;
        }
    }
    vsync_run_ = 0;

    // AGC refinement: sync tip over the pulse, blanking from the back porch
    // after the burst (8.2..9.2 us).
    if (edge_measured && pulse_end_ > pulse_begin_) {
        // Average over the middle half of the slicer-asserted pulse extent.
        // Fixed offsets from the edge estimate drift onto the filter-ringing
        // overshoot as AGC scale errors bias the threshold crossing, which
        // turns the level feedback unstable.
        int64_t w = pulse_end_ - pulse_begin_;
        float tip = 0.0f;
        int tn = 0;
        for (int64_t j = pulse_begin_ + w / 4; j < pulse_end_ - w / 4; ++j, ++tn)
            tip += ire_s(j);
        tip /= static_cast<float>(tn);
        float blank = 0.0f;
        int n = 0;
        for (int64_t j = e + 82 * samples_per_us_ / 10;
             j < e + 92 * samples_per_us_ / 10; ++j, ++n)
            blank += ire_s(j);
        blank /= static_cast<float>(n);
        agc_.update_from_ire(tip, blank);
    }

    if (line_no_ >= kActiveStartLine &&
        line_no_ < kActiveStartLine + kActiveLines) {
        decode_row(edge);
    }
}

void NtscDecoder::decode_row(double edge) {
    Frame& f = tb_.back();
    int row = (line_no_ - kActiveStartLine) * 2;
    if (row < 0 || row + 1 >= Frame::kHeight) return;
    uint32_t* out0 = f.rgba.data() + static_cast<size_t>(row) * Frame::kWidth;
    uint32_t* out1 = out0 + Frame::kWidth;

    const double full_start = edge + 9.4 * samples_per_us_;
    const double full_span = 52.6 * samples_per_us_;
    // TV-style overscan: display only the central part of the active line.
    const double crop = full_span * cfg_.overscan;
    const double active_start = full_start + crop;
    const double active_span = full_span - 2.0 * crop;
    const double step = active_span / Frame::kWidth;
    const bool color = cfg_.mode == Config::Mode::Color;

    // Burst: ~9+ cycles starting 5.3 us after the edge; gate 3.2 us.
    BurstMeasurement burst;
    double phi = 0.0;
    if (color) {
        int64_t g0 = static_cast<int64_t>(std::llround(edge)) +
                     53 * samples_per_us_ / 10;
        int gn = 32 * samples_per_us_ / 10;
        static thread_local std::vector<float> gate;
        gate.resize(gn);
        for (int j = 0; j < gn; ++j) gate[j] = chroma_at(g0 + j);
        double theta0 = std::fmod(omega_sc_ * static_cast<double>(g0), 2.0 * M_PI);
        burst = measure_burst(gate.data(), gate.size(), theta0, omega_sc_,
                              kMinBurstAmp);
        stats_.burst_amp.store(burst.amplitude, std::memory_order_relaxed);
        phi = burst.phase + cfg_.hue_deg * M_PI / 180.0;
    }

    if (color && burst.valid) {
        // Demod U/V across the line (from just after the burst through the
        // end of active video plus the UV filter half-width).
        int64_t a0 = static_cast<int64_t>(std::llround(edge)) +
                     85 * samples_per_us_ / 10;
        int64_t a1 = static_cast<int64_t>(active_start + active_span) +
                     uv_delay_ + 2;
        size_t n = static_cast<size_t>(a1 - a0);
        su_.resize(n);
        sv_.resize(n);
        // Subcarrier as a rotating phasor: one complex multiply per sample
        // instead of sin+cos (which at 20 MSPS costs more than the FIRs).
        // Float drift over one line (~1e-4) is far below chroma tolerances.
        double th0 = std::fmod(omega_sc_ * static_cast<double>(a0), 2.0 * M_PI) + phi;
        std::complex<float> ph(static_cast<float>(std::cos(th0)),
                               static_cast<float>(std::sin(th0)));
        const std::complex<float> rot(static_cast<float>(std::cos(omega_sc_)),
                                      static_cast<float>(std::sin(omega_sc_)));
        for (size_t j = 0; j < n; ++j) {
            float c = chroma_at(a0 + static_cast<int64_t>(j));
            su_[j] = 2.0f * c * ph.imag();
            sv_[j] = 2.0f * c * ph.real();
            ph *= rot;
        }
        // Zero-history per-line LPF (edge transients land in blanking).
        // Forward dot product (taps are symmetric, so no reversal needed)
        // keeps the inner loop auto-vectorizable.
        size_t nt = uv_taps_.size();
        suf_.assign(n, 0.0f);
        svf_.assign(n, 0.0f);
        const float* t = uv_taps_.data();
        for (size_t i = nt - 1; i < n; ++i) {
            const float* wu = su_.data() + (i - (nt - 1));
            const float* wv = sv_.data() + (i - (nt - 1));
            float au = 0.0f, av = 0.0f;
            for (size_t k = 0; k < nt; ++k) {
                au += wu[k] * t[k];
                av += wv[k] * t[k];
            }
            suf_[i] = au;
            svf_[i] = av;
        }
        const float sat = cfg_.saturation;
        for (int px = 0; px < Frame::kWidth; ++px) {
            double p = active_start + px * step;
            float y = ire_frac(p) - chroma_frac(p);
            // filtered UV value centered at abs pos p
            int64_t idx = static_cast<int64_t>(p) - a0 + uv_delay_;
            float u = 0.0f, v = 0.0f;
            if (idx >= 0 && static_cast<size_t>(idx) < n) {
                u = suf_[static_cast<size_t>(idx)] * sat;
                v = svf_[static_cast<size_t>(idx)] * sat;
            }
            float yn = y / 100.0f, un = u / 100.0f, vn = v / 100.0f;
            float r = yn + 1.140f * vn;
            float g = yn - 0.395f * un - 0.581f * vn;
            float b = yn + 2.032f * un;
            auto q = [](float x) {
                return static_cast<uint32_t>(
                    std::clamp(x, 0.0f, 1.0f) * 255.0f + 0.5f);
            };
            uint32_t px32 = 0xff000000u | (q(b) << 16) | (q(g) << 8) | q(r);
            out0[px] = px32;
            out1[px] = px32;
        }
    } else {
        for (int px = 0; px < Frame::kWidth; ++px) {
            double p = active_start + px * step;
            float y = ire_frac(p) - chroma_frac(p);
            uint32_t g = static_cast<uint32_t>(
                std::clamp(y / 100.0f, 0.0f, 1.0f) * 255.0f + 0.5f);
            uint32_t px32 = 0xff000000u | (g << 16) | (g << 8) | g;
            out0[px] = px32;
            out1[px] = px32;
        }
    }
}

void NtscDecoder::trim_buffers() {
    // Keep enough history behind the decode position; drop the rest.
    int64_t keep_from;
    if (state_ == State::Track)
        keep_from = static_cast<int64_t>(pll_.next_edge) -
                    static_cast<int64_t>(nominal_period_);
    else
        keep_from = cursor_ - static_cast<int64_t>(nominal_period_);
    keep_from -= chroma_delay_ + 16;
    if (keep_from <= base_) return;
    size_t drop = static_cast<size_t>(keep_from - base_);
    if (drop < 32768 || drop > comp_.size()) return;
    comp_.erase(comp_.begin(), comp_.begin() + static_cast<long>(drop));
    chromab_.erase(chromab_.begin(), chromab_.begin() + static_cast<long>(drop));
    syncb_.erase(syncb_.begin(), syncb_.begin() + static_cast<long>(drop));
    base_ += static_cast<int64_t>(drop);
}

}  // namespace famidec
