#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../config.hpp"
#include "agc.hpp"
#include "fir.hpp"
#include "frame.hpp"
#include "sync.hpp"

namespace famidec {

// Composite (detected envelope, raw units, positive-going sync) -> RGB frames.
class NtscDecoder {
public:
    static constexpr double kFsc = 315e6 / 88.0;      // 3.579545... MHz
    static constexpr double kLineRate = 15734.264;    // Hz (NTSC nominal)
    static constexpr int kActiveLines = 240;
    static constexpr int kActiveStartLine = 13;       // lines after vsync end

    struct Stats {
        std::atomic<bool> line_locked{false};
        std::atomic<float> burst_amp{0.0f};
        // Raw (pre-IRE) AGC levels. For the FM path these are direct
        // frequency measurements (freq = -raw * dev, or +raw with --invert),
        // which is what the AFC uses to find the carrier offset.
        std::atomic<float> agc_tip_raw{0.0f};
        std::atomic<float> agc_blank_raw{0.0f};
        std::atomic<uint64_t> frames{0};
        std::atomic<uint64_t> lines{0};
        std::atomic<uint64_t> lines_coasted{0};
        std::atomic<float> line_period{0.0f};
        // Absolute input-sample index of the most recently published
        // frame's vsync, and total samples fed into the decoder — both in
        // the same coordinate system (for latency estimation that stays
        // correct across input drops).
        std::atomic<uint64_t> frame_sample_pos{0};
        std::atomic<uint64_t> samples_in{0};
        std::atomic<bool> agc_seeded{false};  // false = AGC re-bootstrapping
    };

    NtscDecoder(const Config& cfg, TripleBuffer& out);
    ~NtscDecoder();

    // Feed raw detected envelope samples.
    void process(const float* raw, size_t n);

    // Thread-safe feed-forward of a known composite level shift (AFC
    // re-tune); applied on the DSP thread at the next process() call.
    void shift_levels(float delta_raw) {
        float cur = pending_shift_.load(std::memory_order_relaxed);
        while (!pending_shift_.compare_exchange_weak(
            cur, cur + delta_raw, std::memory_order_relaxed)) {
        }
    }

    // Force a clean re-acquisition (equivalent to a freshly constructed
    // decoder): drops a false lock the runtime cannot otherwise escape. On a
    // faded/lost signal the flywheel can stay "locked" onto a wrong line
    // phase and never fall back to Search, so the picture tears until the
    // process is restarted; this gives the same recovery without a restart.
    // Thread-safe: applied on the DSP thread at the next process().
    void request_reset() {
        pending_reset_.store(true, std::memory_order_relaxed);
    }

    const Stats& stats() const { return stats_; }

private:
    enum class State { Search, Track };

    void decode_lines();
    void reset_state();
    void freerun_line(int64_t start);
    void raster_line(int64_t start);
    bool find_hsync_edge(double lo, double hi, double* edge_out,
                         int64_t* pulse_begin = nullptr,
                         int64_t* pulse_end = nullptr) const;
    void handle_line(double edge, bool edge_measured);
    void decode_row(double edge);
    void trim_buffers();

    // comp_/chromab_ hold the RAW detected envelope (and its bandpass);
    // accessors convert with the current AGC state. The chroma bandpass of
    // the affine raw->IRE map is a pure scale (the offset is DC-rejected).
    float ire(int64_t abs) const {
        return agc_.to_ire(comp_[static_cast<size_t>(abs - base_)]);
    }
    // Sync-path IRE: same composite through a ~1 MHz LPF (delay
    // compensated). Sync pulses carry no detail above that, so slicing on
    // the narrow stream cuts ~7 dB of noise power — the difference between
    // holding and losing lock on a weak signal. Video decoding keeps the
    // full-bandwidth ire().
    float ire_s(int64_t abs) const {
        return agc_.to_ire(
            syncb_[static_cast<size_t>(abs - base_ + sync_delay_)]);
    }
    float ire_frac(double abs) const;
    float chroma_at(int64_t abs) const {
        return agc_.chroma_scale() *
               chromab_[static_cast<size_t>(abs - base_ + chroma_delay_)];
    }
    float chroma_frac(double abs) const;
    int64_t comp_end() const { return base_ + static_cast<int64_t>(comp_.size()); }
    int64_t chroma_end() const { return comp_end() - chroma_delay_; }

    const Config& cfg_;
    TripleBuffer& tb_;
    Stats stats_;

    double fs_;
    double nominal_period_;
    double omega_sc_;          // rad/sample of the color subcarrier
    int samples_per_us_;

    Agc agc_;
    LinePll pll_;
    State state_ = State::Search;

    // Composite (IRE), chroma bandpass and sync lowpass streams,
    // absolute-indexed.
    std::vector<float> comp_;
    std::vector<float> chromab_;
    std::vector<float> syncb_;
    int64_t base_ = 0;
    int64_t chroma_delay_;
    int64_t sync_delay_;
    bool chroma_ok_;  // false when --rate puts chroma past Nyquist
    FirFilterF chroma_bpf_;
    FirFilterF sync_lpf_;
    std::vector<float> uv_taps_;
    int uv_delay_;

    // Search state. Acquire a lock only after a short run of consecutive
    // line-spaced edges, and seed the flywheel period from their average: a
    // single line's interval is too noisy and can seed a wrong period that
    // then tears the picture.
    static constexpr int kAcquireEdges = 3;  // consecutive intervals required
    int acquire_edges_ = kAcquireEdges;      // FPVDEC_ACQ_EDGES overrides
    double search_prev_edge_ = -1.0;
    int search_good_ = 0;
    double search_interval_sum_ = 0.0;
    // Re-bootstrap the AGC after this many Search lines without a lock. A
    // level shift (e.g. a fade or an AFC re-tune) can leave the AGC seed
    // mis-scaled so the sync slicer never fires; the consecutive-miss reset
    // is defeated by the occasional spurious edge, so key off elapsed Search
    // lines instead. ~4 fields at the NTSC line rate.
    static constexpr int kSearchReseed = 1100;
    bool reseed_ = true;  // FPVDEC_NO_RESEED=1 disables (debug/compare)
    int search_lines_ = 0;
    int64_t cursor_ = 0;
    int freerun_row_ = 0;  // "snow" rows painted while unlocked
    // Fixed-raster (--freerun) vertical flywheel: a long-period (one field
    // ~= 262 lines) analogue of the horizontal LinePll. The frame boundary
    // snaps to a detected vsync block only when it lands near the expected
    // field length, and free-wheels at the nominal length otherwise, so the
    // picture holds vertically instead of rolling.
    static constexpr int kVFieldMin = 220;  // reject a vsync earlier than this
    static constexpr int kVFieldMax = 280;  // coast-wrap if vsync is missed
    int raster_line_no_ = 0;    // lines since the last field boundary
    int raster_vsync_run_ = 0;  // consecutive vsync-asserted lines
    bool vfly_ = true;  // FPVDEC_NO_VFLY=1 disables the flywheel (debug)
    // Horizontal phase predictor for the raster: advances one nominal line
    // each row and snaps to the actual hsync when found (period stays
    // nominal, so no skew), holding the picture horizontally.
    double raster_h_pos_ = 0.0;
    bool raster_h_found_ = false;
    int search_misses_ = 0;  // consecutive lines with no sync found
    std::atomic<float> pending_shift_{0.0f};
    std::atomic<bool> pending_reset_{false};  // force a clean re-acquisition

    // Extent of the most recently found hsync pulse (slicer-asserted range).
    int64_t pulse_begin_ = 0;
    int64_t pulse_end_ = 0;

    // Vertical state
    int vsync_run_ = 0;
    int line_no_ = 0;
    uint64_t frame_seq_ = 0;

    // False-lock watchdog. A lock the flywheel cannot actually track (wrong
    // line period) keeps failing the narrow hsync window and leaning on the
    // wide-search rescue, which re-anchors the phase but leaves the period
    // wrong and resets the coast counter — so it never drops to Search on
    // its own and the picture tears until a restart. Counting narrow-window
    // misses per ~field catches this: a healthy lock misses only the
    // vertical-blanking lines, a false lock misses most of them.
    static constexpr int kFalseLockField = 262;   // lines per evaluation
    static constexpr int kFalseLockMisses = 105;   // ~40% of a field
    static constexpr int kFalseLockFields = 3;     // sustained this many
    int track_line_ctr_ = 0;
    int narrow_miss_ctr_ = 0;
    int bad_field_run_ = 0;
    bool watchdog_ = true;  // FPVDEC_NO_WATCHDOG=1 disables (debug/compare)

    // Per-line scratch
    std::vector<float> su_, sv_, suf_, svf_;

    FILE* dump_fp_ = nullptr;
};

}  // namespace famidec
