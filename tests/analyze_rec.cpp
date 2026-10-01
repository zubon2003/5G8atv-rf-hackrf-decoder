// Offline replay/analysis harness: feed a recorded cs8 IQ file through the
// exact live FM DSP chain + NtscDecoder and log per-window stats so a
// stuck/false lock can be studied without hardware.
//   usage: analyze_rec <file.cs8> [window_samples]
// Columns: t_s locked period fps coast_frac frames
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "config.hpp"
#include "dsp/dc_blocker.hpp"
#include "dsp/fir.hpp"
#include "dsp/fm_detector.hpp"
#include "dsp/frame.hpp"
#include "dsp/ntsc_decoder.hpp"

using namespace famidec;

// Minimal 24bpp bottom-up BMP writer for a decoded frame.
static void write_bmp(const Frame& f, const char* path) {
    const int w = Frame::kWidth, h = Frame::kHeight;
    const int row = w * 3;  // 640*3 = 1920, already 4-byte aligned
    const uint32_t img = static_cast<uint32_t>(row) * h;
    const uint32_t off = 54;
    std::FILE* fp = std::fopen(path, "wb");
    if (!fp) return;
    uint8_t hdr[54] = {0};
    hdr[0] = 'B'; hdr[1] = 'M';
    uint32_t fsz = off + img;
    hdr[2] = fsz; hdr[3] = fsz >> 8; hdr[4] = fsz >> 16; hdr[5] = fsz >> 24;
    hdr[10] = off;
    hdr[14] = 40;
    hdr[18] = w; hdr[19] = w >> 8;
    hdr[22] = h; hdr[23] = h >> 8;
    hdr[26] = 1; hdr[28] = 24;
    std::fwrite(hdr, 1, 54, fp);
    std::vector<uint8_t> line(static_cast<size_t>(row));
    for (int y = h - 1; y >= 0; --y) {
        const uint32_t* src = f.rgba.data() + static_cast<size_t>(y) * w;
        for (int x = 0; x < w; ++x) {
            uint32_t p = src[x];  // memory R,G,B,A
            line[3 * x + 0] = (p >> 16) & 0xff;  // B
            line[3 * x + 1] = (p >> 8) & 0xff;   // G
            line[3 * x + 2] = p & 0xff;          // R
        }
        std::fwrite(line.data(), 1, static_cast<size_t>(row), fp);
    }
    std::fclose(fp);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.cs8> [window_samples]\n", argv[0]);
        return 2;
    }
    std::FILE* fp = std::fopen(argv[1], "rb");
    if (!fp) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }
    Config cfg;  // live defaults: 10 MSPS, dev 5 MHz, no offset, NTSC
    if (std::getenv("FREERUN")) cfg.freerun_display = true;
    const double fs = cfg.sample_rate;
    const size_t window = argc > 2 ? std::strtoul(argv[2], nullptr, 10)
                                   : static_cast<size_t>(fs / 10);  // 0.1 s

    TripleBuffer tb;
    NtscDecoder dec(cfg, tb);
    DcBlocker dcb;
    FirFilterC chan_lpf(design_lowpass(std::min(8.0e6, fs * 0.49), fs, 47));
    FmDetector fm_det(fs, cfg.fm_dev_hz, cfg.invert);

    constexpr size_t kBlock = 1 << 16;  // 65536 bytes = 32768 complex
    std::vector<uint8_t> raw(kBlock);
    std::vector<std::complex<float>> iq(kBlock / 2);
    std::vector<float> comp(kBlock / 2);

    uint64_t total_samples = 0, win_acc = 0;
    uint64_t prev_frames = 0, prev_lines = 0, prev_coasted = 0;
    double win_amp_ = 0.0;
    uint64_t win_amp_n_ = 0;

    // Bulk vertical-sync detector, independent of horizontal lock: heavily
    // smooth the composite (sync tip is the raw maximum), track a slow
    // floor/ceiling, and flag a sustained (> ~1 line) excursion toward sync
    // as a field's vsync. This is what a --fixed-h mode would key off.
    const bool vscan = std::getenv("VSYNC_SCAN") != nullptr;
    double s_ema = 0.0, vblk = 0.0, vtip = 0.0;
    bool v_init = false, v_above = false;
    int64_t v_run_start = 0, v_above_len = 0, v_last = -1;
    std::vector<int64_t> vpos;      // detected vsync sample positions
    const double kSalpha = 0.006;   // ~half-line smoothing
    const int64_t kVminDur = 760;   // > 1 line asserted = vsync, not hsync
    // Dump a frame BMP when crossing each of these times (seconds).
    const double dump_t[] = {5, 15, 22, 30, 40, 48, 53, 56, 62};
    size_t dump_i = 0;
    const char* dump_dir = std::getenv("DUMP_DIR");
    // Sequence dump: newest frame every 1/SEQ_FPS of signal time in
    // [SEQ_T0, SEQ_T1], numbered — for encoding a real-time-aligned video.
    const char* seq_dir = std::getenv("SEQ_DIR");
    const double seq_t0 = std::getenv("SEQ_T0") ? std::atof(std::getenv("SEQ_T0")) : 0.0;
    const double seq_t1 = std::getenv("SEQ_T1") ? std::atof(std::getenv("SEQ_T1")) : 1e9;
    const double seq_fps = std::getenv("SEQ_FPS") ? std::atof(std::getenv("SEQ_FPS")) : 30.0;
    const uint64_t seq_step = static_cast<uint64_t>(fs / seq_fps);
    uint64_t seq_next = static_cast<uint64_t>(seq_t0 * fs);
    int seq_n = 0;
    Frame seq_held;
    bool seq_have = false;
    std::printf("t_s locked period fps coast_frac frames lines\n");
    for (;;) {
        size_t n = std::fread(raw.data(), 1, raw.size(), fp);
        if (n < 2) break;
        size_t ns = n / 2;
        double amp_acc = 0.0;
        for (size_t i = 0; i < ns; ++i) {
            int qi = static_cast<int8_t>(raw[2 * i]);
            int qq = static_cast<int8_t>(raw[2 * i + 1]);
            amp_acc += std::abs(qi) + std::abs(qq);
            std::complex<float> c(qi / 128.0f, qq / 128.0f);
            iq[i] = dcb.process(c);
        }
        win_amp_ += amp_acc;
        win_amp_n_ += ns;
        chan_lpf.process(iq.data(), iq.data(), ns);
        fm_det.process(iq.data(), comp.data(), ns);
        if (vscan) {
            for (size_t i = 0; i < ns; ++i) {
                double x = comp[i];
                int64_t g = static_cast<int64_t>(total_samples) +
                            static_cast<int64_t>(i);
                if (!v_init) { s_ema = vblk = vtip = x; v_init = true; }
                s_ema += kSalpha * (x - s_ema);
                vblk += (s_ema < vblk ? 0.01 : 2e-5) * (s_ema - vblk);
                vtip += (s_ema > vtip ? 0.01 : 2e-5) * (s_ema - vtip);
                double thr = vblk + 0.55 * (vtip - vblk);
                if (s_ema > thr) {
                    if (!v_above) { v_above = true; v_run_start = g; v_above_len = 0; }
                    ++v_above_len;
                } else if (v_above) {
                    v_above = false;
                    if (v_above_len >= kVminDur &&
                        (v_last < 0 || v_run_start - v_last > 40000)) {
                        vpos.push_back(v_run_start);
                        v_last = v_run_start;
                    }
                }
            }
        }
        dec.process(comp.data(), ns);

        total_samples += ns;
        double t_now = static_cast<double>(total_samples) / fs;
        if (seq_dir && t_now >= seq_t0 && t_now <= seq_t1) {
            const Frame* nf = tb.acquire();
            if (nf) { seq_held = *nf; seq_have = true; }
            while (total_samples >= seq_next && seq_have) {
                char path[512];
                std::snprintf(path, sizeof(path), "%s/f%05d.bmp", seq_dir,
                              seq_n++);
                write_bmp(seq_held, path);
                seq_next += seq_step;
            }
        }
        if (dump_dir && dump_i < sizeof(dump_t) / sizeof(dump_t[0]) &&
            t_now >= dump_t[dump_i]) {
            const Frame* f = tb.acquire();
            if (f) {
                char path[512];
                std::snprintf(path, sizeof(path), "%s/rec_t%02.0f.bmp",
                              dump_dir, dump_t[dump_i]);
                write_bmp(*f, path);
            }
            ++dump_i;
        }
        win_acc += ns;
        if (win_acc >= window) {
            const auto& s = dec.stats();
            uint64_t frames = s.frames.load();
            uint64_t lines = s.lines.load();
            uint64_t coasted = s.lines_coasted.load();
            double dt = static_cast<double>(win_acc) / fs;
            double fps = static_cast<double>(frames - prev_frames) / dt;
            uint64_t dl = lines - prev_lines;
            double coast_frac =
                dl ? static_cast<double>(coasted - prev_coasted) / dl : 0.0;
            double amp = win_amp_n_ ? win_amp_ / win_amp_n_ : 0.0;
            std::printf("%.2f %d %.3f %.1f %.3f %llu %llu %d %.2f\n",
                        static_cast<double>(total_samples) / fs,
                        s.line_locked.load() ? 1 : 0, s.line_period.load(), fps,
                        coast_frac, static_cast<unsigned long long>(frames),
                        static_cast<unsigned long long>(lines),
                        s.agc_seeded.load() ? 1 : 0, amp);
            win_amp_ = 0.0;
            win_amp_n_ = 0;
            prev_frames = frames;
            prev_lines = lines;
            prev_coasted = coasted;
            win_acc = 0;
        }
    }
    std::fclose(fp);
    if (vscan) {
        // Expected NTSC field spacing (interlaced fields at 59.94 Hz).
        const double field = fs / 59.94;
        std::printf("# VSYNC_SCAN: expected field spacing %.0f samp (%.2f ms)\n",
                    field, field / fs * 1000.0);
        std::printf("sec  vsync_n  regular%%  mean_ms\n");
        // per-2s buckets
        for (int b = 0; b * 2 * static_cast<int>(fs) <
                        static_cast<int>(total_samples); ++b) {
            int64_t lo = static_cast<int64_t>(b) * 2 * static_cast<int64_t>(fs);
            int64_t hi = lo + 2 * static_cast<int64_t>(fs);
            int n = 0, reg = 0;
            double sum = 0.0;
            int cnt = 0;
            for (size_t k = 1; k < vpos.size(); ++k) {
                if (vpos[k] < lo || vpos[k] >= hi) continue;
                ++n;
                double sp = static_cast<double>(vpos[k] - vpos[k - 1]);
                sum += sp;
                ++cnt;
                // "regular" if within 12% of one field (or two, for
                // frame-rate detection or a single missed field)
                if (std::abs(sp - field) < 0.12 * field ||
                    std::abs(sp - 2 * field) < 0.12 * 2 * field)
                    ++reg;
            }
            if (n)
                std::printf("%2d-%2d  %4d    %5.0f%%   %6.1f\n", b * 2,
                            b * 2 + 2, n, cnt ? 100.0 * reg / cnt : 0.0,
                            cnt ? sum / cnt / fs * 1000.0 : 0.0);
        }
    }
    std::fprintf(stderr, "done: %.2f s, %llu frames\n",
                 static_cast<double>(total_samples) / fs,
                 static_cast<unsigned long long>(dec.stats().frames.load()));
    return 0;
}
