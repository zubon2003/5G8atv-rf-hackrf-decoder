#pragma once

#include <string>

namespace famidec {

struct Config {
    // RF
    double video_carrier_hz = 5.800e9;  // 5.8 GHz FPV band F, channel 4
    double sample_rate = 10e6;          // FPV FM video fits in +-4.5 MHz
                                        // (measured: <0.3% energy outside);
                                        // halves CPU and USB load vs 20 MSPS
    double offset_hz = 0.0;             // no offset tuning: DC spike removed by
                                        // the DcBlocker (band too wide to offset)
    int lna_gain = 40;         // 0-40, step 8. Max: FPV signals are weak
                               // and the LNA is what buys range (the VGA
                               // only scales; use --amp for more)
    int vga_gain = 20;         // 0-62, step 2
    bool amp = true;           // +14 dB RF preamp; FPV signals are weak
                               // (--no-amp or the b key to disable)
    bool gain_auto = true;     // auto LNA/VGA from ADC peak + clip stats;
                               // manual via --gain manual, explicit
                               // --lna/--vga, or the l/g keys. The RF amp
                               // is never auto-switched (intermod risk).

    enum class Input { HackRF, File };
    Input input = Input::HackRF;
    std::string file_path;
    bool loop = false;

    enum class Mode { Color, Gray };
    Mode mode = Mode::Color;

    // FM-ATV parameters.
    double fm_dev_hz = 5.0e6;       // FM peak deviation (--dev)
    bool invert = false;            // --invert: flip discriminator polarity
                                    // for non-standard VTX
    bool afc = true;                // auto re-tune onto the VTX center
                                    // (--no-afc to disable)
    bool freerun_display = false;   // when unlocked, draw the composite on a
                                    // fixed nominal H/V raster (rolls but
                                    // shows a present signal) instead of
                                    // per-line snow. --freerun or the f key.
    double video_lpf_hz = 0.0;      // post-detector real LPF, 0 = off.
                                    // At 10 MSPS the FM audio subcarrier is
                                    // outside Nyquist, so this is purely an
                                    // optional noise filter (--lpf 4.2e6
                                    // limits to the NTSC video bandwidth)

    std::string record_path;          // tee raw IQ to .cs8 while decoding
    std::string dump_composite_path;  // post-AGC composite as f32
    bool spectrum = false;            // PSD printout mode, no video
    bool headless = false;            // decode without SDL window (dump frames)
    std::string dump_frames_prefix;   // write decoded frames as PPM
    int dump_frame_count = 0;

    // Color trims
    float saturation = 1.0f;
    float hue_deg = 0.0f;

    // Horizontal overscan crop, fraction of the active line removed from
    // EACH side. 0 = show the full active line: FPV cameras put OSD text
    // right at the edges (TV-style hiding was 0.047).
    float overscan = 0.0f;

    double center_hz() const { return video_carrier_hz + offset_hz; }
};

}  // namespace famidec
