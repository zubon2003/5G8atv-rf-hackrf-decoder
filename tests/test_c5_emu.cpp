// Checks of the --c5 lane emulation (src/dsp/c5_emu.hpp) against the
// Direct Gain V5 behaviour it reproduces.
#include <cmath>
#include <cstdlib>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

#include "dsp/c5_emu.hpp"

using namespace famidec;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++failures;
}

constexpr double kFs = 10e6;

// FM carrier (5 MHz/s-ish video-like tone) of amplitude `amp` plus complex
// noise of rms `noise`, `seconds` long, amplitude scaled per sample by env().
template <class Env>
std::vector<std::complex<float>> signal(double seconds, double amp, double noise, Env env,
                                        unsigned seed = 1) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> g(0.0f, static_cast<float>(noise / std::sqrt(2.0)));
    const size_t n = static_cast<size_t>(seconds * kFs);
    std::vector<std::complex<float>> x(n);
    double ph = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double t = k / kFs;
        const double f = 2.0e6 * std::sin(2 * M_PI * 15734.0 * t);  // line-rate sweep
        ph += 2 * M_PI * f / kFs;
        const double a = amp * env(t);
        x[k] = {static_cast<float>(a * std::cos(ph)) + g(rng),
                static_cast<float>(a * std::sin(ph)) + g(rng)};
    }
    return x;
}

}  // namespace

int main() {
    // Lane: ADC bits {9,6,5,4}. Small codes are code/16; |code| >= 128 wraps.
    {
        C5EmuStats st;
        C5Emulator e(kFs, 0.0, -30.0, &st);
        std::complex<float> x[4] = {{100.0f / 512, -1.0f / 512}, {200.0f / 512, 0}, {-600.0f / 512, 0}, {0, 0}};
        e.process(x, 4);
        check(std::fabs(x[0].real() - 13.0f / 16) < 1e-6 && std::fabs(x[0].imag() + 1.0f / 16) < 1e-6,
              "lane: +100 -> cell 6, -1 -> cell -1");
        check(std::fabs(x[1].real() - 9.0f / 16) < 1e-6, "lane: +200 wraps to cell 4 (bits 8..7 dropped)");
        check(std::fabs(x[2].real() + 15.0f / 16) < 1e-6, "lane: -600 rails at -512 -> cell -8");
    }
    // Steady carrier: acquires into the healthy band, then no writes.
    {
        C5EmuStats st;
        C5Emulator e(kFs, 6.0, -30.0, &st);
        auto x = signal(0.6, 0.30, 0.01, [](double) { return 1.0; });
        e.process(x.data(), x.size() / 2);
        const uint32_t after_acquire = st.writes.load();
        e.process(x.data() + x.size() / 2, x.size() - x.size() / 2);
        std::printf("     steady: gain %.1f dB p50 %d coh %d writes %u (late %u)\n", st.gain_db.load(),
                    st.p50.load(), st.coherence.load(), st.writes.load(), st.writes.load() - after_acquire);
        check(st.p50.load() >= 13 && st.p50.load() <= 32, "steady: P50 inside the healthy band");
        check(st.writes.load() - after_acquire == 0, "steady: zero writes once healthy");
    }
    // A 12 dB fade lasting 100 ms: corrected (moderate/severe), then back.
    {
        C5EmuStats st;
        C5Emulator e(kFs, 6.0, -30.0, &st);
        auto x = signal(0.8, 0.30, 0.004, [](double t) { return (t > 0.3 && t < 0.4) ? 0.25 : 1.0; });
        e.process(x.data(), x.size());
        std::printf("     fade: writes %u severe %u moderate %u p50 %d\n", st.writes.load(),
                    st.severe.load(), st.moderate.load(), st.p50.load());
        check(st.writes.load() >= 2 && st.writes.load() <= 8, "fade: a few writes, no hunting");
        check(st.p50.load() >= 13 && st.p50.load() <= 32, "fade: back in the healthy band");
    }
    // A 0.3 ms full-scale burst is ignored; a 5 ms one is an overload.
    {
        C5EmuStats st;
        C5Emulator e(kFs, 6.0, -30.0, &st);
        auto x = signal(0.5, 0.30, 0.01, [](double t) {
            if (t > 0.30 && t < 0.3003) return 30.0;
            return 1.0;
        });
        // Acquisition from the maximum gain overloads on a strong carrier (as
        // on the board); count only from just before the burst.
        const size_t pre = static_cast<size_t>(0.29 * kFs);
        e.process(x.data(), pre);
        const uint32_t b0 = st.bursts.load(), o0 = st.overloads.load(), w0 = st.writes.load();
        e.process(x.data() + pre, x.size() - pre);
        std::printf("     burst 0.3 ms: bursts +%u overloads +%u writes +%u\n", st.bursts.load() - b0,
                    st.overloads.load() - o0, st.writes.load() - w0);
        check(st.bursts.load() > b0 && st.overloads.load() == o0 && st.writes.load() == w0,
              "burst 0.3 ms: ignored, no overload, no write");
        C5EmuStats st2;
        C5Emulator e2(kFs, 6.0, -30.0, &st2);
        auto y = signal(0.5, 0.30, 0.01, [](double t) { return (t > 0.30 && t < 0.305) ? 30.0 : 1.0; });
        e2.process(y.data(), y.size());
        check(st2.overloads.load() >= 1, "burst 5 ms: overload drop");
        check(st2.p50.load() >= 13 && st2.p50.load() <= 32, "after overload: healthy again");
    }
    if (std::getenv("C5_SWEEP")) {
        for (double fixed : {-3.0, 0.0, 3.0})
            for (double amp : {0.10, 0.14, 0.18, 0.24, 0.30, 0.40, 0.50}) {
                auto x = signal(0.6, amp, 0.02, [](double) { return 1.0; }, 3);
                C5EmuStats s;
                // Start at a fixed gain (max = start) to land in the wrap zone.
                C5Emulator e(kFs, fixed, -30.0, &s, false);
                e.process(x.data(), x.size());
                std::printf("     sweep start %+.0f dB amp %.2f: gain %+.1f p50 %2d p95 %2d coh %2d writes %u damp %u\n",
                            fixed, amp, s.gain_db.load(), s.p50.load(), s.p95.load(), s.coherence.load(),
                            s.writes.load(), s.damp_events.load());
            }
    }
    // Experimental wrap detection (--c5-wrap). A clean carrier appearing at
    // the listening gain converges without it and must not trigger it; a
    // sustained strong noise-like signal (the rule's target, and what Wi-Fi
    // looks like) drops the gain.
    {
        auto x = signal(1.0, 0.17, 0.01, [](double t) { return t < 0.1 ? 0.0 : 1.0; });
        C5EmuStats on;
        C5Emulator e_on(kFs, 6.0, -30.0, &on, true);
        e_on.process(x.data(), x.size());
        std::printf("     wrap rule, clean carrier: gain %.1f dB p50 %d coh %d writes %u wraps %u\n",
                    on.gain_db.load(), on.p50.load(), on.coherence.load(), on.writes.load(), on.wraps.load());
        check(on.wraps.load() == 0 && on.p50.load() >= 13 && on.p50.load() <= 32,
              "wrap rule: clean carrier untouched, healthy");
        auto n = signal(0.2, 0.0, 0.20, [](double) { return 1.0; }, 5);
        C5EmuStats a, b;
        C5Emulator e_a(kFs, 0.0, -30.0, &a, true), e_b(kFs, 0.0, -30.0, &b, false);
        auto n2 = n;
        e_a.process(n.data(), n.size());
        e_b.process(n2.data(), n2.size());
        std::printf("     wrap rule, strong noise: on gain %.1f dB wraps %u | off p95 %d coh %d gain %.1f dB\n",
                    a.gain_db.load(), a.wraps.load(), b.p95.load(), b.coherence.load(), b.gain_db.load());
        check(a.wraps.load() >= 1 && a.gain_db.load() < b.gain_db.load(),
              "wrap rule: sustained strong noise-like signal drops the gain");
    }
    // Auto listening gain: quiet and loud HackRF noise both end at P50 3..5
    // as on the C5's table maximum; a carrier leaves the listening gain alone.
    for (double rms : {0.002, 0.004, 0.05}) {  // 0.05: louder than VGA 10 gives
        auto x = signal(1.5, 0.0, rms, [](double) { return 1.0; }, 7);
        C5EmuStats s;
        C5Emulator e(kFs, 20.0, -30.0, &s, false, true);
        e.process(x.data(), x.size());
        std::printf("     auto listen, noise rms %.3f: listen %+.1f dB p50 %d coh %d writes %u\n", rms,
                    s.listen_db.load(), s.p50.load(), s.coherence.load(), s.writes.load());
        check(s.p50.load() >= 3 && s.p50.load() <= 5,
              "auto listen: noise ends at P50 3..5");
    }
    {
        auto x = signal(0.5, 0.30, 0.004, [](double) { return 1.0; });
        C5EmuStats s;
        C5Emulator e(kFs, 20.0, -30.0, &s, false, true);
        e.process(x.data(), x.size());
        check(std::fabs(s.listen_db.load() - 20.0f) < 0.01f && s.p50.load() >= 13 && s.p50.load() <= 32,
              "auto listen: a carrier does not move the listening gain");
    }
    // Noise only: listens at the maximum gain.
    {
        C5EmuStats st;
        C5Emulator e(kFs, 6.0, -30.0, &st);
        auto x = signal(0.3, 0.0, 0.02, [](double) { return 1.0; });
        e.process(x.data(), x.size());
        std::printf("     noise: gain %.1f dB p50 %d coh %d\n", st.gain_db.load(), st.p50.load(),
                    st.coherence.load());
        check(std::fabs(st.gain_db.load() - 6.0f) < 0.01f, "noise only: stays at the maximum gain");
    }
    std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
