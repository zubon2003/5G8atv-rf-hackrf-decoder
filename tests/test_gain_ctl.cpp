// Unit tests for the C5VRX-style fast gain control (src/dsp/gain_ctl.hpp):
// the |IQ| statistics on synthetic cs8 blocks, and the controller decisions
// on scripted observation sequences. No hardware.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "dsp/gain_ctl.hpp"

using namespace famidec;

static int g_fail = 0;
#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (!(cond)) {                                          \
            ++g_fail;                                           \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__);    \
            std::printf(__VA_ARGS__);                           \
            std::printf("\n");                                  \
        }                                                       \
    } while (0)

// cs8 block: FM carrier of amplitude `amp` (codes) with a slow sine message
// of +-dev_hz at 10 MSPS, plus complex noise and a DC offset like the
// HackRF's LO leakage.
static std::vector<uint8_t> make_block(double amp, double noise, double dev_hz,
                                       double dc = 3.0, size_t n = 131072,
                                       unsigned seed = 1) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> g(0.0, 1.0);
    std::vector<uint8_t> out(2 * n);
    const double fs = 10e6;
    double ph = 0.0;
    for (size_t k = 0; k < n; ++k) {
        double f = 1.0e6 + dev_hz * std::sin(2 * M_PI * 15734.0 * k / fs);
        ph += 2 * M_PI * f / fs;
        double i = amp * std::cos(ph) + noise * g(rng) + dc;
        double q = amp * std::sin(ph) + noise * g(rng) + dc;
        auto q8 = [](double v) {
            long r = std::lround(v);
            if (r > 127) r = 127;
            if (r < -128) r = -128;
            return static_cast<uint8_t>(static_cast<int8_t>(r));
        };
        out[2 * k] = q8(i);
        out[2 * k + 1] = q8(q);
    }
    return out;
}

static IqStats stats_of(double amp, double noise, double dev = 2.5e6) {
    auto b = make_block(amp, noise, dev);
    return compute_iq_stats(b.data(), b.size(), 4);
}

static void test_stats() {
    IqStats s = stats_of(40.0, 2.0);
    CHECK(std::abs(s.p50 - 40) <= 3, "carrier p50 %d, want ~40", s.p50);
    CHECK(s.clip_frac == 0.0f, "no clipping expected, got %f", s.clip_frac);
    CHECK(s.coherence > 0.3f, "FM carrier coherence %.2f, want > 0.3",
          s.coherence);

    IqStats n = stats_of(0.0, 20.0);
    CHECK(n.coherence < 0.1f, "noise coherence %.2f (DC removed), want < 0.1",
          n.coherence);

    IqStats c = stats_of(200.0, 2.0);
    CHECK(c.clip_frac > 0.1f, "overdriven carrier should clip, got %f",
          c.clip_frac);
    CHECK(c.p95 >= 118, "overdriven p95 %d", c.p95);
}

static IqStats obs(uint64_t seq, int p50, int p95, float coh = 0.8f,
                   float clip = 0.0f) {
    IqStats s;
    s.seq = seq;
    s.p50 = p50;
    s.p95 = p95;
    s.coherence = coh;
    s.clip_frac = clip;
    s.n = 32768;
    return s;
}

static void test_hold_in_band() {
    FastGainController c;
    GainSetting cur{40, 20}, next;
    for (uint64_t k = 1; k <= 100; ++k)
        CHECK(!c.update(obs(k, 34, 60), k * 0.013, cur, &next),
              "write in band at %llu", static_cast<unsigned long long>(k));
    CHECK(c.writes() == 0 && c.healthy() == 100, "hold: writes %llu healthy %llu",
          static_cast<unsigned long long>(c.writes()),
          static_cast<unsigned long long>(c.healthy()));
}

static void test_one_jump() {
    FastGainController c;
    GainSetting cur{40, 20}, next;
    // Median at half the target: +6 dB in one write, VGA only.
    CHECK(c.update(obs(1, 17, 30), 0.0, cur, &next), "weak carrier: no write");
    CHECK(next.lna == 40 && next.vga == 26, "weak: got L%d V%d, want L40 V26",
          next.lna, next.vga);
    c.written(1);
    // The next transfers still carry the old gain: ignored while settling.
    for (uint64_t k = 2; k <= 6; ++k)
        CHECK(!c.update(obs(k, 17, 30), k * 0.013, next, &next),
              "acted on unsettled transfer %llu", static_cast<unsigned long long>(k));
    CHECK(c.update(obs(7, 17, 30), 0.1, GainSetting{40, 26}, &next),
          "settled out-of-band observation should act again");
}

static void test_emergency() {
    FastGainController c;
    GainSetting cur{40, 30}, next;
    CHECK(c.update(obs(1, 120, 175, 0.8f, 0.2f), 0.0, cur, &next),
          "clipping must write");
    CHECK(cur.total() - next.total() >= 12, "heavy clip: only %d dB down",
          cur.total() - next.total());
    CHECK(c.emergencies() == 1, "emergency not counted");
}

static void test_noise_listens() {
    FastGainController c;
    GainSetting cur{40, 40}, next;
    // Loud noise, no carrier, no clipping: keep the gain (listen).
    CHECK(!c.update(obs(1, 80, 110, 0.05f), 0.0, cur, &next),
          "noise without a carrier must not pull the gain down");
    // Quiet noise, no carrier: raise towards the band.
    CHECK(c.update(obs(2, 10, 20, 0.05f), 0.02, cur, &next) &&
              next.total() > cur.total(),
          "quiet noise should raise the gain");
}

static void test_split() {
    GainSetting g = FastGainController::split(70, 40);  // VGA 30 fits
    CHECK(g.lna == 40 && g.vga == 30, "split(70,40) L%d V%d", g.lna, g.vga);
    g = FastGainController::split(20, 40);  // VGA would go negative
    CHECK(g.lna == 0 && g.vga == 20, "split(20,40) L%d V%d", g.lna, g.vga);
    g = FastGainController::split(102, 0);  // VGA would exceed 62
    CHECK(g.lna == 40 && g.vga == 62, "split(102,0) L%d V%d", g.lna, g.vga);
}

static void test_anti_hunt() {
    FastGainParams p;
    p.settle_transfers = 0;
    FastGainController c(p);
    GainSetting cur{40, 30}, next;
    double t = 0.0;
    uint64_t seq = 0;
    auto step = [&](int p50) {
        ++seq;
        t += 0.013;
        // P95 stays below the overload line: this is about hunting, not
        // the (never damped) emergency path.
        bool w = c.update(obs(seq, p50, p50 + 20), t, cur, &next);
        if (w) {
            cur = next;
            c.written(seq);
        }
        return w;
    };
    CHECK(step(15), "up 1");    // up
    CHECK(step(70), "down 1");  // reversal #1
    CHECK(step(15), "up 2");    // reversal #2 within 0.3 s -> damped
    CHECK(c.damped(t), "two quick reversals should damp");
    // Damped: needs damp_windows (4) consecutive out-of-band windows.
    CHECK(!step(70) && !step(70) && !step(70), "acted before 4 windows");
    CHECK(step(70), "4th consecutive window should act");
    // Overload is never damped.
    ++seq;
    t += 0.013;
    CHECK(c.update(obs(seq, 120, 170, 0.8f, 0.05f), t, cur, &next),
          "overload must bypass damping");
}

int main() {
    test_stats();
    test_hold_in_band();
    test_one_jump();
    test_emergency();
    test_noise_listens();
    test_split();
    test_anti_hunt();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("gain_ctl: all checks passed\n");
    return 0;
}
