#include "hackrf_source.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <thread>

namespace famidec {

namespace {
constexpr size_t kRingBytes = 1u << 25;  // 32 MiB ~ 1.6 s at 10 MSPS
}

HackRfSource::HackRfSource(const Config& cfg)
    : cfg_(cfg), ring_(kRingBytes), lna_(cfg.lna_gain), vga_(cfg.vga_gain) {}

HackRfSource::~HackRfSource() { stop(); }

int HackRfSource::rx_callback(hackrf_transfer* transfer) {
    auto* self = static_cast<HackRfSource*>(transfer->rx_ctx);
    if (!self->running_.load(std::memory_order_relaxed)) return -1;
    const uint8_t* data = transfer->buffer;
    size_t len = static_cast<size_t>(transfer->valid_length);
    // Cheap clip + peak detection on a stride to keep the USB thread light.
    uint64_t clips = 0;
    int mx = 0;
    for (size_t i = 0; i < len; i += 64) {
        int v = static_cast<int8_t>(data[i]);
        if (v < 0) v = -v;
        if (v > mx) mx = v;
        if (v >= 127) ++clips;
    }
    if (clips) self->clipped_.fetch_add(clips, std::memory_order_relaxed);
    // Amplitude distribution + carrier coherence for the fast gain control
    // (every 4th sample: ~33k probes per 256 KiB transfer).
    {
        IqStats st = compute_iq_stats(data, len, 4);
        st.seq = self->transfer_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> lk(self->stats_mu_);
        self->stats_ = st;
    }
    int cur = self->peak_.load(std::memory_order_relaxed);
    while (mx > cur &&
           !self->peak_.compare_exchange_weak(cur, mx,
                                              std::memory_order_relaxed)) {
    }
    self->total_.fetch_add(len, std::memory_order_relaxed);
    if (!self->ring_.push(data, len))
        self->dropped_.fetch_add(len, std::memory_order_relaxed);
    return 0;
}

bool HackRfSource::start() {
    int r = hackrf_init();
    if (r != HACKRF_SUCCESS) {
        error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    r = hackrf_open(&dev_);
    if (r != HACKRF_SUCCESS) {
        error_ = std::string("hackrf_open: ") +
                 hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    hackrf_set_sample_rate(dev_, cfg_.sample_rate);
    // 0.9*rate, not the usual 0.75: at 10 MSPS the analog filter must stay
    // open through the chroma upper sideband (+-4.6 MHz), or color washes
    // out just like a too-narrow digital channel filter.
    hackrf_set_baseband_filter_bandwidth(
        dev_, hackrf_compute_baseband_filter_bw(
                  static_cast<uint32_t>(cfg_.sample_rate * 0.9)));
    hackrf_set_freq(dev_, static_cast<uint64_t>(cfg_.center_hz()));
    hackrf_set_amp_enable(dev_, cfg_.amp ? 1 : 0);
    hackrf_set_lna_gain(dev_, static_cast<uint32_t>(lna_));
    hackrf_set_vga_gain(dev_, static_cast<uint32_t>(vga_));
    running_.store(true, std::memory_order_relaxed);
    r = hackrf_start_rx(dev_, rx_callback, this);
    if (r != HACKRF_SUCCESS) {
        error_ = std::string("hackrf_start_rx: ") +
                 hackrf_error_name(static_cast<hackrf_error>(r));
        running_.store(false, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void HackRfSource::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (dev_) {
        hackrf_stop_rx(dev_);
        hackrf_close(dev_);
        dev_ = nullptr;
        hackrf_exit();
    }
}

size_t HackRfSource::read(uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len && running_.load(std::memory_order_relaxed)) {
        size_t n = ring_.pop(buf + got, len - got);
        got += n;
        if (n == 0)
            std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return got;
}

float HackRfSource::ring_fill() const {
    return static_cast<float>(ring_.readable()) /
           static_cast<float>(ring_.capacity());
}

bool HackRfSource::set_center_freq(double center_hz) {
    if (!dev_) return false;
    return hackrf_set_freq(dev_, static_cast<uint64_t>(center_hz)) ==
           HACKRF_SUCCESS;
}

bool HackRfSource::set_amp(bool on) {
    if (!dev_) return false;
    return hackrf_set_amp_enable(dev_, on ? 1 : 0) == HACKRF_SUCCESS;
}

bool HackRfSource::set_gains(int lna, int vga) {
    lna_ = std::clamp((lna / 8) * 8, 0, 40);
    vga_ = std::clamp((vga / 2) * 2, 0, 62);
    if (!dev_) return false;
    return hackrf_set_lna_gain(dev_, static_cast<uint32_t>(lna_)) ==
               HACKRF_SUCCESS &&
           hackrf_set_vga_gain(dev_, static_cast<uint32_t>(vga_)) ==
               HACKRF_SUCCESS;
}

}  // namespace famidec
