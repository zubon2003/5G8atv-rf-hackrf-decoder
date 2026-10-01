#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <libhackrf/hackrf.h>

#include "../config.hpp"
#include "../dsp/gain_ctl.hpp"
#include "../dsp/ring_buffer.hpp"
#include "sample_source.hpp"

namespace famidec {

class HackRfSource : public ISampleSource {
public:
    explicit HackRfSource(const Config& cfg);
    ~HackRfSource() override;

    bool start() override;
    void stop() override;
    size_t read(uint8_t* buf, size_t len) override;

    uint64_t dropped_bytes() const override {
        return dropped_.load(std::memory_order_relaxed);
    }
    uint64_t total_bytes() const override {
        return total_.load(std::memory_order_relaxed);
    }
    uint64_t buffered_bytes() const override { return ring_.readable(); }
    uint64_t clipped_samples() const override {
        return clipped_.load(std::memory_order_relaxed);
    }
    float ring_fill() const override;

    bool set_gains(int lna, int vga);
    bool set_center_freq(double center_hz);
    bool set_amp(bool on);
    int lna() const { return lna_; }
    int vga() const { return vga_; }

    // Peak |sample| (0..127) observed since the last call; resets on read.
    // Fed by the same strided scan as the clip counter, for the gain AGC.
    int take_peak() { return peak_.exchange(0, std::memory_order_relaxed); }

    // |IQ| statistics of the most recent USB transfer, for the fast gain
    // control. Returns false when nothing newer than `after_seq` arrived.
    bool latest_stats(uint64_t after_seq, IqStats* out) {
        std::lock_guard<std::mutex> lk(stats_mu_);
        if (stats_.seq <= after_seq) return false;
        *out = stats_;
        return true;
    }
    // Number of USB transfers received so far (IqStats::seq of the newest).
    uint64_t transfer_seq() const {
        return transfer_seq_.load(std::memory_order_relaxed);
    }

    const std::string& error() const { return error_; }

private:
    static int rx_callback(hackrf_transfer* transfer);

    const Config& cfg_;
    hackrf_device* dev_ = nullptr;
    SpscRing ring_;
    std::atomic<uint64_t> dropped_{0};
    std::atomic<uint64_t> total_{0};
    std::atomic<uint64_t> clipped_{0};
    std::atomic<int> peak_{0};
    std::atomic<uint64_t> transfer_seq_{0};
    std::mutex stats_mu_;
    IqStats stats_;
    std::atomic<bool> running_{false};
    int lna_, vga_;
    std::string error_;
};

}  // namespace famidec
