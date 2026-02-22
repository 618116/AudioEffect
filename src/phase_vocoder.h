#pragma once
// ============================================================================
// phase_vocoder.h - Outline for a phase-vocoder time stretcher
//
// This file is intentionally an implementation outline.
// It wires state, data flow, and method boundaries so the DSP parts can be
// filled in incrementally.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include "simple_fft.h"
#include "time_stretcher.h"

class PhaseVocoder : public TimeStretcher {
protected:
    void onInit() override {
        fft_size_ = next_power_of_two(frame_size_);
        analysis_hop_ = std::max(1, fft_size_ / 4);  // Typical 75% overlap.
        bin_count_ = (fft_size_ / 2) + 1;
        fft_.init(fft_size_);

        generate_hann_window(fft_size_, window_);
        resize_state_buffers();
    }

    void onReset() override {
        analysis_read_pos_ = 0.0;
        has_phase_history_ = false;

        for (int ch = 0; ch < channel_count_; ++ch) {
            std::fill(hop_frame_[ch].begin(), hop_frame_[ch].end(), 0.0f);
            std::fill(time_frame_[ch].begin(), time_frame_[ch].end(), 0.0f);
            std::fill(prev_phase_[ch].begin(), prev_phase_[ch].end(), 0.0f);
            std::fill(sum_phase_[ch].begin(), sum_phase_[ch].end(), 0.0f);
        }
    }

    void produce_frames(int needed_output) override {
        // Keep producing hop-sized chunks until enough output is buffered.
        while (output_ring_buffer_.buffered() < needed_output) {
            if (!produce_one_hop()) {
                break;
            }
        }
        compact();
    }

private:
    int fft_size_ = 0;
    int analysis_hop_ = 0;
    int bin_count_ = 0;
    double analysis_read_pos_ = 0.0;
    bool has_phase_history_ = false;
    SimpleFFT fft_;

    std::vector<float> window_;
    std::vector<std::vector<float>> time_frame_;
    std::vector<std::vector<float>> hop_frame_;
    std::vector<float*> hop_ptrs_;

    std::vector<std::vector<std::complex<float>>> spectrum_;
    std::vector<std::vector<float>> prev_phase_;
    std::vector<std::vector<float>> sum_phase_;

    static int next_power_of_two(int value) {
        int n = 1;
        while (n < value) n <<= 1;
        return std::max(2, n);
    }

    int synthesis_hop() const {
        // Ratio > 1.0 means slower output, so synthesis hop is smaller.
        float ratio = std::max(0.01f, time_stretch_ratio_);
        int hop = static_cast<int>(std::lround(
            static_cast<double>(analysis_hop_) / static_cast<double>(ratio)));
        return std::max(1, hop);
    }

    void resize_state_buffers() {
        time_frame_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
        hop_frame_.assign(channel_count_, std::vector<float>(analysis_hop_, 0.0f));
        spectrum_.assign(
            channel_count_, std::vector<std::complex<float>>(bin_count_));
        prev_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        sum_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));

        hop_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            hop_ptrs_[ch] = hop_frame_[ch].data();
        }
    }

    bool produce_one_hop() {
        const int syn_hop = synthesis_hop();
        if (static_cast<int>(hop_frame_[0].size()) != syn_hop) {
            hop_frame_.assign(channel_count_, std::vector<float>(syn_hop, 0.0f));
            for (int ch = 0; ch < channel_count_; ++ch) {
                hop_ptrs_[ch] = hop_frame_[ch].data();
            }
        }

        const int read_pos = static_cast<int>(std::floor(analysis_read_pos_));
        const int available = input_ring_buffer_.buffered();

        if (read_pos + fft_size_ > available) {
            if (!input_ended_) return false;
            // End-of-input handling is intentionally left as TODO.
            return false;
        }

        if (output_ring_buffer_.writable() < syn_hop) return false;

        for (int ch = 0; ch < channel_count_; ++ch) {
            input_ring_buffer_.peek(ch, time_frame_[ch].data(), 0, read_pos, fft_size_);

            // TODO(phase-vocoder):
            // 1) Apply analysis window.
            // 2) Run FFT (fft_.forward_real) -> spectrum_[ch].
            // 3) Compute phase delta vs prev_phase_[ch] using expected bin advance.
            // 4) Accumulate corrected phase into sum_phase_[ch].
            // 5) Rebuild complex bins with stretched phase.
            // 6) Run IFFT (fft_.inverse_real) and overlap-add into synthesis buffer.
            // 7) Export the next syn_hop samples into hop_frame_[ch].

            // Placeholder so the class stays compile-safe until DSP is added.
            std::fill(hop_frame_[ch].begin(), hop_frame_[ch].end(), 0.0f);
        }

        has_phase_history_ = true;
        output_ring_buffer_.write(
            const_cast<const float* const*>(hop_ptrs_.data()), 0, syn_hop);
        analysis_read_pos_ += static_cast<double>(analysis_hop_);
        return true;
    }

    void compact() {
        // Keep one FFT frame of history before the read position.
        int safe_discard = static_cast<int>(std::floor(analysis_read_pos_)) - fft_size_;
        if (safe_discard <= 0) return;

        const int to_discard = std::min(safe_discard, input_ring_buffer_.buffered());
        input_ring_buffer_.discard(to_discard);
        analysis_read_pos_ -= static_cast<double>(to_discard);
    }
};
