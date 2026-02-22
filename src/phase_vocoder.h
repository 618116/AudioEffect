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
        fft_size_ = kFixedFftSize_;
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
            std::fill(synthesis_accum_[ch].begin(), synthesis_accum_[ch].end(), 0.0f);
            std::fill(synthesis_norm_[ch].begin(), synthesis_norm_[ch].end(), 0.0f);
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
    static constexpr int kFixedFftSize_ = 512;
    static constexpr float kPi_ = 3.14159265358979323846f;
    static constexpr float kTwoPi_ = 2.0f * kPi_;
    int fft_size_ = 0;
    int analysis_hop_ = 0;
    int bin_count_ = 0;
    double analysis_read_pos_ = 0.0;
    bool has_phase_history_ = false;
    SimpleFFT fft_;
    static constexpr float kNormFloor_ = 1.0e-6f;
    static constexpr float kDrainEpsilon_ = 1.0e-8f;

    std::vector<float> window_;
    std::vector<std::vector<float>> time_frame_;
    std::vector<std::vector<float>> synthesis_accum_;
    std::vector<std::vector<float>> synthesis_norm_;
    std::vector<std::vector<float>> hop_frame_;
    std::vector<float*> hop_ptrs_;

    std::vector<std::vector<std::complex<float>>> spectrum_;
    std::vector<std::vector<float>> prev_phase_;
    std::vector<std::vector<float>> sum_phase_;

    static float wrap_phase(float phase) {
        phase = std::fmod(phase + kPi_, kTwoPi_);
        if (phase < 0.0f) phase += kTwoPi_;
        return phase - kPi_;
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
        synthesis_accum_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
        synthesis_norm_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
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
        int valid_samples = fft_size_;
        bool frame_is_fully_padded = false;

        if (read_pos + fft_size_ > available) {
            if (!input_ended_) return false;
            valid_samples = std::max(0, available - read_pos);
            frame_is_fully_padded = (valid_samples == 0);
        }

        if (output_ring_buffer_.writable() < syn_hop) return false;

        float hop_peak = 0.0f;
        for (int ch = 0; ch < channel_count_; ++ch) {
            auto& frame = time_frame_[ch];
            auto& accum = synthesis_accum_[ch];
            auto& norm = synthesis_norm_[ch];

            std::fill(frame.begin(), frame.end(), 0.0f);
            if (valid_samples > 0) {
                input_ring_buffer_.peek(ch, frame.data(), 0, read_pos, valid_samples);
            }

            for (int i = 0; i < fft_size_; ++i) {
                frame[i] *= window_[i];
            }

            fft_.forward_real(frame.data(), spectrum_[ch].data());

            // Basic phase vocoder update: estimate per-bin phase increment on
            // analysis hop, then propagate phase on synthesis hop.
            auto& prev = prev_phase_[ch];
            auto& sum = sum_phase_[ch];
            for (int k = 0; k < bin_count_; ++k) {
                const float mag = std::abs(spectrum_[ch][k]);
                const float phase = std::arg(spectrum_[ch][k]);

                if (!has_phase_history_) {
                    prev[k] = phase;
                    sum[k] = phase;
                    spectrum_[ch][k] = std::polar(mag, phase);
                    continue;
                }

                const float expected_analysis =
                    kTwoPi_ * static_cast<float>(k) *
                    static_cast<float>(analysis_hop_) /
                    static_cast<float>(fft_size_);
                const float expected_synthesis =
                    kTwoPi_ * static_cast<float>(k) *
                    static_cast<float>(syn_hop) /
                    static_cast<float>(fft_size_);

                const float delta = wrap_phase(phase - prev[k] - expected_analysis);
                sum[k] = wrap_phase(sum[k] + expected_synthesis +
                                    delta * static_cast<float>(syn_hop) /
                                        static_cast<float>(analysis_hop_));
                prev[k] = phase;
                spectrum_[ch][k] = std::polar(mag, sum[k]);
            }

            fft_.inverse_real(spectrum_[ch].data(), frame.data());

            for (int i = 0; i < fft_size_; ++i) {
                const float w = window_[i];
                const float w2 = w * w;
                accum[i] += frame[i] * w;
                norm[i] += w2;
            }

            for (int i = 0; i < syn_hop; ++i) {
                const float denom = norm[i];
                const float out =
                    (denom > kNormFloor_) ? (accum[i] / denom) : 0.0f;
                hop_frame_[ch][i] = out;
                hop_peak = std::max(hop_peak, std::fabs(out));
            }

            const int remain = fft_size_ - syn_hop;
            if (remain > 0) {
                std::move(accum.begin() + syn_hop, accum.end(), accum.begin());
                std::move(norm.begin() + syn_hop, norm.end(), norm.begin());
            }
            std::fill(accum.begin() + remain, accum.end(), 0.0f);
            std::fill(norm.begin() + remain, norm.end(), 0.0f);
        }

        if (input_ended_ && frame_is_fully_padded && hop_peak <= kDrainEpsilon_) {
            return false;
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
