#pragma once
// ============================================================================
// wsola.h - Time-stretcher with overlap-add (OLA, no similarity search)
//
// Inherits I/O ring buffer management from TimeStretcher.
// Implements produce_frames(): hop-based overlap-add at ratio-adjusted positions.
// ============================================================================

#include "time_stretcher.h"

class WSOLA : public TimeStretcher {
protected:
    void onInit() override {
        synthesis_hop_ = std::max(1, frame_size_ / 2);

        fade_in_.assign(synthesis_hop_, 0.0f);
        fade_out_.assign(synthesis_hop_, 0.0f);

        prev_frame_.assign(channel_count_, std::vector<float>(frame_size_, 0.0f));
        hop_frame_.assign(channel_count_, std::vector<float>(synthesis_hop_, 0.0f));
        hop_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            hop_ptrs_[ch] = hop_frame_[ch].data();
        }

        if (synthesis_hop_ == 1) {
            fade_in_[0] = 1.0f;
            fade_out_[0] = 0.0f;
            return;
        }

        const double pi = 3.14159265358979323846;
        const double denom = static_cast<double>(synthesis_hop_ - 1);
        for (int i = 0; i < synthesis_hop_; ++i) {
            double x = static_cast<double>(i) / denom;
            float fade_in = static_cast<float>(0.5 - 0.5 * std::cos(pi * x));
            fade_in_[i] = fade_in;
            fade_out_[i] = 1.0f - fade_in;
        }
    }

    void produce_frames(int needed_output) override {
        while (output_ring_buffer_.buffered() < needed_output) {
            if (!produce_one_hop()) {
                break;
            }
        }
        compact();
    }

    void onReset() override {
        read_position_ = 0.0;
        has_prev_frame_ = false;
        for (int ch = 0; ch < channel_count_; ++ch) {
            std::fill(prev_frame_[ch].begin(), prev_frame_[ch].end(), 0.0f);
            std::fill(hop_frame_[ch].begin(), hop_frame_[ch].end(), 0.0f);
        }
    }

private:
    int synthesis_hop_ = 0;
    double read_position_ = 0.0;
    bool has_prev_frame_ = false;
    std::vector<float> fade_in_;
    std::vector<float> fade_out_;
    std::vector<std::vector<float>> prev_frame_;
    std::vector<std::vector<float>> hop_frame_;
    std::vector<float*> hop_ptrs_;

    // Produce one synthesis hop using overlap-add.
    // Advance read position by synthesis_hop_ * ratio.
    bool produce_one_hop() {
        int read_pos = static_cast<int>(std::floor(read_position_));

        // Need frame_size_ samples ahead of read position in input buffer
        if (read_pos + frame_size_ > input_ring_buffer_.buffered()) {
            return false;
        }

        // Check output buffer has space
        if (output_ring_buffer_.writable() < synthesis_hop_) {
            return false;
        }

        // Read frame from input at current position (non-consuming)
        input_ring_buffer_.peek(temp_ptrs_.data(), 0, read_pos, frame_size_);

        if (!has_prev_frame_) {
            for (int ch = 0; ch < channel_count_; ++ch) {
                std::copy(temp_frame_[ch].begin(), temp_frame_[ch].end(),
                          prev_frame_[ch].begin());
                std::copy(prev_frame_[ch].begin(),
                          prev_frame_[ch].begin() + synthesis_hop_,
                          hop_frame_[ch].begin());
            }
            has_prev_frame_ = true;
        } else {
            int prev_overlap_pos = frame_size_ - synthesis_hop_;
            for (int ch = 0; ch < channel_count_; ++ch) {
                for (int i = 0; i < synthesis_hop_; ++i) {
                    float prev_sample = prev_frame_[ch][prev_overlap_pos + i];
                    float cur_sample = temp_frame_[ch][i];
                    hop_frame_[ch][i] = prev_sample * fade_out_[i]
                                      + cur_sample * fade_in_[i];
                }
                std::copy(temp_frame_[ch].begin(), temp_frame_[ch].end(),
                          prev_frame_[ch].begin());
            }
        }

        // Write one hop to output ring buffer
        output_ring_buffer_.write(
            const_cast<const float* const*>(hop_ptrs_.data()), 0, synthesis_hop_);

        // Advance read position by synthesis_hop_ * ratio
        read_position_ += static_cast<double>(synthesis_hop_)
                        * static_cast<double>(time_stretch_ratio_);

        return true;
    }

    // Discard consumed input samples behind read position
    void compact() {
        int safe_discard = static_cast<int>(std::floor(read_position_)) - frame_size_;
        if (safe_discard <= 0) {
            return;
        }

        int available = input_ring_buffer_.buffered();
        int to_discard = std::min(safe_discard, available);

        input_ring_buffer_.discard(to_discard);
        read_position_ -= to_discard;
    }
};
