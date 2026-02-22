#pragma once
// ============================================================================
// wsola.h - Basic time-stretcher (Step 1: ratio only, no overlap-add)
//
// API:
//   init(sample_rate, channel_count)          - allocate, configure
//   reset()                                   - clear state (on seek/discontinuity)
//   setRatio(ratio)                           - 0.5-2.0, 1.0 = no stretch
//   getNumNeededSamples(output_sample_count)  - how many input samples to provide
//   process(in, inLen, out, outLen)           - deinterleaved float**
//
// Frame size: 30ms (e.g. 1440 samples @ 48kHz)
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "ring_buffer.h"

#define FRAME_MS 30

class WSOLA {
public:
    void init(int sample_rate, int channel_count) {
        sampling_rate_ = sample_rate;
        channel_count_ = channel_count;
        frame_size_ = sampling_rate_ * FRAME_MS / 1000;

        int capacity = frame_size_ * 4;
        input_ring_buffer_.init(channel_count_, capacity);
        output_ring_buffer_.init(channel_count_, capacity);

        // Temporary buffer for reading frames from input ring buffer
        temp_frame_.resize(channel_count_);
        temp_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            temp_frame_[ch].resize(frame_size_, 0.0f);
            temp_ptrs_[ch] = temp_frame_[ch].data();
        }

        reset();
    }

    void reset() {
        read_position_ = 0.0;
        input_ring_buffer_.reset();
        output_ring_buffer_.reset();
    }

    void setRatio(float ratio) {
        float clamped_ratio = std::clamp(ratio, 0.5f, 2.0f);
        time_stretch_ratio_ = std::round(clamped_ratio * 10.0f) / 10.0f;
    }

    float getRatio() const { return time_stretch_ratio_; }

    int getNumNeededSamples(int output_sample_count) const {
        return static_cast<int>(
            std::ceil(output_sample_count * static_cast<double>(time_stretch_ratio_)));
    }

    void process(const float* const* input, int input_sample_count,
                 float** output, int output_sample_count) {
        // Push input into input ring buffer
        input_ring_buffer_.write(input, 0, input_sample_count);

        // Produce frames into output ring buffer
        while (output_ring_buffer_.buffered() < output_sample_count) {
            if (!produce_one_frame()) {
                break;
            }
        }

        // Drain output ring buffer to caller
        int available_output = output_ring_buffer_.buffered();
        int to_drain = std::min(available_output, output_sample_count);
        output_ring_buffer_.drain(output, 0, to_drain);

        // Zero-fill remainder
        if (to_drain < output_sample_count) {
            for (int ch = 0; ch < channel_count_; ++ch) {
                std::memset(output[ch] + to_drain, 0,
                            (output_sample_count - to_drain) * sizeof(float));
            }
        }

        // Discard consumed input samples
        compact();
    }

private:
    int sampling_rate_ = 44100;
    int channel_count_ = 2;
    int frame_size_ = 0;
    float time_stretch_ratio_ = 1.0f;
    double read_position_ = 0.0;

    MultiChannelRingBuffer input_ring_buffer_;
    MultiChannelRingBuffer output_ring_buffer_;

    std::vector<std::vector<float>> temp_frame_;
    std::vector<float*> temp_ptrs_;

    // Copy one frame from input ring buffer to output ring buffer.
    // Advance read position by frame_size * ratio.
    bool produce_one_frame() {
        int read_pos = static_cast<int>(read_position_);

        // Need frame_size_ samples ahead of read position in input buffer
        if (read_pos + frame_size_ > input_ring_buffer_.buffered()) {
            return false;
        }

        // Check output buffer has space
        if (output_ring_buffer_.free() < frame_size_) {
            return false;
        }

        // Read frame from input at current position (non-consuming)
        input_ring_buffer_.read(temp_ptrs_.data(), 0, read_pos, frame_size_);

        // Write frame to output ring buffer
        output_ring_buffer_.write(
            const_cast<const float* const*>(temp_ptrs_.data()), 0, frame_size_);

        // Advance read position by frame_size * ratio
        read_position_ += static_cast<double>(frame_size_)
                        * static_cast<double>(time_stretch_ratio_);

        return true;
    }

    // Discard consumed input samples behind read position
    void compact() {
        int safe_discard = static_cast<int>(read_position_) - frame_size_;
        if (safe_discard <= 0) {
            return;
        }

        int available = input_ring_buffer_.buffered();
        int to_discard = std::min(safe_discard, available);

        // Advance the read pointer of input ring buffer (discard samples)
        input_ring_buffer_.discard(to_discard);

        read_position_ -= to_discard;
    }
};
