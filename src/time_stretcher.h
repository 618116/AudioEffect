#pragma once
// ============================================================================
// time_stretcher.h - Base class for time-stretch algorithms
//
// Owns the input/output ring buffers and the process() shell.
// Subclasses implement produce_frames() with their specific algorithm.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "ring_buffer.h"

#define FRAME_MS 30

class TimeStretcher {
public:
    virtual ~TimeStretcher() = default;

    void init(int sample_rate, int channel_count) {
        sampling_rate_ = sample_rate;
        channel_count_ = channel_count;
        frame_size_ = sampling_rate_ * FRAME_MS / 1000;

        int capacity = frame_size_ * 4;
        input_ring_buffer_.init(channel_count_, capacity);
        output_ring_buffer_.init(channel_count_, capacity);

        temp_frame_.resize(channel_count_);
        temp_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            temp_frame_[ch].resize(frame_size_, 0.0f);
            temp_ptrs_[ch] = temp_frame_[ch].data();
        }

        onInit();
        reset();
    }

    void reset() {
        input_ring_buffer_.reset();
        output_ring_buffer_.reset();
        input_ended_ = false;
        onReset();
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

    int process(const float* const* input, int input_sample_count, bool input_ended,
                float** output, int output_sample_count) {
        // Push input into input ring buffer
        if (input_sample_count > 0) {
            input_ring_buffer_.write(input, 0, input_sample_count);
        }

        // Once end-of-input is signaled, keep it sticky until reset.
        input_ended_ = input_ended_ || input_ended;

        // Subclass fills output ring buffer
        produce_frames(output_sample_count);

        // Read output ring buffer to caller
        int available_output = output_ring_buffer_.buffered();
        int to_drain = std::min(available_output, output_sample_count);
        output_ring_buffer_.read(output, 0, to_drain);

        // Zero-fill remainder
        if (to_drain < output_sample_count) {
            for (int ch = 0; ch < channel_count_; ++ch) {
                std::memset(output[ch] + to_drain, 0,
                            (output_sample_count - to_drain) * sizeof(float));
            }
        }

        return to_drain;
    }

protected:
    // Subclass implements: fill output_ring_buffer_ until it has >= needed_output samples.
    virtual void produce_frames(int needed_output) = 0;

    // Optional hooks for subclass-specific init/reset.
    virtual void onInit() {}
    virtual void onReset() {}

    int sampling_rate_ = 44100;
    int channel_count_ = 2;
    int frame_size_ = 0;
    float time_stretch_ratio_ = 1.0f;
    bool input_ended_ = false;

    MultiChannelRingBuffer input_ring_buffer_;
    MultiChannelRingBuffer output_ring_buffer_;

    std::vector<std::vector<float>> temp_frame_;
    std::vector<float*> temp_ptrs_;
};
