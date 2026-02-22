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

        int capacity = frame_size_ * 10;
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

    void setPhaseControl(float control) {
        phase_control_ = std::clamp(control, 0.0f, 1.0f);
    }

    float getPhaseControl() const { return phase_control_; }

    int getNumNeededSamples(int output_sample_count) const {
        return static_cast<int>(
            std::ceil(output_sample_count * static_cast<double>(time_stretch_ratio_)));
    }

    int process(const float* const* input, int input_sample_count, bool input_ended,
                float** output, int output_sample_count) {
        const bool passthrough = (time_stretch_ratio_ == 1.0f);
        if (passthrough) {
            // Keep passthrough bit-perfect and avoid stale buffered audio when
            // ratio toggles between 1.0 and stretched modes.
            if (!passthrough_active_) {
                reset();
                passthrough_active_ = true;
            }

            const int to_copy = std::min(input_sample_count, output_sample_count);
            for (int ch = 0; ch < channel_count_; ++ch) {
                if (to_copy > 0) {
                    std::memcpy(output[ch], input[ch], to_copy * sizeof(float));
                }
                if (to_copy < output_sample_count) {
                    std::memset(output[ch] + to_copy, 0,
                                (output_sample_count - to_copy) * sizeof(float));
                }
            }

            input_ended_ = input_ended;
            return to_copy;
        }

        if (passthrough_active_) {
            reset();
            passthrough_active_ = false;
        }

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

    // Build a full Hann window: w[n] = 0.5 - 0.5 cos(2*pi*n/(N-1))
    static void generate_hann_window(int size, std::vector<float>& window) {
        window.assign(size, 0.0f);
        if (size <= 0) return;
        if (size == 1) {
            window[0] = 1.0f;
            return;
        }

        const double pi = 3.14159265358979323846;
        const double denom = static_cast<double>(size - 1);
        for (int i = 0; i < size; ++i) {
            const double phase = (2.0 * pi * static_cast<double>(i)) / denom;
            window[i] = static_cast<float>(0.5 - 0.5 * std::cos(phase));
        }
    }

    // Build crossfade ramps from a half-Hann shape.
    // fade_in[n] = 0.5 - 0.5 cos(pi*n/(N-1))
    // fade_out[n] = 1 - fade_in[n]
    static void generate_hann_crossfade(int size, std::vector<float>& fade_in,
                                        std::vector<float>& fade_out) {
        fade_in.assign(size, 0.0f);
        fade_out.assign(size, 0.0f);
        if (size <= 0) return;
        if (size == 1) {
            fade_in[0] = 1.0f;
            fade_out[0] = 0.0f;
            return;
        }

        const double pi = 3.14159265358979323846;
        const double denom = static_cast<double>(size - 1);
        for (int i = 0; i < size; ++i) {
            const double x = static_cast<double>(i) / denom;
            const float in = static_cast<float>(0.5 - 0.5 * std::cos(pi * x));
            fade_in[i] = in;
            fade_out[i] = 1.0f - in;
        }
    }

    int sampling_rate_ = 44100;
    int channel_count_ = 2;
    int frame_size_ = 0;
    float time_stretch_ratio_ = 1.0f;
    float phase_control_ = 0.0f;
    bool input_ended_ = false;
    bool passthrough_active_ = false;

    MultiChannelRingBuffer input_ring_buffer_;
    MultiChannelRingBuffer output_ring_buffer_;

    std::vector<std::vector<float>> temp_frame_;
    std::vector<float*> temp_ptrs_;
};
