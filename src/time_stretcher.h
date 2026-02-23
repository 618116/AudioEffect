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

class TimeStretcher {
public:
    virtual ~TimeStretcher() = default;

    void init(int sample_rate, int channel_count) {
        sampling_rate_ = sample_rate;
        channel_count_ = channel_count;

        onInit();

        int capacity = getRequiredBufferCapacity();
        input_ring_buffer_.init(channel_count_, capacity);
        output_ring_buffer_.init(channel_count_, capacity);

        reset();
    }

    void reset() {
        input_ring_buffer_.reset();
        output_ring_buffer_.reset();
        input_ended_ = false;
        onReset();
    }

    void setPlaybackRate(float rate) {
        float clamped = std::clamp(rate, 0.5f, 2.0f);
        playback_rate_ = std::round(clamped * 10.0f) / 10.0f;
    }

    float getPlaybackRate() const { return playback_rate_; }

    void setPhaseControl(float control) {
        phase_control_ = std::clamp(control, 0.0f, 1.0f);
    }

    float getPhaseControl() const { return phase_control_; }

    int getNumNeededSamples(int output_sample_count) const {
        return static_cast<int>(
            std::ceil(output_sample_count * static_cast<double>(playback_rate_)));
    }

    int process(const float* const* input, int input_sample_count, bool input_ended,
                float** output, int output_sample_count) {
        const bool passthrough = (playback_rate_ == 1.0f);
        if (passthrough) {
            // Keep passthrough bit-perfect and avoid stale buffered audio when
            // playback rate toggles between 1.0 and stretched modes.
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

    // Subclass returns the required ring buffer capacity.
    virtual int getRequiredBufferCapacity() const = 0;

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

    int sampling_rate_ = 44100;
    int channel_count_ = 2;
    float playback_rate_ = 1.0f;
    float phase_control_ = 0.0f;
    bool input_ended_ = false;
    bool passthrough_active_ = false;

    MultiChannelRingBuffer input_ring_buffer_;
    MultiChannelRingBuffer output_ring_buffer_;
};
