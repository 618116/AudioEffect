#pragma once
// ============================================================================
// wsola.h - Minimal real-time WSOLA time-stretcher
//
// API:
//   init(sample_rate, channel_count)          - allocate, configure
//   reset()                             - clear state (on seek/discontinuity)
//   setRatio(ratio)                     - 0.5-2.0, 1.0 = no stretch
//   getNumNeededSamples(output_sample_count)  - how many input samples to provide
//   process(in, inLen, out, outLen)     - deinterleaved float**
//
// ============================================================================

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <vector>

#include "ring_buffer.h"

struct WSOLAConfig {
    int overlapLength = 128;  // Crossfade length (samples). Determines lowest
                              // frequency handled cleanly: ~sample_rate/overlapLength Hz.
                              // 128 @ 48kHz ~= 375Hz+. Use 256 for 187Hz+, 512 for 94Hz+.

    int seekWindow = 128;     // Cross-correlation search range (+/- samples).
                              // Larger = better match, more CPU. 128 is good up to 1.5x.

    int sequenceLength = 256; // Samples copied between splice points.
                              // Must be >= overlapLength. 256-512 is a good range.
};

class WSOLA {
public:
    using Config = WSOLAConfig;

    void init(int sample_rate, int channel_count, Config configuration = Config()) {
        sampling_rate_ = sample_rate;
        channel_count_ = channel_count;
        configuration_ = configuration;

        assert(configuration_.sequenceLength >= configuration_.overlapLength);

        // History buffer per channel: enough to hold input for correlation search.
        history_capacity_ = (configuration_.sequenceLength
                          + configuration_.seekWindow * 2
                          + configuration_.overlapLength) * 4;
        history_.resize(channel_count_);
        overlap_buffer_.resize(channel_count_);
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            history_[channel_index].resize(history_capacity_, 0.0f);
            overlap_buffer_[channel_index].resize(configuration_.overlapLength, 0.0f);
        }

        // Output ring buffer: hold several full sequences worth of output.
        output_ring_buffer_.init(channel_count_, configuration_.sequenceLength * 4);

        reset();
    }

    void reset() {
        history_length_ = 0;
        read_position_ = 0.0;
        has_overlap_tail_ = false;
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            std::fill(history_[channel_index].begin(), history_[channel_index].end(), 0.0f);
            std::fill(overlap_buffer_[channel_index].begin(), overlap_buffer_[channel_index].end(), 0.0f);
        }
        output_ring_buffer_.reset();
    }

    void setRatio(float ratio) {
        float clamped_ratio = std::clamp(ratio, 0.5f, 2.0f);
        time_stretch_ratio_ = std::round(clamped_ratio * 10.0f) / 10.0f;
    }

    float getRatio() const { return time_stretch_ratio_; }

    // How many input samples you should provide for the given output length.
    int getNumNeededSamples(int output_sample_count) const {
        if (std::abs(time_stretch_ratio_ - 1.0f) < 0.005f) {
            return output_sample_count;
        }

        // Account for samples already buffered in the output ring buffer.
        int buffered_output = output_ring_buffer_.buffered();
        int still_need_output = std::max(0, output_sample_count - buffered_output);
        if (still_need_output == 0) {
            return 0;
        }

        // We consume input at ratio speed. Add margin for seek + sequence lookahead.
        int total_ahead = static_cast<int>(std::ceil(static_cast<double>(still_need_output)
                                                   * static_cast<double>(time_stretch_ratio_)))
                       + configuration_.sequenceLength
                       + configuration_.seekWindow;
        int available_input_samples = history_length_ - static_cast<int>(read_position_);
        return std::max(0, total_ahead - available_input_samples);
    }

    // Process: read from input, write to output.
    // input[ch][0..input_sample_count-1], output[ch][0..output_sample_count-1]
    void process(const float* const* input, int input_sample_count,
                 float** output, int output_sample_count) {
        float stretch_ratio = time_stretch_ratio_;

        // --- Fast path: passthrough ---
        if (std::abs(stretch_ratio - 1.0f) < 0.005f && !has_overlap_tail_ && output_ring_buffer_.buffered() == 0) {
            int passthrough_sample_count = std::min(input_sample_count, output_sample_count);
            for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                std::memcpy(output[channel_index], input[channel_index], passthrough_sample_count * sizeof(float));
                if (output_sample_count > passthrough_sample_count) {
                    std::memset(output[channel_index] + passthrough_sample_count,
                                0,
                                (output_sample_count - passthrough_sample_count) * sizeof(float));
                }
            }
            return;
        }

        // --- Push input into history ---
        push_history(input, input_sample_count);

        // --- Produce full WSOLA sequences into the output ring buffer ---
        while (output_ring_buffer_.buffered() < output_sample_count) {
            if (!produce_one_sequence(stretch_ratio)) {
                break; // not enough history data
            }
        }

        // --- Drain output ring buffer to caller ---
        int available_output = output_ring_buffer_.buffered();
        int to_drain = std::min(available_output, output_sample_count);
        output_ring_buffer_.drain(output, 0, to_drain);

        // Zero-fill any remainder if we ran out of data
        if (to_drain < output_sample_count) {
            for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                std::memset(output[channel_index] + to_drain,
                            0,
                            (output_sample_count - to_drain) * sizeof(float));
            }
        }

        // --- Compact history: discard consumed samples ---
        compact();
    }

    int latencySamples() const { return configuration_.overlapLength + configuration_.seekWindow; }
    float latencyMs() const { return 1000.0f * latencySamples() / sampling_rate_; }

private:
    int sampling_rate_ = 44100;
    int channel_count_ = 2;
    float time_stretch_ratio_ = 1.0f;
    Config configuration_;

    // Input history
    std::vector<std::vector<float>> history_;        // [ch][sample]
    std::vector<std::vector<float>> overlap_buffer_;  // [ch][overlapLength] crossfade tail
    int history_capacity_ = 0;
    int history_length_ = 0;
    double read_position_ = 0.0;
    bool has_overlap_tail_ = false;

    // Output ring buffer: decouples WSOLA sequence size from caller's block size.
    MultiChannelRingBuffer output_ring_buffer_;

    // Produce one full WSOLA sequence into the output ring buffer.
    // Returns false if not enough history data.
    bool produce_one_sequence(float stretch_ratio) {
        int nominal_position = static_cast<int>(read_position_);

        // Check we have enough history ahead
        int required_history_samples = nominal_position + configuration_.sequenceLength + configuration_.seekWindow;
        if (required_history_samples > history_length_) {
            return false;
        }

        // Check we have room in the output buffer
        int sequence_output_length = has_overlap_tail_
            ? configuration_.sequenceLength  // crossfade(overlapLen) + body(seqLen - overlapLen)
            : configuration_.sequenceLength;
        if (output_ring_buffer_.free() < sequence_output_length) {
            return false;
        }

        // Find best splice position
        int best_overlap_offset = has_overlap_tail_ ? find_best_overlap(nominal_position) : 0;
        int splice_position = nominal_position + best_overlap_offset;

        if (has_overlap_tail_) {
            // Crossfade: blend overlap_buffer_ with history at splice_position
            for (int i = 0; i < configuration_.overlapLength; ++i) {
                float blend_factor = static_cast<float>(i)
                                  / static_cast<float>(configuration_.overlapLength - 1);
                for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                    float sample = overlap_buffer_[channel_index][i] * (1.0f - blend_factor)
                                 + history_[channel_index][splice_position + i] * blend_factor;
                    output_ring_buffer_.writeSample(channel_index, sample);
                }
                output_ring_buffer_.advanceWrite();
            }

            // Body (non-crossfaded portion)
            int body_length = configuration_.sequenceLength - configuration_.overlapLength;
            int body_start = splice_position + configuration_.overlapLength;
            for (int i = 0; i < body_length; ++i) {
                for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                    output_ring_buffer_.writeSample(channel_index, history_[channel_index][body_start + i]);
                }
                output_ring_buffer_.advanceWrite();
            }
        } else {
            // First sequence - no crossfade
            for (int i = 0; i < configuration_.sequenceLength; ++i) {
                for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                    output_ring_buffer_.writeSample(channel_index, history_[channel_index][splice_position + i]);
                }
                output_ring_buffer_.advanceWrite();
            }
        }

        // Save tail of this sequence for next crossfade
        save_overlap(splice_position + configuration_.sequenceLength - configuration_.overlapLength);

        // Advance read position: consume input at ratio rate
        read_position_ += static_cast<double>(configuration_.sequenceLength)
                      * static_cast<double>(stretch_ratio);

        return true;
    }

    void push_history(const float* const* input, int input_sample_count) {
        int available_space = history_capacity_ - history_length_;
        int samples_to_push = std::min(input_sample_count, available_space);
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            std::memcpy(
                history_[channel_index].data() + history_length_,
                input[channel_index],
                samples_to_push * sizeof(float));
        }
        history_length_ += samples_to_push;
    }

    // Normalized cross-correlation search.
    // Compare overlap_buffer_[] against history at positions around nominal_position.
    // Returns offset from nominal_position.
    int find_best_overlap(int nominal_position) {
        int search_start = std::max(0, nominal_position - configuration_.seekWindow);
        int search_end = std::min(
            history_length_ - configuration_.overlapLength,
            nominal_position + configuration_.seekWindow);
        if (search_start >= search_end) {
            return 0;
        }

        // Energy of overlap tail (constant across search)
        float overlap_energy = 0.0f;
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            for (int sample_index = 0; sample_index < configuration_.overlapLength; ++sample_index) {
                overlap_energy += overlap_buffer_[channel_index][sample_index]
                              * overlap_buffer_[channel_index][sample_index];
            }
        }

        // Initial candidate energy at position search_start
        float candidate_energy = 0.0f;
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            for (int sample_index = 0; sample_index < configuration_.overlapLength; ++sample_index) {
                candidate_energy += history_[channel_index][search_start + sample_index]
                                 * history_[channel_index][search_start + sample_index];
            }
        }

        int best_offset = 0;
        float best_correlation = -1e30f;

        for (int candidate_position = search_start; candidate_position < search_end; ++candidate_position) {
            float correlation = 0.0f;
            for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                for (int sample_index = 0; sample_index < configuration_.overlapLength; ++sample_index) {
                    correlation += overlap_buffer_[channel_index][sample_index]
                                * history_[channel_index][candidate_position + sample_index];
                }
            }

            float normalization = std::sqrt(overlap_energy * candidate_energy);
            if (normalization > 1e-8f) {
                correlation /= normalization;
            }

            if (correlation > best_correlation) {
                best_correlation = correlation;
                best_offset = candidate_position - nominal_position;
            }

            // Slide candidate_energy: remove leaving sample, add entering sample
            if (candidate_position + 1 < search_end) {
                for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
                    float leaving_sample = history_[channel_index][candidate_position];
                    float entering_sample = history_[channel_index][candidate_position + configuration_.overlapLength];
                    candidate_energy += entering_sample * entering_sample
                                    - leaving_sample * leaving_sample;
                }
                candidate_energy = std::max(0.0f, candidate_energy);
            }
        }

        return best_offset;
    }

    void save_overlap(int history_position) {
        int clamped_position = std::clamp(history_position, 0, history_length_ - configuration_.overlapLength);
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            std::memcpy(
                overlap_buffer_[channel_index].data(),
                history_[channel_index].data() + clamped_position,
                configuration_.overlapLength * sizeof(float));
        }
        has_overlap_tail_ = true;
    }

    // Discard history samples behind the read cursor
    void compact() {
        int samples_to_discard = static_cast<int>(read_position_)
                             - configuration_.seekWindow
                             - configuration_.overlapLength;
        if (samples_to_discard <= 0) {
            return;
        }
        samples_to_discard = std::min(samples_to_discard, history_length_);

        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            std::memmove(
                history_[channel_index].data(),
                history_[channel_index].data() + samples_to_discard,
                (history_length_ - samples_to_discard) * sizeof(float));
        }
        history_length_ -= samples_to_discard;
        read_position_ -= samples_to_discard;
    }
};
