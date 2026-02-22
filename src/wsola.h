#pragma once
// ============================================================================
// wsola.h - Basic time-stretcher (Step 1: ratio only, no overlap-add)
//
// Inherits I/O ring buffer management from TimeStretcher.
// Implements produce_frames(): copy frames at ratio-adjusted positions.
// ============================================================================

#include "time_stretcher.h"

class WSOLA : public TimeStretcher {
protected:
    void produce_frames(int needed_output) override {
        while (output_ring_buffer_.buffered() < needed_output) {
            if (!produce_one_frame()) {
                break;
            }
        }
        compact();
    }

    void onReset() override {
        read_position_ = 0.0;
    }

private:
    double read_position_ = 0.0;

    // Copy one frame from input ring buffer to output ring buffer.
    // Advance read position by frame_size * ratio.
    bool produce_one_frame() {
        int read_pos = static_cast<int>(read_position_);

        // Need frame_size_ samples ahead of read position in input buffer
        if (read_pos + frame_size_ > input_ring_buffer_.buffered()) {
            return false;
        }

        // Check output buffer has space
        if (output_ring_buffer_.writable() < frame_size_) {
            return false;
        }

        // Read frame from input at current position (non-consuming)
        input_ring_buffer_.peek(temp_ptrs_.data(), 0, read_pos, frame_size_);

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

        input_ring_buffer_.discard(to_discard);
        read_position_ -= to_discard;
    }
};
