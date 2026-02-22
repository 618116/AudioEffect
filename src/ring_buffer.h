#pragma once

#include <cassert>
#include <vector>

class MultiChannelRingBuffer {
public:
    void init(int channel_count, int capacity) {
        assert(channel_count > 0);
        assert(capacity > 1);

        channel_count_ = channel_count;
        capacity_ = capacity;
        read_ = 0;
        write_ = 0;

        data_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            data_[ch].assign(capacity_, 0.0f);
        }
    }

    void reset() {
        read_ = 0;
        write_ = 0;
        for (int ch = 0; ch < channel_count_; ++ch) {
            std::fill(data_[ch].begin(), data_[ch].end(), 0.0f);
        }
    }

    // Number of samples available to read.
    int buffered() const {
        int count = write_ - read_;
        if (count < 0) count += capacity_;
        return count;
    }

    // Number of samples that can be written before full.
    int writable() const {
        return capacity_ - 1 - buffered();
    }

    // Write samples into the buffer.
    void write(const float* const* src, int src_offset, int count) {
        for (int ch = 0; ch < channel_count_; ++ch) {
            int pos = write_;
            for (int i = 0; i < count; ++i) {
                data_[ch][pos] = src[ch][src_offset + i];
                if (++pos >= capacity_) pos = 0;
            }
        }
        write_ = (write_ + count) % capacity_;
    }

    // Read from an arbitrary position (relative to read pointer) without consuming.
    void peek(float** output, int output_offset, int position, int count) const {
        for (int ch = 0; ch < channel_count_; ++ch) {
            int pos = (read_ + position) % capacity_;
            for (int i = 0; i < count; ++i) {
                output[ch][output_offset + i] = data_[ch][pos];
                if (++pos >= capacity_) pos = 0;
            }
        }
    }

    // Read and consume samples.
    void read(float** output, int output_offset, int count) {
        peek(output, output_offset, 0, count);
        read_ = (read_ + count) % capacity_;
    }

    // Advance read pointer without copying (discard consumed samples).
    void discard(int count) {
        read_ = (read_ + count) % capacity_;
    }

private:
    std::vector<std::vector<float>> data_;
    int channel_count_ = 0;
    int capacity_ = 0;
    int read_ = 0;
    int write_ = 0;
};
