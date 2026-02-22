#pragma once

#include <algorithm>
#include <cassert>
#include <cstring>
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
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            data_[channel_index].assign(capacity_, 0.0f);
        }
    }

    void reset() {
        read_ = 0;
        write_ = 0;
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            std::fill(data_[channel_index].begin(), data_[channel_index].end(), 0.0f);
        }
    }

    int buffered() const {
        int count = write_ - read_;
        if (count < 0) count += capacity_;
        return count;
    }

    int free() const {
        return capacity_ - 1 - buffered();
    }

    void write(const float* const* src, int src_offset, int count) {
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            int write_pos = write_;
            for (int i = 0; i < count; ++i) {
                data_[channel_index][write_pos] = src[channel_index][src_offset + i];
                write_pos++;
                if (write_pos >= capacity_) write_pos = 0;
            }
        }
        write_ = (write_ + count) % capacity_;
    }

    void writeSample(int channel_index, float value) {
        data_[channel_index][write_] = value;
    }

    void advanceWrite() {
        write_++;
        if (write_ >= capacity_) write_ = 0;
    }

    void drain(float** output, int output_offset, int count) {
        for (int channel_index = 0; channel_index < channel_count_; ++channel_index) {
            int read_pos = read_;
            for (int i = 0; i < count; ++i) {
                output[channel_index][output_offset + i] = data_[channel_index][read_pos];
                read_pos++;
                if (read_pos >= capacity_) read_pos = 0;
            }
        }
        read_ = (read_ + count) % capacity_;
    }

private:
    std::vector<std::vector<float>> data_;
    int channel_count_ = 0;
    int capacity_ = 0;
    int read_ = 0;
    int write_ = 0;
};
