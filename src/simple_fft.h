#pragma once
// ============================================================================
// simple_fft.h - Lightweight radix-2 FFT/IFFT helper
//
// - Complex FFT/IFFT (in-place): O(N log N), power-of-two size only
// - Real signal helpers: forward_real() / inverse_real()
// ============================================================================

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <vector>

class SimpleFFT {
public:
    bool init(int fft_size) {
        if (!is_power_of_two(fft_size)) {
            fft_size_ = 0;
            log2_size_ = 0;
            bit_reversal_.clear();
            twiddle_.clear();
            scratch_.clear();
            return false;
        }

        fft_size_ = fft_size;
        log2_size_ = 0;
        for (int n = fft_size_; n > 1; n >>= 1) {
            ++log2_size_;
        }

        bit_reversal_.resize(fft_size_);
        for (int i = 0; i < fft_size_; ++i) {
            bit_reversal_[i] = reverse_bits(i, log2_size_);
        }

        twiddle_.resize(fft_size_ / 2);
        const double pi = 3.14159265358979323846;
        for (int k = 0; k < fft_size_ / 2; ++k) {
            const double angle = -2.0 * pi * static_cast<double>(k)
                               / static_cast<double>(fft_size_);
            twiddle_[k] = std::complex<float>(
                static_cast<float>(std::cos(angle)),
                static_cast<float>(std::sin(angle)));
        }

        scratch_.assign(fft_size_, std::complex<float>(0.0f, 0.0f));
        return true;
    }

    int size() const { return fft_size_; }
    int bin_count() const { return (fft_size_ / 2) + 1; }
    bool ready() const { return fft_size_ > 0; }

    void fft(std::vector<std::complex<float>>& data) const {
        assert(ready());
        assert(static_cast<int>(data.size()) == fft_size_);
        transform(data, false);
    }

    void ifft(std::vector<std::complex<float>>& data) const {
        assert(ready());
        assert(static_cast<int>(data.size()) == fft_size_);
        transform(data, true);
    }

    // half_spectrum must have bin_count() elements.
    void forward_real(const float* time_input, std::complex<float>* half_spectrum) {
        assert(ready());
        assert(time_input != nullptr);
        assert(half_spectrum != nullptr);

        for (int i = 0; i < fft_size_; ++i) {
            scratch_[i] = std::complex<float>(time_input[i], 0.0f);
        }
        transform(scratch_, false);

        for (int k = 0; k < bin_count(); ++k) {
            half_spectrum[k] = scratch_[k];
        }
    }

    // half_spectrum must have bin_count() elements.
    // time_output must have size() elements.
    void inverse_real(const std::complex<float>* half_spectrum, float* time_output) {
        assert(ready());
        assert(half_spectrum != nullptr);
        assert(time_output != nullptr);

        scratch_[0] = half_spectrum[0];
        for (int k = 1; k < fft_size_ / 2; ++k) {
            scratch_[k] = half_spectrum[k];
            scratch_[fft_size_ - k] = std::conj(half_spectrum[k]);
        }
        scratch_[fft_size_ / 2] = half_spectrum[fft_size_ / 2];

        transform(scratch_, true);
        for (int i = 0; i < fft_size_; ++i) {
            time_output[i] = scratch_[i].real();
        }
    }

private:
    static bool is_power_of_two(int value) {
        return value >= 2 && (value & (value - 1)) == 0;
    }

    static int reverse_bits(int value, int bit_count) {
        int result = 0;
        for (int i = 0; i < bit_count; ++i) {
            result = (result << 1) | (value & 1);
            value >>= 1;
        }
        return result;
    }

    void transform(std::vector<std::complex<float>>& data, bool inverse) const {
        for (int i = 0; i < fft_size_; ++i) {
            const int j = bit_reversal_[i];
            if (i < j) {
                std::swap(data[i], data[j]);
            }
        }

        for (int len = 2; len <= fft_size_; len <<= 1) {
            const int half_len = len >> 1;
            const int twiddle_step = fft_size_ / len;

            for (int start = 0; start < fft_size_; start += len) {
                for (int i = 0; i < half_len; ++i) {
                    const std::complex<float> tw =
                        inverse ? std::conj(twiddle_[i * twiddle_step])
                                : twiddle_[i * twiddle_step];

                    const std::complex<float> u = data[start + i];
                    const std::complex<float> v = data[start + i + half_len] * tw;

                    data[start + i] = u + v;
                    data[start + i + half_len] = u - v;
                }
            }
        }

        if (inverse) {
            const float scale = 1.0f / static_cast<float>(fft_size_);
            for (int i = 0; i < fft_size_; ++i) {
                data[i] *= scale;
            }
        }
    }

    int fft_size_ = 0;
    int log2_size_ = 0;
    std::vector<int> bit_reversal_;
    std::vector<std::complex<float>> twiddle_;
    std::vector<std::complex<float>> scratch_;
};

