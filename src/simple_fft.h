#pragma once
// ============================================================================
// simple_fft.h - Lightweight radix-2 FFT/IFFT helper
//
// - Complex FFT/IFFT (in-place): O(N log N), power-of-two size only
// - Real signal helpers: forward_real() / inverse_real()
//   (use N/2 complex FFT internally for ~2x speedup)
// ============================================================================

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <complex>
#include <vector>

class SimpleFFT {
public:
    bool init(int fft_size) {
        if (!is_power_of_two(fft_size)) {
            fft_size_ = 0;
            log2_size_ = 0;
            bit_reversal_.clear();
            twiddle_fwd_.clear();
            twiddle_inv_.clear();
            stage_twiddle_offset_.clear();
            scratch_.clear();
            half_scratch_.clear();
            real_twiddle_.clear();
            return false;
        }

        fft_size_ = fft_size;
        log2_size_ = 0;
        for (int n = fft_size_; n > 1; n >>= 1) {
            ++log2_size_;
        }

        // Bit-reversal table
        bit_reversal_.resize(fft_size_);
        for (int i = 0; i < fft_size_; ++i) {
            bit_reversal_[i] = reverse_bits(i, log2_size_);
        }

        // Per-stage contiguous twiddle layout for cache-friendly access.
        // Stage s (len = 2^(s+1)) needs half_len = 2^s twiddle factors.
        // Total entries = 1 + 2 + 4 + ... + N/2 = N - 1.
        const double PI = 3.14159265358979323846;
        int total_twiddles = fft_size_ - 1;
        twiddle_fwd_.resize(total_twiddles);
        twiddle_inv_.resize(total_twiddles);
        stage_twiddle_offset_.resize(log2_size_);

        int offset = 0;
        for (int s = 0; s < log2_size_; ++s) {
            int half_len = 1 << s;
            stage_twiddle_offset_[s] = offset;
            for (int i = 0; i < half_len; ++i) {
                double angle = -2.0 * PI * static_cast<double>(i)
                             / static_cast<double>(half_len * 2);
                auto fwd = std::complex<float>(
                    static_cast<float>(std::cos(angle)),
                    static_cast<float>(std::sin(angle)));
                twiddle_fwd_[offset + i] = fwd;
                twiddle_inv_[offset + i] = std::conj(fwd);
            }
            offset += half_len;
        }

        scratch_.resize(fft_size_);

        // Prepare half-size tables for real-FFT trick (N/2 complex FFT)
        const int half_n = fft_size_ / 2;
        if (half_n >= 2) {
            half_log2_ = log2_size_ - 1;
            half_bit_reversal_.resize(half_n);
            for (int i = 0; i < half_n; ++i) {
                half_bit_reversal_[i] = reverse_bits(i, half_log2_);
            }

            int half_total = half_n - 1;
            half_twiddle_fwd_.resize(half_total);
            half_twiddle_inv_.resize(half_total);
            half_stage_offset_.resize(half_log2_);

            offset = 0;
            for (int s = 0; s < half_log2_; ++s) {
                int half_len = 1 << s;
                half_stage_offset_[s] = offset;
                for (int i = 0; i < half_len; ++i) {
                    double angle = -2.0 * PI * static_cast<double>(i)
                                 / static_cast<double>(half_len * 2);
                    auto fwd = std::complex<float>(
                        static_cast<float>(std::cos(angle)),
                        static_cast<float>(std::sin(angle)));
                    half_twiddle_fwd_[offset + i] = fwd;
                    half_twiddle_inv_[offset + i] = std::conj(fwd);
                }
                offset += half_len;
            }

            half_scratch_.resize(half_n);

            // Post-processing twiddle for real-FFT unpack/repack
            real_twiddle_.resize(half_n / 2);
            for (int k = 0; k < half_n / 2; ++k) {
                double angle = -2.0 * PI * static_cast<double>(k + 1)
                             / static_cast<double>(fft_size_);
                real_twiddle_[k] = std::complex<float>(
                    static_cast<float>(std::cos(angle)),
                    static_cast<float>(std::sin(angle)));
            }
        }

        return true;
    }

    int size() const { return fft_size_; }
    int bin_count() const { return (fft_size_ / 2) + 1; }
    bool ready() const { return fft_size_ > 0; }

    void fft(std::vector<std::complex<float>>& data) const {
        assert(ready());
        assert(static_cast<int>(data.size()) == fft_size_);
        transform_core(data.data(), fft_size_, log2_size_,
                        bit_reversal_.data(),
                        twiddle_fwd_.data(),
                        stage_twiddle_offset_.data());
    }

    void ifft(std::vector<std::complex<float>>& data) const {
        assert(ready());
        assert(static_cast<int>(data.size()) == fft_size_);
        transform_core(data.data(), fft_size_, log2_size_,
                        bit_reversal_.data(),
                        twiddle_inv_.data(),
                        stage_twiddle_offset_.data());

        const float scale = 1.0f / static_cast<float>(fft_size_);
        std::complex<float>* p = data.data();
        for (int i = 0; i < fft_size_; ++i) {
            p[i] *= scale;
        }
    }

    // half_spectrum must have bin_count() elements.
    void forward_real(const float* time_input, std::complex<float>* half_spectrum) {
        assert(ready());
        assert(time_input != nullptr);
        assert(half_spectrum != nullptr);

        const int half_n = fft_size_ / 2;

        // Pack even/odd samples into a half-size complex array:
        //   z[k] = x[2k] + j * x[2k+1]
        std::complex<float>* z = half_scratch_.data();
        for (int k = 0; k < half_n; ++k) {
            z[k] = std::complex<float>(time_input[2 * k], time_input[2 * k + 1]);
        }

        // N/2 complex FFT
        transform_core(z, half_n, half_log2_,
                        half_bit_reversal_.data(),
                        half_twiddle_fwd_.data(),
                        half_stage_offset_.data());

        // Unpack into N/2+1 real-FFT bins
        // X[0] is trivial
        half_spectrum[0] = std::complex<float>(
            z[0].real() + z[0].imag(), 0.0f);
        half_spectrum[half_n] = std::complex<float>(
            z[0].real() - z[0].imag(), 0.0f);

        const std::complex<float> J(0.0f, 1.0f);
        const std::complex<float>* tw = real_twiddle_.data();
        for (int k = 1; k <= half_n / 2; ++k) {
            const std::complex<float> zk = z[k];
            const std::complex<float> zn = std::conj(z[half_n - k]);

            // E = (Z[k] + Z*[N/2-k]) / 2,  O = (Z[k] - Z*[N/2-k]) / (2j)
            const std::complex<float> e = (zk + zn) * 0.5f;
            const std::complex<float> o = (zk - zn) * 0.5f * std::conj(J);
            const std::complex<float> w = tw[k - 1];

            half_spectrum[k] = e + w * o;
            half_spectrum[half_n - k] = std::conj(e - w * o);
        }
    }

    // half_spectrum must have bin_count() elements.
    // time_output must have size() elements.
    void inverse_real(const std::complex<float>* half_spectrum, float* time_output) {
        assert(ready());
        assert(half_spectrum != nullptr);
        assert(time_output != nullptr);

        const int half_n = fft_size_ / 2;

        // Repack half_spectrum into half-size complex array for inverse
        std::complex<float>* z = half_scratch_.data();

        z[0] = std::complex<float>(
            (half_spectrum[0].real() + half_spectrum[half_n].real()) * 0.5f,
            (half_spectrum[0].real() - half_spectrum[half_n].real()) * 0.5f);

        const std::complex<float> J(0.0f, 1.0f);
        const std::complex<float>* tw = real_twiddle_.data();
        for (int k = 1; k <= half_n / 2; ++k) {
            const std::complex<float> xk = half_spectrum[k];
            const std::complex<float> xn = std::conj(half_spectrum[half_n - k]);

            const std::complex<float> e = (xk + xn) * 0.5f;
            const std::complex<float> o = (xk - xn) * 0.5f;
            // Undo forward twiddle: multiply by conj(w) then by j
            const std::complex<float> w = std::conj(tw[k - 1]);

            z[k] = e + J * (w * o);
            z[half_n - k] = std::conj(e - J * (w * o));
        }

        // N/2 inverse complex FFT
        transform_core(z, half_n, half_log2_,
                        half_bit_reversal_.data(),
                        half_twiddle_inv_.data(),
                        half_stage_offset_.data());

        const float scale = 1.0f / static_cast<float>(half_n);
        for (int k = 0; k < half_n; ++k) {
            time_output[2 * k]     = z[k].real() * scale;
            time_output[2 * k + 1] = z[k].imag() * scale;
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

    // Branchless in-place FFT core operating on raw pointers.
    // The caller selects forward/inverse by passing the appropriate
    // twiddle table (twiddle_fwd_ or twiddle_inv_). No branch in the loop.
    static void transform_core(
        std::complex<float>* data,
        int n,
        int log2n,
        const int* bit_rev,
        const std::complex<float>* stage_twiddles,
        const int* stage_offsets)
    {
        // Bit-reversal permutation
        for (int i = 0; i < n; ++i) {
            const int j = bit_rev[i];
            if (i < j) {
                std::swap(data[i], data[j]);
            }
        }

        // Butterfly stages
        for (int s = 0; s < log2n; ++s) {
            const int half_len = 1 << s;
            const int len = half_len << 1;
            const std::complex<float>* tw = stage_twiddles + stage_offsets[s];

            for (int start = 0; start < n; start += len) {
                std::complex<float>* p = data + start;
                std::complex<float>* q = p + half_len;

                for (int i = 0; i < half_len; ++i) {
                    const std::complex<float> u = p[i];
                    const std::complex<float> v = q[i] * tw[i];
                    p[i] = u + v;
                    q[i] = u - v;
                }
            }
        }
    }

    int fft_size_ = 0;
    int log2_size_ = 0;

    // Full-size tables
    std::vector<int> bit_reversal_;
    std::vector<std::complex<float>> twiddle_fwd_;
    std::vector<std::complex<float>> twiddle_inv_;
    std::vector<int> stage_twiddle_offset_;
    std::vector<std::complex<float>> scratch_;

    // Half-size tables for real-FFT trick
    int half_log2_ = 0;
    std::vector<int> half_bit_reversal_;
    std::vector<std::complex<float>> half_twiddle_fwd_;
    std::vector<std::complex<float>> half_twiddle_inv_;
    std::vector<int> half_stage_offset_;
    std::vector<std::complex<float>> half_scratch_;
    std::vector<std::complex<float>> real_twiddle_;
};
