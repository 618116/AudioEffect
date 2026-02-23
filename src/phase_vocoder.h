#pragma once
// ============================================================================
// phase_vocoder.h - Outline for a phase-vocoder time stretcher
//
// This file is intentionally an implementation outline.
// It wires state, data flow, and method boundaries so the DSP parts can be
// filled in incrementally.
// ============================================================================

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <vector>

#include "simple_fft.h"
#include "time_stretcher.h"

class PhaseVocoder : public TimeStretcher {
protected:
    int getRequiredBufferCapacity() const override {
        return kFixedFftSize_ * 3;
    }

    void onInit() override {
        fft_size_ = kFixedFftSize_;
        analysis_hop_ = std::max(1, fft_size_ / 4);  // Typical 75% overlap.
        bin_count_ = (fft_size_ / 2) + 1;
        fft_.init(fft_size_);

        generate_hann_window(fft_size_, window_);
        resize_state_buffers();
    }

    void onReset() override {
        analysis_read_pos_ = 0.0;
        has_phase_history_ = false;

        for (int ch = 0; ch < channel_count_; ++ch) {
            std::fill(hop_frame_[ch].begin(), hop_frame_[ch].end(), 0.0f);
            std::fill(time_frame_[ch].begin(), time_frame_[ch].end(), 0.0f);
            std::fill(synthesis_accum_[ch].begin(), synthesis_accum_[ch].end(), 0.0f);
            std::fill(synthesis_norm_[ch].begin(), synthesis_norm_[ch].end(), 0.0f);
            std::fill(prev_phase_[ch].begin(), prev_phase_[ch].end(), 0.0f);
            std::fill(sum_phase_[ch].begin(), sum_phase_[ch].end(), 0.0f);
            std::fill(legacy_phase_[ch].begin(), legacy_phase_[ch].end(), 0.0f);
            std::fill(magnitude_[ch].begin(), magnitude_[ch].end(), 0.0f);
            std::fill(original_phase_[ch].begin(), original_phase_[ch].end(), 0.0f);
            std::fill(horizontal_[ch].begin(), horizontal_[ch].end(),
                      std::complex<float>(0.0f, 0.0f));
            std::fill(consensus_complex_[ch].begin(), consensus_complex_[ch].end(),
                      std::complex<float>(0.0f, 0.0f));
            std::fill(gradient_unit_[ch].begin(), gradient_unit_[ch].end(),
                      std::complex<float>(1.0f, 0.0f));
        }
    }

    void produce_frames(int needed_output) override {
        // Keep producing hop-sized chunks until enough output is buffered.
        while (output_ring_buffer_.buffered() < needed_output) {
            if (!produce_one_hop()) {
                break;
            }
        }
        compact();
    }

private:
    static constexpr int kFixedFftSize_ = 4096;              /* FFTサイズ（デモでは固定） */
    static constexpr float kPi_ = 3.14159265358979323846f;   
    static constexpr float kTwoPi_ = 2.0f * kPi_;            
    static constexpr float kMinPlaybackRate_ = 0.5f;         /* 内部計算で許容する最小ストレッチ比（デモでは固定） */
    int fft_size_ = 0;                                        /* 現在使っているFFTサイズ */
    int analysis_hop_ = 0;                                    /* 解析側のホップサイズ */
    int max_syn_hop_ = 0;                                     /* 合成ホップの最大値（バッファ確保用） */
    int bin_count_ = 0;                                       /* 実数FFTの有効ビン数 */
    double analysis_read_pos_ = 0.0;                          /* 入力リングバッファの読み取り位置 */
    bool has_phase_history_ = false;                          /* 前回フレームの位相履歴があるか */
    SimpleFFT fft_;                                           /* FFT/IFFTを実行するヘルパ */
    static constexpr float kNormFloor_ = 1.0e-6f;            /* 正規化時のゼロ割り防止しきい値 */
    static constexpr float kDrainEpsilon_ = 1.0e-8f;         /* 入力終端時の無音判定しきい値 */
    static constexpr float kConsensusNormFloorSq_ = 1.0e-20f;/* consensus正規化の最小ノルム二乗 */

    std::vector<float> window_;                               /* ハン窓テーブル */
    std::vector<std::vector<float>> time_frame_;             /* チャンネルごとの時間領域フレーム */
    std::vector<std::vector<float>> synthesis_accum_;        /* OLA加算バッファ */
    std::vector<std::vector<float>> synthesis_norm_;         /* OLA正規化係数バッファ */
    std::vector<std::vector<float>> hop_frame_;              /* 1ホップ分の出力一時バッファ */
    std::vector<float*> hop_ptrs_;                           /* ring buffer書き込み用ポインタ配列 */

    std::vector<std::vector<std::complex<float>>> spectrum_;         /* 周波数領域スペクトル */
    std::vector<std::vector<float>> prev_phase_;                     /* 前回の解析位相 */
    std::vector<std::vector<float>> sum_phase_;                      /* 位相累積値（PV本体） */
    std::vector<std::vector<float>> legacy_phase_;                   /* 最終採用する位相 */
    std::vector<std::vector<float>> magnitude_;                      /* 各ビンの振幅 */
    std::vector<std::vector<float>> original_phase_;                 /* 入力スペクトルの元位相 */
    std::vector<std::vector<std::complex<float>>> horizontal_;       /* 水平方向伝播用スペクトル */
    std::vector<std::vector<std::complex<float>>> consensus_complex_;/* consensus適用後スペクトル */
    std::vector<std::vector<std::complex<float>>> gradient_unit_;    /* 隣接ビン位相差の単位ベクトル */

    std::vector<int> below_index_;                            /* 下側隣接ビンのindex */
    std::vector<int> above_index_;                            /* 上側隣接ビンのindex */
    std::vector<float> below_mask_;                           /* 下側隣接ビンが存在するかのマスク */
    std::vector<float> above_mask_;                           /* 上側隣接ビンが存在するかのマスク */

    /* 位相を [-pi, pi) の範囲に折り返す */
    static float wrap_phase(float phase) {
        phase = std::fmod(phase + kPi_, kTwoPi_);
        if (phase < 0.0f) phase += kTwoPi_;
        return phase - kPi_;
    }

    /* 現在の再生レートから合成ホップサイズを計算する */
    int synthesis_hop() const {
        const float rate = std::max(kMinPlaybackRate_, playback_rate_);
        int hop = static_cast<int>(std::lround(
            static_cast<double>(analysis_hop_) / static_cast<double>(rate)));
        return std::max(1, hop);
    }

    /* FFT設定とチャンネル数に合わせて内部バッファを再確保する */
    void resize_state_buffers() {
        max_syn_hop_ = std::max(
            1, static_cast<int>(std::lround(static_cast<double>(analysis_hop_)
                                            / static_cast<double>(kMinPlaybackRate_))));
        time_frame_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
        synthesis_accum_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
        synthesis_norm_.assign(channel_count_, std::vector<float>(fft_size_, 0.0f));
        hop_frame_.assign(channel_count_, std::vector<float>(max_syn_hop_, 0.0f));
        spectrum_.assign(
            channel_count_, std::vector<std::complex<float>>(bin_count_));
        prev_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        sum_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        legacy_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        magnitude_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        original_phase_.assign(channel_count_, std::vector<float>(bin_count_, 0.0f));
        horizontal_.assign(
            channel_count_, std::vector<std::complex<float>>(bin_count_));
        consensus_complex_.assign(
            channel_count_, std::vector<std::complex<float>>(bin_count_));
        gradient_unit_.assign(
            channel_count_, std::vector<std::complex<float>>(bin_count_));

        below_index_.assign(bin_count_, 0);
        above_index_.assign(bin_count_, 0);
        below_mask_.assign(bin_count_, 0.0f);
        above_mask_.assign(bin_count_, 0.0f);
        const int last = std::max(0, bin_count_ - 1);
        for (int k = 0; k < bin_count_; ++k) {
            const int below = std::max(0, k - 1);
            const int above = std::min(last, k + 1);
            const float has_below = (k > 0) ? 1.0f : 0.0f;
            const float has_above = (k < last) ? 1.0f : 0.0f;

            below_index_[k] = below;
            above_index_[k] = above;
            below_mask_[k] = has_below;
            above_mask_[k] = has_above;
        }

        hop_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            hop_ptrs_[ch] = hop_frame_[ch].data();
        }
    }

    /* 1ホップ分の解析・位相処理・合成を実行する */
    bool produce_one_hop() {
        const int syn_hop = synthesis_hop();
        if (syn_hop > max_syn_hop_) {
            assert(false && "syn_hop exceeds preallocated hop frame size");
            return false;
        }

        const int read_pos = static_cast<int>(std::floor(analysis_read_pos_));
        const int available = input_ring_buffer_.buffered();
        int valid_samples = fft_size_;
        bool frame_is_fully_padded = false;

        if (read_pos + fft_size_ > available) {
            if (!input_ended_) return false;
            valid_samples = std::max(0, available - read_pos);
            frame_is_fully_padded = (valid_samples == 0);
        }

        if (output_ring_buffer_.writable() < syn_hop) return false;

        // Step 1: analysis FFT for each channel.
        for (int ch = 0; ch < channel_count_; ++ch) {
            auto& frame = time_frame_[ch];
            std::fill(frame.begin(), frame.end(), 0.0f);
            if (valid_samples > 0) {
                input_ring_buffer_.peek(ch, frame.data(), 0, read_pos, valid_samples);
            }
            for (int i = 0; i < fft_size_; ++i) {
                frame[i] *= window_[i];
            }
            fft_.forward_real(frame.data(), spectrum_[ch].data());
        }

        const bool use_consensus =
            has_phase_history_ && channel_count_ > 0 && phase_control_ > 0.0f;
        const float consensus_weight = phase_control_;
        const float hop_ratio =
            static_cast<float>(syn_hop) / static_cast<float>(analysis_hop_);

        /* 単純な水平位相補正 */
        for (int ch = 0; ch < channel_count_; ++ch) {
            auto& prev = prev_phase_[ch];
            auto& sum = sum_phase_[ch];
            auto& legacy = legacy_phase_[ch];
            auto& mag_out = magnitude_[ch];
            auto& phase_orig = original_phase_[ch];
            for (int k = 0; k < bin_count_; ++k) {
                const float mag = std::abs(spectrum_[ch][k]);
                const float phase = std::arg(spectrum_[ch][k]);
                mag_out[k] = mag;
                phase_orig[k] = phase;

                if (!has_phase_history_) {
                    prev[k] = phase;
                    sum[k] = phase;
                    legacy[k] = phase;
                    continue;
                }

                const float expected_analysis =
                    kTwoPi_ * static_cast<float>(k) *
                    static_cast<float>(analysis_hop_) /
                    static_cast<float>(fft_size_);
                const float expected_synthesis =
                    kTwoPi_ * static_cast<float>(k) *
                    static_cast<float>(syn_hop) /
                    static_cast<float>(fft_size_);

                const float delta =
                    wrap_phase(phase - prev[k] - expected_analysis);
                sum[k] =
                    wrap_phase(sum[k] + expected_synthesis + delta * hop_ratio);
                prev[k] = phase;
                legacy[k] = sum[k];
            }
        }

        /* 垂直位相補正 */
        if (use_consensus) {
            for (int ch = 0; ch < channel_count_; ++ch) {
                auto& grad = gradient_unit_[ch];
                auto& horiz = horizontal_[ch];
                auto& cons = consensus_complex_[ch];
                auto& phase_orig = original_phase_[ch];
                auto& legacy = legacy_phase_[ch];
                auto& mag_out = magnitude_[ch];

                /* bin kとk-1の間の位相勾配（時間領域の群遅延）を計算する */
                grad[0] = std::complex<float>(1.0f, 0.0f);
                for (int k = 1; k < bin_count_; ++k) {
                    /* 高速化のためにpolarの代わりに複素共役積でやる */
                    /* const float dphi = wrap_phase(phase_orig[k] - phase_orig[k - 1]);
                     * grad[k] = std::polar(1.0f, dphi); */

                    /* 複素共役の積でベクトルを求めて角度だけ抽出する */
                    std::complex<float> delta_vec = spectrum_[ch][k] * std::conj(spectrum_[ch][k - 1]);
                    const float norm_sq = std::norm(delta_vec);
                    grad[k] = (norm_sq > kConsensusNormFloorSq_)? (delta_vec * (1.0f / std::sqrt(norm_sq))): std::complex<float>(1.0f, 0.0f);
                }

                /* 水平位相更新で得られたスペクトル点を複素数表現に変換して保存 */
                for (int k = 0; k < bin_count_; ++k) {
                    horiz[k] = std::polar(mag_out[k], legacy[k]);
                }

                for (int k = 0; k < bin_count_; ++k) {
                    const int below = below_index_[k];
                    const int above = above_index_[k];
                    const float wb = consensus_weight * below_mask_[k];
                    const float wa = consensus_weight * above_mask_[k];

                    /* bin kの上下の隣接点から、水平スペクトル点を位相勾配で回転させたものを得る */
                    const std::complex<float> p_below = horiz[below] * grad[k];
                    const std::complex<float> p_above = horiz[above] * std::conj(grad[above]); /* 逆方向は共役複素数 */

                    const std::complex<float> consensus_raw = (horiz[k] + wb * p_below + wa * p_above);
                    /* const float denom = 1.0f + consensus_weight * neighbor_count_[k];
                     * consensus_raw /= denom;
                     * 重み付き平均なので理論的には必要だが、この振幅はどうせ正規化でなくなるので省略
                     */

                    const float raw_mag_sq = std::norm(consensus_raw);
                    const float safe_mag_sq = raw_mag_sq + 1.0e-20f; /* if分を避けるために最小値を足す（0割防止） */
                    const float inv_mag = 1.0f / std::sqrt(safe_mag_sq);

                    const std::complex<float> consensus_unit = consensus_raw * inv_mag; /* 正規化 */
                    const std::complex<float> blended = consensus_unit * mag_out[k]; /* 元の振幅を復元 */

                    cons[k] = (raw_mag_sq > kConsensusNormFloorSq_)? blended : horiz[k]; /* 垂直位相補正の相殺で振幅が消えてたら水平位相補正だけにする */
                }
            }
        }

        /* IFFT & Overlap-add */
        float hop_peak = 0.0f;
        for (int ch = 0; ch < channel_count_; ++ch) {
            auto& frame = time_frame_[ch];
            auto& accum = synthesis_accum_[ch];
            auto& norm = synthesis_norm_[ch];
            auto& legacy = legacy_phase_[ch];
            auto& mag_out = magnitude_[ch];
            auto& cons = consensus_complex_[ch];
            for (int k = 0; k < bin_count_; ++k) {
                if (use_consensus) {
                    spectrum_[ch][k] = cons[k];
                } else {
                    spectrum_[ch][k] = std::polar(mag_out[k], legacy[k]);
                }
            }

            fft_.inverse_real(spectrum_[ch].data(), frame.data());

            for (int i = 0; i < fft_size_; ++i) {
                const float win = window_[i];
                const float win2 = win * win;
                accum[i] += frame[i] * win;
                norm[i] += win2;
            }

            for (int i = 0; i < syn_hop; ++i) {
                const float denom = norm[i];
                const float out =
                    (denom > kNormFloor_) ? (accum[i] / denom) : 0.0f;
                hop_frame_[ch][i] = out;
                hop_peak = std::max(hop_peak, std::fabs(out));
            }

            const int remain = fft_size_ - syn_hop;
            if (remain > 0) {
                std::move(accum.begin() + syn_hop, accum.end(), accum.begin());
                std::move(norm.begin() + syn_hop, norm.end(), norm.begin());
            }
            std::fill(accum.begin() + remain, accum.end(), 0.0f);
            std::fill(norm.begin() + remain, norm.end(), 0.0f);
        }

        if (input_ended_ && frame_is_fully_padded && hop_peak <= kDrainEpsilon_) {
            return false;
        }

   has_phase_history_ = true;
        output_ring_buffer_.write(
            const_cast<const float* const*>(hop_ptrs_.data()), 0, syn_hop);
        analysis_read_pos_ += static_cast<double>(analysis_hop_);
        return true;
    }

    /* 読み取り済み入力を破棄してリングバッファを圧縮する */
    void compact() {
        // Keep one FFT frame of history before the read position.
        int safe_discard = static_cast<int>(std::floor(analysis_read_pos_)) - fft_size_;
        if (safe_discard <= 0) return;

        const int to_discard = std::min(safe_discard, input_ring_buffer_.buffered());
        input_ring_buffer_.discard(to_discard);
        analysis_read_pos_ -= static_cast<double>(to_discard);
    }
};
