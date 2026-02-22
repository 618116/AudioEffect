#pragma once
/* ============================================================================
 * wsola.h - WSOLA タイムストレッチャー（相互相関による類似度検索付き）
 *
 * TimeStretcher から入出力リングバッファ管理を継承。
 * produce_frames(): 相互相関で最適オフセットを探索し、
 *                   Hann窓によるオーバーラップ加算でフレームを合成。
 * ============================================================================ */

#include "time_stretcher.h"

class WSOLA : public TimeStretcher {
protected:
    /* 初期化処理（TimeStretcher から呼ばれる） */
    void onInit() override {
        /* 合成ホップサイズをフレームサイズの半分に設定（最小1） */
        synthesis_hop_ = std::max(1, frame_size_ / 2);
        /* 相互相関の探索範囲をホップサイズの半分に設定 */
        search_range_ = synthesis_hop_ / 2;

        /* Hannベースのクロスフェードカーブを生成 */
        generate_hann_crossfade(synthesis_hop_, fade_in_, fade_out_);

        /* 前フレーム保持用バッファを全チャンネル分確保 */
        prev_frame_.assign(channel_count_, std::vector<float>(frame_size_, 0.0f));
        /* ホップ単位の出力バッファを全チャンネル分確保 */
        hop_frame_.assign(channel_count_, std::vector<float>(synthesis_hop_, 0.0f));
        /* 各チャンネルのホップバッファへのポインタ配列を構築 */
        hop_ptrs_.resize(channel_count_);
        for (int ch = 0; ch < channel_count_; ++ch) {
            hop_ptrs_[ch] = hop_frame_[ch].data();
        }

        /* 相互相関の候補領域読み込み用バッファを確保 */
        correlation_buf_.assign(synthesis_hop_, 0.0f);

        /* フェードカーブ生成は TimeStretcher 側の共通ヘルパーに委譲 */
    }

    /* 必要な出力サンプル数に達するまでホップ単位で生成 */
    void produce_frames(int needed_output) override {
        /* 出力バッファの蓄積量が不足している間、ホップを生成し続ける */
        while (output_ring_buffer_.buffered() < needed_output) {
            if (!produce_one_hop()) {
                break; /* 入力不足または終了条件でループ脱出 */
            }
        }
        compact(); /* 読み終わった入力データを破棄 */
    }

    /* 状態リセット処理 */
    void onReset() override {
        read_position_ = 0.0; /* 読み取り位置を先頭に戻す */
        has_prev_frame_ = false; /* 前フレーム保持フラグをクリア */
        for (int ch = 0; ch < channel_count_; ++ch) {
            std::fill(prev_frame_[ch].begin(), prev_frame_[ch].end(), 0.0f); /* 前フレームバッファをゼロクリア */
            std::fill(hop_frame_[ch].begin(), hop_frame_[ch].end(), 0.0f); /* ホップバッファをゼロクリア */
        }
    }

private:
    int synthesis_hop_ = 0;                          /* 合成ホップサイズ（出力の刻み幅） */
    int search_range_ = 0;                           /* 相互相関の探索範囲（片側） */
    double read_position_ = 0.0;                     /* 入力バッファ上の現在の読み取り位置（小数点精度） */
    bool has_prev_frame_ = false;                    /* 前フレームが存在するかどうか */
    static constexpr float kDrainEpsilon_ = 1.0e-8f; /* 終端判定用の微小閾値 */
    std::vector<float> fade_in_;                     /* フェードインウィンドウ */
    std::vector<float> fade_out_;                    /* フェードアウトウィンドウ */
    std::vector<std::vector<float>> prev_frame_;     /* 前フレームデータ（チャンネル×フレームサイズ） */
    std::vector<std::vector<float>> hop_frame_;      /* ホップ出力データ（チャンネル×ホップサイズ） */
    std::vector<float*> hop_ptrs_;                   /* hop_frame_ の各チャンネルへのポインタ配列 */
    std::vector<float> correlation_buf_;             /* 相互相関の候補領域読み込み用バッファ */

    /* 相互相関により最適なフレームオフセットを探索する */
    int find_best_offset(int nominal_read_pos) {
        int available = input_ring_buffer_.buffered();
        /* テンプレート：前フレーム末尾の synthesis_hop_ サンプル（ch0 のみ使用） */
        const float* tmpl = prev_frame_[0].data() + (frame_size_ - synthesis_hop_);

        float best_corr = -1e30f;
        int best_delta = 0;

        for (int delta = -search_range_; delta <= search_range_; ++delta) {
            int candidate_pos = nominal_read_pos + delta;
            /* 候補位置が有効範囲外ならスキップ */
            if (candidate_pos < 0
                || candidate_pos + synthesis_hop_ > available) {
                continue;
            }
            /* 候補領域を入力バッファから読み込み */
            input_ring_buffer_.peek(
                0, correlation_buf_.data(), 0, candidate_pos, synthesis_hop_);
            /* 相互相関を計算 */
            float corr = 0.0f;
            for (int i = 0; i < synthesis_hop_; ++i) {
                corr += tmpl[i] * correlation_buf_[i];
            }
            if (corr > best_corr) {
                best_corr = corr;
                best_delta = delta;
            }
        }
        return best_delta;
    }

    /* オーバーラップ加算で1ホップ分の合成出力を生成する */
    bool produce_one_hop() {
        int read_pos = static_cast<int>(std::floor(read_position_)); /* 読み取り位置を整数化 */
        int available = input_ring_buffer_.buffered(); /* 入力バッファの残りサンプル数 */
        bool frame_is_fully_padded = false; /* フレーム全体がゼロ埋めかどうか */

        /* 相互相関による最適オフセット探索（前フレームが存在する場合のみ） */
        if (has_prev_frame_ && !input_ended_) {
            int delta = find_best_offset(read_pos);
            read_pos += delta;
            read_position_ += static_cast<double>(delta);
        }

        /* 入力バッファに十分なサンプルがあるか確認 */
        if (read_pos + frame_size_ > available) {
            /* 入力終了が通知されていなければ、データ待ち */
            if (!input_ended_) {
                return false;
            }

            /* 入力終了済み：テンポラリフレームをゼロ埋め */
            for (int ch = 0; ch < channel_count_; ++ch) {
                std::fill(temp_frame_[ch].begin(), temp_frame_[ch].end(), 0.0f);
            }

            if (read_pos < available) {
                /* 残りのサンプルだけ読み込み、残りはゼロのまま */
                int available_from_read = available - read_pos;
                for (int ch = 0; ch < channel_count_; ++ch) {
                    input_ring_buffer_.peek(
                        ch, temp_ptrs_[ch], 0, read_pos, available_from_read);
                }
            } else {
                /* 読み取り位置が利用可能範囲を完全に超えている */
                frame_is_fully_padded = true;
            }
        } else {
            /* 通常読み取り：入力バッファから1フレーム分を非消費的に取得 */
            for (int ch = 0; ch < channel_count_; ++ch) {
                input_ring_buffer_.peek(ch, temp_ptrs_[ch], 0, read_pos, frame_size_);
            }
        }

        /* 出力バッファに書き込み可能な空きがあるか確認 */
        if (output_ring_buffer_.writable() < synthesis_hop_) {
            return false;
        }

        if (!has_prev_frame_) {
            /* 最初のフレーム：オーバーラップなしでそのまま出力 */
            for (int ch = 0; ch < channel_count_; ++ch) {
                /* 現在のフレームを前フレームとして保存 */
                std::copy(temp_frame_[ch].begin(), temp_frame_[ch].end(),
                          prev_frame_[ch].begin());
                /* 先頭ホップ分をそのまま出力バッファにコピー */
                std::copy(prev_frame_[ch].begin(),
                          prev_frame_[ch].begin() + synthesis_hop_,
                          hop_frame_[ch].begin());
            }
            has_prev_frame_ = true; /* 前フレーム保持フラグをセット */
        } else {
            /* 2フレーム目以降：前フレーム末尾と現フレーム先頭をクロスフェード */
            int prev_overlap_pos = frame_size_ - synthesis_hop_; /* 前フレームのオーバーラップ開始位置 */
            for (int ch = 0; ch < channel_count_; ++ch) {
                for (int i = 0; i < synthesis_hop_; ++i) {
                    float prev_sample = prev_frame_[ch][prev_overlap_pos + i]; /* 前フレームの重複部分 */
                    float cur_sample = temp_frame_[ch][i]; /* 現フレームの先頭部分 */
                    /* フェードアウト×前フレーム + フェードイン×現フレーム でクロスフェード */
                    hop_frame_[ch][i] = prev_sample * fade_out_[i]
                                      + cur_sample * fade_in_[i];
                }
                /* 現フレームを次回のために前フレームとして保存 */
                std::copy(temp_frame_[ch].begin(), temp_frame_[ch].end(),
                          prev_frame_[ch].begin());
            }
        }

        /* 入力終了かつ全ゼロ埋めの場合、出力がほぼ無音なら終了 */
        if (input_ended_ && frame_is_fully_padded) {
            float hop_peak = 0.0f; /* ホップ内のピーク振幅 */
            for (int ch = 0; ch < channel_count_; ++ch) {
                for (int i = 0; i < synthesis_hop_; ++i) {
                    float v = std::fabs(hop_frame_[ch][i]);
                    if (v > hop_peak) {
                        hop_peak = v; /* 最大振幅を更新 */
                    }
                }
            }
            /* ピーク振幅が閾値以下なら、残響が消えたとみなし終了 */
            if (hop_peak <= kDrainEpsilon_) {
                return false;
            }
        }

        /* 1ホップ分を出力リングバッファに書き込み */
        output_ring_buffer_.write(
            const_cast<const float* const*>(hop_ptrs_.data()), 0, synthesis_hop_);

        /* 読み取り位置をタイムストレッチ比率に応じて進める */
        read_position_ += static_cast<double>(synthesis_hop_)
                        * static_cast<double>(time_stretch_ratio_);

        return true; /* 1ホップの生成に成功 */
    }

    /* 読み取り済みの入力サンプルを破棄してバッファを圧縮する */
    void compact() {
        /* 安全に破棄できるサンプル数を計算（読み取り位置 - フレームサイズ - 探索範囲分の余裕） */
        int safe_discard = static_cast<int>(std::floor(read_position_))
                         - frame_size_ - search_range_;
        if (safe_discard <= 0) {
            /* 破棄不可：入力終了かつバッファ空の場合、読み取り位置を制限 */
            if (input_ended_ && input_ring_buffer_.buffered() == 0
                && read_position_ > static_cast<double>(frame_size_)) {
                read_position_ = static_cast<double>(frame_size_);
            }
            return;
        }

        int available = input_ring_buffer_.buffered(); /* 入力バッファの残りサンプル数 */
        int to_discard = std::min(safe_discard, available); /* 実際に破棄するサンプル数 */

        input_ring_buffer_.discard(to_discard); /* 入力バッファから破棄 */
        read_position_ -= to_discard; /* 読み取り位置を破棄分だけ戻す */

        /* 入力終了かつバッファ空の場合、読み取り位置がフレームサイズを超えないよう制限 */
        if (input_ended_ && input_ring_buffer_.buffered() == 0
            && read_position_ > static_cast<double>(frame_size_)) {
            read_position_ = static_cast<double>(frame_size_);
        }
    }
};
