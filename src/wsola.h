#pragma once
// ============================================================================
// wsola.h â€” Minimal real-time WSOLA time-stretcher
//
// API:
//   init(sampleRate, channels)          â€” allocate, configure
//   reset()                             â€” clear state (on seek/discontinuity)
//   setRatio(ratio)                     â€” 0.5â€“2.0, 1.0 = no stretch
//   getNumNeededSamples(outputSamples)  â€” how many input samples to provide
//   process(in, inLen, out, outLen)     â€” deinterleaved float**
//
// ============================================================================

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cassert>

struct WSOLAConfig {
    int overlapLen = 128;   // Crossfade length (samples). Determines lowest
                            // frequency handled cleanly: ~sampleRate/overlapLen Hz.
                            // 128 @ 48kHz â‰ˆ 375Hz+. Use 256 for 187Hz+, 512 for 94Hz+.

    int seekWin    = 128;   // Cross-correlation search range (Â±samples).
                            // Larger = better match, more CPU. 128 is good up to 1.5Ã—.

    int seqLen     = 256;   // Samples copied between splice points.
                            // Must be >= overlapLen. 256â€“512 is a good range.
};

class WSOLA {
public:
    using Config = WSOLAConfig;

    void init(int sampleRate, int channels, Config config = Config()) {
        sr_       = sampleRate;
        ch_       = channels;
        cfg_      = config;

        assert(cfg_.seqLen >= cfg_.overlapLen);

        // History buffer per channel: enough to hold input for correlation search.
        histCap_ = (cfg_.seqLen + cfg_.seekWin * 2 + cfg_.overlapLen) * 4;
        hist_.resize(ch_);
        overlap_.resize(ch_);
        for (int c = 0; c < ch_; ++c) {
            hist_[c].resize(histCap_, 0.0f);
            overlap_[c].resize(cfg_.overlapLen, 0.0f);
        }

        reset();
    }

    void reset() {
        histLen_    = 0;
        readPos_    = 0.0;
        hasOverlap_ = false;
        for (int c = 0; c < ch_; ++c) {
            std::fill(hist_[c].begin(), hist_[c].end(), 0.0f);
            std::fill(overlap_[c].begin(), overlap_[c].end(), 0.0f);
        }
    }

    void setRatio(float ratio) {
        ratio_ = std::clamp(ratio, 0.5f, 2.0f);
    }

    float getRatio() const { return ratio_; }

    // How many input samples you should provide for the given output length.
    int getNumNeededSamples(int outputSamples) const {
        if (std::abs(ratio_ - 1.0f) < 0.005f) return outputSamples;
        // We consume input at ratio speed. Add margin for seek + sequence lookahead.
        int totalAhead = (int)std::ceil((double)outputSamples * (double)ratio_)
                       + cfg_.seqLen + cfg_.seekWin;
        int available  = histLen_ - (int)readPos_;
        return std::max(0, totalAhead - available);
    }

    // Process: read from input, write to output.
    // input[ch][0..inputSamples-1], output[ch][0..outputSamples-1]
    void process(const float* const* input, int inputSamples,
                 float**             output, int outputSamples)
    {
        float r = ratio_;

        // --- Fast path: passthrough ---
        if (std::abs(r - 1.0f) < 0.005f && !hasOverlap_) {
            int n = std::min(inputSamples, outputSamples);
            for (int c = 0; c < ch_; ++c) {
                std::memcpy(output[c], input[c], n * sizeof(float));
                if (outputSamples > n)
                    std::memset(output[c] + n, 0, (outputSamples - n) * sizeof(float));
            }
            return;
        }

        // --- Push input into history ---
        pushHistory(input, inputSamples);

        // --- Produce output via WSOLA ---
        int written = 0;

        while (written < outputSamples) {
            int nominalPos = (int)readPos_;

            // Check we have enough history ahead
            int needed = nominalPos + cfg_.seqLen + cfg_.seekWin;
            if (needed > histLen_) {
                // Starved â€” zero-fill remainder
                for (int c = 0; c < ch_; ++c)
                    std::memset(output[c] + written, 0,
                                (outputSamples - written) * sizeof(float));
                break;
            }

            // Find best splice position
            int bestOffset = hasOverlap_ ? findBestOverlap(nominalPos) : 0;
            int splicePos  = nominalPos + bestOffset;

            // Body length = sequence minus crossfade region
            int bodyLen = cfg_.seqLen - cfg_.overlapLen;

            if (hasOverlap_) {
                // Crossfade overlap tail into new sequence
                int fadeLen = std::min(cfg_.overlapLen, outputSamples - written);
                crossfade(output, written, splicePos, fadeLen);
                written += fadeLen;

                // Copy body (non-crossfaded portion)
                int toWrite = std::min(bodyLen, outputSamples - written);
                if (toWrite > 0) {
                    copyFromHist(output, written, splicePos + cfg_.overlapLen, toWrite);
                    written += toWrite;
                }
            } else {
                // First sequence â€” no crossfade
                int toWrite = std::min(cfg_.seqLen, outputSamples - written);
                copyFromHist(output, written, splicePos, toWrite);
                written += toWrite;
            }

            // Save tail of this sequence for next crossfade
            saveOverlap(splicePos + cfg_.seqLen - cfg_.overlapLen);

            // Advance read position: consume input at ratio rate
            readPos_ += (double)cfg_.seqLen * (double)r;
        }

        // --- Compact history: discard consumed samples ---
        compact();
    }

    int latencySamples() const { return cfg_.overlapLen + cfg_.seekWin; }
    float latencyMs()    const { return 1000.0f * latencySamples() / sr_; }

private:
    int   sr_  = 44100;
    int   ch_  = 2;
    float ratio_ = 1.0f;
    Config cfg_;

    std::vector<std::vector<float>> hist_;     // [ch][sample] â€” input history
    std::vector<std::vector<float>> overlap_;  // [ch][overlapLen] â€” crossfade tail
    int    histCap_    = 0;
    int    histLen_    = 0;
    double readPos_    = 0.0;
    bool   hasOverlap_ = false;

    // ------------------------------------------------------------------
    void pushHistory(const float* const* input, int n) {
        int space = histCap_ - histLen_;
        int push  = std::min(n, space);
        for (int c = 0; c < ch_; ++c)
            std::memcpy(hist_[c].data() + histLen_, input[c], push * sizeof(float));
        histLen_ += push;
    }

    // ------------------------------------------------------------------
    // Normalized cross-correlation search.
    // Compare overlap_[] against history at positions around nominalPos.
    // Returns offset from nominalPos.
    // ------------------------------------------------------------------
    int findBestOverlap(int nominalPos) {
        int lo = std::max(0, nominalPos - cfg_.seekWin);
        int hi = std::min(histLen_ - cfg_.overlapLen, nominalPos + cfg_.seekWin);
        if (lo >= hi) return 0;

        // Energy of overlap tail (constant across search)
        float energyA = 0.0f;
        for (int c = 0; c < ch_; ++c)
            for (int i = 0; i < cfg_.overlapLen; ++i)
                energyA += overlap_[c][i] * overlap_[c][i];

        // Initial candidate energy at position lo
        float energyB = 0.0f;
        for (int c = 0; c < ch_; ++c)
            for (int i = 0; i < cfg_.overlapLen; ++i)
                energyB += hist_[c][lo + i] * hist_[c][lo + i];

        int   bestOff  = 0;
        float bestCorr = -1e30f;

        for (int pos = lo; pos < hi; ++pos) {
            // Dot product across all channels
            float corr = 0.0f;
            for (int c = 0; c < ch_; ++c)
                for (int i = 0; i < cfg_.overlapLen; ++i)
                    corr += overlap_[c][i] * hist_[c][pos + i];

            // Normalize
            float denom = std::sqrt(energyA * energyB);
            if (denom > 1e-8f) corr /= denom;

            if (corr > bestCorr) {
                bestCorr = corr;
                bestOff  = pos - nominalPos;
            }

            // Slide energyB: remove leaving sample, add entering sample
            if (pos + 1 < hi) {
                for (int c = 0; c < ch_; ++c) {
                    float out = hist_[c][pos];
                    float in  = hist_[c][pos + cfg_.overlapLen];
                    energyB += in * in - out * out;
                }
                energyB = std::max(0.0f, energyB);
            }
        }

        return bestOff;
    }

    // ------------------------------------------------------------------
    // Crossfade overlap_[] with history at splicePos, write to output
    // ------------------------------------------------------------------
    void crossfade(float** output, int outPos, int histPos, int fadeLen) {
        for (int i = 0; i < fadeLen; ++i) {
            float t = (float)i / (float)(cfg_.overlapLen - 1);
            for (int c = 0; c < ch_; ++c) {
                output[c][outPos + i] = overlap_[c][i] * (1.0f - t)
                                      + hist_[c][histPos + i] * t;
            }
        }
    }

    // ------------------------------------------------------------------
    void copyFromHist(float** output, int outPos, int histPos, int n) {
        int safe = std::min(n, histLen_ - histPos);
        if (safe <= 0) return;
        for (int c = 0; c < ch_; ++c)
            std::memcpy(output[c] + outPos, hist_[c].data() + histPos,
                        safe * sizeof(float));
    }

    // ------------------------------------------------------------------
    void saveOverlap(int histPos) {
        int pos = std::clamp(histPos, 0, histLen_ - cfg_.overlapLen);
        for (int c = 0; c < ch_; ++c)
            std::memcpy(overlap_[c].data(), hist_[c].data() + pos,
                        cfg_.overlapLen * sizeof(float));
        hasOverlap_ = true;
    }

    // ------------------------------------------------------------------
    // Discard history samples behind the read cursor
    // ------------------------------------------------------------------
    void compact() {
        int discard = (int)readPos_ - cfg_.seekWin - cfg_.overlapLen;
        if (discard <= 0) return;
        discard = std::min(discard, histLen_);

        for (int c = 0; c < ch_; ++c)
            std::memmove(hist_[c].data(), hist_[c].data() + discard,
                         (histLen_ - discard) * sizeof(float));
        histLen_  -= discard;
        readPos_  -= discard;
    }
};