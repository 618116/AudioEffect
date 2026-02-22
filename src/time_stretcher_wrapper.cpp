// time_stretcher_wrapper.cpp
#include "phase_vocoder.h"
#include "time_stretcher.h"
#include "wsola.h"

#include <emscripten.h>

namespace {

constexpr int kAlgorithmWsola = 0;
constexpr int kAlgorithmPhaseVocoder = 1;

WSOLA g_wsola;
PhaseVocoder g_phase_vocoder;
TimeStretcher* g_processor = &g_wsola;

int g_sample_rate = 48000;
int g_channels = 2;
bool g_initialized = false;
int g_algorithm = kAlgorithmWsola;
float g_ratio = 1.0f;
float g_phase_control = 0.0f;

int sanitize_algorithm(int algorithm) {
    return (algorithm == kAlgorithmPhaseVocoder)
               ? kAlgorithmPhaseVocoder
               : kAlgorithmWsola;
}

TimeStretcher* select_processor(int algorithm) {
    if (algorithm == kAlgorithmPhaseVocoder) {
        return &g_phase_vocoder;
    }
    return &g_wsola;
}

void apply_runtime_controls() {
    g_processor->setRatio(g_ratio);
    g_processor->setPhaseControl(g_phase_control);
}

void reinit_current_processor() {
    g_processor->init(g_sample_rate, g_channels);
    apply_runtime_controls();
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
void ts_init(int sampleRate, int channels) {
    g_sample_rate = sampleRate;
    g_channels = channels;
    g_initialized = true;
    reinit_current_processor();
}

EMSCRIPTEN_KEEPALIVE
void ts_setAlgorithm(int algorithm) {
    const int next = sanitize_algorithm(algorithm);
    if (next == g_algorithm && g_initialized) {
        return;
    }
    g_algorithm = next;
    g_processor = select_processor(g_algorithm);

    if (g_initialized) {
        reinit_current_processor();
    }
}

EMSCRIPTEN_KEEPALIVE
void ts_reset() {
    g_processor->reset();
}

EMSCRIPTEN_KEEPALIVE
void ts_setRatio(float ratio) {
    g_ratio = ratio;
    g_processor->setRatio(g_ratio);
}

EMSCRIPTEN_KEEPALIVE
void ts_setPhaseControl(float control) {
    g_phase_control = control;
    g_processor->setPhaseControl(g_phase_control);
}

EMSCRIPTEN_KEEPALIVE
int ts_getNumNeededSamples(int outputSamples) {
    return g_processor->getNumNeededSamples(outputSamples);
}

EMSCRIPTEN_KEEPALIVE
int ts_process(const float* const* input, int inputSamples, int inputEnded,
               float** output, int outputSamples) {
    return g_processor->process(
        input, inputSamples, inputEnded != 0, output, outputSamples);
}

}
