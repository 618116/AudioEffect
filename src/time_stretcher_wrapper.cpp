// time_stretcher_wrapper.cpp
#include "phase_vocoder.h"
#include "time_stretcher.h"

#include <emscripten.h>

namespace {

PhaseVocoder g_phase_vocoder;

int g_sample_rate = 48000;
int g_channels = 2;
bool g_initialized = false;
float g_playback_rate = 1.0f;
float g_phase_control = 0.0f;

void apply_runtime_controls() {
    g_phase_vocoder.setPlaybackRate(g_playback_rate);
    g_phase_vocoder.setPhaseControl(g_phase_control);
}

void reinit_processor() {
    g_phase_vocoder.init(g_sample_rate, g_channels);
    apply_runtime_controls();
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE
void ts_init(int sampleRate, int channels) {
    g_sample_rate = sampleRate;
    g_channels = channels;
    g_initialized = true;
    reinit_processor();
}

EMSCRIPTEN_KEEPALIVE
void ts_reset() {
    g_phase_vocoder.reset();
}

EMSCRIPTEN_KEEPALIVE
void ts_setPlaybackRate(float rate) {
    g_playback_rate = rate;
    g_phase_vocoder.setPlaybackRate(g_playback_rate);
}

EMSCRIPTEN_KEEPALIVE
void ts_setPhaseControl(float control) {
    g_phase_control = control;
    g_phase_vocoder.setPhaseControl(g_phase_control);
}

EMSCRIPTEN_KEEPALIVE
int ts_getNumNeededSamples(int outputSamples) {
    return g_phase_vocoder.getNumNeededSamples(outputSamples);
}

EMSCRIPTEN_KEEPALIVE
int ts_process(const float* const* input, int inputSamples, int inputEnded,
               float** output, int outputSamples) {
    return g_phase_vocoder.process(
        input, inputSamples, inputEnded != 0, output, outputSamples);
}

}
