// time_stretcher_wrapper.cpp
#include "wsola.h"
#include <emscripten.h>

static WSOLA g_processor;

extern "C" {

EMSCRIPTEN_KEEPALIVE
void ts_init(int sampleRate, int channels) {
    g_processor.init(sampleRate, channels);
}

EMSCRIPTEN_KEEPALIVE
void ts_reset() {
    g_processor.reset();
}

EMSCRIPTEN_KEEPALIVE
void ts_setRatio(float ratio) {
    g_processor.setRatio(ratio);
}

EMSCRIPTEN_KEEPALIVE
int ts_getNumNeededSamples(int outputSamples) {
    return g_processor.getNumNeededSamples(outputSamples);
}

EMSCRIPTEN_KEEPALIVE
int ts_process(const float* const* input, int inputSamples, int inputEnded,
               float** output, int outputSamples) {
    return g_processor.process(
        input, inputSamples, inputEnded != 0, output, outputSamples);
}

}
