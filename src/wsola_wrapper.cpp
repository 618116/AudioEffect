// wsola_wrapper.cpp
#include "wsola.h"
#include <emscripten.h>

static WSOLA g_wsolaProcessor;

extern "C" {

EMSCRIPTEN_KEEPALIVE
void wsola_init(int sampleRate, int channels) {
    g_wsolaProcessor.init(sampleRate, channels);
}

EMSCRIPTEN_KEEPALIVE
void wsola_reset() {
    g_wsolaProcessor.reset();
}

EMSCRIPTEN_KEEPALIVE
void wsola_setRatio(float ratio) {
    g_wsolaProcessor.setRatio(ratio);
}

EMSCRIPTEN_KEEPALIVE
int wsola_getNumNeededSamples(int outputSamples) {
    return g_wsolaProcessor.getNumNeededSamples(outputSamples);
}

EMSCRIPTEN_KEEPALIVE
void wsola_process(const float* const* input, int inputSamples,
                   float** output, int outputSamples) {
    g_wsolaProcessor.process(input, inputSamples, output, outputSamples);
}

}
