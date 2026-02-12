// wsola_wasm.cpp
#include "wsola.h"
#include <emscripten.h>

static WSOLA g_wsola;

extern "C" {

EMSCRIPTEN_KEEPALIVE
void wsola_init(int sampleRate, int channels) {
    g_wsola.init(sampleRate, channels);
}

EMSCRIPTEN_KEEPALIVE
void wsola_reset() {
    g_wsola.reset();
}

EMSCRIPTEN_KEEPALIVE
void wsola_setRatio(float ratio) {
    g_wsola.setRatio(ratio);
}

EMSCRIPTEN_KEEPALIVE
int wsola_getNumNeededSamples(int outputSamples) {
    return g_wsola.getNumNeededSamples(outputSamples);
}

EMSCRIPTEN_KEEPALIVE
void wsola_process(const float* const* input, int inputSamples,
                   float** output, int outputSamples) {
    g_wsola.process(input, inputSamples, output, outputSamples);
}

}