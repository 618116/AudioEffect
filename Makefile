EMSDK      ?= D:/Emscipten/emsdk
EMCC        = python $(EMSDK)/upstream/emscripten/emcc.py

SRC_DIR     = src
BUILD_DIR   = build

SRC         = $(SRC_DIR)/time_stretcher_wrapper.cpp
OUT         = $(BUILD_DIR)/pv.js

CXXFLAGS    = -O3 -std=c++17 -I$(SRC_DIR)
LDFLAGS     = -s EXPORTED_FUNCTIONS="['_malloc','_free','_ts_init','_ts_reset','_ts_setRatio','_ts_setPhaseControl','_ts_getNumNeededSamples','_ts_process']" \
              -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap','HEAPU32','HEAPF32']" \
              -s MODULARIZE=1 \
              -s ASSERTIONS=0 \
              -s ENVIRONMENT=web,worker \
              -s ALLOW_MEMORY_GROWTH=0 \
              -s INITIAL_MEMORY=1mb \
              -s SINGLE_FILE=1

WORKLET     = $(BUILD_DIR)/pv-worklet.js
PROC_SRC    = sample/pv-processor.js

all: $(WORKLET)

$(OUT): $(SRC) $(SRC_DIR)/phase_vocoder.h $(SRC_DIR)/simple_fft.h $(SRC_DIR)/time_stretcher.h $(SRC_DIR)/ring_buffer.h | $(BUILD_DIR)
	$(EMCC) $(CXXFLAGS) $(LDFLAGS) $(SRC) -o $(OUT)

$(WORKLET): $(OUT) $(PROC_SRC)
	python -c "open('$(WORKLET)','wb').write(open('$(OUT)','rb').read()+b'\n'+open('$(PROC_SRC)','rb').read())"

$(BUILD_DIR):
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

clean:
	del /q $(BUILD_DIR)\pv.js $(BUILD_DIR)\pv-worklet.js 2>nul || ver >nul

.PHONY: all clean
