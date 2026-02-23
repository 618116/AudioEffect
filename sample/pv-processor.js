// pv-processor.js - AudioWorklet processor for Phase Vocoder time stretching
// Loaded as a blob URL combined with pv.js (Emscripten WASM module)

const BLOCK_SIZE = 128; // AudioWorklet render quantum
const MAX_INPUT = 2048; // Max input samples per process call

class PVProcessor extends AudioWorkletProcessor {
  constructor(options) {
    super();
    this.ready = false;
    this.playing = false;
    this.wasm = null;

    // Audio source buffer (received via message)
    this.audioData = null; // Float32Array[]
    this.readPos = 0;
    this.totalSamples = 0;
    this.numChannels = 2;

    // WASM heap pointers
    this.inPtrs = 0; // float** for input
    this.outPtrs = 0; // float** for output
    this.inBufs = []; // float* per channel
    this.outBufs = []; // float* per channel

    this.progressCounter = 0;

    this.port.onmessage = (e) => this.onMessage(e.data);

    const opts = options.processorOptions || {};
    this.initWasm(opts.sampleRate || sampleRate, opts.channels || 2);
  }

  async initWasm(sr, ch) {
    try {
      // Module is defined by the concatenated pv.js (Emscripten MODULARIZE output)
      const mod = await Module();
      this.wasm = mod;
      this.numChannels = ch;
      mod._ts_init(sr, ch);
      this.allocBuffers(ch);
      this.ready = true;
      this.port.postMessage({ type: 'ready' });
    } catch (e) {
      this.port.postMessage({ type: 'error', message: String(e) });
    }
  }

  allocBuffers(ch) {
    const M = this.wasm;
    this.inPtrs = M._malloc(ch * 4);
    this.outPtrs = M._malloc(ch * 4);
    this.inBufs = [];
    this.outBufs = [];
    for (let c = 0; c < ch; c++) {
      const ib = M._malloc(MAX_INPUT * 4);
      const ob = M._malloc(BLOCK_SIZE * 4);
      this.inBufs.push(ib);
      this.outBufs.push(ob);
      M.HEAPU32[(this.inPtrs >> 2) + c] = ib;
      M.HEAPU32[(this.outPtrs >> 2) + c] = ob;
    }
  }

  onMessage(data) {
    switch (data.type) {
      case 'audio':
        this.audioData = data.channelData;
        this.totalSamples = data.channelData[0].length;
        this.readPos = 0;
        if (this.wasm) this.wasm._ts_reset();
        break;
      case 'play':
        if (this.readPos >= this.totalSamples) {
          this.readPos = 0;
          if (this.wasm) this.wasm._ts_reset();
        }
        this.playing = true;
        break;
      case 'stop':
        this.playing = false;
        this.readPos = 0;
        if (this.wasm) this.wasm._ts_reset();
        break;
      case 'pause':
        this.playing = false;
        break;
      case 'playbackRate':
        if (this.wasm) this.wasm._ts_setPlaybackRate(data.value);
        break;
      case 'phase':
        if (this.wasm) this.wasm._ts_setPhaseControl(data.value);
        break;
      case 'seek':
        this.readPos = Math.max(0, Math.floor(data.position));
        if (this.wasm) this.wasm._ts_reset();
        break;
    }
  }

  process(inputs, outputs, parameters) {
    if (!this.ready || !this.playing || !this.audioData) return true;

    const output = outputs[0];
    const M = this.wasm;
    const ch = this.numChannels;
    const needed = M._ts_getNumNeededSamples(BLOCK_SIZE);

    if (needed > MAX_INPUT) return true;

    const remaining = this.totalSamples - this.readPos;
    const avail = remaining > 0 ? Math.min(needed, remaining) : 0;
    const eof = remaining <= 0 ? 1 : 0;

    // Copy only real input samples to WASM heap. EOF tail padding is handled in C++.
    if (avail > 0) {
      for (let c = 0; c < ch; c++) {
        const dst = new Float32Array(M.HEAPF32.buffer, this.inBufs[c], avail);
        dst.set(this.audioData[c].subarray(this.readPos, this.readPos + avail));
      }
    }
    this.readPos += avail;

    // Process one render block
    const produced = M._ts_process(
      this.inPtrs, avail, eof, this.outPtrs, BLOCK_SIZE);

    // Copy output from WASM heap
    for (let c = 0; c < ch; c++) {
      const src = new Float32Array(M.HEAPF32.buffer, this.outBufs[c], BLOCK_SIZE);
      if (c < output.length) output[c].set(src);
    }
    // Mono source: duplicate to right channel
    if (ch === 1 && output.length > 1) {
      output[1].set(output[0]);
    }

    if (eof === 1 && produced === 0) {
      this.playing = false;
      this.port.postMessage({ type: 'ended' });
      return true;
    }

    // Report progress ~every 50ms (every 18 blocks at 48kHz)
    if (++this.progressCounter >= 18) {
      this.progressCounter = 0;
      this.port.postMessage({
        type: 'progress',
        position: this.readPos,
        total: this.totalSamples,
      });
    }

    return true;
  }
}

registerProcessor('pv-processor', PVProcessor);
