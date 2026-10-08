# visgrok roadmap

Goal: a reliable, from-scratch replacement for the libsigrok `sipeed-slogic-analyzer`
driver, with continuous lossless streaming to disk, real-time analysis and a TUI.

Status legend: ✅ done · 🚧 in progress · ⬜ planned

## Phase 0: protocol analysis
- ✅ Workspace skeleton (`visgrok` library, `visgrok-tui` binary).
- ✅ Document the USB protocol of the SLogic family from Sipeed's libsigrok fork,
  their other sources and the connected SLogic16 U3 → [PROTOCOL.md](PROTOCOL.md).
- ✅ Verify requests, sample format, rate encoding and quirks on a SLogic16 U3.

## Phase 1: hardware-independent core
- ✅ Canonical sample blocks (sigrok-compatible 1/2-byte units).
- ✅ Fast transition extraction (word-at-a-time skip of unchanged runs).
- ✅ Per-channel statistics: edge counts, duty, period/jitter, pulse-width histograms.
- ✅ Role auto-detection: idle, free-running clock, UART (with baud snapping),
  I2C SCL/SDA pairing, SPI clock/data/CS grouping.
- ✅ Streaming decoders: UART (5–9 bits, parity, break), I2C, SPI (modes 0–3).
- ✅ Streaming sigrok `.sr` writer (stored zip, Zip64 for large captures),
  verified with `sigrok-cli`.
- ✅ Synthetic source for development without hardware.
- ✅ Compressed `.vgk` capture format ([FORMAT.md](FORMAT.md)): parallel zstd
  chunks via compcol, stored fallback under load, CRCs, crash-tolerant, index;
  replay (`-i`) and conversion to `.sr`.

## Phase 2: SLogic driver
- ✅ Device discovery (VID/PID table, bootloader detection, serial selection) via `rawusb`.
- ✅ SET_CONFIGURATION(1) when unconfigured, reset on open (device may be left in reset).
- ✅ Register/AUX configuration with ready, selector-echo and read-back checks;
  sample rate validated against the 8-bit divider and the bandwidth limit
  (no silent fallback to the maximum rate, unlike sigrok).
- ✅ Streaming with a ring of async bulk transfers resubmitted from the callback,
  ~40 ms per transfer, bounded queue, idle/stall detection.
- ✅ 4-byte head artifact dropped once per acquisition (byte counter);
  4/8/16-channel unpacking with odd-byte carry.
- ✅ Start-up rate verification with automatic restart (device sometimes latches
  the previous sample rate; PROTOCOL.md §6.3).
- ✅ `examples/selftest`: emulation-pattern capture checked sample by sample.
  Results on macOS: 0 errors up to 200 MB/s in all modes; at 400 MB/s
  (16ch@200M, 8ch@400M, 4ch@800M) ~5–10 short gaps per GB remain.
- ⬜ Investigate the residual 400 MB/s losses (Linux comparison; rawusb macOS
  submission path; larger/fewer transfers).
- ⬜ Combo 8 support is implemented from source but untested (no hardware).
- ⬜ SLogic32 U3: 32-channel mode.
- ⬜ Hardware tests with real signals in 4/8-channel modes; unplug during capture.

## Phase 3: TUI and pipeline
- ✅ Acquisition / writer / analysis threads; the writer is lossless (back-pressure),
  analysis skips blocks when behind and says so.
- ✅ Live channel table (level, edges, frequency, duty, min pulse, role).
- ✅ Role assignment: auto-apply, accept per channel, manual UART/I2C/SPI.
- ✅ Waveform view with zoom, pause; decoded event log; headless mode.
- ✅ Device options in the CLI (`--channels`, `--threshold`, `--serial`, `--list`, `--emulation`).
- ⬜ Triggers (start recording on a condition) and pre-trigger ring buffer.
- ⬜ Per-decoder settings in the UI (SPI mode/CS polarity, UART parity/inversion,
  MOSI/MISO swap) and persistence of role assignments.
- ⬜ Export decoded events (text/CSV/JSON) alongside the `.sr` file.

## Phase 4: more analysis
- ⬜ More decoders: 1-Wire, CAN, I2S, JTAG/SWD, PWM/servo measurement.
- ⬜ Analysis on a parallel thread pool for very high rates.
- ⬜ Glitch detection (pulses below a threshold), frequency drift tracking.
