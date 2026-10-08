# visgrok roadmap

Goal: a reliable, from-scratch replacement for the libsigrok `sipeed-slogic-analyzer`
driver, with continuous lossless streaming to disk, real-time analysis and a TUI.

Status legend: ✅ done · 🚧 in progress · ⬜ planned

## Phase 0: protocol analysis
- ✅ Workspace skeleton (`visgrok` library, `visgrok-tui` binary).
- 🚧 Document the USB protocol of the SLogic family from the libsigrok driver,
  Sipeed's sources and the connected SLogic16 U3 → [PROTOCOL.md](PROTOCOL.md).
- ⬜ Verify each request and the sample format on hardware.

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

## Phase 2: SLogic driver
- ⬜ Device discovery (VID/PID table, serial selection) via `rawusb`.
- ⬜ Configuration: sample rate, channel count/bit width, thresholds.
- ⬜ Streaming acquisition with several asynchronous bulk transfers in flight,
  sized to the sample rate; explicit overflow detection.
- ⬜ Clean start/stop/reset so the device can be reused without replugging.
- ⬜ Wire format → canonical block conversion (4/8/16-channel modes).
- ⬜ Hardware tests: loopback of a known signal, long-duration soak at max
  sustainable rate, stop/start cycling, unplug during capture.

## Phase 3: TUI and pipeline
- ✅ Acquisition / writer / analysis threads; the writer is lossless (back-pressure),
  analysis skips blocks when behind and says so.
- ✅ Live channel table (level, edges, frequency, duty, min pulse, role).
- ✅ Role assignment: auto-apply, accept per channel, manual UART/I2C/SPI.
- ✅ Waveform view with zoom, pause; decoded event log; headless mode.
- ⬜ Device options in the CLI (`--channels`, threshold, device serial).
- ⬜ Triggers (start recording on a condition) and pre-trigger ring buffer.
- ⬜ Per-decoder settings in the UI (SPI mode/CS polarity, UART parity/inversion,
  MOSI/MISO swap) and persistence of role assignments.
- ⬜ Export decoded events (text/CSV/JSON) alongside the `.sr` file.

## Phase 4: more analysis
- ⬜ More decoders: 1-Wire, CAN, I2S, JTAG/SWD, PWM/servo measurement.
- ⬜ Analysis on a parallel thread pool for very high rates.
- ⬜ Glitch detection (pulses below a threshold), frequency drift tracking.
