# visgrok roadmap

Goal: a reliable, from-scratch replacement for the libsigrok `sipeed-slogic-analyzer`
driver, with continuous lossless streaming to disk, real-time analysis and a TUI.

Status legend: ✅ done · 🚧 in progress · ⬜ planned

## Phase 0: protocol analysis
- ✅ Crate skeleton: `visgrok` library plus the `visgrok` binary (feature `cli`).
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
- ✅ UART automatic baud detection without snapping to standard rates, following
  rate changes in both directions (retroactive re-decode of the frame where the
  rate changed): e.g. 21.5 kbaud → 2 Mbaud negotiation.
- ✅ SPI with D/C line, CPOL inferred from the idle clock, word resync on clock
  pauses without CS; SSD1306 layer: full command decoding, RAM pointer tracking
  in all addressing modes, live framebuffer rendered in the TUI (braille).
- ✅ `--demo device`: synthetic UART negotiation + SSD1306 for testing.
- ✅ SD card bus decoder (CLK/CMD/DAT0-3): commands, R1/R1b/R2/R3/R6/R7
  responses (status flags and state, CID, CSD capacity, OCR), data blocks in
  1-/4-bit mode with CRC16, write CRC status and busy; data expected only after
  data commands so busy isn't misread; block sizes confirmed by CRC (CMD16 /
  CMD42 18-byte password blocks); CMD42 payload decoding; clock-relative glitch
  filter, frame resynchronization and plausibility checks against CMD crosstalk.
  Verified on a real 20 s head-unit boot capture: all 29,257 blocks CRC-clean.
  SD-over-SPI not yet.
- ✅ Streaming sigrok `.sr` writer (stored zip, Zip64 for large captures),
  verified with `sigrok-cli`.
- ✅ Synthetic source for development without hardware.
- ✅ Compressed `.vgk` capture format ([FORMAT.md](FORMAT.md)): parallel zstd
  chunks via compcol, stored fallback under load, CRCs, crash-tolerant, index;
  replay (`-i`) and conversion to `.sr`.
- ✅ Format conversion (`visgrok convert`): `.vgk`, `.sr` (read stored/deflated,
  write stored/deflated, zip64), `.vcd` (read/write, rate from comment or
  timestamp GCD), raw `.bin`; verified against sigrok-cli.
- ✅ GitHub CI: fmt, clippy -D warnings and tests on Linux/Windows/macOS, MSRV
  1.89, rustdoc -D warnings.

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
- 🚧 SLogic32 U3 (`359f:3032`): implemented from the protocol spec, not yet
  tested on hardware. 32-bit samples throughout (4-byte units in blocks, `.vgk`
  and `.sr`), 1400/800 MHz base clocks (4ch→1.4 GHz, 8ch→800, 16ch→400,
  32ch→200 MHz, up to 800 MB/s), TUI shows only active/assigned channels when
  there are more than 16 (`v` toggles all). To verify: the divider width, and
  Sipeed's notes that reduced modes may carry the high channel bytes or read
  zeros on real pins.
- ⬜ Hardware tests with real signals in 4/8-channel modes; unplug during capture.

## Phase 3: TUI and pipeline
- ✅ Acquisition / writer / analysis threads; the writer is lossless (back-pressure),
  analysis skips blocks when behind and says so.
- ✅ Live channel table (level, edges, frequency, duty, min pulse, role).
- ✅ Role assignment: auto-apply, accept per channel, role picker, `--role CH=ROLE`.
- ✅ Waveform view with zoom, pause; decoded event log; headless mode.
- ✅ Device options in the CLI (`--channels`, `--threshold`, `--serial`, `--list`, `--emulation`).
- ✅ Web UI (`visgrok web`): capture control, channel names/roles, live and
  recorded browsing with zoom/pan/minimap/follow-live, decoded events overlaid
  and listed; std-only HTTP/WebSocket server; sample store (overview tiles,
  recent-block ring, on-disk chunk index) keeps the capture independent of the
  UI (verified: 100 MS/s sustained while hammered; 0.2–7 ms views on a 20 s,
  4-billion-sample capture); background import of .sr/.vcd and decoding.
- ✅ Sidecar files (`<capture>.json`): channel names, roles/buses, decoder and
  recording settings, bookmarks, notes; written by recordings and the web UI,
  read by replay, `--info` and the web UI, carried by `convert`.
- ⬜ Web UI: measurements (cursors, deltas,
  frequency), decoder settings beyond SPI protocol/UART format, SSD1306 screen.
- ⬜ Triggers (start recording on a condition) and pre-trigger ring buffer.
- ⬜ Per-decoder settings in the UI (SPI mode/CS polarity, UART parity/inversion,
  MOSI/MISO swap) and persistence of role assignments.
- ⬜ Export decoded events (text/CSV/JSON) alongside the `.sr` file.

## Phase 4: more analysis
- ⬜ More decoders: SD over SPI, 1-Wire, CAN, I2S, JTAG/SWD, PWM/servo measurement.
- ⬜ Analysis on a parallel thread pool for very high rates.
- ⬜ Glitch detection (pulses below a threshold), frequency drift tracking.
