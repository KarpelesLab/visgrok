# visgrok

A from-scratch Rust driver for Sipeed SLogic logic analyzers (SLogic16 U3,
SLogic Combo 8, SLogic Lite 8), with continuous streaming to disk, real-time
signal analysis and a terminal UI.

- `crates/visgrok` — library: USB protocol (on top of [`rawusb`](https://github.com/KarpelesLab/rawusb)), streaming, analysis, file output.
- `crates/visgrok-tui` — `visgrok` binary: live TUI while recording.

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the USB protocol and
[docs/ROADMAP.md](docs/ROADMAP.md) for the plan.

## Usage

```sh
cargo build --release
./target/release/visgrok --list
# 16 channels at the maximum rate (200 MHz), 1.65 V threshold, record, auto-detect roles:
./target/release/visgrok -c 16 -t 1.65 -o capture.vgk --auto
# Replay a capture through the analyzers, or convert it for PulseView:
./target/release/visgrok -i capture.vgk --headless --auto
./target/release/visgrok -i capture.vgk -o capture.sr --headless
# No hardware: synthetic UART/I2C/SPI/clock signals
./target/release/visgrok --demo --auto
# Verify the USB link: device test pattern checked sample by sample
cargo run --release -p visgrok --example selftest -- 16 100000000 5
```

Recordings use visgrok's compressed `.vgk` format by default (typically 30× to
25 000× smaller than raw; see [docs/FORMAT.md](docs/FORMAT.md)). Name the
output `*.sr` to write a sigrok session for PulseView / `sigrok-cli` directly.

TUI keys: `↑↓` select channel, `a` apply auto-detected roles, `A` accept the
selected channel's suggestion, `u` UART (again to cycle baud), `i` I2C on the
selected channel and the next, `s` SPI on four channels starting at the
selection, `x` clear role, `+`/`-` zoom, space pause, `r` reset stats, `q` quit.
