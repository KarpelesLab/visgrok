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

### Decoding your buses

Assign pins with `-r CH=ROLE` (or interactively: select a channel, press Enter):

```sh
# UART with automatic baud detection that follows rate changes
# (e.g. 21.5 kbaud negotiation switching to 2 Mbaud):
visgrok -c 8 -s 50M -r 0=uart
# SPI to an SSD1306 OLED: SCLK, MOSI/DI, D/C, optional CS; live framebuffer in the TUI
visgrok -c 8 -s 50M -r 1=spi-clk -r 2=spi-mosi -r 3=spi-dc -r 4=spi-cs --spi-proto ssd1306
# Try both without hardware:
visgrok --demo device -r 0=uart -r 1=spi-clk -r 2=spi-mosi -r 3=spi-dc -r 4=spi-cs --spi-proto ssd1306
```

Roles: `uart`, `uart:BAUD`, `spi-clk`, `spi-mosi`, `spi-miso`, `spi-cs`, `spi-dc`,
`i2c-scl:SDA_CH`, `i2c-sda:SCL_CH`, `idle`. SPI options: `--spi-mode 0..3`
(default: polarity from the idle clock, CPHA 0), `--spi-cs-high`,
`--spi-proto raw|ssd1306|ssd1306:128x32` (no D/C pin: 3-wire 9-bit mode).
Pick a sample rate at least ~10× the fastest bit rate; decoding needs every
block, and at the 400 MB/s maximum the analyzer may have to skip some.

TUI keys: `↑↓` select channel, `Enter` pick a role, `a` apply auto-detected
roles, `A` accept the selected channel's suggestion, `u` UART (auto; again to
start from a fixed rate), `i` I2C on the selected channel and the next, `x`
clear role, `p` SPI protocol (raw / SSD1306 128×64 / 128×32), `m` SPI mode,
`+`/`-` zoom, space pause, `r` reset stats, `q` quit.
