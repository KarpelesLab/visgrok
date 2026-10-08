# visgrok

A from-scratch Rust driver for Sipeed SLogic logic analyzers (SLogic16 U3,
SLogic32 U3, SLogic Combo 8), with continuous streaming to disk, real-time
signal analysis and a terminal UI.

One crate, `visgrok`:

- the library (`src/`): USB driver (on top of [`rawusb`](https://github.com/KarpelesLab/rawusb)), streaming, analysis, decoders, file formats;
- the `visgrok` binary (`src/bin/visgrok/`, feature `cli`, on by default): live TUI, recording, replay and conversion.

```sh
cargo install visgrok            # the binary
cargo add visgrok --no-default-features   # just the library
```

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
cargo run --release --example selftest -- 16 100000000 5
```

Convert between formats (`.vgk`, `.sr` — stored or deflated, read from
sigrok/PulseView too — `.vcd`, raw `.bin`) without re-analysis:

```sh
visgrok convert capture.vgk capture.sr        # for PulseView (deflated)
visgrok convert pulseview.sr capture.vgk      # sigrok session to visgrok
visgrok convert capture.vgk capture.vcd       # GTKWave, simulators
visgrok convert dump.bin capture.vgk -s 100M -c 8   # raw samples need rate + channels
visgrok --info any.sr                         # per-channel activity summary
```

Recordings use visgrok's compressed `.vgk` format by default (typically 30× to
25 000× smaller than raw; see [docs/FORMAT.md](docs/FORMAT.md)). Name the
output `*.sr` to write a sigrok session for PulseView / `sigrok-cli` directly.

### Web UI

```sh
visgrok web                        # http://127.0.0.1:8090/, captures in ./captures
visgrok web --dir ~/captures navi.sr   # open a capture right away
```

From the browser: record (SLogic or demo source; channels, rate, threshold),
assign channel names and roles (UART, SPI/SSD1306, I2C, SD card), browse any
part of a live or recorded capture (wheel to zoom, drag to pan, minimap,
`End` to follow live, `[` `]` to jump between decoded events) and click
decoded events to jump to them. The view position is kept in the URL, so
links point at a moment in a capture.

Channel names, roles (buses), decoder settings, bookmarks (`b`) and notes
are saved in a sidecar next to the capture (`capture.vgk.json`, see
[docs/FORMAT.md](docs/FORMAT.md)) and restored when it is opened again —
also by `visgrok -i` and `visgrok --info`.

The capture never waits for the page: data streams to disk and the browser
reads it through a sample store (a 0.2% overview for zoomed-out views, raw
chunks for zoomed-in ones). `.sr`/`.vcd` files are imported once into a
`.vgk` next to them. It binds to localhost by default (`--listen` to change).

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
`i2c-scl:SDA_CH`, `i2c-sda:SCL_CH`, `sd-clk`, `sd-cmd`, `sd-dat0`..`sd-dat3`, `idle`.

SD cards (native SD bus): commands and responses by name with CRC7 checks
(R1 status/state, CID, CSD capacity, OCR/SDHC, RCA), data blocks with per-line
CRC16 in 1- or 4-bit mode, write CRC status tokens and busy times:

```sh
visgrok -c 8 -s 200M -r 0=sd-clk -r 1=sd-cmd -r 2=sd-dat0 -r 3=sd-dat1 -r 4=sd-dat2 -r 5=sd-dat3
```

Sample at least 4× the SD clock (default speed 25 MHz → 100 MHz or more;
high speed 50 MHz → 200 MHz+). UHS-I SDR50/SDR104 clocks are too fast. SPI options: `--spi-mode 0..3`
(default: polarity from the idle clock, CPHA 0), `--spi-cs-high`,
`--spi-proto raw|ssd1306|ssd1306:128x32` (no D/C pin: 3-wire 9-bit mode).
Pick a sample rate at least ~10× the fastest bit rate; decoding needs every
block, and at the 400 MB/s maximum the analyzer may have to skip some.

TUI keys: `↑↓` select channel, `Enter` pick a role, `a` apply auto-detected
roles, `A` accept the selected channel's suggestion, `u` UART (auto; again to
start from a fixed rate), `i` I2C on the selected channel and the next, `x`
clear role, `p` SPI protocol (raw / SSD1306 128×64 / 128×32), `m` SPI mode,
`+`/`-` zoom, space pause, `r` reset stats, `q` quit.
