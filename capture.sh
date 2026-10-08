#!/bin/sh
# Capture and live-decode the target board with a SLogic16 U3.
#
# Wiring:
#   D0 = 8 MHz clock      D1 = reset
#   D2 = UART (1-wire, 8E1, auto baud: 21.5k -> 2M negotiation is followed)
#   D3 = SPI SCLK   D4 = SPI D/C   D5 = SPI CS   D6 = SPI MOSI   (SSD1306 OLED)
#
# 8 channels at 200 MHz (200 MB/s, a rate with no USB losses in testing):
# 25 samples per 8 MHz clock cycle, 100 per bit at 2 Mbaud, 10 per SPI
# half-period at 10 MHz. Recorded to a compressed .vgk (replay with
# `visgrok -i FILE`, convert for PulseView with `-i FILE -o FILE.sr --headless`).
#
# Environment overrides:
#   RATE=100M            sample rate
#   OUT=file.vgk         output file (OUT= to not record)
#   THRESHOLD=1.65       input threshold in volts (device default is ~2.0 V)
#   DURATION=10          stop after N seconds
# Extra arguments are passed to visgrok, e.g. `./capture.sh --headless`.
set -eu
cd "$(dirname "$0")"

RATE=${RATE:-200M}
OUT=${OUT-capture-$(date +%Y%m%d-%H%M%S).vgk}

cargo build --release -q -p visgrok-tui

set -- -c 8 -s "$RATE" \
    -n 0=CLK8M -n 1=RESET -n 2=UART -n 3=SCLK -n 4=DC -n 5=CS -n 6=MOSI \
    -r 2=uart --uart-format 8E1 \
    -r 3=spi-clk -r 4=spi-dc -r 5=spi-cs -r 6=spi-mosi --spi-proto ssd1306 \
    "$@"
[ -n "$OUT" ] && set -- "$@" -o "$OUT"
[ -n "${THRESHOLD:-}" ] && set -- "$@" -t "$THRESHOLD"
[ -n "${DURATION:-}" ] && set -- "$@" -d "$DURATION"

exec ./target/release/visgrok "$@"
