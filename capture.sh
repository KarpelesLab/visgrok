#!/bin/sh
# Capture and live-decode the target board with a SLogic16 U3.
#
# Wiring:
#   D0 = SE clock (8 MHz)   D1 = SE reset
#   D2 = SE I/O: the SE <-> MCU ISO 7816 link (8E2, 372 clocks per bit until
#        the PPS selects a faster rate), decoded as Ledger SEPROXYHAL packets
#        with APDUs reassembled; characters are timed from the clock on D0
#        and each reset release on D1 starts a new session
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
# Afterwards: `visgrok log FILE -o FILE.log` for the timeline and every
# message (the roles and protocols are saved in the capture's sidecar).
set -eu
cd "$(dirname "$0")"

RATE=${RATE:-200M}
OUT=${OUT-capture-$(date +%Y%m%d-%H%M%S).vgk}

cargo build --release -q

set -- -c 8 -s "$RATE" \
    -n 0=CLK8M -n 1=RESET -n 2=UART -n 3=SCLK -n 4=DC -n 5=CS -n 6=MOSI \
    -r 0=iso-clk -r 1=iso-rst -r 2=iso-io --uart-proto seproxyhal \
    -r 3=spi-clk -r 4=spi-dc -r 5=spi-cs -r 6=spi-mosi --spi-proto ssd1306 \
    "$@"
[ -n "$OUT" ] && set -- "$@" -o "$OUT"
[ -n "${THRESHOLD:-}" ] && set -- "$@" -t "$THRESHOLD"
[ -n "${DURATION:-}" ] && set -- "$@" -d "$DURATION"

exec ./target/release/visgrok "$@"
