#!/bin/sh
# Capture and live-decode a Ledger Nano S with a SLogic16 U3.
#
# https://github.com/KarpelesLab/ledger_nano_s_re
# 0 = CLK   = TP 30   SE clock (~12 MHz)
# 1 = UART  = TP 32   SE <-> MCU I/O (ISO 7816, then SEPROXYHAL at 3 Mbaud)
# 2 = RESET = TP 29   SE reset (noisy near the default ~2.0 V threshold;
#                     pulses under 400 clock cycles are ignored as glitches)
#
# The Nano S MCU draws the screen from BAGL elements the SE sends over the
# SEPROXYHAL link: the reconstructed screen shows live (TUI panel / web
# viewer), and after recording, every distinct screen is written as a PNG
# to FILE-screens/.
#
# View only (nothing written to disk: no capture, sidecar or screens):
#   ./capture_nanos.sh --view [visgrok args...]
#
# Environment overrides:
#   RATE=100M            sample rate
#   OUT=file.vgk         output file (OUT= to not record, same as --view)
#   THRESHOLD=1.0        input threshold in volts
#   DURATION=10          stop after N seconds
#   SCALE=4              PNG pixel scale for the saved screens
# Extra arguments are passed to visgrok, e.g. `./capture_nanos.sh --headless`.

set -eu
cd "$(dirname "$0")"

RATE=${RATE:-200M}
OUT=${OUT-capture-nanos-$(date +%Y%m%d-%H%M%S).vgk}
if [ "${1:-}" = --view ]; then
    shift
    OUT=
fi

echo -n "Building (if needed)..."
cargo build --release -q
echo -ne "\r"

set -- -c 8 -s "$RATE" \
    -n 0=CLK -n 1=UART -n 2=RESET \
    -r 0=iso-clk -r 1=iso-io -r 2=iso-rst --uart-proto seproxyhal \
    "$@"
[ -n "$OUT" ] && set -- "$@" -o "$OUT"
[ -n "${THRESHOLD:-}" ] && set -- "$@" -t "$THRESHOLD"
[ -n "${DURATION:-}" ] && set -- "$@" -d "$DURATION"

# The release build (cargo's target directory may be configured elsewhere).
visgrok() { cargo run --release -q -- "$@"; }

visgrok "$@" || true

if [ -n "$OUT" ] && [ -f "$OUT" ]; then
    visgrok screens "$OUT" -o "${OUT%.vgk}-screens" --scale "${SCALE:-4}" ||
        echo "no screens saved"
fi
