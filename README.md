# visgrok

A from-scratch Rust driver for Sipeed SLogic logic analyzers (SLogic16 U3,
SLogic Combo 8, SLogic Lite 8), with continuous streaming to disk, real-time
signal analysis and a terminal UI.

- `crates/visgrok` — library: USB protocol (on top of [`rawusb`](https://github.com/KarpelesLab/rawusb)), streaming, analysis, file output.
- `crates/visgrok-tui` — `visgrok` binary: live TUI while recording.

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the USB protocol and
[docs/ROADMAP.md](docs/ROADMAP.md) for the plan.
