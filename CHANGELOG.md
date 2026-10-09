# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.2](https://github.com/KarpelesLab/visgrok/compare/v0.1.1...v0.1.2) - 2026-10-09

### Other

- authentication handshake, timeline; `visgrok log` command
- Ledger SEPROXYHAL decoder over ISO 7816; ISO 7816 without clock line
- querying a capture from scripts and agents; ignore event caches
- query commands for analyzing a capture (for people and agents)
- always write the activity overview
- ISO 7816: card clock and reset lines, sessions

## [0.1.1](https://github.com/KarpelesLab/visgrok/compare/v0.1.0...v0.1.1) - 2026-10-08

### Other

- Tighten the public API for semver stability
- Event inspector: lines, bits and field meanings for any decoded event
- Web UI: fix event dialog covering the page; relative context times
- Web UI: short event labels, hover details, event inspector
- Web UI: auto-detect roles and protocols when reviewing a capture
- Simplify the widths example
- Robust detection on real-world captures: glitches, mixed rates, noise
- Neutral example names in docs
- Add an ISO 7816-3 (smart card) protocol layer for UART
- Web UI: display state at any moment, cursor readout, Δt measurement
- Better detection and SSD1306 decoding on real-world signals
- Capture sidecar files: channels, buses, decoder settings, bookmarks, notes
- Web control mode: browser UI to record, assign roles and browse captures
