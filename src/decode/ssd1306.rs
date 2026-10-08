//! SSD1306 (and compatible) OLED controller decoder, on top of SPI.
//!
//! Bytes with D/C low are commands (and their arguments), bytes with D/C
//! high are display RAM writes. Without a D/C line, 3-wire SPI is assumed:
//! 9-bit words whose first bit is D/C. The decoder follows the RAM address
//! pointer through page, horizontal and vertical addressing modes and keeps
//! a framebuffer of what the panel shows.

use super::spi::{Spi, SpiConfig};
use super::{Annotation, Decoder, DisplayView, Event};
use crate::edges::Transition;

const PROTO: &str = "SSD1306";

/// Streaming SSD1306 decoder.
pub struct Ssd1306 {
    spi: Spi,
    three_wire: bool,
    width: usize,
    height: usize,
    ram: Vec<u8>,
    // Address pointer state.
    mode: u8,
    col: usize,
    page: usize,
    col_start: usize,
    col_end: usize,
    page_start: usize,
    page_end: usize,
    // Command parser state.
    cmd: Option<(u8, u64)>,
    args: Vec<u8>,
    // Display state.
    on: bool,
    inverted: bool,
    seg_remap: bool,
    com_flip: bool,
    start_line: usize,
    contrast: u8,
    // Current run of data bytes: (start sample, count, page, col).
    run: Option<(u64, u64, usize, usize, u64)>,
    updates: u64,
    scratch: Vec<Annotation>,
}

impl Ssd1306 {
    /// Creates a decoder for a `width`×`height` panel (128×64 or 128×32).
    /// When `spi.dc` is `None`, 3-wire (9-bit) SPI is used.
    pub fn new(mut spi: SpiConfig, width: usize, height: usize) -> Ssd1306 {
        let three_wire = spi.dc.is_none();
        if three_wire {
            spi.word_bits = 9;
        }
        let pages = height.div_ceil(8);
        Ssd1306 {
            spi: Spi::new(spi),
            three_wire,
            width,
            height,
            ram: vec![0; width * pages],
            mode: 2,
            col: 0,
            page: 0,
            col_start: 0,
            col_end: width - 1,
            page_start: 0,
            page_end: pages - 1,
            cmd: None,
            args: Vec::new(),
            on: false,
            inverted: false,
            seg_remap: false,
            com_flip: false,
            start_line: 0,
            contrast: 0x7f,
            run: None,
            updates: 0,
            scratch: Vec::new(),
        }
    }

    fn pages(&self) -> usize {
        self.height.div_ceil(8)
    }

    fn note(out: &mut Vec<Annotation>, start: u64, end: u64, text: String) {
        out.push(Annotation {
            start,
            end,
            event: Event::Protocol { proto: PROTO, text },
        });
    }

    fn flush_run(&mut self, out: &mut Vec<Annotation>) {
        if let Some((start, n, page, col, end)) = self.run.take() {
            self.updates += 1;
            let mode = ["horizontal", "vertical", "page"][self.mode.min(2) as usize];
            Self::note(
                out,
                start,
                end,
                format!("write {n} bytes at page {page} col {col} ({mode} addressing)"),
            );
        }
    }

    fn data(&mut self, byte: u8, start: u64, end: u64, out: &mut Vec<Annotation>) {
        // A long pause also ends a run (separate screen updates).
        if let Some(r) = &self.run
            && start.saturating_sub(r.4) > 64 * (r.4 - r.0) / r.1.max(1)
        {
            self.flush_run(out);
        }
        match &mut self.run {
            Some(r) => {
                r.1 += 1;
                r.4 = end;
            }
            None => self.run = Some((start, 1, self.page, self.col, end)),
        }
        if self.page < self.pages() && self.col < self.width {
            self.ram[self.page * self.width + self.col] = byte;
        }
        match self.mode {
            0 => {
                self.col += 1;
                if self.col > self.col_end {
                    self.col = self.col_start;
                    self.page += 1;
                    if self.page > self.page_end {
                        self.page = self.page_start;
                    }
                }
            }
            1 => {
                self.page += 1;
                if self.page > self.page_end {
                    self.page = self.page_start;
                    self.col += 1;
                    if self.col > self.col_end {
                        self.col = self.col_start;
                    }
                }
            }
            _ => {
                self.col += 1;
                if self.col >= self.width {
                    self.col = self.col_start;
                }
            }
        }
    }

    /// Finishes any pending write run (e.g. at the end of a capture).
    pub fn flush(&mut self, out: &mut Vec<Annotation>) {
        self.flush_run(out);
    }

    /// Number of argument bytes that follow a command byte.
    fn arg_count(cmd: u8) -> usize {
        match cmd {
            0x81 | 0x8d | 0x20 | 0xa8 | 0xd3 | 0xd5 | 0xd9 | 0xda | 0xdb | 0xd6 => 1,
            0x21 | 0x22 | 0xa3 => 2,
            0x29 | 0x2a => 5,
            0x26 | 0x27 => 6,
            _ => 0,
        }
    }

    fn command(&mut self, c: u8, a: &[u8]) -> String {
        let pages = self.pages();
        match c {
            0x00..=0x0f => {
                self.col = (self.col & 0xf0) | c as usize;
                self.col_start = self.col;
                format!("set column low nibble → col {}", self.col)
            }
            0x10..=0x1f => {
                self.col = (self.col & 0x0f) | ((c as usize & 0x0f) << 4);
                self.col_start = self.col;
                format!("set column high nibble → col {}", self.col)
            }
            0x20 => {
                self.mode = a[0] & 3;
                let m = ["horizontal", "vertical", "page", "invalid"][self.mode as usize];
                format!("memory addressing mode: {m}")
            }
            0x21 => {
                self.col_start = a[0] as usize & 0x7f;
                self.col_end = (a[1] as usize & 0x7f).min(self.width - 1);
                self.col = self.col_start;
                format!("column address {}..{}", self.col_start, self.col_end)
            }
            0x22 => {
                self.page_start = a[0] as usize & 7;
                self.page_end = (a[1] as usize & 7).min(pages - 1);
                self.page = self.page_start;
                format!("page address {}..{}", self.page_start, self.page_end)
            }
            0x26 | 0x27 => format!(
                "{} horizontal scroll pages {}..{} interval {}",
                if c == 0x26 { "right" } else { "left" },
                a[1] & 7,
                a[3] & 7,
                a[2] & 7
            ),
            0x29 | 0x2a => format!("vertical+horizontal scroll, offset {}", a[4] & 0x3f),
            0x2e => "deactivate scroll".into(),
            0x2f => "activate scroll".into(),
            0x40..=0x7f => {
                self.start_line = (c & 0x3f) as usize;
                format!("display start line {}", self.start_line)
            }
            0x81 => {
                self.contrast = a[0];
                format!("contrast {}", a[0])
            }
            0x8d => format!("charge pump {}", if a[0] & 4 != 0 { "on" } else { "off" }),
            0xa0 | 0xa1 => {
                self.seg_remap = c == 0xa1;
                format!("segment remap {}", if self.seg_remap { "on (mirrored)" } else { "off" })
            }
            0xa3 => format!("vertical scroll area {} rows from {}", a[1], a[0]),
            0xa4 => "display follows RAM".into(),
            0xa5 => "entire display on".into(),
            0xa6 | 0xa7 => {
                self.inverted = c == 0xa7;
                (if self.inverted { "inverse display" } else { "normal display" }).into()
            }
            0xa8 => format!("multiplex ratio {}", a[0] as u32 + 1),
            0xae | 0xaf => {
                self.on = c == 0xaf;
                format!("display {}", if self.on { "ON" } else { "OFF" })
            }
            0xb0..=0xb7 => {
                self.page = (c & 7) as usize;
                format!("page start {}", self.page)
            }
            0xc0 | 0xc8 => {
                self.com_flip = c == 0xc8;
                format!("COM scan {}", if self.com_flip { "remapped (flipped)" } else { "normal" })
            }
            0xd3 => format!("display offset {}", a[0] & 0x3f),
            0xd5 => format!("clock divide {} / oscillator {}", (a[0] & 0xf) + 1, a[0] >> 4),
            0xd6 => format!("zoom {}", if a[0] & 1 != 0 { "on" } else { "off" }),
            0xd9 => format!("pre-charge phase1 {} phase2 {}", a[0] & 0xf, a[0] >> 4),
            0xda => format!("COM pins config {:#04x}", a[0]),
            0xdb => format!("VCOMH deselect level {:#04x}", a[0]),
            0xe3 => "NOP".into(),
            0xd8 => "vendor command 0xd8 (area color / low power mode on some controllers)".into(),
            _ => format!("unknown command {c:#04x}"),
        }
    }

    fn byte(&mut self, b: u8, dc: bool, start: u64, end: u64, out: &mut Vec<Annotation>) {
        if dc {
            if self.cmd.is_some() {
                Self::note(out, start, end, "data byte while command arguments were expected".into());
                self.cmd = None;
            }
            self.data(b, start, end, out);
            return;
        }
        self.flush_run(out);
        match self.cmd {
            None => {
                self.args.clear();
                if Self::arg_count(b) == 0 {
                    let text = self.command(b, &[]);
                    Self::note(out, start, end, format!("{b:02x}: {text}"));
                } else {
                    self.cmd = Some((b, start));
                }
            }
            Some((c, s)) => {
                self.args.push(b);
                if self.args.len() == Self::arg_count(c) {
                    let args = std::mem::take(&mut self.args);
                    let text = self.command(c, &args);
                    let hex: Vec<String> = args.iter().map(|x| format!("{x:02x}")).collect();
                    Self::note(out, s, end, format!("{c:02x} {}: {text}", hex.join(" ")));
                    self.cmd = None;
                }
            }
        }
    }
}

impl Decoder for Ssd1306 {
    fn name(&self) -> String {
        format!("{PROTO} {}x{} ({})", self.width, self.height, self.spi.name())
    }

    fn channels(&self) -> u32 {
        self.spi.channels()
    }

    fn init(&mut self, state: u32) {
        self.spi.init(state);
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        self.scratch.clear();
        self.spi.transition(t, &mut self.scratch);
        let events = std::mem::take(&mut self.scratch);
        for a in &events {
            match a.event {
                Event::SpiWord { mosi: Some(w), dc, .. } => {
                    let (dc, b) = if self.three_wire {
                        (w >> 8 & 1 != 0, w as u8)
                    } else {
                        (dc.unwrap_or(true), w as u8)
                    };
                    self.byte(b, dc, a.start, a.end, out);
                }
                // Many hosts toggle CS around every byte: keep the write run
                // going; commands (and address jumps) end it.
                Event::SpiSelect(false) => {}
                _ => {}
            }
        }
        self.scratch = events;
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
        // End a write run once the bus has been quiet for a while.
        if let Some(r) = &self.run
            && to.saturating_sub(r.4) > 64 * (r.4 - r.0) / r.1.max(1) + 1000
        {
            self.flush_run(out);
        }
    }

    fn display(&self) -> Option<DisplayView> {
        let pages = self.pages();
        let mut pixels = vec![false; self.width * self.height];
        for y in 0..self.height {
            // Segment/COM remaps (A0/A1, C0/C8) only compensate for how the
            // glass is mounted: hosts draw RAM upright for their panel, so
            // RAM is shown as is (with the start line and inversion).
            let ry = (y + self.start_line) % (pages * 8);
            let sy = y;
            for x in 0..self.width {
                let rx = x;
                let on = self.ram[(ry / 8) * self.width + rx] >> (ry % 8) & 1 != 0;
                pixels[sy * self.width + x] = on ^ self.inverted;
            }
        }
        Some(DisplayView {
            title: PROTO,
            width: self.width,
            height: self.height,
            pixels,
            on: self.on,
            updates: self.updates,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::spi::tests::Bus;
    use super::*;

    #[test]
    fn init_sequence_and_framebuffer() {
        let mut bus = Bus::new(false);
        // Typical init: display off, clock, mux, horizontal mode, remaps, columns, pages, on.
        bus.send(
            &[
                0xae, 0xd5, 0x80, 0xa8, 0x3f, 0x20, 0x00, 0xa1, 0xc8, 0x21, 0x00, 0x7f, 0x22, 0x00, 0x07, 0xaf,
            ],
            false,
            true,
        );
        // Fill: first column all on, then 127 zero columns, then page 1 starts with 0x01.
        let mut frame = vec![0u8; 128 * 8];
        frame[0] = 0xff;
        frame[128] = 0x01;
        bus.send(&frame, true, true);
        let mut cfg = SpiConfig::new(0, Some(1), None, Some(2));
        cfg.dc = Some(3);
        let mut d = Ssd1306::new(cfg, 128, 64);
        d.init(0b100);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        d.advance(bus.at + 1_000_000, &mut out); // end of capture ends the run
        let texts: Vec<String> = out
            .iter()
            .filter_map(|a| match &a.event {
                Event::Protocol { text, .. } => Some(text.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(texts[0], "ae: display OFF");
        assert!(texts.iter().any(|t| t == "20 00: memory addressing mode: horizontal"), "{texts:?}");
        assert!(texts.iter().any(|t| t == "af: display ON"));
        assert_eq!(texts.last().unwrap(), "write 1024 bytes at page 0 col 0 (horizontal addressing)");
        let v = d.display().unwrap();
        assert!(v.on);
        assert!((0..8).all(|y| v.pixels[y * 128]));
        assert!(!v.pixels[1]);
        assert!(v.pixels[8 * 128]); // page 1 bit 0 is row 8 col 0
        assert!(!v.pixels[9 * 128]);
    }
}
