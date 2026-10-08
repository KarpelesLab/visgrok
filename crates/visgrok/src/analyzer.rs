//! Real-time analysis pipeline: blocks in, statistics, role suggestions,
//! decoded annotations and a recent-activity window out.

use std::collections::VecDeque;

use crate::block::Block;
use crate::decode::i2c::I2c;
use crate::decode::spi::{Spi, SpiConfig};
use crate::decode::ssd1306::Ssd1306;
use crate::decode::uart::{Uart, UartConfig};
use crate::decode::{Annotation, Decoder};
use crate::edges::{EdgeDetector, Transition};
use crate::roles::{self, Correlator, Role, Suggestion};
use crate::stats::Stats;

/// How many recent transitions are kept for waveform display.
pub const WAVE_HISTORY: usize = 1 << 16;
/// How many recent annotations are kept.
pub const ANNOTATION_HISTORY: usize = 1 << 16;

/// Higher-level protocol carried over SPI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiProtocol {
    /// Plain SPI words.
    Raw,
    /// SSD1306-style OLED controller (commands and display RAM).
    Ssd1306 {
        /// Panel width in pixels.
        width: usize,
        /// Panel height in pixels.
        height: usize,
    },
}

impl SpiProtocol {
    /// Parses `raw`, `ssd1306`, `ssd1306:128x32`.
    pub fn parse(s: &str) -> Result<SpiProtocol, String> {
        let (name, arg) = s.split_once(':').map_or((s, None), |(n, a)| (n, Some(a)));
        match name.to_ascii_lowercase().as_str() {
            "raw" | "spi" | "none" => Ok(SpiProtocol::Raw),
            "ssd1306" | "sh1106" | "oled" => {
                let (w, h) = match arg {
                    Some(a) => {
                        let (w, h) = a.split_once('x').ok_or("size must look like 128x64")?;
                        (w.parse().map_err(|_| "bad width")?, h.parse().map_err(|_| "bad height")?)
                    }
                    None => (128, 64),
                };
                Ok(SpiProtocol::Ssd1306 { width: w, height: h })
            }
            _ => Err(format!("unknown SPI protocol {name:?} (raw, ssd1306[:WxH])")),
        }
    }
}

/// Settings applied when building decoders from roles.
#[derive(Clone, Debug)]
pub struct DecoderOptions {
    /// SPI mode (0..=3); `None` infers CPOL from the clock idle level.
    pub spi_mode: Option<u8>,
    /// SPI chip select is active high.
    pub spi_cs_active_high: bool,
    /// Protocol layered on SPI.
    pub spi_protocol: SpiProtocol,
    /// UART decoders follow baud rate changes even when a rate is given.
    pub uart_auto: bool,
}

impl Default for DecoderOptions {
    fn default() -> Self {
        DecoderOptions { spi_mode: None, spi_cs_active_high: false, spi_protocol: SpiProtocol::Raw, uart_auto: true }
    }
}

/// An annotation tagged with the index of the decoder that produced it.
#[derive(Clone, Debug)]
pub struct Tagged {
    /// Index into [`Analyzer::decoders`].
    pub decoder: usize,
    /// The annotation.
    pub annotation: Annotation,
}

/// Streaming analysis of a capture.
pub struct Analyzer {
    samplerate: u64,
    channels: usize,
    edges: EdgeDetector,
    scratch: Vec<Transition>,
    stats: Stats,
    corr: Correlator,
    decoders: Vec<Box<dyn Decoder>>,
    ann_scratch: Vec<Annotation>,
    /// Recent annotations, oldest first.
    pub annotations: VecDeque<Tagged>,
    /// Recent transitions, oldest first.
    pub wave: VecDeque<Transition>,
    /// Total number of annotations produced.
    pub annotation_count: u64,
    next: Option<u64>,
    /// Samples skipped because blocks were dropped before analysis.
    pub gaps: u64,
    /// Channels excluded from analysis (still recorded).
    ignored: u16,
}

impl Analyzer {
    /// Creates an analyzer for `channels` channels at `samplerate` Hz.
    pub fn new(channels: usize, samplerate: u64) -> Analyzer {
        let mask = if channels >= 16 { 0xffff } else { (1u16 << channels) - 1 };
        Analyzer {
            samplerate,
            channels,
            edges: EdgeDetector::new(mask),
            scratch: Vec::new(),
            stats: Stats::new(channels),
            corr: Correlator::new(channels),
            decoders: Vec::new(),
            ann_scratch: Vec::new(),
            annotations: VecDeque::new(),
            wave: VecDeque::new(),
            annotation_count: 0,
            next: None,
            gaps: 0,
            ignored: 0,
        }
    }

    fn mask(&self) -> u16 {
        let all = if self.channels >= 16 { 0xffff } else { (1u16 << self.channels) - 1 };
        all & !self.ignored
    }

    /// Excludes channels from analysis. A fast free-running clock produces
    /// an edge every few samples; ignoring it keeps decoding of the other
    /// channels real-time. Ignored channels read as constant 0.
    pub fn set_ignored(&mut self, mask: u16) {
        self.ignored = mask;
        self.next = None; // restart edge tracking with the new mask
    }

    /// Channels excluded from analysis.
    pub fn ignored(&self) -> u16 {
        self.ignored
    }

    /// Sample rate in Hz.
    pub fn samplerate(&self) -> u64 {
        self.samplerate
    }

    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Per-channel statistics.
    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    /// Current line state, once known.
    pub fn state(&self) -> Option<u16> {
        self.edges.state()
    }

    /// Active decoders.
    pub fn decoders(&self) -> &[Box<dyn Decoder>] {
        &self.decoders
    }

    /// Replaces the active decoders.
    pub fn set_decoders(&mut self, mut decoders: Vec<Box<dyn Decoder>>) {
        if let Some(s) = self.edges.state() {
            for d in &mut decoders {
                d.init(s);
            }
        }
        self.decoders = decoders;
        self.annotations.clear();
    }

    /// Builds decoders matching the given per-channel roles.
    pub fn decoders_for_roles(&self, roles: &[Role], opts: &DecoderOptions) -> Vec<Box<dyn Decoder>> {
        let mut out: Vec<Box<dyn Decoder>> = Vec::new();
        let find = |want: &Role| roles.iter().position(|r| r == want).map(|c| c as u8);
        for (i, r) in roles.iter().enumerate() {
            match r {
                Role::Uart { baud } => {
                    let mut cfg = UartConfig::auto(i as u8);
                    cfg.baud = (*baud != 0).then_some(*baud);
                    cfg.auto = opts.uart_auto || *baud == 0;
                    out.push(Box::new(Uart::new(cfg, self.samplerate)))
                }
                Role::I2cScl { sda } => out.push(Box::new(I2c::new(i as u8, *sda))),
                Role::SpiClk => {
                    // Explicit MOSI/MISO roles win; otherwise use auto-detected
                    // data lines on this clock (first one as MOSI).
                    let data: Vec<u8> = roles
                        .iter()
                        .enumerate()
                        .filter(|(_, r)| matches!(r, Role::SpiData { clk } if *clk as usize == i))
                        .map(|(j, _)| j as u8)
                        .collect();
                    let mosi = find(&Role::SpiMosi).or(data.first().copied());
                    let miso = find(&Role::SpiMiso).or(data.get(1).copied());
                    let mut cfg = SpiConfig::new(i as u8, mosi, miso, find(&Role::SpiCs));
                    cfg.dc = find(&Role::SpiDc);
                    cfg.mode = opts.spi_mode;
                    cfg.cs_active_high = opts.spi_cs_active_high;
                    match opts.spi_protocol {
                        SpiProtocol::Raw => out.push(Box::new(Spi::new(cfg))),
                        SpiProtocol::Ssd1306 { width, height } => {
                            out.push(Box::new(Ssd1306::new(cfg, width, height)))
                        }
                    }
                }
                _ => {}
            }
        }
        out
    }

    /// Proposes a role for every channel from what has been seen so far.
    pub fn suggest(&self) -> Vec<Suggestion> {
        roles::detect(&self.stats, &self.corr, self.samplerate)
    }

    /// Forgets statistics (but keeps decoders), e.g. after the signals changed.
    pub fn reset_stats(&mut self) {
        self.stats = Stats::new(self.channels);
        self.corr = Correlator::new(self.channels);
        if let Some(s) = self.edges.state() {
            self.stats.init(s);
            self.corr.init(s);
        }
    }

    /// Processes the next block of samples.
    pub fn process(&mut self, block: &Block) {
        if block.is_empty() {
            return;
        }
        let first = block.sample(0);
        match self.next {
            None => self.restart(first, block.start),
            Some(n) if n != block.start => {
                self.gaps += block.start.saturating_sub(n);
                self.restart(first, block.start);
            }
            _ => {}
        }
        self.next = Some(block.end());

        self.scratch.clear();
        self.edges.process(block, &mut self.scratch);
        self.stats.process(&self.scratch);
        self.stats.advance(block.end());
        self.corr.process(&self.scratch);

        let mut produced: Vec<Tagged> = Vec::new();
        for (di, d) in self.decoders.iter_mut().enumerate() {
            let mask = d.channels();
            self.ann_scratch.clear();
            for t in self.scratch.iter().filter(|t| t.changed() & mask != 0) {
                d.transition(t, &mut self.ann_scratch);
            }
            d.advance(block.end(), &mut self.ann_scratch);
            produced.extend(self.ann_scratch.drain(..).map(|a| Tagged { decoder: di, annotation: a }));
        }
        produced.sort_by_key(|t| t.annotation.start);
        for t in produced {
            self.annotation_count += 1;
            if self.annotations.len() == ANNOTATION_HISTORY {
                self.annotations.pop_front();
            }
            self.annotations.push_back(t);
        }

        let skip = self.scratch.len().saturating_sub(WAVE_HISTORY);
        for t in &self.scratch[skip..] {
            if self.wave.len() == WAVE_HISTORY {
                self.wave.pop_front();
            }
            self.wave.push_back(*t);
        }
    }

    fn restart(&mut self, first: u16, at: u64) {
        let mask = self.mask();
        self.edges = EdgeDetector::new(mask);
        let s = first & mask;
        if !self.stats.is_initialized() {
            self.stats.init(s);
            self.corr.init(s);
        }
        for d in &mut self.decoders {
            d.init(s);
        }
        // Keep the waveform consistent: record the jump as a transition.
        if let Some(last) = self.wave.back()
            && last.now != s
        {
            self.wave.push_back(Transition { at, prev: last.now, now: s });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthesizes 8-channel data: ch0 clock (period 10), ch1 UART 8N1 at
    /// 100 samples/bit, ch2/ch3 I2C (SCL/SDA).
    fn synth() -> Vec<u8> {
        let n = 400_000usize;
        let mut v = vec![0u8; n];
        // ch0 clock
        for (i, s) in v.iter_mut().enumerate() {
            if (i / 5) % 2 == 1 {
                *s |= 1;
            }
        }
        // ch1 UART: idle high, bytes back to back with 3 idle bits
        let mut uart_bits = Vec::new();
        for b in 0u32..300 {
            let byte = (b * 37 + 11) as u8;
            uart_bits.push(false);
            for k in 0..8 {
                uart_bits.push(byte >> k & 1 != 0);
            }
            uart_bits.extend([true; 4]);
        }
        for (i, s) in v.iter_mut().enumerate() {
            if uart_bits.get(i / 100).copied().unwrap_or(true) {
                *s |= 2;
            }
        }
        // ch2 SCL, ch3 SDA: transactions every 5000 samples, 50 samples per phase
        let mut lines = vec![(true, true); n];
        let mut t = 1000;
        let mut val = 0u8;
        while t + 5000 < n {
            let mut seq: Vec<(bool, bool)> = vec![(true, true), (true, false), (false, false)];
            for byte in [0xa0u8, val, val.wrapping_mul(3)] {
                for k in (0..8).rev().map(|k| byte >> k & 1 != 0).chain([false]) {
                    seq.extend([(false, k), (true, k), (false, k)]);
                }
            }
            seq.extend([(false, false), (true, false), (true, true)]);
            for (j, p) in seq.into_iter().enumerate() {
                for x in 0..50 {
                    lines[t + j * 50 + x] = p;
                }
            }
            t += 5000;
            val = val.wrapping_add(1);
        }
        for (s, (scl, sda)) in v.iter_mut().zip(lines) {
            *s |= (scl as u8) << 2 | (sda as u8) << 3;
        }
        v
    }

    #[test]
    fn detects_roles_and_decodes() {
        let samplerate = 11_520_000; // 115200 baud at 100 samples/bit
        let data = synth();
        let mut a = Analyzer::new(8, samplerate);
        for (i, c) in data.chunks(65536).enumerate() {
            a.process(&Block::new(i as u64 * 65536, 1, c.to_vec()));
        }
        let s = a.suggest();
        assert!(matches!(s[0].role, Role::Clock { hz } if (hz - 1_152_000.0).abs() < 1.0), "{:?}", s[0]);
        assert_eq!(s[1].role, Role::Uart { baud: 115200 });
        assert_eq!(s[2].role, Role::I2cScl { sda: 3 });
        assert_eq!(s[3].role, Role::I2cSda { scl: 2 });
        assert_eq!(s[4].role, Role::Idle);

        let roles: Vec<Role> = s.into_iter().map(|s| s.role).collect();
        let decs = a.decoders_for_roles(&roles, &DecoderOptions::default());
        assert_eq!(decs.len(), 2);
        let mut a2 = Analyzer::new(8, samplerate);
        a2.set_decoders(decs);
        for (i, c) in data.chunks(65536).enumerate() {
            a2.process(&Block::new(i as u64 * 65536, 1, c.to_vec()));
        }
        let uart: Vec<_> = a2.annotations.iter().filter(|t| t.decoder == 0).collect();
        assert_eq!(uart.len(), 300);
        let i2c_addr = a2
            .annotations
            .iter()
            .filter(|t| matches!(t.annotation.event, crate::decode::Event::I2cAddress { addr: 0x50, .. }))
            .count();
        assert!(i2c_addr >= 70, "{i2c_addr}");
    }
}

#[cfg(test)]
mod device_tests {
    use super::*;
    use crate::decode::Event;
    use crate::source::Source;
    use crate::synth::Synth;

    #[test]
    fn uart_negotiation_and_oled() {
        let sr = 50_000_000;
        let mut src = Synth::device(sr, Some(sr / 4)); // 250 ms
        let mut a = Analyzer::new(8, sr);
        let roles = vec![
            Role::Uart { baud: 0 },
            Role::SpiClk,
            Role::SpiMosi,
            Role::SpiDc,
            Role::SpiCs,
            Role::Unknown,
            Role::Unknown,
            Role::Unknown,
        ];
        let opts = DecoderOptions { spi_protocol: SpiProtocol::Ssd1306 { width: 128, height: 64 }, ..Default::default() };
        let mut text = String::new();
        let mut bauds = Vec::new();
        let mut oled = Vec::new();
        let mut started = false;
        while let Some(b) = src.next_block().unwrap() {
            if !started {
                let d = a.decoders_for_roles(&roles, &opts);
                a.set_decoders(d);
                started = true;
            }
            let before = a.annotation_count;
            a.process(&b);
            let new = (a.annotation_count - before) as usize;
            for t in a.annotations.iter().skip(a.annotations.len() - new.min(a.annotations.len())) {
                match &t.annotation.event {
                    Event::UartByte { value, framing_error: false, .. } => text.push(*value as u8 as char),
                    Event::UartBaud { baud } => bauds.push(*baud),
                    Event::Protocol { text, .. } => oled.push(text.clone()),
                    _ => {}
                }
            }
        }
        assert!(text.starts_with("AT+BAUD=2000000\r\nOK\r\nfast packet 1.0: the quick brown fox"), "{text:?}");
        assert!(text.contains("fast packet 1.19: the quick brown fox jumps over the lazy dog\r\n"), "{text:?}");
        assert_eq!(&bauds[..3], &[21_500, 2_000_000, 21_500], "{bauds:?}");
        assert_eq!(oled[0], "ae: display OFF");
        assert!(oled.iter().any(|t| t == "af: display ON"));
        assert!(oled.iter().filter(|t| t.starts_with("write 1024 bytes at page 0 col 0")).count() >= 5, "{oled:?}");
        let d = a.decoders()[1].display().unwrap();
        assert!(d.on);
        // Border pixels are lit; with A1/C8 (mirrored) the border is still a border.
        assert!(d.pixels[0] && d.pixels[127] && d.pixels[63 * 128]);
        assert!(!d.pixels[32 * 128 + 64] || d.pixels.iter().filter(|&&p| p).count() > 300);
    }
}
