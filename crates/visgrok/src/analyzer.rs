//! Real-time analysis pipeline: blocks in, statistics, role suggestions,
//! decoded annotations and a recent-activity window out.

use std::collections::VecDeque;

use crate::block::Block;
use crate::decode::i2c::I2c;
use crate::decode::spi::{Spi, SpiConfig};
use crate::decode::uart::{UartConfig, UartDecoder};
use crate::decode::{Annotation, Decoder};
use crate::edges::{EdgeDetector, Transition};
use crate::roles::{self, Correlator, Role, Suggestion};
use crate::stats::Stats;

/// How many recent transitions are kept for waveform display.
pub const WAVE_HISTORY: usize = 1 << 16;
/// How many recent annotations are kept.
pub const ANNOTATION_HISTORY: usize = 4096;

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
        }
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
    pub fn decoders_for_roles(&self, roles: &[Role]) -> Vec<Box<dyn Decoder>> {
        let mut out: Vec<Box<dyn Decoder>> = Vec::new();
        for (i, r) in roles.iter().enumerate() {
            match r {
                Role::Uart { baud } => {
                    out.push(Box::new(UartDecoder::new(UartConfig::new(i as u8, *baud), self.samplerate)))
                }
                Role::I2cScl { sda } => out.push(Box::new(I2c::new(i as u8, *sda))),
                Role::SpiClk => {
                    let data: Vec<u8> = roles
                        .iter()
                        .enumerate()
                        .filter(|(_, r)| matches!(r, Role::SpiData { clk } if *clk as usize == i))
                        .map(|(j, _)| j as u8)
                        .collect();
                    let cs = roles.iter().position(|r| *r == Role::SpiCs).map(|c| c as u8);
                    // Without knowing direction, call the first data line MOSI.
                    out.push(Box::new(Spi::new(SpiConfig::new(i as u8, data.first().copied(), data.get(1).copied(), cs))));
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

        self.ann_scratch.clear();
        for (di, d) in self.decoders.iter_mut().enumerate() {
            let mask = d.channels();
            for t in self.scratch.iter().filter(|t| t.changed() & mask != 0) {
                d.transition(t, &mut self.ann_scratch);
            }
            d.advance(block.end(), &mut self.ann_scratch);
            for a in self.ann_scratch.drain(..) {
                self.annotation_count += 1;
                if self.annotations.len() == ANNOTATION_HISTORY {
                    self.annotations.pop_front();
                }
                self.annotations.push_back(Tagged { decoder: di, annotation: a });
            }
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
        let mask = if self.channels >= 16 { 0xffff } else { (1u16 << self.channels) - 1 };
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
        let decs = a.decoders_for_roles(&roles);
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
