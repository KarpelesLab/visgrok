//! SPI decoder.

use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

/// SPI bus configuration.
#[derive(Clone, Debug)]
pub struct SpiConfig {
    /// Clock channel.
    pub clk: u8,
    /// Controller-out channel.
    pub mosi: Option<u8>,
    /// Controller-in channel.
    pub miso: Option<u8>,
    /// Chip-select channel.
    pub cs: Option<u8>,
    /// Chip select is active high.
    pub cs_active_high: bool,
    /// SPI mode 0..=3 (bit 1 = CPOL, bit 0 = CPHA).
    pub mode: u8,
    /// Bits per word.
    pub word_bits: u8,
    /// Least significant bit first.
    pub lsb_first: bool,
}

impl SpiConfig {
    /// Mode 0, 8-bit MSB-first words.
    pub fn new(clk: u8, mosi: Option<u8>, miso: Option<u8>, cs: Option<u8>) -> SpiConfig {
        SpiConfig { clk, mosi, miso, cs, cs_active_high: false, mode: 0, word_bits: 8, lsb_first: false }
    }
}

/// Streaming SPI decoder.
pub struct Spi {
    cfg: SpiConfig,
    clk: bool,
    selected: bool,
    mosi: u32,
    miso: u32,
    nbits: u8,
    word_start: u64,
}

impl Spi {
    /// Creates a decoder.
    pub fn new(cfg: SpiConfig) -> Spi {
        Spi { cfg, clk: false, selected: true, mosi: 0, miso: 0, nbits: 0, word_start: 0 }
    }

    fn cs_active(&self, state: u16) -> bool {
        match self.cfg.cs {
            Some(c) => (state >> c & 1 != 0) == self.cfg.cs_active_high,
            None => true,
        }
    }

    fn reset_word(&mut self) {
        self.mosi = 0;
        self.miso = 0;
        self.nbits = 0;
    }
}

fn bit(state: u16, ch: Option<u8>) -> u32 {
    ch.map_or(0, |c| (state >> c & 1) as u32)
}

impl Decoder for Spi {
    fn name(&self) -> String {
        format!("SPI clk=ch{} mode {}", self.cfg.clk, self.cfg.mode)
    }

    fn channels(&self) -> u16 {
        [Some(self.cfg.clk), self.cfg.mosi, self.cfg.miso, self.cfg.cs]
            .into_iter()
            .flatten()
            .fold(0, |m, c| m | 1 << c)
    }

    fn init(&mut self, state: u16) {
        self.clk = state >> self.cfg.clk & 1 != 0;
        self.selected = self.cs_active(state);
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let sel = self.cs_active(t.now);
        if sel != self.selected {
            self.selected = sel;
            self.reset_word();
            out.push(Annotation { start: t.at, end: t.at, event: Event::SpiSelect(sel) });
        }
        let clk = t.now >> self.cfg.clk & 1 != 0;
        if clk == self.clk {
            return;
        }
        self.clk = clk;
        if !self.selected {
            return;
        }
        let cpol = self.cfg.mode & 2 != 0;
        let cpha = self.cfg.mode & 1 != 0;
        // Leading edge leaves the idle level; sample on leading edge when CPHA=0.
        let leading = clk != cpol;
        if leading == cpha {
            return;
        }
        if self.nbits == 0 {
            self.word_start = t.at;
        }
        let (mo, mi) = (bit(t.now, self.cfg.mosi), bit(t.now, self.cfg.miso));
        if self.cfg.lsb_first {
            self.mosi |= mo << self.nbits;
            self.miso |= mi << self.nbits;
        } else {
            self.mosi = self.mosi << 1 | mo;
            self.miso = self.miso << 1 | mi;
        }
        self.nbits += 1;
        if self.nbits == self.cfg.word_bits {
            out.push(Annotation {
                start: self.word_start,
                end: t.at,
                event: Event::SpiWord {
                    mosi: self.cfg.mosi.map(|_| self.mosi),
                    miso: self.cfg.miso.map(|_| self.miso),
                },
            });
            self.reset_word();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode0_word() {
        // ch0 clk, ch1 mosi, ch2 cs (active low)
        let mut d = Spi::new(SpiConfig::new(0, Some(1), None, Some(2)));
        d.init(0b100);
        let mut out = Vec::new();
        let mut st = 0b100u16;
        let mut at = 0;
        let mut push = |v: u16, out: &mut Vec<Annotation>, d: &mut Spi| {
            at += 1;
            if v != st {
                d.transition(&Transition { at, prev: st, now: v }, out);
                st = v;
            }
        };
        push(0b000, &mut out, &mut d);
        for k in (0..8).rev() {
            let b = ((0xa5u16 >> k) & 1) << 1;
            push(b, &mut out, &mut d);
            push(b | 1, &mut out, &mut d);
            push(b, &mut out, &mut d);
        }
        push(0b100, &mut out, &mut d);
        let ev: Vec<Event> = out.into_iter().map(|a| a.event).collect();
        assert_eq!(
            ev,
            vec![
                Event::SpiSelect(true),
                Event::SpiWord { mosi: Some(0xa5), miso: None },
                Event::SpiSelect(false)
            ]
        );
    }
}
