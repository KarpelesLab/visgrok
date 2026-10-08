//! SPI decoder, with optional data/command (D/C) line.

use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

/// SPI bus configuration.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct SpiConfig {
    /// Clock channel.
    pub clk: u8,
    /// Controller-out channel.
    pub mosi: Option<u8>,
    /// Controller-in channel.
    pub miso: Option<u8>,
    /// Chip-select channel.
    pub cs: Option<u8>,
    /// Data/command channel (e.g. SSD1306 "D/C#": high = data). Sampled
    /// with the last bit of each word.
    pub dc: Option<u8>,
    /// Chip select is active high.
    pub cs_active_high: bool,
    /// SPI mode 0..=3 (bit 1 = CPOL, bit 0 = CPHA). `None`: CPOL from the
    /// clock's idle level, CPHA = 0.
    pub mode: Option<u8>,
    /// Bits per word.
    pub word_bits: u8,
    /// Least significant bit first.
    pub lsb_first: bool,
}

impl SpiConfig {
    /// Automatic mode, 8-bit MSB-first words.
    pub fn new(clk: u8, mosi: Option<u8>, miso: Option<u8>, cs: Option<u8>) -> SpiConfig {
        SpiConfig {
            clk,
            mosi,
            miso,
            cs,
            dc: None,
            cs_active_high: false,
            mode: None,
            word_bits: 8,
            lsb_first: false,
        }
    }
}

/// Streaming SPI decoder.
pub struct Spi {
    cfg: SpiConfig,
    cpol: bool,
    clk: bool,
    selected: bool,
    mosi: u32,
    miso: u32,
    nbits: u8,
    word_start: u64,
    /// Last clock edge and the shortest clock half-period seen, used to
    /// resynchronize words on clock pauses when there is no chip select.
    last_clk: Option<u64>,
    min_half: u64,
}

impl Spi {
    /// Creates a decoder.
    pub fn new(cfg: SpiConfig) -> Spi {
        let cpol = cfg.mode.is_some_and(|m| m & 2 != 0);
        Spi {
            cfg,
            cpol,
            clk: cpol,
            selected: true,
            mosi: 0,
            miso: 0,
            nbits: 0,
            word_start: 0,
            last_clk: None,
            min_half: u64::MAX,
        }
    }

    /// Effective SPI mode.
    pub fn mode(&self) -> u8 {
        (self.cpol as u8) << 1 | self.cfg.mode.map_or(0, |m| m & 1)
    }

    fn cs_active(&self, state: u32) -> bool {
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

fn bit(state: u32, ch: Option<u8>) -> u32 {
    ch.map_or(0, |c| state >> c & 1)
}

impl Decoder for Spi {
    fn name(&self) -> String {
        let mut s = format!("SPI clk=ch{}", self.cfg.clk);
        if let Some(c) = self.cfg.mosi {
            s += &format!(" mosi=ch{c}");
        }
        if let Some(c) = self.cfg.miso {
            s += &format!(" miso=ch{c}");
        }
        if let Some(c) = self.cfg.cs {
            s += &format!(" cs=ch{c}");
        }
        if let Some(c) = self.cfg.dc {
            s += &format!(" dc=ch{c}");
        }
        s + &format!(" mode {}", self.mode())
    }

    fn channels(&self) -> u32 {
        [Some(self.cfg.clk), self.cfg.mosi, self.cfg.miso, self.cfg.cs, self.cfg.dc]
            .into_iter()
            .flatten()
            .fold(0, |m, c| m | 1 << c)
    }

    fn init(&mut self, state: u32) {
        self.clk = state >> self.cfg.clk & 1 != 0;
        self.selected = self.cs_active(state);
        if self.cfg.mode.is_none() && (self.cfg.cs.is_none() || !self.selected) {
            self.cpol = self.clk;
        }
        self.reset_word();
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let sel = self.cs_active(t.now);
        if sel != self.selected {
            self.selected = sel;
            self.reset_word();
            // Learn the clock polarity from its level when selection starts.
            if sel && self.cfg.mode.is_none() {
                self.cpol = t.now >> self.cfg.clk & 1 != 0;
            }
            out.push(Annotation {
                start: t.at,
                end: t.at,
                event: Event::SpiSelect(sel),
            });
        }
        let clk = t.now >> self.cfg.clk & 1 != 0;
        if clk == self.clk {
            return;
        }
        self.clk = clk;
        if let Some(prev) = self.last_clk {
            let half = t.at - prev;
            // Without CS, a long pause in the clock marks a word boundary.
            if self.cfg.cs.is_none() && self.min_half != u64::MAX && half > 16 * self.min_half {
                if self.nbits != 0 {
                    self.reset_word();
                }
                if self.cfg.mode.is_none() {
                    // The level the clock rested at is its idle level.
                    self.cpol = !clk;
                }
            } else {
                self.min_half = self.min_half.min(half.max(1));
            }
        }
        self.last_clk = Some(t.at);
        if !self.selected {
            return;
        }
        let cpha = self.cfg.mode.is_some_and(|m| m & 1 != 0);
        // Leading edge leaves the idle level; sample on it when CPHA = 0.
        let leading = clk != self.cpol;
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
                    dc: self.cfg.dc.map(|c| t.now >> c & 1 != 0),
                },
            });
            self.reset_word();
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Signal builder: ch0 clk, ch1 mosi, ch2 cs (active low), ch3 dc.
    pub(crate) struct Bus {
        pub st: u32,
        pub at: u64,
        pub tr: Vec<Transition>,
        pub idle_clk: u32,
    }

    impl Bus {
        pub(crate) fn new(cpol: bool) -> Bus {
            let idle_clk = cpol as u32;
            Bus {
                st: 0b100 | idle_clk,
                at: 0,
                tr: Vec::new(),
                idle_clk,
            }
        }
        pub(crate) fn set(&mut self, v: u32) {
            self.at += 10;
            if v != self.st {
                self.tr.push(Transition {
                    at: self.at,
                    prev: self.st,
                    now: v,
                });
                self.st = v;
            }
        }
        /// Sends bytes in mode 0/2 (sample on the leading edge).
        pub(crate) fn send(&mut self, bytes: &[u8], dc: bool, cs: bool) {
            let base = (dc as u32) << 3 | self.idle_clk;
            if cs {
                self.set(base);
            }
            for &b in bytes {
                for k in (0..8).rev() {
                    let d = ((b >> k & 1) as u32) << 1;
                    self.set(base | d);
                    self.set((base | d) ^ 1);
                    self.set(base | d);
                }
                self.at += 30;
            }
            if cs {
                self.set(base | 0b100);
            }
        }
    }

    fn run(d: &mut Spi, bus: &Bus, init: u32) -> Vec<Event> {
        d.init(init);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        out.into_iter().map(|a| a.event).collect()
    }

    #[test]
    fn mode0_with_cs_and_dc() {
        let mut bus = Bus::new(false);
        bus.send(&[0xae, 0x20], false, true);
        bus.send(&[0xa5], true, true);
        let mut cfg = SpiConfig::new(0, Some(1), None, Some(2));
        cfg.dc = Some(3);
        let ev = run(&mut Spi::new(cfg), &bus, 0b100);
        assert_eq!(
            ev,
            vec![
                Event::SpiSelect(true),
                Event::SpiWord {
                    mosi: Some(0xae),
                    miso: None,
                    dc: Some(false)
                },
                Event::SpiWord {
                    mosi: Some(0x20),
                    miso: None,
                    dc: Some(false)
                },
                Event::SpiSelect(false),
                Event::SpiSelect(true),
                Event::SpiWord {
                    mosi: Some(0xa5),
                    miso: None,
                    dc: Some(true)
                },
                Event::SpiSelect(false),
            ]
        );
    }

    #[test]
    fn auto_cpol_without_cs() {
        // Clock idles high (mode 2/3 style; mode 2 samples on the falling edge).
        let mut bus = Bus::new(true);
        bus.send(&[0x3c, 0x81], false, false);
        let cfg = SpiConfig::new(0, Some(1), None, None);
        let mut d = Spi::new(cfg);
        let ev = run(&mut d, &bus, 0b101);
        assert_eq!(d.mode(), 2);
        let words: Vec<_> = ev
            .iter()
            .filter_map(|e| match e {
                Event::SpiWord { mosi, .. } => *mosi,
                _ => None,
            })
            .collect();
        assert_eq!(words, vec![0x3c, 0x81]);
    }
}
