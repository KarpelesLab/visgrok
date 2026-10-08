//! Asynchronous serial (UART) decoder.

use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

/// Parity mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Parity {
    /// No parity bit.
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// UART line settings.
#[derive(Clone, Debug)]
pub struct UartConfig {
    /// Channel number.
    pub channel: u8,
    /// Bits per second.
    pub baud: u32,
    /// Data bits (5..=9).
    pub data_bits: u8,
    /// Parity.
    pub parity: Parity,
    /// Inverted line (idle low).
    pub inverted: bool,
}

impl UartConfig {
    /// 8N1 on `channel` at `baud`.
    pub fn new(channel: u8, baud: u32) -> UartConfig {
        UartConfig { channel, baud, data_bits: 8, parity: Parity::None, inverted: false }
    }
}

/// Streaming UART decoder.
struct Uart {
    cfg: UartConfig,
    bit: f64,
    level: bool,
    /// Start of the frame being received.
    frame: Option<u64>,
}

impl Uart {
    /// Creates a decoder for a stream sampled at `samplerate` Hz.
    fn new(cfg: UartConfig, samplerate: u64) -> Uart {
        let bit = samplerate as f64 / cfg.baud as f64;
        Uart { cfg, bit, level: true, frame: None }
    }

    fn frame_bits(&self) -> u32 {
        1 + self.cfg.data_bits as u32 + (self.cfg.parity != Parity::None) as u32 + 1
    }

    /// Time of the middle of bit `k` of a frame starting at `start`.
    fn mid(&self, start: u64, k: u32) -> u64 {
        start + ((k as f64 + 0.5) * self.bit) as u64
    }

    /// Completes the current frame if its stop bit is before `before`; `history`
    /// gives the line level at any sample in the frame.
    fn finish(&mut self, before: u64, history: &[(u64, bool)], out: &mut Vec<Annotation>) {
        let Some(start) = self.frame else { return };
        let nbits = self.frame_bits();
        let stop_mid = self.mid(start, nbits - 1);
        if stop_mid >= before {
            return;
        }
        let level_at = |x: u64| -> bool {
            let mut l = false; // start bit is low
            for &(at, v) in history {
                if at <= x {
                    l = v;
                } else {
                    break;
                }
            }
            l
        };
        let mut value = 0u16;
        let mut ones = 0;
        for k in 0..self.cfg.data_bits as u32 {
            if level_at(self.mid(start, 1 + k)) {
                value |= 1 << k;
                ones += 1;
            }
        }
        let mut parity_error = false;
        if self.cfg.parity != Parity::None {
            let p = level_at(self.mid(start, 1 + self.cfg.data_bits as u32)) as u32;
            let total = ones + p;
            parity_error = match self.cfg.parity {
                Parity::Even => !total.is_multiple_of(2),
                _ => total.is_multiple_of(2),
            };
        }
        let stop = level_at(stop_mid);
        let end = start + (nbits as f64 * self.bit) as u64;
        let event = if !stop && value == 0 {
            Event::UartBreak
        } else {
            Event::UartByte { value, framing_error: !stop, parity_error }
        };
        out.push(Annotation { start, end, event });
        self.frame = None;
    }
}

/// Uart keeps a short per-frame edge history to sample bits after the fact.
pub struct UartDecoder {
    uart: Uart,
    history: Vec<(u64, bool)>,
}

impl UartDecoder {
    /// Creates a UART decoder.
    pub fn new(cfg: UartConfig, samplerate: u64) -> UartDecoder {
        UartDecoder { uart: Uart::new(cfg, samplerate), history: Vec::new() }
    }
}

impl Decoder for UartDecoder {
    fn name(&self) -> String {
        format!("UART ch{} {}", self.uart.cfg.channel, self.uart.cfg.baud)
    }

    fn channels(&self) -> u16 {
        1 << self.uart.cfg.channel
    }

    fn init(&mut self, state: u16) {
        self.uart.level = (state >> self.uart.cfg.channel & 1 != 0) ^ self.uart.cfg.inverted;
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let ch = self.uart.cfg.channel;
        if t.changed() >> ch & 1 == 0 {
            return;
        }
        self.uart.finish(t.at, &self.history, out);
        let level = (t.now >> ch & 1 != 0) ^ self.uart.cfg.inverted;
        self.uart.level = level;
        if self.uart.frame.is_none() {
            self.history.clear();
            if !level {
                self.uart.frame = Some(t.at);
            }
        } else {
            self.history.push((t.at, level));
        }
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
        self.uart.finish(to, &self.history, out);
        if self.uart.frame.is_none() {
            self.history.clear();
            // A line that is still low after a break starts no new frame until
            // it goes high again.
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds transitions for 8N1 bytes at `bit` samples per bit.
    fn encode(bytes: &[u8], bit: u64, gap: u64) -> Vec<Transition> {
        let mut levels = Vec::new();
        for &b in bytes {
            levels.push(false);
            for k in 0..8 {
                levels.push(b >> k & 1 != 0);
            }
            levels.push(true);
            for _ in 0..gap {
                levels.push(true);
            }
        }
        let mut out = Vec::new();
        let mut cur = true;
        for (i, &l) in levels.iter().enumerate() {
            if l != cur {
                out.push(Transition { at: 100 + i as u64 * bit, prev: cur as u16, now: l as u16 });
                cur = l;
            }
        }
        out
    }

    #[test]
    fn decodes_bytes() {
        let mut d = UartDecoder::new(UartConfig::new(0, 115_200), 115_200 * 10);
        d.init(1);
        let mut out = Vec::new();
        for t in encode(b"Hi\xff\x00U", 10, 0) {
            d.transition(&t, &mut out);
        }
        d.advance(10_000, &mut out);
        let vals: Vec<u16> = out
            .iter()
            .map(|a| match a.event {
                Event::UartByte { value, framing_error: false, .. } => value,
                ref e => panic!("{e:?}"),
            })
            .collect();
        assert_eq!(vals, vec![b'H' as u16, b'i' as u16, 0xff, 0x00, b'U' as u16]);
    }
}
