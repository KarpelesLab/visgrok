//! I2C decoder.

use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

/// Streaming I2C decoder.
pub struct I2c {
    scl: u8,
    sda: u8,
    scl_level: bool,
    sda_level: bool,
    /// Inside a transfer (after START).
    active: bool,
    /// Next byte is an address byte.
    expect_addr: bool,
    bits: u16,
    nbits: u8,
    byte_start: u64,
}

impl I2c {
    /// Creates a decoder with SCL and SDA on the given channels.
    pub fn new(scl: u8, sda: u8) -> I2c {
        I2c {
            scl,
            sda,
            scl_level: true,
            sda_level: true,
            active: false,
            expect_addr: false,
            bits: 0,
            nbits: 0,
            byte_start: 0,
        }
    }
}

impl Decoder for I2c {
    fn name(&self) -> String {
        format!("I2C scl=ch{} sda=ch{}", self.scl, self.sda)
    }

    fn channels(&self) -> u32 {
        1 << self.scl | 1 << self.sda
    }

    fn init(&mut self, state: u32) {
        self.scl_level = state >> self.scl & 1 != 0;
        self.sda_level = state >> self.sda & 1 != 0;
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let scl = t.now >> self.scl & 1 != 0;
        let sda = t.now >> self.sda & 1 != 0;
        let scl_changed = scl != self.scl_level;
        let sda_changed = sda != self.sda_level;

        if sda_changed && !scl_changed && scl {
            // SDA moving while SCL is high: START (falling) or STOP (rising).
            if !sda {
                out.push(Annotation { start: t.at, end: t.at, event: Event::I2cStart });
                self.active = true;
                self.expect_addr = true;
            } else {
                out.push(Annotation { start: t.at, end: t.at, event: Event::I2cStop });
                self.active = false;
            }
            self.bits = 0;
            self.nbits = 0;
        } else if scl_changed && scl && self.active {
            // Data is sampled on SCL rising edges.
            if self.nbits == 0 {
                self.byte_start = t.at;
            }
            self.bits = self.bits << 1 | sda as u16;
            self.nbits += 1;
            if self.nbits == 9 {
                let ack = self.bits & 1 == 0;
                let byte = (self.bits >> 1) as u8;
                let event = if self.expect_addr {
                    self.expect_addr = false;
                    Event::I2cAddress { addr: byte >> 1, read: byte & 1 != 0, ack }
                } else {
                    Event::I2cData { value: byte, ack }
                };
                out.push(Annotation { start: self.byte_start, end: t.at, event });
                self.bits = 0;
                self.nbits = 0;
            }
        }
        self.scl_level = scl;
        self.sda_level = sda;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generates transitions for a write of `bytes` to 7-bit `addr`, ch0=SCL ch1=SDA.
    fn write(addr: u8, bytes: &[u8]) -> Vec<Transition> {
        let mut st = 0b11u32;
        let mut at = 0u64;
        let mut out = Vec::new();
        let mut set = |st: &mut u32, at: &mut u64, v: u32| {
            *at += 10;
            if v != *st {
                out.push(Transition { at: *at, prev: *st, now: v });
                *st = v;
            }
        };
        set(&mut st, &mut at, 0b01); // START: SDA low, SCL high
        set(&mut st, &mut at, 0b00);
        let mut frame = vec![(addr << 1, true)];
        frame.extend(bytes.iter().map(|&b| (b, true)));
        for (b, ack) in frame {
            let bits: Vec<bool> = (0..8).rev().map(|k| b >> k & 1 != 0).chain([!ack]).collect();
            for bit in bits {
                let sda = (bit as u32) << 1;
                set(&mut st, &mut at, sda);
                set(&mut st, &mut at, sda | 1);
                set(&mut st, &mut at, sda);
            }
        }
        set(&mut st, &mut at, 0b00);
        set(&mut st, &mut at, 0b01);
        set(&mut st, &mut at, 0b11); // STOP
        out
    }

    #[test]
    fn decodes_write() {
        let mut d = I2c::new(0, 1);
        d.init(0b11);
        let mut out = Vec::new();
        for t in write(0x50, &[0x12, 0xab]) {
            d.transition(&t, &mut out);
        }
        let ev: Vec<Event> = out.into_iter().map(|a| a.event).collect();
        assert_eq!(
            ev,
            vec![
                Event::I2cStart,
                Event::I2cAddress { addr: 0x50, read: false, ack: true },
                Event::I2cData { value: 0x12, ack: true },
                Event::I2cData { value: 0xab, ack: true },
                Event::I2cStop,
            ]
        );
    }
}
