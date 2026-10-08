//! Synthetic signal source for testing and demos without hardware.

use std::io;
use std::time::{Duration, Instant};

use crate::block::Block;
use crate::source::{CaptureInfo, Source};

/// Generates 8 channels at a fixed sample rate, paced in real time:
/// ch0 1 MHz clock, ch1 UART 115200 8N1 text, ch2/ch3 I2C at 100 kHz,
/// ch4..ch7 SPI (clk, mosi, miso, cs) mode 0 at 1 MHz, idle otherwise.
pub struct Synth {
    samplerate: u64,
    pos: u64,
    block: usize,
    started: Option<Instant>,
    limit: Option<u64>,
    stopped: bool,
    uart: Wave,
    i2c: Wave,
    spi: Wave,
}

/// A precomputed periodic waveform of per-sample bits.
struct Wave {
    bits: Vec<u8>,
}

impl Wave {
    fn at(&self, i: u64) -> u8 {
        self.bits[(i % self.bits.len() as u64) as usize]
    }
}

impl Synth {
    /// Creates a generator at `samplerate` Hz (at least 10 MHz recommended).
    /// `limit` stops the stream after that many samples.
    pub fn new(samplerate: u64, limit: Option<u64>) -> Synth {
        let sr = samplerate as f64;
        // UART: a text message repeated with idle gaps.
        let msg = b"Hello from visgrok synthetic UART!\r\n";
        let bit = sr / 115_200.0;
        let mut uart_levels = Vec::new();
        for &b in msg {
            uart_levels.push(false);
            uart_levels.extend((0..8).map(|k| b >> k & 1 != 0));
            uart_levels.extend([true, true]);
        }
        uart_levels.extend([true; 200]);
        let uart = Wave { bits: resample(&uart_levels, bit, 1 << 1) };

        // I2C: write 3 bytes to 0x50, then idle for a few ms.
        let half = sr / 200_000.0 / 1.5;
        let mut phases: Vec<(bool, bool)> = vec![(true, true); 40];
        phases.extend([(true, false), (false, false)]);
        for byte in [0xa0u8, 0x00, 0x42, 0x17] {
            for b in (0..8).rev().map(|k| byte >> k & 1 != 0).chain([false]) {
                phases.extend([(false, b), (true, b), (false, b)]);
            }
        }
        phases.extend([(false, false), (true, false), (true, true)]);
        phases.extend([(true, true); 400]);
        let i2c = Wave {
            bits: resample_multi(&phases.iter().map(|&(c, d)| (c as u8) << 2 | (d as u8) << 3).collect::<Vec<_>>(), half),
        };

        // SPI mode 0, CS active low, 4-byte transfers.
        let half = sr / 2_000_000.0;
        let mut st: Vec<u8> = vec![0x80; 20]; // cs high
        st.extend([0x00; 2]);
        for (mo, mi) in [(0x9fu8, 0xffu8), (0x00, 0xef), (0x00, 0x40), (0x00, 0x18)] {
            for k in (0..8).rev() {
                let d = (mo >> k & 1) << 5 | (mi >> k & 1) << 6;
                st.extend([d, d | 0x10]);
            }
        }
        st.extend([0x00, 0x80]);
        st.extend([0x80; 200]);
        let spi = Wave { bits: resample_multi(&st, half) };

        Synth {
            samplerate,
            pos: 0,
            block: (samplerate / 100).max(1024) as usize,
            started: None,
            limit,
            stopped: false,
            uart,
            i2c,
            spi,
        }
    }
}

fn resample(levels: &[bool], per: f64, bitmask: u8) -> Vec<u8> {
    let n = (levels.len() as f64 * per) as usize;
    (0..n).map(|i| if levels[((i as f64) / per) as usize] { bitmask } else { 0 }).collect()
}

fn resample_multi(states: &[u8], per: f64) -> Vec<u8> {
    let n = (states.len() as f64 * per) as usize;
    (0..n).map(|i| states[((i as f64) / per) as usize]).collect()
}

impl Source for Synth {
    fn info(&self) -> CaptureInfo {
        CaptureInfo { device: "synthetic".into(), channels: 8, samplerate: self.samplerate, unit_size: 1 }
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        if self.stopped || self.limit.is_some_and(|l| self.pos >= l) {
            return Ok(None);
        }
        let started = *self.started.get_or_insert_with(Instant::now);
        let mut n = self.block;
        if let Some(l) = self.limit {
            n = n.min((l - self.pos) as usize);
        }
        // Pace to real time.
        let due = started + Duration::from_secs_f64((self.pos + n as u64) as f64 / self.samplerate as f64);
        if let Some(wait) = due.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
        let clk_half = (self.samplerate / 2_000_000).max(1);
        let data: Vec<u8> = (self.pos..self.pos + n as u64)
            .map(|i| ((i / clk_half) & 1) as u8 | self.uart.at(i) | self.i2c.at(i) | self.spi.at(i))
            .collect();
        let b = Block::new(self.pos, 1, data);
        self.pos += n as u64;
        Ok(Some(b))
    }

    fn stop(&mut self) {
        self.stopped = true;
    }
}
