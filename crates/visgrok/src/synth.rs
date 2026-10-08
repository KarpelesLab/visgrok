//! Synthetic signal source for testing and demos without hardware.
//!
//! Two scenarios: [`Synth::new`] (clock + UART + I2C + SPI on 8 channels) and
//! [`Synth::device`] (UART that negotiates from 21.5 kbaud to 2 Mbaud, plus an
//! SSD1306 OLED on SPI with D/C and CS).

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
    device: Option<(Track, Track)>,
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
            device: None,
        }
    }

    /// Device scenario at `samplerate` (50 MHz or more recommended):
    /// - D0: UART. A 21.5 kbaud exchange negotiates 2 Mbaud, a burst of
    ///   2 Mbaud traffic follows, then the link drops back to 21.5 kbaud.
    /// - D1 SCLK, D2 MOSI, D3 D/C, D4 CS: an SSD1306 128×64 OLED driven at
    ///   4 MHz (SPI mode 0): init sequence, then ~25 frames per second of an
    ///   animation.
    pub fn device(samplerate: u64, limit: Option<u64>) -> Synth {
        let mut s = Synth::new(samplerate, limit);
        let sr = samplerate as f64;
        let mut k = 0u64;
        let uart = Track::new(move || {
            k += 1;
            let slow = sr / 21_500.0;
            let fast = sr / 2_000_000.0;
            let mut v = Vec::new();
            idle(&mut v, sr * 0.001, 1);
            uart_bytes(&mut v, b"AT+BAUD=2000000\r\n", slow, 1);
            idle(&mut v, sr * 0.002, 1);
            uart_bytes(&mut v, b"OK\r\n", slow, 1);
            idle(&mut v, sr * 0.001, 1);
            for n in 0..20 {
                let line = format!("fast packet {k}.{n}: the quick brown fox jumps over the lazy dog\r\n");
                uart_bytes(&mut v, line.as_bytes(), fast, 1);
            }
            idle(&mut v, sr * 0.002, 1);
            uart_bytes(&mut v, b"\x00\x00", fast, 1); // reset handshake at 2M
            idle(&mut v, sr * 0.003, 1);
            v
        });
        let mut frame = 0u64;
        let spi = Track::new(move || {
            let half = sr / 8_000_000.0;
            let mut v = Vec::new();
            if frame == 0 {
                let init = [0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12, 0x81, 0xcf, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0xaf];
                spi_bytes(&mut v, &init, false, half);
            }
            spi_bytes(&mut v, &[0x21, 0, 127, 0x22, 0, 7], false, half);
            spi_bytes(&mut v, &oled_frame(frame), true, half);
            frame += 1;
            let rest = sr * 0.04 - v.len() as f64;
            idle(&mut v, rest, 0x10);
            v
        });
        s.device = Some((uart, spi));
        s
    }
}

/// An endless signal built from segments generated on demand.
struct Track {
    cur: Vec<u8>,
    start: u64,
    next_segment: Box<dyn FnMut() -> Vec<u8> + Send>,
}

impl Track {
    fn new(next_segment: impl FnMut() -> Vec<u8> + Send + 'static) -> Track {
        Track { cur: Vec::new(), start: 0, next_segment: Box::new(next_segment) }
    }

    /// Bits at sample `i`; `i` must not go backwards.
    fn at(&mut self, i: u64) -> u8 {
        while i >= self.start + self.cur.len() as u64 {
            self.start += self.cur.len() as u64;
            self.cur = (self.next_segment)();
        }
        self.cur[(i - self.start) as usize]
    }
}

fn idle(v: &mut Vec<u8>, samples: f64, level: u8) {
    v.extend(std::iter::repeat_n(level, samples.max(0.0) as usize));
}

/// 8N1 UART on bit 0.
fn uart_bytes(v: &mut Vec<u8>, bytes: &[u8], bit: f64, gap_bits: usize) {
    let base = v.len() as f64;
    let mut t = 0.0;
    for &b in bytes {
        let bits = std::iter::once(0).chain((0..8).map(|k| b >> k & 1)).chain(std::iter::repeat_n(1, 1 + gap_bits));
        for x in bits {
            t += bit;
            while (v.len() as f64) < base + t {
                v.push(x);
            }
        }
    }
}

/// SPI mode 0 on bits 1 (SCLK), 2 (MOSI), 3 (D/C), 4 (CS, active low).
fn spi_bytes(v: &mut Vec<u8>, bytes: &[u8], dc: bool, half: f64) {
    let d = (dc as u8) << 3;
    let base = v.len() as f64;
    let mut t = 0.0;
    let mut put = |v: &mut Vec<u8>, x: u8, dur: f64| {
        t += dur;
        while (v.len() as f64) < base + t {
            v.push(x);
        }
    };
    put(v, d, half);
    for &b in bytes {
        for k in (0..8).rev() {
            let m = (b >> k & 1) << 2;
            put(v, d | m, half);
            put(v, d | m | 2, half);
        }
    }
    put(v, d, half);
    put(v, d | 0x10, half);
}

/// An animated 128×64 frame in SSD1306 horizontal-addressing order.
fn oled_frame(n: u64) -> Vec<u8> {
    let mut px = vec![false; 128 * 64];
    for x in 0..128 {
        px[x] = true;
        px[63 * 128 + x] = true;
    }
    for y in 0..64 {
        px[y * 128] = true;
        px[y * 128 + 127] = true;
    }
    // A bouncing 16×16 square.
    let t = n as i64;
    let bx = (t * 3).rem_euclid(2 * 110);
    let by = (t * 2).rem_euclid(2 * 46);
    let (bx, by) = (if bx > 110 { 220 - bx } else { bx } + 1, if by > 46 { 92 - by } else { by } + 1);
    for y in by..by + 16 {
        for x in bx..bx + 16 {
            px[y as usize * 128 + x as usize] = true;
        }
    }
    // A frame counter as a bar along the bottom.
    for x in 2..2 + (n % 124) as usize {
        px[60 * 128 + x] = true;
        px[61 * 128 + x] = true;
    }
    let mut out = vec![0u8; 1024];
    for page in 0..8 {
        for x in 0..128 {
            let mut b = 0u8;
            for bit in 0..8 {
                if px[(page * 8 + bit) * 128 + x] {
                    b |= 1 << bit;
                }
            }
            out[page * 128 + x] = b;
        }
    }
    out
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
        let data: Vec<u8> = match &mut self.device {
            Some((uart, spi)) => (self.pos..self.pos + n as u64).map(|i| uart.at(i) | spi.at(i)).collect(),
            None => {
                let clk_half = (self.samplerate / 2_000_000).max(1);
                (self.pos..self.pos + n as u64)
                    .map(|i| ((i / clk_half) & 1) as u8 | self.uart.at(i) | self.i2c.at(i) | self.spi.at(i))
                    .collect()
            }
        };
        let b = Block::new(self.pos, 1, data);
        self.pos += n as u64;
        Ok(Some(b))
    }

    fn stop(&mut self) {
        self.stopped = true;
    }
}
