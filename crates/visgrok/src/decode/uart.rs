//! Asynchronous serial (UART) decoder with automatic and adaptive baud rate.
//!
//! Edges are buffered from the start of the first undecoded frame, and a
//! frame is decoded only once its stop bit is in the past. This lets the
//! decoder look at the actual pulse widths before committing:
//!
//! - **Locking:** with no known rate, the bit time is estimated from the
//!   pulse widths of the first burst (shortest pulse cluster, refined so all
//!   pulses are integer multiples of it). Rates are not snapped to standard
//!   values, so odd rates like 21.5 kbaud decode exactly.
//! - **Speed-ups:** a frame containing pulses much shorter than one bit means
//!   the line got faster (e.g. after a baud-rate negotiation). The decoder
//!   re-estimates from that frame on and decodes it again at the new rate.
//! - **Slow-downs:** consecutive framing errors whose pulses are all longer
//!   than a bit trigger a re-estimation as well.
//!
//! Each rate change is reported as [`Event::UartBaud`].

use std::collections::VecDeque;

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
    /// Initial (or fixed) baud rate. `None` locks onto the first traffic.
    pub baud: Option<u32>,
    /// Follow rate changes automatically. When false, `baud` is fixed.
    pub auto: bool,
    /// Data bits (5..=9).
    pub data_bits: u8,
    /// Parity.
    pub parity: Parity,
    /// Inverted line (idle low).
    pub inverted: bool,
    /// Stop bits (1 or 2). All are checked; a low stop bit is a framing
    /// error. ISO 7816 characters use 2 (the guard time).
    pub stop_bits: u8,
    /// Detect data bits, parity and stop bits from the first frames
    /// (overrides the three fields above once detected).
    pub auto_format: bool,
}

impl UartConfig {
    /// 8N1 at a fixed `baud`.
    pub fn new(channel: u8, baud: u32) -> UartConfig {
        UartConfig { channel, baud: Some(baud), auto: false, data_bits: 8, parity: Parity::None, inverted: false, stop_bits: 1, auto_format: false }
    }

    /// 8N1 with automatic, adaptive baud rate detection.
    pub fn auto(channel: u8) -> UartConfig {
        UartConfig {
            channel,
            baud: None,
            auto: true,
            data_bits: 8,
            parity: Parity::None,
            inverted: false,
            stop_bits: 1,
            auto_format: true,
        }
    }

    /// The frame format as text, e.g. `8E2`.
    pub fn format(&self) -> String {
        let p = match self.parity {
            Parity::None => 'N',
            Parity::Even => 'E',
            Parity::Odd => 'O',
        };
        format!("{}{p}{}", self.data_bits, self.stop_bits)
    }
}

/// Rounds a measured rate for display: standard rates within 1.5% snap,
/// others keep three significant digits.
pub fn nice_baud(measured: f64) -> u32 {
    if let Some(b) = crate::roles::BAUD_RATES
        .iter()
        .copied()
        .find(|&b| ((b as f64 - measured) / b as f64).abs() < 0.015)
    {
        return b;
    }
    let mag = 10f64.powi(measured.log10().floor() as i32 - 2);
    ((measured / mag).round() * mag) as u32
}

/// Outcome of frame format detection.
enum Detect {
    /// Not enough frames yet.
    Wait,
    /// No format fits: the "frames" are noise.
    NoFit,
    /// Detected data bits, parity and stop bits.
    Found(u8, Parity, u8),
}

/// Result of decoding one frame.
struct Frame {
    value: u16,
    framing_error: bool,
    parity_error: bool,
    /// Shortest pulse fully inside the frame, in samples.
    min_width: Option<u64>,
    /// End of the frame (after the stop bit).
    end: u64,
}

/// Streaming UART decoder.
pub struct Uart {
    cfg: UartConfig,
    samplerate: u64,
    /// Samples per bit, once known.
    bit: Option<f64>,
    /// Edges (time, logical level after the edge), from the start of the
    /// first undecoded frame.
    edges: VecDeque<(u64, bool)>,
    /// Pulses shorter than this many samples are ignored.
    glitch: u64,
    /// The frame format is known (given, or detected).
    format_known: bool,
    /// Baud rate last reported.
    announced: Option<u32>,
}

/// Kept for compatibility with the earlier name.
pub type UartDecoder = Uart;

impl Uart {
    /// Creates a decoder for a stream sampled at `samplerate` Hz.
    pub fn new(cfg: UartConfig, samplerate: u64) -> Uart {
        let bit = cfg.baud.map(|b| samplerate as f64 / b as f64);
        // 25 ns: well below a bit at any rate this decoder can follow (4 Mbaud
        // = 250 ns), well above typical probe glitches.
        let glitch = (samplerate as f64 * 25e-9) as u64;
        let format_known = !cfg.auto_format;
        Uart { cfg, samplerate, bit, edges: VecDeque::new(), glitch, format_known, announced: None }
    }

    /// Current baud rate estimate.
    pub fn baud(&self) -> Option<f64> {
        self.bit.map(|b| self.samplerate as f64 / b)
    }

    fn frame_bits(&self) -> u32 {
        1 + self.cfg.data_bits as u32 + (self.cfg.parity != Parity::None) as u32 + self.stop_bits()
    }

    fn stop_bits(&self) -> u32 {
        self.cfg.stop_bits.clamp(1, 2) as u32
    }

    /// Estimates the bit time from edges starting at index `from`, using the
    /// first burst only (up to a long idle gap).
    fn estimate(&self, from: usize, settled: bool) -> Option<f64> {
        self.estimate_burst(from, settled).0
    }

    /// Like [`Uart::estimate`], also returning how many edges the first burst
    /// has and whether it is complete (followed by a gap).
    fn estimate_burst(&self, from: usize, settled: bool) -> (Option<f64>, usize, bool) {
        let mut widths = Vec::new();
        let mut min = u64::MAX;
        let mut prev: Option<u64> = None;
        let mut complete = false;
        for &(at, _) in self.edges.iter().skip(from) {
            if let Some(p) = prev {
                let w = at - p;
                if widths.len() >= 6 && w > 40 * min {
                    complete = true;
                    break; // gap between bursts
                }
                min = min.min(w);
                widths.push(w);
                if widths.len() >= 256 {
                    break;
                }
            }
            prev = Some(at);
        }
        let edges = widths.len() + 1;
        // Want enough evidence, unless the burst is over (the line went idle).
        if widths.len() < if settled || complete { 4 } else { 12 } {
            return (None, edges, complete);
        }
        let mut sorted = widths.clone();
        sorted.sort_unstable();
        // Smallest width that has company (rejects isolated glitches).
        let unit0 = sorted
            .iter()
            .copied()
            .find(|&w| sorted.iter().filter(|&&x| x * 4 >= w * 3 && x * 4 <= w * 5).count() >= 2)
            .unwrap_or(sorted[0])
            .max(1) as f64;
        let max_run = (self.frame_bits() + 1) as f64;
        let (mut sum, mut bits, mut fit, mut total) = (0.0, 0.0, 0, 0);
        for &w in &widths {
            let r = w as f64 / unit0;
            if r > max_run + 0.5 {
                continue; // inter-frame idle
            }
            total += 1;
            let k = r.round().max(1.0);
            if (r - k).abs() < 0.25 {
                fit += 1;
                sum += w as f64;
                bits += k;
            }
        }
        ((total >= 4 && fit * 5 >= total * 4).then(|| sum / bits), edges, complete)
    }

    /// Decodes the frame starting at `edges[i]` with `bit` samples per bit.
    fn decode_frame(&self, i: usize, bit: f64) -> Frame {
        let t0 = self.edges[i].0;
        let nbits = self.frame_bits();
        let mid = |k: u32| t0 + ((k as f64 + 0.5) * bit) as u64;
        let end = t0 + (nbits as f64 * bit) as u64;
        let level_at = |x: u64| -> bool {
            let mut l = false;
            for &(at, v) in self.edges.iter().skip(i) {
                if at > x {
                    break;
                }
                l = v;
            }
            l
        };
        let mut value = 0u16;
        let mut ones = 0;
        for k in 0..self.cfg.data_bits as u32 {
            if level_at(mid(1 + k)) {
                value |= 1 << k;
                ones += 1;
            }
        }
        let mut parity_error = false;
        if self.cfg.parity != Parity::None {
            let p = level_at(mid(1 + self.cfg.data_bits as u32)) as u32;
            parity_error = match self.cfg.parity {
                Parity::Even => !(ones + p).is_multiple_of(2),
                _ => (ones + p).is_multiple_of(2),
            };
        }
        let stops = self.stop_bits();
        let framing_error = (nbits - stops..nbits).any(|k| !level_at(mid(k)));
        // Pulses that start and end inside the frame (before the stop bit).
        let limit = t0 + ((nbits - stops) as f64 * bit) as u64;
        let mut min_width = None;
        let mut prev = t0;
        for &(at, _) in self.edges.iter().skip(i + 1) {
            if at > limit {
                break;
            }
            let w = at - prev;
            min_width = Some(min_width.map_or(w, |m: u64| m.min(w)));
            prev = at;
        }
        Frame { value, framing_error, parity_error, min_width, end }
    }

    fn set_bit(&mut self, bit: f64, at: u64, out: &mut Vec<Annotation>) {
        self.bit = Some(bit);
        if self.format_known {
            self.announce(at, out);
        }
    }

    fn announce(&mut self, at: u64, out: &mut Vec<Annotation>) {
        let Some(b) = self.baud() else { return };
        let new = nice_baud(b);
        if self.announced != Some(new) {
            self.announced = Some(new);
            out.push(Annotation { start: at, end: at, event: Event::UartBaud { baud: new } });
        }
    }

    /// Detects the frame format from the buffered frames at `bit` samples
    /// per bit (see [`Detect`]).
    fn detect_format(&self, bit: f64, now: u64) -> Detect {
        // Sample bits 1..=11 of up to 16 frames, plus start-to-start spacing.
        let mut frames: Vec<([bool; 12], Option<f64>)> = Vec::new();
        let mut i = 0;
        let n = self.edges.len();
        let first = self.edges.front().map_or(now, |e| e.0);
        while frames.len() < 16 {
            while i < n && self.edges[i].1 {
                i += 1;
            }
            if i >= n {
                break;
            }
            let t0 = self.edges[i].0;
            if t0 + (11.6 * bit) as u64 >= now {
                break;
            }
            let mut bits = [false; 12];
            let mut k = i;
            let mut level = false;
            for (b, slot) in bits.iter_mut().enumerate().skip(1) {
                let x = t0 + ((b as f64 + 0.5) * bit) as u64;
                while k + 1 < n && self.edges[k + 1].0 <= x {
                    k += 1;
                    level = self.edges[k].1;
                }
                *slot = level;
            }
            // The next start bit can't come before 9.5 bit times.
            let next = (i + 1..n).find(|&j| !self.edges[j].1 && self.edges[j].0 as f64 >= t0 as f64 + 9.5 * bit);
            frames.push((bits, next.map(|j| (self.edges[j].0 - t0) as f64 / bit)));
            match next {
                Some(j) => i = j,
                None => break,
            }
        }
        let waited = (now - first) as f64 / bit;
        if frames.len() < 6 && frames.len() < 16 && waited < 2000.0 {
            return Detect::Wait;
        }
        if frames.is_empty() {
            return Detect::NoFit;
        }
        let min_spacing = frames.iter().filter_map(|f| f.1).map(|s| s.round() as u32).min();
        let fits = |d: usize, p: Parity| -> bool {
            frames.iter().all(|(bits, spacing)| {
                let len = spacing.map_or(12, |s| s.round() as usize).min(12);
                let has_p = p != Parity::None;
                let stop = 1 + d + has_p as usize;
                if stop >= len || !bits[stop] {
                    return false;
                }
                if has_p {
                    let ones = bits[1..=d + 1].iter().filter(|&&b| b).count();
                    if (p == Parity::Even) != (ones % 2 == 0) {
                        return false;
                    }
                }
                true
            })
        };
        let enough = frames.len() >= 4;
        let candidates = [(8, Parity::Even), (8, Parity::Odd), (8, Parity::None), (7, Parity::Even), (7, Parity::Odd)];
        let found = candidates.iter().copied().find(|&(d, p)| {
            fits(d, p) && (p == Parity::None || enough || !fits(8, Parity::None))
        });
        match found {
            Some((d, p)) => {
                let used = 1 + d as u32 + (p != Parity::None) as u32;
                let stop = min_spacing.map_or(1, |m| m.saturating_sub(used).clamp(1, 2)) as u8;
                Detect::Found(d as u8, p, stop)
            }
            None => Detect::NoFit,
        }
    }

    /// True when the line has been quiet for much longer than the shortest
    /// pending pulse, i.e. the current burst is over.
    fn settled(&self, now: u64) -> bool {
        let quiet = now.saturating_sub(self.edges.back().map_or(now, |e| e.0));
        let min = self.edges.iter().zip(self.edges.iter().skip(1)).map(|(a, b)| b.0 - a.0).min();
        min.is_some_and(|m| quiet > 12 * m)
    }

    /// Decodes every frame whose stop bit is before `now`.
    fn run(&mut self, now: u64, out: &mut Vec<Annotation>) {
        loop {
            // Skip to the next start bit (a falling edge).
            while self.edges.front().is_some_and(|e| e.1) {
                self.edges.pop_front();
            }
            let Some(&(t0, _)) = self.edges.front() else { return };
            let bit = match self.bit {
                Some(b) => b,
                None => {
                    let quiet = now.saturating_sub(self.edges.back().unwrap().0) as f64;
                    let (first, n, complete) = self.estimate_burst(0, false);
                    if first.is_none() && complete {
                        // A burst that fits no bit time (noise, power-up
                        // glitches): drop it and look at the next one.
                        self.edges.drain(..n.min(self.edges.len()));
                        continue;
                    }
                    let est = first.or_else(|| self.estimate(0, true).filter(|&b| quiet > 20.0 * b));
                    match est {
                        Some(b) => {
                            self.set_bit(b, t0, out);
                            b
                        }
                        None => {
                            if self.edges.len() > 1 << 16 {
                                self.edges.drain(..1 << 15);
                            }
                            return;
                        }
                    }
                }
            };
            if !self.format_known {
                match self.detect_format(bit, now) {
                    Detect::Wait => return,
                    Detect::NoFit => {
                        // Locked on something that isn't serial data: drop
                        // this burst and start over.
                        let (_, n, _) = self.estimate_burst(0, true);
                        self.edges.drain(..n.max(1).min(self.edges.len()));
                        if self.cfg.baud.is_none() {
                            self.bit = None;
                        }
                        continue;
                    }
                    Detect::Found(d, p, stop) => {
                        self.cfg.data_bits = d;
                        self.cfg.parity = p;
                        // Two stop bits and one stop bit plus idle time look
                        // the same; report the count, but only enforce one so
                        // back-to-back traffic (e.g. after a speed change)
                        // still decodes.
                        self.cfg.stop_bits = stop;
                        let label = self.cfg.format();
                        self.cfg.stop_bits = 1;
                        self.format_known = true;
                        self.announce(t0, out);
                        out.push(Annotation { start: t0, end: t0, event: Event::UartFormat { format: label } });
                    }
                }
            }
            let nbits = self.frame_bits();
            let stop_mid = t0 + ((nbits as f64 - 0.5) * bit) as u64;
            if stop_mid >= now {
                return;
            }
            let f = self.decode_frame(0, bit);

            if self.cfg.auto {
                let too_fast = f.min_width.is_some_and(|w| (w as f64) < 0.7 * bit);
                let too_slow = f.framing_error && f.min_width.is_none_or(|w| w as f64 > 1.5 * bit);
                if too_fast || too_slow {
                    let settled = self.settled(now);
                    match self.estimate(0, false).or_else(|| if settled { self.estimate(0, true) } else { None }) {
                        Some(nb) if (nb / bit - 1.0).abs() > 0.2 => {
                            self.set_bit(nb, t0, out);
                            continue; // decode this frame again at the new rate
                        }
                        None if !settled => return, // wait for more edges
                        _ => {}
                    }
                }
            }

            let event = if f.framing_error && f.value == 0 {
                Event::UartBreak
            } else {
                Event::UartByte { value: f.value, framing_error: f.framing_error, parity_error: f.parity_error }
            };
            out.push(Annotation { start: t0, end: f.end, event });
            while self.edges.front().is_some_and(|e| e.0 < stop_mid) {
                self.edges.pop_front();
            }
        }
    }
}

impl Decoder for Uart {
    fn name(&self) -> String {
        // The current rate is reported through `UartBaud` events rather than
        // the name, which labels past lines too.
        match (self.cfg.auto, self.baud()) {
            (true, _) => format!("UART ch{} auto", self.cfg.channel),
            (false, b) => format!("UART ch{} {}", self.cfg.channel, b.map_or(0, nice_baud)),
        }
    }

    fn channels(&self) -> u16 {
        1 << self.cfg.channel
    }

    fn init(&mut self, _state: u16) {
        self.edges.clear();
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let ch = self.cfg.channel;
        if t.changed() >> ch & 1 == 0 {
            return;
        }
        // Pulses shorter than the glitch limit are noise: cancel them.
        if let Some(&(at, _)) = self.edges.back()
            && t.at - at < self.glitch
        {
            self.edges.pop_back();
            return;
        }
        self.run(t.at, out);
        let level = (t.now >> ch & 1 != 0) ^ self.cfg.inverted;
        if self.edges.is_empty() && level {
            return; // back to idle with nothing pending
        }
        self.edges.push_back((t.at, level));
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
        self.run(to, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Transitions for 8N1 `bytes` at `bit` samples per bit, starting at
    /// `start` with `gap` idle bits between bytes. Returns (transitions, end).
    fn encode(bytes: &[u8], bit: f64, start: u64, gap: u32) -> (Vec<Transition>, u64) {
        let mut levels = Vec::new();
        for &b in bytes {
            levels.push(false);
            levels.extend((0..8).map(|k| b >> k & 1 != 0));
            levels.push(true);
            levels.extend(std::iter::repeat_n(true, gap as usize));
        }
        let mut out = Vec::new();
        let mut cur = true;
        for (i, &l) in levels.iter().enumerate() {
            if l != cur {
                let at = start + (i as f64 * bit) as u64;
                out.push(Transition { at, prev: cur as u16, now: l as u16 });
                cur = l;
            }
        }
        (out, start + (levels.len() as f64 * bit) as u64)
    }

    fn bytes_of(out: &[Annotation]) -> Vec<u8> {
        out.iter()
            .filter_map(|a| match a.event {
                Event::UartByte { value, framing_error: false, .. } => Some(value as u8),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn fixed_rate() {
        let mut d = Uart::new(UartConfig::new(0, 115_200), 115_200 * 10);
        d.init(1);
        let mut out = Vec::new();
        for t in encode(b"Hi\xff\x00U", 10.0, 100, 0).0 {
            d.transition(&t, &mut out);
        }
        d.advance(10_000, &mut out);
        assert_eq!(bytes_of(&out), b"Hi\xff\x00U");
    }

    #[test]
    fn auto_lock_odd_rate() {
        let sr = 50_000_000u64;
        let bit = sr as f64 / 21_500.0;
        let mut d = Uart::new(UartConfig::auto(0), sr);
        let mut out = Vec::new();
        let (tr, end) = encode(b"\x55AT+SPEED?\r\n", bit, 1000, 1);
        for t in tr {
            d.transition(&t, &mut out);
        }
        d.advance(end + 100_000, &mut out);
        assert_eq!(bytes_of(&out), b"\x55AT+SPEED?\r\n");
        assert!(out.iter().any(|a| a.event == Event::UartBaud { baud: 21_500 }), "{out:?}");
        assert!(out.iter().any(|a| a.event == Event::UartFormat { format: "8N2".into() } || a.event == Event::UartFormat { format: "8N1".into() }), "{out:?}");
    }

    #[test]
    fn locks_after_power_on_noise_8e2() {
        // 200 MHz: a burst of 1-50 sample glitches and odd pulses (like a
        // supply ramp), then an 8E2 ATR at 21.5 kbaud.
        let sr = 200_000_000u64;
        let mut tr = Vec::new();
        let mut level = 0u16;
        let mut at = 1000u64;
        let mut x = 12345u32;
        for _ in 0..157 {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            at += 1 + (x % 1700) as u64;
            tr.push(Transition { at, prev: level, now: level ^ 1 });
            level ^= 1;
        }
        if level == 0 {
            at += 7;
            tr.push(Transition { at, prev: 0, now: 1 });
        }
        let bit = sr as f64 / 21_500.0;
        let atr = [0x3bu8, 0x1b, 0x87, 0x05, 0x32, 0x2e, 0x35, 0x2e, 0x31, 0x04, 0x33, 0x00, 0x00, 0x04];
        let mut levels = Vec::new();
        for &b in &atr {
            levels.push(false);
            levels.extend((0..8).map(|k| b >> k & 1 != 0));
            levels.push(b.count_ones() % 2 == 1); // even parity
            levels.extend([true, true]); // stop + guard
        }
        let start = at + 2_000_000;
        let mut cur = true;
        for (i, &l) in levels.iter().enumerate() {
            if l != cur {
                tr.push(Transition { at: start + (i as f64 * bit) as u64, prev: cur as u16, now: l as u16 });
                cur = l;
            }
        }
        // Fully automatic: rate and format (8E2) detected from the traffic.
        let mut d = Uart::new(UartConfig::auto(0), sr);
        d.init(0);
        let mut out = Vec::new();
        for t in &tr {
            d.transition(t, &mut out);
        }
        d.advance(start + (levels.len() as f64 * bit) as u64 + 10_000_000, &mut out);
        let got: Vec<u8> = out
            .iter()
            .filter(|a| a.start >= start)
            .filter_map(|a| match a.event {
                Event::UartByte { value, framing_error: false, parity_error: false } => Some(value as u8),
                _ => None,
            })
            .collect();
        assert_eq!(got, atr);
        let fmts: Vec<_> = out
            .iter()
            .filter_map(|a| match &a.event {
                Event::UartFormat { format } => Some(format.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(fmts, vec!["8E2".to_string()]);
        // Nothing decoded from the noise.
        assert!(out.iter().all(|a| a.start >= start), "{:?}", &out[..3.min(out.len())]);
    }

    #[test]
    fn second_stop_bit_is_checked() {
        // 8E2 frame of 0x00 whose second stop bit is low (a new start bit
        // arrives one bit early): the first frame is a framing error.
        let bit = 100.0;
        let mut levels = vec![true; 5];
        let frame = |levels: &mut Vec<bool>, b: u8, stop2: bool| {
            levels.push(false);
            levels.extend((0..8).map(|k| b >> k & 1 != 0));
            levels.push(b.count_ones() % 2 == 1);
            levels.push(true);
            levels.push(stop2);
        };
        frame(&mut levels, 0x55, false);
        levels.extend([true; 30]);
        let mut tr = Vec::new();
        let mut cur = true;
        for (i, &l) in levels.iter().enumerate() {
            if l != cur {
                tr.push(Transition { at: (i as f64 * bit) as u64, prev: cur as u16, now: l as u16 });
                cur = l;
            }
        }
        let mut cfg = UartConfig::new(0, 10_000);
        cfg.parity = Parity::Even;
        cfg.stop_bits = 2;
        let mut d = Uart::new(cfg, 1_000_000);
        let mut out = Vec::new();
        for t in &tr {
            d.transition(t, &mut out);
        }
        d.advance(100_000, &mut out);
        assert!(matches!(out[0].event, Event::UartByte { value: 0x55, framing_error: true, parity_error: false }), "{out:?}");
    }

    #[test]
    fn follows_speed_change() {
        let sr = 200_000_000u64;
        let slow = sr as f64 / 21_500.0;
        let fast = sr as f64 / 2_000_000.0;
        let mut d = Uart::new(UartConfig::auto(0), sr);
        let mut out = Vec::new();
        let (a, end) = encode(b"hello, switch to 2M\r\n", slow, 1000, 2);
        let (b, end2) = encode(b"\x00\x01fast data \xa5\x5a at 2Mbaud!", fast, end + 100_000, 0);
        let (c, end3) = encode(b"bye", slow, end2 + 500_000, 1);
        for t in a.iter().chain(&b).chain(&c) {
            d.transition(t, &mut out);
        }
        d.advance(end3 + 1_000_000, &mut out);
        let got = bytes_of(&out);
        let want: Vec<u8> = [&b"hello, switch to 2M\r\n"[..], b"\x00\x01fast data \xa5\x5a at 2Mbaud!", b"bye"].concat();
        assert_eq!(String::from_utf8_lossy(&got), String::from_utf8_lossy(&want));
        let bauds: Vec<u32> = out
            .iter()
            .filter_map(|a| match a.event {
                Event::UartBaud { baud } => Some(baud),
                _ => None,
            })
            .collect();
        assert_eq!(bauds, vec![21_500, 2_000_000, 21_500]);
    }
}
