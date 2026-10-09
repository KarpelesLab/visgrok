//! ISO/IEC 7816-3 (smart card) protocol layer over the UART decoder.
//!
//! The card's I/O line is a half-duplex asynchronous serial link (8E2 at
//! clock/372 baud by default). On top of the bytes, this decoder reports:
//!
//! - the **ATR** (answer to reset): convention, interface bytes with the
//!   Fi/Di speed parameters, offered protocols, historical bytes and TCK;
//! - **PPS** exchanges (protocol and parameter selection): the requested
//!   protocol and Fi/Di, the check byte, and whether the card accepted;
//! - later traffic grouped into frames (bursts separated by idle time),
//!   identified as T=1 blocks (NAD, PCB, LEN, information field, LRC) when
//!   they have that structure; or, with [`Iso7816::seproxyhal`], decoded as
//!   Ledger SEPROXYHAL packets (see [`super::seph`]).
//!
//! Characters are 8E2 unless another format is configured. Without more
//! lines, the underlying UART decoder finds the rate by itself, and an
//! accepted PPS sets the new rate from the Fi/Di it selects and the rate
//! measured before (the clock is the same). With the card's **clock** and
//! **reset** lines assigned, decoding follows the card's state instead:
//!
//! - each release of reset starts a new numbered session (reported as an
//!   event); the I/O line is ignored while reset is held, so power-up
//!   glitches and the reset itself don't produce bytes;
//! - characters are 8E2 at exactly 372 clock cycles per bit (the default
//!   etu), measured from the clock, until a PPS selects other Fi/Di values
//!   (reserved values leave the rate to automatic detection).

use super::seph::Seph;
use super::uart::{Parity, Uart, UartConfig};
use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

const PROTO: &str = "ISO7816";

/// Clock rate conversion factors, indexed by the Fi nibble (`None`: RFU).
pub(crate) const FI: [Option<u32>; 16] = [
    Some(372),
    Some(372),
    Some(558),
    Some(744),
    Some(1116),
    Some(1488),
    Some(1860),
    None,
    None,
    Some(512),
    Some(768),
    Some(1024),
    Some(1536),
    Some(2048),
    None,
    None,
];

/// Baud rate adjustment factors, indexed by the Di nibble (`None`: RFU).
pub(crate) const DI: [Option<u32>; 16] = [
    None,
    Some(1),
    Some(2),
    Some(4),
    Some(8),
    Some(16),
    Some(32),
    Some(64),
    Some(12),
    Some(20),
    None,
    None,
    None,
    None,
    None,
    None,
];

/// Clock cycles per bit selected by a PPS1 byte (`None`: reserved values).
/// Ledger devices (`ledger`) use the reserved Fi index 8 as 256: PPS1 0x87
/// (Fi index 8, Di 64) gives 4 clocks per bit, 2 Mbaud on their ~8 MHz
/// card clock.
pub(crate) fn pps_etu(b: u8, ledger: bool) -> Option<f64> {
    let f = match FI[(b >> 4) as usize] {
        None if ledger && b >> 4 == 8 => Some(256),
        f => f,
    };
    f.zip(DI[(b & 15) as usize]).map(|(f, d)| f as f64 / d as f64)
}

/// Describes a TA1 / PPS1 byte (`FI` high nibble, `DI` low nibble).
pub(crate) fn fidi(b: u8) -> String {
    let (f, d) = (FI[(b >> 4) as usize], DI[(b & 15) as usize]);
    match (f, d) {
        (Some(f), Some(d)) => format!("Fi={f} Di={d} (etu = {} clocks)", f as f64 / d as f64),
        _ => format!("Fi index {} Di index {} (reserved values)", b >> 4, b & 15),
    }
}

/// What the byte stream is expected to carry next.
#[derive(Debug, PartialEq)]
enum Phase {
    /// Waiting for an ATR (after reset / long idle).
    Atr,
    /// After the ATR: PPS or protocol traffic.
    Session,
}

/// Measures the card clock from its rising edges: the average period over
/// the current run of edges (restarted when the clock stops).
#[derive(Default)]
struct ClockMeter {
    first: u64,
    last: u64,
    cycles: u64,
}

impl ClockMeter {
    fn rising(&mut self, at: u64) {
        // A pause of more than 100 typical periods: the clock was stopped.
        let stopped = self.cycles > 0 && at - self.last > 100 * ((self.last - self.first) / self.cycles).max(1);
        if self.cycles == 0 && self.last == 0 || stopped {
            self.first = at;
            self.cycles = 0;
        } else {
            self.cycles += 1;
        }
        self.last = at;
    }

    /// Samples per clock cycle, once 16 cycles were seen.
    fn samples_per_cycle(&self) -> Option<f64> {
        (self.cycles >= 16).then(|| (self.last - self.first) as f64 / self.cycles as f64)
    }
}

/// Streaming ISO 7816-3 decoder.
pub struct Iso7816 {
    uart: Uart,
    /// Configuration the UART decoder is rebuilt from at each session.
    cfg: UartConfig,
    samplerate: u64,
    /// Card clock line, if assigned, and its measurement.
    clk: Option<u8>,
    clock: ClockMeter,
    /// Card reset line, if assigned (active low).
    rst: Option<u8>,
    /// Reset is held: the I/O line is ignored.
    in_reset: bool,
    /// Sessions started (reset releases seen).
    session: u32,
    /// Clock cycles per bit to apply at the next character (set at reset
    /// release and after a PPS; applied once the clock is measured).
    pending_etu: Option<f64>,
    phase: Phase,
    /// Bytes of the frame being collected: (value, start, end).
    frame: Vec<(u8, u64, u64)>,
    /// PPS request waiting for its response.
    pps_request: Option<Vec<u8>>,
    /// Protocol from the ATR / PPS (0 or 1).
    protocol: u8,
    /// Clock cycles per bit currently in use (372 until a PPS).
    etu: f64,
    /// SEPROXYHAL decoder for the traffic after the ATR / PPS.
    seph: Option<Seph>,
    scratch: Vec<Annotation>,
}

impl Iso7816 {
    /// Creates a decoder for the card I/O line described by `cfg`. Without
    /// a given frame format (`cfg.auto_format`), characters are 8E2.
    pub fn new(mut cfg: UartConfig, samplerate: u64) -> Iso7816 {
        if cfg.auto_format {
            cfg.data_bits = 8;
            cfg.parity = Parity::Even;
            cfg.stop_bits = 2;
            cfg.auto_format = false;
        }
        Iso7816 {
            uart: Uart::new(cfg.clone(), samplerate),
            cfg,
            samplerate,
            clk: None,
            clock: ClockMeter::default(),
            rst: None,
            in_reset: false,
            session: 0,
            pending_etu: None,
            phase: Phase::Atr,
            frame: Vec::new(),
            pps_request: None,
            protocol: 0,
            etu: 372.0,
            seph: None,
            scratch: Vec::new(),
        }
    }

    /// Decodes the traffic after the ATR and PPS as Ledger SEPROXYHAL
    /// packets (the link between a Ledger secure element and its MCU).
    pub fn seproxyhal(mut self) -> Iso7816 {
        self.seph = Some(Seph::new());
        self
    }

    /// Uses the card's clock line `clk` and reset line `rst` (either may be
    /// `None`) to time characters and split the traffic into sessions. With
    /// either line, characters are decoded as 8E2.
    pub fn with_lines(mut self, clk: Option<u8>, rst: Option<u8>) -> Iso7816 {
        self.clk = clk;
        self.rst = rst;
        if clk.is_some() || rst.is_some() {
            self.cfg.data_bits = 8;
            self.cfg.parity = Parity::Even;
            self.cfg.stop_bits = 2;
            self.cfg.auto_format = false;
            self.uart = Uart::new(self.cfg.clone(), self.samplerate);
        }
        if clk.is_some() {
            self.pending_etu = Some(372.0);
        }
        self
    }

    fn note(out: &mut Vec<Annotation>, at: u64, text: String) {
        out.push(Annotation {
            start: at,
            end: at,
            event: Event::Protocol {
                proto: PROTO,
                text,
                data: None,
            },
        });
    }

    /// Starts decoding characters afresh (new session or new rate). With
    /// `etu` clock cycles per bit, the rate is fixed from the clock once it
    /// is measured; without, the UART decoder detects it.
    fn restart_uart(&mut self, etu: Option<f64>) {
        let mut cfg = self.cfg.clone();
        cfg.baud = None;
        cfg.auto = true;
        if etu.is_some() && self.clk.is_some() {
            cfg.auto = false;
        }
        self.uart = Uart::new(cfg, self.samplerate);
        self.pending_etu = if self.clk.is_some() { etu } else { None };
    }

    /// Applies a pending clock-derived rate, if the clock is measured.
    fn apply_rate(&mut self, at: u64, out: &mut Vec<Annotation>) {
        let (Some(etu), Some(spc)) = (self.pending_etu, self.clock.samples_per_cycle()) else {
            return;
        };
        self.pending_etu = None;
        self.uart.set_bit_time(etu * spc, at, out);
    }

    fn reset_line(&mut self, high: bool, at: u64, out: &mut Vec<Annotation>) {
        if high == !self.in_reset {
            return;
        }
        if high {
            self.in_reset = false;
            self.session += 1;
            self.phase = Phase::Atr;
            self.pps_request = None;
            self.protocol = 0;
            self.etu = 372.0;
            self.restart_uart(Some(372.0));
            let clock = match self.clock.samples_per_cycle() {
                Some(spc) => format!(", card clock {}", crate::roles::fmt_hz(self.samplerate as f64 / spc)),
                None if self.clk.is_some() => ", card clock not running yet".into(),
                None => String::new(),
            };
            Self::note(out, at, format!("── session {}: reset released{clock} ──", self.session));
        } else {
            let mut ev = std::mem::take(&mut self.scratch);
            ev.clear();
            self.uart.advance(at, &mut ev);
            self.handle(&ev, at, out);
            self.scratch = ev;
            self.frame_done(out);
            if let Some(s) = self.seph.as_mut() {
                s.reset(out);
            }
            self.in_reset = true;
            Self::note(out, at, format!("── session {}: reset asserted ──", self.session));
        }
    }

    fn note_data(out: &mut Vec<Annotation>, start: u64, end: u64, text: String, data: &[u8]) {
        out.push(Annotation {
            start,
            end,
            event: Event::Protocol {
                proto: PROTO,
                text,
                data: Some(data.into()),
            },
        });
    }

    /// Idle time that ends a frame: 12 character times at the current rate
    /// (at least 50 µs).
    fn gap(&self) -> u64 {
        let ch = self.uart.baud().map_or(0.0, |b| 12.0 * 12.0 * self.samplerate as f64 / b);
        (ch as u64).max(self.samplerate / 20_000)
    }

    /// Length of an ATR from its first bytes, once known.
    fn atr_len(b: &[u8]) -> Option<usize> {
        if b.len() < 2 {
            return None;
        }
        let mut i = 1;
        let mut y = b[1] >> 4;
        let k = (b[1] & 15) as usize;
        let mut tck = false;
        let mut first_td = true;
        loop {
            let mut n = 0;
            for bit in 0..4 {
                if y >> bit & 1 != 0 {
                    n += 1;
                }
            }
            let td_at = if y & 8 != 0 { Some(i + n) } else { None };
            i += n;
            match td_at {
                Some(td) => {
                    let v = *b.get(td)?;
                    if v & 15 != 0 || !first_td {
                        tck |= v & 15 != 0;
                    }
                    first_td = false;
                    y = v >> 4;
                }
                None => break,
            }
        }
        Some(i + 1 + k + tck as usize)
    }

    fn describe_atr(b: &[u8]) -> String {
        let mut s = match b[0] {
            0x3b => "ATR (direct convention)".to_string(),
            0x3f => "ATR (inverse convention)".to_string(),
            t => format!("ATR (TS {t:02x}?)"),
        };
        // Interface bytes start after TS and T0.
        let mut i = 2;
        let mut y = b[1] >> 4;
        let k = (b[1] & 15) as usize;
        let mut level = 1;
        let mut protocols = Vec::new();
        let mut tck_needed = false;
        loop {
            for (bit, name) in [(0, "TA"), (1, "TB"), (2, "TC")] {
                if y >> bit & 1 != 0 {
                    let v = b[i];
                    i += 1;
                    s += &match (name, level) {
                        ("TA", 1) => format!(", TA1={v:02x} {}", fidi(v)),
                        ("TC", 1) => format!(", TC1={v:02x} (extra guard time {v})"),
                        _ => format!(", {name}{level}={v:02x}"),
                    };
                }
            }
            if y & 8 == 0 {
                break;
            }
            let td = b[i];
            i += 1;
            let t = td & 15;
            if !protocols.contains(&t) {
                protocols.push(t);
            }
            tck_needed |= t != 0;
            s += &format!(", TD{level}={td:02x} (T={t})");
            y = td >> 4;
            level += 1;
        }
        if protocols.is_empty() {
            protocols.push(0);
        }
        let hist = &b[i..i + k];
        let hex: Vec<String> = hist.iter().map(|x| format!("{x:02x}")).collect();
        let text: String = hist
            .iter()
            .map(|&c| if (0x20..0x7f).contains(&c) { c as char } else { '.' })
            .collect();
        let ps: Vec<String> = protocols.iter().map(|t| format!("T={t}")).collect();
        s += &format!("; protocols {}; {k} historical bytes {} '{text}'", ps.join(" "), hex.join(" "));
        if tck_needed {
            let ok = b[1..].iter().fold(0, |x, v| x ^ v) == 0;
            s += if ok { "; TCK ok" } else { "; TCK MISMATCH" };
        }
        s
    }

    fn pps_len(b: &[u8]) -> Option<usize> {
        let p0 = *b.get(1)?;
        Some(3 + (p0 >> 4 & 7).count_ones() as usize)
    }

    fn describe_pps(b: &[u8]) -> String {
        let p0 = b[1];
        let mut s = format!("T={}", p0 & 15);
        let mut i = 2;
        if p0 & 0x10 != 0 {
            s += &format!(", PPS1={:02x} {}", b[i], fidi(b[i]));
            i += 1;
        }
        if p0 & 0x20 != 0 {
            s += &format!(", PPS2={:02x}", b[i]);
            i += 1;
        }
        if p0 & 0x40 != 0 {
            s += &format!(", PPS3={:02x}", b[i]);
        }
        let ok = b.iter().fold(0, |x, v| x ^ v) == 0;
        s + if ok { ", PCK ok" } else { ", PCK MISMATCH" }
    }

    /// Reports a complete frame of bytes.
    fn frame_done(&mut self, out: &mut Vec<Annotation>) {
        if self.frame.is_empty() {
            return;
        }
        let bytes: Vec<u8> = self.frame.iter().map(|f| f.0).collect();
        let (start, end) = (self.frame[0].1, self.frame.last().unwrap().2);
        let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ");
        if self.phase == Phase::Atr && matches!(bytes[0], 0x3b | 0x3f) && Self::atr_len(&bytes) == Some(bytes.len()) {
            Self::note_data(out, start, end, Self::describe_atr(&bytes), &bytes);
            self.phase = Phase::Session;
            self.pps_request = None;
            if self.clk.is_none() && self.rst.is_none() {
                // A new ATR: the card was reset, back to the default rate.
                self.etu = 372.0;
            }
            if let Some(s) = self.seph.as_mut() {
                s.reset(out);
            }
            // The first offered protocol, until a PPS says otherwise.
            self.protocol = Self::atr_protocol(&bytes);
        } else if bytes[0] == 0xff && Self::pps_len(&bytes) == Some(bytes.len()) {
            match self.pps_request.take() {
                None => {
                    Self::note_data(out, start, end, format!("PPS request: {}", Self::describe_pps(&bytes)), &bytes);
                    self.pps_request = Some(bytes.clone());
                }
                Some(req) => {
                    let verdict = if req == bytes {
                        "accepted"
                    } else {
                        "answered with different parameters"
                    };
                    Self::note_data(
                        out,
                        start,
                        end,
                        format!("PPS response ({verdict}): {}", Self::describe_pps(&bytes)),
                        &bytes,
                    );
                    self.protocol = bytes[1] & 15;
                    // The new rate applies from the next character.
                    let p0 = bytes[1];
                    let etu = if p0 & 0x10 != 0 {
                        pps_etu(bytes[2], self.seph.is_some())
                    } else {
                        Some(372.0)
                    };
                    if self.clk.is_some() && req == bytes {
                        self.restart_uart(etu);
                        let what = match etu {
                            Some(e) => format!("{e} clock cycles per bit"),
                            None => "reserved Fi/Di, rate detected from the traffic".into(),
                        };
                        Self::note(out, end, format!("── new rate: {what} ──"));
                    } else if req == bytes
                        && let (Some(e), Some(baud)) = (etu, self.uart.baud())
                    {
                        // No clock line: the clock is the one that gave the
                        // current rate, so the new bit time follows from
                        // the ratio of the etus.
                        let bit = self.samplerate as f64 / baud * e / self.etu;
                        self.etu = e;
                        self.uart.set_bit_time(bit, end, out);
                        Self::note(
                            out,
                            end,
                            format!(
                                "── new rate: {e} clock cycles per bit, {} baud ──",
                                super::uart::nice_baud(self.samplerate as f64 / bit)
                            ),
                        );
                    }
                }
            }
        } else if bytes.len() >= 4 && bytes[2] as usize + 4 == bytes.len() && bytes.iter().fold(0, |x, v| x ^ v) == 0 {
            let (nad, pcb, len) = (bytes[0], bytes[1], bytes[2]);
            let kind = match pcb >> 6 {
                0 | 1 => format!("I-block seq {}{}", pcb >> 6 & 1, if pcb & 0x20 != 0 { " more" } else { "" }),
                2 => format!("R-block seq {}{}", pcb >> 4 & 1, if pcb & 3 != 0 { " error" } else { "" }),
                _ => {
                    let what = ["RESYNCH", "IFS", "ABORT", "WTX"].get((pcb & 3) as usize).copied().unwrap_or("?");
                    format!("S-block {what} {}", if pcb & 0x20 != 0 { "response" } else { "request" })
                }
            };
            Self::note_data(
                out,
                start,
                end,
                format!(
                    "T=1 {kind}, NAD {nad:02x}, {len} bytes: {} (LRC ok)",
                    hex(&bytes[3..3 + len as usize])
                ),
                &bytes,
            );
        } else {
            let what = if self.phase == Phase::Atr { "data before an ATR" } else { "frame" };
            Self::note_data(
                out,
                start,
                end,
                format!("{what} (T={}), {} bytes: {}", self.protocol, bytes.len(), hex(&bytes)),
                &bytes,
            );
        }
        self.frame.clear();
    }

    /// The first protocol the ATR offers (T=0 without TD1).
    fn atr_protocol(b: &[u8]) -> u8 {
        let y = b[1] >> 4;
        if y & 8 == 0 {
            return 0;
        }
        b.get(2 + (y & 7).count_ones() as usize).map_or(0, |td| td & 15)
    }

    fn handle(&mut self, events: &[Annotation], now: u64, out: &mut Vec<Annotation>) {
        for a in events {
            out.push(a.clone());
            match a.event {
                Event::UartByte { value, .. } => {
                    if let Some(seph) = self.seph.as_mut() {
                        // ATRs and PPS exchanges stay with the ISO layer
                        // (0x3b and 0xff are no SEPROXYHAL tags).
                        let v = value as u8;
                        let iso =
                            !self.frame.is_empty() || seph.idle() && (v == 0x3b || v == 0xff || v == 0x3f && self.phase == Phase::Atr);
                        if !iso {
                            seph.push(v, a.start, a.end, out);
                            continue;
                        }
                        if self.frame.is_empty() && v == 0x3b {
                            // An ATR between packets: the SE was reset.
                            self.phase = Phase::Atr;
                        }
                    }
                    if let Some(&(_, _, last)) = self.frame.last()
                        && a.start.saturating_sub(last) > self.gap()
                    {
                        self.frame_done(out);
                    }
                    self.frame.push((value as u8, a.start, a.end));
                    // An ATR or PPS is complete as soon as its length is.
                    let bytes: Vec<u8> = self.frame.iter().map(|f| f.0).collect();
                    let done = (self.phase == Phase::Atr && matches!(bytes[0], 0x3b | 0x3f) && Self::atr_len(&bytes) == Some(bytes.len()))
                        || (bytes[0] == 0xff && Self::pps_len(&bytes) == Some(bytes.len()));
                    if done {
                        self.frame_done(out);
                    }
                }
                Event::UartBreak => {
                    // A long low line: the card was probably reset.
                    self.frame_done(out);
                    if let Some(s) = self.seph.as_mut() {
                        s.flush(out);
                    }
                    self.phase = Phase::Atr;
                }
                _ => {}
            }
        }
        if let Some(&(_, _, last)) = self.frame.last()
            && now.saturating_sub(last) > self.gap()
        {
            self.frame_done(out);
        }
        // A packet interrupted for 20 ms won't be completed.
        let limit = self.samplerate / 50;
        if let Some(s) = self.seph.as_mut()
            && s.pending_since().is_some_and(|last| now.saturating_sub(last) > limit)
        {
            s.flush(out);
        }
    }
}

impl Decoder for Iso7816 {
    fn name(&self) -> String {
        let mut lines = String::new();
        if let Some(c) = self.clk {
            lines += &format!(" clk=ch{c}");
        }
        if let Some(c) = self.rst {
            lines += &format!(" rst=ch{c}");
        }
        let proto = if self.seph.is_some() { "ISO7816 SEPH" } else { PROTO };
        if lines.is_empty() {
            return format!("{proto} ({})", self.uart.name());
        }
        // The rate follows the card; keep the name stable.
        format!("{proto}{lines} (UART ch{})", self.cfg.channel)
    }

    fn channels(&self) -> u32 {
        [self.clk, self.rst]
            .into_iter()
            .flatten()
            .fold(self.uart.channels(), |m, c| m | 1 << c)
    }

    fn init(&mut self, state: u32) {
        self.uart.init(state);
        if let Some(r) = self.rst {
            self.in_reset = state >> r & 1 == 0;
        }
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let changed = t.changed();
        if let Some(c) = self.clk
            && changed >> c & 1 != 0
            && t.now >> c & 1 != 0
        {
            self.clock.rising(t.at);
        }
        if let Some(r) = self.rst
            && changed >> r & 1 != 0
        {
            self.reset_line(t.now >> r & 1 != 0, t.at, out);
        }
        if self.in_reset || changed & self.uart.channels() == 0 {
            return;
        }
        let mut ev = std::mem::take(&mut self.scratch);
        ev.clear();
        if self.pending_etu.is_some() {
            self.apply_rate(t.at, &mut ev);
        }
        self.uart.transition(t, &mut ev);
        self.handle(&ev, t.at, out);
        self.scratch = ev;
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
        if self.in_reset {
            return;
        }
        let mut ev = std::mem::take(&mut self.scratch);
        ev.clear();
        self.uart.advance(to, &mut ev);
        self.handle(&ev, to, out);
        self.scratch = ev;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A card bus: ch0 clock (10 samples per cycle), ch1 reset, ch2 I/O.
    /// Reset is released at 100k samples; the card sends `atr` (8E2, 372
    /// clocks per bit) 20k samples later. A glitch on I/O while reset is
    /// held must not produce bytes.
    #[test]
    fn clock_and_reset() {
        let atr = [0x3b, 0x02, 0x41, 0x42];
        let bit = 3720u64;
        // I/O levels over time: (sample, level).
        let mut io = vec![(50_000u64, false), (50_040, true)];
        let mut t = 120_000;
        for &b in &atr {
            let parity = (b as u32).count_ones() % 2 == 1;
            let mut bits = vec![false];
            bits.extend((0..8).map(|k| b >> k & 1 != 0));
            bits.push(parity);
            bits.extend([true, true]);
            for (k, &v) in bits.iter().enumerate() {
                io.push((t + k as u64 * bit, v));
            }
            t += 12 * bit;
        }
        let end = t + 20 * bit;
        // Merge clock, reset and I/O into transitions.
        let mut tr = Vec::new();
        let mut state = 0b100u32; // I/O idle high, reset low, clock low
        let mut io_i = 0;
        for at in (5..end).step_by(5) {
            let mut now = state ^ 1; // clock toggles every 5 samples
            if at >= 100_000 {
                now |= 0b010;
            }
            while io_i < io.len() && io[io_i].0 <= at {
                now = if io[io_i].1 { now | 0b100 } else { now & !0b100 };
                io_i += 1;
            }
            tr.push(Transition { at, prev: state, now });
            state = now;
        }
        let mut d = Iso7816::new(UartConfig::auto(2), 100_000_000).with_lines(Some(0), Some(1));
        d.init(0b100);
        let mut out = Vec::new();
        for t in &tr {
            d.transition(t, &mut out);
        }
        d.advance(end + 100_000, &mut out);
        let text: Vec<String> = out
            .iter()
            .filter_map(|a| match &a.event {
                Event::Protocol { text, .. } => Some(text.clone()),
                _ => None,
            })
            .collect();
        assert!(
            text[0].starts_with("── session 1: reset released, card clock 10.000 MHz"),
            "{text:?}"
        );
        assert!(text.iter().any(|t| t.starts_with("ATR (direct convention)")), "{text:?}");
        let bytes: Vec<u16> = out
            .iter()
            .filter_map(|a| match a.event {
                Event::UartByte {
                    value,
                    framing_error: false,
                    parity_error: false,
                } => Some(value),
                _ => None,
            })
            .collect();
        assert_eq!(bytes, [0x3b, 0x02, 0x41, 0x42], "{out:?}");
        assert_eq!(d.name(), "ISO7816 clk=ch0 rst=ch1 (UART ch2)");
    }

    fn texts(bytes: &[(u64, u8)], gap_samples: u64) -> Vec<String> {
        // Drive the frame logic directly with synthetic UART byte events.
        let mut d = Iso7816::new(UartConfig::auto(0), 10_000_000);
        let mut out = Vec::new();
        let ev: Vec<Annotation> = bytes
            .iter()
            .map(|&(at, v)| Annotation {
                start: at,
                end: at + 100,
                event: Event::UartByte {
                    value: v as u16,
                    framing_error: false,
                    parity_error: false,
                },
            })
            .collect();
        d.handle(&ev, bytes.last().unwrap().0 + gap_samples, &mut out);
        out.into_iter()
            .filter_map(|a| match a.event {
                Event::Protocol { text, .. } => Some(text),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn atr_pps_and_block() {
        let atr = [0x3bu8, 0x1b, 0x87, 0x05, 0x32, 0x2e, 0x35, 0x2e, 0x31, 0x04, 0x33, 0x00, 0x00, 0x04];
        let mut seq: Vec<(u64, u8)> = atr.iter().enumerate().map(|(i, &b)| (1000 + i as u64 * 120, b)).collect();
        let pps = [0xffu8, 0x10, 0x87, 0x68];
        seq.extend(pps.iter().enumerate().map(|(i, &b)| (100_000 + i as u64 * 120, b)));
        seq.extend(pps.iter().enumerate().map(|(i, &b)| (200_000 + i as u64 * 120, b)));
        // A T=1-shaped block: NAD 00, PCB 00, LEN 3, INF, LRC.
        let mut blk = vec![0x00u8, 0x00, 0x03, 0xa0, 0xb1, 0xc2];
        blk.push(blk.iter().fold(0, |x, v| x ^ v));
        seq.extend(blk.iter().enumerate().map(|(i, &b)| (300_000 + i as u64 * 120, b)));
        let t = texts(&seq, 100_000);
        assert_eq!(t.len(), 4, "{t:#?}");
        assert!(
            t[0].starts_with("ATR (direct convention), TA1=87 Fi index 8 Di index 7 (reserved values)"),
            "{}",
            t[0]
        );
        assert!(
            t[0].contains("protocols T=0; 11 historical bytes 05 32 2e 35 2e 31 04 33 00 00 04 '.2.5.1.3...'"),
            "{}",
            t[0]
        );
        assert_eq!(t[1], "PPS request: T=0, PPS1=87 Fi index 8 Di index 7 (reserved values), PCK ok");
        assert!(t[2].starts_with("PPS response (accepted)"), "{}", t[2]);
        assert_eq!(t[3], "T=1 I-block seq 0, NAD 00, 3 bytes: a0 b1 c2 (LRC ok)");
    }

    #[test]
    fn ledger_pps() {
        assert_eq!(pps_etu(0x97, true), Some(8.0));
        assert_eq!(pps_etu(0x87, true), Some(4.0));
        assert_eq!(pps_etu(0x87, false), None);
    }

    #[test]
    fn standard_ta1() {
        assert_eq!(fidi(0x96), "Fi=512 Di=32 (etu = 16 clocks)");
        assert_eq!(fidi(0x11), "Fi=372 Di=1 (etu = 372 clocks)");
    }
}
