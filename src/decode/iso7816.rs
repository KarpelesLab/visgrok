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
//!   they have that structure.
//!
//! The underlying UART decoder follows the speed change after a PPS by
//! itself; frames are reported with the UART events.

use super::uart::{Uart, UartConfig};
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

/// Streaming ISO 7816-3 decoder.
pub struct Iso7816 {
    uart: Uart,
    samplerate: u64,
    phase: Phase,
    /// Bytes of the frame being collected: (value, start, end).
    frame: Vec<(u8, u64, u64)>,
    /// PPS request waiting for its response.
    pps_request: Option<Vec<u8>>,
    /// Protocol from the ATR / PPS (0 or 1).
    protocol: u8,
    scratch: Vec<Annotation>,
}

impl Iso7816 {
    /// Creates a decoder for the card I/O line described by `cfg`.
    pub fn new(cfg: UartConfig, samplerate: u64) -> Iso7816 {
        Iso7816 {
            uart: Uart::new(cfg, samplerate),
            samplerate,
            phase: Phase::Atr,
            frame: Vec::new(),
            pps_request: None,
            protocol: 0,
            scratch: Vec::new(),
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
    }
}

impl Decoder for Iso7816 {
    fn name(&self) -> String {
        format!("{PROTO} ({})", self.uart.name())
    }

    fn channels(&self) -> u32 {
        self.uart.channels()
    }

    fn init(&mut self, state: u32) {
        self.uart.init(state);
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let mut ev = std::mem::take(&mut self.scratch);
        ev.clear();
        self.uart.transition(t, &mut ev);
        self.handle(&ev, t.at, out);
        self.scratch = ev;
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
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
    fn standard_ta1() {
        assert_eq!(fidi(0x96), "Fi=512 Di=32 (etu = 16 clocks)");
        assert_eq!(fidi(0x11), "Fi=372 Di=1 (etu = 372 clocks)");
    }
}
