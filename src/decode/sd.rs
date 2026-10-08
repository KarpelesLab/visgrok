//! SD card bus decoder (native SD mode: CLK, CMD, DAT0..DAT3).
//!
//! Everything is sampled on rising CLK edges, using the line levels just
//! before the edge (valid for default speed and high speed; UHS-I SDR50/104
//! clocks are faster than a logic analyzer can follow).
//!
//! - **CMD:** 48-bit command frames (start, direction, index, argument,
//!   CRC7, end) and responses, whose shape comes from the command that
//!   caused them: R1/R1b card status, R2 (136-bit CID/CSD), R3 (OCR), R6
//!   (RCA), R7 (interface condition). CRC7 is checked where present.
//! - **DAT:** data blocks are only expected after data commands, so DAT0
//!   held low for busy is not mistaken for data. Bus width comes from ACMD6
//!   or from all assigned DAT lines starting a block together; block size
//!   from the command (CMD16 block length, 8 bytes for SCR, 64 for SD
//!   status and CMD6). Every block's per-line CRC16 is checked; writes also
//!   decode the card's CRC status token and busy time.

use super::{Annotation, Decoder, Event};
use crate::edges::Transition;

const PROTO: &str = "SD";

/// SD bus pins.
#[derive(Clone, Debug)]
pub struct SdConfig {
    /// Clock.
    pub clk: u8,
    /// Command line.
    pub cmd: u8,
    /// DAT0..DAT3 (DAT0 is needed for data; DAT1..3 for 4-bit transfers).
    pub dat: [Option<u8>; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Resp {
    None,
    R1,
    R1b,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dir {
    Read,
    Write,
}

/// Data the host and card agreed to transfer.
#[derive(Clone, Debug)]
struct Pending {
    dir: Dir,
    /// Candidate block sizes, the expected one first. The block length is
    /// variable (CMD16, e.g. 18 bytes for a CMD42 password block) and the
    /// capture may not include the command that set it, so the size whose
    /// CRC16 and end bit check out wins.
    sizes: Vec<usize>,
    /// Blocks left; `None` until STOP_TRANSMISSION (or CMD23's count).
    left: Option<u32>,
    /// Address of the next block (block or byte address, as sent).
    addr: u32,
    what: &'static str,
}

#[derive(Debug)]
enum DatState {
    Idle,
    Block {
        start: u64,
        width: usize,
        lines: Vec<Vec<bool>>,
        clocks: usize,
    },
    /// Waiting for (or reading) the CRC status token after a written block.
    CrcStatus {
        bits: Vec<bool>,
        start: u64,
    },
    /// Just after a CRC status token: DAT0 low now means the card is busy
    /// programming (never the start of the next block).
    AfterToken,
    Busy {
        start: u64,
    },
}

/// Streaming SD bus decoder.
pub struct Sd {
    cfg: SdConfig,
    clk: bool,
    // CMD line.
    cmd_bits: Vec<bool>,
    /// Sample index of each bit in `cmd_bits` (for resynchronizing).
    cmd_times: Vec<u64>,
    cmd_len: usize,
    cmd_start: u64,
    app_next: bool,
    last: Option<(u8, bool, u32)>,
    expect: Resp,
    // Card state learned from the traffic.
    sdhc: Option<bool>,
    block_len: usize,
    bus4: Option<bool>,
    block_count: Option<u32>,
    /// Block length before an unconfirmed CMD16, restored if it fails.
    block_len_prev: Option<usize>,
    // DAT lines.
    pending: Option<Pending>,
    dat: DatState,
    samplerate: u64,
    // Glitch filter: recent changes on CMD/DAT and pending rising edges.
    /// Line state before the first entry of `history`.
    base: u32,
    history: std::collections::VecDeque<Transition>,
    rises: std::collections::VecDeque<u64>,
    last_rise: Option<u64>,
    /// CMD frames ignored as line noise since the last real frame.
    noise: u32,
}

impl Sd {
    /// Creates a decoder for a capture at `samplerate` Hz.
    pub fn new(cfg: SdConfig, samplerate: u64) -> Sd {
        Sd {
            cfg,
            clk: false,
            cmd_bits: Vec::with_capacity(136),
            cmd_times: Vec::with_capacity(136),
            cmd_len: 0,
            cmd_start: 0,
            app_next: false,
            last: None,
            expect: Resp::None,
            sdhc: None,
            block_len: 512,
            bus4: None,
            block_count: None,
            block_len_prev: None,
            pending: None,
            dat: DatState::Idle,
            samplerate,
            base: 0,
            history: Default::default(),
            rises: Default::default(),
            last_rise: None,
            noise: 0,
        }
    }

    fn note(out: &mut Vec<Annotation>, start: u64, end: u64, text: String) {
        out.push(Annotation {
            start,
            end,
            event: Event::Protocol { proto: PROTO, text },
        });
    }

    fn line(state: u32, ch: u8) -> bool {
        state >> ch & 1 != 0
    }

    // ---------------------------------------------------------------- CMD

    fn cmd_bit(&mut self, b: bool, at: u64, out: &mut Vec<Annotation>) {
        if self.cmd_len == 0 {
            if !b {
                self.cmd_bits.clear();
                self.cmd_times.clear();
                self.cmd_bits.push(false);
                self.cmd_times.push(at);
                self.cmd_start = at;
                self.cmd_len = 2; // decided after the direction bit
            }
            return;
        }
        self.cmd_bits.push(b);
        self.cmd_times.push(at);
        loop {
            if self.cmd_bits.len() >= 2 && self.cmd_len == 2 {
                self.cmd_len = if self.cmd_bits[1] || self.expect != Resp::R2 { 48 } else { 136 };
            }
            if self.cmd_bits.len() < self.cmd_len {
                return;
            }
            let bits = std::mem::take(&mut self.cmd_bits);
            let times = std::mem::take(&mut self.cmd_times);
            let ok = if bits[1] {
                self.command(&bits, at, out)
            } else {
                self.response(&bits, at, out)
            };
            self.cmd_len = 0;
            if ok {
                self.cmd_bits = bits;
                self.cmd_times = times;
                self.cmd_bits.clear();
                self.cmd_times.clear();
                return;
            }
            // Not a frame: a glitch looked like a start bit. A real frame
            // may begin later inside these bits; restart at the next 0.
            let Some(k) = bits.iter().skip(1).position(|b| !b).map(|p| p + 1) else {
                return;
            };
            self.cmd_bits = bits[k..].to_vec();
            self.cmd_times = times[k..].to_vec();
            self.cmd_start = self.cmd_times[0];
            self.cmd_len = 2;
        }
    }

    /// Handles a host command frame; false if it isn't one (bad CRC).
    fn command(&mut self, bits: &[bool], end: u64, out: &mut Vec<Annotation>) -> bool {
        let idx = field(bits, 2, 6) as u8;
        let arg = field(bits, 8, 32) as u32;
        let crc = field(bits, 40, 7) as u8;
        let crc_ok = crc7(&bits[..40]) == crc && bits[47];
        if !crc_ok {
            // A host never sends a bad CRC on a sane bus (and the card would
            // flag COM_CRC_ERROR); this is crosstalk on an idle CMD line.
            self.noise += 1;
            return false;
        }
        // CRC7 lets 1 in 128 noise frames through; reject implausible ones:
        // unknown commands, and anything but STOP/STATUS/reset while a data
        // transfer is running.
        let app = self.app_next;
        let transferring = self.pending.is_some() || matches!(self.dat, DatState::Block { .. });
        if command_info(idx, app).0 == "(unknown)" || (transferring && !app && !matches!(idx, 0 | 12 | 13)) {
            self.noise += 1;
            return false;
        }
        self.flush_noise(out);
        self.app_next = idx == 55 && !app;
        self.last = Some((idx, app, arg));
        let (name, resp) = command_info(idx, app);
        self.expect = resp;

        let addr = |a: u32| -> String {
            match self.sdhc {
                Some(true) => format!("block {a}"),
                Some(false) => format!("byte address {a:#x}"),
                None => format!("address {a:#x}"),
            }
        };
        let detail = match (app, idx) {
            (false, 8) => format!("voltage {}, check {:#04x}", vhs(arg >> 8 & 0xf), arg & 0xff),
            (false, 3) | (false, 2) | (false, 0) | (false, 12) => String::new(),
            (false, 7) | (false, 9) | (false, 10) | (false, 13) | (false, 15) | (false, 55) => {
                format!("RCA {:#06x}", arg >> 16)
            }
            (false, 6) => format!(
                "{} group1 {} group2 {}",
                if arg >> 31 != 0 { "set" } else { "check" },
                arg & 0xf,
                arg >> 4 & 0xf
            ),
            (false, 16) => format!("{arg} bytes"),
            (false, 17) | (false, 18) | (false, 24) | (false, 25) | (false, 32) | (false, 33) => addr(arg),
            (false, 23) => format!("{} blocks", arg & 0xffff),
            (true, 6) => format!("{}-bit", if arg & 3 == 2 { 4 } else { 1 }),
            (true, 41) => format!(
                "HCS {}, XPC {}, S18R {}, OCR window {:#08x}",
                arg >> 30 & 1,
                arg >> 28 & 1,
                arg >> 24 & 1,
                arg & 0xff_ffff
            ),
            _ => format!("arg {arg:#010x}"),
        };
        let prefix = if app { "ACMD" } else { "CMD" };
        let mut text = format!("{prefix}{idx} {name}");
        if !detail.is_empty() {
            text += &format!(" {detail}");
        }
        Self::note(out, self.cmd_start, end, text);

        // What happens next on the DAT lines.
        match (app, idx) {
            (false, 0) => {
                // Back to defaults: 512-byte blocks, 1-bit bus.
                self.block_len = 512;
                self.bus4 = Some(false);
                self.block_count = None;
                self.pending = None;
            }
            (false, 12) => {
                if let DatState::Block { start, .. } = self.dat {
                    Self::note(out, start, end, "data block cut short by STOP_TRANSMISSION".into());
                    self.dat = DatState::Idle;
                }
                self.pending = None;
            }
            (false, 16) => {
                self.block_len_prev = Some(self.block_len);
                self.block_len = arg as usize;
            }
            (false, 23) => self.block_count = Some(arg & 0xffff),
            (true, 6) => self.bus4 = Some(arg & 3 == 2),
            _ => {}
        }
        // Memory blocks: 512 bytes on SDHC/SDXC whatever CMD16 said, the
        // CMD16 length on SDSC; both when the card type is unknown.
        let mem: Vec<usize> = match self.sdhc {
            Some(true) => vec![512],
            Some(false) => vec![self.block_len],
            None => dedup(vec![self.block_len, 512]),
        };
        // CMD42 always uses the CMD16 length (also on SDHC); it is usually
        // 2 + password length (18 for a 16-byte password), so accept any of
        // those when the CMD16 that set it wasn't captured.
        let lock: Vec<usize> = dedup(std::iter::once(self.block_len).chain(2..=34).collect());
        let read = |sizes: Vec<usize>, left: Option<u32>, what: &'static str| Pending {
            dir: Dir::Read,
            sizes,
            left,
            addr: arg,
            what,
        };
        let write = |sizes: Vec<usize>, left: Option<u32>, what: &'static str| Pending {
            dir: Dir::Write,
            sizes,
            left,
            addr: arg,
            what,
        };
        let p = match (app, idx) {
            (false, 17) => Some(read(mem.clone(), Some(1), "read block")),
            (false, 18) => Some(read(mem.clone(), self.block_count.take(), "read block")),
            (false, 24) => Some(write(mem.clone(), Some(1), "write block")),
            (false, 25) => Some(write(mem.clone(), self.block_count.take(), "write block")),
            (false, 6) => Some(read(vec![64], Some(1), "switch function status")),
            (false, 19) => Some(read(vec![64], Some(1), "tuning block")),
            (false, 30) => Some(read(vec![4], Some(1), "write protection bits")),
            (false, 42) => Some(write(lock, Some(1), "lock/unlock data")),
            (false, 56) => Some(Pending {
                dir: if arg & 1 != 0 { Dir::Read } else { Dir::Write },
                sizes: mem.clone(),
                left: Some(1),
                addr: arg,
                what: "general command data",
            }),
            (true, 13) => Some(read(vec![64], Some(1), "SD status")),
            (true, 22) => Some(read(vec![4], Some(1), "number of written blocks")),
            (true, 51) => Some(read(vec![8], Some(1), "SCR")),
            _ => None,
        };
        if p.is_some() {
            self.pending = p;
        }
        true
    }

    /// Handles a card response frame; false if it is noise.
    fn response(&mut self, bits: &[bool], end: u64, out: &mut Vec<Annotation>) -> bool {
        let start = self.cmd_start;
        // A response nobody asked for is noise, even with a valid CRC.
        if self.expect == Resp::None {
            self.noise += 1;
            return false;
        }
        self.flush_noise(out);
        let for_cmd = match self.last {
            Some((i, app, _)) => format!("{}{i}", if app { "ACMD" } else { "CMD" }),
            None => "?".into(),
        };
        let text = match self.expect {
            Resp::R2 => {
                // bits[8..135] = register[127:1]; CRC7 covers [127:8].
                let reg: Vec<bool> = bits[8..135].to_vec();
                let crc = field(&reg, 120, 7) as u8;
                let ok = crc7(&reg[..120]) == crc && bits[135];
                let is_cid = matches!(self.last, Some((2 | 10, false, _)));
                let body = if is_cid { cid(&reg) } else { csd(&reg) };
                format!("R2 ({for_cmd}) {body}{}", if ok { "" } else { " [CRC ERROR]" })
            }
            Resp::R3 => {
                let ocr = field(bits, 8, 32) as u32;
                let ready = ocr >> 31 != 0;
                if ready {
                    self.sdhc = Some(ocr >> 30 & 1 != 0);
                }
                format!(
                    "R3 ({for_cmd}) OCR {ocr:#010x}: {}{}{}",
                    if ready { "ready" } else { "busy (initializing)" },
                    if ready {
                        if ocr >> 30 & 1 != 0 { ", SDHC/SDXC" } else { ", SDSC" }
                    } else {
                        ""
                    },
                    if ocr >> 24 & 1 != 0 { ", 1.8V accepted" } else { "" }
                )
            }
            _ => {
                let idx = field(bits, 2, 6) as u8;
                let payload = field(bits, 8, 32) as u32;
                let crc = field(bits, 40, 7) as u8;
                let ok = crc7(&bits[..40]) == crc && bits[47];
                let body = match self.expect {
                    Resp::R6 => format!("RCA {:#06x}, {}", payload >> 16, r6_status(payload & 0xffff)),
                    Resp::R7 => format!("voltage {} accepted, check {:#04x}", vhs(payload >> 8 & 0xf), payload & 0xff),
                    Resp::R4 | Resp::R5 => format!("{payload:#010x}"),
                    _ => r1_status(payload),
                };
                let kind = match self.expect {
                    Resp::R1b => "R1b",
                    Resp::R4 => "R4",
                    Resp::R5 => "R5",
                    Resp::R6 => "R6",
                    Resp::R7 => "R7",
                    Resp::None => "unexpected response",
                    _ => "R1",
                };
                let crc_note = if ok || matches!(self.expect, Resp::R4) {
                    ""
                } else {
                    " [CRC ERROR]"
                };
                // An R1 with ILLEGAL_COMMAND means no data will follow.
                if payload >> 22 & 1 != 0 && matches!(self.expect, Resp::R1 | Resp::R1b) {
                    self.pending = None;
                }
                // A rejected CMD16 leaves the block length unchanged.
                if idx == 16
                    && let Some(prev) = self.block_len_prev.take()
                    && payload & (1 << 29 | 1 << 22 | 1 << 19) != 0
                {
                    self.block_len = prev;
                }
                format!("{kind} (CMD{idx}) {body}{crc_note}")
            }
        };
        self.expect = Resp::None;
        Self::note(out, start, end, text);
        true
    }

    fn flush_noise(&mut self, out: &mut Vec<Annotation>) {
        if self.noise > 0 {
            let at = self.cmd_start;
            Self::note(
                out,
                at,
                at,
                format!(
                    "(ignored {} noise frame{} on CMD)",
                    self.noise,
                    if self.noise == 1 { "" } else { "s" }
                ),
            );
            self.noise = 0;
        }
    }

    // ---------------------------------------------------------------- DAT

    fn dat_clock(&mut self, state: u32, at: u64, out: &mut Vec<Annotation>) {
        let Some(d0) = self.cfg.dat[0] else { return };
        let dat0 = Self::line(state, d0);
        match &mut self.dat {
            DatState::Idle => {
                if dat0 {
                    return;
                }
                let Some(p) = &self.pending else {
                    self.dat = DatState::Busy { start: at };
                    return;
                };
                let have4 = self.cfg.dat.iter().all(Option::is_some);
                let all_low = self.cfg.dat.iter().flatten().all(|&c| !Self::line(state, c));
                let width = if have4 && self.bus4.unwrap_or(all_low) { 4 } else { 1 };
                let max = p.sizes.iter().max().copied().unwrap_or(512);
                let cap = clocks_for(max, width);
                self.dat = DatState::Block {
                    start: at,
                    width,
                    lines: vec![Vec::with_capacity(cap); width],
                    clocks: 0,
                };
            }
            DatState::Block {
                start,
                width,
                lines,
                clocks,
            } => {
                for (k, line) in lines.iter_mut().enumerate() {
                    let ch = self.cfg.dat[k].unwrap();
                    line.push(Self::line(state, ch));
                }
                *clocks += 1;
                let (start, width, clocks) = (*start, *width, *clocks);
                let sizes = self.pending.as_ref().map_or(vec![512], |p| p.sizes.clone());
                // Smallest candidate first: finish as soon as one checks out.
                let matched = sizes
                    .iter()
                    .copied()
                    .filter(|&s| clocks_for(s, width) == clocks)
                    .find(|&s| block_ok(lines, width, s) == (true, true));
                let last = sizes.iter().map(|&s| clocks_for(s, width)).max().unwrap_or(0) == clocks;
                if matched.is_some() || last {
                    let lines = std::mem::take(lines);
                    self.finish_block(start, at, width, &lines, matched, out);
                }
            }
            DatState::CrcStatus { bits, start } => {
                if bits.is_empty() {
                    if dat0 {
                        return; // waiting for the token's start bit
                    }
                    *start = at;
                }
                bits.push(dat0);
                if bits.len() == 5 {
                    let code = (bits[1] as u8) << 2 | (bits[2] as u8) << 1 | bits[3] as u8;
                    let what = match code {
                        0b010 => "data accepted",
                        0b101 => "rejected: CRC error",
                        0b110 => "rejected: write error",
                        _ => "invalid CRC status token",
                    };
                    Self::note(out, *start, at, format!("write CRC status {code:03b}: {what}"));
                    self.dat = DatState::AfterToken;
                }
            }
            DatState::AfterToken => {
                self.dat = if dat0 { DatState::Idle } else { DatState::Busy { start: at } };
            }
            DatState::Busy { start } => {
                if dat0 {
                    let us = (at - *start) as f64 / self.samplerate.max(1) as f64 * 1e6;
                    Self::note(out, *start, at, format!("busy for {us:.1} µs"));
                    self.dat = DatState::Idle;
                }
            }
        }
    }

    /// Reports a block. `matched` is the size whose CRC checked out; `None`
    /// means none did, and the expected size is reported with the error.
    fn finish_block(&mut self, start: u64, end: u64, width: usize, lines: &[Vec<bool>], matched: Option<usize>, out: &mut Vec<Annotation>) {
        let Some(mut p) = self.pending.clone() else {
            self.dat = DatState::Idle;
            return;
        };
        let size = matched.unwrap_or(p.sizes[0]);
        let (crc_ok, end_ok) = block_ok(lines, width, size);
        let inferred = matched.is_some() && p.sizes[0] != size;
        // Reassemble bytes: in 4-bit mode each clock carries a nibble,
        // DAT3 = MSB; in 1-bit mode DAT0 carries bytes MSB first.
        let mut bytes = Vec::with_capacity(size);
        for i in 0..size {
            let mut b = 0u8;
            for k in 0..8 {
                let bit = if width == 4 {
                    let clock = i * 2 + k / 4;
                    lines[3 - (k % 4)][clock]
                } else {
                    lines[0][i * 8 + k]
                };
                b = b << 1 | bit as u8;
            }
            bytes.push(b);
        }
        let extra = if p.what == "lock/unlock data" {
            lock_data(&bytes)
        } else {
            String::new()
        };
        let preview: String = bytes.iter().take(16).map(|b| format!("{b:02x} ")).collect();
        let ascii: String = bytes
            .iter()
            .take(16)
            .map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' })
            .collect();
        let addr = if p.what.ends_with("block") {
            match self.sdhc {
                Some(true) => format!(" {}", p.addr),
                _ => format!(" @{:#x}", p.addr),
            }
        } else {
            String::new()
        };
        Self::note(
            out,
            start,
            end,
            format!(
                "{}{addr}: {} B{}, {}-bit, CRC {}{}{extra} · {preview}|{ascii}|",
                p.what,
                size,
                if inferred { " (size from CRC)" } else { "" },
                width,
                if crc_ok { "ok" } else { "ERROR" },
                if end_ok { "" } else { ", bad end bit" },
            ),
        );
        // Later blocks of the same transfer have the size that checked out.
        if matched.is_some() {
            p.sizes = vec![size];
        }
        // Next block, if any.
        let step = if self.sdhc == Some(true) { 1 } else { size as u32 };
        let next = Pending {
            addr: p.addr.wrapping_add(step),
            left: p.left.map(|n| n.saturating_sub(1)),
            ..p.clone()
        };
        self.pending = (next.left != Some(0)).then_some(next);
        self.dat = if p.dir == Dir::Write {
            DatState::CrcStatus {
                bits: Vec::new(),
                start: end,
            }
        } else {
            DatState::Idle
        };
    }
}

impl Decoder for Sd {
    fn name(&self) -> String {
        let dats: Vec<String> = self.cfg.dat.iter().flatten().map(|c| format!("ch{c}")).collect();
        format!("SD clk=ch{} cmd=ch{} dat={}", self.cfg.clk, self.cfg.cmd, dats.join(","))
    }

    fn channels(&self) -> u32 {
        self.cfg
            .dat
            .iter()
            .flatten()
            .fold(1 << self.cfg.clk | 1 << self.cfg.cmd, |m, &c| m | 1 << c)
    }

    fn init(&mut self, state: u32) {
        self.clk = Self::line(state, self.cfg.clk);
        self.base = state;
        self.history.clear();
        self.rises.clear();
        self.last_rise = None;
    }

    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>) {
        let data_mask = self.channels() & !(1 << self.cfg.clk);
        if t.changed() & data_mask != 0 {
            self.history.push_back(*t);
        }
        let clk = Self::line(t.now, self.cfg.clk);
        let rising = clk && !self.clk;
        self.clk = clk;
        if rising {
            self.rises.push_back(t.at);
        }
        self.drain(t.at, false, out);
    }

    fn advance(&mut self, to: u64, out: &mut Vec<Annotation>) {
        self.drain(to, false, out);
    }
}

impl Sd {
    /// Line state at sample `x` according to the change history.
    fn state_at(&self, x: u64) -> u32 {
        let i = self.history.partition_point(|t| t.at <= x);
        if i == 0 { self.base } else { self.history[i - 1].now }
    }

    /// The level of `ch` just before the rising edge at `e`, ignoring pulses
    /// shorter than `w` samples (crosstalk from neighbouring lines: a real
    /// CMD/DAT level lasts at least one clock period).
    fn filtered(&self, ch: u8, e: u64, w: u64) -> bool {
        let x = e.saturating_sub(1);
        let raw = Self::line(self.state_at(x), ch);
        let changes = |t: &&Transition| t.changed() >> ch & 1 != 0;
        let started = self.history.iter().filter(|t| t.at <= x).rev().find(changes).map(|t| t.at);
        let ended = self.history.iter().filter(|t| t.at > x).find(changes).map(|t| t.at);
        match (started, ended) {
            (Some(s), Some(e2)) if e2 - s < w => !raw,
            _ => raw,
        }
    }

    /// Processes rising edges once enough of what follows them is known to
    /// judge glitches (up to ¾ of the local clock period past the edge).
    fn drain(&mut self, now: u64, _flush: bool, out: &mut Vec<Annotation>) {
        while let Some(&e) = self.rises.front() {
            let prev = self.last_rise.map(|p| e - p);
            let next = self.rises.get(1).map(|&n| n - e);
            let period = match (prev, next) {
                (Some(a), Some(b)) => a.min(b),
                (Some(a), None) => a,
                (None, Some(b)) => b,
                (None, None) => break, // wait for a second edge
            };
            let w = period * 3 / 4;
            if next.is_none() && now < e + w {
                break;
            }
            let mut state = self.state_at(e.saturating_sub(1));
            for ch in std::iter::once(self.cfg.cmd).chain(self.cfg.dat.iter().flatten().copied()) {
                if self.filtered(ch, e, w) {
                    state |= 1 << ch;
                } else {
                    state &= !(1 << ch);
                }
            }
            self.rises.pop_front();
            self.last_rise = Some(e);
            self.cmd_bit(Self::line(state, self.cfg.cmd), e, out);
            self.dat_clock(state, e, out);
            // Forget changes older than the edge before this one.
            let keep_from = e.saturating_sub(period * 2);
            while self.history.front().is_some_and(|t| t.at < keep_from) {
                self.base = self.history.pop_front().unwrap().now;
            }
        }
    }
}

/// Clocks a `size`-byte block takes on a `width`-bit bus: data, CRC16, end bit.
fn clocks_for(size: usize, width: usize) -> usize {
    size * 8 / width + 16 + 1
}

/// Checks a block of `size` bytes: (every line's CRC16 matches, end bits high).
fn block_ok(lines: &[Vec<bool>], width: usize, size: usize) -> (bool, bool) {
    let data = size * 8 / width;
    if lines.iter().any(|l| l.len() < data + 17) {
        return (false, false);
    }
    let crc = lines.iter().all(|l| crc16(&l[..data]) == field(&l[data..], 0, 16) as u16);
    (crc, lines.iter().all(|l| l[data + 16]))
}

fn dedup(mut v: Vec<usize>) -> Vec<usize> {
    let mut seen = Vec::new();
    v.retain(|x| {
        let new = !seen.contains(x);
        seen.push(*x);
        new
    });
    v
}

/// Describes a CMD42 lock/unlock data block.
fn lock_data(b: &[u8]) -> String {
    if b.is_empty() {
        return String::new();
    }
    // Flags: bit0 SET_PWD, bit1 CLR_PWD, bit2 LOCK_UNLOCK (1 = lock),
    // bit3 ERASE. With none of SET/CLR/ERASE, bit2 = 0 means unlock.
    let f = b[0];
    let mut ops = Vec::new();
    if f & 0x08 != 0 {
        ops.push("ERASE (forced)");
    }
    if f & 0x01 != 0 {
        ops.push("SET_PWD");
    }
    if f & 0x02 != 0 {
        ops.push("CLR_PWD");
    }
    if f & 0x04 != 0 {
        ops.push("LOCK");
    } else if f & 0x0b == 0 {
        ops.push("UNLOCK");
    }
    let mut s = format!(", {}", ops.join(" + "));
    if let Some(&len) = b.get(1) {
        let pwd = &b[2..(2 + len as usize).min(b.len())];
        let text = if pwd.iter().all(|c| c.is_ascii_graphic() || *c == b' ') {
            format!("'{}'", String::from_utf8_lossy(pwd))
        } else {
            pwd.iter().map(|c| format!("{c:02x}")).collect::<String>()
        };
        s += &format!(", password ({len} bytes) {text}");
    }
    s
}

/// Reads `n` bits MSB-first from `bits[at..]`.
fn field(bits: &[bool], at: usize, n: usize) -> u64 {
    bits[at..at + n].iter().fold(0u64, |v, &b| v << 1 | b as u64)
}

/// CRC7 (x^7 + x^3 + 1) over a bit sequence.
fn crc7(bits: &[bool]) -> u8 {
    let mut crc = 0u8;
    for &b in bits {
        let inv = b ^ (crc >> 6 & 1 != 0);
        crc = (crc << 1) & 0x7f;
        if inv {
            crc ^= 0x09;
        }
    }
    crc
}

/// CRC16-CCITT (x^16 + x^12 + x^5 + 1, init 0) over a bit sequence.
fn crc16(bits: &[bool]) -> u16 {
    let mut crc = 0u16;
    for &b in bits {
        let inv = b ^ (crc >> 15 != 0);
        crc <<= 1;
        if inv {
            crc ^= 0x1021;
        }
    }
    crc
}

fn vhs(v: u32) -> &'static str {
    match v {
        1 => "2.7-3.6V",
        2 => "low voltage range",
        _ => "unknown",
    }
}

const STATES: [&str; 9] = ["idle", "ready", "ident", "stby", "tran", "data", "rcv", "prg", "dis"];

fn r1_status(s: u32) -> String {
    let flags = [
        (31, "OUT_OF_RANGE"),
        (30, "ADDRESS_ERROR"),
        (29, "BLOCK_LEN_ERROR"),
        (28, "ERASE_SEQ_ERROR"),
        (27, "ERASE_PARAM"),
        (26, "WP_VIOLATION"),
        (25, "CARD_IS_LOCKED"),
        (24, "LOCK_UNLOCK_FAILED"),
        (23, "COM_CRC_ERROR"),
        (22, "ILLEGAL_COMMAND"),
        (21, "CARD_ECC_FAILED"),
        (20, "CC_ERROR"),
        (19, "ERROR"),
        (16, "CSD_OVERWRITE"),
        (15, "WP_ERASE_SKIP"),
        (13, "ERASE_RESET"),
        (8, "READY_FOR_DATA"),
        (5, "APP_CMD"),
        (3, "AKE_SEQ_ERROR"),
    ];
    let state = (s >> 9 & 0xf) as usize;
    let mut out = format!("status {s:#010x} state={}", STATES.get(state).copied().unwrap_or("?"));
    for (bit, name) in flags {
        if s >> bit & 1 != 0 {
            out += " ";
            out += name;
        }
    }
    out
}

fn r6_status(s: u32) -> String {
    // R6 packs status bits 23, 22, 19 into bits 15, 14, 13.
    let full = (s >> 15 & 1) << 23 | (s >> 14 & 1) << 22 | (s >> 13 & 1) << 19 | (s & 0x1fff);
    r1_status(full)
}

/// `reg` holds register bits [127:1]; returns bit `n` of the register.
fn reg_field(reg: &[bool], hi: usize, lo: usize) -> u64 {
    field(reg, 127 - hi, hi - lo + 1)
}

fn cid(reg: &[bool]) -> String {
    let ch = |hi: usize| -> char {
        let c = reg_field(reg, hi, hi - 7) as u8;
        if c.is_ascii_graphic() || c == b' ' { c as char } else { '.' }
    };
    let oid: String = [ch(119), ch(111)].iter().collect();
    let pnm: String = [ch(103), ch(95), ch(87), ch(79), ch(71)].iter().collect();
    let prv = reg_field(reg, 63, 56);
    let mdt = reg_field(reg, 19, 8);
    format!(
        "CID: manufacturer {:#04x} OEM '{oid}' product '{pnm}' rev {}.{} serial {:#010x} made {}-{:02}",
        reg_field(reg, 127, 120),
        prv >> 4,
        prv & 0xf,
        reg_field(reg, 55, 24),
        2000 + (mdt >> 4),
        mdt & 0xf
    )
}

fn csd(reg: &[bool]) -> String {
    let structure = reg_field(reg, 127, 126);
    let bytes = match structure {
        0 => {
            let c_size = reg_field(reg, 73, 62);
            let mult = reg_field(reg, 49, 47);
            let bl = reg_field(reg, 83, 80);
            (c_size + 1) << (mult + 2) << bl
        }
        1 => (reg_field(reg, 69, 48) + 1) * 512 * 1024,
        _ => (reg_field(reg, 75, 48) + 1) * 512 * 1024,
    };
    let speed = match reg_field(reg, 103, 96) {
        0x32 => "25 MHz",
        0x5a => "50 MHz",
        0x0b => "100 MHz",
        0x2b => "200 MHz",
        _ => "other",
    };
    format!(
        "CSD v{}: capacity {:.2} GB ({} bytes), max speed {speed}, read block {} B",
        structure + 1,
        bytes as f64 / 1e9,
        bytes,
        1u64 << reg_field(reg, 83, 80)
    )
}

fn command_info(idx: u8, app: bool) -> (&'static str, Resp) {
    use Resp::*;
    if app {
        return match idx {
            6 => ("SET_BUS_WIDTH", R1),
            13 => ("SD_STATUS", R1),
            22 => ("SEND_NUM_WR_BLOCKS", R1),
            23 => ("SET_WR_BLK_ERASE_COUNT", R1),
            41 => ("SD_SEND_OP_COND", R3),
            42 => ("SET_CLR_CARD_DETECT", R1),
            51 => ("SEND_SCR", R1),
            _ => ("(application command)", R1),
        };
    }
    match idx {
        0 => ("GO_IDLE_STATE", None),
        2 => ("ALL_SEND_CID", R2),
        3 => ("SEND_RELATIVE_ADDR", R6),
        4 => ("SET_DSR", None),
        5 => ("IO_SEND_OP_COND", R4),
        6 => ("SWITCH_FUNC", R1),
        7 => ("SELECT/DESELECT_CARD", R1b),
        8 => ("SEND_IF_COND", R7),
        9 => ("SEND_CSD", R2),
        10 => ("SEND_CID", R2),
        11 => ("VOLTAGE_SWITCH", R1),
        12 => ("STOP_TRANSMISSION", R1b),
        13 => ("SEND_STATUS", R1),
        15 => ("GO_INACTIVE_STATE", None),
        16 => ("SET_BLOCKLEN", R1),
        17 => ("READ_SINGLE_BLOCK", R1),
        18 => ("READ_MULTIPLE_BLOCK", R1),
        19 => ("SEND_TUNING_BLOCK", R1),
        20 => ("SPEED_CLASS_CONTROL", R1b),
        23 => ("SET_BLOCK_COUNT", R1),
        24 => ("WRITE_BLOCK", R1),
        25 => ("WRITE_MULTIPLE_BLOCK", R1),
        27 => ("PROGRAM_CSD", R1),
        28 => ("SET_WRITE_PROT", R1b),
        29 => ("CLR_WRITE_PROT", R1b),
        30 => ("SEND_WRITE_PROT", R1),
        32 => ("ERASE_WR_BLK_START", R1),
        33 => ("ERASE_WR_BLK_END", R1),
        38 => ("ERASE", R1b),
        42 => ("LOCK_UNLOCK", R1),
        52 => ("IO_RW_DIRECT", R5),
        53 => ("IO_RW_EXTENDED", R5),
        55 => ("APP_CMD", R1),
        56 => ("GEN_CMD", R1),
        _ => ("(unknown)", R1),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Bus generator: ch0 CLK, ch1 CMD, ch2..5 DAT0..3. Lines change while
    /// CLK is low; the card side samples on the rising edge.
    pub(crate) struct Bus {
        st: u32,
        at: u64,
        pub tr: Vec<Transition>,
    }

    impl Bus {
        pub(crate) fn new() -> Bus {
            Bus {
                st: 0b11_1110,
                at: 0,
                tr: Vec::new(),
            }
        }

        fn set(&mut self, v: u32) {
            self.at += 5;
            if v != self.st {
                self.tr.push(Transition {
                    at: self.at,
                    prev: self.st,
                    now: v,
                });
                self.st = v;
            }
        }

        /// One clock with CMD = `cmd` and DAT nibble `dat` (DAT3..0).
        pub(crate) fn clock(&mut self, cmd: bool, dat: u8) {
            let v = (cmd as u32) << 1 | (dat as u32 & 0xf) << 2;
            self.set(v);
            self.set(v | 1);
            self.set(v);
        }

        pub(crate) fn idle(&mut self, n: usize) {
            for _ in 0..n {
                self.clock(true, 0xf);
            }
        }

        fn cmd_frame(&mut self, bits: &[bool]) {
            for &b in bits {
                self.clock(b, 0xf);
            }
        }

        pub(crate) fn command(&mut self, idx: u8, arg: u32, host: bool) {
            let mut bits = vec![false, host];
            bits.extend((0..6).rev().map(|k| idx >> k & 1 != 0));
            bits.extend((0..32).rev().map(|k| arg >> k & 1 != 0));
            let c = crc7(&bits);
            bits.extend((0..7).rev().map(|k| c >> k & 1 != 0));
            bits.push(true);
            self.cmd_frame(&bits);
        }

        pub(crate) fn r2(&mut self, reg: u128) {
            let mut regbits: Vec<bool> = (0..128).rev().map(|k| reg >> k & 1 != 0).collect();
            let c = crc7(&regbits[..120]);
            for k in 0..7 {
                regbits[120 + k] = c >> (6 - k) & 1 != 0;
            }
            regbits[127] = true;
            let mut bits = vec![false, false, true, true, true, true, true, true];
            bits.extend(&regbits[..127]);
            bits.push(true);
            self.cmd_frame(&bits);
        }

        /// A 4-bit data block with per-line CRC16.
        pub(crate) fn block4(&mut self, data: &[u8]) {
            let nibbles: Vec<u8> = data.iter().flat_map(|&b| [b >> 4, b & 0xf]).collect();
            let crcs: Vec<u16> = (0..4)
                .map(|line| crc16(&nibbles.iter().map(|n| n >> line & 1 != 0).collect::<Vec<_>>()))
                .collect();
            self.clock(true, 0x0);
            for &n in &nibbles {
                self.clock(true, n);
            }
            for k in (0..16).rev() {
                let n = (0..4).fold(0u8, |acc, line| acc | ((crcs[line] >> k & 1) as u8) << line);
                self.clock(true, n);
            }
            self.clock(true, 0xf);
        }
    }

    impl Bus {
        /// A 1-bit data block on DAT0 with CRC16 (DAT1..3 held high).
        pub(crate) fn block1(&mut self, data: &[u8]) {
            let bits: Vec<bool> = data.iter().flat_map(|&b| (0..8).rev().map(move |k| b >> k & 1 != 0)).collect();
            let c = crc16(&bits);
            self.clock(true, 0xe);
            for &b in &bits {
                self.clock(true, 0xe | b as u8);
            }
            for k in (0..16).rev() {
                self.clock(true, 0xe | (c >> k & 1) as u8);
            }
            self.clock(true, 0xf);
        }

        /// The card's CRC status token on DAT0, then `busy` clocks of busy.
        pub(crate) fn crc_status(&mut self, code: u8, busy: usize) {
            self.idle(2);
            self.clock(true, 0xe);
            for k in (0..3).rev() {
                self.clock(true, 0xe | (code >> k & 1));
            }
            self.clock(true, 0xf);
            for _ in 0..busy {
                self.clock(true, 0xe);
            }
            self.idle(2);
        }
    }

    fn run(bus: &Bus, cfg: SdConfig) -> Vec<String> {
        let mut d = Sd::new(cfg, 10_000_000);
        d.init(0b11_1110);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        out.into_iter()
            .map(|a| match a.event {
                Event::Protocol { text, .. } => text,
                e => format!("{e:?}"),
            })
            .collect()
    }

    fn cfg4() -> SdConfig {
        SdConfig {
            clk: 0,
            cmd: 1,
            dat: [Some(2), Some(3), Some(4), Some(5)],
        }
    }

    #[test]
    fn crcs() {
        // CMD0 with zero argument has CRC7 0x4a (frame byte 0x95).
        let mut bits = vec![false, true];
        bits.extend([false; 6]);
        bits.extend([false; 32]);
        assert_eq!(crc7(&bits), 0x4a);
        // CRC16 of 512 bytes of 0xff on one line (known value 0x7fa1).
        assert_eq!(crc16(&[true; 512 * 8]), 0x7fa1);
    }

    #[test]
    fn init_and_read() {
        let mut bus = Bus::new();
        bus.idle(8);
        bus.command(8, 0x1aa, true);
        bus.idle(2);
        bus.command(8, 0x1aa, false);
        bus.idle(8);
        bus.command(55, 0, true);
        bus.idle(2);
        bus.command(55, 0x120, false);
        bus.idle(8);
        bus.command(41, 0x4030_0000, true);
        bus.idle(2);
        // R3: index field 111111, OCR ready + CCS, CRC field all ones.
        let ocr: u32 = 0xc0ff_8000;
        let mut bits = vec![false, false, true, true, true, true, true, true];
        bits.extend((0..32).rev().map(|k| ocr >> k & 1 != 0));
        bits.extend([true; 8]);
        bus.cmd_frame(&bits);
        bus.idle(8);
        bus.command(2, 0, true);
        bus.idle(2);
        // CID: MID 0x03, OID "SD", PNM "SU08G", PRV 8.0, PSN 0x12345678, MDT 2013-07.
        let cid: u128 = 0x03u128 << 120
            | (u16::from_be_bytes(*b"SD") as u128) << 104
            | (u64::from_be_bytes([0, 0, 0, b'S', b'U', b'0', b'8', b'G']) as u128) << 64
            | 0x80u128 << 56
            | 0x1234_5678u128 << 24
            | 0x0d7u128 << 8;
        bus.r2(cid);
        bus.idle(8);
        bus.command(55, 0xaaaa_0000, true);
        bus.idle(2);
        bus.command(55, 0x920, false);
        bus.idle(8);
        bus.command(6, 2, true); // ACMD6: 4-bit
        bus.idle(2);
        bus.command(6, 0x920, false);
        bus.idle(8);
        bus.command(17, 4096, true);
        bus.idle(2);
        bus.command(17, 0x900, false);
        bus.idle(4);
        let data: Vec<u8> = (0..512u32).map(|i| (i * 7) as u8).collect();
        bus.block4(&data);
        bus.idle(8);

        let t = run(&bus, cfg4());
        let want = [
            "CMD8 SEND_IF_COND voltage 2.7-3.6V, check 0xaa",
            "R7 (CMD8) voltage 2.7-3.6V accepted, check 0xaa",
            "CMD55 APP_CMD RCA 0x0000",
            "R1 (CMD55) status 0x00000120 state=idle READY_FOR_DATA APP_CMD",
            "ACMD41 SD_SEND_OP_COND HCS 1, XPC 0, S18R 0, OCR window 0x300000",
            "R3 (ACMD41) OCR 0xc0ff8000: ready, SDHC/SDXC",
            "CMD2 ALL_SEND_CID",
            "R2 (CMD2) CID: manufacturer 0x03 OEM 'SD' product 'SU08G' rev 8.0 serial 0x12345678 made 2013-07",
            "CMD55 APP_CMD RCA 0xaaaa",
            "R1 (CMD55) status 0x00000920 state=tran READY_FOR_DATA APP_CMD",
            "ACMD6 SET_BUS_WIDTH 4-bit",
            "R1 (CMD6) status 0x00000920 state=tran READY_FOR_DATA APP_CMD",
            "CMD17 READ_SINGLE_BLOCK block 4096",
            "R1 (CMD17) status 0x00000900 state=tran READY_FOR_DATA",
        ];
        assert_eq!(&t[..want.len()], &want, "{t:#?}");
        assert!(
            t[want.len()].starts_with("read block 4096: 512 B, 4-bit, CRC ok · 00 07 0e 15 1c 23 2a 31"),
            "{}",
            t[want.len()]
        );
        assert_eq!(t.len(), want.len() + 1, "{t:#?}");
    }

    #[test]
    fn multi_block_write_with_crc_status() {
        let mut bus = Bus::new();
        bus.idle(4);
        // SDHC card (via R3), 4-bit bus (via ACMD6).
        bus.command(55, 0xaaaa_0000, true);
        bus.idle(2);
        bus.command(55, 0x920, false);
        bus.idle(4);
        bus.command(6, 2, true);
        bus.idle(2);
        bus.command(6, 0x920, false);
        bus.idle(4);
        bus.command(23, 2, true);
        bus.idle(2);
        bus.command(23, 0x900, false);
        bus.idle(4);
        bus.command(25, 100, true);
        bus.idle(2);
        bus.command(25, 0x900, false);
        bus.idle(4);
        bus.block4(&[0xaa; 512]);
        bus.crc_status(0b010, 20);
        bus.block4(&[0x55; 512]);
        bus.crc_status(0b101, 3);
        bus.idle(4);
        let mut d = Sd::new(cfg4(), 10_000_000);
        d.sdhc = Some(true);
        d.init(0b11_1110);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        let t: Vec<String> = out
            .into_iter()
            .filter_map(|a| match a.event {
                Event::Protocol { text, .. } => Some(text),
                _ => None,
            })
            .collect();
        let tail = &t[t.len() - 6..];
        assert!(tail[0].starts_with("write block 100: 512 B, 4-bit, CRC ok · aa aa"), "{t:#?}");
        assert_eq!(tail[1], "write CRC status 010: data accepted");
        assert_eq!(tail[2], "busy for 30.0 µs");
        assert!(tail[3].starts_with("write block 101: 512 B, 4-bit, CRC ok · 55 55"), "{t:#?}");
        assert_eq!(tail[4], "write CRC status 101: rejected: CRC error");
        assert_eq!(tail[5], "busy for 4.5 µs");
    }

    #[test]
    fn one_bit_read_csd_and_stop() {
        let mut bus = Bus::new();
        bus.idle(4);
        bus.command(9, 0xaaaa_0000, true);
        bus.idle(2);
        // CSD v1 with READ_BL_LEN 9, C_SIZE 3869, C_SIZE_MULT 7, TRAN_SPEED 0x32.
        let (c_size, mult, bl): (u128, u128, u128) = (3869, 7, 9);
        let csd: u128 = 0x32u128 << 96 | bl << 80 | c_size << 62 | mult << 47;
        bus.r2(csd);
        bus.idle(4);
        bus.command(18, 0x1000, true);
        bus.idle(2);
        bus.command(18, 0x900, false);
        bus.idle(4);
        bus.block1(b"0123456789abcdef");
        bus.idle(2);
        bus.command(12, 0, true);
        bus.idle(2);
        bus.command(12, 0xb00, false);
        bus.idle(4);
        // Only DAT0 assigned: 1-bit mode. Block length 16 via CMD16 in a real
        // card; set it directly here.
        let mut d = Sd::new(
            SdConfig {
                clk: 0,
                cmd: 1,
                dat: [Some(2), None, None, None],
            },
            10_000_000,
        );
        d.block_len = 16;
        d.sdhc = Some(false);
        d.init(0b11_1110);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        let t: Vec<String> = out
            .into_iter()
            .filter_map(|a| match a.event {
                Event::Protocol { text, .. } => Some(text),
                _ => None,
            })
            .collect();
        let bytes = (c_size as u64 + 1) << (mult + 2) << bl;
        assert_eq!(
            t[1],
            format!(
                "R2 (CMD9) CSD v1: capacity {:.2} GB ({bytes} bytes), max speed 25 MHz, read block 512 B",
                bytes as f64 / 1e9
            )
        );
        assert_eq!(t[2], "CMD18 READ_MULTIPLE_BLOCK byte address 0x1000");
        assert!(t[4].starts_with("read block @0x1000: 16 B, 1-bit, CRC ok · 30 31 32"), "{t:#?}");
        assert!(t[4].ends_with("|0123456789abcdef|"), "{t:#?}");
        assert_eq!(t[5], "CMD12 STOP_TRANSMISSION");
        assert!(t[6].starts_with("R1b (CMD12) status 0x00000b00 state=data"), "{t:#?}");
        assert_eq!(t.len(), 7, "{t:#?}");
    }

    fn texts(out: Vec<Annotation>) -> Vec<String> {
        out.into_iter()
            .filter_map(|a| match a.event {
                Event::Protocol { text, .. } => Some(text),
                _ => None,
            })
            .collect()
    }

    fn decode(bus: &Bus, d: &mut Sd) -> Vec<String> {
        d.init(0b11_1110);
        let mut out = Vec::new();
        for t in &bus.tr {
            d.transition(t, &mut out);
        }
        texts(out)
    }

    fn cmd_r1(bus: &mut Bus, idx: u8, arg: u32, status: u32) {
        bus.command(idx, arg, true);
        bus.idle(2);
        bus.command(idx, status, false);
        bus.idle(4);
    }

    fn lock_block(pwd: &[u8], flags: u8) -> Vec<u8> {
        let mut b = vec![flags, pwd.len() as u8];
        b.extend_from_slice(pwd);
        b
    }

    #[test]
    fn cmd42_password_block_with_cmd16() {
        // SDHC card, 4-bit: CMD16 18, CMD42 (set password + lock), then
        // CMD16 512 and a normal read.
        let mut bus = Bus::new();
        bus.idle(4);
        cmd_r1(&mut bus, 16, 18, 0x900);
        bus.command(42, 0, true);
        bus.idle(2);
        bus.command(42, 0x900, false);
        bus.idle(4);
        bus.block4(&lock_block(b"0123456789abcdef", 0x05));
        bus.crc_status(0b010, 10);
        cmd_r1(&mut bus, 16, 512, 0x900);
        cmd_r1(&mut bus, 17, 7, 0x2000900); // R1 with CARD_IS_LOCKED, still reads in this test
        bus.block4(&[0x42; 512]);
        bus.idle(4);
        let mut d = Sd::new(cfg4(), 10_000_000);
        d.sdhc = Some(true);
        d.bus4 = Some(true);
        let t = decode(&bus, &mut d);
        let lock = t.iter().find(|s| s.starts_with("lock/unlock data")).expect("lock block");
        assert!(
            lock.starts_with("lock/unlock data: 18 B, 4-bit, CRC ok, SET_PWD + LOCK, password (16 bytes) '0123456789abcdef'"),
            "{t:#?}"
        );
        assert!(t.iter().any(|s| s == "write CRC status 010: data accepted"));
        assert!(
            t.iter().any(|s| s.starts_with("read block 7: 512 B, 4-bit, CRC ok · 42 42")),
            "{t:#?}"
        );
    }

    #[test]
    fn cmd42_size_inferred_without_cmd16() {
        // Capture starts after CMD16: the decoder still assumes 512.
        let mut bus = Bus::new();
        bus.idle(4);
        bus.command(42, 0, true);
        bus.idle(2);
        bus.command(42, 0x900, false);
        bus.idle(4);
        bus.block4(&lock_block(b"secret", 0x00)); // unlock, 8 bytes
        bus.crc_status(0b010, 5);
        let mut d = Sd::new(cfg4(), 10_000_000);
        d.bus4 = Some(true);
        let t = decode(&bus, &mut d);
        assert!(
            t.iter()
                .any(|s| s.starts_with("lock/unlock data: 8 B (size from CRC), 4-bit, CRC ok, UNLOCK, password (6 bytes) 'secret'")),
            "{t:#?}"
        );
        assert!(t.iter().any(|s| s == "write CRC status 010: data accepted"), "{t:#?}");
    }

    #[test]
    fn one_bit_cmd42_and_rejected_cmd16() {
        let mut bus = Bus::new();
        bus.idle(4);
        cmd_r1(&mut bus, 16, 18, 0x900);
        // A rejected CMD16 (BLOCK_LEN_ERROR) must not change the length.
        cmd_r1(&mut bus, 16, 3000, 0x2000_0900);
        bus.command(42, 0, true);
        bus.idle(2);
        bus.command(42, 0x900, false);
        bus.idle(4);
        bus.block1(&lock_block(b"0123456789abcdef", 0x02)); // clear password
        bus.crc_status(0b010, 5);
        let mut d = Sd::new(
            SdConfig {
                clk: 0,
                cmd: 1,
                dat: [Some(2), None, None, None],
            },
            10_000_000,
        );
        let t = decode(&bus, &mut d);
        assert!(
            t.iter()
                .any(|s| s.starts_with("lock/unlock data: 18 B, 1-bit, CRC ok, CLR_PWD, password (16 bytes)")),
            "{t:#?}"
        );
        assert_eq!(d.block_len, 18);
    }

    #[test]
    fn busy_is_not_data() {
        let mut bus = Bus::new();
        bus.idle(4);
        bus.command(7, 0xaaaa_0000, true);
        bus.idle(2);
        bus.command(7, 0x700, false);
        // Card holds DAT0 low (busy) for a while.
        for _ in 0..50 {
            bus.clock(true, 0xe);
        }
        bus.idle(4);
        let t = run(&bus, cfg4());
        assert_eq!(t.last().unwrap(), "busy for 75.0 µs", "{t:#?}");
        assert!(!t.iter().any(|s| s.contains("block")));
    }
}
