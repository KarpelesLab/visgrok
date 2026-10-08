//! Bit-level breakdown of a single decoded event, for detailed views.
//!
//! Decoders report what a stretch of bus traffic means, not where each bit
//! sits: keeping that for every event would cost far more than the events
//! themselves. [`inspect`] recomputes it on demand for one event. Given the
//! decoder's description (its [`Decoder::name`](crate::decode::Decoder::name)),
//! the event and the raw levels around it ([`Signal`]), it returns the bit
//! on each line at each sampling point and the fields those bits form
//! (start bit, command index, argument, CRC, data bytes...), each with an
//! explanation of its value.

use crate::block::Sample;
use crate::decode::sd;
use crate::decode::uart::estimate_bit_time;

/// Levels of some channels over a sample range, as a list of changes.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Signal {
    /// First sample covered.
    pub start: u64,
    /// Sample after the last one covered.
    pub end: u64,
    /// State at `start`.
    pub init: Sample,
    /// Later states, as (first sample, state), in order.
    pub changes: Vec<(u64, Sample)>,
}

impl Signal {
    /// A signal starting at sample `start` in state `init`.
    pub fn new(start: u64, init: Sample) -> Signal {
        Signal {
            start,
            end: start + 1,
            init,
            changes: Vec::new(),
        }
    }

    /// Records the state at sample `at` (after every earlier one).
    pub fn push(&mut self, at: u64, s: Sample) {
        if self.changes.last().map_or(self.init, |c| c.1) != s {
            self.changes.push((at, s));
        }
        self.end = self.end.max(at + 1);
    }

    /// State at sample `at`.
    pub fn state(&self, at: u64) -> Sample {
        let i = self.changes.partition_point(|c| c.0 <= at);
        if i == 0 { self.init } else { self.changes[i - 1].1 }
    }

    /// Level of channel `ch` at sample `at`.
    pub fn level(&self, ch: u8, at: u64) -> bool {
        self.state(at) >> ch & 1 != 0
    }

    /// Changes of channel `ch` in `from..to`, as (sample, new level).
    pub fn edges(&self, ch: u8, from: u64, to: u64) -> Vec<(u64, bool)> {
        let mut prev = self.level(ch, from.saturating_sub(1).max(self.start));
        let i = self.changes.partition_point(|c| c.0 < from);
        let mut out = Vec::new();
        for &(at, s) in self.changes[i..].iter().take_while(|c| c.0 < to) {
            let l = s >> ch & 1 != 0;
            if l != prev {
                out.push((at, l));
                prev = l;
            }
        }
        out
    }

    /// Level of `ch` just before sample `e`, ignoring a pulse shorter than
    /// `w` samples there (crosstalk, as the decoders do).
    fn before(&self, ch: u8, e: u64, w: u64) -> bool {
        let x = e.saturating_sub(1);
        let raw = self.level(ch, x);
        let i = self.changes.partition_point(|c| c.0 <= x);
        let changed = |c: &&(u64, Sample), k: usize| {
            let prev = if k == 0 { self.init } else { self.changes[k - 1].1 };
            (c.1 ^ prev) >> ch & 1 != 0
        };
        let started = self.changes[..i]
            .iter()
            .enumerate()
            .rev()
            .find(|(k, c)| changed(c, *k))
            .map(|(_, c)| c.0);
        let ended = self.changes[i..]
            .iter()
            .enumerate()
            .find(|(k, c)| changed(c, i + k))
            .map(|(_, c)| c.0);
        match (started, ended) {
            (Some(s), Some(t)) if t - s < w => !raw,
            _ => raw,
        }
    }
}

/// Where a [`Field`] is shown.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Place {
    /// On a channel: the value of one bit sampled there.
    Line(u8),
    /// On an annotation row of that name (e.g. `"frame"`, `"bytes"`).
    Row(&'static str),
}

/// A bit or a group of bits and what it means.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Field {
    /// First sample.
    pub start: u64,
    /// Sample after the end.
    pub end: u64,
    /// Where it belongs.
    pub place: Place,
    /// Short label (`"1"`, `"CMD42"`, `"CRC 0x3b ok"`).
    pub label: String,
    /// Explanation of the value (empty for plain bits).
    pub detail: String,
    /// The value is wrong (bad CRC, stop bit low...).
    pub bad: bool,
}

/// The event to break down.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Request<'a> {
    /// Name of the decoder that produced the event.
    pub source: &'a str,
    /// First sample of the event.
    pub start: u64,
    /// Sample after the event.
    pub end: u64,
    /// The event's description.
    pub text: &'a str,
    /// The event's payload, if any.
    pub data: Option<&'a [u8]>,
    /// UART bit time in samples (from the decoder's last rate report).
    pub bit_time: Option<f64>,
    /// UART frame format (e.g. `"8E2"`) from the decoder's last report.
    pub format: Option<&'a str>,
}

impl<'a> Request<'a> {
    /// The event `text` of decoder `source` spanning `start..end`, without
    /// payload or UART timing hints (set those fields when known).
    pub fn new(source: &'a str, start: u64, end: u64, text: &'a str) -> Request<'a> {
        Request {
            source,
            start,
            end,
            text,
            data: None,
            bit_time: None,
            format: None,
        }
    }
}

enum Bus {
    Sd {
        clk: u8,
        cmd: u8,
        dat: Vec<u8>,
    },
    Uart {
        ch: u8,
        iso: bool,
        clk: Option<u8>,
        rst: Option<u8>,
    },
    Spi(SpiPins),
    I2c {
        scl: u8,
        sda: u8,
    },
}

struct SpiPins {
    clk: u8,
    mosi: Option<u8>,
    miso: Option<u8>,
    cs: Option<u8>,
    dc: Option<u8>,
    mode: u8,
}

/// Value of `key=chN` in a decoder name.
fn pin(source: &str, key: &str) -> Option<u8> {
    let pat = format!("{key}=ch");
    let at = source.find(&pat)? + pat.len();
    let digits: String = source[at..].chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

fn bus(source: &str) -> Option<Bus> {
    if source.starts_with("SD ") {
        let dat = source
            .split_once("dat=")
            .map(|(_, d)| {
                d.split_whitespace()
                    .next()
                    .unwrap_or("")
                    .split(',')
                    .filter_map(|c| c.strip_prefix("ch")?.parse().ok())
                    .collect()
            })
            .unwrap_or_default();
        return Some(Bus::Sd {
            clk: pin(source, "clk")?,
            cmd: pin(source, "cmd")?,
            dat,
        });
    }
    if source.starts_with("I2C ") {
        return Some(Bus::I2c {
            scl: pin(source, "scl")?,
            sda: pin(source, "sda")?,
        });
    }
    if let Some(i) = source.find("SPI clk=") {
        let s = &source[i..];
        let mode = s
            .split_once(" mode ")
            .and_then(|(_, m)| m.chars().next()?.to_digit(10))
            .unwrap_or(0) as u8;
        return Some(Bus::Spi(SpiPins {
            clk: pin(s, "clk")?,
            mosi: pin(s, "mosi"),
            miso: pin(s, "miso"),
            cs: pin(s, "cs"),
            dc: pin(s, "dc"),
            mode,
        }));
    }
    if let Some(i) = source.find("UART ch") {
        let digits: String = source[i + 7..].chars().take_while(|c| c.is_ascii_digit()).collect();
        return Some(Bus::Uart {
            ch: digits.parse().ok()?,
            iso: source.starts_with("ISO7816"),
            clk: pin(source, "clk"),
            rst: pin(source, "rst"),
        });
    }
    None
}

/// The lines a decoder uses, with their role, in display order.
pub fn lines(source: &str) -> Vec<(u8, &'static str)> {
    const DAT: [&str; 4] = ["DAT0", "DAT1", "DAT2", "DAT3"];
    match bus(source) {
        Some(Bus::Sd { clk, cmd, dat }) => {
            let mut v = vec![(clk, "CLK"), (cmd, "CMD")];
            v.extend(dat.iter().zip(DAT).map(|(&c, n)| (c, n)));
            v
        }
        Some(Bus::Uart { ch, iso, clk, rst }) => {
            let mut v = vec![(ch, if iso { "I/O" } else { "UART" })];
            v.extend(clk.map(|c| (c, "CLK")));
            v.extend(rst.map(|c| (c, "RST")));
            v
        }
        Some(Bus::Spi(p)) => [
            (p.cs, "CS"),
            (Some(p.clk), "CLK"),
            (p.mosi, "MOSI"),
            (p.miso, "MISO"),
            (p.dc, "D/C"),
        ]
        .into_iter()
        .filter_map(|(c, n)| Some((c?, n)))
        .collect(),
        Some(Bus::I2c { scl, sda }) => vec![(scl, "SCL"), (sda, "SDA")],
        None => Vec::new(),
    }
}

/// Bitmask of [`lines`].
pub fn mask(source: &str) -> Sample {
    lines(source).iter().fold(0, |m, &(c, _)| m | 1 << c)
}

/// The sample range to fetch for `req`: the event and a margin around it.
pub fn window(req: &Request) -> (u64, u64) {
    let span = req.end.saturating_sub(req.start);
    let m = (span / 8).max(64).max(req.bit_time.unwrap_or(0.0) as u64 * 2);
    (req.start.saturating_sub(m), req.end + m)
}

/// Breaks `req` down using the levels in `sig` (see [`window`]). Unknown
/// decoders and events give no fields.
pub fn inspect(req: &Request, sig: &Signal) -> Vec<Field> {
    let mut out = Out::default();
    match bus(req.source) {
        Some(Bus::Sd { clk, cmd, dat }) => sd_event(req, sig, clk, cmd, &dat, &mut out),
        Some(Bus::Uart { ch, iso, .. }) => uart_event(req, sig, ch, iso, &mut out),
        Some(Bus::Spi(p)) => spi_event(req, sig, &p, &mut out),
        Some(Bus::I2c { scl, sda }) => i2c_event(req, sig, scl, sda, &mut out),
        None => {}
    }
    // Plain bits are only useful zoomed in; past this many, rows and
    // waveforms say enough.
    const MAX_BITS: usize = 40_000;
    if out.0.iter().filter(|f| matches!(f.place, Place::Line(_))).count() > MAX_BITS {
        out.0.retain(|f| !matches!(f.place, Place::Line(_)));
    }
    out.0
}

#[derive(Default)]
struct Out(Vec<Field>);

impl Out {
    fn bit(&mut self, ch: u8, (start, end): (u64, u64), v: bool) {
        self.0.push(Field {
            start,
            end,
            place: Place::Line(ch),
            label: (v as u8).to_string(),
            detail: String::new(),
            bad: false,
        });
    }

    fn field(&mut self, row: &'static str, start: u64, end: u64, label: impl Into<String>, detail: impl Into<String>, bad: bool) {
        self.0.push(Field {
            start,
            end,
            place: Place::Row(row),
            label: label.into(),
            detail: detail.into(),
            bad,
        });
    }
}

fn ascii(b: u8) -> String {
    if (0x20..0x7f).contains(&b) {
        format!(" '{}'", b as char)
    } else {
        String::new()
    }
}

/// Cells around sampling points: each one reaches halfway to its neighbours.
fn cells(points: &[u64]) -> Vec<(u64, u64)> {
    let n = points.len();
    (0..n)
        .map(|k| {
            let left = if k > 0 {
                (points[k] - points[k - 1]) / 2
            } else if n > 1 {
                (points[1] - points[0]) / 2
            } else {
                1
            };
            let right = if k + 1 < n { (points[k + 1] - points[k]) / 2 } else { left };
            (points[k] - left.max(1), points[k] + right.max(1))
        })
        .collect()
}

/// Clock edges of `ch` to level `to` in the signal, skipping pulses shorter
/// than 2 samples.
fn clock_edges(sig: &Signal, ch: u8, to: bool) -> Vec<u64> {
    let e = sig.edges(ch, sig.start, sig.end);
    (0..e.len())
        .filter(|&k| e[k].1 == to && e.get(k + 1).is_none_or(|n| n.0 - e[k].0 >= 2))
        .map(|k| e[k].0)
        .collect()
}

fn median_gap(points: &[u64]) -> u64 {
    let mut g: Vec<u64> = points.windows(2).map(|w| w[1] - w[0]).collect();
    if g.is_empty() {
        return 1;
    }
    g.sort_unstable();
    g[g.len() / 2].max(1)
}

// ------------------------------------------------------------------ SD

fn sd_event(req: &Request, sig: &Signal, clk: u8, cmd: u8, dat: &[u8], out: &mut Out) {
    let edges = clock_edges(sig, clk, true);
    if edges.len() < 2 {
        return;
    }
    let w = (median_gap(&edges) / 3).max(2);
    let first = edges.partition_point(|&e| e < req.start);
    let edges = &edges[first..];
    let cell = cells(edges);
    let sample = |ch: u8, k: usize| sig.before(ch, edges[k], w);
    let t = req.text.trim_start();
    if t.starts_with("CMD") || t.starts_with("ACMD") || is_response(t) {
        sd_frame(t, edges, &cell, cmd, &sample, out);
    } else if t.contains("CRC status") {
        if let (Some(&d0), true) = (dat.first(), edges.len() >= 5) {
            let bits: Vec<bool> = (0..5).map(|k| sample(d0, k)).collect();
            for (k, &b) in bits.iter().enumerate() {
                out.bit(d0, cell[k], b);
            }
            let code = (bits[1] as u8) << 2 | (bits[2] as u8) << 1 | bits[3] as u8;
            let what = match code {
                0b010 => "data accepted",
                0b101 => "rejected: CRC error",
                0b110 => "rejected: write error",
                _ => "invalid token",
            };
            out.field("frame", cell[0].0, cell[0].1, "S", "start bit (0)", bits[0]);
            out.field(
                "frame",
                cell[1].0,
                cell[3].1,
                format!("{code:03b}"),
                format!("CRC status: {what}"),
                code != 0b010,
            );
            out.field("frame", cell[4].0, cell[4].1, "E", "end bit (1)", !bits[4]);
        }
    } else if t.starts_with("busy") {
        out.field(
            "frame",
            req.start,
            req.end,
            "busy",
            "the card holds DAT0 low while it programs the data",
            false,
        );
    } else if req.data.is_some() || t.contains(" block") {
        sd_block(req, edges, &cell, dat, &sample, out);
    }
}

fn is_response(t: &str) -> bool {
    let b = t.as_bytes();
    b.len() > 1 && b[0] == b'R' && b[1].is_ascii_digit()
}

fn sd_frame(t: &str, edges: &[u64], cell: &[(u64, u64)], cmd: u8, sample: &dyn Fn(u8, usize) -> bool, out: &mut Out) {
    let response = is_response(t);
    let r2 = t.starts_with("R2");
    let n = if r2 { 136 } else { 48 };
    if edges.len() < n {
        return;
    }
    let bits: Vec<bool> = (0..n).map(|k| sample(cmd, k)).collect();
    for (k, &b) in bits.iter().enumerate() {
        out.bit(cmd, cell[k], b);
    }
    let span = |a: usize, b: usize| (cell[a].0, cell[b - 1].1);
    let mut f = |a: usize, b: usize, label: String, detail: String, bad: bool| {
        let (s, e) = span(a, b);
        out.field("frame", s, e, label, detail, bad);
    };
    f(0, 1, "S".into(), "start bit (0)".into(), bits[0]);
    let dir = bits[1];
    f(
        1,
        2,
        if dir { "host" } else { "card" }.into(),
        format!(
            "transmission bit {}: {}",
            dir as u8,
            if dir {
                "host → card (command)"
            } else {
                "card → host (response)"
            }
        ),
        false,
    );
    // The command a response answers ("R1 (CMD13) ..." / "R1 (ACMD41) ...").
    let echo: Option<(u8, bool)> = t.split_once("CMD").and_then(|(pre, rest)| {
        let d: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        Some((d.parse().ok()?, pre.ends_with('A')))
    });
    let index = sd::field(&bits, 2, 6) as u8;
    if r2 {
        f(2, 8, "111111".into(), "reserved".into(), index != 0x3f);
        let reg = &bits[8..135]; // register bits [127:1]
        let what = if t.contains("CSD") || matches!(echo, Some((9, _))) {
            sd::csd(reg)
        } else {
            sd::cid(reg)
        };
        let crc_rx = sd::field(reg, 120, 7) as u8;
        let crc = sd::crc7(&reg[..120]);
        f(
            8,
            128,
            what.split(':').next().unwrap_or("register").to_string(),
            what.clone(),
            false,
        );
        f(
            128,
            135,
            format!("CRC {crc_rx:#04x}"),
            crc_text(crc_rx, crc, "CRC7 of the register"),
            crc != crc_rx,
        );
    } else {
        let arg = sd::field(&bits, 8, 32) as u32;
        let r3 = response && t.starts_with("R3");
        if r3 {
            f(2, 8, "111111".into(), "reserved".into(), index != 0x3f);
        } else if response {
            let expect = echo.map(|e| e.0);
            f(
                2,
                8,
                format!("CMD{index}"),
                format!("command index echoed: {index}"),
                expect.is_some_and(|e| e != index),
            );
        } else {
            let app = t.starts_with("ACMD");
            let name = sd::command_info(index, app).0;
            f(
                2,
                8,
                format!("{}CMD{index}", if app { "A" } else { "" }),
                format!("command index {index} ({index:06b}) = {name}"),
                false,
            );
        }
        let detail = if response {
            response_arg(t, arg)
        } else {
            command_arg(index, t.starts_with("ACMD"), arg)
        };
        f(8, 40, format!("{arg:#010x}"), detail, false);
        if r3 {
            f(40, 47, "1111111".into(), "reserved (no CRC in R3)".into(), false);
        } else {
            let crc_rx = sd::field(&bits, 40, 7) as u8;
            let crc = sd::crc7(&bits[..40]);
            f(
                40,
                47,
                format!("CRC {crc_rx:#04x}"),
                crc_text(crc_rx, crc, "CRC7 of the first 40 bits"),
                crc != crc_rx,
            );
        }
    }
    f(n - 1, n, "E".into(), "end bit (1)".into(), !bits[n - 1]);
}

fn crc_text(rx: u8, calc: u8, what: &str) -> String {
    if rx == calc {
        format!("{what}: {rx:#04x}, correct")
    } else {
        format!("{what}: received {rx:#04x}, computed {calc:#04x} (mismatch)")
    }
}

fn command_arg(idx: u8, app: bool, arg: u32) -> String {
    let rca = |extra: &str| format!("RCA {:#06x} [31:16]{extra}", arg >> 16);
    match (app, idx) {
        (true, 6) => format!(
            "bus width [1:0] = {}: {}",
            arg & 3,
            match arg & 3 {
                0 => "1-bit",
                2 => "4-bit",
                _ => "reserved",
            }
        ),
        (true, 41) => format!(
            "HCS (host supports SDHC/SDXC) [30] = {}, XPC [28] = {}, S18R (1.8 V request) [24] = {}, voltage window [23:0] = {:#08x}",
            arg >> 30 & 1,
            arg >> 28 & 1,
            arg >> 24 & 1,
            arg & 0xff_ffff
        ),
        (true, 23) => format!("pre-erase block count [22:0] = {}", arg & 0x7f_ffff),
        (true, _) => format!("argument {arg:#010x}"),
        (false, 17 | 18 | 24 | 25 | 32 | 33) => format!(
            "data address {arg} ({arg:#x}): block {arg} on SDHC/SDXC (byte offset {:#x}), or a byte address on SDSC",
            arg as u64 * 512
        ),
        (false, 7) if arg >> 16 == 0 => "RCA 0: deselect all cards".into(),
        (false, 7 | 9 | 10 | 15 | 55 | 4) => rca(""),
        (false, 13) => rca(&format!(", send task status [15] = {}", arg >> 15 & 1)),
        (false, 8) => format!(
            "voltage supplied [11:8] = {} ({}), check pattern [7:0] = {:#04x}",
            arg >> 8 & 0xf,
            sd::vhs(arg >> 8 & 0xf),
            arg & 0xff
        ),
        (false, 16) => format!("block length = {arg} bytes"),
        (false, 23) => format!("block count = {}", arg & 0xffff),
        (false, 6) => format!(
            "mode [31] = {} ({}), function groups 6..1 = {:x} {:x} {:x} {:x} {:x} {:x}",
            arg >> 31,
            if arg >> 31 == 1 { "switch" } else { "check" },
            arg >> 20 & 0xf,
            arg >> 16 & 0xf,
            arg >> 12 & 0xf,
            arg >> 8 & 0xf,
            arg >> 4 & 0xf,
            arg & 0xf
        ),
        (false, 0 | 2 | 3 | 12 | 42) if arg == 0 => "stuff bits (unused, 0)".into(),
        _ => format!("argument {arg:#010x}"),
    }
}

fn response_arg(t: &str, arg: u32) -> String {
    if t.starts_with("R3") {
        format!(
            "OCR: power-up done [31] = {} ({}), CCS [30] = {} ({}), UHS-II [29] = {}, S18A (1.8 V accepted) [24] = {}, voltage window [23:15] = {:#05x}",
            arg >> 31,
            if arg >> 31 == 1 { "ready" } else { "busy" },
            arg >> 30 & 1,
            if arg >> 30 & 1 == 1 {
                "SDHC/SDXC, block addressing"
            } else {
                "SDSC, byte addressing"
            },
            arg >> 29 & 1,
            arg >> 24 & 1,
            arg >> 15 & 0x1ff
        )
    } else if t.starts_with("R6") {
        format!(
            "new RCA [31:16] = {:#06x}; card status bits: {}",
            arg >> 16,
            sd::r6_status(arg & 0xffff)
        )
    } else if t.starts_with("R7") {
        format!(
            "voltage accepted [11:8] = {} ({}), check pattern echo [7:0] = {:#04x}",
            arg >> 8 & 0xf,
            sd::vhs(arg >> 8 & 0xf),
            arg & 0xff
        )
    } else {
        format!("card status: {}", sd::r1_status(arg))
    }
}

fn sd_block(req: &Request, edges: &[u64], cell: &[(u64, u64)], dat: &[u8], sample: &dyn Fn(u8, usize) -> bool, out: &mut Out) {
    let Some(&d0) = dat.first() else { return };
    let Some(s) = (0..edges.len().min(8)).find(|&k| !sample(d0, k)) else {
        return;
    };
    let width = if dat.len() == 4 && dat.iter().all(|&c| !sample(c, s)) {
        4
    } else {
        1
    };
    let lines = &dat[..width];
    // Size: the payload, or "N B" in the text.
    let size = req.data.map(|d| d.len()).or_else(|| {
        let (pre, _) = req.text.split_once(" B")?;
        pre.rsplit(' ').next()?.parse().ok()
    });
    let Some(size) = size else { return };
    let data_clocks = size * 8 / width;
    let total = 1 + data_clocks + 16 + 1;
    if edges.len() < s + total {
        return;
    }
    let bits: Vec<Vec<bool>> = lines.iter().map(|&c| (0..total).map(|k| sample(c, s + k)).collect()).collect();
    for (l, &c) in lines.iter().enumerate() {
        for k in 0..total {
            out.bit(c, cell[s + k], bits[l][k]);
        }
    }
    let span = |a: usize, b: usize| (cell[s + a].0, cell[s + b - 1].1);
    let (a, b) = span(0, 1);
    out.field(
        "frame",
        a,
        b,
        "S",
        format!("start bit (0 on {} line{})", width, if width > 1 { "s" } else { "" }),
        bits.iter().any(|l| l[0]),
    );
    let (a, b) = span(1, 1 + data_clocks);
    out.field(
        "frame",
        a,
        b,
        format!("data {size} B"),
        format!("{size} bytes, {width}-bit bus, most significant bit first"),
        false,
    );
    let mut crc_detail = Vec::new();
    let mut crc_bad = false;
    for (l, line) in bits.iter().enumerate() {
        let rx = sd::field(line, 1 + data_clocks, 16) as u16;
        let calc = sd::crc16(&line[1..1 + data_clocks]);
        crc_bad |= rx != calc;
        crc_detail.push(if rx == calc {
            format!("DAT{l} {rx:#06x} ok")
        } else {
            format!("DAT{l} received {rx:#06x}, computed {calc:#06x}")
        });
    }
    let (a, b) = span(1 + data_clocks, 1 + data_clocks + 16);
    out.field(
        "frame",
        a,
        b,
        if crc_bad { "CRC16 bad" } else { "CRC16 ok" },
        format!("CRC16 per line: {}", crc_detail.join(", ")),
        crc_bad,
    );
    let (a, b) = span(total - 1, total);
    out.field("frame", a, b, "E", "end bit (1)", bits.iter().any(|l| !l[total - 1]));
    // Bytes: in 4-bit mode two clocks per byte (DAT3..DAT0, high nibble
    // first); in 1-bit mode eight.
    let per = 8 / width;
    let mut bytes = Vec::with_capacity(size);
    for i in 0..size {
        let mut v = 0u8;
        for k in 0..per {
            for l in (0..width).rev() {
                v = v << 1 | bits[l][1 + i * per + k] as u8;
            }
        }
        bytes.push(v);
        let (a, b) = span(1 + i * per, 1 + (i + 1) * per);
        out.field("bytes", a, b, format!("{v:02x}"), format!("byte {i}: {v:#04x}{}", ascii(v)), false);
    }
    if req.text.starts_with("lock/unlock data") && bytes.len() >= 2 {
        let f = bytes[0];
        let (a, b) = span(1, 1 + per);
        out.field(
            "meaning",
            a,
            b,
            format!("flags {f:#04x}"),
            format!("lock/unlock flags: {}", sd::lock_data(&bytes[..1]).trim_start_matches(", ")),
            false,
        );
        let len = bytes[1] as usize;
        let (a, b) = span(1 + per, 1 + 2 * per);
        out.field(
            "meaning",
            a,
            b,
            format!("PWDS_LEN {len}"),
            format!("password length {len} bytes"),
            false,
        );
        let end = (2 + len).min(size);
        if end > 2 {
            let (a, b) = span(1 + 2 * per, 1 + end * per);
            let pwd = sd::lock_data(&bytes[..end]);
            out.field(
                "meaning",
                a,
                b,
                "password",
                format!("password{}", pwd.split(", password").nth(1).unwrap_or("")),
                false,
            );
        }
    }
}

// ------------------------------------------------------------------ UART

struct Format {
    data: u32,
    parity: Option<bool>, // Some(even)
    stop: u32,
}

fn parse_format(f: Option<&str>, iso: bool) -> Format {
    let default = if iso { "8E2" } else { "8N1" };
    let f = f.unwrap_or(default).as_bytes();
    let get = |s: &[u8]| -> Option<Format> {
        Some(Format {
            data: (*s.first()? as char).to_digit(10)?,
            parity: match s.get(1)? {
                b'N' => None,
                b'E' => Some(true),
                b'O' => Some(false),
                _ => return None,
            },
            stop: (*s.get(2)? as char).to_digit(10)?.clamp(1, 2),
        })
    };
    get(f).or_else(|| get(default.as_bytes())).unwrap()
}

fn uart_event(req: &Request, sig: &Signal, ch: u8, iso: bool, out: &mut Out) {
    let fmt = parse_format(req.format, iso);
    let nbits = 1 + fmt.data + fmt.parity.is_some() as u32 + fmt.stop;
    let t = req.text;
    let single = t == "BREAK"
        || (t.len() >= 2 && t.as_bytes()[..2].iter().all(|c| c.is_ascii_hexdigit()) && t.as_bytes().get(2).is_none_or(|&c| c == b' '));
    let bit = if single && !iso {
        (req.end - req.start) as f64 / nbits as f64
    } else if let Some(b) = req.bit_time {
        b
    } else {
        let e = sig.edges(ch, sig.start, sig.end);
        let widths: Vec<u64> = e.windows(2).map(|w| w[1].0 - w[0].0).collect();
        match estimate_bit_time(&widths, nbits as f64) {
            Some(b) => b,
            None => return,
        }
    };
    if bit < 2.0 {
        return;
    }
    let falls: Vec<u64> = sig
        .edges(ch, sig.start, sig.end)
        .into_iter()
        .filter(|e| !e.1)
        .map(|e| e.0)
        .collect();
    let mut from = req.start.saturating_sub((bit / 2.0) as u64);
    let mut n = 0;
    let mut chars = Vec::new();
    while let Some(&t0) = falls.iter().find(|&&f| f >= from) {
        if t0 >= req.end || n >= 4096 {
            break;
        }
        n += 1;
        let at = |k: u32| t0 + ((k as f64 + 0.5) * bit) as u64;
        let cell = |k: u32| (t0 + (k as f64 * bit) as u64, t0 + ((k + 1) as f64 * bit) as u64);
        let mut v = 0u32;
        let mut ones = 0;
        out.field("bits", cell(0).0, cell(0).1, "S", "start bit (0)", sig.level(ch, at(0)));
        out.bit(ch, cell(0), sig.level(ch, at(0)));
        for k in 0..fmt.data {
            let b = sig.level(ch, at(1 + k));
            v |= (b as u32) << k;
            ones += b as u32;
            out.bit(ch, cell(1 + k), b);
            out.field(
                "bits",
                cell(1 + k).0,
                cell(1 + k).1,
                format!("D{k}"),
                format!("data bit {k} (sent least significant first) = {}", b as u8),
                false,
            );
        }
        let mut k = 1 + fmt.data;
        let mut perr = false;
        if let Some(even) = fmt.parity {
            let p = sig.level(ch, at(k));
            perr = (ones + p as u32).is_multiple_of(2) != even;
            out.bit(ch, cell(k), p);
            out.field(
                "bits",
                cell(k).0,
                cell(k).1,
                "P",
                format!(
                    "{} parity bit = {}: {}",
                    if even { "even" } else { "odd" },
                    p as u8,
                    if perr { "mismatch" } else { "ok" }
                ),
                perr,
            );
            k += 1;
        }
        let mut ferr = false;
        for j in 0..fmt.stop {
            let s = sig.level(ch, at(k + j));
            ferr |= !s;
            out.bit(ch, cell(k + j), s);
            let what = if iso && j == 1 { "guard time (1)" } else { "stop bit (1)" };
            out.field("bits", cell(k + j).0, cell(k + j).1, "Sp", what, !s);
        }
        let (a, b) = (cell(0).0, cell(nbits - 1).1);
        let mut detail = format!("{v:#04x}{} — {} data bits LSB first", ascii(v as u8), fmt.data);
        if iso {
            detail += " (direct convention)";
        }
        if perr {
            detail += ", parity error";
        }
        if ferr {
            detail += ", framing error (stop bit low)";
        }
        out.field("chars", a, b, format!("{v:02x}"), detail, perr || ferr);
        chars.push((a, b, v as u8));
        // The next character starts after the first stop bit.
        from = at(nbits - fmt.stop) + 1;
    }
    if iso {
        iso_meaning(req.text, &chars, out);
    }
}

/// What each character of an ATR, a PPS or a T=1 block stands for.
fn iso_meaning(text: &str, chars: &[(u64, u64, u8)], out: &mut Out) {
    use crate::decode::iso7816::fidi;
    let mut put = |i: usize, j: usize, label: String, detail: String| {
        if j > i && j <= chars.len() {
            out.field("meaning", chars[i].0, chars[j - 1].1, label, detail, false);
        }
    };
    let b: Vec<u8> = chars.iter().map(|c| c.2).collect();
    if text.starts_with("ATR") && b.len() >= 2 {
        let conv = match b[0] {
            0x3b => "direct convention (high = 1, LSB first)",
            0x3f => "inverse convention (low = 1, MSB first)",
            _ => "unknown convention",
        };
        put(0, 1, "TS".into(), format!("initial character {:#04x}: {conv}", b[0]));
        let k = (b[1] & 15) as usize;
        let present = |y: u8, level: u32| {
            let names: Vec<String> = ["TA", "TB", "TC", "TD"]
                .iter()
                .enumerate()
                .filter(|(bit, _)| y >> bit & 1 != 0)
                .map(|(_, n)| format!("{n}{level}"))
                .collect();
            match names.len() {
                0 => "no interface bytes".to_string(),
                1 => names[0].clone() + " follows",
                _ => names.join(" ") + " follow",
            }
        };
        put(
            1,
            2,
            "T0".into(),
            format!("format byte {:#04x}: {}; {k} historical bytes", b[1], present(b[1] >> 4, 1)),
        );
        let (mut i, mut y, mut level, mut proto) = (2, b[1] >> 4, 1u32, 0u8);
        let mut tck = false;
        'walk: loop {
            for (bit, name) in [(0, "TA"), (1, "TB"), (2, "TC")] {
                if y >> bit & 1 == 0 {
                    continue;
                }
                let Some(&v) = b.get(i) else { break 'walk };
                let detail = match (name, level) {
                    ("TA", 1) => format!("clock rate and baud divisors: {}", fidi(v)),
                    ("TB", 1) => "programming voltage (deprecated)".into(),
                    ("TC", 1) => format!("extra guard time: {v} etu"),
                    ("TA", 2) => format!(
                        "specific mode, protocol T={}{}",
                        v & 15,
                        if v & 0x80 != 0 { ", cannot change" } else { "" }
                    ),
                    ("TC", 2) if proto == 0 => format!("T=0 waiting time integer WI = {v}"),
                    ("TA", _) if proto == 1 => format!("T=1 IFSC (card frame size) = {v} bytes"),
                    ("TB", _) if proto == 1 => format!("T=1 block waiting BWI = {}, character waiting CWI = {}", v >> 4, v & 15),
                    ("TC", _) if proto == 1 => format!("T=1 error detection: {}", if v & 1 != 0 { "CRC" } else { "LRC" }),
                    _ => format!("interface byte {v:#04x}"),
                };
                put(i, i + 1, format!("{name}{level}"), detail);
                i += 1;
            }
            if y & 8 == 0 {
                break;
            }
            let Some(&td) = b.get(i) else { break };
            proto = td & 15;
            tck |= proto != 0;
            put(
                i,
                i + 1,
                format!("TD{level}"),
                format!("protocol T={proto}; {}", present(td >> 4, level + 1)),
            );
            i += 1;
            y = td >> 4;
            level += 1;
        }
        if k > 0 && i + k <= b.len() {
            let hist = &b[i..i + k];
            let text: String = hist
                .iter()
                .map(|&c| if (0x20..0x7f).contains(&c) { c as char } else { '.' })
                .collect();
            let hex: Vec<String> = hist.iter().map(|x| format!("{x:02x}")).collect();
            put(
                i,
                i + k,
                format!("historical ({k})"),
                format!("historical bytes {} '{text}'", hex.join(" ")),
            );
            i += k;
        }
        if tck && i < b.len() {
            let ok = b[1..=i].iter().fold(0, |x, v| x ^ v) == 0;
            put(
                i,
                i + 1,
                "TCK".into(),
                format!(
                    "check byte {:#04x}: XOR of T0..TCK {}",
                    b[i],
                    if ok { "is 0, ok" } else { "is not 0 (mismatch)" }
                ),
            );
        }
    } else if text.starts_with("PPS") && b.len() >= 3 {
        put(0, 1, "PPSS".into(), "PPS start (FF)".into());
        let p0 = b[1];
        let mut opt = Vec::new();
        for (bit, n) in [(4, "PPS1"), (5, "PPS2"), (6, "PPS3")] {
            if p0 >> bit & 1 != 0 {
                opt.push(n);
            }
        }
        put(
            1,
            2,
            "PPS0".into(),
            format!(
                "protocol T={}; {}",
                p0 & 15,
                match opt.len() {
                    0 => "no parameter bytes".into(),
                    1 => opt[0].to_string() + " follows",
                    _ => opt.join(" ") + " follow",
                }
            ),
        );
        let mut i = 2;
        for n in opt {
            let Some(&v) = b.get(i) else { break };
            let detail = if n == "PPS1" {
                format!("clock rate and baud divisors: {}", fidi(v))
            } else {
                format!("{v:#04x}")
            };
            put(i, i + 1, n.into(), detail);
            i += 1;
        }
        if i < b.len() {
            let ok = b[..=i].iter().fold(0, |x, v| x ^ v) == 0;
            put(
                i,
                i + 1,
                "PCK".into(),
                format!(
                    "check byte: XOR of all PPS bytes {}",
                    if ok { "is 0, ok" } else { "is not 0 (mismatch)" }
                ),
            );
        }
    } else if text.starts_with("T=1") && b.len() >= 4 {
        let (nad, pcb, len) = (b[0], b[1], b[2] as usize);
        put(
            0,
            1,
            "NAD".into(),
            format!("node address {nad:#04x}: destination {}, source {}", nad >> 4 & 7, nad & 7),
        );
        let kind = match pcb >> 6 {
            0 | 1 => format!(
                "I-block (information), sequence {}{}",
                pcb >> 6 & 1,
                if pcb & 0x20 != 0 { ", more data follows" } else { "" }
            ),
            2 => format!(
                "R-block (receive ready), sequence {}{}",
                pcb >> 4 & 1,
                if pcb & 3 != 0 { ", error reported" } else { "" }
            ),
            _ => format!(
                "S-block {} {}",
                ["RESYNCH", "IFS", "ABORT", "WTX"].get((pcb & 3) as usize).copied().unwrap_or("?"),
                if pcb & 0x20 != 0 { "response" } else { "request" }
            ),
        };
        put(1, 2, "PCB".into(), format!("protocol control byte {pcb:#04x}: {kind}"));
        put(2, 3, format!("LEN {len}"), format!("{len} information bytes follow"));
        put(3, 3 + len, "INF".into(), "information field".into());
        if 3 + len < b.len() {
            let ok = b[..=3 + len].iter().fold(0, |x, v| x ^ v) == 0;
            put(
                3 + len,
                4 + len,
                "LRC".into(),
                format!("XOR of the block {}", if ok { "is 0, ok" } else { "is not 0 (mismatch)" }),
            );
        }
    }
}

// ------------------------------------------------------------------ SPI

fn spi_event(req: &Request, sig: &Signal, p: &SpiPins, out: &mut Out) {
    // As the decoder does: every clock edge counts (no glitch filter);
    // modes 0 and 3 sample on the rising edge, 1 and 2 on the falling one,
    // and words restart when chip select changes.
    let rising = matches!(p.mode, 0 | 3);
    let all: Vec<u64> = sig
        .edges(p.clk, req.start, req.end + 1)
        .into_iter()
        .filter(|e| e.1 == rising)
        .map(|e| e.0)
        .collect();
    // Chip select is active at the level most clock edges see.
    let cs_active = p.cs.map(|c| all.iter().filter(|&&e| sig.level(c, e)).count() * 2 > all.len());
    let selected = |e: u64| p.cs.zip(cs_active).is_none_or(|(c, a)| sig.level(c, e) == a);
    let edges: Vec<u64> = all.into_iter().filter(|&e| selected(e)).collect();
    if edges.is_empty() {
        return;
    }
    let cell = cells(&edges);
    let sample = |ch: u8, k: usize| sig.level(ch, edges[k]);
    for (k, c) in cell.iter().enumerate() {
        for ch in [p.mosi, p.miso].into_iter().flatten() {
            out.bit(ch, *c, sample(ch, k));
        }
    }
    // A chip select change between two edges restarts the word; without
    // chip select, so does a long pause in the clock.
    let gap = median_gap(&edges);
    let restart = |a: u64, b: u64| match p.cs {
        Some(c) => !sig.edges(c, a, b + 1).is_empty(),
        None => b - a > gap * 16,
    };
    let mut k = 0;
    while k + 8 <= edges.len() {
        if let Some(j) = (k + 1..k + 8).find(|&j| restart(edges[j - 1], edges[j])) {
            k = j;
            continue;
        }
        let word = |ch: u8| (k..k + 8).fold(0u8, |v, j| v << 1 | sample(ch, j) as u8);
        let mut label = Vec::new();
        let mut detail = Vec::new();
        if let Some(c) = p.mosi {
            let v = word(c);
            label.push(format!("{v:02x}"));
            detail.push(format!("MOSI {v:#04x}{}", ascii(v)));
        }
        if let Some(c) = p.miso {
            let v = word(c);
            label.push(format!("{v:02x}"));
            detail.push(format!("MISO {v:#04x}{}", ascii(v)));
        }
        if let Some(c) = p.dc {
            let d = sig.level(c, edges[k + 7]);
            detail.push(format!("D/C = {} ({})", d as u8, if d { "data" } else { "command" }));
        }
        out.field(
            "words",
            cell[k].0,
            cell[k + 7].1,
            label.join("/"),
            format!("{} — mode {}, MSB first", detail.join(", "), p.mode),
            false,
        );
        k += 8;
    }
}

// ------------------------------------------------------------------ I2C

fn i2c_event(req: &Request, sig: &Signal, scl: u8, sda: u8, out: &mut Out) {
    let t = req.text;
    if t == "START" || t == "STOP" {
        let detail = if t == "START" {
            "SDA falls while SCL is high"
        } else {
            "SDA rises while SCL is high"
        };
        out.field("frame", req.start, req.end.max(req.start + 1), t, detail, false);
        return;
    }
    let edges: Vec<u64> = clock_edges(sig, scl, true)
        .into_iter()
        .filter(|&e| e >= req.start && e < req.end + 2)
        .collect();
    if edges.len() < 9 {
        return;
    }
    let edges = &edges[..9];
    let cell = cells(edges);
    let bits: Vec<bool> = edges.iter().map(|&e| sig.level(sda, e)).collect();
    for (k, &b) in bits.iter().enumerate() {
        out.bit(sda, cell[k], b);
    }
    let v = bits[..8].iter().fold(0u8, |v, &b| v << 1 | b as u8);
    let ack = !bits[8];
    if t.starts_with("addr") {
        out.field(
            "frame",
            cell[0].0,
            cell[6].1,
            format!("{:#04x}", v >> 1),
            format!("7-bit address {:#04x}, MSB first", v >> 1),
            false,
        );
        out.field(
            "frame",
            cell[7].0,
            cell[7].1,
            if v & 1 == 1 { "R" } else { "W" },
            format!("direction bit {}: {}", v & 1, if v & 1 == 1 { "read" } else { "write" }),
            false,
        );
    } else {
        out.field(
            "frame",
            cell[0].0,
            cell[7].1,
            format!("{v:02x}"),
            format!("data byte {v:#04x}{}, MSB first", ascii(v)),
            false,
        );
    }
    out.field(
        "frame",
        cell[8].0,
        cell[8].1,
        if ack { "ACK" } else { "NAK" },
        format!(
            "acknowledge bit {}: {}",
            bits[8] as u8,
            if ack { "the receiver pulled SDA low" } else { "not acknowledged" }
        ),
        false,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_queries() {
        let mut s = Signal::new(100, 0b01);
        for at in 101..120 {
            s.push(at, if at >= 110 { 0b10 } else { 0b01 });
        }
        assert_eq!(s.changes, vec![(110, 0b10)]);
        assert!(s.level(0, 109) && !s.level(0, 110));
        assert_eq!(s.edges(1, 100, 120), vec![(110, true)]);
        assert_eq!(s.end, 120);
    }

    #[test]
    fn names() {
        assert_eq!(
            lines("SD clk=ch7 cmd=ch1 dat=ch0,ch2,ch4,ch6"),
            vec![(7, "CLK"), (1, "CMD"), (0, "DAT0"), (2, "DAT1"), (4, "DAT2"), (6, "DAT3")]
        );
        assert_eq!(lines("ISO7816 (UART ch3 auto)"), vec![(3, "I/O")]);
        assert_eq!(
            lines("ISO7816 clk=ch0 rst=ch1 (UART ch2)"),
            vec![(2, "I/O"), (0, "CLK"), (1, "RST")]
        );
        assert_eq!(lines("SSD1306 128x64 (SPI clk=ch1 mosi=ch2 cs=ch0 dc=ch3 mode 0)").len(), 4);
        assert_eq!(lines("I2C scl=ch4 sda=ch5"), vec![(4, "SCL"), (5, "SDA")]);
    }

    /// A UART 8N1 frame for 0x41 at 10 samples per bit.
    #[test]
    fn uart_frame() {
        let mut s = Signal::new(0, 1);
        let bits = [0, 1, 0, 0, 0, 0, 0, 1, 0, 1];
        for at in 1..200u64 {
            let k = (at as i64 - 50).div_euclid(10);
            let v = if (0..10).contains(&k) { bits[k as usize] } else { 1 };
            s.push(at, v);
        }
        let req = Request {
            source: "UART ch0 auto",
            start: 50,
            end: 150,
            text: "41 'A'",
            data: None,
            bit_time: None,
            format: None,
        };
        let f = inspect(&req, &s);
        let ch = f.iter().find(|f| f.place == Place::Row("chars")).unwrap();
        assert_eq!(ch.label, "41");
        assert!(!ch.bad);
        assert_eq!(f.iter().filter(|f| f.place == Place::Line(0)).count(), 10);
    }

    #[test]
    fn iso7816_roles() {
        let chars: Vec<(u64, u64, u8)> = [0x3b, 0x95, 0x13, 0x81, 0x01, 0x80, 0x73, 0xff, 0x01, 0x00, 0x0b]
            .iter()
            .enumerate()
            .map(|(i, &b)| (i as u64 * 12, i as u64 * 12 + 12, b))
            .collect();
        let mut out = Out::default();
        iso_meaning("ATR (direct convention)", &chars, &mut out);
        let labels: Vec<&str> = out.0.iter().map(|f| f.label.as_str()).collect();
        assert_eq!(labels, ["TS", "T0", "TA1", "TD1", "TD2", "historical (5)", "TCK"]);
        assert!(out.0[3].detail.contains("T=1"), "{:?}", out.0[3]);
        let mut out = Out::default();
        let pps: Vec<(u64, u64, u8)> = [0xff, 0x10, 0x95, 0x7a]
            .iter()
            .enumerate()
            .map(|(i, &b)| (i as u64, i as u64 + 1, b))
            .collect();
        iso_meaning("PPS request", &pps, &mut out);
        let labels: Vec<&str> = out.0.iter().map(|f| f.label.as_str()).collect();
        assert_eq!(labels, ["PPSS", "PPS0", "PPS1", "PCK"]);
        assert!(out.0[3].detail.contains("ok"));
    }

    /// An SD command frame (CMD17, arg 0x1000) on a synthetic bus.
    #[test]
    fn sd_command() {
        // CLK ch0 (period 10, rising at 10k+5), CMD ch1 changes at 10k.
        let mut bits = vec![false, true];
        bits.extend((0..6).rev().map(|i| 17 >> i & 1 == 1));
        bits.extend((0..32).rev().map(|i| 0x1000u32 >> i & 1 == 1));
        let crc = sd::crc7(&bits);
        bits.extend((0..7).rev().map(|i| crc >> i & 1 == 1));
        bits.push(true);
        let mut s = Signal::new(0, 0b10);
        for at in 1..1000u64 {
            let k = at / 10;
            let cmd = if (2..50).contains(&k) { bits[k as usize - 2] } else { true };
            s.push(at, (at % 10 >= 5) as u32 | (cmd as u32) << 1);
        }
        let req = Request {
            source: "SD clk=ch0 cmd=ch1 dat=ch2",
            start: 25,
            end: 505,
            text: "CMD17 READ_SINGLE_BLOCK block 4096",
            data: None,
            bit_time: None,
            format: None,
        };
        let f = inspect(&req, &s);
        let labels: Vec<&str> = f
            .iter()
            .filter(|f| f.place == Place::Row("frame"))
            .map(|f| f.label.as_str())
            .collect();
        assert_eq!(labels[..4], ["S", "host", "CMD17", "0x00001000"]);
        assert!(f.iter().all(|f| !f.bad), "{f:?}");
        assert!(f.iter().any(|f| f.detail.contains("block 4096")));
    }
}
