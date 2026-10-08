//! Value Change Dump (IEEE 1364 VCD): the standard waveform interchange
//! format (GTKWave, simulators, PulseView import). Only changes are stored,
//! so it is compact for logic signals and grows with edge count.

use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::Path;

use crate::block::{Block, Sample, channel_mask};
use crate::edges::{EdgeDetector, Transition};
use crate::source::{CaptureInfo, Source};

fn invalid(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

/// VCD time units, in femtoseconds.
const UNITS: [(&str, u128); 6] = [
    ("s", 1_000_000_000_000_000),
    ("ms", 1_000_000_000_000),
    ("us", 1_000_000_000),
    ("ns", 1_000_000),
    ("ps", 1_000),
    ("fs", 1),
];

/// Identifier for channel `i` (printable ASCII, as VCD wants).
fn ident(i: usize) -> String {
    char::from(b'!' + i as u8).to_string()
}

/// Writes a VCD file from sample blocks.
pub struct VcdWriter<W: Write> {
    out: W,
    det: EdgeDetector,
    channels: usize,
    /// Timestamp units per sample.
    step: u128,
    started: bool,
    samples: u64,
    pos: u64,
    raw: u64,
    unit_size: usize,
    tr: Vec<Transition>,
}

impl VcdWriter<BufWriter<File>> {
    /// Creates `path`.
    pub fn create(path: impl AsRef<Path>, info: &CaptureInfo) -> io::Result<VcdWriter<BufWriter<File>>> {
        VcdWriter::new(BufWriter::with_capacity(1 << 20, File::create(path)?), info)
    }
}

impl<W: Write> VcdWriter<W> {
    /// Writes the header to `out`.
    pub fn new(mut out: W, info: &CaptureInfo) -> io::Result<VcdWriter<W>> {
        // Pick the largest time unit that divides the sample period exactly.
        let period_fs = 1_000_000_000_000_000u128 / info.samplerate.max(1) as u128;
        let exact = 1_000_000_000_000_000u128.is_multiple_of(info.samplerate.max(1) as u128);
        let (unit, unit_fs) = UNITS
            .iter()
            .copied()
            .find(|&(_, u)| exact && period_fs.is_multiple_of(u))
            .unwrap_or(("fs", 1));
        let step = period_fs / unit_fs;
        let mut h = String::new();
        h += "$version visgrok $end\n";
        h += &format!(
            "$comment device={} samplerate={} $end\n",
            info.device.replace('$', ""),
            info.samplerate
        );
        h += &format!("$timescale 1 {unit} $end\n");
        h += "$scope module logic $end\n";
        for i in 0..info.channels {
            let name: String = info.name(i).chars().map(|c| if c.is_whitespace() { '_' } else { c }).collect();
            h += &format!("$var wire 1 {} {name} $end\n", ident(i));
        }
        h += "$upscope $end\n$enddefinitions $end\n";
        out.write_all(h.as_bytes())?;
        Ok(VcdWriter {
            out,
            det: EdgeDetector::new(channel_mask(info.channels)),
            channels: info.channels,
            step,
            started: false,
            samples: 0,
            pos: h.len() as u64,
            raw: 0,
            unit_size: info.unit_size,
            tr: Vec::new(),
        })
    }

    fn put(&mut self, s: &str) -> io::Result<()> {
        self.out.write_all(s.as_bytes())?;
        self.pos += s.len() as u64;
        Ok(())
    }

    /// Appends packed samples.
    pub fn write(&mut self, data: &[u8]) -> io::Result<()> {
        let block = Block::new(self.samples, self.unit_size, data.to_vec());
        if block.is_empty() {
            return Ok(());
        }
        if !self.started {
            self.started = true;
            let first = block.sample(0);
            let mut s = String::from("#0\n$dumpvars\n");
            for i in 0..self.channels {
                s += &format!("{}{}\n", first >> i & 1, ident(i));
            }
            s += "$end\n";
            self.put(&s)?;
        }
        self.tr.clear();
        self.det.process(&block, &mut self.tr);
        let mut s = String::new();
        for t in &self.tr {
            s += &format!("#{}\n", t.at as u128 * self.step);
            let mut c = t.changed();
            while c != 0 {
                let i = c.trailing_zeros() as usize;
                c &= c - 1;
                s += &format!("{}{}\n", t.now >> i & 1, ident(i));
            }
        }
        self.put(&s)?;
        self.samples = block.end();
        self.raw += data.len() as u64;
        Ok(())
    }

    /// Bytes written.
    pub fn bytes_written(&self) -> u64 {
        self.pos
    }

    /// Raw sample bytes consumed.
    pub fn raw_written(&self) -> u64 {
        self.raw
    }

    /// Writes the final timestamp (so the duration survives) and flushes.
    pub fn finish(mut self) -> io::Result<W> {
        let end = format!("#{}\n", self.samples as u128 * self.step);
        self.put(&end)?;
        self.out.flush()?;
        Ok(self.out)
    }
}

/// Reads a VCD file as sample blocks. The sample rate comes from a
/// `samplerate=` comment (as written by visgrok), the caller, or the
/// greatest common divisor of all timestamps.
pub struct VcdReader {
    lines: io::Lines<BufReader<File>>,
    info: CaptureInfo,
    /// Map from VCD identifier to channel.
    ids: Vec<(String, usize)>,
    /// Timestamp units per sample.
    step: u128,
    state: Sample,
    /// Sample index up to which `state` has been emitted.
    emitted: u64,
    /// Pending timestamp (in samples) whose changes are being collected.
    at: u64,
    /// Tokens of the current line not processed yet.
    tokens: std::collections::VecDeque<String>,
    done: bool,
}

const BLOCK: u64 = 1 << 20;

struct Header {
    unit_fs: u128,
    ids: Vec<(String, usize)>,
    names: Vec<String>,
    rate: Option<u64>,
    device: Option<String>,
}

fn parse_header(lines: &mut io::Lines<BufReader<File>>) -> io::Result<Header> {
    let mut text = String::new();
    for line in lines.by_ref() {
        let line = line?;
        text.push_str(&line);
        text.push('\n');
        if line.contains("$enddefinitions") {
            break;
        }
    }
    let mut h = Header {
        unit_fs: 1_000_000,
        ids: Vec::new(),
        names: Vec::new(),
        rate: None,
        device: None,
    };
    let toks: Vec<&str> = text.split_whitespace().collect();
    let mut i = 0;
    while i < toks.len() {
        match toks[i] {
            "$timescale" => {
                let mut spec = String::new();
                i += 1;
                while i < toks.len() && toks[i] != "$end" {
                    spec += toks[i];
                    i += 1;
                }
                let digits: String = spec.chars().take_while(|c| c.is_ascii_digit()).collect();
                let unit = &spec[digits.len()..];
                let mult: u128 = digits.parse().unwrap_or(1);
                let u = UNITS
                    .iter()
                    .find(|(n, _)| *n == unit)
                    .map(|u| u.1)
                    .ok_or_else(|| invalid(format!("bad timescale {spec:?}")))?;
                h.unit_fs = mult * u;
            }
            "$var" => {
                // $var <type> <size> <id> <name> [range] $end
                if i + 4 < toks.len() && toks[i + 2] == "1" {
                    if h.ids.len() >= 32 {
                        return Err(invalid("more than 32 one-bit signals"));
                    }
                    let ch = h.ids.len();
                    h.ids.push((toks[i + 3].to_string(), ch));
                    h.names.push(toks[i + 4].to_string());
                }
                while i < toks.len() && toks[i] != "$end" {
                    i += 1;
                }
            }
            "$comment" => {
                i += 1;
                while i < toks.len() && toks[i] != "$end" {
                    if let Some(v) = toks[i].strip_prefix("samplerate=") {
                        h.rate = v.parse().ok();
                    }
                    if let Some(v) = toks[i].strip_prefix("device=") {
                        h.device = Some(v.to_string());
                    }
                    i += 1;
                }
            }
            _ => {}
        }
        i += 1;
    }
    if h.ids.is_empty() {
        return Err(invalid("no 1-bit signals in VCD"));
    }
    Ok(h)
}

fn gcd(a: u128, b: u128) -> u128 {
    if b == 0 { a } else { gcd(b, a % b) }
}

impl VcdReader {
    /// Opens a VCD file. `samplerate` overrides the detected rate.
    pub fn open(path: impl AsRef<Path>, samplerate: Option<u64>) -> io::Result<VcdReader> {
        let path = path.as_ref();
        let mut lines = BufReader::with_capacity(1 << 20, File::open(path)?).lines();
        let h = parse_header(&mut lines)?;
        let rate = match samplerate.or(h.rate) {
            Some(r) => r,
            None => {
                // First pass: the GCD of all timestamps is the sample period.
                let mut g = 0u128;
                for line in lines {
                    let line = line?;
                    if let Some(t) = line.trim().strip_prefix('#').and_then(|t| t.parse::<u128>().ok()) {
                        g = gcd(g, t);
                    }
                }
                let period_fs = g.max(1) * h.unit_fs;
                (1_000_000_000_000_000u128 / period_fs).max(1) as u64
            }
        };
        let mut lines = BufReader::with_capacity(1 << 20, File::open(path)?).lines();
        parse_header(&mut lines)?;
        let period_fs = 1_000_000_000_000_000u128 / rate as u128;
        let step = (period_fs / h.unit_fs).max(1);
        let channels = h.ids.len();
        Ok(VcdReader {
            lines,
            info: CaptureInfo {
                device: h.device.unwrap_or_else(|| "VCD".into()),
                channels,
                samplerate: rate,
                unit_size: crate::block::unit_size_for(channels),
                names: h.names,
            },
            ids: h.ids,
            step,
            state: 0,
            emitted: 0,
            at: 0,
            tokens: Default::default(),
            done: false,
        })
    }

    /// Emits `state` for samples `emitted..to`, at most one block's worth.
    fn fill(&mut self, to: u64) -> Option<Block> {
        if to <= self.emitted {
            return None;
        }
        let n = (to - self.emitted).min(BLOCK);
        let unit = self.info.unit_size;
        let bytes = self.state.to_le_bytes();
        let mut data = Vec::with_capacity(n as usize * unit);
        for _ in 0..n {
            data.extend_from_slice(&bytes[..unit]);
        }
        let b = Block::new(self.emitted, unit, data);
        self.emitted += n;
        Some(b)
    }
}

impl Source for VcdReader {
    fn info(&self) -> CaptureInfo {
        self.info.clone()
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        loop {
            // Output the current state up to the pending timestamp first.
            if let Some(b) = self.fill(self.at) {
                return Ok(Some(b));
            }
            if self.done {
                return Ok(None);
            }
            let Some(tok) = self.tokens.pop_front() else {
                match self.lines.next() {
                    Some(line) => self.tokens.extend(line?.split_whitespace().map(str::to_string)),
                    None => self.done = true,
                }
                continue;
            };
            if let Some(t) = tok.strip_prefix('#') {
                // A new time: everything before it goes out with the old
                // state (on the next loop turn) before applying changes.
                if let Ok(t) = t.parse::<u128>() {
                    self.at = (t / self.step) as u64;
                }
            } else if let Some(v) = tok.chars().next().filter(|c| "01xXzZ".contains(*c)) {
                let id = &tok[1..];
                if let Some(&(_, ch)) = self.ids.iter().find(|(i, _)| i == id) {
                    if v == '1' {
                        self.state |= 1 << ch;
                    } else {
                        self.state &= !(1 << ch);
                    }
                }
            }
        }
    }

    fn stop(&mut self) {
        self.done = true;
        self.at = self.emitted;
    }
}
