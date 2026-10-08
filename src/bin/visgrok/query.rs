//! One-shot commands that answer questions about a recorded capture, for
//! people and for scripts or agents: what is on each channel, what the
//! decoders found (with time ranges, filters and JSON output), one event in
//! detail, raw edges and levels, clock analysis, decoded byte streams and
//! display screens as PNG files.
//!
//! Decoding uses the roles and decoder settings from the capture's sidecar
//! (overridable with `--role` and the decoder options). The result is cached
//! next to the capture (`<capture>.events`) and reused while the capture
//! and the settings stay the same.

// Per-channel tables are indexed by channel number throughout.
#![allow(clippy::needless_range_loop)]

use std::fmt::Write as _;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use visgrok::analyzer::{Analyzer, DecoderOptions, SpiProtocol, UartProtocol, parse_uart_format};
use visgrok::block::Sample;
use visgrok::decode::Event;
use visgrok::formats::ReadOptions;
use visgrok::inspect::{self, Place, Request, Signal};
use visgrok::json::{Json, jstr};
use visgrok::roles::{Role, fmt_hz};
use visgrok::sidecar::Sidecar;
use visgrok::store::{SampleStore, TILE};
use visgrok::{Block, CaptureInfo, EdgeDetector, Transition};

/// The capture and how to decode it.
#[derive(clap::Args, Debug, Clone)]
pub struct Cap {
    /// Capture file (.vgk, .sr, .vcd).
    pub file: PathBuf,
    /// Channel role, overriding the sidecar: CH=ROLE, where CH is a number or
    /// a channel name, e.g. `-r UART=iso-io -r 0=iso-clk -r SCLK=none`.
    /// Roles: uart, uart:BAUD, spi-clk, spi-mosi, spi-miso, spi-cs, spi-dc,
    /// i2c-scl:SDA, i2c-sda:SCL, sd-clk, sd-cmd, sd-dat0..3, iso-io, iso-clk,
    /// iso-rst, none.
    #[arg(short, long = "role", value_name = "CH=ROLE")]
    pub roles: Vec<String>,
    /// Ignore the roles and decoder settings saved in the capture's sidecar.
    #[arg(long)]
    pub no_sidecar: bool,
    /// Protocol on top of SPI: raw, ssd1306, ssd1306:128x32.
    #[arg(long, value_parser = SpiProtocol::parse)]
    pub spi_proto: Option<SpiProtocol>,
    /// SPI mode 0..3.
    #[arg(long)]
    pub spi_mode: Option<u8>,
    /// Protocol on top of UART: raw, iso7816.
    #[arg(long, value_parser = UartProtocol::parse)]
    pub uart_proto: Option<UartProtocol>,
    /// UART frame format: auto, 8N1, 8E2, ...
    #[arg(long)]
    pub uart_format: Option<String>,
    /// Decode again instead of using the cached result.
    #[arg(long)]
    pub no_cache: bool,
}

/// A time range.
#[derive(clap::Args, Debug, Clone, Default)]
pub struct Range {
    /// Start: seconds (`39.67`), with a unit (`39675ms`, `120us`, `50ns`) or
    /// a sample index (`#7934000`).
    #[arg(long)]
    pub from: Option<String>,
    /// End, in the same forms as --from.
    #[arg(long)]
    pub to: Option<String>,
    /// Only session N: from a reset release to the next reset (needs a reset
    /// line role such as iso-rst).
    #[arg(long)]
    pub session: Option<u32>,
}

/// Query commands.
#[derive(clap::Subcommand, Debug)]
pub enum Query {
    /// What a capture contains: channels and their activity, buses, decoders
    /// with event counts, sessions.
    Summary {
        #[command(flatten)]
        cap: Cap,
        /// Skip decoding (channel information only).
        #[arg(long)]
        no_decode: bool,
        /// Print JSON.
        #[arg(long)]
        json: bool,
    },
    /// Sessions delimited by a reset line (e.g. smart card resets).
    Sessions {
        #[command(flatten)]
        cap: Cap,
        /// Print JSON lines.
        #[arg(long)]
        json: bool,
    },
    /// Decoded events, optionally in a time range, from one decoder or
    /// matching a text.
    Events {
        #[command(flatten)]
        cap: Cap,
        #[command(flatten)]
        range: Range,
        /// Only events from decoders whose name contains this (e.g. ISO, SPI, UART ch2).
        #[arg(short, long)]
        decoder: Option<String>,
        /// Only events whose text contains this (case-insensitive).
        #[arg(short, long)]
        grep: Option<String>,
        /// Leave out rate/format/session notes.
        #[arg(long)]
        no_notes: bool,
        /// At most this many events.
        #[arg(short = 'n', long)]
        limit: Option<usize>,
        /// Print JSON lines (one event per line).
        #[arg(long)]
        json: bool,
    },
    /// One event in detail: full description, payload, and every bit and
    /// field of it on the lines it was decoded from.
    Event {
        #[command(flatten)]
        cap: Cap,
        /// Time inside the event (or just before it), as in --from.
        #[arg(long)]
        at: String,
        /// Only consider events from decoders whose name contains this.
        #[arg(short, long)]
        decoder: Option<String>,
        /// Print JSON.
        #[arg(long)]
        json: bool,
    },
    /// Decoded byte streams: UART / smart card characters and SPI words,
    /// grouped into bursts (and by D/C for SPI).
    Data {
        #[command(flatten)]
        cap: Cap,
        #[command(flatten)]
        range: Range,
        /// Only decoders whose name contains this.
        #[arg(short, long)]
        decoder: Option<String>,
        /// Print JSON lines (one burst per line).
        #[arg(long)]
        json: bool,
    },
    /// Writes every distinct display screen (e.g. SSD1306 over SPI) as a PNG.
    Screens {
        #[command(flatten)]
        cap: Cap,
        #[command(flatten)]
        range: Range,
        /// Output directory.
        #[arg(short, long, default_value = "screens")]
        out: PathBuf,
        /// Pixel size in the PNG.
        #[arg(long, default_value_t = 4)]
        scale: u32,
        /// Also write screens identical to the previous one.
        #[arg(long)]
        all: bool,
        /// Print JSON lines.
        #[arg(long)]
        json: bool,
    },
    /// Raw transitions on some channels.
    Edges {
        #[command(flatten)]
        cap: Cap,
        #[command(flatten)]
        range: Range,
        /// Channels (numbers or names); all when omitted.
        #[arg(short, long = "ch")]
        channels: Vec<String>,
        /// At most this many transitions.
        #[arg(short = 'n', long, default_value_t = 1000)]
        limit: usize,
        /// Print JSON lines.
        #[arg(long)]
        json: bool,
    },
    /// Channel levels at one moment.
    Levels {
        #[command(flatten)]
        cap: Cap,
        /// Time, as in --from.
        #[arg(long)]
        at: String,
        /// Print JSON.
        #[arg(long)]
        json: bool,
    },
    /// Clock analysis of one channel: when it runs, frequency, period
    /// spread, duty cycle and drift.
    Clock {
        #[command(flatten)]
        cap: Cap,
        #[command(flatten)]
        range: Range,
        /// Channel (number or name).
        #[arg(long = "ch")]
        channel: String,
        /// Print JSON.
        #[arg(long)]
        json: bool,
    },
}

/// Runs a query command, exiting with an error message on failure.
pub fn run(q: &Query) {
    if let Err(e) = dispatch(q) {
        eprintln!("visgrok: {e}");
        std::process::exit(1);
    }
}

fn dispatch(q: &Query) -> Result<(), String> {
    match q {
        Query::Summary { cap, no_decode, json } => summary(&Ctx::open(cap)?, !no_decode, *json),
        Query::Sessions { cap, json } => {
            let ctx = Ctx::open(cap)?;
            let d = ctx.decoded()?;
            sessions_cmd(&ctx, &d, *json)
        }
        Query::Events {
            cap,
            range,
            decoder,
            grep,
            no_notes,
            limit,
            json,
        } => {
            let ctx = Ctx::open(cap)?;
            let d = ctx.decoded()?;
            let (from, to) = ctx.range(range, Some(&d))?;
            let grep = grep.as_ref().map(|g| g.to_lowercase());
            let mut n = 0;
            let mut out = io::stdout().lock();
            for e in d.events.iter().filter(|e| e.end.max(e.start + 1) > from && e.start < to) {
                if !d.matches(e, decoder.as_deref()) || (*no_notes && e.kind == Kind::Note) {
                    continue;
                }
                if grep.as_ref().is_some_and(|g| !e.text.to_lowercase().contains(g)) {
                    continue;
                }
                if limit.is_some_and(|l| n >= l) {
                    eprintln!("visgrok: stopped at --limit {n}");
                    break;
                }
                n += 1;
                let line = if *json { ctx.event_json(&d, e) } else { ctx.event_line(&d, e) };
                if writeln!(out, "{line}").is_err() {
                    break;
                }
            }
            if !*json {
                eprintln!("{n} events");
            }
            Ok(())
        }
        Query::Event { cap, at, decoder, json } => {
            let ctx = Ctx::open(cap)?;
            let d = ctx.decoded()?;
            let at = ctx.sample(at)?;
            event_cmd(&ctx, &d, at, decoder.as_deref(), *json)
        }
        Query::Data { cap, range, decoder, json } => {
            let ctx = Ctx::open(cap)?;
            let d = ctx.decoded()?;
            let r = ctx.range(range, Some(&d))?;
            data_cmd(&ctx, &d, r, decoder.as_deref(), *json)
        }
        Query::Screens {
            cap,
            range,
            out,
            scale,
            all,
            json,
        } => {
            let ctx = Ctx::open(cap)?;
            let d = ctx.decoded()?;
            let r = ctx.range(range, Some(&d))?;
            screens_cmd(&ctx, &d, r, out, (*scale).clamp(1, 64), *all, *json)
        }
        Query::Edges {
            cap,
            range,
            channels,
            limit,
            json,
        } => {
            let ctx = Ctx::open(cap)?;
            let d = if range.session.is_some() { Some(ctx.decoded()?) } else { None };
            let r = ctx.range(range, d.as_ref())?;
            let mask = if channels.is_empty() {
                visgrok::block::channel_mask(ctx.info.channels)
            } else {
                channels
                    .iter()
                    .map(|c| ctx.channel(c).map(|c| 1 << c))
                    .sum::<Result<Sample, String>>()?
            };
            edges_cmd(&ctx, r, mask, *limit, *json)
        }
        Query::Levels { cap, at, json } => {
            let ctx = Ctx::open(cap)?;
            let at = ctx.sample(at)?;
            levels_cmd(&ctx, at, *json)
        }
        Query::Clock { cap, range, channel, json } => {
            let ctx = Ctx::open(cap)?;
            let d = if range.session.is_some() { Some(ctx.decoded()?) } else { None };
            let r = ctx.range(range, d.as_ref())?;
            let ch = ctx.channel(channel)?;
            clock_cmd(&ctx, r, ch, *json)
        }
    }
}

// ---------------------------------------------------------------- context

/// An opened capture with its roles and decoder settings.
struct Ctx {
    file: PathBuf,
    info: CaptureInfo,
    names: Vec<String>,
    roles: Vec<Role>,
    opts: DecoderOptions,
    /// Random access, for .vgk files.
    store: Option<Arc<SampleStore>>,
    /// Total samples, when known without reading everything.
    total: Option<u64>,
    no_cache: bool,
}

impl Ctx {
    fn open(cap: &Cap) -> Result<Ctx, String> {
        let file = cap.file.clone();
        let fail = |e: io::Error| format!("{}: {e}", file.display());
        let src = visgrok::formats::open(&file, &ReadOptions::default()).map_err(fail)?;
        let info = src.info();
        drop(src);
        let store = SampleStore::open(&file).ok();
        let total = store.as_ref().map(|s| s.total());
        let sc = if cap.no_sidecar {
            None
        } else {
            Sidecar::load(&file).map_err(|e| e.to_string())?
        };
        let n = info.channels;
        let mut names: Vec<String> = (0..n).map(|i| info.name(i)).collect();
        let mut roles = vec![Role::Unknown; n];
        let mut opts = DecoderOptions::default();
        if let Some(sc) = &sc {
            for (i, nm) in sc.names.iter().enumerate().take(n) {
                if !nm.is_empty() {
                    names[i] = nm.clone();
                }
            }
            roles = sc.effective_roles(n);
            opts = sc.options.clone();
        }
        let mut ctx = Ctx {
            file,
            info,
            names,
            roles,
            opts,
            store,
            total,
            no_cache: cap.no_cache,
        };
        for spec in &cap.roles {
            let (ch, r) = spec.split_once('=').ok_or_else(|| format!("--role {spec:?}: expected CH=ROLE"))?;
            let ch = ctx.channel(ch)? as usize;
            ctx.roles[ch] = match r.trim() {
                "" | "-" | "none" | "unknown" => Role::Unknown,
                r => Role::parse(r)?,
            };
        }
        if let Some(p) = cap.spi_proto {
            ctx.opts.spi_protocol = p;
        }
        if cap.spi_mode.is_some() {
            ctx.opts.spi_mode = cap.spi_mode;
        }
        if let Some(p) = cap.uart_proto {
            ctx.opts.uart_protocol = p;
        }
        if let Some(f) = &cap.uart_format {
            ctx.opts.uart_format = parse_uart_format(f)?;
        }
        Ok(ctx)
    }

    fn sr(&self) -> f64 {
        self.info.samplerate as f64
    }

    fn secs(&self, s: u64) -> f64 {
        s as f64 / self.sr()
    }

    /// A channel by number (`3`, `D3`, `ch3`) or name.
    fn channel(&self, spec: &str) -> Result<u8, String> {
        let s = spec.trim();
        let num = s.trim_start_matches("ch").trim_start_matches(['D', 'd']);
        if let Ok(n) = num.parse::<usize>()
            && n < self.info.channels
        {
            return Ok(n as u8);
        }
        self.names
            .iter()
            .position(|n| n.eq_ignore_ascii_case(s))
            .map(|i| i as u8)
            .ok_or_else(|| format!("no channel {spec:?} (channels: {})", self.names.join(", ")))
    }

    /// A time as a sample index (see [`Range::from`]).
    fn sample(&self, spec: &str) -> Result<u64, String> {
        let s = spec.trim();
        if let Some(n) = s.strip_prefix('#') {
            return n.parse().map_err(|_| format!("bad sample index {spec:?}"));
        }
        let (num, mul) = [("ns", 1e-9), ("us", 1e-6), ("µs", 1e-6), ("ms", 1e-3), ("s", 1.0)]
            .iter()
            .find_map(|(u, m)| s.strip_suffix(u).map(|n| (n, *m)))
            .unwrap_or((s, 1.0));
        let v: f64 = num
            .trim()
            .parse()
            .map_err(|_| format!("bad time {spec:?} (e.g. 39.67, 120ms, #1000)"))?;
        Ok((v * mul * self.sr()).round().max(0.0) as u64)
    }

    /// The samples a range covers.
    fn range(&self, r: &Range, d: Option<&Decoded>) -> Result<(u64, u64), String> {
        let (mut from, mut to) = (0, u64::MAX);
        if let Some(n) = r.session {
            let d = d.ok_or("--session needs decoding")?;
            let s = d
                .sessions()
                .into_iter()
                .find(|s| s.n == n)
                .ok_or_else(|| format!("no session {n} (assign a reset role, e.g. -r RESET=iso-rst; see `visgrok sessions`)"))?;
            from = s.start;
            to = s.end.unwrap_or(u64::MAX);
        }
        if let Some(f) = &r.from {
            from = from.max(self.sample(f)?);
        }
        if let Some(t) = &r.to {
            to = to.min(self.sample(t)?);
        }
        if to <= from {
            return Err("empty time range".into());
        }
        Ok((from, to))
    }

    fn fmt_t(&self, s: u64) -> String {
        format!("{:.6}", self.secs(s))
    }

    // ------------------------------------------------------------ samples

    /// Calls `f` with consecutive blocks covering `from..to` (clipped to the
    /// capture); stops when `f` returns false.
    fn blocks(&self, from: u64, to: u64, mut f: impl FnMut(&Block) -> bool) -> Result<(), String> {
        const STEP: u64 = 1 << 22;
        let err = |e: io::Error| format!("{}: {e}", self.file.display());
        if let Some(st) = &self.store {
            let to = to.min(st.total());
            let unit = self.info.unit_size;
            let mut at = from;
            while at < to {
                let (got, data) = st.read(at, to.min(at + STEP)).map_err(err)?;
                if data.is_empty() {
                    break;
                }
                let b = Block::new(got, unit, data);
                at = b.end();
                if !f(&b) {
                    break;
                }
            }
            return Ok(());
        }
        let mut src = visgrok::formats::open(&self.file, &ReadOptions::default()).map_err(err)?;
        while let Some(b) = src.next_block().map_err(err)? {
            if b.end() <= from {
                continue;
            }
            if b.start >= to {
                break;
            }
            let a = from.saturating_sub(b.start) as usize;
            let z = (to.min(b.end()) - b.start) as usize;
            let part = if a == 0 && z == b.len() { b } else { b.slice(a, z) };
            if !f(&part) {
                break;
            }
        }
        Ok(())
    }

    /// Transitions of the channels in `mask` in `from..to`, with the state
    /// at the first covered sample. `f` returns false to stop.
    fn transitions(&self, from: u64, to: u64, mask: Sample, mut f: impl FnMut(&Transition) -> bool) -> Result<Option<Sample>, String> {
        let mut det = EdgeDetector::new(mask);
        let mut buf = Vec::new();
        let mut init = None;
        let mut last: Option<Sample> = None;
        self.blocks(from, to, |b| {
            if init.is_none() && !b.is_empty() {
                init = Some(b.sample(0) & mask);
                last = init;
            }
            buf.clear();
            det.process(b, &mut buf);
            for t in &buf {
                if t.changed() & mask == 0 {
                    continue;
                }
                let prev = last.unwrap_or(t.prev & mask);
                let now = t.now & mask;
                last = Some(now);
                if !f(&Transition { at: t.at, prev, now }) {
                    return false;
                }
            }
            true
        })?;
        Ok(init)
    }

    /// The channels in `mask` over `from..to` as a [`Signal`].
    fn signal(&self, from: u64, to: u64, mask: Sample) -> Result<Signal, String> {
        if let Some(st) = &self.store {
            return st.signal(from, to, mask).map_err(|e| e.to_string());
        }
        let mut sig: Option<Signal> = None;
        self.blocks(from, to, |b| {
            for (i, s) in b.samples().enumerate() {
                let at = b.start + i as u64;
                match &mut sig {
                    None => sig = Some(Signal::new(at, s & mask)),
                    Some(g) => g.push(at, s & mask),
                }
            }
            true
        })?;
        sig.ok_or_else(|| "no samples there".to_string())
    }

    // ------------------------------------------------------------ decoding

    fn cache_path(&self) -> PathBuf {
        let mut p = self.file.clone().into_os_string();
        p.push(".events");
        PathBuf::from(p)
    }

    /// What the cache must match: the program, the file and the settings.
    fn cache_key(&self) -> String {
        let meta = std::fs::metadata(&self.file).ok();
        let mtime = meta
            .as_ref()
            .and_then(|m| m.modified().ok())
            .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
            .map_or(0, |d| d.as_secs());
        let roles: Vec<String> = self.roles.iter().map(|r| r.id()).collect();
        format!(
            "visgrok {} | {} bytes {} | roles {} | {:?}",
            env!("CARGO_PKG_VERSION"),
            meta.map_or(0, |m| m.len()),
            mtime,
            roles.join(","),
            self.opts
        )
    }

    /// Decoded events, from the cache or by decoding the capture.
    fn decoded(&self) -> Result<Decoded, String> {
        let key = self.cache_key();
        if !self.no_cache
            && let Some(d) = Decoded::load(&self.cache_path(), &key)
        {
            return Ok(d);
        }
        let d = self.decode()?;
        if let Err(e) = d.save(&self.cache_path(), &key) {
            eprintln!("visgrok: not caching events: {e}");
        }
        Ok(d)
    }

    fn decode(&self) -> Result<Decoded, String> {
        let mut a = Analyzer::new(self.info.channels, self.info.samplerate);
        let decs = a.decoders_for_roles(&self.roles, &self.opts);
        let mut d = Decoded {
            decoders: decs.iter().map(|x| x.name()).collect(),
            ..Default::default()
        };
        let lowest: Vec<u8> = decs.iter().map(|x| x.channels().trailing_zeros() as u8).collect();
        if decs.is_empty() {
            eprintln!("visgrok: no decoders (assign roles with -r CH=ROLE, or save them in the web UI)");
            return Ok(d);
        }
        a.set_decoders(decs);
        let t0 = Instant::now();
        let mut shown = Instant::now();
        let total = self.total;
        let mut src = visgrok::formats::open(&self.file, &ReadOptions::default()).map_err(|e| e.to_string())?;
        while let Some(b) = src.next_block().map_err(|e| e.to_string())? {
            let before = a.annotation_count;
            a.process(&b);
            let new = ((a.annotation_count - before) as usize).min(a.annotations.len());
            let skip = a.annotations.len() - new;
            for t in a.annotations.iter().skip(skip) {
                d.add(&t.annotation, t.decoder as u16, lowest.get(t.decoder).copied().unwrap_or(0));
            }
            if shown.elapsed().as_secs_f64() > 1.0 {
                shown = Instant::now();
                match total {
                    Some(t) if t > 0 => eprint!("\rdecoding {:.0}%...", b.end() as f64 * 100.0 / t as f64),
                    _ => eprint!("\rdecoding {:.1} s...", self.secs(b.end())),
                }
            }
        }
        if t0.elapsed().as_secs_f64() > 1.0 {
            eprintln!("\rdecoded {} events in {:.1} s", d.events.len(), t0.elapsed().as_secs_f64());
        }
        Ok(d)
    }

    // ------------------------------------------------------------ output

    fn event_line(&self, d: &Decoded, e: &Ev) -> String {
        format!(
            "{:>12} s  {:<10} {}",
            self.fmt_t(e.start),
            short_source(&d.decoders[e.dec as usize]),
            e.text
        )
    }

    fn event_json(&self, d: &Decoded, e: &Ev) -> String {
        let mut s = format!(
            r#"{{"t":{},"dur":{},"start":{},"end":{},"decoder":{},"kind":"{}","text":{}"#,
            self.fmt_t(e.start),
            fmt_g(self.secs(e.end.saturating_sub(e.start))),
            e.start,
            e.end,
            jstr(&d.decoders[e.dec as usize]),
            e.kind.id(),
            jstr(&e.text)
        );
        if let Some(b) = &e.data {
            let _ = write!(s, r#","data":"{}""#, hex(b));
        }
        s.push('}');
        s
    }
}

/// A float without trailing noise.
fn fmt_g(v: f64) -> String {
    let s = format!("{v:.9}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() { "0".into() } else { s.to_string() }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(s.get(i * 2..i * 2 + 2)?, 16).ok())
        .collect()
}

/// "ISO7816 clk=ch0 rst=ch1 (UART ch2)" -> "ISO7816", "UART ch2 auto" -> "UART ch2".
fn short_source(s: &str) -> String {
    let w: Vec<&str> = s.split_whitespace().collect();
    match w.as_slice() {
        [a, b, ..] if b.starts_with("ch") => format!("{a} {b}"),
        [a, ..] => a.to_string(),
        [] => String::new(),
    }
}

// ---------------------------------------------------------------- events

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    /// A UART / smart card character (`data` holds it).
    Byte,
    /// An SPI word (`data` holds MOSI then MISO, when assigned).
    Word,
    /// A protocol message (`data` holds its payload, if any).
    Proto,
    /// Rate, format, session and other notes.
    Note,
    Other,
}

impl Kind {
    fn id(self) -> &'static str {
        match self {
            Kind::Byte => "byte",
            Kind::Word => "word",
            Kind::Proto => "protocol",
            Kind::Note => "note",
            Kind::Other => "other",
        }
    }

    fn parse(s: &str) -> Kind {
        match s {
            "byte" => Kind::Byte,
            "word" => Kind::Word,
            "protocol" => Kind::Proto,
            "note" => Kind::Note,
            _ => Kind::Other,
        }
    }
}

#[derive(Clone, Debug)]
struct Ev {
    start: u64,
    end: u64,
    dec: u16,
    ch: u8,
    kind: Kind,
    text: String,
    data: Option<Vec<u8>>,
}

#[derive(Clone, Debug)]
struct Screen {
    at: u64,
    dec: u16,
    title: String,
    width: usize,
    height: usize,
    on: bool,
    pixels: Vec<bool>,
}

struct Session {
    n: u32,
    start: u64,
    end: Option<u64>,
}

/// Everything the decoders reported.
#[derive(Default)]
struct Decoded {
    decoders: Vec<String>,
    events: Vec<Ev>,
    screens: Vec<Screen>,
}

impl Decoded {
    fn add(&mut self, a: &visgrok::decode::Annotation, dec: u16, ch: u8) {
        let (kind, data) = match &a.event {
            Event::Frame(v) => {
                self.screens.push(Screen {
                    at: a.start,
                    dec,
                    title: v.title.to_string(),
                    width: v.width,
                    height: v.height,
                    on: v.on,
                    pixels: v.pixels.clone(),
                });
                return;
            }
            Event::UartByte { value, .. } => (Kind::Byte, Some(vec![*value as u8])),
            Event::SpiWord { mosi, miso, .. } => (Kind::Word, Some([mosi, miso].iter().filter_map(|v| v.map(|v| v as u8)).collect())),
            Event::Protocol { text, data, .. } => (
                if text.starts_with("──") { Kind::Note } else { Kind::Proto },
                data.as_ref().map(|d| d.to_vec()),
            ),
            Event::UartBaud { .. } | Event::UartFormat { .. } => (Kind::Note, None),
            _ => (Kind::Other, None),
        };
        self.events.push(Ev {
            start: a.start,
            end: a.end,
            dec,
            ch,
            kind,
            text: a.event.to_string(),
            data,
        });
    }

    fn matches(&self, e: &Ev, decoder: Option<&str>) -> bool {
        decoder.is_none_or(|d| self.decoders[e.dec as usize].to_lowercase().contains(&d.to_lowercase()))
    }

    /// Sessions from the decoders' reset notes.
    fn sessions(&self) -> Vec<Session> {
        let mut out: Vec<Session> = Vec::new();
        for e in self.events.iter().filter(|e| e.kind == Kind::Note) {
            let Some(rest) = e.text.strip_prefix("── session ") else {
                continue;
            };
            let Some((n, what)) = rest.split_once(':') else { continue };
            let Ok(n) = n.parse::<u32>() else { continue };
            if what.trim_start().starts_with("reset released") {
                out.push(Session {
                    n,
                    start: e.start,
                    end: None,
                });
            } else if let Some(s) = out.iter_mut().rev().find(|s| s.n == n) {
                s.end = Some(e.start);
            }
        }
        out
    }

    fn save(&self, path: &Path, key: &str) -> io::Result<()> {
        let tmp = path.with_extension("events.tmp");
        let mut w = io::BufWriter::new(std::fs::File::create(&tmp)?);
        let decs: Vec<String> = self.decoders.iter().map(|d| jstr(d)).collect();
        writeln!(w, r#"{{"key":{},"decoders":[{}]}}"#, jstr(key), decs.join(","))?;
        for e in &self.events {
            writeln!(
                w,
                r#"["e",{},{},{},{},"{}",{},"{}"]"#,
                e.start,
                e.end,
                e.dec,
                e.ch,
                e.kind.id(),
                jstr(&e.text),
                e.data.as_deref().map(hex).unwrap_or_default()
            )?;
        }
        for s in &self.screens {
            let mut bits = vec![0u8; s.pixels.len().div_ceil(8)];
            for (i, &p) in s.pixels.iter().enumerate() {
                if p {
                    bits[i / 8] |= 1 << (i % 8);
                }
            }
            writeln!(
                w,
                r#"["s",{},{},{},{},{},{},"{}"]"#,
                s.at,
                s.dec,
                jstr(&s.title),
                s.width,
                s.height,
                s.on as u8,
                hex(&bits)
            )?;
        }
        w.flush()?;
        drop(w);
        std::fs::rename(tmp, path)
    }

    fn load(path: &Path, key: &str) -> Option<Decoded> {
        let f = std::fs::File::open(path).ok()?;
        let mut lines = io::BufReader::new(f).lines();
        let head = Json::parse(&lines.next()?.ok()?)?;
        if head.get("key")?.str()? != key {
            return None;
        }
        let Some(Json::Arr(decs)) = head.get("decoders") else { return None };
        let mut d = Decoded {
            decoders: decs.iter().filter_map(|x| x.str().map(str::to_string)).collect(),
            ..Default::default()
        };
        for line in lines {
            let Json::Arr(v) = Json::parse(&line.ok()?)? else { return None };
            let num = |i: usize| v.get(i).and_then(Json::num).unwrap_or(0.0) as u64;
            let s = |i: usize| v.get(i).and_then(Json::str).unwrap_or("");
            match s(0) {
                "e" => d.events.push(Ev {
                    start: num(1),
                    end: num(2),
                    dec: num(3) as u16,
                    ch: num(4) as u8,
                    kind: Kind::parse(s(5)),
                    text: s(6).to_string(),
                    data: (!s(7).is_empty()).then(|| unhex(s(7))).flatten(),
                }),
                "s" => {
                    let (w, h) = (num(4) as usize, num(5) as usize);
                    let bits = unhex(s(7))?;
                    d.screens.push(Screen {
                        at: num(1),
                        dec: num(2) as u16,
                        title: s(3).to_string(),
                        width: w,
                        height: h,
                        on: num(6) != 0,
                        pixels: (0..w * h).map(|i| bits.get(i / 8).is_some_and(|b| b >> (i % 8) & 1 != 0)).collect(),
                    })
                }
                _ => return None,
            }
        }
        Some(d)
    }
}

// ---------------------------------------------------------------- commands

fn summary(ctx: &Ctx, decode: bool, json: bool) -> Result<(), String> {
    let sr = ctx.sr();
    // Channel activity from the overview tiles (.vgk), without reading
    // every sample.
    let mut activity: Vec<Option<(u64, u64, f64, usize)>> = vec![None; ctx.info.channels];
    let mut levels_at_start: Option<Sample> = None;
    if let Some(st) = &ctx.store {
        if st.loaded() < st.total() {
            eprintln!(
                "visgrok: this file has no activity overview; reading all of it (re-save it with `visgrok convert IN OUT.vgk` to make this instant)"
            );
        }
        while st.loaded() < st.total() {
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        let tiles = st.tiles();
        levels_at_start = tiles.first().map(|t| t.first);
        for (ch, a) in activity.iter_mut().enumerate() {
            let active: Vec<usize> = tiles
                .iter()
                .enumerate()
                .filter(|(_, t)| t.changed >> ch & 1 != 0)
                .map(|(i, _)| i)
                .collect();
            if let (Some(&f), Some(&l)) = (active.first(), active.last()) {
                // Bursts: runs of consecutive tiles with changes.
                let bursts = 1 + active.windows(2).filter(|w| w[1] != w[0] + 1).count();
                *a = Some((
                    f as u64 * TILE,
                    (l as u64 + 1) * TILE,
                    active.len() as f64 / tiles.len().max(1) as f64,
                    bursts,
                ));
            }
        }
    }
    let d = if decode { Some(ctx.decoded()?) } else { None };
    let mut sc = Sidecar::default();
    sc.names = ctx.names.clone();
    sc.roles = ctx.roles.iter().map(|r| (*r != Role::Unknown).then(|| r.clone())).collect();
    sc.options = ctx.opts.clone();
    let buses = sc.buses();
    let duration = ctx.total.map(|t| t as f64 / sr);
    if json {
        let mut s = format!(
            r#"{{"file":{},"device":{},"channels":{},"samplerate":{},"duration":{},"buses":[{}],"lines":["#,
            jstr(&ctx.file.display().to_string()),
            jstr(&ctx.info.device),
            ctx.info.channels,
            ctx.info.samplerate,
            duration.map_or("null".into(), fmt_g),
            buses.iter().map(|b| jstr(b)).collect::<Vec<_>>().join(",")
        );
        for ch in 0..ctx.info.channels {
            if ch > 0 {
                s.push(',');
            }
            let _ = write!(
                s,
                r#"{{"ch":{ch},"name":{},"role":{},"level_at_start":{}"#,
                jstr(&ctx.names[ch]),
                jstr(&ctx.roles[ch].id()),
                levels_at_start.map_or("null".into(), |l| (l >> ch & 1).to_string())
            );
            if let Some((f, l, frac, bursts)) = activity[ch] {
                let _ = write!(
                    s,
                    r#","active_from":{},"active_to":{},"active_fraction":{},"bursts":{bursts}"#,
                    fmt_g(f as f64 / sr),
                    fmt_g(l as f64 / sr),
                    fmt_g(frac)
                );
            } else if ctx.store.is_some() {
                s.push_str(r#","active_from":null"#);
            }
            s.push('}');
        }
        s.push(']');
        if let Some(d) = &d {
            s.push_str(r#","decoders":["#);
            for (i, name) in d.decoders.iter().enumerate() {
                let n = d.events.iter().filter(|e| e.dec as usize == i && e.kind != Kind::Note).count();
                let screens = d.screens.iter().filter(|e| e.dec as usize == i).count();
                if i > 0 {
                    s.push(',');
                }
                let _ = write!(s, r#"{{"name":{},"events":{n},"screens":{screens}}}"#, jstr(name));
            }
            s.push_str(r#"],"sessions":["#);
            for (i, x) in d.sessions().iter().enumerate() {
                if i > 0 {
                    s.push(',');
                }
                let _ = write!(
                    s,
                    r#"{{"n":{},"start":{},"end":{}}}"#,
                    x.n,
                    fmt_g(x.start as f64 / sr),
                    x.end.map_or("null".into(), |e| fmt_g(e as f64 / sr))
                );
            }
            s.push(']');
        }
        s.push('}');
        println!("{s}");
        return Ok(());
    }
    println!("file:      {}", ctx.file.display());
    println!("device:    {}", ctx.info.device);
    println!("channels:  {} @ {}", ctx.info.channels, fmt_hz(sr));
    if let Some(d) = duration {
        println!("duration:  {d:.6} s");
    }
    for b in &buses {
        println!("bus:       {b}");
    }
    println!();
    println!("ch  name         role          start  activity");
    for ch in 0..ctx.info.channels {
        let lvl = levels_at_start.map_or("?".into(), |l| (if l >> ch & 1 != 0 { "high" } else { "low" }).to_string());
        let act = match activity[ch] {
            Some((f, l, frac, bursts)) => format!(
                "{:.3} s .. {:.3} s, {bursts} burst{} (changing in {:.3}% of the capture)",
                f as f64 / sr,
                l as f64 / sr,
                if bursts == 1 { "" } else { "s" },
                frac * 100.0
            ),
            None if ctx.store.is_some() => "no edges".into(),
            None => "(use `visgrok edges` / `clock` for details)".into(),
        };
        println!("{ch:>2}  {:<12} {:<13} {lvl:<6} {act}", ctx.names[ch], ctx.roles[ch].id());
    }
    if let Some(d) = &d {
        println!();
        for (i, name) in d.decoders.iter().enumerate() {
            let n = d.events.iter().filter(|e| e.dec as usize == i && e.kind != Kind::Note).count();
            let screens = d.screens.iter().filter(|e| e.dec as usize == i).count();
            let extra = if screens > 0 {
                format!(", {screens} screen updates")
            } else {
                String::new()
            };
            println!("decoder:   {name}: {n} events{extra}");
        }
        let ss = d.sessions();
        if !ss.is_empty() {
            println!("sessions:  {} (see `visgrok sessions`)", ss.len());
        }
    }
    Ok(())
}

fn sessions_cmd(ctx: &Ctx, d: &Decoded, json: bool) -> Result<(), String> {
    let ss = d.sessions();
    if ss.is_empty() {
        return Err("no sessions: assign a reset line role (e.g. -r RESET=iso-rst with -r IO=iso-io)".into());
    }
    for s in &ss {
        let end = s.end.unwrap_or(u64::MAX);
        let evs: Vec<&Ev> = d
            .events
            .iter()
            .filter(|e| e.start >= s.start && e.start < end && e.kind != Kind::Note)
            .collect();
        let bytes = evs.iter().filter(|e| e.kind == Kind::Byte).count();
        let msgs: Vec<&&Ev> = evs.iter().filter(|e| e.kind == Kind::Proto).collect();
        if json {
            let m: Vec<String> = msgs.iter().map(|e| ctx.event_json(d, e)).collect();
            println!(
                r#"{{"session":{},"start":{},"end":{},"duration":{},"bytes":{bytes},"messages":[{}]}}"#,
                s.n,
                ctx.fmt_t(s.start),
                s.end.map_or("null".into(), |e| ctx.fmt_t(e)),
                s.end.map_or("null".into(), |e| fmt_g(ctx.secs(e - s.start))),
                m.join(",")
            );
        } else {
            let dur = s.end.map_or("(open)".into(), |e| format!("{:.6} s", ctx.secs(e - s.start)));
            println!(
                "session {}: {} s .. {} ({dur}), {bytes} characters",
                s.n,
                ctx.fmt_t(s.start),
                s.end.map_or("end".into(), |e| format!("{} s", ctx.fmt_t(e)))
            );
            for e in msgs {
                println!("    +{:.6} s  {}", ctx.secs(e.start - s.start), e.text);
            }
        }
    }
    Ok(())
}

fn event_cmd(ctx: &Ctx, d: &Decoded, at: u64, decoder: Option<&str>, json: bool) -> Result<(), String> {
    // The most specific event containing `at`, else the next one to start.
    let cand = d.events.iter().filter(|e| e.kind != Kind::Note && d.matches(e, decoder));
    let inside = cand
        .clone()
        .filter(|e| e.start <= at && at < e.end.max(e.start + 1))
        .min_by_key(|e| e.end - e.start);
    let e = inside
        .or_else(|| cand.filter(|e| e.start >= at).min_by_key(|e| e.start))
        .ok_or("no event at or after that time")?;
    let source = &d.decoders[e.dec as usize];
    let mut req = Request::new(source, e.start, e.end, &e.text);
    req.data = e.data.as_deref();
    // UART timing as the decoder last reported it.
    let last_note = |prefix: &str| {
        d.events
            .iter()
            .rfind(|x| x.dec == e.dec && x.start <= e.start && x.text.starts_with(prefix))
            .and_then(|x| x.text.trim_start_matches(prefix).split(' ').next().map(str::to_string))
    };
    let baud = last_note("── baud rate ").and_then(|b| b.parse::<f64>().ok());
    let format = last_note("── frame format ");
    req.bit_time = baud.filter(|&b| b > 0.0).map(|b| ctx.sr() / b);
    req.format = format.as_deref();
    let lines = inspect::lines(source);
    let (fields, sig) = if lines.is_empty() {
        (Vec::new(), None)
    } else {
        let (ws, we) = inspect::window(&req);
        let sig = ctx.signal(ws, we, inspect::mask(source))?;
        (inspect::inspect(&req, &sig), Some(sig))
    };
    let name = |ch: u8| ctx.names.get(ch as usize).cloned().unwrap_or(format!("D{ch}"));
    if json {
        let mut s = ctx.event_json(d, e);
        s.pop();
        s.push_str(r#","lines":["#);
        for (i, (ch, role)) in lines.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            let edges: Vec<String> = sig
                .as_ref()
                .map(|g| {
                    g.edges(*ch, g.start, g.end)
                        .iter()
                        .take(10_000)
                        .map(|x| format!("[{},{}]", fmt_g(ctx.secs(x.0)), x.1 as u8))
                        .collect()
                })
                .unwrap_or_default();
            let init = sig.as_ref().map_or(0, |g| g.level(*ch, g.start) as u8);
            let _ = write!(
                s,
                r#"{{"ch":{ch},"name":{},"role":"{role}","level_at_start":{init},"edges":[{}]}}"#,
                jstr(&name(*ch)),
                edges.join(",")
            );
        }
        s.push_str(r#"],"fields":["#);
        for (i, f) in fields.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            let place = match &f.place {
                Place::Line(ch) => format!(r#""line":{}"#, jstr(&name(*ch))),
                Place::Row(r) => format!(r#""row":"{r}""#),
                _ => r#""row":"?""#.into(),
            };
            let _ = write!(
                s,
                r#"{{"start":{},"end":{},{place},"label":{},"detail":{},"bad":{}}}"#,
                fmt_g(ctx.secs(f.start)),
                fmt_g(ctx.secs(f.end)),
                jstr(&f.label),
                jstr(&f.detail),
                f.bad
            );
        }
        s.push_str("]}");
        println!("{s}");
        return Ok(());
    }
    println!("{}", e.text);
    println!(
        "  {source} · at {} s (sample {}) · lasts {}",
        ctx.fmt_t(e.start),
        e.start,
        fmt_dur(ctx.secs(e.end.saturating_sub(e.start)))
    );
    if let Some(b) = &e.data
        && b.len() > 1
    {
        println!("\npayload ({} bytes):", b.len());
        for (i, row) in b.chunks(16).enumerate() {
            let h: Vec<String> = row.iter().map(|x| format!("{x:02x}")).collect();
            let a: String = row
                .iter()
                .map(|&c| if (0x20..0x7f).contains(&c) { c as char } else { '.' })
                .collect();
            println!("  {:04x}  {:<48} {a}", i * 16, h.join(" "));
        }
    }
    let rows: Vec<_> = fields.iter().filter(|f| matches!(f.place, Place::Row(_))).collect();
    if !rows.is_empty() {
        println!("\nfields (time from the event start):");
        for f in rows.iter().take(600) {
            let Place::Row(r) = &f.place else { continue };
            let off = f.start as f64 - e.start as f64;
            println!(
                "  {:>12}  {:<8} {:<16} {}{}",
                fmt_dur_signed(off / ctx.sr()),
                r,
                f.label,
                f.detail,
                if f.bad { "  [BAD]" } else { "" }
            );
        }
        if rows.len() > 600 {
            println!("  ... {} more (use --json)", rows.len() - 600);
        }
    }
    for (ch, role) in &lines {
        let bits: String = fields
            .iter()
            .filter(|f| f.place == Place::Line(*ch))
            .map(|f| f.label.as_str())
            .collect::<Vec<_>>()
            .concat();
        if !bits.is_empty() {
            let shown = if bits.len() > 256 {
                format!("{}… ({} bits)", &bits[..256], bits.len())
            } else {
                bits
            };
            println!("\n{role} ({}) bits: {shown}", name(*ch));
        }
    }
    Ok(())
}

fn fmt_dur(s: f64) -> String {
    let a = s.abs();
    if a >= 1.0 {
        format!("{s:.6} s")
    } else if a >= 1e-3 {
        format!("{:.3} ms", s * 1e3)
    } else if a >= 1e-6 {
        format!("{:.3} µs", s * 1e6)
    } else {
        format!("{:.1} ns", s * 1e9)
    }
}

fn fmt_dur_signed(s: f64) -> String {
    if s >= 0.0 { format!("+{}", fmt_dur(s)) } else { fmt_dur(s) }
}

fn data_cmd(ctx: &Ctx, d: &Decoded, (from, to): (u64, u64), decoder: Option<&str>, json: bool) -> Result<(), String> {
    // Bursts: consecutive bytes of one decoder (and one D/C level for SPI)
    // without a pause longer than 12 character times.
    struct Burst {
        dec: u16,
        start: u64,
        end: u64,
        dc: Option<bool>,
        mosi: Vec<u8>,
        miso: Vec<u8>,
    }
    let mut open: Vec<Burst> = Vec::new();
    let mut done: Vec<Burst> = Vec::new();
    // Decoders that report characters or words give those; display
    // decoders (SSD1306) only report commands and RAM writes, whose bytes
    // are in their payload or at the start of their text.
    let raw: Vec<bool> = (0..d.decoders.len())
        .map(|i| {
            d.events
                .iter()
                .any(|e| e.dec as usize == i && matches!(e.kind, Kind::Byte | Kind::Word))
        })
        .collect();
    for e in d.events.iter().filter(|e| e.start >= from && e.start < to && d.matches(e, decoder)) {
        let display = d.decoders[e.dec as usize].starts_with("SSD1306");
        let proto_bytes = || -> Option<Vec<u8>> {
            if e.data.is_some() {
                return e.data.clone();
            }
            let (head, _) = e.text.split_once(':')?;
            head.split_whitespace().map(|h| u8::from_str_radix(h, 16).ok()).collect()
        };
        let (bytes, dc) = match e.kind {
            Kind::Byte | Kind::Word => (
                e.data.clone(),
                (e.kind == Kind::Word && e.text.ends_with(']')).then(|| e.text.ends_with("[data]")),
            ),
            Kind::Proto if !raw[e.dec as usize] => (proto_bytes(), display.then_some(e.data.is_some())),
            _ => continue,
        };
        let Some(b) = bytes else { continue };
        let b = &b;
        let len = (e.end - e.start).max(1);
        let has_miso = e.kind == Kind::Word && e.text.contains("miso ") && !e.text.contains("miso --");
        let has_mosi = e.kind != Kind::Word || !e.text.starts_with("mosi --");
        let whole = e.kind == Kind::Proto;
        let k = open.iter().position(|x| x.dec == e.dec);
        let fresh = match k {
            Some(i) => {
                let x = &open[i];
                e.start.saturating_sub(x.end) > 12 * len || x.dc != dc
            }
            None => true,
        };
        if fresh {
            if let Some(i) = k {
                done.push(open.remove(i));
            }
            open.push(Burst {
                dec: e.dec,
                start: e.start,
                end: e.end,
                dc,
                mosi: Vec::new(),
                miso: Vec::new(),
            });
        }
        let x = open.iter_mut().find(|x| x.dec == e.dec).unwrap();
        x.end = e.end;
        if whole {
            x.mosi.extend_from_slice(b);
            continue;
        }
        let mut it = b.iter();
        if has_mosi && let Some(&v) = it.next() {
            x.mosi.push(v);
        }
        if has_miso && let Some(&v) = it.next() {
            x.miso.push(v);
        }
    }
    done.extend(open);
    done.sort_by_key(|b| b.start);
    if done.is_empty() {
        return Err("no characters or SPI words in that range (see `visgrok summary` for the decoders)".into());
    }
    for b in &done {
        let src = &d.decoders[b.dec as usize];
        if json {
            let mut s = format!(
                r#"{{"t":{},"end":{},"decoder":{},"bytes":{}"#,
                ctx.fmt_t(b.start),
                ctx.fmt_t(b.end),
                jstr(src),
                b.mosi.len().max(b.miso.len())
            );
            if let Some(dc) = b.dc {
                let _ = write!(s, r#","dc":"{}""#, if dc { "data" } else { "command" });
            }
            let _ = write!(s, r#","data":"{}""#, hex(&b.mosi));
            if !b.miso.is_empty() {
                let _ = write!(s, r#","miso":"{}""#, hex(&b.miso));
            }
            s.push('}');
            println!("{s}");
            continue;
        }
        let dc = match b.dc {
            Some(true) => " data",
            Some(false) => " cmd ",
            None => "",
        };
        println!(
            "{:>12} s  {:<10}{dc} {:>4} B  {}",
            ctx.fmt_t(b.start),
            short_source(src),
            b.mosi.len(),
            preview(&b.mosi, 64)
        );
        if !b.miso.is_empty() {
            println!("{:>12}    {:<10}{dc} {:>4} B  MISO {}", "", "", b.miso.len(), preview(&b.miso, 64));
        }
    }
    Ok(())
}

/// Up to `n` bytes in hex, with their text when printable.
fn preview(b: &[u8], n: usize) -> String {
    let h: Vec<String> = b.iter().take(n).map(|x| format!("{x:02x}")).collect();
    let a: String = b
        .iter()
        .take(n)
        .map(|&c| if (0x20..0x7f).contains(&c) { c as char } else { '.' })
        .collect();
    format!("{}{}  |{a}|", h.join(" "), if b.len() > n { " …" } else { "" })
}

fn screens_cmd(ctx: &Ctx, d: &Decoded, (from, to): (u64, u64), out: &Path, scale: u32, all: bool, json: bool) -> Result<(), String> {
    let shots: Vec<&Screen> = d.screens.iter().filter(|s| s.at >= from && s.at < to).collect();
    if shots.is_empty() {
        let hint = if d.screens.is_empty() {
            "no display decoder output (assign SPI roles with --spi-proto ssd1306)"
        } else {
            "no screen updates in that range"
        };
        return Err(hint.into());
    }
    std::fs::create_dir_all(out).map_err(|e| format!("{}: {e}", out.display()))?;
    let mut prev: Option<&Vec<bool>> = None;
    let mut n = 0;
    let mut skipped = 0;
    for s in shots {
        if !all && prev == Some(&s.pixels) {
            skipped += 1;
            continue;
        }
        prev = Some(&s.pixels);
        n += 1;
        let path = out.join(format!("screen-{n:04}-{:.6}s.png", ctx.secs(s.at)));
        let png = render_png(s, scale)?;
        std::fs::write(&path, png).map_err(|e| format!("{}: {e}", path.display()))?;
        let lit = s.pixels.iter().filter(|&&p| p).count();
        if json {
            println!(
                r#"{{"t":{},"file":{},"decoder":{},"width":{},"height":{},"on":{},"lit_pixels":{lit}}}"#,
                ctx.fmt_t(s.at),
                jstr(&path.display().to_string()),
                jstr(&d.decoders[s.dec as usize]),
                s.width,
                s.height,
                s.on
            );
        } else {
            println!(
                "{:>12} s  {}  {}x{} {}{}, {lit} pixels lit",
                ctx.fmt_t(s.at),
                path.display(),
                s.width,
                s.height,
                s.title,
                if s.on { "" } else { " (panel off)" }
            );
        }
    }
    eprintln!(
        "{n} screens written to {}{}",
        out.display(),
        if skipped > 0 {
            format!(" ({skipped} identical updates skipped)")
        } else {
            String::new()
        }
    );
    Ok(())
}

/// The screen as a PNG: lit pixels light on dark, `scale` x `scale` each.
fn render_png(s: &Screen, scale: u32) -> Result<Vec<u8>, String> {
    let (w, h) = (s.width as u32 * scale, s.height as u32 * scale);
    let (on, off) = if s.on {
        ([0xe8, 0xf4, 0xff], [0x08, 0x0a, 0x10])
    } else {
        ([0x60, 0x66, 0x70], [0x08, 0x0a, 0x10])
    };
    let mut rgb = Vec::with_capacity((w * h * 3) as usize);
    for y in 0..h {
        for x in 0..w {
            let p = s.pixels[(y / scale) as usize * s.width + (x / scale) as usize];
            rgb.extend_from_slice(if p { &on } else { &off });
        }
    }
    oxideav_png::encode_rgb8(w, h, &rgb, &oxideav_png::EncodeOptions::default()).map_err(|e| format!("PNG: {e}"))
}

fn edges_cmd(ctx: &Ctx, (from, to): (u64, u64), mask: Sample, limit: usize, json: bool) -> Result<(), String> {
    let mut n = 0;
    let mut last: Vec<Option<u64>> = vec![None; ctx.info.channels];
    let mut out = io::stdout().lock();
    let mut more = false;
    ctx.transitions(from, to, mask, |t| {
        for ch in 0..ctx.info.channels {
            if t.changed() >> ch & 1 == 0 {
                continue;
            }
            if n >= limit {
                more = true;
                return false;
            }
            n += 1;
            let rise = t.now >> ch & 1 != 0;
            let since = last[ch].map(|l| ctx.secs(t.at - l));
            last[ch] = Some(t.at);
            let line = if json {
                format!(
                    r#"{{"t":{},"sample":{},"ch":{ch},"name":{},"edge":"{}","since_prev":{}}}"#,
                    fmt_g(ctx.secs(t.at)),
                    t.at,
                    jstr(&ctx.names[ch]),
                    if rise { "rise" } else { "fall" },
                    since.map_or("null".into(), fmt_g)
                )
            } else {
                format!(
                    "{:>15} s  {:<10} {}  {}",
                    format!("{:.9}", ctx.secs(t.at)),
                    ctx.names[ch],
                    if rise { "rise" } else { "fall" },
                    since.map_or(String::new(), |s| format!("(+{})", fmt_dur(s)))
                )
            };
            if writeln!(out, "{line}").is_err() {
                return false;
            }
        }
        true
    })?;
    if more {
        eprintln!("visgrok: stopped at --limit {limit} (narrow with --from/--to/--ch or raise -n)");
    }
    Ok(())
}

fn levels_cmd(ctx: &Ctx, at: u64, json: bool) -> Result<(), String> {
    let mut state = None;
    ctx.blocks(at, at + 1, |b| {
        state = Some(b.sample(0));
        false
    })?;
    let s = state.ok_or("that time is past the end of the capture")?;
    if json {
        let items: Vec<String> = (0..ctx.info.channels)
            .map(|ch| format!(r#"{{"ch":{ch},"name":{},"level":{}}}"#, jstr(&ctx.names[ch]), s >> ch & 1))
            .collect();
        println!(r#"{{"t":{},"sample":{at},"levels":[{}]}}"#, fmt_g(ctx.secs(at)), items.join(","));
    } else {
        println!("at {} s (sample {at}):", ctx.fmt_t(at));
        for ch in 0..ctx.info.channels {
            println!("  {ch:>2} {:<12} {}", ctx.names[ch], if s >> ch & 1 != 0 { "high" } else { "low" });
        }
    }
    Ok(())
}

/// One stretch of a running clock.
struct Run {
    start: u64,
    end: u64,
    cycles: u64,
    min_period: u64,
    max_period: u64,
    /// Sum and sum of squares of rise-to-rise periods (samples).
    sum: f64,
    sum2: f64,
    high: u64,
    /// Frequency over successive windows (for drift).
    win_start: u64,
    win_cycles: u64,
    win_min: f64,
    win_max: f64,
}

fn clock_cmd(ctx: &Ctx, (from, to): (u64, u64), ch: u8, json: bool) -> Result<(), String> {
    let sr = ctx.sr();
    let window = (sr * 0.01) as u64; // 10 ms windows for drift
    let mut runs: Vec<Run> = Vec::new();
    let mut last_rise: Option<u64> = None;
    let mut last_fall: Option<u64> = None;
    let mut last_edge: Option<u64> = None;
    let mut cur: Option<Run> = None;
    let close = |r: Run, runs: &mut Vec<Run>| {
        if r.cycles >= 2 {
            runs.push(r);
        }
    };
    let t0 = Instant::now();
    let mut shown = Instant::now();
    ctx.transitions(from, to, 1 << ch, |t| {
        if shown.elapsed().as_secs_f64() > 1.0 {
            shown = Instant::now();
            eprint!("\rreading {:.1} s...", ctx.secs(t.at));
        }
        let rise = t.now >> ch & 1 != 0;
        // A pause much longer than the period ends the run.
        if let (Some(r), Some(le)) = (&cur, last_edge) {
            let typical = (r.sum / r.cycles.max(1) as f64).max(2.0);
            if r.cycles >= 2 && (t.at - le) as f64 > 50.0 * typical {
                close(cur.take().unwrap(), &mut runs);
                last_rise = None;
                last_fall = None;
            }
        }
        last_edge = Some(t.at);
        if rise {
            match (last_rise, &mut cur) {
                (Some(p), Some(r)) => {
                    let period = t.at - p;
                    r.cycles += 1;
                    r.min_period = r.min_period.min(period);
                    r.max_period = r.max_period.max(period);
                    r.sum += period as f64;
                    r.sum2 += (period * period) as f64;
                    r.end = t.at;
                    r.win_cycles += 1;
                    if t.at - r.win_start >= window {
                        let f = r.win_cycles as f64 * sr / (t.at - r.win_start) as f64;
                        r.win_min = r.win_min.min(f);
                        r.win_max = r.win_max.max(f);
                        r.win_start = t.at;
                        r.win_cycles = 0;
                    }
                }
                (Some(p), None) => {
                    let period = t.at - p;
                    cur = Some(Run {
                        start: p,
                        end: t.at,
                        cycles: 1,
                        min_period: period,
                        max_period: period,
                        sum: period as f64,
                        sum2: (period * period) as f64,
                        high: 0,
                        win_start: t.at,
                        win_cycles: 0,
                        win_min: f64::MAX,
                        win_max: 0.0,
                    })
                }
                _ => {}
            }
            last_rise = Some(t.at);
        } else {
            if let (Some(r), Some(lr)) = (&mut cur, last_rise) {
                r.high += t.at - lr;
            }
            last_fall = Some(t.at);
        }
        let _ = last_fall;
        true
    })?;
    if let Some(r) = cur.take() {
        close(r, &mut runs);
    }
    if t0.elapsed().as_secs_f64() > 1.0 {
        eprintln!("\r{:<40}", "");
    }
    if runs.is_empty() {
        return Err(format!("{} does not toggle like a clock in that range", ctx.names[ch as usize]));
    }
    let ns = |samples: f64| samples / sr * 1e9;
    for (i, r) in runs.iter().enumerate() {
        let mean = r.sum / r.cycles as f64;
        let var = (r.sum2 / r.cycles as f64 - mean * mean).max(0.0);
        let freq = sr / mean;
        let duty = r.high as f64 / (r.end - r.start).max(1) as f64;
        let drift = (r.win_max > 0.0).then(|| ((r.win_min / freq - 1.0) * 1e6, (r.win_max / freq - 1.0) * 1e6));
        if json {
            println!(
                r#"{{"channel":{},"run":{},"start":{},"end":{},"cycles":{},"frequency_hz":{},"period_ns":{},"period_min_ns":{},"period_max_ns":{},"period_stddev_ns":{},"duty":{},"drift_ppm":{},"sample_period_ns":{}}}"#,
                jstr(&ctx.names[ch as usize]),
                i + 1,
                ctx.fmt_t(r.start),
                ctx.fmt_t(r.end),
                r.cycles,
                fmt_g(freq),
                fmt_g(ns(mean)),
                fmt_g(ns(r.min_period as f64)),
                fmt_g(ns(r.max_period as f64)),
                fmt_g(ns(var.sqrt())),
                fmt_g(duty),
                drift.map_or("null".into(), |(a, b)| format!("[{},{}]", fmt_g(a), fmt_g(b))),
                fmt_g(1e9 / sr)
            );
        } else {
            println!(
                "run {}: {} s .. {} s ({}), {} cycles",
                i + 1,
                ctx.fmt_t(r.start),
                ctx.fmt_t(r.end),
                fmt_dur(ctx.secs(r.end - r.start)),
                r.cycles
            );
            println!("  frequency  {} (period {:.3} ns)", fmt_hz(freq), ns(mean));
            println!(
                "  period     {:.1} .. {:.1} ns, std dev {:.2} ns (sampling resolution {:.2} ns)",
                ns(r.min_period as f64),
                ns(r.max_period as f64),
                ns(var.sqrt()),
                1e9 / sr
            );
            println!("  duty       {:.1}% high", duty * 100.0);
            if let Some((a, b)) = drift {
                println!("  drift      {a:+.0} .. {b:+.0} ppm between 10 ms windows");
            }
        }
    }
    if !json && runs.len() > 1 {
        let gaps: Vec<String> = runs.windows(2).map(|w| fmt_dur(ctx.secs(w[1].start - w[0].end))).take(8).collect();
        println!("stopped between runs for {}", gaps.join(", "));
    }
    Ok(())
}
