//! `visgrok`: capture from an SLogic logic analyzer (or a synthetic source),
//! write everything to a sigrok `.sr` file and show live analysis in a TUI.

mod pipeline;
mod ui;
mod web;

use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use visgrok::Source;
use visgrok::analyzer::{DecoderOptions, SpiProtocol, UartProtocol, parse_uart_format};
use visgrok::formats::{Format, ReadOptions, WriteOptions};
use visgrok::roles::{Role, fmt_hz};
use visgrok::sidecar::Sidecar;
use visgrok::slogic::{Config, Pattern, SLogic};
use visgrok::srzip::SrCompression;
use visgrok::synth::Synth;

use crate::pipeline::{Pipeline, Setup};

/// Command-line options.
#[derive(Parser, Debug)]
#[command(version, about = "Live capture and analysis for Sipeed SLogic logic analyzers")]
pub struct Args {
    #[command(subcommand)]
    command: Option<Command>,
    /// Output file; the format follows the extension: .vgk (default,
    /// compressed), .sr (sigrok), .vcd, .bin. Nothing is written when omitted.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Replay a capture (.vgk, .sr, .vcd; .bin with -s and -c) through the
    /// analyzers instead of capturing.
    #[arg(short, long)]
    input: Option<PathBuf>,
    /// Sample rate, e.g. 20M, 100M. Defaults to the device's maximum for the
    /// channel count (SLogic16 U3: 16ch 200M, 8ch 400M, 4ch 800M; SLogic32 U3:
    /// 32ch 200M, 16ch 400M, 8ch 800M, 4ch 1400M); 20M with --demo.
    #[arg(short, long, value_parser = parse_rate)]
    samplerate: Option<u64>,
    /// Number of channels to capture: 4, 8 or 16 (SLogic16 U3), up to 32
    /// (SLogic32 U3).
    #[arg(short, long, default_value_t = 16)]
    channels: usize,
    /// Input threshold in volts (default: device default, about 2.0 V).
    #[arg(short, long)]
    threshold: Option<f64>,
    /// Select the device with this serial number.
    #[arg(long)]
    serial: Option<String>,
    /// Summarize a capture file (per-channel activity over time) and exit.
    #[arg(long, value_name = "FILE")]
    info: Option<PathBuf>,
    /// List connected devices and exit.
    #[arg(long)]
    list: bool,
    /// Capture the device's built-in emulation pattern instead of the inputs.
    #[arg(long)]
    emulation: bool,
    /// Stop after this many seconds.
    #[arg(short, long)]
    duration: Option<f64>,
    /// Use a synthetic signal generator instead of hardware: `bus` (clock,
    /// UART, I2C, SPI) or `device` (UART negotiating 21.5k→2M baud on D0,
    /// SSD1306 OLED on D1 SCLK, D2 MOSI, D3 D/C, D4 CS).
    #[arg(long, num_args = 0..=1, default_missing_value = "bus", value_name = "SCENARIO")]
    demo: Option<String>,
    /// No TUI: print a status line every second.
    #[arg(long)]
    headless: bool,
    /// Apply auto-detected channel roles (and start decoders) automatically.
    #[arg(long)]
    auto: bool,
    /// Assign a channel role, e.g. `--role 0=uart`, `--role 1=uart:115200`,
    /// `--role 2=spi-clk --role 3=spi-mosi --role 4=spi-dc --role 5=spi-cs`,
    /// `--role 6=i2c-scl:7 --role 7=i2c-sda:6`, `--role 8=sd-clk --role 9=sd-cmd
    /// --role 10=sd-dat0 ... --role 13=sd-dat3`. Repeatable.
    #[arg(short, long = "role", value_name = "CH=ROLE")]
    roles: Vec<String>,
    /// Name a channel, e.g. `--name 0=CLK`. Used in the UI and recorded files.
    #[arg(short, long = "name", value_name = "CH=NAME")]
    names: Vec<String>,
    /// SPI mode 0..3 (default: clock polarity from its idle level, CPHA 0).
    #[arg(long)]
    spi_mode: Option<u8>,
    /// SPI chip select is active high.
    #[arg(long)]
    spi_cs_high: bool,
    /// Protocol on top of SPI: raw, ssd1306, ssd1306:128x32.
    #[arg(long, default_value = "raw", value_parser = SpiProtocol::parse)]
    spi_proto: SpiProtocol,
    /// UART frame format: auto (detect from the traffic), or 8N1, 8E2, 8O1,
    /// 7E1, ... to force one.
    #[arg(long, default_value = "auto")]
    uart_format: String,
    /// Protocol on top of UART: raw, iso7816 (smart card: ATR, PPS, T=1).
    #[arg(long, default_value = "raw", value_parser = UartProtocol::parse)]
    uart_proto: UartProtocol,
    /// Keep UART rates given with --role fixed instead of following changes.
    #[arg(long)]
    uart_fixed: bool,
}

/// Subcommands.
#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Web control mode: serve a browser UI (capture control, roles,
    /// browsing live and recorded captures) on a local HTTP port.
    Web {
        /// Address to listen on (use 0.0.0.0:PORT to allow other machines).
        #[arg(short, long, default_value = "127.0.0.1:8090")]
        listen: String,
        /// Directory for new captures, and listed by "Open capture".
        #[arg(short, long, default_value = "captures")]
        dir: PathBuf,
        /// Capture to open right away (.vgk, .sr, .vcd).
        open: Option<PathBuf>,
    },
    /// Convert a capture between formats: .vgk, .sr, .vcd, .bin (by
    /// extension). No analysis; every sample is copied.
    Convert {
        /// Input file.
        input: PathBuf,
        /// Output file.
        output: PathBuf,
        /// Sample rate of a raw .bin input (or override for .vcd).
        #[arg(short, long, value_parser = parse_rate)]
        samplerate: Option<u64>,
        /// Channel count of a raw .bin input.
        #[arg(short, long)]
        channels: Option<usize>,
        /// Store .sr chunks uncompressed instead of deflating them.
        #[arg(long)]
        sr_store: bool,
        /// Input format, when the extension doesn't tell (vgk, sr, vcd, bin).
        #[arg(long)]
        from: Option<String>,
        /// Output format, when the extension doesn't tell.
        #[arg(long)]
        to: Option<String>,
    },
}

/// Parses `20M`, `1.5G`, `400k` or a plain number of Hz.
fn parse_rate(s: &str) -> Result<u64, String> {
    let s = s.trim().trim_end_matches("Hz").trim_end_matches("hz").trim();
    let (num, mul) = match s.chars().last() {
        Some('k' | 'K') => (&s[..s.len() - 1], 1e3),
        Some('m' | 'M') => (&s[..s.len() - 1], 1e6),
        Some('g' | 'G') => (&s[..s.len() - 1], 1e9),
        _ => (s, 1.0),
    };
    let v: f64 = num.trim().parse().map_err(|e| format!("bad rate {s:?}: {e}"))?;
    Ok((v * mul).round() as u64)
}

fn open_source(args: &Args) -> Result<Box<dyn Source>, String> {
    let limit = |rate: u64| args.duration.map(|d| (d * rate as f64) as u64);
    if let Some(p) = &args.input {
        let mut ropts = ReadOptions::default();
        ropts.samplerate = args.samplerate;
        ropts.channels = Some(args.channels);
        let r = visgrok::formats::open(p, &ropts).map_err(|e| format!("{}: {e}", p.display()))?;
        return Ok(r);
    }
    if let Some(scenario) = &args.demo {
        return match scenario.as_str() {
            "bus" => {
                let rate = args.samplerate.unwrap_or(20_000_000);
                Ok(Box::new(Synth::new(rate, limit(rate))))
            }
            "device" => {
                let rate = args.samplerate.unwrap_or(50_000_000);
                Ok(Box::new(Synth::device(rate, limit(rate))))
            }
            other => Err(format!("unknown demo scenario {other:?} (bus, device)")),
        };
    }
    let dev = SLogic::open(args.serial.as_deref()).map_err(|e| e.to_string())?;
    let rate = args.samplerate.unwrap_or_else(|| dev.model().max_samplerate(args.channels));
    let mut cfg = Config::new(args.channels, rate);
    cfg.threshold = args.threshold;
    cfg.limit = limit(rate);
    if args.emulation {
        cfg.pattern = Pattern::Emulation;
    }
    if let Err(e) = dev.validate(&cfg) {
        let m = dev.model();
        let rates: Vec<String> = m
            .samplerates()
            .into_iter()
            .filter(|&r| r <= m.max_samplerate(args.channels))
            .map(|r| fmt_hz(r as f64))
            .collect();
        return Err(format!("{e}\nvalid rates for {} channels: {}", args.channels, rates.join(", ")));
    }
    Ok(Box::new(dev.start(cfg).map_err(|e| e.to_string())?))
}

fn main() {
    let args = Args::parse();
    if let Some(Command::Web { listen, dir, open }) = &args.command {
        let opts = web::WebOptions {
            listen: listen.clone(),
            dir: dir.clone(),
            open: open.clone(),
        };
        if let Err(e) = web::run(opts) {
            eprintln!("visgrok: {e}");
            std::process::exit(1);
        }
        return;
    }
    if let Some(Command::Convert {
        input,
        output,
        samplerate,
        channels,
        sr_store,
        from,
        to,
    }) = &args.command
    {
        let fmt = |s: &Option<String>| -> Option<Format> {
            s.as_deref().map(|n| {
                Format::parse(n).unwrap_or_else(|| {
                    eprintln!("visgrok: unknown format {n:?} (vgk, sr, vcd, bin)");
                    std::process::exit(2)
                })
            })
        };
        let mut ropts = ReadOptions::default();
        ropts.samplerate = *samplerate;
        ropts.channels = *channels;
        ropts.format = fmt(from);
        let mut wopts = WriteOptions::default();
        wopts.sr_compression = if *sr_store { SrCompression::Store } else { SrCompression::Deflate };
        wopts.format = fmt(to);
        let t = std::time::Instant::now();
        let mut last = std::time::Instant::now();
        // The sidecar travels with the capture.
        if let Ok(Some(mut sc)) = Sidecar::load(input) {
            sc.capture = output.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
            if let Err(e) = sc.save(output) {
                eprintln!("visgrok: copying sidecar: {e}");
            }
        }
        let r = visgrok::formats::convert(input, output, &ropts, &wopts, |n| {
            if last.elapsed().as_secs_f64() > 1.0 {
                eprint!("\r{n} samples...");
                last = std::time::Instant::now();
            }
        });
        match r {
            Ok(c) => {
                let insize = std::fs::metadata(input).map_or(0, |m| m.len());
                eprintln!(
                    "\r{} → {}: {} samples ({:.3} s, {} ch @ {}), {} → {} in {:.1}s",
                    input.display(),
                    output.display(),
                    c.samples,
                    c.samples as f64 / c.info.samplerate as f64,
                    c.info.channels,
                    fmt_hz(c.info.samplerate as f64),
                    pipeline::fmt_bytes(insize),
                    pipeline::fmt_bytes(c.bytes),
                    t.elapsed().as_secs_f64()
                );
            }
            Err(e) => {
                eprintln!("visgrok: {e}");
                std::process::exit(1);
            }
        }
        return;
    }
    if let Some(p) = &args.info {
        if let Err(e) = info(p) {
            eprintln!("visgrok: {}: {e}", p.display());
            std::process::exit(1);
        }
        return;
    }
    if args.list {
        match visgrok::slogic::list() {
            Ok(v) if v.is_empty() => println!("no devices found"),
            Ok(v) => v.iter().for_each(|f| println!("{f}")),
            Err(e) => eprintln!("visgrok: {e}"),
        }
        return;
    }
    let source = match open_source(&args) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("visgrok: {e}");
            std::process::exit(1);
        }
    };
    let mut extra = Vec::new();
    if let Some(t) = args.threshold {
        extra.push(("threshold_v".to_string(), t.to_string()));
    }
    if args.emulation {
        extra.push(("pattern".to_string(), "emulation".to_string()));
    }
    let mut roles: Vec<Option<Role>> = Vec::new();
    for spec in &args.roles {
        let parsed = spec
            .split_once('=')
            .ok_or_else(|| format!("--role {spec:?}: expected CH=ROLE"))
            .and_then(|(ch, r)| {
                let ch: usize = ch
                    .trim_start_matches(['D', 'd'])
                    .parse()
                    .map_err(|_| format!("bad channel in {spec:?}"))?;
                Ok((ch, Role::parse(r)?))
            });
        match parsed {
            Ok((ch, role)) => {
                if roles.len() <= ch {
                    roles.resize(ch + 1, None);
                }
                roles[ch] = Some(role);
            }
            Err(e) => {
                eprintln!("visgrok: {e}");
                std::process::exit(2);
            }
        }
    }
    let mut options = DecoderOptions::default();
    options.spi_mode = args.spi_mode;
    options.spi_cs_active_high = args.spi_cs_high;
    options.spi_protocol = args.spi_proto;
    options.uart_auto = !args.uart_fixed;
    options.uart_protocol = args.uart_proto;
    options.uart_format = match parse_uart_format(&args.uart_format) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("visgrok: --uart-format: {e}");
            std::process::exit(2);
        }
    };
    let mut names: Vec<Option<String>> = Vec::new();
    for spec in &args.names {
        let Some((ch, name)) = spec.split_once('=') else {
            eprintln!("visgrok: --name {spec:?}: expected CH=NAME");
            std::process::exit(2);
        };
        let Ok(ch) = ch.trim_start_matches(['D', 'd']).parse::<usize>() else {
            eprintln!("visgrok: --name {spec:?}: bad channel");
            std::process::exit(2);
        };
        if names.len() <= ch {
            names.resize(ch + 1, None);
        }
        names[ch] = Some(name.to_string());
    }
    // Replaying a capture without --role: use its sidecar (roles, names and
    // decoder settings saved with it).
    let (mut roles, mut options, mut names) = (roles, options, names);
    if let Some(input) = &args.input
        && args.roles.is_empty()
    {
        match Sidecar::load(input) {
            Ok(Some(sc)) => {
                eprintln!("visgrok: using {}", visgrok::sidecar::path_for(input).display());
                for b in sc.buses() {
                    eprintln!("  {b}");
                }
                roles = sc.roles.clone();
                options = sc.options.clone();
                if names.is_empty() {
                    names = sc.names.iter().map(|n| (!n.is_empty()).then(|| n.clone())).collect();
                }
            }
            Ok(None) => {}
            Err(e) => eprintln!("visgrok: {e}"),
        }
    }
    let setup = Setup {
        extra,
        roles,
        options,
        names,
        lossless_analysis: args.input.is_some(),
        ..Default::default()
    };
    let pipe = match Pipeline::start(source, args.output.clone(), setup) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("visgrok: {e}");
            std::process::exit(1);
        }
    };
    // Recording: describe the capture in its sidecar (again at the end, with
    // any roles changed in the TUI).
    let save_sidecar = |pipe: &Pipeline| {
        let (Some(out), None) = (&args.output, &args.input) else { return };
        let mut sc = Sidecar::default();
        sc.capture = out.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
        sc.names = pipe.names.clone();
        sc.roles = pipe.roles.lock().unwrap().clone();
        sc.options = pipe.options.lock().unwrap().clone();
        sc.device = Some(pipe.info.device.clone());
        sc.samplerate = Some(pipe.info.samplerate);
        sc.threshold = args.threshold;
        if let Err(e) = sc.save(out) {
            eprintln!("visgrok: saving sidecar: {e}");
        }
    };
    save_sidecar(&pipe);
    let result = if args.headless {
        headless(&pipe, args.auto)
    } else {
        ui::run(&pipe, args.auto)
    };
    pipe.stop();
    let summary = pipe.join();
    save_sidecar(&pipe);
    if let Err(e) = result {
        eprintln!("visgrok: {e}");
    }
    eprintln!("{summary}");
    if pipe.error().is_some() {
        std::process::exit(1);
    }
}

fn headless(pipe: &Pipeline, auto: bool) -> std::io::Result<()> {
    let mut applied = false;
    let mut next_status = 1.0;
    loop {
        let finished = pipe.finished();
        std::thread::sleep(Duration::from_millis(100));
        for line in pipe.drain_log() {
            println!("{line}");
        }
        if finished {
            break;
        }
        if pipe.seconds() < next_status {
            continue;
        }
        next_status += 1.0;
        if auto && !applied && pipe.seconds() >= 2.0 {
            pipe.apply_suggestions();
            applied = true;
            for (i, r) in pipe.effective_roles().iter().enumerate() {
                eprintln!("ch{i}: {r}");
            }
        }
        eprintln!("{}", pipe.status_line());
    }
    Ok(())
}

/// Prints a per-channel summary of a capture, with an activity timeline.
fn info(path: &std::path::Path) -> std::io::Result<()> {
    use visgrok::stats::Stats;
    use visgrok::{EdgeDetector, Transition};

    let mut r = visgrok::formats::open(path, &ReadOptions::default())?;
    let m = r.info();
    let sr = m.samplerate as f64;
    println!("file:       {}", path.display());
    println!("device:     {}", m.device);
    println!("channels:   {} @ {}", m.channels, fmt_hz(sr));
    if let Ok(v) = visgrok::vgk::VgkReader::open(path) {
        for (k, v) in &v.meta().extra {
            println!("{k:<11} {v}");
        }
    }
    match Sidecar::load(path) {
        Ok(Some(sc)) => {
            println!("sidecar:    {}", visgrok::sidecar::path_for(path).display());
            for b in sc.buses() {
                println!("  bus:      {b}");
            }
            for b in &sc.bookmarks {
                println!("  bookmark: {:.6} s  {}", b.sample as f64 / sr, b.label);
            }
            if !sc.notes.is_empty() {
                println!("  notes:    {}", sc.notes.replace('\n', "\n            "));
            }
        }
        Ok(None) => {}
        Err(e) => eprintln!("visgrok: {e}"),
    }
    let n = m.channels;
    let mask = visgrok::block::channel_mask(n);
    let mut det = EdgeDetector::new(mask);
    let mut stats = Stats::new(n);
    let mut tr: Vec<Transition> = Vec::new();
    // Edges per channel per time bucket.
    let mut buckets: Vec<Vec<u64>> = Vec::new();
    let mut first_state = None;
    let mut samples = 0u64;
    // Bucket size: aim for ~60 columns; computed once the total is known
    // (from the index) or default to one second.
    let total = visgrok::vgk::total_samples(path).ok().flatten().unwrap_or(m.samplerate * 60);
    let bucket = (total / 60).max(1);
    while let Some(b) = r.next_block()? {
        if first_state.is_none() {
            let s = b.sample(0) & mask;
            first_state = Some(s);
            stats.init(s);
        }
        tr.clear();
        det.process(&b, &mut tr);
        stats.process(&tr);
        stats.advance(b.end());
        for t in &tr {
            let k = (t.at / bucket) as usize;
            if buckets.len() <= k {
                buckets.resize(k + 1, vec![0; n]);
            }
            let mut c = t.changed();
            while c != 0 {
                let ch = c.trailing_zeros() as usize;
                c &= c - 1;
                buckets[k][ch] += 1;
            }
        }
        samples = b.end();
    }
    buckets.resize(samples.div_ceil(bucket) as usize, vec![0; n]);
    println!("duration:   {:.3} s ({} samples)", samples as f64 / sr, samples);
    // Without an index the total was a guess: merge columns down to ~60.
    let mut bucket = bucket;
    if buckets.len() > 70 {
        let k = buckets.len().div_ceil(60);
        buckets = buckets
            .chunks(k)
            .map(|g| (0..n).map(|ch| g.iter().map(|b| b[ch]).sum()).collect())
            .collect();
        bucket *= k as u64;
    }
    println!();
    println!(
        "{:<8} {:>5} {:>12} {:>14} {:>7} {:>11}  first/last edge",
        "channel", "start", "edges", "frequency", "duty", "min pulse"
    );
    for (i, c) in stats.channels.iter().enumerate() {
        let name = m.name(i);
        let start = first_state.map_or("-", |s| if s >> i & 1 != 0 { "HIGH" } else { "low" });
        let freq = c.median_period().map(|p| fmt_hz(sr / p as f64)).unwrap_or_default();
        let duty = c.duty().map(|d| format!("{:.1}%", d * 100.0)).unwrap_or_default();
        let minp = Some(c.min_high.min(c.min_low))
            .filter(|&v| v != u64::MAX)
            .map(|v| format!("{:.0} ns", v as f64 / sr * 1e9))
            .unwrap_or_default();
        let span = match (buckets.iter().position(|b| b[i] > 0), buckets.iter().rposition(|b| b[i] > 0)) {
            (Some(a), Some(z)) => format!(
                "{:.1}s .. {:.1}s",
                (a as u64 * bucket) as f64 / sr,
                ((z as u64 + 1) * bucket) as f64 / sr
            ),
            _ => "-".into(),
        };
        println!("{name:<8} {start:>5} {:>12} {freq:>14} {duty:>7} {minp:>11}  {span}", c.edges());
    }
    println!();
    println!(
        "activity ({:.2} s per column; ' ' none, ░▒▓█ increasing edge rate):",
        bucket as f64 / sr
    );
    for i in 0..n {
        let name = m.name(i);
        let max = buckets.iter().map(|b| b[i]).max().unwrap_or(0);
        let line: String = buckets
            .iter()
            .map(|b| match b[i] {
                0 => ' ',
                v if max > 0 && v * 4 <= max => '░',
                v if v * 2 <= max => '▒',
                v if v * 4 <= max * 3 => '▓',
                _ => '█',
            })
            .collect();
        println!("{name:<8} |{line}|");
    }
    Ok(())
}
