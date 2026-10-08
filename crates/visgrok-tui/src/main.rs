//! `visgrok`: capture from an SLogic logic analyzer (or a synthetic source),
//! write everything to a sigrok `.sr` file and show live analysis in a TUI.

mod pipeline;
mod ui;

use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use visgrok::Source;
use visgrok::analyzer::{DecoderOptions, SpiProtocol, parse_uart_format};
use visgrok::roles::{Role, fmt_hz};
use visgrok::slogic::{Config, Pattern, SLogic};
use visgrok::synth::Synth;
use visgrok::vgk::VgkReader;

use crate::pipeline::{Pipeline, Setup};

/// Command-line options.
#[derive(Parser, Debug)]
#[command(version, about = "Live capture and analysis for Sipeed SLogic logic analyzers")]
pub struct Args {
    /// Output file: compressed visgrok capture (.vgk), or a sigrok session
    /// when the name ends in .sr. Nothing is written when omitted.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Replay a .vgk capture instead of capturing (combine with -o to convert).
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
    /// Summarize a .vgk capture (per-channel activity over time) and exit.
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
    /// Keep UART rates given with --role fixed instead of following changes.
    #[arg(long)]
    uart_fixed: bool,
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
        let r = VgkReader::open(p).map_err(|e| format!("{}: {e}", p.display()))?;
        return Ok(Box::new(r));
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
                let ch: usize = ch.trim_start_matches(['D', 'd']).parse().map_err(|_| format!("bad channel in {spec:?}"))?;
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
    let options = DecoderOptions {
        spi_mode: args.spi_mode,
        spi_cs_active_high: args.spi_cs_high,
        spi_protocol: args.spi_proto,
        uart_auto: !args.uart_fixed,
        uart_format: match parse_uart_format(&args.uart_format) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("visgrok: --uart-format: {e}");
                std::process::exit(2);
            }
        },
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
    let setup = Setup { extra, roles, options, names, lossless_analysis: args.input.is_some() };
    let pipe = match Pipeline::start(source, args.output.clone(), setup) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("visgrok: {e}");
            std::process::exit(1);
        }
    };
    let result = if args.headless { headless(&pipe, args.auto) } else { ui::run(&pipe, args.auto) };
    pipe.stop();
    let summary = pipe.join();
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

    let mut r = VgkReader::open(path)?;
    let m = r.meta().clone();
    let sr = m.samplerate as f64;
    println!("file:       {}", path.display());
    println!("device:     {}", m.device);
    println!("channels:   {} @ {}", m.channels, fmt_hz(sr));
    for (k, v) in &m.extra {
        println!("{k:<11} {v}");
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
    let total = visgrok::vgk::total_samples(path)?.unwrap_or(m.samplerate * 60);
    let bucket = (total / 60).max(1);
    while let Some(b) = r.read_block()? {
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
    println!("duration:   {:.3} s ({} samples){}", samples as f64 / sr, samples, if r.truncated { ", file truncated" } else { "" });
    println!();
    println!("{:<8} {:>5} {:>12} {:>14} {:>7} {:>11}  first/last edge", "channel", "start", "edges", "frequency", "duty", "min pulse");
    for (i, c) in stats.channels.iter().enumerate() {
        let name = m.names.get(i).cloned().unwrap_or_else(|| format!("D{i}"));
        let start = first_state.map_or("-", |s| if s >> i & 1 != 0 { "HIGH" } else { "low" });
        let freq = c.median_period().map(|p| fmt_hz(sr / p as f64)).unwrap_or_default();
        let duty = c.duty().map(|d| format!("{:.1}%", d * 100.0)).unwrap_or_default();
        let minp = Some(c.min_high.min(c.min_low)).filter(|&v| v != u64::MAX).map(|v| format!("{:.0} ns", v as f64 / sr * 1e9)).unwrap_or_default();
        let span = match (buckets.iter().position(|b| b[i] > 0), buckets.iter().rposition(|b| b[i] > 0)) {
            (Some(a), Some(z)) => format!("{:.1}s .. {:.1}s", (a as u64 * bucket) as f64 / sr, ((z as u64 + 1) * bucket) as f64 / sr),
            _ => "-".into(),
        };
        println!("{name:<8} {start:>5} {:>12} {freq:>14} {duty:>7} {minp:>11}  {span}", c.edges());
    }
    println!();
    println!("activity ({:.2} s per column; ' ' none, ░▒▓█ increasing edge rate):", bucket as f64 / sr);
    for i in 0..n {
        let name = m.names.get(i).cloned().unwrap_or_else(|| format!("D{i}"));
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
