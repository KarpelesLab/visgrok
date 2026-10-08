//! `visgrok`: capture from an SLogic logic analyzer (or a synthetic source),
//! write everything to a sigrok `.sr` file and show live analysis in a TUI.

mod pipeline;
mod ui;

use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use visgrok::Source;
use visgrok::roles::fmt_hz;
use visgrok::slogic::{Config, Pattern, SLogic};
use visgrok::synth::Synth;

use crate::pipeline::Pipeline;

/// Command-line options.
#[derive(Parser, Debug)]
#[command(version, about = "Live capture and analysis for Sipeed SLogic logic analyzers")]
pub struct Args {
    /// Output file (sigrok session, .sr). Nothing is written when omitted.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Sample rate, e.g. 20M, 100M. Defaults to the device's maximum for the
    /// channel count (16ch: 200M, 8ch: 400M, 4ch: 800M); 20M with --demo.
    #[arg(short, long, value_parser = parse_rate)]
    samplerate: Option<u64>,
    /// Number of channels to capture (4, 8 or 16).
    #[arg(short, long, default_value_t = 16)]
    channels: usize,
    /// Input threshold in volts (default: device default, about 2.0 V).
    #[arg(short, long)]
    threshold: Option<f64>,
    /// Select the device with this serial number.
    #[arg(long)]
    serial: Option<String>,
    /// List connected devices and exit.
    #[arg(long)]
    list: bool,
    /// Capture the device's built-in emulation pattern instead of the inputs.
    #[arg(long)]
    emulation: bool,
    /// Stop after this many seconds.
    #[arg(short, long)]
    duration: Option<f64>,
    /// Use a synthetic signal generator instead of hardware.
    #[arg(long)]
    demo: bool,
    /// No TUI: print a status line every second.
    #[arg(long)]
    headless: bool,
    /// Apply auto-detected channel roles (and start decoders) automatically.
    #[arg(long)]
    auto: bool,
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
    if args.demo {
        let rate = args.samplerate.unwrap_or(20_000_000);
        return Ok(Box::new(Synth::new(rate, limit(rate))));
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
    let pipe = match Pipeline::start(source, args.output.clone()) {
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
    while !pipe.finished() {
        std::thread::sleep(Duration::from_secs(1));
        if auto && !applied && pipe.seconds() >= 2.0 {
            pipe.apply_suggestions();
            applied = true;
            for (i, r) in pipe.effective_roles().iter().enumerate() {
                eprintln!("ch{i}: {r}");
            }
        }
        eprintln!("{}", pipe.status_line());
        for line in pipe.drain_log() {
            println!("{line}");
        }
    }
    Ok(())
}
