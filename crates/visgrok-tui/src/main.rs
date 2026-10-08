//! `visgrok`: capture from an SLogic logic analyzer (or a synthetic source),
//! write everything to a sigrok `.sr` file and show live analysis in a TUI.

mod pipeline;
mod ui;

use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use visgrok::Source;
use visgrok::synth::Synth;

use crate::pipeline::Pipeline;

/// Command-line options.
#[derive(Parser, Debug)]
#[command(version, about = "Live capture and analysis for Sipeed SLogic logic analyzers")]
pub struct Args {
    /// Output file (sigrok session, .sr). Nothing is written when omitted.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Sample rate, e.g. 20M, 100M, 400k.
    #[arg(short, long, default_value = "20M", value_parser = parse_rate)]
    samplerate: u64,
    /// Number of channels to capture.
    #[arg(short, long)]
    channels: Option<usize>,
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
    let limit = args.duration.map(|d| (d * args.samplerate as f64) as u64);
    if args.demo {
        return Ok(Box::new(Synth::new(args.samplerate, limit)));
    }
    Err("no hardware driver yet; use --demo".into())
}

fn main() {
    let args = Args::parse();
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
