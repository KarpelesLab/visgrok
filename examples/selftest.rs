//! Hardware self-test: captures the device's emulation pattern and checks
//! every sample, which detects any data lost between the device and the host.
//!
//! Usage: selftest [channels] [rate_hz] [seconds]
//!        selftest FILE.vgk   (verify a recorded emulation-pattern capture)
use std::time::Instant;

use visgrok::Source;
use visgrok::slogic::{Config, Pattern, SLogic};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if let Some(path) = std::env::args().nth(1).filter(|a| a.ends_with(".vgk")) {
        let mut r = visgrok::vgk::VgkReader::open(&path)?;
        let channels = r.meta().channels;
        let t0 = Instant::now();
        let (samples, errors) = check(&mut r, channels)?;
        println!(
            "{path}: {samples} samples read in {:.2}s, {errors} mismatches, index total {:?}, truncated {}",
            t0.elapsed().as_secs_f64(),
            r.total,
            r.truncated
        );
        return Ok(());
    }
    let mut args = std::env::args().skip(1);
    let channels: usize = args.next().map_or(Ok(16), |s| s.parse())?;
    let rate: u64 = args.next().map_or(Ok(20_000_000), |s| s.parse())?;
    let secs: f64 = args.next().map_or(Ok(2.0), |s| s.parse())?;

    for f in visgrok::slogic::list()? {
        println!("found {f}");
    }
    let dev = SLogic::open(None)?;
    let mut cfg = Config::new(channels, rate);
    cfg.pattern = Pattern::Emulation;
    cfg.limit = Some((rate as f64 * secs) as u64);
    let t0 = Instant::now();
    let mut cap = dev.start(cfg)?;
    let (samples, errors) = check(&mut cap, channels)?;
    let dt = t0.elapsed().as_secs_f64();
    let st = cap.stats();
    println!(
        "{samples} samples in {dt:.2}s, {} transfers, {} timeouts, {} restarts, measured {:.1} MB/s (expected {:.1}); {errors} mismatches",
        st.transfers,
        st.timeouts,
        st.restarts,
        st.measured_rate.unwrap_or(0.0) / 1e6,
        cap.config().byte_rate() / 1e6
    );
    Ok(())
}

/// Checks every sample against the emulation pattern; returns (samples, mismatches).
fn check(src: &mut dyn Source, channels: usize) -> std::io::Result<(u64, u64)> {
    let mask = visgrok::block::channel_mask(channels);
    let mut expected: Option<u64> = None;
    let mut errors = 0u64;
    let mut samples = 0u64;
    while let Some(b) = src.next_block()? {
        for (k, v) in b.samples().enumerate() {
            let i = b.start + k as u64;
            // The pattern's phase at start is arbitrary at high rates: lock
            // onto it from the first sample.
            let e = match expected {
                Some(base) => {
                    let n = i + base;
                    ((n & !7) | (7 - (n & 7))) as u32 & mask
                }
                None => {
                    let v = v as u64;
                    // Find n with pattern(n) == v (mod mask): n = (v & !7) | (7 - (v & 7)).
                    let n = (v & !7) | (7 - (v & 7));
                    expected = Some(n.wrapping_sub(i));
                    v as u32
                }
            };
            if v != e {
                if errors < 10 {
                    println!("mismatch at sample {i}: got {v:#010x} expected {e:#010x}");
                }
                errors += 1;
                // Resynchronize after a gap.
                let n = (v as u64 & !7) | (7 - (v as u64 & 7));
                expected = Some(n.wrapping_sub(i));
            }
        }
        samples = b.end();
    }
    Ok((samples, errors))
}
