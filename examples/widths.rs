//! Pulse widths of one channel of a capture (recent ones, in order) and the
//! UART bit time they suggest. Usage: widths FILE CHANNEL
use visgrok::analyzer::Analyzer;
use visgrok::formats::{ReadOptions, open};

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let ch: usize = a[2].parse().unwrap();
    let mut src = open(a[1].as_ref(), &ReadOptions::default())?;
    let info = src.info();
    let mut an = Analyzer::new(info.channels, info.samplerate);
    while let Some(b) = src.next_block()? {
        an.process(&b);
    }
    let c = &an.stats().channels[ch];
    let w = c.recent_widths_ordered();
    println!("{} edges, duty {:?}, level now {}", c.edges(), c.duty(), c.level);
    println!("recent widths (samples, oldest first): {:?}", &w[..w.len().min(120)]);
    match visgrok::decode::uart::estimate_bit_time(&w, 10.0) {
        Some(bit) => println!("UART bit time {bit:.1} samples ({:.0} baud)", info.samplerate as f64 / bit),
        None => println!("no UART bit time fits"),
    }
    Ok(())
}
