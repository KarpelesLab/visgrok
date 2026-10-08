//! Measures analyzer throughput with an 8 MHz clock on D0 (8 channels).
//! Usage: analyzer_bench [samplerate_hz] [ignore_mask]
use std::time::Instant;

use visgrok::Block;
use visgrok::analyzer::{Analyzer, DecoderOptions, SpiProtocol};
use visgrok::roles::Role;

fn main() {
    let sr: u64 = std::env::args().nth(1).map_or(100_000_000, |s| s.parse().unwrap());
    let ignore: u16 = std::env::args().nth(2).map_or(0, |s| s.parse().unwrap());
    let n = sr as usize; // one second
    let per = sr as f64 / 16_000_000.0; // half period of 8 MHz
    let data: Vec<u8> = (0..n).map(|i| ((i as f64 / per) as u64 & 1) as u8).collect();
    let mut a = Analyzer::new(8, sr);
    a.set_ignored(ignore);
    let roles = vec![Role::Unknown, Role::Unknown, Role::Uart { baud: 0 }, Role::SpiClk, Role::SpiDc, Role::SpiCs, Role::SpiMosi, Role::Unknown];
    let opts = DecoderOptions { spi_protocol: SpiProtocol::Ssd1306 { width: 128, height: 64 }, ..Default::default() };
    let d = a.decoders_for_roles(&roles, &opts);
    a.set_decoders(d);
    let t = Instant::now();
    for (i, c) in data.chunks(1 << 18).enumerate() {
        a.process(&Block::new((i << 18) as u64, 1, c.to_vec()));
    }
    let dt = t.elapsed().as_secs_f64();
    println!("1 s of 8ch @ {} MHz analyzed in {dt:.3} s ({:.0} MS/s)", sr / 1_000_000, n as f64 / dt / 1e6);
}
