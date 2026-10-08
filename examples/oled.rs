//! Renders the SSD1306 screen reconstructed from a capture (text art).
//! Usage: oled FILE CLK MOSI DC [CS]
use visgrok::analyzer::{Analyzer, DecoderOptions, SpiProtocol};
use visgrok::formats::{ReadOptions, open};
use visgrok::roles::Role;

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let mut src = open(a[1].as_ref(), &ReadOptions::default())?;
    let info = src.info();
    let mut roles = vec![Role::Unknown; info.channels];
    roles[a[2].parse::<usize>().unwrap()] = Role::SpiClk;
    roles[a[3].parse::<usize>().unwrap()] = Role::SpiMosi;
    roles[a[4].parse::<usize>().unwrap()] = Role::SpiDc;
    if let Some(cs) = a.get(5) {
        roles[cs.parse::<usize>().unwrap()] = Role::SpiCs;
    }
    let mut an = Analyzer::new(info.channels, info.samplerate);
    let opts = DecoderOptions {
        spi_protocol: SpiProtocol::Ssd1306 { width: 128, height: 64 },
        ..Default::default()
    };
    let d = an.decoders_for_roles(&roles, &opts);
    an.set_decoders(d);
    while let Some(b) = src.next_block()? {
        an.process(&b);
    }
    let v = an.decoders()[0].display().expect("display");
    println!("{} {}x{} on={} updates={}", v.title, v.width, v.height, v.on, v.updates);
    let flip = std::env::var("FLIPY").is_ok();
    let px = |x: usize, y: usize| v.pixels[(if flip { v.height - 1 - y } else { y }) * v.width + x];
    for y in (0..v.height).step_by(2) {
        let row: String = (0..v.width)
            .map(|x| match (px(x, y), px(x, y + 1)) {
                (true, true) => '█',
                (true, false) => '▀',
                (false, true) => '▄',
                _ => ' ',
            })
            .collect();
        println!("|{row}|");
    }
    Ok(())
}
