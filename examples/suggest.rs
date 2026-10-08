//! Prints the auto-detected role of every channel of a capture.
//! Usage: suggest FILE
use visgrok::analyzer::Analyzer;
use visgrok::formats::{ReadOptions, open};

fn main() -> std::io::Result<()> {
    let path = std::env::args().nth(1).expect("usage: suggest FILE");
    let mut src = open(path.as_ref(), &ReadOptions::default())?;
    let info = src.info();
    let mut a = Analyzer::new(info.channels, info.samplerate);
    while let Some(b) = src.next_block()? {
        a.process(&b);
    }
    for (i, s) in a.suggest().iter().enumerate() {
        println!("{:<6} {:<28} {:.0}%", info.name(i), s.role.to_string(), s.confidence * 100.0);
    }
    Ok(())
}
