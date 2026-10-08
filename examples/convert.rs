//! Converts between capture formats (format from the extension).
//! Usage: convert IN OUT [samplerate channels]   (rate/channels for raw .bin input)
use visgrok::formats::{ReadOptions, WriteOptions, convert};
use visgrok::srzip::SrCompression;

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let mut ropts = ReadOptions::default();
    ropts.samplerate = a.get(3).map(|s| s.parse().unwrap());
    ropts.channels = a.get(4).map(|s| s.parse().unwrap());
    let mut wopts = WriteOptions::default();
    wopts.sr_compression = SrCompression::Deflate;
    let r = convert(a[1].as_ref(), a[2].as_ref(), &ropts, &wopts, |_| {})?;
    println!(
        "{} samples, {} ch @ {} Hz, {} bytes",
        r.samples, r.info.channels, r.info.samplerate, r.bytes
    );
    Ok(())
}
