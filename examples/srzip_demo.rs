//! Writes a small synthetic `.sr` file (a counter on 8 channels).
use visgrok::srzip::SrZipWriter;

fn main() -> std::io::Result<()> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "demo.sr".into());
    let ch: Vec<String> = (0..8).map(|i| format!("D{i}")).collect();
    let mut w = SrZipWriter::create(&path, &ch, 1_000_000, 1)?;
    let data: Vec<u8> = (0..10_000_000u32).map(|i| (i / 3) as u8).collect();
    w.write(&data)?;
    w.finish()?;
    Ok(())
}
