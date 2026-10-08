//! Prints transitions from a .vgk capture.
//! Usage: vgk_edges FILE CHANNEL_MASK [FROM_S] [TO_S] [MAX]
use visgrok::EdgeDetector;
use visgrok::vgk::VgkReader;

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let mut r = VgkReader::open(&a[1])?;
    let mask: u16 = a[2].parse().unwrap();
    let from: f64 = a.get(3).map_or(0.0, |s| s.parse().unwrap());
    let to: f64 = a.get(4).map_or(f64::MAX, |s| s.parse().unwrap());
    let max: usize = a.get(5).map_or(200, |s| s.parse().unwrap());
    let sr = r.meta().samplerate as f64;
    let names = r.meta().names.clone();
    let mut det = EdgeDetector::new(mask);
    let mut tr = Vec::new();
    let mut shown = 0;
    let mut last = [0u64; 16];
    while let Some(b) = r.read_block()? {
        if (b.end() as f64) < from * sr {
            // still feed the detector so levels are right
            tr.clear();
            det.process(&b, &mut tr);
            for t in &tr {
                let mut c = t.changed();
                while c != 0 {
                    let ch = c.trailing_zeros() as usize;
                    c &= c - 1;
                    last[ch] = t.at;
                }
            }
            continue;
        }
        if b.start as f64 > to * sr {
            break;
        }
        tr.clear();
        det.process(&b, &mut tr);
        for t in &tr {
            if (t.at as f64) < from * sr || (t.at as f64) > to * sr {
                continue;
            }
            let mut c = t.changed();
            while c != 0 {
                let ch = c.trailing_zeros() as usize;
                c &= c - 1;
                let dt = (t.at - last[ch]) as f64 / sr;
                last[ch] = t.at;
                let dir = if t.now >> ch & 1 != 0 { "rise" } else { "fall" };
                println!("{:14.9}s {:<6} {dir}  (+{:.3} us since previous edge)", t.at as f64 / sr, names[ch], dt * 1e6);
                shown += 1;
            }
            if shown >= max {
                return Ok(());
            }
        }
    }
    Ok(())
}
