//! Debug: pulse width histogram of one channel. Usage: widths FILE CH
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
    let mut w = c.recent_widths().to_vec();
    w.sort_unstable();
    println!("{} widths, duty {:?}, level {}, edges {}", w.len(), c.duty(), c.level, c.edges());
    println!("{:?}", &w[..w.len().min(80)]);
    println!("... {:?}", &w[w.len().saturating_sub(20)..]);
    let glitch = (info.samplerate as f64 * 25e-9) as u64;
    let mut m: Vec<u64> = Vec::new();
    let mut merge_next = false;
    for x in c.recent_widths_ordered() {
        if x <= glitch {
            if let Some(l) = m.last_mut() {
                *l += x;
                merge_next = true;
            }
        } else if merge_next {
            *m.last_mut().unwrap() += x;
            merge_next = false;
        } else {
            m.push(x);
        }
    }
    let mut ms = m.clone();
    ms.sort_unstable();
    println!("merged {}: {:?}", ms.len(), &ms[..ms.len().min(60)]);
    println!("ordered: {:?}", &m[..m.len().min(60)]);
    Ok(())
}
