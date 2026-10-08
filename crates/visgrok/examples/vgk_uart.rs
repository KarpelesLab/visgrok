//! Decodes one channel of a .vgk capture as UART.
//! Usage: vgk_uart FILE CHANNEL [FORMAT e.g. 8N1/8E1/8E2] [FROM_S] [TO_S]
use visgrok::decode::uart::{Parity, Uart, UartConfig};
use visgrok::decode::{Decoder, Event};
use visgrok::vgk::VgkReader;
use visgrok::EdgeDetector;

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let mut r = VgkReader::open(&a[1])?;
    let ch: u8 = a[2].parse().unwrap();
    let fmt = a.get(3).map_or("8N1", String::as_str).as_bytes().to_vec();
    let from: f64 = a.get(4).map_or(0.0, |s| s.parse().unwrap());
    let to: f64 = a.get(5).map_or(f64::MAX, |s| s.parse().unwrap());
    let sr = r.meta().samplerate;
    let mut cfg = UartConfig::auto(ch);
    cfg.data_bits = fmt[0] - b'0';
    cfg.parity = match fmt[1] {
        b'E' => Parity::Even,
        b'O' => Parity::Odd,
        _ => Parity::None,
    };
    let mut d = Uart::new(cfg, sr);
    let mut det = EdgeDetector::new(1 << ch);
    let mut tr = Vec::new();
    let mut out = Vec::new();
    let mut started = false;
    while let Some(b) = r.read_block()? {
        if (b.end() as f64) < from * sr as f64 {
            tr.clear();
            det.process(&b, &mut tr);
            continue;
        }
        if !started {
            d.init(det.state().unwrap_or(0));
            started = true;
        }
        tr.clear();
        det.process(&b, &mut tr);
        for t in &tr {
            d.transition(t, &mut out);
        }
        d.advance(b.end(), &mut out);
        if b.start as f64 > to * sr as f64 {
            break;
        }
    }
    for a in &out {
        let t = a.start as f64 / sr as f64;
        match &a.event {
            Event::UartByte { value, framing_error, parity_error } => println!(
                "{t:.6}s {value:02x}{}{}",
                if *framing_error { " FRAMING" } else { "" },
                if *parity_error { " PARITY" } else { "" }
            ),
            e => println!("{t:.6}s {e:?}"),
        }
    }
    Ok(())
}
