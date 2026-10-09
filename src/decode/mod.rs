//! Streaming protocol decoders working on transitions.

pub mod hci;
pub mod i2c;
pub mod iso7816;
pub mod sd;
pub mod seph;
pub mod spi;
pub mod ssd1306;
pub mod uart;

use crate::edges::Transition;

/// Something a decoder recognized, spanning samples `start..end`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Annotation {
    /// First sample of the annotated region.
    pub start: u64,
    /// Sample one past the annotated region.
    pub end: u64,
    /// What was decoded.
    pub event: Event,
}

impl Event {
    /// A message from a protocol decoder: `text` describes it, `data` holds
    /// the bytes it is about.
    pub fn protocol(proto: &'static str, text: impl Into<String>, data: Option<std::sync::Arc<[u8]>>) -> Event {
        Event::Protocol {
            proto,
            text: text.into(),
            data,
        }
    }
}

impl Annotation {
    /// An event spanning samples `start..end`.
    pub fn new(start: u64, end: u64, event: Event) -> Annotation {
        Annotation { start, end, event }
    }
}

/// Decoded protocol events.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Event {
    /// A UART frame.
    #[non_exhaustive]
    UartByte {
        /// Data bits.
        value: u16,
        /// Stop bit was low.
        framing_error: bool,
        /// Parity mismatch (if parity is enabled).
        parity_error: bool,
    },
    /// UART line held low for a whole frame or longer.
    UartBreak,
    /// The UART decoder detected the frame format.
    #[non_exhaustive]
    UartFormat {
        /// e.g. `8E2`.
        format: String,
    },
    /// The UART decoder locked onto (or switched to) a baud rate.
    #[non_exhaustive]
    UartBaud {
        /// New rate in bits per second.
        baud: u32,
    },
    /// I2C start or repeated start.
    I2cStart,
    /// I2C stop.
    I2cStop,
    /// I2C address byte.
    #[non_exhaustive]
    I2cAddress {
        /// 7-bit address.
        addr: u8,
        /// Read (true) or write transfer.
        read: bool,
        /// Acknowledged by a target.
        ack: bool,
    },
    /// I2C data byte.
    #[non_exhaustive]
    I2cData {
        /// Byte value.
        value: u8,
        /// Acknowledged.
        ack: bool,
    },
    /// SPI chip select change.
    SpiSelect(bool),
    /// One SPI word.
    #[non_exhaustive]
    SpiWord {
        /// Word clocked on MOSI, if assigned.
        mosi: Option<u32>,
        /// Word clocked on MISO, if assigned.
        miso: Option<u32>,
        /// Level of the D/C line at the last bit, if assigned (true = data).
        dc: Option<bool>,
    },
    /// What a display shows after an update (e.g. SSD1306 RAM writes).
    Frame(std::sync::Arc<DisplayView>),
    /// A message from a higher-level protocol decoder (e.g. SSD1306).
    #[non_exhaustive]
    Protocol {
        /// Protocol name.
        proto: &'static str,
        /// Human-readable description.
        text: String,
        /// The bytes the event is about (a data block, an ATR, a screen
        /// write...), for detailed views.
        data: Option<std::sync::Arc<[u8]>>,
    },
}

/// One line describing the event, as the command line and web UI show it.
impl std::fmt::Display for Event {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Event::UartByte {
                value,
                framing_error,
                parity_error,
            } => {
                let c = char::from_u32(*value as u32).filter(|c| c.is_ascii_graphic() || *c == ' ');
                let mut s = match c {
                    Some(c) => format!("{value:02x} '{c}'"),
                    None => format!("{value:02x}"),
                };
                if *framing_error {
                    s += " [framing error]";
                }
                if *parity_error {
                    s += " [parity error]";
                }
                s
            }
            Event::UartBreak => "BREAK".into(),
            Event::I2cStart => "START".into(),
            Event::I2cStop => "STOP".into(),
            Event::I2cAddress { addr, read, ack } => {
                format!(
                    "addr {addr:#04x} {} {}",
                    if *read { "R" } else { "W" },
                    if *ack { "ACK" } else { "NAK" }
                )
            }
            Event::I2cData { value, ack } => format!("data {value:02x} {}", if *ack { "ACK" } else { "NAK" }),
            Event::SpiSelect(true) => "CS asserted".into(),
            Event::SpiSelect(false) => "CS released".into(),
            Event::SpiWord { mosi, miso, dc } => {
                let f = |v: &Option<u32>| v.map_or("--".to_string(), |v| format!("{v:02x}"));
                let mut s = format!("mosi {} miso {}", f(mosi), f(miso));
                if let Some(dc) = dc {
                    s += if *dc { " [data]" } else { " [cmd]" };
                }
                s
            }
            Event::UartBaud { baud } => format!("── baud rate {baud} ──"),
            Event::UartFormat { format } => format!("── frame format {format} ──"),
            Event::Protocol { text, .. } => text.clone(),
            Event::Frame(v) => format!("{} screen update {}", v.title, v.updates),
        };
        f.write_str(&s)
    }
}

impl DisplayView {
    /// A blank, switched-off `width` x `height` display.
    pub fn new(title: &'static str, width: usize, height: usize) -> DisplayView {
        DisplayView {
            title,
            width,
            height,
            pixels: vec![false; width * height],
            on: false,
            updates: 0,
        }
    }
}

/// A streaming decoder.
pub trait Decoder: Send {
    /// Short description, e.g. `"UART ch3 115200"`.
    fn name(&self) -> String;
    /// Bitmask of channels this decoder looks at.
    fn channels(&self) -> u32;
    /// Sets the line state at the start of capture.
    fn init(&mut self, state: u32);
    /// Feeds one transition (only called when one of [`Decoder::channels`] changed).
    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>);
    /// Signals that no transition happens before sample `to`; lets decoders
    /// complete frames that end without a following edge.
    fn advance(&mut self, _to: u64, _out: &mut Vec<Annotation>) {}
    /// A picture of a display this decoder reconstructs, if any.
    fn display(&self) -> Option<DisplayView> {
        None
    }
}

/// A monochrome display reconstructed by a decoder.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct DisplayView {
    /// Controller name.
    pub title: &'static str,
    /// Width in pixels.
    pub width: usize,
    /// Height in pixels.
    pub height: usize,
    /// Row-major pixels, true = lit.
    pub pixels: Vec<bool>,
    /// Panel switched on.
    pub on: bool,
    /// Number of RAM write bursts seen.
    pub updates: u64,
}
