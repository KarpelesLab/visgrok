//! Streaming protocol decoders working on transitions.

pub mod i2c;
pub mod spi;
pub mod ssd1306;
pub mod uart;

use crate::edges::Transition;

/// Something a decoder recognized, spanning samples `start..end`.
#[derive(Clone, Debug, PartialEq)]
pub struct Annotation {
    /// First sample of the annotated region.
    pub start: u64,
    /// Sample one past the annotated region.
    pub end: u64,
    /// What was decoded.
    pub event: Event,
}

/// Decoded protocol events.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// A UART frame.
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
    UartFormat {
        /// e.g. `8E2`.
        format: String,
    },
    /// The UART decoder locked onto (or switched to) a baud rate.
    UartBaud {
        /// New rate in bits per second.
        baud: u32,
    },
    /// I2C start or repeated start.
    I2cStart,
    /// I2C stop.
    I2cStop,
    /// I2C address byte.
    I2cAddress {
        /// 7-bit address.
        addr: u8,
        /// Read (true) or write transfer.
        read: bool,
        /// Acknowledged by a target.
        ack: bool,
    },
    /// I2C data byte.
    I2cData {
        /// Byte value.
        value: u8,
        /// Acknowledged.
        ack: bool,
    },
    /// SPI chip select change.
    SpiSelect(bool),
    /// One SPI word.
    SpiWord {
        /// Word clocked on MOSI, if assigned.
        mosi: Option<u32>,
        /// Word clocked on MISO, if assigned.
        miso: Option<u32>,
        /// Level of the D/C line at the last bit, if assigned (true = data).
        dc: Option<bool>,
    },
    /// A message from a higher-level protocol decoder (e.g. SSD1306).
    Protocol {
        /// Protocol name.
        proto: &'static str,
        /// Human-readable description.
        text: String,
    },
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
#[derive(Clone, Debug)]
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
