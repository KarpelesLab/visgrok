//! Streaming protocol decoders working on transitions.

pub mod i2c;
pub mod spi;
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
    },
}

/// A streaming decoder.
pub trait Decoder: Send {
    /// Short description, e.g. `"UART ch3 115200"`.
    fn name(&self) -> String;
    /// Bitmask of channels this decoder looks at.
    fn channels(&self) -> u16;
    /// Sets the line state at the start of capture.
    fn init(&mut self, state: u16);
    /// Feeds one transition (only called when one of [`Decoder::channels`] changed).
    fn transition(&mut self, t: &Transition, out: &mut Vec<Annotation>);
    /// Signals that no transition happens before sample `to`; lets decoders
    /// complete frames that end without a following edge.
    fn advance(&mut self, _to: u64, _out: &mut Vec<Annotation>) {}
}
