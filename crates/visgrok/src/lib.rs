//! visgrok: driver, streaming and real-time analysis for Sipeed SLogic logic analyzers.
//!
//! - [`block`]: canonical sample representation.
//! - [`edges`]: transition extraction.
//! - [`stats`]: per-channel timing statistics.
//! - [`roles`]: channel role assignment and auto-detection.
//! - [`decode`]: streaming protocol decoders (UART, I2C, SPI).
//! - [`srzip`]: streaming sigrok `.sr` session writer.

pub mod block;
pub mod decode;
pub mod edges;
pub mod roles;
pub mod srzip;
pub mod stats;

pub use block::Block;
pub use edges::{EdgeDetector, Transition};
