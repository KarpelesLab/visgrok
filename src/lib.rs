//! visgrok: driver, streaming and real-time analysis for Sipeed SLogic logic analyzers.
//!
//! - [`analyzer`]: the real-time analysis pipeline.
//! - [`block`]: canonical sample representation.
//! - [`edges`]: transition extraction.
//! - [`stats`]: per-channel timing statistics.
//! - [`roles`]: channel role assignment and auto-detection.
//! - [`decode`]: streaming protocol decoders (UART, I2C, SPI).
//! - [`source`]: the [`Source`] trait for block streams; [`synth`]: a synthetic source.
//! - [`sidecar`]: `<capture>.json` files describing channels, buses and notes.
//! - [`slogic`]: the SLogic USB driver.
//! - [`formats`]: reading, writing and converting `.vgk`, `.sr`, `.vcd`, `.bin`.
//! - [`store`]: random-access view of live or recorded captures, for UIs.
//! - [`vgk`]: the compressed visgrok capture format.
//! - [`srzip`]: streaming sigrok `.sr` session writer.

pub mod analyzer;
pub mod block;
pub mod decode;
pub mod edges;
pub mod formats;
pub mod json;
pub mod pool;
pub mod roles;
pub mod sidecar;
pub mod slogic;
pub mod source;
pub mod srzip;
pub mod stats;
pub mod store;
pub mod synth;
pub mod vgk;

pub use block::Block;
pub use edges::{EdgeDetector, Transition};
pub use source::{CaptureInfo, Source};
