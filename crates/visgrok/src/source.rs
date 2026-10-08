//! Sample sources: anything that produces a stream of [`Block`]s.

use std::io;

use crate::block::Block;

/// Static description of a capture stream.
#[derive(Clone, Debug)]
pub struct CaptureInfo {
    /// Human-readable device name.
    pub device: String,
    /// Number of channels in each sample.
    pub channels: usize,
    /// Samples per second.
    pub samplerate: u64,
    /// Bytes per sample in emitted blocks (1 or 2).
    pub unit_size: usize,
}

/// A stream of sample blocks.
pub trait Source: Send {
    /// Describes the stream.
    fn info(&self) -> CaptureInfo;
    /// Returns the next block, or `None` at the end of the capture. Blocks
    /// must be contiguous: each starts where the previous one ended.
    fn next_block(&mut self) -> io::Result<Option<Block>>;
    /// Asks the source to stop; subsequent calls to `next_block` drain what is
    /// buffered and then return `None`.
    fn stop(&mut self) {}
}
