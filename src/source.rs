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
    /// Bytes per sample in emitted blocks (1, 2 or 4).
    pub unit_size: usize,
    /// Channel names; empty for the default `D<n>`.
    pub names: Vec<String>,
}

impl CaptureInfo {
    /// Name of channel `i`.
    pub fn name(&self, i: usize) -> String {
        self.names
            .get(i)
            .filter(|n| !n.is_empty())
            .cloned()
            .unwrap_or_else(|| format!("D{i}"))
    }

    /// All channel names.
    pub fn all_names(&self) -> Vec<String> {
        (0..self.channels).map(|i| self.name(i)).collect()
    }
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
