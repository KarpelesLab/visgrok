//! Canonical in-memory sample representation.
//!
//! Whatever the device's wire format, the driver converts incoming data into
//! [`Block`]s: a run of consecutive samples, each stored as a little-endian
//! unit of 1 byte (up to 8 channels) or 2 bytes (up to 16 channels). Channel
//! `n` is bit `n` of the unit. This matches sigrok's logic packet layout, so
//! blocks can be written to `.sr` files without further conversion.

/// A run of consecutive samples.
#[derive(Clone, Debug)]
pub struct Block {
    /// Index of the first sample of this block since the start of capture.
    pub start: u64,
    /// Bytes per sample: 1 or 2.
    pub unit_size: usize,
    /// Packed samples, `unit_size` bytes each, little endian.
    pub data: Vec<u8>,
}

impl Block {
    /// Creates a block. `data.len()` must be a multiple of `unit_size`.
    pub fn new(start: u64, unit_size: usize, data: Vec<u8>) -> Block {
        assert!(unit_size == 1 || unit_size == 2, "unit_size must be 1 or 2");
        assert_eq!(data.len() % unit_size, 0, "partial sample in block");
        Block { start, unit_size, data }
    }

    /// Number of samples in the block.
    pub fn len(&self) -> usize {
        self.data.len() / self.unit_size
    }

    /// True when the block holds no samples.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Index one past the last sample of this block.
    pub fn end(&self) -> u64 {
        self.start + self.len() as u64
    }

    /// Sample `i` of the block as a channel bitmask.
    #[inline]
    pub fn sample(&self, i: usize) -> u16 {
        match self.unit_size {
            1 => self.data[i] as u16,
            _ => u16::from_le_bytes([self.data[2 * i], self.data[2 * i + 1]]),
        }
    }

    /// Iterates over all samples as channel bitmasks.
    pub fn samples(&self) -> impl Iterator<Item = u16> + '_ {
        (0..self.len()).map(move |i| self.sample(i))
    }
}
