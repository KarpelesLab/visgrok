//! Canonical in-memory sample representation.
//!
//! Whatever the device's wire format, the driver converts incoming data into
//! [`Block`]s: a run of consecutive samples, each stored as a little-endian
//! unit of 1 byte (up to 8 channels), 2 bytes (up to 16) or 4 bytes (up to
//! 32). Channel `n` is bit `n` of the unit. This matches sigrok's logic packet layout, so
//! blocks can be written to `.sr` files without further conversion.

/// One sample as a channel bitmask (channel `n` = bit `n`).
pub type Sample = u32;

/// Mask of the low `channels` channels.
pub fn channel_mask(channels: usize) -> Sample {
    if channels >= 32 { Sample::MAX } else { (1 << channels) - 1 }
}

/// Bytes per sample needed for `channels` channels.
pub fn unit_size_for(channels: usize) -> usize {
    match channels {
        0..=8 => 1,
        9..=16 => 2,
        _ => 4,
    }
}

/// A run of consecutive samples.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Block {
    /// Index of the first sample of this block since the start of capture.
    pub start: u64,
    /// Bytes per sample: 1, 2 or 4.
    pub unit_size: usize,
    /// Packed samples, `unit_size` bytes each, little endian.
    pub data: Vec<u8>,
}

impl Block {
    /// Creates a block. `data.len()` must be a multiple of `unit_size`.
    pub fn new(start: u64, unit_size: usize, data: Vec<u8>) -> Block {
        assert!(matches!(unit_size, 1 | 2 | 4), "unit_size must be 1, 2 or 4");
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
    pub fn sample(&self, i: usize) -> Sample {
        match self.unit_size {
            1 => self.data[i] as Sample,
            2 => u16::from_le_bytes([self.data[2 * i], self.data[2 * i + 1]]) as Sample,
            _ => Sample::from_le_bytes(self.data[4 * i..4 * i + 4].try_into().unwrap()),
        }
    }

    /// Copies samples `from..to` (indices within the block) into a new block.
    pub fn slice(&self, from: usize, to: usize) -> Block {
        let to = to.min(self.len());
        let from = from.min(to);
        Block::new(
            self.start + from as u64,
            self.unit_size,
            self.data[from * self.unit_size..to * self.unit_size].to_vec(),
        )
    }

    /// Iterates over all samples as channel bitmasks.
    pub fn samples(&self) -> impl Iterator<Item = Sample> + '_ {
        (0..self.len()).map(move |i| self.sample(i))
    }
}
