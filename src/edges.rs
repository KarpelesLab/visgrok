//! Transition extraction: turns sample blocks into a sparse list of changes.
//!
//! Almost all real-time analysis works on transitions rather than raw samples:
//! a multi-channel stream at hundreds of MS/s is mostly runs of identical
//! values, and the interesting information is where (and which) bits change.

use crate::block::{Block, Sample};

/// A change of state on one or more channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    /// Sample index at which the new state first appears.
    pub at: u64,
    /// State before the change.
    pub prev: Sample,
    /// State from `at` onwards.
    pub now: Sample,
}

impl Transition {
    /// Bitmask of channels that changed.
    pub fn changed(&self) -> Sample {
        self.prev ^ self.now
    }

    /// Bitmask of channels with a rising edge.
    pub fn rising(&self) -> Sample {
        !self.prev & self.now
    }

    /// Bitmask of channels with a falling edge.
    pub fn falling(&self) -> Sample {
        self.prev & !self.now
    }
}

/// Stateful transition detector spanning block boundaries.
#[derive(Clone, Debug, Default)]
pub struct EdgeDetector {
    last: Option<Sample>,
    /// Channels to watch; changes on other channels are ignored.
    mask: Sample,
}

impl EdgeDetector {
    /// Creates a detector watching the channels in `mask`.
    pub fn new(mask: Sample) -> EdgeDetector {
        EdgeDetector { last: None, mask }
    }

    /// Current state (after the last processed sample), if any sample was seen.
    pub fn state(&self) -> Option<Sample> {
        self.last
    }

    /// Appends the transitions found in `block` to `out`.
    ///
    /// The first sample ever seen establishes the initial state and does not
    /// produce a transition.
    pub fn process(&mut self, block: &Block, out: &mut Vec<Transition>) {
        if block.is_empty() {
            return;
        }
        let mask = self.mask;
        let mut prev = self.last.unwrap_or(block.sample(0) & mask);
        let unit = block.unit_size;
        let per_word = 8 / unit;
        // `prev` and `mask` repeated across a 64-bit word, for skipping runs
        // of unchanged samples eight bytes at a time.
        let splat = |v: Sample| -> u64 {
            let b = (v as u64).to_le_bytes();
            let mut w = [0u8; 8];
            for (i, x) in w.iter_mut().enumerate() {
                *x = b[i % unit];
            }
            u64::from_ne_bytes(w)
        };
        let wmask = splat(mask);
        let mut wprev = splat(prev);
        let data = &block.data;
        let n = block.len();
        let mut i = 0;
        while i < n {
            while i + per_word <= n {
                let w = u64::from_ne_bytes(data[i * unit..i * unit + 8].try_into().unwrap());
                if (w ^ wprev) & wmask != 0 {
                    break;
                }
                i += per_word;
            }
            let end = (i + per_word).min(n);
            while i < end {
                let v = block.sample(i) & mask;
                if v != prev {
                    out.push(Transition {
                        at: block.start + i as u64,
                        prev,
                        now: v,
                    });
                    prev = v;
                    wprev = splat(prev);
                }
                i += 1;
            }
        }
        self.last = Some(prev);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_across_blocks() {
        let mut d = EdgeDetector::new(0xffff);
        let mut out = Vec::new();
        let mut data = vec![0u8; 40];
        data[17] = 1;
        data[18] = 1;
        d.process(&Block::new(0, 1, data), &mut out);
        d.process(&Block::new(40, 1, vec![0, 0, 2]), &mut out);
        assert_eq!(
            out,
            vec![
                Transition { at: 17, prev: 0, now: 1 },
                Transition { at: 19, prev: 1, now: 0 },
                Transition { at: 42, prev: 0, now: 2 },
            ]
        );
    }

    #[test]
    fn detects_16bit() {
        let mut d = EdgeDetector::new(0xffff);
        let mut out = Vec::new();
        let mut s = [0x8000u16; 21];
        s[13] = 0x8001;
        let data: Vec<u8> = s.iter().flat_map(|v| v.to_le_bytes()).collect();
        d.process(&Block::new(100, 2, data), &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(
            out[0],
            Transition {
                at: 113,
                prev: 0x8000,
                now: 0x8001
            }
        );
        assert_eq!(out[1].at, 114);
    }

    #[test]
    fn detects_32bit() {
        let mut d = EdgeDetector::new(u32::MAX);
        let mut out = Vec::new();
        let mut s = [0x8000_0000u32; 23];
        s[9] = 0x8001_0000;
        s[10] = 0x8001_0000;
        s[22] = 0;
        let data: Vec<u8> = s.iter().flat_map(|v| v.to_le_bytes()).collect();
        d.process(&Block::new(0, 4, data), &mut out);
        assert_eq!(
            out,
            vec![
                Transition {
                    at: 9,
                    prev: 0x8000_0000,
                    now: 0x8001_0000
                },
                Transition {
                    at: 11,
                    prev: 0x8001_0000,
                    now: 0x8000_0000
                },
                Transition {
                    at: 22,
                    prev: 0x8000_0000,
                    now: 0
                },
            ]
        );
    }

    #[test]
    fn mask_ignores_channels() {
        let mut d = EdgeDetector::new(0x01);
        let mut out = Vec::new();
        d.process(&Block::new(0, 1, vec![0, 2, 2, 3, 2]), &mut out);
        assert_eq!(
            out,
            vec![Transition { at: 3, prev: 0, now: 1 }, Transition { at: 4, prev: 1, now: 0 }]
        );
    }
}
