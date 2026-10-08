//! Transition extraction: turns sample blocks into a sparse list of changes.
//!
//! Almost all real-time analysis works on transitions rather than raw samples:
//! a 16-channel stream at hundreds of MS/s is mostly runs of identical values,
//! and the interesting information is where (and which) bits change.

use crate::block::Block;

/// A change of state on one or more channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    /// Sample index at which the new state first appears.
    pub at: u64,
    /// State before the change.
    pub prev: u16,
    /// State from `at` onwards.
    pub now: u16,
}

impl Transition {
    /// Bitmask of channels that changed.
    pub fn changed(&self) -> u16 {
        self.prev ^ self.now
    }

    /// Bitmask of channels with a rising edge.
    pub fn rising(&self) -> u16 {
        !self.prev & self.now
    }

    /// Bitmask of channels with a falling edge.
    pub fn falling(&self) -> u16 {
        self.prev & !self.now
    }
}

/// Stateful transition detector spanning block boundaries.
#[derive(Clone, Debug, Default)]
pub struct EdgeDetector {
    last: Option<u16>,
    /// Channels to watch; changes on other channels are ignored.
    mask: u16,
}

impl EdgeDetector {
    /// Creates a detector watching the channels in `mask`.
    pub fn new(mask: u16) -> EdgeDetector {
        EdgeDetector { last: None, mask }
    }

    /// Current state (after the last processed sample), if any sample was seen.
    pub fn state(&self) -> Option<u16> {
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
        let mut prev = match self.last {
            Some(v) => v,
            None => block.sample(0) & mask,
        };
        match block.unit_size {
            1 => scan(&block.data, block.start, prev as u8, mask as u8, |at, p, n| {
                out.push(Transition { at, prev: p as u16, now: n as u16 })
            }, &mut prev),
            _ => scan16(block, mask, &mut prev, out),
        }
        self.last = Some(prev);
    }
}

fn scan(data: &[u8], start: u64, mut prev: u8, mask: u8, mut emit: impl FnMut(u64, u8, u8), last: &mut u16) {
    let mut i = 0;
    while i < data.len() {
        // Skip quickly over runs equal to `prev` eight bytes at a time.
        let splat = u64::from_ne_bytes([prev; 8]);
        let mmask = u64::from_ne_bytes([mask; 8]);
        while i + 8 <= data.len() {
            let w = u64::from_ne_bytes(data[i..i + 8].try_into().unwrap());
            if (w ^ splat) & mmask != 0 {
                break;
            }
            i += 8;
        }
        let end = (i + 8).min(data.len());
        while i < end {
            let v = data[i] & mask;
            if v != prev {
                emit(start + i as u64, prev, v);
                prev = v;
            }
            i += 1;
        }
    }
    *last = prev as u16;
}

fn scan16(block: &Block, mask: u16, prev: &mut u16, out: &mut Vec<Transition>) {
    let data = &block.data;
    let n = block.len();
    let mut i = 0;
    while i < n {
        let p = *prev;
        let splat = u64::from_ne_bytes({
            let b = p.to_le_bytes();
            [b[0], b[1], b[0], b[1], b[0], b[1], b[0], b[1]]
        });
        let mmask = u64::from_ne_bytes({
            let b = mask.to_le_bytes();
            [b[0], b[1], b[0], b[1], b[0], b[1], b[0], b[1]]
        });
        while i + 4 <= n {
            let w = u64::from_ne_bytes(data[2 * i..2 * i + 8].try_into().unwrap());
            if (w ^ splat) & mmask != 0 {
                break;
            }
            i += 4;
        }
        let end = (i + 4).min(n);
        while i < end {
            let v = block.sample(i) & mask;
            if v != *prev {
                out.push(Transition { at: block.start + i as u64, prev: *prev, now: v });
                *prev = v;
            }
            i += 1;
        }
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
        let mut s = vec![0x8000u16; 21];
        s[13] = 0x8001;
        let data: Vec<u8> = s.iter().flat_map(|v| v.to_le_bytes()).collect();
        d.process(&Block::new(100, 2, data), &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0], Transition { at: 113, prev: 0x8000, now: 0x8001 });
        assert_eq!(out[1].at, 114);
    }

    #[test]
    fn mask_ignores_channels() {
        let mut d = EdgeDetector::new(0x01);
        let mut out = Vec::new();
        d.process(&Block::new(0, 1, vec![0, 2, 2, 3, 2]), &mut out);
        assert_eq!(out, vec![Transition { at: 3, prev: 0, now: 1 }, Transition { at: 4, prev: 1, now: 0 }]);
    }
}
