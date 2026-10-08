//! Per-channel signal statistics, computed incrementally from transitions.

use crate::edges::Transition;

/// Number of log2 buckets in the pulse-width histogram (covers 1 .. 2^40 samples).
pub const HIST_BUCKETS: usize = 40;

/// Statistics for one channel.
#[derive(Clone, Debug)]
pub struct ChannelStats {
    /// Current level.
    pub level: bool,
    /// Number of rising edges seen.
    pub rising: u64,
    /// Number of falling edges seen.
    pub falling: u64,
    /// Samples spent high (completed pulses only, plus the current level up to
    /// the last [`Stats::advance`]).
    pub high_samples: u64,
    /// Samples spent low.
    pub low_samples: u64,
    /// Sample index of the last edge.
    pub last_edge: Option<u64>,
    /// Shortest high pulse, in samples.
    pub min_high: u64,
    /// Shortest low pulse, in samples.
    pub min_low: u64,
    /// Histogram of high pulse widths, bucket `k` counts widths in `[2^k, 2^(k+1))`.
    pub high_hist: [u64; HIST_BUCKETS],
    /// Histogram of low pulse widths.
    pub low_hist: [u64; HIST_BUCKETS],
    /// Recent rising-edge to rising-edge periods (ring buffer).
    periods: Vec<u64>,
    period_pos: usize,
    widths: Vec<u64>,
    width_pos: usize,
    last_rise: Option<u64>,
    accounted: u64,
}

const PERIOD_RING: usize = 64;
const WIDTH_RING: usize = 256;

impl Default for ChannelStats {
    fn default() -> Self {
        ChannelStats {
            level: false,
            rising: 0,
            falling: 0,
            high_samples: 0,
            low_samples: 0,
            last_edge: None,
            min_high: u64::MAX,
            min_low: u64::MAX,
            high_hist: [0; HIST_BUCKETS],
            low_hist: [0; HIST_BUCKETS],
            periods: Vec::with_capacity(PERIOD_RING),
            period_pos: 0,
            widths: Vec::with_capacity(WIDTH_RING),
            width_pos: 0,
            last_rise: None,
            accounted: 0,
        }
    }
}

impl ChannelStats {
    /// Total number of edges.
    pub fn edges(&self) -> u64 {
        self.rising + self.falling
    }

    /// Fraction of time high, if any time has been observed.
    pub fn duty(&self) -> Option<f64> {
        let t = self.high_samples + self.low_samples;
        (t > 0).then(|| self.high_samples as f64 / t as f64)
    }

    /// Median of recent rising-to-rising periods, in samples.
    pub fn median_period(&self) -> Option<u64> {
        if self.periods.len() < 4 {
            return None;
        }
        let mut v = self.periods.clone();
        v.sort_unstable();
        Some(v[v.len() / 2])
    }

    /// Coefficient of variation of recent periods (0 = perfectly regular).
    pub fn period_jitter(&self) -> Option<f64> {
        if self.periods.len() < 4 {
            return None;
        }
        let n = self.periods.len() as f64;
        let mean = self.periods.iter().sum::<u64>() as f64 / n;
        let var = self.periods.iter().map(|&p| (p as f64 - mean).powi(2)).sum::<f64>() / n;
        Some(var.sqrt() / mean)
    }

    /// Recent pulse widths (high and low), in samples, in no particular order.
    pub fn recent_widths(&self) -> &[u64] {
        &self.widths
    }

    fn edge(&mut self, at: u64, rising: bool) {
        let since = at - self.accounted;
        if self.level {
            self.high_samples += since;
        } else {
            self.low_samples += since;
        }
        self.accounted = at;
        if let Some(prev) = self.last_edge {
            let width = at - prev;
            if self.widths.len() < WIDTH_RING {
                self.widths.push(width);
            } else {
                self.widths[self.width_pos] = width;
                self.width_pos = (self.width_pos + 1) % WIDTH_RING;
            }
            let bucket = (63 - width.max(1).leading_zeros() as usize).min(HIST_BUCKETS - 1);
            if self.level {
                self.min_high = self.min_high.min(width);
                self.high_hist[bucket] += 1;
            } else {
                self.min_low = self.min_low.min(width);
                self.low_hist[bucket] += 1;
            }
        }
        if rising {
            self.rising += 1;
            if let Some(r) = self.last_rise {
                let p = at - r;
                if self.periods.len() < PERIOD_RING {
                    self.periods.push(p);
                } else {
                    self.periods[self.period_pos] = p;
                    self.period_pos = (self.period_pos + 1) % PERIOD_RING;
                }
            }
            self.last_rise = Some(at);
        } else {
            self.falling += 1;
        }
        self.level = rising;
        self.last_edge = Some(at);
    }

    fn advance(&mut self, to: u64) {
        if to > self.accounted {
            if self.level {
                self.high_samples += to - self.accounted;
            } else {
                self.low_samples += to - self.accounted;
            }
            self.accounted = to;
        }
    }
}

/// Statistics for all channels.
#[derive(Clone, Debug)]
pub struct Stats {
    /// Per-channel statistics, indexed by channel number.
    pub channels: Vec<ChannelStats>,
    /// Samples processed so far.
    pub samples: u64,
    initialized: bool,
}

impl Stats {
    /// Creates statistics for `n` channels.
    pub fn new(n: usize) -> Stats {
        Stats { channels: vec![ChannelStats::default(); n], samples: 0, initialized: false }
    }

    /// Sets the initial state of all channels (the first sample of the capture).
    pub fn init(&mut self, state: u32) {
        for (i, c) in self.channels.iter_mut().enumerate() {
            c.level = state >> i & 1 != 0;
        }
        self.initialized = true;
    }

    /// True once [`Stats::init`] has been called.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Accounts for a batch of transitions.
    pub fn process(&mut self, transitions: &[Transition]) {
        for t in transitions {
            let mut changed = t.changed();
            while changed != 0 {
                let ch = changed.trailing_zeros() as usize;
                changed &= changed - 1;
                if let Some(c) = self.channels.get_mut(ch) {
                    c.edge(t.at, t.now >> ch & 1 != 0);
                }
            }
        }
    }

    /// Marks that all samples up to (excluding) `end` have been processed.
    pub fn advance(&mut self, end: u64) {
        self.samples = self.samples.max(end);
        for c in &mut self.channels {
            c.advance(end);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_wave() {
        let mut s = Stats::new(1);
        s.init(0);
        let mut tr = Vec::new();
        let mut lvl = 0u32;
        for i in 1..=100u64 {
            let now = lvl ^ 1;
            tr.push(Transition { at: i * 10, prev: lvl, now });
            lvl = now;
        }
        s.process(&tr);
        s.advance(1010);
        let c = &s.channels[0];
        assert_eq!(c.rising, 50);
        assert_eq!(c.falling, 50);
        assert_eq!(c.median_period(), Some(20));
        assert!(c.period_jitter().unwrap() < 1e-9);
        assert!((c.duty().unwrap() - 0.5).abs() < 0.02);
        assert_eq!(c.min_high, 10);
    }
}
