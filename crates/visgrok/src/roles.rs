//! Channel roles: manual assignment and automatic detection.
//!
//! Detection is heuristic. It looks at per-channel timing statistics
//! ([`Stats`]) and at cross-channel relationships ([`Correlator`]) and proposes
//! a role with a confidence. The user can accept or override any suggestion.

use std::fmt;

use crate::edges::Transition;
use crate::stats::{ChannelStats, Stats};

/// What a channel carries.
#[derive(Clone, Debug, PartialEq)]
pub enum Role {
    /// Not analyzed.
    Unknown,
    /// No activity.
    Idle,
    /// Free-running clock at the given frequency (Hz).
    Clock {
        /// Frequency in Hz.
        hz: f64,
    },
    /// UART line. The decoder follows rate changes; `baud` is the starting
    /// point (0: detect from the traffic).
    Uart {
        /// Detected or configured baud rate, 0 for automatic.
        baud: u32,
    },
    /// I2C clock; `sda` is the paired data channel.
    I2cScl {
        /// Paired SDA channel.
        sda: u8,
    },
    /// I2C data; `scl` is the paired clock channel.
    I2cSda {
        /// Paired SCL channel.
        scl: u8,
    },
    /// SPI clock (bursty, gated).
    SpiClk,
    /// SPI chip select.
    SpiCs,
    /// SPI data line sampled by `clk` (direction unknown; auto-detected).
    SpiData {
        /// Clock channel.
        clk: u8,
    },
    /// SPI controller-out data (MOSI / DI).
    SpiMosi,
    /// SPI controller-in data (MISO / DO).
    SpiMiso,
    /// SPI data/command select (D/C, high = data), e.g. SSD1306 displays.
    SpiDc,
    /// Generic activity that matched nothing else.
    Data,
}

impl fmt::Display for Role {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Role::Unknown => write!(f, "?"),
            Role::Idle => write!(f, "idle"),
            Role::Clock { hz } => write!(f, "clock {}", fmt_hz(*hz)),
            Role::Uart { baud: 0 } => write!(f, "UART auto"),
            Role::Uart { baud } => write!(f, "UART {baud}"),
            Role::I2cScl { sda } => write!(f, "I2C SCL (sda=ch{sda})"),
            Role::I2cSda { scl } => write!(f, "I2C SDA (scl=ch{scl})"),
            Role::SpiClk => write!(f, "SPI CLK"),
            Role::SpiCs => write!(f, "SPI CS"),
            Role::SpiData { clk } => write!(f, "SPI data (clk=ch{clk})"),
            Role::SpiMosi => write!(f, "SPI MOSI"),
            Role::SpiMiso => write!(f, "SPI MISO"),
            Role::SpiDc => write!(f, "SPI D/C"),
            Role::Data => write!(f, "data"),
        }
    }
}

/// Formats a frequency with an SI prefix.
pub fn fmt_hz(hz: f64) -> String {
    if hz >= 1e6 {
        format!("{:.3} MHz", hz / 1e6)
    } else if hz >= 1e3 {
        format!("{:.3} kHz", hz / 1e3)
    } else {
        format!("{hz:.1} Hz")
    }
}

/// A detected role with a confidence in `0.0..=1.0`.
#[derive(Clone, Debug, PartialEq)]
pub struct Suggestion {
    /// Proposed role.
    pub role: Role,
    /// Confidence.
    pub confidence: f64,
}

/// Standard UART baud rates used to snap detected rates.
pub const BAUD_RATES: &[u32] = &[
    300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 74880, 115200, 230400, 250000, 460800,
    500000, 921600, 1000000, 1500000, 2000000, 3000000, 4000000,
];

/// Cross-channel event counters used to detect buses.
#[derive(Clone, Debug)]
pub struct Correlator {
    n: usize,
    state: u16,
    /// `hi[i][j]`: changes of j while i is high and steady.
    pub(crate) hi: Vec<u64>,
    /// `lo[i][j]`: changes of j while i is low and steady.
    pub(crate) lo: Vec<u64>,
    /// `start[i][j]`: j falls while i is high and steady (I2C START-like).
    pub(crate) start: Vec<u64>,
    /// `stop[i][j]`: j rises while i is high and steady (I2C STOP-like).
    pub(crate) stop: Vec<u64>,
}

impl Correlator {
    /// Creates counters for `n` channels.
    pub fn new(n: usize) -> Correlator {
        let z = vec![0; n * n];
        Correlator { n, state: 0, hi: z.clone(), lo: z.clone(), start: z.clone(), stop: z }
    }

    /// Sets the initial line state.
    pub fn init(&mut self, state: u16) {
        self.state = state;
    }

    /// Accounts for transitions.
    pub fn process(&mut self, transitions: &[Transition]) {
        let n = self.n;
        for t in transitions {
            let changed = t.changed();
            let steady = !changed;
            let mut c = changed;
            while c != 0 {
                let j = c.trailing_zeros() as usize;
                c &= c - 1;
                if j >= n {
                    continue;
                }
                let rising = t.now >> j & 1 != 0;
                for i in 0..n {
                    if i == j || steady >> i & 1 == 0 {
                        continue;
                    }
                    if t.prev >> i & 1 != 0 {
                        self.hi[i * n + j] += 1;
                        if rising {
                            self.stop[i * n + j] += 1;
                        } else {
                            self.start[i * n + j] += 1;
                        }
                    } else {
                        self.lo[i * n + j] += 1;
                    }
                }
            }
            self.state = t.now;
        }
    }

    pub(crate) fn get(&self, v: &[u64], i: usize, j: usize) -> u64 {
        v[i * self.n + j]
    }
}

/// Snaps a measured baud rate to the nearest standard rate within 4%.
pub fn snap_baud(measured: f64) -> Option<u32> {
    BAUD_RATES
        .iter()
        .copied()
        .min_by(|a, b| ((*a as f64 - measured).abs()).total_cmp(&(*b as f64 - measured).abs()))
        .filter(|&b| ((b as f64 - measured) / b as f64).abs() < 0.04)
}

fn uart_score(c: &ChannelStats, samplerate: u64) -> Option<Suggestion> {
    let w = c.recent_widths();
    if w.len() < 20 || c.duty()? < 0.5 || !c.level && c.edges() < 40 {
        return None;
    }
    // The shortest pulse is (usually) one bit; reject outliers by taking the
    // 5th percentile.
    let mut sorted = w.to_vec();
    sorted.sort_unstable();
    let unit = sorted[sorted.len() / 20].max(1) as f64;
    // Pulses within a frame are integer multiples of the bit time; gaps
    // between frames can be anything, so only score pulses up to 10 bits.
    let mut good = 0;
    let mut total = 0;
    for &x in w {
        let r = x as f64 / unit;
        if r <= 10.5 {
            total += 1;
            if (r - r.round()).abs() < 0.2 {
                good += 1;
            }
        }
    }
    if total < 16 {
        return None;
    }
    let fit = good as f64 / total as f64;
    // Refine the bit time using all pulses that are clean multiples.
    let (sum, bits) = w.iter().fold((0.0, 0.0), |(s, b), &x| {
        let r = (x as f64 / unit).round();
        if (1.0..=10.0).contains(&r) && (x as f64 / unit - r).abs() < 0.2 { (s + x as f64, b + r) } else { (s, b) }
    });
    let measured = samplerate as f64 * bits / sum;
    let baud = snap_baud(measured).unwrap_or_else(|| crate::decode::uart::nice_baud(measured));
    (fit > 0.85).then_some(Suggestion { role: Role::Uart { baud }, confidence: fit * 0.9 })
}

/// Proposes a role for each channel.
pub fn detect(stats: &Stats, corr: &Correlator, samplerate: u64) -> Vec<Suggestion> {
    let n = stats.channels.len();
    let mut out: Vec<Suggestion> = vec![Suggestion { role: Role::Unknown, confidence: 0.0 }; n];
    let active: Vec<bool> = stats.channels.iter().map(|c| c.edges() >= 4).collect();

    // Free-running clocks.
    for (i, c) in stats.channels.iter().enumerate() {
        if !active[i] {
            out[i] = Suggestion { role: Role::Idle, confidence: if c.edges() == 0 { 0.9 } else { 0.5 } };
            continue;
        }
        if let (Some(p), Some(j), Some(d)) = (c.median_period(), c.period_jitter(), c.duty()) {
            let recent = c.last_edge.is_some_and(|e| stats.samples.saturating_sub(e) < 4 * p);
            if j < 0.05 && (0.2..0.8).contains(&d) && recent {
                let hz = samplerate as f64 / p as f64;
                out[i] = Suggestion { role: Role::Clock { hz }, confidence: 0.95 - j * 4.0 };
            }
        }
    }

    // I2C pairs: SDA changes mostly while SCL is low, except START/STOP which
    // happen while SCL is high; both lines idle high; SCL never changes while
    // SDA is the only line moving... we look for pairs with a healthy number
    // of start and stop conditions.
    let mut best_i2c: Vec<(f64, usize, usize)> = Vec::new();
    for scl in 0..n {
        for sda in 0..n {
            if scl == sda || !active[scl] || !active[sda] {
                continue;
            }
            let lo = corr.get(&corr.lo, scl, sda);
            let hi = corr.get(&corr.hi, scl, sda);
            let starts = corr.get(&corr.start, scl, sda);
            let stops = corr.get(&corr.stop, scl, sda);
            if starts < 2 || stops < 2 || lo < 8 {
                continue;
            }
            // SDA changes while SCL is high are START/STOP conditions only:
            // a couple per transfer, against several data changes (while SCL
            // is low) per byte. Unrelated signals show no such asymmetry.
            let hi_frac = hi as f64 / (lo + hi) as f64;
            let balanced = starts.abs_diff(stops) <= starts.max(stops) / 4 + 1;
            let ratio = 1.0 - hi_frac;
            let duty_ok = stats.channels[scl].duty().unwrap_or(0.0) > 0.3;
            if hi_frac < 0.35 && balanced && duty_ok {
                best_i2c.push((ratio, scl, sda));
            }
        }
    }
    best_i2c.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut used = vec![false; n];
    for (ratio, scl, sda) in best_i2c {
        if used[scl] || used[sda] {
            continue;
        }
        used[scl] = true;
        used[sda] = true;
        out[scl] = Suggestion { role: Role::I2cScl { sda: sda as u8 }, confidence: ratio * 0.9 };
        out[sda] = Suggestion { role: Role::I2cSda { scl: scl as u8 }, confidence: ratio * 0.9 };
    }

    // UART lines.
    for (i, c) in stats.channels.iter().enumerate() {
        if used[i] || !active[i] || matches!(out[i].role, Role::Clock { .. }) {
            continue;
        }
        if let Some(s) = uart_score(c, samplerate) {
            used[i] = true;
            out[i] = s;
        }
    }

    // SPI: a bursty clock (not free running) on which other lines change
    // almost exclusively while it is in one state, plus an optional CS that
    // changes only while the clock is idle.
    for clk in 0..n {
        if used[clk] || !active[clk] || matches!(out[clk].role, Role::Clock { .. }) {
            continue;
        }
        let mut data = Vec::new();
        for d in 0..n {
            if d == clk || used[d] || !active[d] {
                continue;
            }
            let lo = corr.get(&corr.lo, clk, d);
            let hi = corr.get(&corr.hi, clk, d);
            let tot = lo + hi;
            if tot >= 16 && (lo.max(hi) as f64 / tot as f64) > 0.97 {
                data.push(d);
            }
        }
        // The clock must toggle much more often than the data lines.
        let ce = stats.channels[clk].edges();
        data.retain(|&d| stats.channels[d].edges() * 2 <= ce + 2);
        if data.is_empty() {
            continue;
        }
        used[clk] = true;
        out[clk] = Suggestion { role: Role::SpiClk, confidence: 0.6 };
        for d in data {
            used[d] = true;
            // CS toggles rarely compared to data.
            let is_cs = stats.channels[d].edges() * 16 < ce && stats.channels[d].duty().unwrap_or(0.0) > 0.5;
            out[d] = Suggestion {
                role: if is_cs { Role::SpiCs } else { Role::SpiData { clk: clk as u8 } },
                confidence: 0.5,
            };
        }
    }

    for (i, s) in out.iter_mut().enumerate() {
        if s.role == Role::Unknown && active[i] {
            *s = Suggestion { role: Role::Data, confidence: 0.3 };
        }
    }
    out
}

impl Role {
    /// Parses a role name as used on the command line: `uart`, `uart:115200`,
    /// `spi-clk`, `spi-mosi`, `spi-miso`, `spi-cs`, `spi-dc`, `i2c-scl:SDA`,
    /// `i2c-sda:SCL`, `idle`.
    pub fn parse(s: &str) -> Result<Role, String> {
        let (name, arg) = match s.split_once(':') {
            Some((n, a)) => (n, Some(a)),
            None => (s, None),
        };
        let num = |a: Option<&str>| -> Result<u32, String> {
            a.ok_or_else(|| format!("{name} needs an argument"))?.parse().map_err(|_| format!("bad number in {s:?}"))
        };
        Ok(match name.to_ascii_lowercase().as_str() {
            "uart" | "serial" => Role::Uart { baud: if arg.is_some() { num(arg)? } else { 0 } },
            "spi-clk" | "spi-sclk" | "sclk" | "sck" => Role::SpiClk,
            "spi-mosi" | "spi-di" | "mosi" | "sdi" => Role::SpiMosi,
            "spi-miso" | "spi-do" | "miso" | "sdo" => Role::SpiMiso,
            "spi-cs" | "cs" | "ss" => Role::SpiCs,
            "spi-dc" | "spi-cd" | "dc" | "cd" => Role::SpiDc,
            "i2c-scl" | "scl" => Role::I2cScl { sda: num(arg)? as u8 },
            "i2c-sda" | "sda" => Role::I2cSda { scl: num(arg)? as u8 },
            "idle" | "none" => Role::Idle,
            _ => return Err(format!("unknown role {name:?}")),
        })
    }
}
