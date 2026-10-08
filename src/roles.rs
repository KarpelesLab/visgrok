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
    /// SD card bus clock.
    SdClk,
    /// SD card command line.
    SdCmd,
    /// SD card data line DAT0..DAT3.
    SdDat(u8),
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
            Role::SdClk => write!(f, "SD CLK"),
            Role::SdCmd => write!(f, "SD CMD"),
            Role::SdDat(n) => write!(f, "SD DAT{n}"),
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
    300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 74880, 115200, 230400, 250000, 460800, 500000, 921600, 1000000,
    1500000, 2000000, 3000000, 4000000,
];

/// Cross-channel event counters used to detect buses.
#[derive(Clone, Debug)]
pub struct Correlator {
    n: usize,
    state: u32,
    /// `hi[i][j]`: changes of j while i is high and steady.
    pub(crate) hi: Vec<u64>,
    /// `lo[i][j]`: changes of j while i is low and steady.
    pub(crate) lo: Vec<u64>,
    /// `start[i][j]`: j falls while i is high and steady (I2C START-like).
    pub(crate) start: Vec<u64>,
    /// `stop[i][j]`: j rises while i is high and steady (I2C STOP-like).
    pub(crate) stop: Vec<u64>,
    /// `burst[i][j]`: changes of j shortly after an edge of i (while i is
    /// actively toggling), as opposed to during i's quiet gaps.
    pub(crate) burst: Vec<u64>,
    /// `near[i][j]`: changes of j within ~64 of i's shortest edge intervals
    /// of an edge of i (just before/after a burst of i, as CS and D/C do).
    pub(crate) near: Vec<u64>,
    /// `high_at_rise[i][j]`: rising edges of i at which j is high.
    pub(crate) high_at_rise: Vec<u64>,
    /// Rising edges of each channel.
    pub(crate) rises: Vec<u64>,
    /// Last edge of each channel and its shortest interval between edges.
    last_edge: Vec<Option<u64>>,
    min_gap: Vec<u64>,
    /// Edge intervals up to this many samples are glitches, not periods.
    glitch: u64,
}

impl Correlator {
    /// Creates counters for `n` channels.
    pub fn new(n: usize) -> Correlator {
        Correlator::with_glitch(n, 0)
    }

    /// Like [`Correlator::new`], ignoring edge intervals of up to `glitch`
    /// samples when measuring each channel's shortest period.
    pub fn with_glitch(n: usize, glitch: u64) -> Correlator {
        let z = vec![0; n * n];
        Correlator {
            n,
            state: 0,
            burst: z.clone(),
            near: z.clone(),
            high_at_rise: z.clone(),
            rises: vec![0; n],
            last_edge: vec![None; n],
            min_gap: vec![u64::MAX; n],
            glitch,
            hi: z.clone(),
            lo: z.clone(),
            start: z.clone(),
            stop: z,
        }
    }

    /// Sets the initial line state.
    pub fn init(&mut self, state: u32) {
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
                    // Burst membership: j moved within a few of i's
                    // shortest edge-to-edge intervals after i's last edge.
                    if i != j
                        && let Some(le) = self.last_edge[i]
                        && self.min_gap[i] != u64::MAX
                    {
                        let dt = t.at - le;
                        if dt <= 4 * self.min_gap[i] {
                            self.burst[i * n + j] += 1;
                        }
                        if dt <= 64 * self.min_gap[i] {
                            self.near[i * n + j] += 1;
                        }
                    }
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
            // Per-channel edge bookkeeping, after the pair counts above.
            let mut c = changed;
            while c != 0 {
                let i = c.trailing_zeros() as usize;
                c &= c - 1;
                if i >= n {
                    continue;
                }
                if let Some(le) = self.last_edge[i]
                    && t.at - le > self.glitch
                {
                    self.min_gap[i] = self.min_gap[i].min(t.at - le);
                }
                self.last_edge[i] = Some(t.at);
                if t.now >> i & 1 != 0 {
                    self.rises[i] += 1;
                    for j in 0..n {
                        if j != i && t.now >> j & 1 != 0 {
                            self.high_at_rise[i * n + j] += 1;
                        }
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
    // Pulses under 25 ns are glitches (as in the UART decoder), not bits. A
    // glitch splits a pulse in two: merge the pieces back into one.
    let glitch = (samplerate as f64 * 25e-9) as u64;
    let mut w: Vec<u64> = Vec::new();
    let mut merge_next = false;
    for x in c.recent_widths_ordered() {
        if x <= glitch {
            if let Some(last) = w.last_mut() {
                *last += x;
                merge_next = true;
            }
        } else if merge_next {
            *w.last_mut().unwrap() += x;
            merge_next = false;
        } else {
            w.push(x);
        }
    }
    let w = &w[..];
    if w.len() < 20 {
        return None;
    }
    // Bit time from high+low pulse pairs (robust to slow edges, see
    // estimate_bit_time); traffic must fit it, and bits must be resolvable.
    let bit = crate::decode::uart::estimate_bit_time(w, 10.0)?;
    if bit < 4.0 {
        return None;
    }
    // A UART idles high. Judge it on the recent pulses, leaving out the two
    // longest (a powered-off stretch, the final idle): overall duty is
    // misleading when the line sits low while the device is off. Pulses
    // alternate, and the last one had the opposite of the current level.
    let n = w.len();
    let high = |k: usize| (n - 1 - k).is_multiple_of(2) != c.level;
    let mut longest: Vec<usize> = (0..n).collect();
    longest.sort_unstable_by_key(|&k| std::cmp::Reverse(w[k]));
    let skip = &longest[..2.min(n)];
    let (mut hi, mut all) = (0u64, 0u64);
    for k in (0..n).filter(|k| !skip.contains(k)) {
        all += w[k];
        if high(k) {
            hi += w[k];
        }
    }
    // Busy traffic sits near 50%; an idle-low line is far below.
    let idles_high = all > 0 && hi * 5 >= all * 2;
    if !idles_high {
        return None;
    }
    // Serial data has varied run lengths; a square wave (every pulse pair
    // the same length) fits any bit time but is a clock, not a UART.
    let pairs: Vec<u64> = w.windows(2).map(|p| p[0] + p[1]).filter(|&p| (p as f64) < 21.0 * bit).collect();
    let two = pairs.iter().filter(|&&p| ((p as f64 / bit) - 2.0).abs() < 0.3).count();
    if pairs.is_empty() || two * 10 >= pairs.len() * 9 {
        return None;
    }
    let measured = samplerate as f64 / bit;
    let baud = snap_baud(measured).unwrap_or_else(|| crate::decode::uart::nice_baud(measured));
    Some(Suggestion {
        role: Role::Uart { baud },
        confidence: 0.8,
    })
}

/// Proposes a role for each channel.
pub fn detect(stats: &Stats, corr: &Correlator, samplerate: u64) -> Vec<Suggestion> {
    let n = stats.channels.len();
    let mut out: Vec<Suggestion> = vec![
        Suggestion {
            role: Role::Unknown,
            confidence: 0.0
        };
        n
    ];
    let active: Vec<bool> = stats.channels.iter().map(|c| c.edges() >= 4).collect();

    // Free-running clocks.
    for (i, c) in stats.channels.iter().enumerate() {
        if !active[i] {
            out[i] = Suggestion {
                role: Role::Idle,
                confidence: if c.edges() == 0 { 0.9 } else { 0.5 },
            };
            continue;
        }
        if let (Some(p), Some(j), Some(d)) = (c.median_period(), c.period_jitter(), c.duty()) {
            // Duty while running (pulse widths against the period), so a
            // clock that only runs part of the time still qualifies; one that
            // stopped is still a clock, with less confidence.
            let _ = d;
            let mut w = c.recent_widths().to_vec();
            w.sort_unstable();
            let typical = w.get(w.len() / 2).copied().unwrap_or(0) as f64 / p as f64;
            let recent = c.last_edge.is_some_and(|e| stats.samples.saturating_sub(e) < 4 * p);
            if j < 0.05 && (0.2..0.8).contains(&typical) {
                let hz = samplerate as f64 / p as f64;
                out[i] = Suggestion {
                    role: Role::Clock { hz },
                    confidence: (0.95 - j * 4.0) * if recent { 1.0 } else { 0.8 },
                };
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
        out[scl] = Suggestion {
            role: Role::I2cScl { sda: sda as u8 },
            confidence: ratio * 0.9,
        };
        out[sda] = Suggestion {
            role: Role::I2cSda { scl: scl as u8 },
            confidence: ratio * 0.9,
        };
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
            // Changes tied to the clock's activity: during bursts (data) or
            // right around them (CS, D/C). A line that changes far from the
            // clock (e.g. a UART) is not part of this bus.
            let near = corr.get(&corr.near, clk, d) as f64 / stats.channels[d].edges().max(1) as f64;
            if tot >= 16 && (lo.max(hi) as f64 / tot as f64) > 0.97 && near > 0.8 {
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
        out[clk] = Suggestion {
            role: Role::SpiClk,
            confidence: 0.6,
        };
        // Data lines change during clock bursts; CS and D/C only in the gaps
        // between them. CS then has one level at every clock edge (its active
        // level); D/C differs between command and data bytes.
        let rises = corr.rises[clk].max(1) as f64;
        let mut mosi_done = false;
        for d in data {
            used[d] = true;
            let changes = stats.channels[d].edges().max(1) as f64;
            let in_burst = corr.get(&corr.burst, clk, d) as f64 / changes;
            let high = corr.get(&corr.high_at_rise, clk, d) as f64 / rises;
            let role = if in_burst > 0.5 {
                if mosi_done {
                    Role::SpiMiso
                } else {
                    mosi_done = true;
                    Role::SpiMosi
                }
            } else if !(0.02..=0.98).contains(&high) {
                Role::SpiCs
            } else {
                Role::SpiDc
            };
            out[d] = Suggestion { role, confidence: 0.55 };
        }
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

    for (i, s) in out.iter_mut().enumerate() {
        if s.role == Role::Unknown && active[i] {
            *s = Suggestion {
                role: Role::Data,
                confidence: 0.3,
            };
        }
    }
    out
}

impl Role {
    /// Canonical text form, accepted back by [`Role::parse`] (empty for
    /// [`Role::Unknown`]).
    pub fn id(&self) -> String {
        match self {
            Role::Unknown => String::new(),
            Role::Uart { baud: 0 } => "uart".into(),
            Role::Uart { baud } => format!("uart:{baud}"),
            Role::I2cScl { sda } => format!("i2c-scl:{sda}"),
            Role::I2cSda { scl } => format!("i2c-sda:{scl}"),
            Role::SpiClk => "spi-clk".into(),
            Role::SpiMosi | Role::SpiData { .. } => "spi-mosi".into(),
            Role::SpiMiso => "spi-miso".into(),
            Role::SpiCs => "spi-cs".into(),
            Role::SpiDc => "spi-dc".into(),
            Role::SdClk => "sd-clk".into(),
            Role::SdCmd => "sd-cmd".into(),
            Role::SdDat(n) => format!("sd-dat{n}"),
            Role::Idle | Role::Clock { .. } | Role::Data => "idle".into(),
        }
    }

    /// Parses a role name as used on the command line: `uart`, `uart:115200`,
    /// `spi-clk`, `spi-mosi`, `spi-miso`, `spi-cs`, `spi-dc`, `i2c-scl:SDA`,
    /// `i2c-sda:SCL`, `sd-clk`, `sd-cmd`, `sd-dat0`..`sd-dat3`, `idle`.
    pub fn parse(s: &str) -> Result<Role, String> {
        let (name, arg) = match s.split_once(':') {
            Some((n, a)) => (n, Some(a)),
            None => (s, None),
        };
        let num = |a: Option<&str>| -> Result<u32, String> {
            a.ok_or_else(|| format!("{name} needs an argument"))?
                .parse()
                .map_err(|_| format!("bad number in {s:?}"))
        };
        Ok(match name.to_ascii_lowercase().as_str() {
            "uart" | "serial" => Role::Uart {
                baud: if arg.is_some() { num(arg)? } else { 0 },
            },
            "spi-clk" | "spi-sclk" | "sclk" | "sck" => Role::SpiClk,
            "spi-mosi" | "spi-di" | "mosi" | "sdi" => Role::SpiMosi,
            "spi-miso" | "spi-do" | "miso" | "sdo" => Role::SpiMiso,
            "spi-cs" | "cs" | "ss" => Role::SpiCs,
            "spi-dc" | "spi-cd" | "dc" | "cd" => Role::SpiDc,
            "i2c-scl" | "scl" => Role::I2cScl { sda: num(arg)? as u8 },
            "i2c-sda" | "sda" => Role::I2cSda { scl: num(arg)? as u8 },
            "sd-clk" | "sdclk" => Role::SdClk,
            "sd-cmd" | "sdcmd" => Role::SdCmd,
            "sd-dat0" | "sd-d0" => Role::SdDat(0),
            "sd-dat1" | "sd-d1" => Role::SdDat(1),
            "sd-dat2" | "sd-d2" => Role::SdDat(2),
            "sd-dat3" | "sd-d3" => Role::SdDat(3),
            "idle" | "none" => Role::Idle,
            _ => return Err(format!("unknown role {name:?}")),
        })
    }
}
