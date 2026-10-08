//! Streaming acquisition: a ring of bulk IN transfers resubmitted straight
//! from their completion callbacks, with all processing on the consumer side.

use std::collections::VecDeque;
use std::io;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use rawusb::{Transfer, TransferStatus};

use super::{Config, Error, Model, Pattern, Result, SLogic};
use crate::block::Block;
use crate::source::{CaptureInfo, Source};

/// Transfers kept in flight.
const RING: usize = 16;
/// Transfer size granularity (a multiple of the 1024-byte SuperSpeed packet).
const ALIGN: usize = 32 << 10;
const MIN_XFER: usize = 32 << 10;
const MAX_XFER: usize = 16 << 20;
/// Memory budget for the whole ring.
const MAX_RING_BYTES: usize = 256 << 20;
/// Target duration of data per transfer. Long transfers matter: the device
/// has no flow control and only ~28 KiB of FIFO, and on macOS data is lost
/// right after transfer boundaries at 400 MB/s; fewer boundaries, fewer
/// losses (16 MiB transfers: ~5 small gaps per GB, 1 MiB: ~1600).
const XFER_DURATION: f64 = 0.040;
/// Bytes at the start of each acquisition that are not sample data.
const HEAD_ARTIFACT: usize = 4;
/// Give up when this much data is waiting for the consumer.
const MAX_QUEUED: usize = 1 << 30;
/// Silence after which a running capture is considered dead.
const IDLE_LIMIT: Duration = Duration::from_secs(2);
/// Minimum observation window for the start-up rate check.
const VERIFY_WINDOW: Duration = Duration::from_millis(100);
/// Accepted relative deviation of the measured byte rate.
const VERIFY_TOLERANCE: f64 = 0.25;
/// Restarts attempted when the device runs at the wrong rate.
const MAX_RESTARTS: u32 = 4;

/// Counters describing a running capture.
#[derive(Clone, Debug, Default)]
pub struct CaptureStats {
    /// Raw bytes received from the device.
    pub bytes: u64,
    /// Completed transfers.
    pub transfers: u64,
    /// Transfers that timed out (possibly with partial data).
    pub timeouts: u64,
    /// Restarts because the device ran at the wrong sample rate.
    pub restarts: u32,
    /// Byte rate measured during start-up verification.
    pub measured_rate: Option<f64>,
}

struct Completion {
    status: TransferStatus,
    data: Vec<u8>,
    at: Instant,
}

struct Shared {
    stopping: AtomicBool,
    in_flight: AtomicUsize,
    queued: AtomicUsize,
    error: Mutex<Option<String>>,
    /// Recycled transfer buffers. Reusing them keeps their pages resident, so
    /// the OS does not fault in and wire fresh memory on every submission,
    /// which delays resubmission enough to overflow the device FIFO at high
    /// rates.
    pool: Mutex<Vec<Vec<u8>>>,
}

impl Shared {
    fn buffer(&self, size: usize) -> Vec<u8> {
        match self.pool.lock().unwrap().pop() {
            Some(mut b) => {
                b.resize(size, 0);
                b
            }
            None => vec![0u8; size],
        }
    }

    fn recycle(&self, b: Vec<u8>) {
        let mut p = self.pool.lock().unwrap();
        if p.len() < RING {
            p.push(b);
        }
    }
}

struct Verify {
    first: Option<Instant>,
    last: Instant,
    /// Bytes received after the first completion.
    bytes: usize,
    held: Vec<Vec<u8>>,
}

/// A running capture; implements [`Source`].
pub struct Capture {
    dev: SLogic,
    cfg: Config,
    transfers: Vec<Transfer>,
    rx: Receiver<Completion>,
    shared: Arc<Shared>,
    xfer_size: usize,
    drop_left: usize,
    /// Partial sample left over from the previous transfer (16/32 channels).
    carry: Vec<u8>,
    /// Samples emitted so far.
    pos: u64,
    verify: Option<Verify>,
    ready: VecDeque<Block>,
    last_data: Instant,
    stats: CaptureStats,
    stopped: bool,
    done: bool,
}

impl Capture {
    pub(super) fn start(dev: SLogic, cfg: Config) -> Result<Capture> {
        dev.validate(&cfg)?;
        let rate = cfg.byte_rate();
        let size = ((rate * XFER_DURATION) as usize).div_ceil(ALIGN) * ALIGN;
        let xfer_size = match cfg.transfer_size {
            Some(v) => v.div_ceil(ALIGN).max(1) * ALIGN,
            None => size.clamp(MIN_XFER, MAX_XFER),
        };
        let ring = cfg.transfers.unwrap_or((MAX_RING_BYTES / xfer_size).clamp(4, RING)).max(2);
        let timeout = Duration::from_secs_f64((4.0 * xfer_size as f64 / rate).max(1.0));

        let (tx, rx) = channel();
        let shared = Arc::new(Shared {
            stopping: AtomicBool::new(false),
            in_flight: AtomicUsize::new(0),
            queued: AtomicUsize::new(0),
            error: Mutex::new(None),
            pool: Mutex::new(Vec::new()),
        });
        let ep = dev.model().endpoint();
        let mut transfers = Vec::with_capacity(ring);
        for _ in 0..ring {
            let t = Transfer::bulk(dev.handle(), ep, Vec::new());
            t.set_timeout(timeout)?;
            t.set_callback(callback(tx.clone(), shared.clone(), xfer_size))?;
            transfers.push(t);
        }
        let verify = cfg.verify_rate && dev.model() != Model::Combo8 && cfg.pattern != Pattern::UsbTest;
        let mut c = Capture {
            dev,
            cfg,
            transfers,
            rx,
            shared,
            xfer_size,
            drop_left: HEAD_ARTIFACT,
            carry: Vec::new(),
            pos: 0,
            verify: None,
            ready: VecDeque::new(),
            last_data: Instant::now(),
            stats: CaptureStats::default(),
            stopped: false,
            done: false,
        };
        c.arm(verify)?;
        Ok(c)
    }

    /// The configuration being captured.
    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Capture counters.
    pub fn stats(&self) -> &CaptureStats {
        &self.stats
    }

    /// Drains stale data, configures the device, submits the ring and runs.
    fn arm(&mut self, verify: bool) -> Result<()> {
        self.drain_endpoint();
        self.dev.configure(&self.cfg)?;
        self.drop_left = HEAD_ARTIFACT;
        self.carry.clear();
        self.shared.stopping.store(false, Ordering::SeqCst);
        for t in &self.transfers {
            t.set_buffer(self.shared.buffer(self.xfer_size))?;
            self.shared.in_flight.fetch_add(1, Ordering::SeqCst);
            if let Err(e) = t.submit() {
                self.shared.in_flight.fetch_sub(1, Ordering::SeqCst);
                self.halt_ring();
                return Err(e.into());
            }
        }
        self.verify = verify.then(|| Verify {
            first: None,
            last: Instant::now(),
            bytes: 0,
            held: Vec::new(),
        });
        self.last_data = Instant::now();
        if let Err(e) = self.dev.run(&self.cfg) {
            self.halt_ring();
            return Err(e);
        }
        Ok(())
    }

    /// Reads and discards whatever the endpoint still holds.
    fn drain_endpoint(&self) {
        let mut buf = vec![0u8; 64 << 10];
        for _ in 0..32 {
            match self
                .dev
                .handle()
                .bulk_read(self.dev.model().endpoint(), &mut buf, Duration::from_millis(50))
            {
                Ok(n) if n > 0 => continue,
                _ => break,
            }
        }
    }

    /// Stops the device and waits for every transfer to come back.
    fn halt_ring(&mut self) {
        self.shared.stopping.store(true, Ordering::SeqCst);
        let _ = self.dev.halt();
        for t in &self.transfers {
            let _ = t.cancel();
        }
        for t in &self.transfers {
            let _ = t.wait(Some(Duration::from_secs(2)));
        }
        // Let callbacks that already fired finish delivering.
        let deadline = Instant::now() + Duration::from_secs(2);
        while self.shared.in_flight.load(Ordering::SeqCst) > 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Restarts after a wrong-rate start.
    fn restart(&mut self) -> Result<()> {
        self.halt_ring();
        while let Ok(c) = self.rx.try_recv() {
            self.shared.queued.fetch_sub(c.data.len(), Ordering::SeqCst);
        }
        self.stats.restarts += 1;
        self.arm(true)
    }

    fn expected_rate(&self) -> f64 {
        self.cfg.byte_rate()
    }

    /// Converts raw wire bytes to a block (dropping the head artifact and
    /// unpacking 4-channel data).
    fn convert(&mut self, raw: Vec<u8>) {
        let skip = self.drop_left.min(raw.len());
        self.drop_left -= skip;
        let data = &raw[skip..];
        let (unit, bytes) = match self.cfg.channels {
            4 => {
                let mut out = Vec::with_capacity(data.len() * 2);
                for b in data {
                    out.push(b & 0x0f);
                    out.push(b >> 4);
                }
                (1, out)
            }
            16 | 32 => {
                // Keep a partial sample for the next transfer.
                let unit = self.cfg.channels / 8;
                let mut out = Vec::with_capacity(data.len() + self.carry.len());
                out.append(&mut self.carry);
                out.extend_from_slice(data);
                let whole = out.len() / unit * unit;
                self.carry = out.split_off(whole);
                (unit, out)
            }
            _ => (1, data.to_vec()),
        };
        self.shared.recycle(raw);
        let mut block = Block::new(self.pos, unit, bytes);
        if let Some(limit) = self.cfg.limit {
            let left = limit.saturating_sub(self.pos) as usize;
            if block.len() >= left {
                block.data.truncate(left * unit);
                self.stop_request();
            }
        }
        if block.is_empty() {
            return;
        }
        self.pos = block.end();
        self.ready.push_back(block);
    }

    fn stop_request(&mut self) {
        if !self.stopped {
            self.stopped = true;
            self.shared.stopping.store(true, Ordering::SeqCst);
            let _ = self.dev.halt();
            for t in &self.transfers {
                let _ = t.cancel();
            }
        }
    }

    /// Handles one completion. Returns an error for fatal conditions.
    fn handle(&mut self, c: Completion) -> Result<()> {
        self.shared.queued.fetch_sub(c.data.len(), Ordering::SeqCst);
        match c.status {
            TransferStatus::Completed => self.stats.transfers += 1,
            TransferStatus::TimedOut => self.stats.timeouts += 1,
            TransferStatus::Cancelled => {}
            s if self.stopped => {
                let _ = s;
            }
            s => return Err(Error::Protocol(format!("bulk transfer failed: {s:?}"))),
        }
        if c.data.is_empty() || self.stopped && self.cfg.limit.is_some_and(|l| self.pos >= l) {
            return Ok(());
        }
        self.stats.bytes += c.data.len() as u64;
        self.last_data = c.at;

        let expected = self.expected_rate();
        let Some(v) = &mut self.verify else {
            self.convert(c.data);
            return Ok(());
        };
        match v.first {
            None => v.first = Some(c.at),
            Some(_) => v.bytes += c.data.len(),
        }
        v.last = c.at;
        v.held.push(c.data);
        let first = v.first.unwrap();
        let elapsed = v.last.duration_since(first);
        if elapsed < VERIFY_WINDOW || v.held.len() < 4 {
            return Ok(());
        }
        let measured = v.bytes as f64 / elapsed.as_secs_f64();
        self.stats.measured_rate = Some(measured);
        if (measured / expected - 1.0).abs() > VERIFY_TOLERANCE && !self.stopped {
            if self.stats.restarts >= MAX_RESTARTS {
                return Err(Error::Protocol(format!(
                    "device streams at {:.1} MB/s instead of {:.1} MB/s after {} restarts",
                    measured / 1e6,
                    expected / 1e6,
                    self.stats.restarts
                )));
            }
            self.verify = None;
            self.pos = 0;
            self.ready.clear();
            return self.restart();
        }
        let held = std::mem::take(&mut v.held);
        self.verify = None;
        for d in held {
            self.convert(d);
        }
        Ok(())
    }
}

fn callback(tx: Sender<Completion>, shared: Arc<Shared>, size: usize) -> impl FnMut(&Transfer) + Send + 'static {
    move |t: &Transfer| {
        let status = t.status();
        let n = t.actual_length();
        let mut data = t.take_buffer().unwrap_or_default();
        data.truncate(n);
        let at = Instant::now();
        let mut resubmitted = false;
        if !shared.stopping.load(Ordering::SeqCst) && matches!(status, TransferStatus::Completed | TransferStatus::TimedOut) {
            resubmitted = t.set_buffer(shared.buffer(size)).is_ok() && t.submit().is_ok();
            if !resubmitted {
                let mut e = shared.error.lock().unwrap();
                e.get_or_insert_with(|| "failed to resubmit a bulk transfer".into());
            }
        }
        shared.queued.fetch_add(data.len(), Ordering::SeqCst);
        let _ = tx.send(Completion { status, data, at });
        if !resubmitted {
            shared.in_flight.fetch_sub(1, Ordering::SeqCst);
        }
    }
}

impl Source for Capture {
    fn info(&self) -> CaptureInfo {
        let device = match self.dev.serial() {
            Some(s) => format!("{} #{s}", self.dev.model().name()),
            None => self.dev.model().name().to_string(),
        };
        CaptureInfo {
            device,
            channels: self.cfg.channels,
            samplerate: self.cfg.samplerate,
            unit_size: crate::block::unit_size_for(self.cfg.channels),
            names: Vec::new(),
        }
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        loop {
            if let Some(b) = self.ready.pop_front() {
                return Ok(Some(b));
            }
            if self.done {
                return Ok(None);
            }
            let err = self.shared.error.lock().unwrap().take();
            if let Some(e) = err
                && !self.stopped
            {
                self.stop_request();
                return Err(io::Error::other(e));
            }
            if self.shared.queued.load(Ordering::SeqCst) > MAX_QUEUED {
                self.stop_request();
                return Err(Error::Overrun("consumer too slow; more than 1 GiB of samples queued".into()).into());
            }
            match self.rx.recv_timeout(Duration::from_millis(100)) {
                Ok(c) => {
                    if let Err(e) = self.handle(c) {
                        self.stop_request();
                        return Err(e.into());
                    }
                }
                Err(RecvTimeoutError::Timeout) => {
                    if self.stopped {
                        if self.shared.in_flight.load(Ordering::SeqCst) == 0 {
                            self.done = true;
                        }
                    } else if self.last_data.elapsed() > IDLE_LIMIT {
                        self.stop_request();
                        return Err(io::Error::new(io::ErrorKind::TimedOut, "device stopped streaming"));
                    }
                }
                Err(RecvTimeoutError::Disconnected) => self.done = true,
            }
            if self.stopped && self.shared.in_flight.load(Ordering::SeqCst) == 0 && self.ready.is_empty() {
                // Deliver anything still queued, then finish.
                while let Ok(c) = self.rx.try_recv() {
                    let _ = self.handle(c);
                }
                if let Some(v) = self.verify.take() {
                    for d in v.held {
                        self.convert(d);
                    }
                }
                if self.ready.is_empty() {
                    self.done = true;
                }
            }
        }
    }

    fn stop(&mut self) {
        self.stop_request();
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.halt_ring();
        for t in &self.transfers {
            let _ = t.clear_callback();
        }
    }
}
