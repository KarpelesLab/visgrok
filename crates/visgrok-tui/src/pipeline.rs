//! Capture pipeline: acquisition, lossless disk writing and best-effort analysis
//! on separate threads.
//!
//! The acquisition thread pulls blocks from the source and hands each one to
//! the writer through a deep bounded queue (blocking: if the disk cannot keep
//! up, acquisition stalls and the device reports an overflow, which is surfaced
//! as an error rather than silently losing data). Analysis gets the same blocks
//! through a shallow queue and simply skips blocks when it falls behind.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Instant;

use visgrok::analyzer::{Analyzer, Tagged};
use visgrok::decode::Event;
use visgrok::roles::Role;
use visgrok::srzip::SrZipWriter;
use visgrok::{Block, CaptureInfo, Source};

/// Shared state between the pipeline threads and the UI.
pub struct Pipeline {
    pub info: CaptureInfo,
    pub output: Option<PathBuf>,
    pub analyzer: Mutex<Analyzer>,
    /// User-assigned roles; `None` means "not assigned".
    pub roles: Mutex<Vec<Option<Role>>>,
    samples: AtomicU64,
    blocks_skipped: AtomicU64,
    written: AtomicU64,
    stop: AtomicBool,
    done: AtomicBool,
    error: Mutex<Option<String>>,
    started: Instant,
    log_seen: AtomicU64,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl Pipeline {
    /// Starts the capture threads.
    pub fn start(source: Box<dyn Source>, output: Option<PathBuf>) -> Result<Arc<Pipeline>, String> {
        let info = source.info();
        let names: Vec<String> = (0..info.channels).map(|i| format!("D{i}")).collect();
        let writer = match &output {
            Some(p) => Some(
                SrZipWriter::create(p, &names, info.samplerate, info.unit_size)
                    .map_err(|e| format!("{}: {e}", p.display()))?,
            ),
            None => None,
        };
        let pipe = Pipeline {
            analyzer: Mutex::new(Analyzer::new(info.channels, info.samplerate)),
            roles: Mutex::new(vec![None; info.channels]),
            info,
            output,
            samples: AtomicU64::new(0),
            blocks_skipped: AtomicU64::new(0),
            written: AtomicU64::new(0),
            stop: AtomicBool::new(false),
            done: AtomicBool::new(false),
            error: Mutex::new(None),
            started: Instant::now(),
            log_seen: AtomicU64::new(0),
            threads: Mutex::new(Vec::new()),
        };
        let shared = Arc::new(pipe);
        let (wtx, wrx) = sync_channel::<Arc<Block>>(1024);
        let (atx, arx) = sync_channel::<Arc<Block>>(4);
        let mut threads = Vec::new();
        let s = shared.clone();
        let wtx = writer.is_some().then_some(wtx);
        threads.push(std::thread::spawn(move || s.acquire(source, wtx, atx)));
        if let Some(w) = writer {
            let s = shared.clone();
            threads.push(std::thread::spawn(move || s.write(w, wrx)));
        }
        let s = shared.clone();
        threads.push(std::thread::spawn(move || s.analyze(arx)));
        *shared.threads.lock().unwrap() = threads;
        Ok(shared)
    }

    fn fail(&self, msg: String) {
        let mut e = self.error.lock().unwrap();
        if e.is_none() {
            *e = Some(msg);
        }
        self.stop.store(true, Ordering::SeqCst);
    }

    fn acquire(&self, mut source: Box<dyn Source>, wtx: Option<SyncSender<Arc<Block>>>, atx: SyncSender<Arc<Block>>) {
        let mut stopping = false;
        loop {
            if !stopping && self.stop.load(Ordering::SeqCst) {
                source.stop();
                stopping = true;
            }
            match source.next_block() {
                Ok(Some(b)) => {
                    self.samples.store(b.end(), Ordering::Relaxed);
                    let b = Arc::new(b);
                    if let Some(w) = &wtx
                        && w.send(b.clone()).is_err()
                    {
                        break;
                    }
                    match atx.try_send(b) {
                        Ok(()) => {}
                        Err(TrySendError::Full(_)) => {
                            self.blocks_skipped.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(TrySendError::Disconnected(_)) => {}
                    }
                }
                Ok(None) => break,
                Err(e) => {
                    self.fail(format!("acquisition: {e}"));
                    break;
                }
            }
        }
        self.done.store(true, Ordering::SeqCst);
    }

    fn write(&self, mut w: SrZipWriter<std::io::BufWriter<std::fs::File>>, rx: Receiver<Arc<Block>>) {
        for b in rx {
            if let Err(e) = w.write(&b.data) {
                self.fail(format!("write: {e}"));
                return;
            }
            self.written.store(w.bytes_written(), Ordering::Relaxed);
        }
        match w.finish() {
            Ok(_) => {}
            Err(e) => self.fail(format!("write: {e}")),
        }
    }

    fn analyze(&self, rx: Receiver<Arc<Block>>) {
        // Work in slices so the UI can take the lock between them even when
        // dense signals make analysis slow.
        const SLICE: usize = 1 << 18;
        for b in rx {
            let mut i = 0;
            while i < b.len() {
                if self.stop.load(Ordering::Relaxed) {
                    return;
                }
                let part = b.slice(i, i + SLICE);
                self.analyzer.lock().unwrap().process(&part);
                i += SLICE;
                std::thread::yield_now();
            }
        }
    }

    /// Requests the capture to stop.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    /// True when the source has ended (or failed).
    pub fn finished(&self) -> bool {
        self.done.load(Ordering::SeqCst)
    }

    /// The first error encountered, if any.
    pub fn error(&self) -> Option<String> {
        self.error.lock().unwrap().clone()
    }

    /// Waits for all threads and returns a summary line.
    pub fn join(&self) -> String {
        let threads = std::mem::take(&mut *self.threads.lock().unwrap());
        for t in threads {
            let _ = t.join();
        }
        let mut s = format!(
            "captured {} samples in {:.1}s",
            self.samples.load(Ordering::Relaxed),
            self.seconds()
        );
        if let Some(p) = &self.output {
            s += &format!(", wrote {} to {}", fmt_bytes(self.written.load(Ordering::Relaxed)), p.display());
        }
        if let Some(e) = self.error() {
            s += &format!("; ERROR: {e}");
        }
        s
    }

    /// Wall-clock seconds since start.
    pub fn seconds(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// Samples acquired so far.
    pub fn samples(&self) -> u64 {
        self.samples.load(Ordering::Relaxed)
    }

    /// Bytes written to the output so far.
    pub fn written(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }

    /// Blocks the analysis thread skipped because it was behind.
    pub fn blocks_skipped(&self) -> u64 {
        self.blocks_skipped.load(Ordering::Relaxed)
    }

    /// Effective role per channel: the user assignment, else nothing.
    pub fn effective_roles(&self) -> Vec<Role> {
        self.roles.lock().unwrap().iter().map(|r| r.clone().unwrap_or(Role::Unknown)).collect()
    }

    /// Rebuilds decoders from the current role assignments.
    pub fn rebuild_decoders(&self) {
        let roles = self.effective_roles();
        let mut a = self.analyzer.lock().unwrap();
        let d = a.decoders_for_roles(&roles);
        a.set_decoders(d);
        self.log_seen.store(a.annotation_count, Ordering::Relaxed);
    }

    /// Assigns every channel without a manual role its suggested role, then
    /// rebuilds decoders.
    pub fn apply_suggestions(&self) {
        let sugg = self.analyzer.lock().unwrap().suggest();
        {
            let mut roles = self.roles.lock().unwrap();
            for (r, s) in roles.iter_mut().zip(sugg) {
                if r.is_none() && s.role != Role::Unknown {
                    *r = Some(s.role);
                }
            }
        }
        self.rebuild_decoders();
    }

    /// One-line status for headless mode.
    pub fn status_line(&self) -> String {
        let t = self.seconds();
        let n = self.samples();
        format!(
            "{:7.1}s {:>14} samples ({:.1} MS/s) written {} analysis-skipped {}",
            t,
            n,
            n as f64 / t / 1e6,
            fmt_bytes(self.written()),
            self.blocks_skipped()
        )
    }

    /// Formatted annotations produced since the previous call.
    pub fn drain_log(&self) -> Vec<String> {
        let a = self.analyzer.lock().unwrap();
        let seen = self.log_seen.swap(a.annotation_count, Ordering::Relaxed);
        let new = (a.annotation_count - seen).min(a.annotations.len() as u64) as usize;
        let skip = a.annotations.len() - new;
        let names: Vec<String> = a.decoders().iter().map(|d| d.name()).collect();
        a.annotations
            .iter()
            .skip(skip)
            .map(|t| format_annotation(t, &names, a.samplerate()))
            .collect()
    }
}

/// Renders one annotation as text.
pub fn format_annotation(t: &Tagged, names: &[String], samplerate: u64) -> String {
    let ts = t.annotation.start as f64 / samplerate as f64;
    let name = names.get(t.decoder).map(String::as_str).unwrap_or("?");
    format!("{ts:12.6}s {name:<24} {}", format_event(&t.annotation.event))
}

/// Renders a decoded event.
pub fn format_event(e: &Event) -> String {
    match e {
        Event::UartByte { value, framing_error, parity_error } => {
            let c = char::from_u32(*value as u32).filter(|c| c.is_ascii_graphic() || *c == ' ');
            let mut s = match c {
                Some(c) => format!("{value:02x} '{c}'"),
                None => format!("{value:02x}"),
            };
            if *framing_error {
                s += " [framing error]";
            }
            if *parity_error {
                s += " [parity error]";
            }
            s
        }
        Event::UartBreak => "BREAK".into(),
        Event::I2cStart => "START".into(),
        Event::I2cStop => "STOP".into(),
        Event::I2cAddress { addr, read, ack } => {
            format!("addr {addr:#04x} {} {}", if *read { "R" } else { "W" }, if *ack { "ACK" } else { "NAK" })
        }
        Event::I2cData { value, ack } => format!("data {value:02x} {}", if *ack { "ACK" } else { "NAK" }),
        Event::SpiSelect(true) => "CS asserted".into(),
        Event::SpiSelect(false) => "CS released".into(),
        Event::SpiWord { mosi, miso } => {
            let f = |v: &Option<u32>| v.map_or("--".to_string(), |v| format!("{v:02x}"));
            format!("mosi {} miso {}", f(mosi), f(miso))
        }
    }
}

/// Formats a byte count.
pub fn fmt_bytes(n: u64) -> String {
    let n = n as f64;
    if n >= 1e9 {
        format!("{:.2} GB", n / 1e9)
    } else if n >= 1e6 {
        format!("{:.1} MB", n / 1e6)
    } else if n >= 1e3 {
        format!("{:.1} kB", n / 1e3)
    } else {
        format!("{n} B")
    }
}
