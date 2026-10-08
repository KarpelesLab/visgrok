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

use visgrok::analyzer::{Analyzer, DecoderOptions, Tagged};
use visgrok::decode::Event;
use visgrok::formats::{SampleWriter, WriteOptions};
use visgrok::roles::Role;
use visgrok::store::{SampleStore, StoredEvent};
use visgrok::{Block, CaptureInfo, Source};

/// What to start a pipeline with besides the source and output.
#[derive(Default)]
pub struct Setup {
    /// Extra metadata recorded in `.vgk` files.
    pub extra: Vec<(String, String)>,
    /// Initial role per channel.
    pub roles: Vec<Option<Role>>,
    /// Decoder settings.
    pub options: DecoderOptions,
    /// Channel names (`None`: `D<n>`).
    pub names: Vec<Option<String>>,
    /// Never skip blocks in analysis (replaying a file: the source can wait).
    pub lossless_analysis: bool,
    /// Use this writer instead of creating one from the output path.
    pub writer: Option<Box<dyn SampleWriter>>,
    /// Store that receives decoded events (web UI).
    pub store: Option<Arc<SampleStore>>,
}

/// Shared state between the pipeline threads and the UI.
pub struct Pipeline {
    pub info: CaptureInfo,
    pub output: Option<PathBuf>,
    pub analyzer: Mutex<Analyzer>,
    /// User-assigned roles; `None` means "not assigned".
    pub roles: Mutex<Vec<Option<Role>>>,
    /// Channel names (default `D<n>`).
    pub names: Vec<String>,
    /// Decoder settings (SPI mode/protocol, UART auto-baud).
    pub options: Mutex<DecoderOptions>,
    samples: AtomicU64,
    blocks_skipped: AtomicU64,
    written: AtomicU64,
    raw_written: AtomicU64,
    stop: AtomicBool,
    done: AtomicBool,
    error: Mutex<Option<String>>,
    started: Instant,
    log_seen: AtomicU64,
    threads: Mutex<Vec<JoinHandle<()>>>,
    store: Option<Arc<SampleStore>>,
}

impl Pipeline {
    /// Starts the capture threads.
    ///
    /// The output format follows the extension: `.sr` writes a sigrok
    /// session, anything else the compressed visgrok format. `extra` is
    /// recorded in the visgrok file's metadata.
    pub fn start(source: Box<dyn Source>, output: Option<PathBuf>, setup: Setup) -> Result<Arc<Pipeline>, String> {
        let info = source.info();
        let Setup {
            extra,
            mut roles,
            options,
            names,
            lossless_analysis,
            writer,
            store,
        } = setup;
        roles.resize(info.channels, None);
        let names: Vec<String> = (0..info.channels)
            .map(|i| names.get(i).cloned().flatten().unwrap_or_else(|| format!("D{i}")))
            .collect();
        let writer = match (writer, &output) {
            (Some(w), _) => Some(Recorder(w)),
            (None, Some(p)) => Some(Recorder::create(p, &info, &names, extra).map_err(|e| format!("{}: {e}", p.display()))?),
            (None, None) => None,
        };
        let pipe = Pipeline {
            analyzer: Mutex::new(Analyzer::new(info.channels, info.samplerate)),
            roles: Mutex::new(roles),
            names,
            options: Mutex::new(options),
            info,
            output,
            samples: AtomicU64::new(0),
            blocks_skipped: AtomicU64::new(0),
            written: AtomicU64::new(0),
            raw_written: AtomicU64::new(0),
            stop: AtomicBool::new(false),
            done: AtomicBool::new(false),
            error: Mutex::new(None),
            started: Instant::now(),
            log_seen: AtomicU64::new(0),
            threads: Mutex::new(Vec::new()),
            store,
        };
        pipe.rebuild_decoders();
        let shared = Arc::new(pipe);
        let (wtx, wrx) = sync_channel::<Arc<Block>>(1024);
        let (atx, arx) = sync_channel::<Arc<Block>>(4);
        let mut threads = Vec::new();
        let s = shared.clone();
        let wtx = writer.is_some().then_some(wtx);
        threads.push(std::thread::spawn(move || s.acquire(source, wtx, atx, lossless_analysis)));
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

    fn acquire(&self, mut source: Box<dyn Source>, wtx: Option<SyncSender<Arc<Block>>>, atx: SyncSender<Arc<Block>>, lossless: bool) {
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
                    if lossless {
                        let _ = atx.send(b);
                        continue;
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

    fn write(&self, mut w: Recorder, rx: Receiver<Arc<Block>>) {
        for b in rx {
            if let Err(e) = w.0.write_block(&b) {
                self.fail(format!("write: {e}"));
                return;
            }
            self.written.store(w.bytes_written(), Ordering::Relaxed);
            self.raw_written.store(w.raw_written(), Ordering::Relaxed);
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
                let mut a = self.analyzer.lock().unwrap();
                let before = a.annotation_count;
                a.process(&part);
                if let Some(store) = &self.store {
                    let new = ((a.annotation_count - before) as usize).min(a.annotations.len());
                    if new > 0 {
                        let names: Vec<(Arc<str>, u8)> = a
                            .decoders()
                            .iter()
                            .map(|d| (Arc::from(d.name()), d.channels().trailing_zeros() as u8))
                            .collect();
                        let skip = a.annotations.len() - new;
                        for t in a.annotations.iter().skip(skip) {
                            if let Event::Frame(v) = &t.annotation.event {
                                store.add_frame(t.annotation.start, v.clone());
                            }
                        }
                        store.add_events(
                            a.annotations
                                .iter()
                                .skip(skip)
                                .filter(|t| !matches!(t.annotation.event, Event::Frame(_)))
                                .map(|t| {
                                    let (source, channel) = names.get(t.decoder).cloned().unwrap_or((Arc::from("?"), 0));
                                    StoredEvent::from_annotation(&t.annotation, source, channel)
                                }),
                        );
                    }
                }
                drop(a);
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
            s += &format!(", wrote {} to {}", self.written_text(), p.display());
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

    /// Output size as text, with the compression ratio when meaningful.
    pub fn written_text(&self) -> String {
        let w = self.written();
        let raw = self.raw_written.load(Ordering::Relaxed);
        if w > 0 && raw > w * 11 / 10 {
            format!("{} ({:.0}x)", fmt_bytes(w), raw as f64 / w as f64)
        } else {
            fmt_bytes(w)
        }
    }

    /// Blocks the analysis thread skipped because it was behind.
    pub fn blocks_skipped(&self) -> u64 {
        self.blocks_skipped.load(Ordering::Relaxed)
    }

    /// Effective role per channel: the user assignment, else nothing.
    pub fn effective_roles(&self) -> Vec<Role> {
        self.roles
            .lock()
            .unwrap()
            .iter()
            .map(|r| r.clone().unwrap_or(Role::Unknown))
            .collect()
    }

    /// Rebuilds decoders from the current role assignments.
    pub fn rebuild_decoders(&self) {
        let roles = self.effective_roles();
        let mut a = self.analyzer.lock().unwrap();
        let opts = self.options.lock().unwrap().clone();
        let d = a.decoders_for_roles(&roles, &opts);
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
            self.written_text(),
            self.blocks_skipped()
        )
    }

    /// Formatted annotations produced since the previous call.
    pub fn drain_log(&self) -> Vec<String> {
        let a = self.analyzer.lock().unwrap();
        let seen = self.log_seen.swap(a.annotation_count, Ordering::Relaxed);
        let pending = a.annotation_count - seen;
        let new = pending.min(a.annotations.len() as u64) as usize;
        let skip = a.annotations.len() - new;
        let names: Vec<String> = a.decoders().iter().map(|d| d.name()).collect();
        let mut out = Vec::with_capacity(new + 1);
        if pending > new as u64 {
            out.push(format!(
                "[{} decoded events not shown: log history overflowed]",
                pending - new as u64
            ));
        }
        out.extend(
            a.annotations
                .iter()
                .skip(skip)
                .filter(|t| !matches!(t.annotation.event, Event::Frame(_)))
                .map(|t| format_annotation(t, &names, a.samplerate())),
        );
        out
    }
}

/// Renders one annotation as text.
pub fn format_annotation(t: &Tagged, names: &[String], samplerate: u64) -> String {
    let ts = t.annotation.start as f64 / samplerate as f64;
    let name = names.get(t.decoder).map(String::as_str).unwrap_or("?");
    format!("{ts:12.6}s {name:<24} {}", t.annotation.event)
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

/// An output file in any supported format.
struct Recorder(Box<dyn SampleWriter>);

impl Recorder {
    fn create(path: &std::path::Path, info: &CaptureInfo, names: &[String], extra: Vec<(String, String)>) -> std::io::Result<Recorder> {
        let mut info = info.clone();
        info.names = names.to_vec();
        let mut opts = WriteOptions::default();
        opts.extra = extra;
        Ok(Recorder(visgrok::formats::create(path, &info, &opts)?))
    }

    fn bytes_written(&self) -> u64 {
        self.0.bytes_written()
    }

    fn raw_written(&self) -> u64 {
        self.0.raw_written()
    }

    fn finish(self) -> std::io::Result<()> {
        self.0.finish()
    }
}
