//! Random-access view of a capture, live or recorded, for browsing UIs.
//!
//! A [`SampleStore`] answers "what do channels look like over samples
//! `a..b`, in `n` columns" ([`SampleStore::view`]) without ever being on the
//! capture's critical path:
//!
//! - an **overview** of 8 bytes per [`TILE`] samples (the state at the tile
//!   start and the channels that change inside it) answers zoomed-out views
//!   of any length in milliseconds;
//! - zoomed-in views read raw samples from a **ring of recent blocks** kept
//!   in memory, or from the `.vgk` file's chunks (decompressed on demand,
//!   with a small cache), located through a **chunk index** that the writer
//!   publishes as each chunk reaches the disk.
//!
//! During a live capture the writer thread feeds the store
//! ([`StoreWriter`]); locks are only held to push or clone `Arc`s, never
//! while copying or decompressing. Decoded protocol events are kept as
//! [`StoredEvent`]s for range queries.

use std::collections::VecDeque;
use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};

use crate::block::{Block, Sample, channel_mask};
use crate::formats::SampleWriter;
use crate::source::CaptureInfo;
use crate::vgk::{ChunkRef, Meta, VgkWriter, read_chunk_at, scan};

/// Samples per overview tile.
pub const TILE: u64 = 4096;
/// Raw bytes of recent blocks kept in memory beyond what's on disk.
const RECENT_BYTES: usize = 256 << 20;
/// Decompressed chunks cached for zoomed-in views.
const CACHE_CHUNKS: usize = 8;

/// Overview of [`TILE`] consecutive samples.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tile {
    /// State at the first sample of the tile.
    pub first: Sample,
    /// Channels whose level changes anywhere inside the tile.
    pub changed: Sample,
}

/// One column of a view, for one channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    /// Low throughout.
    Low,
    /// High throughout.
    High,
    /// Toggles inside the column.
    Busy,
    /// No data (not captured yet, or not loaded).
    Unknown,
}

impl Level {
    /// One-character encoding: `0`, `1`, `x`, `?`.
    pub fn char(self) -> char {
        match self {
            Level::Low => '0',
            Level::High => '1',
            Level::Busy => 'x',
            Level::Unknown => '?',
        }
    }
}

/// A decoded event, kept for range queries.
#[derive(Clone, Debug)]
pub struct StoredEvent {
    /// First sample.
    pub start: u64,
    /// Sample after the end.
    pub end: u64,
    /// Decoder that produced it (e.g. `"UART ch2 auto"`).
    pub source: Arc<str>,
    /// Lowest channel the decoder uses (where UIs draw the event).
    pub channel: u8,
    /// Description.
    pub text: String,
}

struct Tiles {
    done: Vec<Tile>,
    /// Accumulators for the tile being filled: (first, or, and, count).
    acc: Option<(Sample, Sample, Sample, u64)>,
}

/// Random-access capture data (see the module docs).
pub struct SampleStore {
    info: RwLock<CaptureInfo>,
    path: PathBuf,
    unit: usize,
    mask: Sample,
    tiles: RwLock<Tiles>,
    chunks: RwLock<Vec<ChunkRef>>,
    recent: Mutex<VecDeque<Arc<Block>>>,
    total: AtomicU64,
    /// Samples covered by chunks on disk.
    on_disk: AtomicU64,
    live: AtomicBool,
    /// Background loading progress (samples processed), for UIs.
    loaded: AtomicU64,
    file: Mutex<Option<File>>,
    cache: Mutex<VecDeque<(u64, Arc<Vec<u8>>)>>,
    events: RwLock<Vec<StoredEvent>>,
    /// Display contents after each update, in sample order.
    frames: RwLock<Vec<(u64, Arc<crate::decode::DisplayView>)>>,
}

impl SampleStore {
    fn new(info: CaptureInfo, path: PathBuf, live: bool) -> SampleStore {
        SampleStore {
            unit: info.unit_size,
            mask: channel_mask(info.channels),
            info: RwLock::new(info),
            path,
            tiles: RwLock::new(Tiles {
                done: Vec::new(),
                acc: None,
            }),
            chunks: RwLock::new(Vec::new()),
            recent: Mutex::new(VecDeque::new()),
            total: AtomicU64::new(0),
            on_disk: AtomicU64::new(0),
            live: AtomicBool::new(live),
            loaded: AtomicU64::new(0),
            file: Mutex::new(None),
            cache: Mutex::new(VecDeque::new()),
            events: RwLock::new(Vec::new()),
            frames: RwLock::new(Vec::new()),
        }
    }

    /// Creates a store for a live capture recorded to `path` (`.vgk`), and
    /// the writer that feeds it; hand the writer to the capture pipeline.
    pub fn create_live(path: &Path, info: &CaptureInfo, extra: Vec<(String, String)>) -> io::Result<(Arc<SampleStore>, StoreWriter)> {
        let store = Arc::new(SampleStore::new(info.clone(), path.to_path_buf(), true));
        let mut meta = Meta::from_info(info);
        meta.extra = extra;
        let mut w = VgkWriter::create(path, &meta)?;
        let s = store.clone();
        w.set_on_chunk(move |c| s.publish_chunk(c));
        Ok((store.clone(), StoreWriter { store, w: Some(w) }))
    }

    /// Opens a recorded `.vgk` capture. Without a stored overview, tiles are
    /// computed on a background thread ([`SampleStore::loaded`] tracks it).
    pub fn open(path: &Path) -> io::Result<Arc<SampleStore>> {
        let sc = scan(path)?;
        let m = &sc.meta;
        let info = CaptureInfo {
            device: m.device.clone(),
            channels: m.channels,
            samplerate: m.samplerate,
            unit_size: m.unit_size,
            names: m.names.clone(),
        };
        let store = Arc::new(SampleStore::new(info, path.to_path_buf(), false));
        store.total.store(sc.samples, Ordering::SeqCst);
        store.on_disk.store(sc.samples, Ordering::SeqCst);
        *store.chunks.write().unwrap() = sc.chunks;
        match sc.overview {
            Some((t, tiles)) if t == TILE && tiles.len() as u64 >= sc.samples / TILE => {
                store.tiles.write().unwrap().done = tiles;
                store.loaded.store(sc.samples, Ordering::SeqCst);
            }
            _ => {
                let s = store.clone();
                std::thread::spawn(move || s.build_tiles());
            }
        }
        Ok(store)
    }

    /// Imports any supported capture file (`.sr`, `.vcd`, ...) into a new
    /// `.vgk` with an overview, reporting progress in samples.
    pub fn import(src: &Path, dst: &Path, progress: &AtomicU64, cancel: &AtomicBool) -> io::Result<()> {
        let mut source = crate::formats::open(src, &crate::formats::ReadOptions::default())?;
        let info = source.info();
        let (_, mut w) = SampleStore::create_live(dst, &info, vec![("imported_from".into(), src.display().to_string())])?;
        while let Some(b) = source.next_block()? {
            if cancel.load(Ordering::SeqCst) {
                return Err(io::Error::new(io::ErrorKind::Interrupted, "import cancelled"));
            }
            let end = b.end();
            w.write_block(&Arc::new(b))?;
            progress.store(end, Ordering::SeqCst);
        }
        Box::new(w).finish()
    }

    /// Computes the overview by reading every chunk (files without one).
    fn build_tiles(&self) {
        let chunks = self.chunks.read().unwrap().clone();
        let Ok(mut f) = File::open(&self.path) else { return };
        for c in chunks {
            let Ok(raw) = read_chunk_at(&mut f, c.offset) else { return };
            self.add_tiles(&Block::new(c.first, self.unit, raw));
            self.loaded.store(c.first + c.samples, Ordering::SeqCst);
        }
    }

    /// Capture description.
    pub fn info(&self) -> CaptureInfo {
        self.info.read().unwrap().clone()
    }

    /// Renames channels (UI only; files keep their recorded names).
    pub fn set_names(&self, names: Vec<String>) {
        self.info.write().unwrap().names = names;
    }

    /// Path of the `.vgk` file.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Samples captured so far.
    pub fn total(&self) -> u64 {
        self.total.load(Ordering::SeqCst)
    }

    /// Whether the capture is still running.
    pub fn is_live(&self) -> bool {
        self.live.load(Ordering::SeqCst)
    }

    /// Samples whose overview is ready (equals [`SampleStore::total`] once
    /// background loading is done).
    pub fn loaded(&self) -> u64 {
        if self.is_live() {
            self.total()
        } else {
            self.loaded.load(Ordering::SeqCst)
        }
    }

    fn publish_chunk(&self, c: ChunkRef) {
        self.chunks.write().unwrap().push(c);
        self.on_disk.store(c.first + c.samples, Ordering::SeqCst);
        self.trim_recent();
    }

    /// Drops in-memory blocks that are on disk, beyond the memory budget.
    fn trim_recent(&self) {
        let on_disk = self.on_disk.load(Ordering::SeqCst);
        let mut r = self.recent.lock().unwrap();
        let mut bytes: usize = r.iter().map(|b| b.data.len()).sum();
        while bytes > RECENT_BYTES
            && let Some(b) = r.front()
        {
            // Never drop data that isn't readable from the file yet, unless
            // the writer is so far behind that memory would run away.
            if b.end() > on_disk && bytes < 4 * RECENT_BYTES {
                break;
            }
            bytes -= b.data.len();
            r.pop_front();
        }
    }

    fn add_tiles(&self, b: &Block) {
        let mut t = self.tiles.write().unwrap();
        let Tiles { done, acc } = &mut *t;
        let mask = self.mask;
        let mut i = 0;
        let n = b.len();
        while i < n {
            let (first, or, and, count) = acc.get_or_insert_with(|| {
                let s = b.sample(i) & mask;
                (s, s, s, 0)
            });
            let take = ((TILE - *count) as usize).min(n - i);
            for k in i..i + take {
                let s = b.sample(k) & mask;
                *or |= s;
                *and &= s;
            }
            *count += take as u64;
            i += take;
            if *count == TILE {
                done.push(Tile {
                    first: *first,
                    changed: *or ^ *and,
                });
                *acc = None;
            }
        }
    }

    /// Feeds a block of a live capture (called by [`StoreWriter`]).
    fn append(&self, b: &Arc<Block>) {
        self.add_tiles(b);
        self.recent.lock().unwrap().push_back(b.clone());
        self.total.store(b.end(), Ordering::SeqCst);
        self.trim_recent();
    }

    /// Reads raw samples `start..end` (clamped to what exists), as packed
    /// units. Returns the first sample index actually covered.
    pub fn read(&self, start: u64, end: u64) -> io::Result<(u64, Vec<u8>)> {
        let end = end.min(self.total());
        if start >= end {
            return Ok((start, Vec::new()));
        }
        let unit = self.unit as u64;
        let mut out = Vec::with_capacity(((end - start) * unit) as usize);
        let mut at = start;
        // Clone the relevant Arcs under the lock, copy outside it.
        let recent: Vec<Arc<Block>> = self
            .recent
            .lock()
            .unwrap()
            .iter()
            .filter(|b| b.end() > start && b.start < end)
            .cloned()
            .collect();
        let recent_start = recent.first().map_or(u64::MAX, |b| b.start);
        if at < recent_start {
            let chunks: Vec<ChunkRef> = {
                let c = self.chunks.read().unwrap();
                let i = c.partition_point(|c| c.first + c.samples <= at);
                c[i..].iter().take_while(|c| c.first < end.min(recent_start)).copied().collect()
            };
            for c in chunks {
                if c.first > at {
                    break; // hole: report what we have
                }
                let data = self.chunk_data(c)?;
                let from = (at - c.first) * unit;
                let to = ((end.min(c.first + c.samples)) - c.first) * unit;
                out.extend_from_slice(&data[from as usize..to as usize]);
                at = c.first + (to / unit);
                if at >= end {
                    break;
                }
            }
        }
        for b in recent {
            if at >= end {
                break;
            }
            if b.end() <= at {
                continue;
            }
            if b.start > at {
                if out.is_empty() {
                    at = b.start; // nothing before: start here
                } else {
                    break;
                }
            }
            let from = ((at - b.start) * unit) as usize;
            let to = ((end.min(b.end()) - b.start) * unit) as usize;
            out.extend_from_slice(&b.data[from..to]);
            at = end.min(b.end());
        }
        let first = at - out.len() as u64 / unit;
        Ok((first, out))
    }

    fn chunk_data(&self, c: ChunkRef) -> io::Result<Arc<Vec<u8>>> {
        if let Some((_, d)) = self.cache.lock().unwrap().iter().find(|(o, _)| *o == c.offset) {
            return Ok(d.clone());
        }
        let raw = {
            let mut f = self.file.lock().unwrap();
            if f.is_none() {
                *f = Some(File::open(&self.path)?);
            }
            read_chunk_at(f.as_mut().unwrap(), c.offset)?
        };
        let d = Arc::new(raw);
        let mut cache = self.cache.lock().unwrap();
        cache.push_back((c.offset, d.clone()));
        while cache.len() > CACHE_CHUNKS {
            cache.pop_front();
        }
        Ok(d)
    }

    /// Per-channel levels of samples `start..end` in `columns` columns.
    /// Returns one `Vec<Level>` per channel.
    pub fn view(&self, start: u64, end: u64, columns: usize) -> io::Result<Vec<Vec<Level>>> {
        let channels = self.info().channels;
        let columns = columns.clamp(1, 8192);
        let mut out = vec![vec![Level::Unknown; columns]; channels];
        if end <= start {
            return Ok(out);
        }
        let span = (end - start) as f64 / columns as f64;
        let col_range = |c: usize| (start + (c as f64 * span) as u64, start + ((c + 1) as f64 * span).ceil() as u64);
        let mut put = |c: usize, first: Sample, changed: Sample| {
            for (ch, o) in out.iter_mut().enumerate() {
                o[c] = if changed >> ch & 1 != 0 {
                    Level::Busy
                } else if first >> ch & 1 != 0 {
                    Level::High
                } else {
                    Level::Low
                };
            }
        };
        if span >= TILE as f64 {
            let tiles = self.tiles.read().unwrap();
            let t = &tiles.done;
            for c in 0..columns {
                let (a, b) = col_range(c);
                let (ta, tb) = ((a / TILE) as usize, (b.div_ceil(TILE) as usize).max(a as usize / TILE as usize + 1));
                if ta >= t.len() {
                    break;
                }
                let first = t[ta].first;
                let changed = t[ta..tb.min(t.len())].iter().fold(0, |m, x| m | x.changed | (x.first ^ first));
                put(c, first, changed);
            }
            return Ok(out);
        }
        let (got, data) = self.read(start, end)?;
        let unit = self.unit;
        let n = data.len() / unit;
        let sample = |i: usize| -> Sample {
            let b = &data[i * unit..i * unit + unit];
            let mut v = [0u8; 4];
            v[..unit].copy_from_slice(b);
            Sample::from_le_bytes(v) & self.mask
        };
        for c in 0..columns {
            let (a, b) = col_range(c);
            if b <= got || a >= got + n as u64 {
                continue;
            }
            let (ia, ib) = ((a.max(got) - got) as usize, ((b.min(got + n as u64)) - got) as usize);
            if ia >= ib {
                continue;
            }
            let first = sample(ia);
            let (mut or, mut and) = (first, first);
            for i in ia..ib {
                let s = sample(i);
                or |= s;
                and &= s;
            }
            put(c, first, or ^ and);
        }
        Ok(out)
    }

    /// Records decoded events (in roughly increasing start order).
    pub fn add_events(&self, ev: impl IntoIterator<Item = StoredEvent>) {
        self.events.write().unwrap().extend(ev);
    }

    /// Forgets decoded events and display frames (before re-decoding).
    pub fn clear_events(&self) {
        self.events.write().unwrap().clear();
        self.frames.write().unwrap().clear();
    }

    /// Records what a display shows from sample `at` on.
    pub fn add_frame(&self, at: u64, view: Arc<crate::decode::DisplayView>) {
        let mut f = self.frames.write().unwrap();
        // Bounded memory: past the cap, keep every other old frame.
        if f.len() >= 100_000 {
            let mut i = 0;
            f.retain(|_| {
                i += 1;
                i % 2 == 0
            });
        }
        f.push((at, view));
    }

    /// The display as it was at sample `at` (the last update before it),
    /// with the update's sample index; also the number of frames.
    pub fn frame_at(&self, at: u64) -> (Option<(u64, Arc<crate::decode::DisplayView>)>, usize) {
        let f = self.frames.read().unwrap();
        let i = f.partition_point(|(s, _)| *s <= at);
        ((i > 0).then(|| f[i - 1].clone()), f.len())
    }

    /// Number of decoded events.
    pub fn event_count(&self) -> usize {
        self.events.read().unwrap().len()
    }

    /// Events overlapping `start..end`, at most `limit`.
    pub fn events(&self, start: u64, end: u64, limit: usize) -> Vec<StoredEvent> {
        let ev = self.events.read().unwrap();
        // Events are appended roughly in order; tolerate small disorder.
        let i = ev.partition_point(|e| e.end < start.saturating_sub(1 << 20));
        ev[i..]
            .iter()
            .filter(|e| e.end >= start && e.start < end)
            .take(limit)
            .cloned()
            .collect()
    }

    /// The first event starting after `at` (or the last one before it),
    /// for "next/previous event" navigation.
    pub fn event_near(&self, at: u64, forward: bool) -> Option<StoredEvent> {
        let ev = self.events.read().unwrap();
        if forward {
            ev.iter().find(|e| e.start > at).cloned()
        } else {
            ev.iter().rev().find(|e| e.start < at).cloned()
        }
    }

    /// Current overview tiles (a copy), e.g. to save with the file.
    pub fn tiles(&self) -> Vec<Tile> {
        self.tiles.read().unwrap().done.clone()
    }
}

/// The capture's disk writer, also feeding a [`SampleStore`].
pub struct StoreWriter {
    store: Arc<SampleStore>,
    w: Option<VgkWriter<std::io::BufWriter<File>>>,
}

impl SampleWriter for StoreWriter {
    /// Feeds a block (shared, so the store keeps it without copying).
    fn write_block(&mut self, b: &Arc<Block>) -> io::Result<()> {
        self.store.append(b);
        self.w.as_mut().unwrap().write(&b.data)
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        let start = self.store.total();
        let b = Arc::new(Block::new(start, self.store.unit, data.to_vec()));
        self.write_block(&b)
    }

    fn bytes_written(&self) -> u64 {
        self.w.as_ref().map_or(0, |w| w.bytes_written())
    }

    fn raw_written(&self) -> u64 {
        self.w.as_ref().map_or(0, |w| w.raw_written())
    }

    fn finish(mut self: Box<Self>) -> io::Result<()> {
        let mut w = self.w.take().unwrap();
        w.set_overview(TILE, &self.store.tiles());
        w.finish()?;
        self.store.live.store(false, Ordering::SeqCst);
        self.store.loaded.store(self.store.total(), Ordering::SeqCst);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info() -> CaptureInfo {
        CaptureInfo {
            device: "t".into(),
            channels: 4,
            samplerate: 1_000_000,
            unit_size: 1,
            names: Vec::new(),
        }
    }

    /// ch0 toggles every 1000 samples; ch1 every 100_000; ch2 always high.
    fn sample(i: u64) -> u8 {
        ((i / 1000) & 1) as u8 | (((i / 100_000) & 1) as u8) << 1 | 4
    }

    #[test]
    fn live_then_reopen() {
        let dir = std::env::temp_dir().join(format!("visgrok-store-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("s.vgk");
        let (store, mut w) = SampleStore::create_live(&path, &info(), Vec::new()).unwrap();
        let n = 10_000_000u64;
        let mut pos = 0;
        while pos < n {
            let len = 300_001.min(n - pos);
            let data: Vec<u8> = (pos..pos + len).map(sample).collect();
            w.write_block(&Arc::new(Block::new(pos, 1, data))).unwrap();
            pos += len;
        }
        // While live: a zoomed-in view in the middle and a full overview.
        let v = store.view(2_000_000, 2_004_000, 4).unwrap();
        assert_eq!(v[0][0], Level::Low);
        assert_eq!(v[0][1], Level::High);
        assert_eq!(v[2], vec![Level::High; 4]);
        let all = store.view(0, n, 100).unwrap();
        assert!(all[0].iter().all(|l| *l == Level::Busy));
        assert_eq!(all[3], vec![Level::Low; 100]);
        let (first, raw) = store.read(5_000_000, 5_000_010).unwrap();
        assert_eq!(first, 5_000_000);
        assert_eq!(raw, (5_000_000..5_000_010).map(sample).collect::<Vec<_>>());
        Box::new(w).finish().unwrap();

        // Reopened from disk: same answers, overview loaded from the file.
        let s2 = SampleStore::open(&path).unwrap();
        assert_eq!(s2.total(), n);
        assert_eq!(s2.loaded(), n);
        assert_eq!(s2.view(0, n, 100).unwrap(), all);
        let (first, raw) = s2.read(9_999_990, 10_000_000).unwrap();
        assert_eq!(first, 9_999_990);
        assert_eq!(raw, (9_999_990..10_000_000).map(sample).collect::<Vec<_>>());
        let z = s2.view(150_000, 250_000, 10).unwrap();
        assert_eq!(z[1][0], Level::High); // 150k..160k: ch1 high (100k..200k)
        assert_eq!(z[1][9], Level::Low); // 240k..250k: ch1 low
    }
}
