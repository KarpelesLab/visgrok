//! The visgrok capture format (`.vgk`): compressed, streamable, crash-tolerant.
//!
//! ```text
//! file    = header chunk* [index footer]
//! header  = "VISGROK\0" u32:version u32:meta_len meta u32:crc32(meta)
//! meta    = UTF-8 "key=value\n" lines (see [`Meta`])
//! chunk   = "VGKC" u8:kind u8:codec u16:0 u64:first_sample
//!           u32:raw_len u32:stored_len u32:crc32(raw) u32:crc32(chunk header[0..28])
//!           payload[stored_len]
//! footer  = "VGKEND\0\0" u64:offset of the index chunk
//! ```
//!
//! All integers are little endian. Sample chunks (`kind` 1) hold
//! `raw_len / unit_size` consecutive samples in the same layout as
//! [`crate::Block`], compressed independently with zstd (`codec` 1)
//! or stored (`codec` 0). Independent chunks let the writer compress on
//! several threads and let readers seek. The index chunk (`kind` 0xFE)
//! lists `(u64 offset, u64 first_sample)` for every sample chunk and has the
//! total sample count in `first_sample`. A file cut short by a crash is still
//! readable up to its last complete chunk; the index is only an accelerator.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;

use compcol::vec::{compress_to_vec_with, decompress_to_vec_capped};
use compcol::zstd::{EncoderConfig, Zstd};

use crate::block::Block;
use crate::pool::OrderedPool;
use crate::source::{CaptureInfo, Source};
use crate::srzip::crc32;

/// File magic.
pub const MAGIC: &[u8; 8] = b"VISGROK\0";
/// Format version.
pub const VERSION: u32 = 1;
const CHUNK_MAGIC: &[u8; 4] = b"VGKC";
const FOOTER_MAGIC: &[u8; 8] = b"VGKEND\0\0";
const CHUNK_HEADER: usize = 32;
/// Default raw bytes per chunk.
pub const DEFAULT_CHUNK: usize = 4 << 20;
/// Chunks waiting for compression before new ones are stored uncompressed.
const MAX_BACKLOG: usize = 48;

/// Chunk kinds.
pub mod kind {
    /// Logic samples.
    pub const SAMPLES: u8 = 1;
    /// Overview tiles (see [`crate::store`]): `u64` tile size, `u64` count,
    /// then per tile `u32` state at the tile start and `u32` mask of the
    /// channels that change within it. Written before the index.
    pub const OVERVIEW: u8 = 2;
    /// Index of sample chunks.
    pub const INDEX: u8 = 0xfe;
}

/// Payload codecs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Codec {
    /// Uncompressed.
    Store = 0,
    /// Zstandard.
    Zstd = 1,
}

/// Capture metadata.
#[derive(Clone, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct Meta {
    /// Device description.
    pub device: String,
    /// Number of channels.
    pub channels: usize,
    /// Samples per second.
    pub samplerate: u64,
    /// Bytes per sample (1 or 2).
    pub unit_size: usize,
    /// Channel names, one per channel.
    pub names: Vec<String>,
    /// Capture start, milliseconds since the Unix epoch.
    pub started_ms: u64,
    /// Any other `key=value` pairs (threshold, serial, pattern, ...).
    pub extra: Vec<(String, String)>,
}

impl Meta {
    /// Metadata for a capture described by `info`, started now.
    pub fn from_info(info: &CaptureInfo) -> Meta {
        Meta {
            device: info.device.clone(),
            channels: info.channels,
            samplerate: info.samplerate,
            unit_size: info.unit_size,
            names: info.all_names(),
            started_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_millis() as u64),
            extra: Vec::new(),
        }
    }

    fn encode(&self) -> String {
        let mut s = String::new();
        let clean = |v: &str| v.replace(['\n', '\r'], " ");
        s += &format!("device={}\n", clean(&self.device));
        s += &format!("channels={}\n", self.channels);
        s += &format!("samplerate={}\n", self.samplerate);
        s += &format!("unit_size={}\n", self.unit_size);
        s += &format!(
            "names={}\n",
            self.names.iter().map(|n| clean(n).replace(',', "_")).collect::<Vec<_>>().join(",")
        );
        s += &format!("started_ms={}\n", self.started_ms);
        for (k, v) in &self.extra {
            s += &format!("{}={}\n", clean(k).replace('=', "_"), clean(v));
        }
        s
    }

    fn decode(text: &str) -> io::Result<Meta> {
        let mut m = Meta::default();
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else { continue };
            let num = |v: &str| v.parse::<u64>().map_err(|_| invalid(format!("bad {k}: {v:?}")));
            match k {
                "device" => m.device = v.to_string(),
                "channels" => m.channels = num(v)? as usize,
                "samplerate" => m.samplerate = num(v)?,
                "unit_size" => m.unit_size = num(v)? as usize,
                "names" => m.names = v.split(',').map(str::to_string).collect(),
                "started_ms" => m.started_ms = num(v)?,
                _ => m.extra.push((k.to_string(), v.to_string())),
            }
        }
        if !matches!(m.unit_size, 1 | 2 | 4) || m.channels == 0 || m.samplerate == 0 {
            return Err(invalid("incomplete metadata".into()));
        }
        Ok(m)
    }
}

fn invalid(msg: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg)
}

fn chunk_header(kind: u8, codec: Codec, first: u64, raw_len: usize, stored_len: usize, crc: u32) -> [u8; CHUNK_HEADER] {
    let mut h = [0u8; CHUNK_HEADER];
    h[0..4].copy_from_slice(CHUNK_MAGIC);
    h[4] = kind;
    h[5] = codec as u8;
    h[8..16].copy_from_slice(&first.to_le_bytes());
    h[16..20].copy_from_slice(&(raw_len as u32).to_le_bytes());
    h[20..24].copy_from_slice(&(stored_len as u32).to_le_bytes());
    h[24..28].copy_from_slice(&crc.to_le_bytes());
    let hc = crc32(&h[..28]);
    h[28..32].copy_from_slice(&hc.to_le_bytes());
    h
}

/// Builds a complete sample chunk (header + payload).
fn encode_chunk(first: u64, raw: &[u8], compress: bool) -> Vec<u8> {
    let crc = crc32(raw);
    let packed = if compress {
        compress_to_vec_with::<Zstd>(raw, EncoderConfig { level: 1 })
            .ok()
            .filter(|p| p.len() < raw.len())
    } else {
        None
    };
    let (codec, payload) = match &packed {
        Some(p) => (Codec::Zstd, p.as_slice()),
        None => (Codec::Store, raw),
    };
    let mut out = Vec::with_capacity(CHUNK_HEADER + payload.len());
    out.extend_from_slice(&chunk_header(kind::SAMPLES, codec, first, raw.len(), payload.len(), crc));
    out.extend_from_slice(payload);
    out
}

struct Job {
    first: u64,
    data: Vec<u8>,
    compress: bool,
}

/// Part of an overview tile: (state at its first sample, OR and AND of
/// its samples, sample count).
type TilePart = (u32, u32, u32, u64);

/// Splits a chunk starting at sample `first` at overview tile boundaries.
fn tile_parts(first: u64, data: &[u8], unit: usize) -> Vec<TilePart> {
    let tile = crate::store::TILE;
    let n = data.len() / unit;
    let sample = |i: usize| {
        let mut v = [0u8; 4];
        v[..unit].copy_from_slice(&data[i * unit..i * unit + unit]);
        u32::from_le_bytes(v)
    };
    let mut out = Vec::with_capacity(n / tile as usize + 2);
    let mut i = 0;
    while i < n {
        let room = (tile - (first + i as u64) % tile) as usize;
        let end = (i + room).min(n);
        let s0 = sample(i);
        let (mut or, mut and) = (s0, s0);
        for k in i + 1..end {
            let s = sample(k);
            or |= s;
            and &= s;
        }
        out.push((s0, or, and, (end - i) as u64));
        i = end;
    }
    out
}

/// Writes a `.vgk` file, compressing chunks on a pool of worker threads.
pub struct VgkWriter<W: Write + Send + 'static> {
    out: W,
    pos: u64,
    unit_size: usize,
    chunk: Vec<u8>,
    chunk_size: usize,
    /// Samples handed to `write` so far.
    samples: u64,
    /// First sample of the chunk being filled.
    chunk_first: u64,
    /// Encodes chunks; yields (raw length, encoded chunk, overview tile
    /// parts) in order.
    pool: OrderedPool<Job, (u64, Vec<u8>, Vec<TilePart>)>,
    /// Overview tiles of the chunks written, and the one being filled.
    tiles: Vec<crate::store::Tile>,
    tile_acc: Option<TilePart>,
    /// Called (after a flush) for every sample chunk that reaches the file.
    on_chunk: Option<Box<dyn FnMut(ChunkRef) + Send>>,
    /// Overview chunk payload to write at `finish`.
    overview: Option<Vec<u8>>,
    index: Vec<(u64, u64)>,
    raw_written: u64,
    stored_chunks: u64,
}

impl VgkWriter<BufWriter<File>> {
    /// Creates `path` and writes the header.
    pub fn create(path: impl AsRef<Path>, meta: &Meta) -> io::Result<VgkWriter<BufWriter<File>>> {
        let f = BufWriter::with_capacity(4 << 20, File::create(path)?);
        VgkWriter::new(f, meta, DEFAULT_CHUNK, None)
    }
}

impl<W: Write + Send + 'static> VgkWriter<W> {
    /// Starts a file on `out`. `threads` defaults to a share of the CPUs.
    pub fn new(mut out: W, meta: &Meta, chunk_size: usize, threads: Option<usize>) -> io::Result<VgkWriter<W>> {
        let text = meta.encode();
        let mut h = Vec::with_capacity(20 + text.len());
        h.extend_from_slice(MAGIC);
        h.extend_from_slice(&VERSION.to_le_bytes());
        h.extend_from_slice(&(text.len() as u32).to_le_bytes());
        h.extend_from_slice(text.as_bytes());
        h.extend_from_slice(&crc32(text.as_bytes()).to_le_bytes());
        out.write_all(&h)?;
        let unit = meta.unit_size;
        let pool = OrderedPool::new(threads, move |job: Job| {
            let parts = tile_parts(job.first, &job.data, unit);
            (job.data.len() as u64, encode_chunk(job.first, &job.data, job.compress), parts)
        });
        let chunk_size = (chunk_size / unit).max(1) * unit;
        Ok(VgkWriter {
            out,
            pos: h.len() as u64,
            unit_size: unit,
            chunk: Vec::with_capacity(chunk_size),
            chunk_size,
            samples: 0,
            chunk_first: 0,
            pool,
            index: Vec::new(),
            raw_written: 0,
            stored_chunks: 0,
            on_chunk: None,
            overview: None,
            tiles: Vec::new(),
            tile_acc: None,
        })
    }

    /// Samples written so far.
    pub fn samples(&self) -> u64 {
        self.samples
    }

    /// Bytes written to the output so far.
    pub fn bytes_written(&self) -> u64 {
        self.pos
    }

    /// Raw sample bytes already committed to the output.
    pub fn raw_written(&self) -> u64 {
        self.raw_written
    }

    /// Chunks stored uncompressed because compression fell behind.
    pub fn stored_chunks(&self) -> u64 {
        self.stored_chunks
    }

    /// Appends packed samples (`unit_size` bytes each).
    pub fn write(&mut self, mut data: &[u8]) -> io::Result<()> {
        debug_assert_eq!(data.len() % self.unit_size, 0);
        self.samples += (data.len() / self.unit_size) as u64;
        while !data.is_empty() {
            let n = (self.chunk_size - self.chunk.len()).min(data.len());
            self.chunk.extend_from_slice(&data[..n]);
            data = &data[n..];
            if self.chunk.len() == self.chunk_size {
                self.submit();
            }
        }
        self.collect(false)
    }

    fn submit(&mut self) {
        if self.chunk.is_empty() {
            return;
        }
        let data = std::mem::replace(&mut self.chunk, Vec::with_capacity(self.chunk_size));
        let first = self.chunk_first;
        self.chunk_first += (data.len() / self.unit_size) as u64;
        // When compression cannot keep up, store chunks as they are rather
        // than stall the capture.
        let compress = self.pool.in_flight() < MAX_BACKLOG;
        if !compress {
            self.stored_chunks += 1;
        }
        self.pool.submit(Job { first, data, compress });
    }

    /// Registers a callback told about every sample chunk once it is on
    /// disk (the output is flushed first), so readers can follow a capture
    /// while it is being written.
    pub(crate) fn set_on_chunk(&mut self, f: impl FnMut(ChunkRef) + Send + 'static) {
        self.on_chunk = Some(Box::new(f));
    }

    /// Sets the overview tiles written by [`VgkWriter::finish`] (by default,
    /// the writer computes them from the samples).
    pub(crate) fn set_overview(&mut self, tile: u64, tiles: &[crate::store::Tile]) {
        let mut p = Vec::with_capacity(16 + tiles.len() * 8);
        p.extend_from_slice(&tile.to_le_bytes());
        p.extend_from_slice(&(tiles.len() as u64).to_le_bytes());
        for t in tiles {
            p.extend_from_slice(&t.first.to_le_bytes());
            p.extend_from_slice(&t.changed.to_le_bytes());
        }
        self.overview = Some(p);
    }

    /// Writes finished chunks in order; with `all`, waits for every one.
    fn collect(&mut self, all: bool) -> io::Result<()> {
        while let Some((raw, bytes, parts)) = self.pool.next(all) {
            for (first, or, and, count) in parts {
                let acc = self.tile_acc.get_or_insert((first, or, and, 0));
                acc.1 |= or;
                acc.2 &= and;
                acc.3 += count;
                if acc.3 == crate::store::TILE {
                    self.tiles.push(crate::store::Tile {
                        first: acc.0,
                        changed: acc.1 ^ acc.2,
                    });
                    self.tile_acc = None;
                }
            }
            let first = u64::from_le_bytes(bytes[8..16].try_into().unwrap());
            let offset = self.pos;
            self.index.push((offset, first));
            self.out.write_all(&bytes)?;
            self.pos += bytes.len() as u64;
            self.raw_written += raw;
            if let Some(f) = &mut self.on_chunk {
                self.out.flush()?;
                f(ChunkRef {
                    offset,
                    first,
                    samples: raw / self.unit_size as u64,
                });
            }
        }
        Ok(())
    }

    /// Flushes everything, writes the index and footer, and returns the
    /// underlying writer.
    pub fn finish(mut self) -> io::Result<W> {
        self.submit();
        self.collect(true)?;
        if self.overview.is_none() {
            let mut tiles = std::mem::take(&mut self.tiles);
            if let Some((first, or, and, _)) = self.tile_acc.take() {
                tiles.push(crate::store::Tile { first, changed: or ^ and });
            }
            self.set_overview(crate::store::TILE, &tiles);
        }
        if let Some(ov) = self.overview.take() {
            let packed = compress_to_vec_with::<Zstd>(&ov, EncoderConfig { level: 3 })
                .ok()
                .filter(|p| p.len() < ov.len());
            let (codec, payload) = match &packed {
                Some(p) => (Codec::Zstd, p.as_slice()),
                None => (Codec::Store, ov.as_slice()),
            };
            let h = chunk_header(kind::OVERVIEW, codec, 0, ov.len(), payload.len(), crc32(&ov));
            self.out.write_all(&h)?;
            self.out.write_all(payload)?;
            self.pos += (h.len() + payload.len()) as u64;
        }
        let mut payload = Vec::with_capacity(self.index.len() * 16);
        for (off, first) in &self.index {
            payload.extend_from_slice(&off.to_le_bytes());
            payload.extend_from_slice(&first.to_le_bytes());
        }
        let index_at = self.pos;
        let h = chunk_header(
            kind::INDEX,
            Codec::Store,
            self.samples,
            payload.len(),
            payload.len(),
            crc32(&payload),
        );
        self.out.write_all(&h)?;
        self.out.write_all(&payload)?;
        self.out.write_all(FOOTER_MAGIC)?;
        self.out.write_all(&index_at.to_le_bytes())?;
        self.pos += (h.len() + payload.len() + 16) as u64;
        self.out.flush()?;
        Ok(self.out)
    }
}

/// Location of a sample chunk in a `.vgk` file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChunkRef {
    /// File offset of the chunk header.
    pub offset: u64,
    /// Index of the chunk's first sample.
    pub first: u64,
    /// Number of samples in the chunk.
    pub samples: u64,
}

/// Decodes a chunk payload given its header, checking length and CRC.
fn decode_payload(h: &[u8; CHUNK_HEADER], payload: Vec<u8>) -> io::Result<Vec<u8>> {
    let first = u64::from_le_bytes(h[8..16].try_into().unwrap());
    let raw_len = u32::from_le_bytes(h[16..20].try_into().unwrap()) as usize;
    let crc = u32::from_le_bytes(h[24..28].try_into().unwrap());
    let raw = match h[5] {
        0 => payload,
        1 => decompress_to_vec_capped::<Zstd>(&payload, raw_len as u64).map_err(|e| invalid(format!("chunk at sample {first}: {e:?}")))?,
        c => return Err(invalid(format!("unknown codec {c}"))),
    };
    if raw.len() != raw_len || crc32(&raw) != crc {
        return Err(invalid(format!("chunk at sample {first}: checksum mismatch")));
    }
    Ok(raw)
}

fn read_header(f: &mut impl Read) -> io::Result<Option<[u8; CHUNK_HEADER]>> {
    let mut h = [0u8; CHUNK_HEADER];
    match f.read_exact(&mut h) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    if &h[..4] != CHUNK_MAGIC || crc32(&h[..28]) != u32::from_le_bytes(h[28..32].try_into().unwrap()) {
        return Ok(None);
    }
    Ok(Some(h))
}

/// Reads and decodes the sample chunk at `offset` (random access).
pub(crate) fn read_chunk_at(f: &mut File, offset: u64) -> io::Result<Vec<u8>> {
    f.seek(SeekFrom::Start(offset))?;
    let h = read_header(f)?.ok_or_else(|| invalid(format!("no chunk at offset {offset}")))?;
    let stored = u32::from_le_bytes(h[20..24].try_into().unwrap()) as usize;
    let mut payload = vec![0u8; stored];
    f.read_exact(&mut payload)?;
    decode_payload(&h, payload)
}

/// Structure of a `.vgk` file, found by walking chunk headers (payloads
/// are skipped, except the overview's).
#[derive(Clone, Debug)]
pub(crate) struct Scan {
    /// Capture metadata.
    pub meta: Meta,
    /// Sample chunks in order.
    pub chunks: Vec<ChunkRef>,
    /// Overview tiles, when the file has them: (tile size, tiles).
    pub overview: Option<(u64, Vec<crate::store::Tile>)>,
    /// Total samples in the readable chunks.
    pub samples: u64,
}

/// Scans a `.vgk` file's chunk headers.
pub(crate) fn scan(path: impl AsRef<Path>) -> io::Result<Scan> {
    let mut f = BufReader::with_capacity(1 << 16, File::open(path)?);
    let meta = VgkReader::new(&mut f)?.meta().clone();
    let unit = meta.unit_size as u64;
    let mut pos = f.stream_position()?;
    let mut chunks = Vec::new();
    let mut overview = None;
    let mut samples = 0;
    while let Some(h) = read_header(&mut f)? {
        let first = u64::from_le_bytes(h[8..16].try_into().unwrap());
        let raw_len = u32::from_le_bytes(h[16..20].try_into().unwrap()) as u64;
        let stored = u32::from_le_bytes(h[20..24].try_into().unwrap()) as u64;
        let end = pos + CHUNK_HEADER as u64 + stored;
        match h[4] {
            kind::SAMPLES => {
                if f.get_ref().metadata()?.len() < end {
                    break; // cut short
                }
                chunks.push(ChunkRef {
                    offset: pos,
                    first,
                    samples: raw_len / unit,
                });
                samples = first + raw_len / unit;
                f.seek_relative(stored as i64)?;
            }
            kind::OVERVIEW => {
                let mut payload = vec![0u8; stored as usize];
                f.read_exact(&mut payload)?;
                let ov = decode_payload(&h, payload)?;
                if ov.len() >= 16 {
                    let tile = u64::from_le_bytes(ov[0..8].try_into().unwrap());
                    let n = u64::from_le_bytes(ov[8..16].try_into().unwrap()) as usize;
                    let tiles = ov[16..]
                        .as_chunks::<8>()
                        .0
                        .iter()
                        .take(n)
                        .map(|c| crate::store::Tile {
                            first: u32::from_le_bytes(c[0..4].try_into().unwrap()),
                            changed: u32::from_le_bytes(c[4..8].try_into().unwrap()),
                        })
                        .collect();
                    overview = Some((tile, tiles));
                }
            }
            kind::INDEX => break,
            _ => f.seek_relative(stored as i64)?,
        }
        pos = end;
    }
    Ok(Scan {
        meta,
        chunks,
        overview,
        samples,
    })
}

/// Reads a `.vgk` file sequentially; also a [`Source`] for replaying captures.
pub struct VgkReader<R: Read> {
    input: R,
    meta: Meta,
    next_sample: u64,
    /// The file ended without an index (e.g. the capture was interrupted).
    pub truncated: bool,
    /// Total samples according to the index, once reached.
    pub total: Option<u64>,
    finished: bool,
}

impl VgkReader<BufReader<File>> {
    /// Opens a file and reads its header.
    pub fn open(path: impl AsRef<Path>) -> io::Result<VgkReader<BufReader<File>>> {
        VgkReader::new(BufReader::with_capacity(1 << 20, File::open(path)?))
    }
}

/// Reads the sample count from the index without scanning the file.
pub fn total_samples(path: impl AsRef<Path>) -> io::Result<Option<u64>> {
    let mut f = File::open(path)?;
    let len = f.seek(SeekFrom::End(0))?;
    if len < 16 + CHUNK_HEADER as u64 {
        return Ok(None);
    }
    f.seek(SeekFrom::End(-16))?;
    let mut foot = [0u8; 16];
    f.read_exact(&mut foot)?;
    if &foot[..8] != FOOTER_MAGIC {
        return Ok(None);
    }
    let at = u64::from_le_bytes(foot[8..].try_into().unwrap());
    f.seek(SeekFrom::Start(at))?;
    let mut h = [0u8; CHUNK_HEADER];
    f.read_exact(&mut h)?;
    if &h[..4] != CHUNK_MAGIC || h[4] != kind::INDEX {
        return Ok(None);
    }
    Ok(Some(u64::from_le_bytes(h[8..16].try_into().unwrap())))
}

impl<R: Read> VgkReader<R> {
    /// Reads the header from `input`.
    pub fn new(mut input: R) -> io::Result<VgkReader<R>> {
        let mut h = [0u8; 16];
        input.read_exact(&mut h)?;
        if &h[..8] != MAGIC {
            return Err(invalid("not a visgrok capture".into()));
        }
        let version = u32::from_le_bytes(h[8..12].try_into().unwrap());
        if version != VERSION {
            return Err(invalid(format!("unsupported version {version}")));
        }
        let len = u32::from_le_bytes(h[12..16].try_into().unwrap()) as usize;
        if len > 1 << 20 {
            return Err(invalid("metadata too large".into()));
        }
        let mut text = vec![0u8; len + 4];
        input.read_exact(&mut text)?;
        let crc = u32::from_le_bytes(text[len..].try_into().unwrap());
        text.truncate(len);
        if crc32(&text) != crc {
            return Err(invalid("metadata checksum mismatch".into()));
        }
        let text = String::from_utf8(text).map_err(|_| invalid("metadata is not UTF-8".into()))?;
        Ok(VgkReader {
            input,
            meta: Meta::decode(&text)?,
            next_sample: 0,
            truncated: false,
            total: None,
            finished: false,
        })
    }

    /// The capture's metadata.
    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    /// Reads the next block of samples, or `None` at the end.
    pub fn read_block(&mut self) -> io::Result<Option<Block>> {
        while !self.finished {
            let mut h = [0u8; CHUNK_HEADER];
            if let Err(e) = self.input.read_exact(&mut h) {
                if e.kind() == io::ErrorKind::UnexpectedEof {
                    self.truncated = true;
                    self.finished = true;
                    return Ok(None);
                }
                return Err(e);
            }
            if &h[..4] != CHUNK_MAGIC || crc32(&h[..28]) != u32::from_le_bytes(h[28..32].try_into().unwrap()) {
                // Garbage at the end of an interrupted file.
                self.truncated = true;
                self.finished = true;
                return Ok(None);
            }
            let first = u64::from_le_bytes(h[8..16].try_into().unwrap());
            let stored = u32::from_le_bytes(h[20..24].try_into().unwrap()) as usize;
            let mut payload = vec![0u8; stored];
            if let Err(e) = self.input.read_exact(&mut payload) {
                if e.kind() == io::ErrorKind::UnexpectedEof {
                    self.truncated = true;
                    self.finished = true;
                    return Ok(None);
                }
                return Err(e);
            }
            match h[4] {
                kind::SAMPLES => {}
                kind::INDEX => {
                    // The index is always last; only the footer follows.
                    self.total = Some(first);
                    self.finished = true;
                    return Ok(None);
                }
                _ => continue, // unknown kinds are skippable by design
            }
            let raw = decode_payload(&h, payload)?;
            if first != self.next_sample {
                return Err(invalid(format!("chunk starts at sample {first}, expected {}", self.next_sample)));
            }
            let b = Block::new(first, self.meta.unit_size, raw);
            self.next_sample = b.end();
            return Ok(Some(b));
        }
        Ok(None)
    }
}

impl<R: Read + Send> Source for VgkReader<R> {
    fn info(&self) -> CaptureInfo {
        CaptureInfo {
            device: self.meta.device.clone(),
            channels: self.meta.channels,
            samplerate: self.meta.samplerate,
            unit_size: self.meta.unit_size,
            names: self.meta.names.clone(),
        }
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        self.read_block()
    }

    fn stop(&mut self) {
        self.finished = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The writer computes the overview itself, also with chunks that do
    /// not line up with tiles.
    #[test]
    fn overview_from_samples() {
        let meta = Meta {
            device: "test".into(),
            channels: 8,
            samplerate: 1_000_000,
            unit_size: 1,
            names: (0..8).map(|i| format!("D{i}")).collect(),
            started_ms: 1,
            extra: Vec::new(),
        };
        // ch0 toggles every 3000 samples; ch1 only in 10000..10010.
        let data: Vec<u8> = (0..30_001u32)
            .map(|i| ((i / 3000) & 1) as u8 | (((10_000..10_010).contains(&i)) as u8) << 1)
            .collect();
        let mut w = VgkWriter::new(Vec::new(), &meta, 5000, Some(2)).unwrap();
        w.write(&data).unwrap();
        let buf = w.finish().unwrap();
        let dir = std::env::temp_dir().join(format!("vgk-ov-{}", std::process::id()));
        std::fs::write(&dir, &buf).unwrap();
        let sc = scan(&dir).unwrap();
        std::fs::remove_file(&dir).ok();
        let (tile, tiles) = sc.overview.expect("overview");
        assert_eq!(tile, crate::store::TILE);
        let want: Vec<crate::store::Tile> = data
            .chunks(tile as usize)
            .map(|c| {
                let (or, and) = c.iter().fold((0u32, 0xffu32), |(o, a), &s| (o | s as u32, a & s as u32));
                crate::store::Tile {
                    first: c[0] as u32,
                    changed: or ^ and,
                }
            })
            .collect();
        assert_eq!(tiles, want);
    }

    #[test]
    fn roundtrip() {
        let meta = Meta {
            device: "test".into(),
            channels: 16,
            samplerate: 1_000_000,
            unit_size: 2,
            names: (0..16).map(|i| format!("D{i}")).collect(),
            started_ms: 1,
            extra: vec![("threshold".into(), "1.65".into())],
        };
        let mut w = VgkWriter::new(Vec::new(), &meta, 16384, Some(3)).unwrap();
        let data: Vec<u8> = (0..50_000u32).flat_map(|i| ((i / 700) as u16).to_le_bytes()).collect();
        for c in data.chunks(3334 * 2) {
            w.write(c).unwrap();
        }
        let buf = w.finish().unwrap();
        assert!(buf.len() < data.len() / 2, "{} vs {}", buf.len(), data.len());

        let mut r = VgkReader::new(&buf[..]).unwrap();
        assert_eq!(r.meta(), &meta);
        let mut out = Vec::new();
        while let Some(b) = r.read_block().unwrap() {
            assert_eq!(b.start as usize, out.len() / 2);
            out.extend(b.data);
        }
        assert_eq!(out, data);
        assert!(!r.truncated);
        assert_eq!(r.total, Some(50_000));
    }

    #[test]
    fn truncated_file_is_readable() {
        let meta = Meta {
            device: "t".into(),
            channels: 8,
            samplerate: 1000,
            unit_size: 1,
            ..Default::default()
        };
        let mut w = VgkWriter::new(Vec::new(), &meta, 100, Some(1)).unwrap();
        w.write(&(0..1000u32).map(|i| (i / 3) as u8).collect::<Vec<_>>()).unwrap();
        let buf = w.finish().unwrap();
        let cut = &buf[..buf.len() / 2];
        let mut r = VgkReader::new(cut).unwrap();
        let mut n = 0;
        while let Some(b) = r.read_block().unwrap() {
            n += b.len();
        }
        assert!(n > 0 && n < 1000);
        assert!(r.truncated);
    }
}
