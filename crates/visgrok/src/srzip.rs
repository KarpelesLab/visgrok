//! Streaming writer for sigrok session files (`.sr`).
//!
//! A `.sr` file is a zip archive holding `version`, `metadata` and the logic
//! data split into `logic-1-<n>` chunks of raw samples. Chunks are buffered in
//! memory and written as stored (uncompressed) entries as soon as they are
//! full, so a capture of any length streams to disk with bounded memory.
//! Zip64 records are emitted when the archive grows past 4 GiB or 65535
//! entries.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Default chunk size, in bytes.
pub const DEFAULT_CHUNK: usize = 4 << 20;

struct Entry {
    name: String,
    crc: u32,
    size: u64,
    offset: u64,
}

/// Writes a sigrok session file incrementally.
pub struct SrZipWriter<W: Write> {
    out: W,
    pos: u64,
    entries: Vec<Entry>,
    chunk: Vec<u8>,
    chunk_size: usize,
    unit_size: usize,
    next_chunk: u32,
    samples: u64,
}

impl SrZipWriter<BufWriter<File>> {
    /// Creates `path` and writes the session header.
    pub fn create(
        path: impl AsRef<Path>,
        channels: &[String],
        samplerate: u64,
        unit_size: usize,
    ) -> io::Result<SrZipWriter<BufWriter<File>>> {
        let f = BufWriter::with_capacity(1 << 20, File::create(path)?);
        SrZipWriter::new(f, channels, samplerate, unit_size, DEFAULT_CHUNK)
    }
}

impl<W: Write> SrZipWriter<W> {
    /// Starts a session on `out`. `chunk_size` is rounded down to a multiple of
    /// `unit_size`.
    pub fn new(
        out: W,
        channels: &[String],
        samplerate: u64,
        unit_size: usize,
        chunk_size: usize,
    ) -> io::Result<SrZipWriter<W>> {
        let chunk_size = (chunk_size / unit_size).max(1) * unit_size;
        let mut w = SrZipWriter {
            out,
            pos: 0,
            entries: Vec::new(),
            chunk: Vec::with_capacity(chunk_size),
            chunk_size,
            unit_size,
            next_chunk: 1,
            samples: 0,
        };
        w.entry("version", b"2")?;
        let mut meta = String::from("[global]\nsigrok version=0.5.2\n\n[device 1]\ncapturefile=logic-1\n");
        meta += &format!("total probes={}\nsamplerate={}\ntotal analog=0\n", channels.len(), samplerate_string(samplerate));
        for (i, name) in channels.iter().enumerate() {
            meta += &format!("probe{}={}\n", i + 1, name);
        }
        meta += &format!("unitsize={unit_size}\n");
        w.entry("metadata", meta.as_bytes())?;
        Ok(w)
    }

    /// Number of samples written so far.
    pub fn samples(&self) -> u64 {
        self.samples
    }

    /// Bytes written to the underlying writer so far.
    pub fn bytes_written(&self) -> u64 {
        self.pos
    }

    /// Appends packed samples (`unit_size` bytes each).
    pub fn write(&mut self, mut data: &[u8]) -> io::Result<()> {
        debug_assert_eq!(data.len() % self.unit_size, 0);
        self.samples += (data.len() / self.unit_size) as u64;
        while !data.is_empty() {
            let room = self.chunk_size - self.chunk.len();
            let n = room.min(data.len());
            self.chunk.extend_from_slice(&data[..n]);
            data = &data[n..];
            if self.chunk.len() == self.chunk_size {
                self.flush_chunk()?;
            }
        }
        Ok(())
    }

    fn flush_chunk(&mut self) -> io::Result<()> {
        if self.chunk.is_empty() {
            return Ok(());
        }
        let name = format!("logic-1-{}", self.next_chunk);
        self.next_chunk += 1;
        let chunk = std::mem::take(&mut self.chunk);
        self.entry(&name, &chunk)?;
        self.chunk = chunk;
        self.chunk.clear();
        Ok(())
    }

    fn put(&mut self, b: &[u8]) -> io::Result<()> {
        self.out.write_all(b)?;
        self.pos += b.len() as u64;
        Ok(())
    }

    fn entry(&mut self, name: &str, data: &[u8]) -> io::Result<()> {
        let crc = crc32(data);
        let offset = self.pos;
        let mut h = Vec::with_capacity(30 + name.len());
        h.extend_from_slice(&0x04034b50u32.to_le_bytes());
        h.extend_from_slice(&20u16.to_le_bytes()); // version needed
        h.extend_from_slice(&0u16.to_le_bytes()); // flags
        h.extend_from_slice(&0u16.to_le_bytes()); // method: stored
        h.extend_from_slice(&0u16.to_le_bytes()); // time
        h.extend_from_slice(&0x21u16.to_le_bytes()); // date: 1980-01-01
        h.extend_from_slice(&crc.to_le_bytes());
        h.extend_from_slice(&(data.len() as u32).to_le_bytes());
        h.extend_from_slice(&(data.len() as u32).to_le_bytes());
        h.extend_from_slice(&(name.len() as u16).to_le_bytes());
        h.extend_from_slice(&0u16.to_le_bytes());
        h.extend_from_slice(name.as_bytes());
        self.put(&h)?;
        self.put(data)?;
        self.entries.push(Entry { name: name.to_string(), crc, size: data.len() as u64, offset });
        Ok(())
    }

    /// Flushes the last chunk and writes the zip central directory.
    pub fn finish(mut self) -> io::Result<W> {
        self.flush_chunk()?;
        let cd_start = self.pos;
        let entries = std::mem::take(&mut self.entries);
        for e in &entries {
            let big = e.offset >= u32::MAX as u64;
            let mut h = Vec::with_capacity(46 + e.name.len() + 12);
            h.extend_from_slice(&0x02014b50u32.to_le_bytes());
            h.extend_from_slice(&(if big { 45u16 } else { 20 }).to_le_bytes()); // made by
            h.extend_from_slice(&(if big { 45u16 } else { 20 }).to_le_bytes()); // needed
            h.extend_from_slice(&0u16.to_le_bytes());
            h.extend_from_slice(&0u16.to_le_bytes());
            h.extend_from_slice(&0u16.to_le_bytes());
            h.extend_from_slice(&0x21u16.to_le_bytes());
            h.extend_from_slice(&e.crc.to_le_bytes());
            h.extend_from_slice(&(e.size as u32).to_le_bytes());
            h.extend_from_slice(&(e.size as u32).to_le_bytes());
            h.extend_from_slice(&(e.name.len() as u16).to_le_bytes());
            h.extend_from_slice(&(if big { 12u16 } else { 0 }).to_le_bytes()); // extra
            h.extend_from_slice(&0u16.to_le_bytes()); // comment
            h.extend_from_slice(&0u16.to_le_bytes()); // disk
            h.extend_from_slice(&0u16.to_le_bytes()); // internal attrs
            h.extend_from_slice(&0u32.to_le_bytes()); // external attrs
            h.extend_from_slice(&(if big { u32::MAX } else { e.offset as u32 }).to_le_bytes());
            h.extend_from_slice(e.name.as_bytes());
            if big {
                h.extend_from_slice(&1u16.to_le_bytes()); // zip64 extra
                h.extend_from_slice(&8u16.to_le_bytes());
                h.extend_from_slice(&e.offset.to_le_bytes());
            }
            self.put(&h)?;
        }
        let cd_size = self.pos - cd_start;
        let count = entries.len() as u64;
        let zip64 = count >= 0xffff || cd_start >= u32::MAX as u64 || cd_size >= u32::MAX as u64;
        if zip64 {
            let eocd64 = self.pos;
            let mut h = Vec::with_capacity(76);
            h.extend_from_slice(&0x06064b50u32.to_le_bytes());
            h.extend_from_slice(&44u64.to_le_bytes());
            h.extend_from_slice(&45u16.to_le_bytes());
            h.extend_from_slice(&45u16.to_le_bytes());
            h.extend_from_slice(&0u32.to_le_bytes());
            h.extend_from_slice(&0u32.to_le_bytes());
            h.extend_from_slice(&count.to_le_bytes());
            h.extend_from_slice(&count.to_le_bytes());
            h.extend_from_slice(&cd_size.to_le_bytes());
            h.extend_from_slice(&cd_start.to_le_bytes());
            // Zip64 end of central directory locator.
            h.extend_from_slice(&0x07064b50u32.to_le_bytes());
            h.extend_from_slice(&0u32.to_le_bytes());
            h.extend_from_slice(&eocd64.to_le_bytes());
            h.extend_from_slice(&1u32.to_le_bytes());
            self.put(&h)?;
        }
        let mut h = Vec::with_capacity(22);
        h.extend_from_slice(&0x06054b50u32.to_le_bytes());
        h.extend_from_slice(&0u16.to_le_bytes());
        h.extend_from_slice(&0u16.to_le_bytes());
        let c16 = if zip64 { 0xffff } else { count as u16 };
        h.extend_from_slice(&c16.to_le_bytes());
        h.extend_from_slice(&c16.to_le_bytes());
        h.extend_from_slice(&(if zip64 { u32::MAX } else { cd_size as u32 }).to_le_bytes());
        h.extend_from_slice(&(if zip64 { u32::MAX } else { cd_start as u32 }).to_le_bytes());
        h.extend_from_slice(&0u16.to_le_bytes());
        self.put(&h)?;
        self.out.flush()?;
        Ok(self.out)
    }
}


/// Formats a sample rate the way sigrok does (`"24 MHz"`, `"1500 kHz"`).
pub fn samplerate_string(hz: u64) -> String {
    if hz >= 1_000_000_000 && hz % 1_000_000_000 == 0 {
        format!("{} GHz", hz / 1_000_000_000)
    } else if hz >= 1_000_000 && hz % 1_000_000 == 0 {
        format!("{} MHz", hz / 1_000_000)
    } else if hz >= 1_000 && hz % 1_000 == 0 {
        format!("{} kHz", hz / 1_000)
    } else {
        format!("{hz} Hz")
    }
}

const fn crc_tables() -> [[u32; 256]; 8] {
    let mut t = [[0u32; 256]; 8];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xedb88320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        t[0][i] = c;
        i += 1;
    }
    let mut i = 0;
    while i < 256 {
        let mut s = 1;
        while s < 8 {
            let prev = t[s - 1][i];
            t[s][i] = (prev >> 8) ^ t[0][(prev & 0xff) as usize];
            s += 1;
        }
        i += 1;
    }
    t
}

static CRC: [[u32; 256]; 8] = crc_tables();

/// CRC-32 (IEEE), slicing-by-8.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = !0u32;
    let mut chunks = data.chunks_exact(8);
    for b in &mut chunks {
        let lo = c ^ u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        c = CRC[7][(lo & 0xff) as usize]
            ^ CRC[6][(lo >> 8 & 0xff) as usize]
            ^ CRC[5][(lo >> 16 & 0xff) as usize]
            ^ CRC[4][(lo >> 24) as usize]
            ^ CRC[3][b[4] as usize]
            ^ CRC[2][b[5] as usize]
            ^ CRC[1][b[6] as usize]
            ^ CRC[0][b[7] as usize];
    }
    for &b in chunks.remainder() {
        c = CRC[0][((c ^ b as u32) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_vectors() {
        assert_eq!(crc32(b""), 0);
        assert_eq!(crc32(b"123456789"), 0xcbf43926);
        assert_eq!(crc32(b"The quick brown fox jumps over the lazy dog"), 0x414fa339);
    }

    #[test]
    fn rates() {
        assert_eq!(samplerate_string(24_000_000), "24 MHz");
        assert_eq!(samplerate_string(1_500_000), "1500 kHz");
        assert_eq!(samplerate_string(1), "1 Hz");
    }

    #[test]
    fn writes_valid_zip() {
        let ch: Vec<String> = (0..8).map(|i| format!("D{i}")).collect();
        let mut w = SrZipWriter::new(Vec::new(), &ch, 1_000_000, 1, 10).unwrap();
        w.write(&[1u8; 25]).unwrap();
        let buf = w.finish().unwrap();
        // version, metadata, 3 chunks
        assert_eq!(&buf[buf.len() - 22..buf.len() - 18], &0x06054b50u32.to_le_bytes());
        assert_eq!(u16::from_le_bytes([buf[buf.len() - 12], buf[buf.len() - 11]]), 5);
    }
}
