//! Reader for sigrok session files (`.sr`): a zip archive with a `metadata`
//! INI file and logic data in `logic-1-<n>` chunks (stored or deflated).

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::Path;

use compcol::deflate::Deflate;
use compcol::vec::decompress_to_vec_capped;

use crate::block::Block;
use crate::source::{CaptureInfo, Source};

fn invalid(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

struct ZipEntry {
    name: String,
    method: u16,
    stored: u64,
    raw: u64,
    offset: u64,
}

fn u16le(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn u32le(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

fn u64le(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(b[at..at + 8].try_into().unwrap())
}

/// Lists the entries of a zip archive (zip64 aware).
fn central_directory(f: &mut File) -> io::Result<Vec<ZipEntry>> {
    let len = f.seek(SeekFrom::End(0))?;
    let tail = len.min(65_557);
    f.seek(SeekFrom::Start(len - tail))?;
    let mut buf = vec![0u8; tail as usize];
    f.read_exact(&mut buf)?;
    let eocd = (0..buf.len().saturating_sub(21))
        .rev()
        .find(|&i| u32le(&buf, i) == 0x0605_4b50)
        .ok_or_else(|| invalid("not a zip archive (no end of central directory)"))?;
    let mut count = u16le(&buf, eocd + 10) as u64;
    let mut cd_size = u32le(&buf, eocd + 12) as u64;
    let mut cd_off = u32le(&buf, eocd + 16) as u64;
    if (count == 0xffff || cd_off == u32::MAX as u64) && eocd >= 20 && u32le(&buf, eocd - 20) == 0x0706_4b50 {
        let at = u64le(&buf, eocd - 20 + 8);
        f.seek(SeekFrom::Start(at))?;
        let mut z = [0u8; 56];
        f.read_exact(&mut z)?;
        if u32le(&z, 0) != 0x0606_4b50 {
            return Err(invalid("bad zip64 end of central directory"));
        }
        count = u64le(&z, 32);
        cd_size = u64le(&z, 40);
        cd_off = u64le(&z, 48);
    }
    f.seek(SeekFrom::Start(cd_off))?;
    let mut cd = vec![0u8; cd_size as usize];
    f.read_exact(&mut cd)?;
    let mut out = Vec::with_capacity(count as usize);
    let mut p = 0;
    for _ in 0..count {
        if p + 46 > cd.len() || u32le(&cd, p) != 0x0201_4b50 {
            return Err(invalid("corrupt zip central directory"));
        }
        let method = u16le(&cd, p + 10);
        let mut stored = u32le(&cd, p + 20) as u64;
        let mut raw = u32le(&cd, p + 24) as u64;
        let name_len = u16le(&cd, p + 28) as usize;
        let extra_len = u16le(&cd, p + 30) as usize;
        let comment_len = u16le(&cd, p + 32) as usize;
        let mut offset = u32le(&cd, p + 42) as u64;
        let name = String::from_utf8_lossy(&cd[p + 46..p + 46 + name_len]).into_owned();
        // Zip64 extended information: present fields replace 0xffffffff ones.
        let mut e = p + 46 + name_len;
        let end = e + extra_len;
        while e + 4 <= end {
            let (id, size) = (u16le(&cd, e), u16le(&cd, e + 2) as usize);
            if id == 1 {
                let mut q = e + 4;
                if raw == u32::MAX as u64 {
                    raw = u64le(&cd, q);
                    q += 8;
                }
                if stored == u32::MAX as u64 {
                    stored = u64le(&cd, q);
                    q += 8;
                }
                if offset == u32::MAX as u64 {
                    offset = u64le(&cd, q);
                }
            }
            e += 4 + size;
        }
        out.push(ZipEntry {
            name,
            method,
            stored,
            raw,
            offset,
        });
        p = end + comment_len;
    }
    Ok(out)
}

fn read_entry(f: &mut File, e: &ZipEntry) -> io::Result<Vec<u8>> {
    f.seek(SeekFrom::Start(e.offset))?;
    let mut h = [0u8; 30];
    f.read_exact(&mut h)?;
    if u32le(&h, 0) != 0x0403_4b50 {
        return Err(invalid(format!("{}: bad local header", e.name)));
    }
    let skip = u16le(&h, 26) as i64 + u16le(&h, 28) as i64;
    f.seek(SeekFrom::Current(skip))?;
    let mut data = vec![0u8; e.stored as usize];
    f.read_exact(&mut data)?;
    match e.method {
        0 => Ok(data),
        8 => decompress_to_vec_capped::<Deflate>(&data, e.raw).map_err(|err| invalid(format!("{}: {err:?}", e.name))),
        m => Err(invalid(format!("{}: unsupported zip method {m}", e.name))),
    }
}

/// Parses sigrok's size strings: `"50 MHz"`, `"1500 kHz"`, `"24000000"`.
pub fn parse_samplerate(s: &str) -> Option<u64> {
    let s = s.trim().trim_end_matches(['H', 'h', 'z', 'Z']).trim();
    let (num, mul) = match s.chars().last()? {
        'k' | 'K' => (&s[..s.len() - 1], 1e3),
        'M' => (&s[..s.len() - 1], 1e6),
        'G' | 'g' => (&s[..s.len() - 1], 1e9),
        _ => (s, 1.0),
    };
    let v: f64 = num.trim().parse().ok()?;
    Some((v * mul).round() as u64)
}

/// Streams the logic data of a `.sr` file.
pub struct SrReader {
    file: File,
    info: CaptureInfo,
    chunks: Vec<ZipEntry>,
    next: usize,
    pos: u64,
    stopped: bool,
}

impl SrReader {
    /// Opens a sigrok session file.
    pub fn open(path: impl AsRef<Path>) -> io::Result<SrReader> {
        let mut file = File::open(path)?;
        let entries = central_directory(&mut file)?;
        let meta_entry = entries
            .iter()
            .find(|e| e.name == "metadata")
            .ok_or_else(|| invalid("no metadata in .sr"))?;
        let meta = String::from_utf8_lossy(&read_entry(&mut file, meta_entry)?).into_owned();
        let mut capturefile = "logic-1".to_string();
        let (mut channels, mut rate, mut unit) = (0usize, 0u64, 0usize);
        let mut names: Vec<(usize, String)> = Vec::new();
        let mut in_device = false;
        for line in meta.lines() {
            let line = line.trim();
            if line.starts_with('[') {
                // Only the first device section is read.
                if in_device && line.starts_with("[device") {
                    break;
                }
                in_device = line.starts_with("[device");
                continue;
            }
            let Some((k, v)) = line.split_once('=') else { continue };
            let (k, v) = (k.trim(), v.trim());
            match k {
                "capturefile" => capturefile = v.to_string(),
                "total probes" => channels = v.parse().unwrap_or(0),
                "samplerate" => rate = parse_samplerate(v).unwrap_or(0),
                "unitsize" => unit = v.parse().unwrap_or(0),
                _ => {
                    if let Some(n) = k.strip_prefix("probe").and_then(|n| n.parse::<usize>().ok()) {
                        names.push((n, v.to_string()));
                    }
                }
            }
        }
        if channels == 0 || rate == 0 {
            return Err(invalid("incomplete .sr metadata (channels or samplerate missing)"));
        }
        if channels > 32 {
            return Err(invalid(format!("{channels} channels: at most 32 are supported")));
        }
        if unit == 0 {
            unit = crate::block::unit_size_for(channels);
        }
        if !matches!(unit, 1 | 2 | 4) {
            return Err(invalid(format!("unsupported unitsize {unit}")));
        }
        let mut chunk_names: Vec<(u64, usize)> = entries
            .iter()
            .enumerate()
            .filter_map(|(i, e)| {
                if e.name == capturefile {
                    return Some((0, i));
                }
                let n = e.name.strip_prefix(&capturefile)?.strip_prefix('-')?.parse().ok()?;
                Some((n, i))
            })
            .collect();
        chunk_names.sort();
        let mut by_index: Vec<Option<ZipEntry>> = entries.into_iter().map(Some).collect();
        let chunks = chunk_names.into_iter().filter_map(|(_, i)| by_index[i].take()).collect();
        let mut n = vec![String::new(); channels];
        for (i, name) in names {
            if (1..=channels).contains(&i) {
                n[i - 1] = name;
            }
        }
        Ok(SrReader {
            file,
            info: CaptureInfo {
                device: "sigrok session".into(),
                channels,
                samplerate: rate,
                unit_size: unit,
                names: n,
            },
            chunks,
            next: 0,
            pos: 0,
            stopped: false,
        })
    }
}

impl Source for SrReader {
    fn info(&self) -> CaptureInfo {
        self.info.clone()
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        while !self.stopped && self.next < self.chunks.len() {
            let i = self.next;
            self.next += 1;
            let mut data = read_entry(&mut self.file, &self.chunks[i])?;
            data.truncate(data.len() / self.info.unit_size * self.info.unit_size);
            if data.is_empty() {
                continue;
            }
            let b = Block::new(self.pos, self.info.unit_size, data);
            self.pos = b.end();
            return Ok(Some(b));
        }
        Ok(None)
    }

    fn stop(&mut self) {
        self.stopped = true;
    }
}
