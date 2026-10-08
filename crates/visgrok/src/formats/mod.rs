//! Capture file formats: reading, writing and converting between them.
//!
//! | Format | Extension | Read | Write | Notes |
//! |---|---|---|---|---|
//! | visgrok | `.vgk` | ✓ | ✓ | zstd chunks, parallel, crash tolerant, best ratio |
//! | sigrok session | `.sr` | ✓ | ✓ | zip; chunks stored or deflated (PulseView, sigrok-cli) |
//! | Value Change Dump | `.vcd` | ✓ | ✓ | text, changes only (GTKWave, simulators) |
//! | raw samples | `.bin` | ✓ | ✓ | 1/2/4-byte little-endian units; reading needs rate and channels |

pub mod sr;
pub mod vcd;

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::block::{Block, unit_size_for};
use crate::source::{CaptureInfo, Source};
use crate::srzip::{SrCompression, SrZipWriter};
use crate::vgk::{Meta, VgkReader, VgkWriter};

/// A capture file format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// visgrok `.vgk`.
    Vgk,
    /// sigrok session `.sr`.
    Sr,
    /// Value Change Dump `.vcd`.
    Vcd,
    /// Raw samples `.bin`.
    Bin,
}

impl Format {
    /// Picks a format from a file extension.
    pub fn from_path(path: &Path) -> Option<Format> {
        match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
            "vgk" => Some(Format::Vgk),
            "sr" => Some(Format::Sr),
            "vcd" => Some(Format::Vcd),
            "bin" | "raw" => Some(Format::Bin),
            _ => None,
        }
    }

    /// Parses a format name (`vgk`, `sr`, `vcd`, `bin`).
    pub fn parse(s: &str) -> Option<Format> {
        Format::from_path(Path::new(&format!("x.{s}")))
    }

    /// Short name.
    pub fn name(self) -> &'static str {
        match self {
            Format::Vgk => "vgk",
            Format::Sr => "sr",
            Format::Vcd => "vcd",
            Format::Bin => "bin",
        }
    }
}

fn unknown(path: &Path) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        format!("{}: unknown format (use .vgk, .sr, .vcd or .bin)", path.display()),
    )
}

/// Options for reading captures.
#[derive(Clone, Debug, Default)]
pub struct ReadOptions {
    /// Sample rate, required for `.bin`, optional override for `.vcd`.
    pub samplerate: Option<u64>,
    /// Channel count, required for `.bin`.
    pub channels: Option<usize>,
    /// Force a format instead of using the extension.
    pub format: Option<Format>,
}

/// Opens a capture file of any supported format as a [`Source`].
pub fn open(path: &Path, opts: &ReadOptions) -> io::Result<Box<dyn Source>> {
    match opts.format.or_else(|| Format::from_path(path)).ok_or_else(|| unknown(path))? {
        Format::Vgk => Ok(Box::new(VgkReader::open(path)?)),
        Format::Sr => Ok(Box::new(sr::SrReader::open(path)?)),
        Format::Vcd => Ok(Box::new(vcd::VcdReader::open(path, opts.samplerate)?)),
        Format::Bin => {
            let (Some(samplerate), Some(channels)) = (opts.samplerate, opts.channels) else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "raw .bin input needs a sample rate and a channel count",
                ));
            };
            Ok(Box::new(BinReader::open(path, samplerate, channels)?))
        }
    }
}

/// Options for writing captures.
#[derive(Clone, Debug)]
pub struct WriteOptions {
    /// Chunk compression for `.sr` output.
    pub sr_compression: SrCompression,
    /// Extra metadata (`.vgk`).
    pub extra: Vec<(String, String)>,
    /// Force a format instead of using the extension.
    pub format: Option<Format>,
}

impl Default for WriteOptions {
    fn default() -> Self {
        WriteOptions {
            sr_compression: SrCompression::Store,
            extra: Vec::new(),
            format: None,
        }
    }
}

/// Something that stores sample data.
pub trait SampleWriter: Send {
    /// Appends packed samples.
    fn write(&mut self, data: &[u8]) -> io::Result<()>;
    /// Bytes written to the file so far.
    fn bytes_written(&self) -> u64;
    /// Raw sample bytes committed so far.
    fn raw_written(&self) -> u64;
    /// Completes the file.
    fn finish(self: Box<Self>) -> io::Result<()>;
}

impl SampleWriter for VgkWriter<BufWriter<File>> {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        VgkWriter::write(self, data)
    }
    fn bytes_written(&self) -> u64 {
        VgkWriter::bytes_written(self)
    }
    fn raw_written(&self) -> u64 {
        VgkWriter::raw_written(self)
    }
    fn finish(self: Box<Self>) -> io::Result<()> {
        VgkWriter::finish(*self).map(drop)
    }
}

impl SampleWriter for SrZipWriter<BufWriter<File>> {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        SrZipWriter::write(self, data)
    }
    fn bytes_written(&self) -> u64 {
        SrZipWriter::bytes_written(self)
    }
    fn raw_written(&self) -> u64 {
        SrZipWriter::raw_written(self)
    }
    fn finish(self: Box<Self>) -> io::Result<()> {
        SrZipWriter::finish(*self).map(drop)
    }
}

impl SampleWriter for vcd::VcdWriter<BufWriter<File>> {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        vcd::VcdWriter::write(self, data)
    }
    fn bytes_written(&self) -> u64 {
        vcd::VcdWriter::bytes_written(self)
    }
    fn raw_written(&self) -> u64 {
        vcd::VcdWriter::raw_written(self)
    }
    fn finish(self: Box<Self>) -> io::Result<()> {
        vcd::VcdWriter::finish(*self).map(drop)
    }
}

/// Raw sample writer.
pub struct BinWriter {
    out: BufWriter<File>,
    n: u64,
}

impl SampleWriter for BinWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        self.out.write_all(data)?;
        self.n += data.len() as u64;
        Ok(())
    }
    fn bytes_written(&self) -> u64 {
        self.n
    }
    fn raw_written(&self) -> u64 {
        self.n
    }
    fn finish(mut self: Box<Self>) -> io::Result<()> {
        self.out.flush()
    }
}

/// Raw sample reader.
pub struct BinReader {
    input: BufReader<File>,
    info: CaptureInfo,
    pos: u64,
    stopped: bool,
}

impl BinReader {
    /// Opens a raw file with `channels` channels at `samplerate`.
    pub fn open(path: &Path, samplerate: u64, channels: usize) -> io::Result<BinReader> {
        Ok(BinReader {
            input: BufReader::with_capacity(1 << 20, File::open(path)?),
            info: CaptureInfo {
                device: "raw".into(),
                channels,
                samplerate,
                unit_size: unit_size_for(channels),
                names: Vec::new(),
            },
            pos: 0,
            stopped: false,
        })
    }
}

impl Source for BinReader {
    fn info(&self) -> CaptureInfo {
        self.info.clone()
    }

    fn next_block(&mut self) -> io::Result<Option<Block>> {
        if self.stopped {
            return Ok(None);
        }
        let unit = self.info.unit_size;
        let mut buf = vec![0u8; (4 << 20) / unit * unit];
        let mut n = 0;
        while n < buf.len() {
            match self.input.read(&mut buf[n..])? {
                0 => break,
                k => n += k,
            }
        }
        buf.truncate(n / unit * unit);
        if buf.is_empty() {
            return Ok(None);
        }
        let b = Block::new(self.pos, unit, buf);
        self.pos = b.end();
        Ok(Some(b))
    }

    fn stop(&mut self) {
        self.stopped = true;
    }
}

/// Creates a capture file of any supported format.
pub fn create(path: &Path, info: &CaptureInfo, opts: &WriteOptions) -> io::Result<Box<dyn SampleWriter>> {
    Ok(match opts.format.or_else(|| Format::from_path(path)).unwrap_or(Format::Vgk) {
        Format::Vgk => {
            let mut meta = Meta::from_info(info);
            meta.extra = opts.extra.clone();
            Box::new(VgkWriter::create(path, &meta)?)
        }
        Format::Sr => Box::new(SrZipWriter::create_with(
            path,
            &info.all_names(),
            info.samplerate,
            info.unit_size,
            opts.sr_compression,
        )?),
        Format::Vcd => Box::new(vcd::VcdWriter::create(path, info)?),
        Format::Bin => Box::new(BinWriter {
            out: BufWriter::with_capacity(1 << 20, File::create(path)?),
            n: 0,
        }),
    })
}

/// Result of a conversion.
#[derive(Clone, Debug)]
pub struct Converted {
    /// Description of the input.
    pub info: CaptureInfo,
    /// Samples copied.
    pub samples: u64,
    /// Size of the output file.
    pub bytes: u64,
}

/// Copies every sample from `input` to `output`, converting formats.
/// `progress` is called with the sample count after each block.
pub fn convert(
    input: &Path,
    output: &Path,
    ropts: &ReadOptions,
    wopts: &WriteOptions,
    mut progress: impl FnMut(u64),
) -> io::Result<Converted> {
    let mut src = open(input, ropts)?;
    let info = src.info();
    let mut w = create(output, &info, wopts)?;
    let mut samples = 0;
    while let Some(b) = src.next_block()? {
        w.write(&b.data)?;
        samples = b.end();
        progress(samples);
    }
    let bytes = w.bytes_written();
    w.finish()?;
    let bytes = std::fs::metadata(output).map_or(bytes, |m| m.len());
    Ok(Converted { info, samples, bytes })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("visgrok-fmt-{}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d.join(name)
    }

    /// Three seconds' worth of a busy 16-channel pattern at a small rate.
    fn sample_data() -> (CaptureInfo, Vec<u8>) {
        let info = CaptureInfo {
            device: "test".into(),
            channels: 12,
            samplerate: 2_000_000,
            unit_size: 2,
            names: (0..12).map(|i| format!("sig{i}")).collect(),
        };
        let mut x = 0x2468u32;
        let data: Vec<u8> = (0..300_000u32)
            .flat_map(|i| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                let noise = if x.is_multiple_of(97) { 0x800 } else { 0 };
                let v = ((i / 3) & 0xff) as u16 | ((i / 1000) as u16 & 7) << 8 | noise;
                v.to_le_bytes()
            })
            .collect();
        (info, data)
    }

    fn write_all(path: &Path, info: &CaptureInfo, data: &[u8], opts: &WriteOptions) {
        let mut w = create(path, info, opts).unwrap();
        for c in data.chunks(77_778 * 2) {
            w.write(c).unwrap();
        }
        w.finish().unwrap();
    }

    fn read_all(path: &Path, opts: &ReadOptions) -> (CaptureInfo, Vec<u8>) {
        let mut src = open(path, opts).unwrap();
        let info = src.info();
        let mut out = Vec::new();
        while let Some(b) = src.next_block().unwrap() {
            assert_eq!(b.start as usize, out.len() / b.unit_size);
            out.extend(b.data);
        }
        (info, out)
    }

    #[test]
    fn roundtrip_every_format() {
        let (info, data) = sample_data();
        for (ext, comp) in [
            ("vgk", SrCompression::Store),
            ("sr", SrCompression::Store),
            ("sr", SrCompression::Deflate),
            ("vcd", SrCompression::Store),
            ("bin", SrCompression::Store),
        ] {
            let path = scratch(&format!("rt-{ext}-{comp:?}.{ext}"));
            let wopts = WriteOptions {
                sr_compression: comp,
                ..Default::default()
            };
            write_all(&path, &info, &data, &wopts);
            let ropts = ReadOptions {
                samplerate: Some(info.samplerate),
                channels: Some(info.channels),
                format: None,
            };
            let (rinfo, back) = read_all(&path, &ropts);
            assert_eq!(rinfo.samplerate, info.samplerate, "{ext}");
            assert_eq!(rinfo.channels, info.channels, "{ext}");
            assert_eq!(back.len(), data.len(), "{ext} {comp:?}");
            assert!(back == data, "{ext} {comp:?}: data differs");
            if ext != "bin" {
                assert_eq!(rinfo.names, info.names, "{ext}");
            }
        }
    }

    #[test]
    fn convert_chain() {
        // vgk -> sr (deflate) -> vcd -> bin -> vgk keeps every sample.
        let (info, data) = sample_data();
        let a = scratch("chain.vgk");
        write_all(&a, &info, &data, &WriteOptions::default());
        let b = scratch("chain.sr");
        let c = scratch("chain.vcd");
        let d = scratch("chain.bin");
        let e = scratch("chain2.vgk");
        let deflate = WriteOptions {
            sr_compression: SrCompression::Deflate,
            ..Default::default()
        };
        convert(&a, &b, &ReadOptions::default(), &deflate, |_| {}).unwrap();
        convert(&b, &c, &ReadOptions::default(), &WriteOptions::default(), |_| {}).unwrap();
        convert(&c, &d, &ReadOptions::default(), &WriteOptions::default(), |_| {}).unwrap();
        let raw = ReadOptions {
            samplerate: Some(info.samplerate),
            channels: Some(info.channels),
            format: None,
        };
        let r = convert(&d, &e, &raw, &WriteOptions::default(), |_| {}).unwrap();
        assert_eq!(r.samples, 300_000);
        let (_, back) = read_all(&e, &ReadOptions::default());
        assert!(back == data);
    }

    #[test]
    fn vcd_rate_from_timestamps() {
        // A foreign VCD without our samplerate comment: 10 ns timescale steps
        // of 50 → 500 ns per sample → 2 MHz.
        let p = scratch("foreign.vcd");
        std::fs::write(
            &p,
            "$timescale 10ns $end\n$scope module top $end\n$var wire 1 a clk $end\n$var wire 1 b data $end\n\
             $upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n0a\n1b\n$end\n#50 1a\n#100\n0a\n0b\n#250\n",
        )
        .unwrap();
        let (info, back) = read_all(&p, &ReadOptions::default());
        assert_eq!(info.samplerate, 2_000_000);
        assert_eq!(info.names, vec!["clk", "data"]);
        assert_eq!(back, vec![2, 3, 0, 0, 0]);
    }
}
