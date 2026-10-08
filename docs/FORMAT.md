# The `.vgk` capture format

visgrok records to its own format by default (`-o capture.vgk`; use a `.sr`
name for a sigrok session instead). It is designed for continuous capture at
up to 400 MB/s:

- **Compressed:** each chunk (4 MiB of raw samples) is compressed independently
  with zstd level 1 ([compcol](https://github.com/KarpelesLab/compcol)) on a pool
  of worker threads. If compression ever falls behind, chunks are stored
  uncompressed instead, so recording never stalls the capture.
- **Streamable and crash-tolerant:** chunks are self-describing with CRCs; a file
  cut short (crash, unplug, power loss) is readable up to its last complete
  chunk. The trailing index only speeds up seeking.
- **Extensible:** unknown chunk kinds are skipped by readers.

Measured on a SLogic16 U3 (16 channels at 200 MHz = 400 MB/s, 5 s):

| Signal | Raw | `.vgk` | Ratio |
|---|---|---|---|
| Idle inputs | 2 GB | 80 kB | ~25 000× |
| Emulation pattern (every sample changes) | 2 GB | 62 MB | 32× |
| 4 channels at 800 MHz, emulation pattern | 4 GB | 539 kB | ~7 400× |

## Layout

All integers are little endian.

```text
file    = header chunk* [index footer]
header  = "VISGROK\0" u32:version(1) u32:meta_len meta u32:crc32(meta)
meta    = UTF-8 "key=value\n" lines
chunk   = "VGKC" u8:kind u8:codec u16:0 u64:first_sample
          u32:raw_len u32:stored_len u32:crc32(raw) u32:crc32(chunk header[0..28])
          payload[stored_len]
footer  = "VGKEND\0\0" u64:offset of the index chunk
```

Metadata keys: `device`, `channels`, `samplerate` (Hz), `unit_size` (1 or 2),
`names` (comma separated), `started_ms` (Unix epoch), plus optional ones such as
`threshold_v` and `pattern`.

Chunk kinds:

| kind | meaning | payload |
|---|---|---|
| `1` | samples | `raw_len / unit_size` consecutive samples, channel *n* = bit *n* of each little-endian unit |
| `0xFE` | index (last chunk) | `(u64 offset, u64 first_sample)` per sample chunk; `first_sample` holds the total sample count |

Codecs: `0` stored, `1` zstd (one frame per chunk).

## Tools

```sh
visgrok -i capture.vgk --headless --auto     # replay through the analyzers
visgrok -i capture.vgk -o capture.sr --headless   # convert for PulseView
cargo run --release --example selftest -- capture.vgk   # verify an emulation-pattern capture
```
