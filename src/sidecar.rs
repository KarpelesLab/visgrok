//! Capture sidecar files: what the signals in a capture *are*.
//!
//! A capture file holds samples; the sidecar next to it (`<capture>.json`,
//! e.g. `run1.vgk.json`, `navi.sr.json`) holds what makes them meaningful:
//! channel names, the role of each channel (which wires form which bus),
//! decoder settings, recording settings, bookmarks and free-form notes. It
//! is plain JSON so it can be read, diffed and edited by hand, it works for
//! any capture format, and it can change after the capture is closed (the
//! `.vgk` file itself is append-only).
//!
//! ```json
//! {
//!   "visgrok": 1,
//!   "capture": "run1.vgk",
//!   "channels": [
//!     {"name": "CLK8M", "role": ""},
//!     {"name": "UART", "role": "uart"},
//!     {"name": "SCLK", "role": "spi-clk"}
//!   ],
//!   "decoders": {"spi_protocol": "ssd1306", "spi_mode": null, "spi_cs_active_high": false,
//!                "uart_format": "auto", "uart_follow": true},
//!   "recording": {"device": "SLogic16 U3", "samplerate": 200000000, "threshold_v": 1.65},
//!   "bookmarks": [{"sample": 1031000000, "label": "CMD42"}],
//!   "notes": "head unit boot, card in slot 1"
//! }
//! ```
//!
//! Buses are described by their channels' roles (`spi-clk`, `spi-mosi`,
//! `spi-dc`, `spi-cs`, `sd-clk`, `sd-cmd`, `sd-dat0`..`3`, `i2c-scl:N`,
//! `uart`, ...); see [`crate::roles::Role::parse`].

use std::io;
use std::path::{Path, PathBuf};

use crate::analyzer::{DecoderOptions, SpiProtocol, parse_uart_format, uart_format_id};
use crate::json::{Json, Obj, jstr};
use crate::roles::Role;

/// Sidecar format version.
pub const VERSION: u32 = 1;

/// A bookmark in a capture.
#[derive(Clone, Debug, PartialEq)]
pub struct Bookmark {
    /// Sample index.
    pub sample: u64,
    /// Label.
    pub label: String,
}

/// Contents of a sidecar file.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sidecar {
    /// Capture file name this describes.
    pub capture: String,
    /// Channel names (empty: default `D<n>`).
    pub names: Vec<String>,
    /// Channel roles (`None`: unassigned).
    pub roles: Vec<Option<Role>>,
    /// Decoder settings.
    pub options: DecoderOptions,
    /// Device used, if recorded.
    pub device: Option<String>,
    /// Sample rate, if recorded.
    pub samplerate: Option<u64>,
    /// Input threshold in volts, if set.
    pub threshold: Option<f64>,
    /// Bookmarks, in sample order.
    pub bookmarks: Vec<Bookmark>,
    /// Free-form notes.
    pub notes: String,
}

/// Path of the sidecar for `capture` (`capture` + `.json`).
pub fn path_for(capture: &Path) -> PathBuf {
    let mut p = capture.as_os_str().to_owned();
    p.push(".json");
    PathBuf::from(p)
}

impl Sidecar {
    /// Loads the sidecar of `capture`, if it has one.
    pub fn load(capture: &Path) -> io::Result<Option<Sidecar>> {
        let p = path_for(capture);
        match std::fs::read_to_string(&p) {
            Ok(text) => Sidecar::parse(&text)
                .map(Some)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, format!("{}: not a visgrok sidecar", p.display()))),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Writes the sidecar of `capture` (atomically: temp file + rename).
    pub fn save(&self, capture: &Path) -> io::Result<()> {
        let p = path_for(capture);
        let tmp = p.with_extension("json.tmp");
        std::fs::write(&tmp, self.to_json())?;
        std::fs::rename(&tmp, &p)
    }

    /// Parses sidecar JSON.
    pub fn parse(text: &str) -> Option<Sidecar> {
        let j = Json::parse(text)?;
        j.get("visgrok")?.num()?;
        let mut s = Sidecar {
            capture: j.get("capture").and_then(Json::str).unwrap_or("").to_string(),
            ..Default::default()
        };
        if let Some(Json::Arr(chans)) = j.get("channels") {
            for c in chans {
                s.names.push(c.get("name").and_then(Json::str).unwrap_or("").to_string());
                let r = c.get("role").and_then(Json::str).unwrap_or("");
                s.roles.push(if r.is_empty() { None } else { Role::parse(r).ok() });
            }
        }
        if let Some(d) = j.get("decoders") {
            let o = &mut s.options;
            if let Some(p) = d.get("spi_protocol").and_then(Json::str).and_then(|p| SpiProtocol::parse(p).ok()) {
                o.spi_protocol = p;
            }
            o.spi_mode = d.get("spi_mode").and_then(Json::num).map(|m| m as u8 & 3);
            o.spi_cs_active_high = d.get("spi_cs_active_high").and_then(Json::bool).unwrap_or(false);
            if let Some(f) = d.get("uart_format").and_then(Json::str).and_then(|f| parse_uart_format(f).ok()) {
                o.uart_format = f;
            }
            o.uart_auto = d.get("uart_follow").and_then(Json::bool).unwrap_or(true);
        }
        if let Some(r) = j.get("recording") {
            s.device = r.get("device").and_then(Json::str).map(str::to_string);
            s.samplerate = r.get("samplerate").and_then(Json::num).map(|v| v as u64);
            s.threshold = r.get("threshold_v").and_then(Json::num);
        }
        if let Some(Json::Arr(b)) = j.get("bookmarks") {
            for m in b {
                if let Some(sample) = m.get("sample").and_then(Json::num) {
                    s.bookmarks.push(Bookmark {
                        sample: sample as u64,
                        label: m.get("label").and_then(Json::str).unwrap_or("").to_string(),
                    });
                }
            }
        }
        s.notes = j.get("notes").and_then(Json::str).unwrap_or("").to_string();
        Some(s)
    }

    /// Serializes to (readable, one channel per line) JSON.
    pub fn to_json(&self) -> String {
        let n = self.names.len().max(self.roles.len());
        let chans: Vec<String> = (0..n)
            .map(|i| {
                let mut c = Obj::new();
                c.str("name", self.names.get(i).map_or("", |s| s));
                c.str("role", &self.roles.get(i).cloned().flatten().map(|r| r.id()).unwrap_or_default());
                format!("    {}", c.finish())
            })
            .collect();
        let o = &self.options;
        let mut d = Obj::new();
        d.str("spi_protocol", &o.spi_protocol.id());
        d.raw("spi_mode", &o.spi_mode.map_or("null".to_string(), |m| m.to_string()));
        d.bool("spi_cs_active_high", o.spi_cs_active_high);
        d.str("uart_format", &uart_format_id(o.uart_format));
        d.bool("uart_follow", o.uart_auto);
        let mut r = Obj::new();
        if let Some(dev) = &self.device {
            r.str("device", dev);
        }
        if let Some(sr) = self.samplerate {
            r.num("samplerate", sr as f64);
        }
        if let Some(t) = self.threshold {
            r.num("threshold_v", t);
        }
        let marks: Vec<String> = self
            .bookmarks
            .iter()
            .map(|b| {
                let mut m = Obj::new();
                m.num("sample", b.sample as f64);
                m.str("label", &b.label);
                format!("    {}", m.finish())
            })
            .collect();
        let list = |v: &[String]| {
            if v.is_empty() {
                "[]".to_string()
            } else {
                format!("[\n{}\n  ]", v.join(",\n"))
            }
        };
        format!(
            "{{\n  \"visgrok\": {VERSION},\n  \"capture\": {},\n  \"channels\": {},\n  \"decoders\": {},\n  \"recording\": {},\n  \"bookmarks\": {},\n  \"notes\": {}\n}}\n",
            jstr(&self.capture),
            list(&chans),
            d.finish(),
            r.finish(),
            list(&marks),
            jstr(&self.notes)
        )
    }

    /// Roles padded to `channels`, unassigned as [`Role::Unknown`].
    pub fn effective_roles(&self, channels: usize) -> Vec<Role> {
        (0..channels)
            .map(|i| self.roles.get(i).cloned().flatten().unwrap_or(Role::Unknown))
            .collect()
    }

    /// Human-readable list of the buses the roles describe.
    pub fn buses(&self) -> Vec<String> {
        let name = |i: usize| self.names.get(i).filter(|n| !n.is_empty()).cloned().unwrap_or(format!("D{i}"));
        let find = |want: &Role| self.roles.iter().position(|r| r.as_ref() == Some(want));
        let mut out = Vec::new();
        for (i, r) in self.roles.iter().enumerate() {
            match r {
                Some(Role::Uart { baud }) => out.push(format!(
                    "UART on {} ({}, {})",
                    name(i),
                    if *baud == 0 {
                        "auto baud".to_string()
                    } else {
                        format!("{baud} baud")
                    },
                    uart_format_id(self.options.uart_format)
                )),
                Some(Role::I2cScl { sda }) => out.push(format!("I2C: SCL {}, SDA {}", name(i), name(*sda as usize))),
                Some(Role::SpiClk) => {
                    let mut s = format!("SPI ({}): SCLK {}", self.options.spi_protocol.id(), name(i));
                    for (label, role) in [
                        ("MOSI", Role::SpiMosi),
                        ("MISO", Role::SpiMiso),
                        ("D/C", Role::SpiDc),
                        ("CS", Role::SpiCs),
                    ] {
                        if let Some(c) = find(&role) {
                            s += &format!(", {label} {}", name(c));
                        }
                    }
                    out.push(s);
                }
                Some(Role::SdClk) => {
                    let mut s = format!("SD card: CLK {}", name(i));
                    if let Some(c) = find(&Role::SdCmd) {
                        s += &format!(", CMD {}", name(c));
                    }
                    for n in 0..4 {
                        if let Some(c) = find(&Role::SdDat(n)) {
                            s += &format!(", DAT{n} {}", name(c));
                        }
                    }
                    out.push(s);
                }
                _ => {}
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let s = Sidecar {
            capture: "run1.vgk".into(),
            names: vec![
                "CLK8M".into(),
                "RESET".into(),
                "UART".into(),
                "SCLK".into(),
                "DC".into(),
                "CS".into(),
                "MOSI".into(),
            ],
            roles: vec![
                None,
                None,
                Some(Role::Uart { baud: 0 }),
                Some(Role::SpiClk),
                Some(Role::SpiDc),
                Some(Role::SpiCs),
                Some(Role::SpiMosi),
            ],
            options: DecoderOptions {
                spi_protocol: SpiProtocol::Ssd1306 { width: 128, height: 64 },
                uart_format: Some((8, crate::decode::uart::Parity::Even, 2)),
                ..Default::default()
            },
            device: Some("SLogic16 U3".into()),
            samplerate: Some(200_000_000),
            threshold: Some(1.65),
            bookmarks: vec![Bookmark {
                sample: 15_820_000,
                label: "ATR".into(),
            }],
            notes: "boot \"test\"".into(),
        };
        let text = s.to_json();
        assert_eq!(Sidecar::parse(&text), Some(s.clone()), "{text}");
        assert_eq!(
            s.buses(),
            vec![
                "UART on UART (auto baud, 8E2)".to_string(),
                "SPI (ssd1306): SCLK SCLK, MOSI MOSI, D/C DC, CS CS".to_string()
            ]
        );
        let dir = std::env::temp_dir().join(format!("visgrok-side-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cap = dir.join("run1.vgk");
        s.save(&cap).unwrap();
        assert!(dir.join("run1.vgk.json").exists());
        assert_eq!(Sidecar::load(&cap).unwrap(), Some(s));
        assert_eq!(Sidecar::load(&dir.join("none.vgk")).unwrap(), None);
    }
}
