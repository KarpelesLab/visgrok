//! Web control mode: `visgrok web` serves a single-page UI and a WebSocket.
//!
//! The browser controls captures (start/stop, device settings), assigns
//! channel roles and names, and browses any part of a live or recorded
//! capture. Every request is answered from a [`SampleStore`]: the capture
//! threads never wait for the UI, and data streams to disk regardless of
//! what the page is looking at.
//!
//! Everything here is std-only: a minimal HTTP/1.1 server, RFC 6455
//! WebSocket framing (with the SHA-1 the handshake needs) and a small JSON
//! reader/writer for the message protocol described in `web/index.html`.

use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use visgrok::Source;
use visgrok::analyzer::{Analyzer, DecoderOptions, SpiProtocol, UartProtocol, parse_uart_format};
use visgrok::json::{Json, Obj, jstr};
use visgrok::roles::{Role, Suggestion, fmt_hz};
use visgrok::sidecar::{Bookmark, Sidecar};
use visgrok::slogic::{Config, SLogic};
use visgrok::store::{SampleStore, StoredEvent};
use visgrok::synth::Synth;
use visgrok::vgk::VgkReader;

use crate::pipeline::{Pipeline, Setup, format_event};

const PAGE: &str = include_str!("web/index.html");

/// Web mode settings.
pub struct WebOptions {
    /// Address to listen on.
    pub listen: String,
    /// Directory for new captures and the "open" list.
    pub dir: PathBuf,
    /// Capture to open at start.
    pub open: Option<PathBuf>,
}

// ---------------------------------------------------------------- session

enum Mode {
    Idle,
    /// Converting a foreign file (.sr, .vcd) into a browsable .vgk.
    Importing {
        file: String,
        progress: Arc<AtomicU64>,
        cancel: Arc<AtomicBool>,
    },
    Live {
        pipe: Arc<Pipeline>,
        store: Arc<SampleStore>,
    },
    Review {
        store: Arc<SampleStore>,
        decode: Option<DecodeJob>,
    },
}

struct DecodeJob {
    cancel: Arc<AtomicBool>,
    progress: Arc<AtomicU64>,
}

struct Session {
    dir: PathBuf,
    mode: Mutex<Mode>,
    roles: Mutex<Vec<Option<Role>>>,
    options: Mutex<DecoderOptions>,
    names: Mutex<Vec<String>>,
    error: Mutex<Option<String>>,
    /// The capture the user sees (for imported files, the original), whose
    /// sidecar (`<capture>.json`) holds roles, names, bookmarks and notes.
    capture: Mutex<Option<PathBuf>>,
    bookmarks: Mutex<Vec<Bookmark>>,
    notes: Mutex<String>,
    /// Recording settings: device, sample rate, threshold.
    recording: Mutex<(Option<String>, Option<u64>, Option<f64>)>,
    /// Auto-detected roles of the reviewed capture (from the background pass).
    suggestions: Arc<Mutex<Vec<Suggestion>>>,
}

fn role_id(r: &Option<Role>) -> String {
    r.as_ref().map(Role::id).unwrap_or_default()
}

impl Session {
    /// The sidecar describing the current capture.
    fn sidecar(&self) -> Sidecar {
        let cap = self.capture.lock().unwrap().clone();
        let rec = self.recording.lock().unwrap().clone();
        Sidecar {
            capture: cap
                .as_ref()
                .and_then(|p| p.file_name())
                .map(|f| f.to_string_lossy().into_owned())
                .unwrap_or_default(),
            names: self.names.lock().unwrap().clone(),
            roles: self.roles.lock().unwrap().clone(),
            options: self.options.lock().unwrap().clone(),
            device: rec.0,
            samplerate: rec.1,
            threshold: rec.2,
            bookmarks: self.bookmarks.lock().unwrap().clone(),
            notes: self.notes.lock().unwrap().clone(),
        }
    }

    /// Writes the sidecar next to the current capture.
    fn save_sidecar(&self) {
        let Some(cap) = self.capture.lock().unwrap().clone() else { return };
        if let Err(e) = self.sidecar().save(&cap) {
            self.set_error(format!("saving {}: {e}", visgrok::sidecar::path_for(&cap).display()));
        }
    }

    /// Adopts a capture's sidecar (or defaults when it has none).
    fn load_sidecar(&self, capture: &Path, default_names: Vec<String>) {
        let sc = match Sidecar::load(capture) {
            Ok(s) => s,
            Err(e) => {
                self.set_error(e.to_string());
                None
            }
        };
        *self.capture.lock().unwrap() = Some(capture.to_path_buf());
        match sc {
            Some(sc) => {
                let names = (0..default_names.len())
                    .map(|i| {
                        sc.names
                            .get(i)
                            .filter(|n| !n.is_empty())
                            .cloned()
                            .unwrap_or_else(|| default_names[i].clone())
                    })
                    .collect();
                *self.names.lock().unwrap() = names;
                *self.roles.lock().unwrap() = sc.roles;
                *self.options.lock().unwrap() = sc.options;
                *self.bookmarks.lock().unwrap() = sc.bookmarks;
                *self.notes.lock().unwrap() = sc.notes;
                *self.recording.lock().unwrap() = (sc.device, sc.samplerate, sc.threshold);
            }
            None => {
                *self.names.lock().unwrap() = default_names;
                *self.bookmarks.lock().unwrap() = Vec::new();
                *self.notes.lock().unwrap() = String::new();
                *self.recording.lock().unwrap() = (None, None, None);
            }
        }
    }

    fn store(&self) -> Option<Arc<SampleStore>> {
        match &*self.mode.lock().unwrap() {
            Mode::Idle | Mode::Importing { .. } => None,
            Mode::Live { store, .. } | Mode::Review { store, .. } => Some(store.clone()),
        }
    }

    fn set_error(&self, e: impl Into<String>) {
        *self.error.lock().unwrap() = Some(e.into());
    }

    fn effective_roles(&self, channels: usize) -> Vec<Role> {
        let mut r: Vec<Role> = self
            .roles
            .lock()
            .unwrap()
            .iter()
            .map(|r| r.clone().unwrap_or(Role::Unknown))
            .collect();
        r.resize(channels, Role::Unknown);
        r
    }

    /// Stops whatever is running.
    fn close(&self) {
        let old = std::mem::replace(&mut *self.mode.lock().unwrap(), Mode::Idle);
        match old {
            Mode::Importing { cancel, .. } => cancel.store(true, Ordering::SeqCst),
            Mode::Live { pipe, .. } => {
                pipe.stop();
                // Let the writer finish the file in the background.
                std::thread::spawn(move || {
                    pipe.join();
                });
            }
            Mode::Review { decode: Some(j), .. } => j.cancel.store(true, Ordering::SeqCst),
            _ => {}
        }
    }

    fn start(&self, msg: &Json) -> Result<(), String> {
        self.close();
        let channels = msg.get("channels").and_then(Json::num).unwrap_or(16.0) as usize;
        let device = msg.get("device").and_then(Json::str).unwrap_or("slogic");
        let rate = msg.get("samplerate").and_then(Json::num).map(|r| r as u64);
        let source: Box<dyn Source> = match device {
            "demo" => Box::new(Synth::device(rate.unwrap_or(50_000_000), None)),
            "demo-bus" => Box::new(Synth::new(rate.unwrap_or(20_000_000), None)),
            _ => {
                let dev = SLogic::open(msg.get("serial").and_then(Json::str)).map_err(|e| e.to_string())?;
                let rate = rate.unwrap_or_else(|| dev.model().max_samplerate(channels));
                let mut cfg = Config::new(channels, rate);
                cfg.threshold = msg.get("threshold").and_then(Json::num);
                dev.validate(&cfg).map_err(|e| e.to_string())?;
                Box::new(dev.start(cfg).map_err(|e| e.to_string())?)
            }
        };
        let mut info = source.info();
        let names = self.names.lock().unwrap().clone();
        info.names = (0..info.channels)
            .map(|i| names.get(i).filter(|n| !n.is_empty()).cloned().unwrap_or(format!("D{i}")))
            .collect();
        std::fs::create_dir_all(&self.dir).map_err(|e| e.to_string())?;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let path = self.dir.join(format!("capture-{stamp}.vgk"));
        let (store, writer) = SampleStore::create_live(&path, &info, Vec::new()).map_err(|e| format!("{}: {e}", path.display()))?;
        let roles = self.roles.lock().unwrap().clone();
        let setup = Setup {
            roles,
            options: self.options.lock().unwrap().clone(),
            names: info.names.iter().cloned().map(Some).collect(),
            writer: Some(Box::new(writer)),
            store: Some(store.clone()),
            ..Default::default()
        };
        let pipe = Pipeline::start(source, Some(path.clone()), setup)?;
        *self.mode.lock().unwrap() = Mode::Live { pipe, store };
        *self.error.lock().unwrap() = None;
        *self.capture.lock().unwrap() = Some(path);
        *self.names.lock().unwrap() = info.names.clone();
        *self.recording.lock().unwrap() = (
            Some(info.device.clone()),
            Some(info.samplerate),
            msg.get("threshold").and_then(Json::num),
        );
        self.bookmarks.lock().unwrap().clear();
        self.notes.lock().unwrap().clear();
        self.save_sidecar();
        Ok(())
    }

    fn open(self: &Arc<Self>, name: &str) -> Result<(), String> {
        // Only files in the capture directory (no path traversal).
        let file = Path::new(name).file_name().ok_or("bad file name")?;
        let path = self.dir.join(file);
        self.open_path(&path)
    }

    fn open_path(self: &Arc<Self>, path: &Path) -> Result<(), String> {
        self.close();
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
        if ext == "vgk" {
            return self.review(path, path);
        }
        // Other formats are imported once (in the background) into a .vgk
        // next to them, with an overview so later opens are instant.
        let out = path.with_extension(format!("{ext}.vgk"));
        if out.exists() {
            return self.review(&out, path);
        }
        let progress = Arc::new(AtomicU64::new(0));
        let cancel = Arc::new(AtomicBool::new(false));
        let file = path.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
        *self.mode.lock().unwrap() = Mode::Importing {
            file,
            progress: progress.clone(),
            cancel: cancel.clone(),
        };
        let (me, src) = (self.clone(), path.to_path_buf());
        std::thread::spawn(move || {
            let tmp = out.with_extension("vgk.part");
            let r = SampleStore::import(&src, &tmp, &progress, &cancel).and_then(|()| std::fs::rename(&tmp, &out));
            match r {
                Ok(()) => {
                    // Not cancelled meanwhile? (Evaluated on its own: review()
                    // takes the mode lock too.)
                    let still = matches!(&*me.mode.lock().unwrap(), Mode::Importing { .. });
                    if still && let Err(e) = me.review(&out, &src) {
                        me.set_error(e);
                    }
                }
                Err(e) => {
                    let _ = std::fs::remove_file(&tmp);
                    if !cancel.load(Ordering::SeqCst) {
                        me.set_error(format!("import {}: {e}", src.display()));
                        *me.mode.lock().unwrap() = Mode::Idle;
                    }
                }
            }
        });
        Ok(())
    }

    /// Opens `vgk` for review; `original` is the file the user opened (it
    /// differs for imported .sr/.vcd files) and holds the sidecar.
    fn review(&self, vgk: &Path, original: &Path) -> Result<(), String> {
        let store = SampleStore::open(vgk).map_err(|e| format!("{}: {e}", vgk.display()))?;
        self.suggestions.lock().unwrap().clear();
        self.load_sidecar(original, store.info().all_names());
        store.set_names(self.names.lock().unwrap().clone());
        *self.mode.lock().unwrap() = Mode::Review { store, decode: None };
        *self.error.lock().unwrap() = None;
        self.redecode();
        Ok(())
    }

    /// Applies role/option changes: live decoders are rebuilt in place;
    /// reviewed files are decoded again in the background.
    fn apply_roles(&self) {
        if let Mode::Live { pipe, .. } = &*self.mode.lock().unwrap() {
            *pipe.roles.lock().unwrap() = self.roles.lock().unwrap().clone();
            *pipe.options.lock().unwrap() = self.options.lock().unwrap().clone();
            pipe.rebuild_decoders();
            return;
        }
        self.redecode();
    }

    fn redecode(&self) {
        let mut mode = self.mode.lock().unwrap();
        let Mode::Review { store, decode } = &mut *mode else { return };
        if let Some(j) = decode.take() {
            j.cancel.store(true, Ordering::SeqCst);
        }
        store.clear_events();
        let info = store.info();
        let roles = self.effective_roles(info.channels);
        // The pass runs even without roles: it also detects them.
        let suggestions = self.suggestions.clone();
        let opts = self.options.lock().unwrap().clone();
        let job = DecodeJob {
            cancel: Arc::new(AtomicBool::new(false)),
            progress: Arc::new(AtomicU64::new(0)),
        };
        let (cancel, progress, store) = (job.cancel.clone(), job.progress.clone(), store.clone());
        *decode = Some(job);
        std::thread::spawn(move || {
            let Ok(mut r) = VgkReader::open(store.path()) else { return };
            let mut a = Analyzer::new(info.channels, info.samplerate);
            let d = a.decoders_for_roles(&roles, &opts);
            a.set_decoders(d);
            let names: Vec<(Arc<str>, u8)> = a
                .decoders()
                .iter()
                .map(|d| (Arc::from(d.name()), d.channels().trailing_zeros() as u8))
                .collect();
            while let Ok(Some(b)) = r.read_block() {
                if cancel.load(Ordering::SeqCst) {
                    return;
                }
                let before = a.annotation_count;
                a.process(&b);
                let new = ((a.annotation_count - before) as usize).min(a.annotations.len());
                let skip = a.annotations.len() - new;
                for t in a.annotations.iter().skip(skip) {
                    if let visgrok::decode::Event::Frame(v) = &t.annotation.event {
                        store.add_frame(t.annotation.start, v.clone());
                    }
                }
                store.add_events(
                    a.annotations
                        .iter()
                        .skip(skip)
                        .filter(|t| !matches!(t.annotation.event, visgrok::decode::Event::Frame(_)))
                        .map(|t| {
                            let (source, channel) = names.get(t.decoder).cloned().unwrap_or((Arc::from("?"), 0));
                            StoredEvent {
                                start: t.annotation.start,
                                end: t.annotation.end,
                                source,
                                channel,
                                text: format_event(&t.annotation.event),
                                data: match &t.annotation.event {
                                    visgrok::decode::Event::Protocol { data, .. } => data.clone(),
                                    _ => None,
                                },
                            }
                        }),
                );
                progress.store(b.end(), Ordering::SeqCst);
            }
            *suggestions.lock().unwrap() = a.suggest();
            progress.store(u64::MAX, Ordering::SeqCst);
        });
    }

    /// Decodes the start of the reviewed capture as raw SPI and checks
    /// whether the command bytes (D/C low) include typical SSD1306 setup
    /// commands (display off/on, multiplex ratio, charge pump, addressing).
    fn looks_like_ssd1306(&self) -> bool {
        let Some(store) = self.store() else { return false };
        let info = store.info();
        let roles = self.effective_roles(info.channels);
        let Ok(mut r) = VgkReader::open(store.path()) else { return false };
        let mut a = Analyzer::new(info.channels, info.samplerate);
        let d = a.decoders_for_roles(&roles, &DecoderOptions::default());
        a.set_decoders(d);
        let mut cmds = std::collections::HashSet::new();
        let mut words = 0;
        while let Ok(Some(b)) = r.read_block() {
            a.process(&b);
            for t in a.annotations.drain(..) {
                if let visgrok::decode::Event::SpiWord {
                    mosi: Some(m),
                    dc: Some(false),
                    ..
                } = t.annotation.event
                {
                    cmds.insert(m as u8);
                }
                words += 1;
            }
            if words > 4096 {
                break;
            }
        }
        let typical = [0xae, 0xaf, 0xa8, 0x8d, 0x20, 0xd5, 0xd9, 0xda, 0xdb, 0x81, 0xa1, 0xc8];
        typical.iter().filter(|c| cmds.contains(c)).count() >= 4
    }

    /// Whether the reviewed capture's UART starts like a smart card: even
    /// parity framing and a first byte of 0x3B or 0x3F (an ATR's TS).
    fn looks_like_iso7816(&self) -> bool {
        let Some(store) = self.store() else { return false };
        let info = store.info();
        let roles: Vec<Role> = self
            .effective_roles(info.channels)
            .into_iter()
            .map(|r| if matches!(r, Role::Uart { .. }) { r } else { Role::Unknown })
            .collect();
        let Ok(mut r) = VgkReader::open(store.path()) else { return false };
        let mut a = Analyzer::new(info.channels, info.samplerate);
        let d = a.decoders_for_roles(&roles, &DecoderOptions::default());
        a.set_decoders(d);
        let (mut even, mut first) = (false, None);
        while let Ok(Some(b)) = r.read_block() {
            a.process(&b);
            for t in a.annotations.drain(..) {
                match t.annotation.event {
                    visgrok::decode::Event::UartFormat { format } => even = format.starts_with("8E"),
                    visgrok::decode::Event::UartByte { value, .. } if first.is_none() => first = Some(value),
                    _ => {}
                }
            }
            if first.is_some() {
                break;
            }
        }
        even && matches!(first, Some(0x3b | 0x3f))
    }

    fn status_json(&self) -> String {
        // Before taking any other lock: sidecar() locks names/roles/options.
        let buses: Vec<String> = self.sidecar().buses().iter().map(|b| jstr(b)).collect();
        let mode = self.mode.lock().unwrap();
        let mut o = Obj::new();
        o.str("type", "status");
        let (store, suggestions, pipe_info): (Option<&Arc<SampleStore>>, Vec<Suggestion>, Option<String>) = match &*mode {
            Mode::Idle => {
                o.str("mode", "idle");
                (None, Vec::new(), None)
            }
            Mode::Importing { file, progress, .. } => {
                o.str("mode", "importing");
                o.str("file", file);
                o.num("imported", progress.load(Ordering::SeqCst) as f64);
                (None, Vec::new(), None)
            }
            Mode::Live { pipe, store } => {
                o.str("mode", "live");
                o.bool("running", !pipe.finished());
                o.num("written", pipe.written() as f64);
                o.num("skipped", pipe.blocks_skipped() as f64);
                if let Some(e) = pipe.error() {
                    o.str("captureError", &e);
                }
                let sugg = pipe.analyzer.lock().unwrap().suggest();
                (Some(store), sugg, Some(pipe.info.device.clone()))
            }
            Mode::Review { store, decode } => {
                o.str("mode", "review");
                if let Some(j) = decode {
                    let p = j.progress.load(Ordering::SeqCst);
                    o.num("decoded", if p == u64::MAX { store.total() as f64 } else { p as f64 });
                }
                (Some(store), self.suggestions.lock().unwrap().clone(), None)
            }
        };
        if let Some(s) = store {
            let info = s.info();
            o.str("device", pipe_info.as_deref().unwrap_or(&info.device));
            o.str(
                "file",
                &s.path().file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default(),
            );
            o.num("total", s.total() as f64);
            o.num("loaded", s.loaded() as f64);
            o.num("samplerate", info.samplerate as f64);
            o.str("rate", &fmt_hz(info.samplerate as f64));
            o.num("channels", info.channels as f64);
            o.num("events", s.event_count() as f64);
            o.num("frames", s.frame_at(u64::MAX).1 as f64);
            let names = self.names.lock().unwrap();
            let roles = self.roles.lock().unwrap();
            let chans: Vec<String> = (0..info.channels)
                .map(|i| {
                    let mut c = Obj::new();
                    c.str("name", names.get(i).filter(|n| !n.is_empty()).map_or(&info.name(i), |n| n));
                    c.str("role", &role_id(&roles.get(i).cloned().flatten()));
                    if let Some(sg) = suggestions.get(i).filter(|s| !matches!(s.role, Role::Unknown)) {
                        c.str("suggest", &sg.role.to_string());
                    }
                    c.finish()
                })
                .collect();
            o.raw("chans", &format!("[{}]", chans.join(",")));
        }
        let (spi, uart_fmt, uart_proto) = {
            let o = self.options.lock().unwrap();
            (
                o.spi_protocol,
                visgrok::analyzer::uart_format_id(o.uart_format),
                o.uart_protocol.id(),
            )
        };
        o.str("uartFormat", &uart_fmt);
        o.str("uartProto", uart_proto);
        o.str(
            "spiProto",
            match spi {
                SpiProtocol::Raw => "raw",
                SpiProtocol::Ssd1306 { height: 32, .. } => "ssd1306:128x32",
                SpiProtocol::Ssd1306 { .. } => "ssd1306",
            },
        );
        let marks: Vec<String> = self
            .bookmarks
            .lock()
            .unwrap()
            .iter()
            .map(|b| format!("[{},{}]", b.sample, jstr(&b.label)))
            .collect();
        o.raw("bookmarks", &format!("[{}]", marks.join(",")));
        o.str("notes", &self.notes.lock().unwrap());
        o.raw("buses", &format!("[{}]", buses.join(",")));
        if let Some(c) = &*self.capture.lock().unwrap() {
            o.str(
                "sidecar",
                &visgrok::sidecar::path_for(c)
                    .file_name()
                    .map(|f| f.to_string_lossy().into_owned())
                    .unwrap_or_default(),
            );
        }
        if let Some(e) = &*self.error.lock().unwrap() {
            o.str("error", e);
        }
        o.finish()
    }

    fn files_json(&self) -> String {
        let mut files: Vec<(String, u64, u64)> = std::fs::read_dir(&self.dir)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().into_owned();
                let ext = name.rsplit('.').next()?.to_ascii_lowercase();
                // Imported copies are opened through their source file.
                if name.ends_with(".sr.vgk") || name.ends_with(".vcd.vgk") {
                    return None;
                }
                if !matches!(ext.as_str(), "vgk" | "sr" | "vcd") {
                    return None;
                }
                let m = e.metadata().ok()?;
                let t = m.modified().ok()?.duration_since(std::time::UNIX_EPOCH).ok()?.as_secs();
                Some((name, m.len(), t))
            })
            .collect();
        files.sort_by_key(|f| std::cmp::Reverse(f.2));
        let items: Vec<String> = files
            .iter()
            .map(|(n, size, t)| {
                let mut o = Obj::new();
                o.str("name", n);
                o.num("size", *size as f64);
                o.num("mtime", *t as f64);
                o.finish()
            })
            .collect();
        format!(
            r#"{{"type":"files","dir":{},"files":[{}]}}"#,
            jstr(&self.dir.display().to_string()),
            items.join(",")
        )
    }

    /// Handles one client message; returns the reply, if any.
    fn handle(self: &Arc<Self>, msg: &Json) -> Option<String> {
        let id = msg.get("id").and_then(Json::num).unwrap_or(0.0);
        let cmd = msg.get("cmd").and_then(Json::str).unwrap_or("");
        let result = |r: Result<(), String>| -> Option<String> {
            match r {
                Ok(()) => Some(self.status_json()),
                Err(e) => {
                    self.set_error(e.clone());
                    Some(format!(r#"{{"type":"error","message":{}}}"#, jstr(&e)))
                }
            }
        };
        match cmd {
            "status" => Some(self.status_json()),
            "files" => Some(self.files_json()),
            "start" => result(self.start(msg)),
            "stop" => {
                if let Mode::Live { pipe, .. } = &*self.mode.lock().unwrap() {
                    pipe.stop();
                }
                Some(self.status_json())
            }
            "close" => {
                self.close();
                Some(self.status_json())
            }
            "open" => result(
                msg.get("file")
                    .and_then(Json::str)
                    .ok_or_else(|| "no file".to_string())
                    .and_then(|f| self.open(f)),
            ),
            "roles" => {
                if let Some(Json::Arr(a)) = msg.get("roles") {
                    let mut roles = Vec::with_capacity(a.len());
                    for r in a {
                        let r = r.str().unwrap_or("");
                        roles.push(if r.is_empty() { None } else { Role::parse(r).ok() });
                    }
                    *self.roles.lock().unwrap() = roles;
                }
                {
                    let mut o = self.options.lock().unwrap();
                    if let Some(p) = msg.get("spiProto").and_then(Json::str).and_then(|p| SpiProtocol::parse(p).ok()) {
                        o.spi_protocol = p;
                    }
                    if let Some(f) = msg.get("uartFormat").and_then(Json::str).and_then(|f| parse_uart_format(f).ok()) {
                        o.uart_format = f;
                    }
                    if let Some(p) = msg.get("uartProto").and_then(Json::str).and_then(|p| UartProtocol::parse(p).ok()) {
                        o.uart_protocol = p;
                    }
                }
                self.apply_roles();
                self.save_sidecar();
                Some(self.status_json())
            }
            "names" => {
                if let Some(Json::Arr(a)) = msg.get("names") {
                    let names: Vec<String> = a.iter().map(|n| n.str().unwrap_or("").to_string()).collect();
                    if let Some(s) = self.store() {
                        let info = s.info();
                        s.set_names(
                            (0..info.channels)
                                .map(|i| names.get(i).filter(|n| !n.is_empty()).cloned().unwrap_or(format!("D{i}")))
                                .collect(),
                        );
                    }
                    *self.names.lock().unwrap() = names;
                    self.save_sidecar();
                }
                Some(self.status_json())
            }
            "view" => {
                let store = self.store()?;
                let start = msg.get("start").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let end = msg.get("end").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let cols = msg.get("columns").and_then(Json::num).unwrap_or(800.0) as usize;
                let rows = match store.view(start, end, cols) {
                    Ok(r) => r,
                    Err(e) => return Some(format!(r#"{{"type":"error","message":{}}}"#, jstr(&e.to_string()))),
                };
                let rows: Vec<String> = rows.iter().map(|r| jstr(&r.iter().map(|l| l.char()).collect::<String>())).collect();
                let tag = msg.get("tag").and_then(Json::str).unwrap_or("main");
                Some(format!(
                    r#"{{"type":"view","id":{id},"tag":{},"start":{start},"end":{end},"columns":{cols},"rows":[{}]}}"#,
                    jstr(tag),
                    rows.join(",")
                ))
            }
            "events" => {
                let store = self.store()?;
                let start = msg.get("start").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let end = msg.get("end").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let limit = msg.get("limit").and_then(Json::num).unwrap_or(2000.0) as usize;
                let items: Vec<String> = store.events(start, end, limit).iter().map(event_json).collect();
                Some(format!(
                    r#"{{"type":"events","id":{id},"start":{start},"end":{end},"items":[{}]}}"#,
                    items.join(",")
                ))
            }
            "auto" => {
                // Give unassigned channels their detected role.
                let sugg: Vec<Suggestion> = match &*self.mode.lock().unwrap() {
                    Mode::Live { pipe, .. } => pipe.analyzer.lock().unwrap().suggest(),
                    _ => self.suggestions.lock().unwrap().clone(),
                };
                {
                    let mut roles = self.roles.lock().unwrap();
                    let len = sugg.len().max(roles.len());
                    roles.resize(len, None);
                    for (r, s) in roles.iter_mut().zip(&sugg) {
                        let decodable = matches!(
                            s.role,
                            Role::Uart { .. }
                                | Role::I2cScl { .. }
                                | Role::I2cSda { .. }
                                | Role::SpiClk
                                | Role::SpiMosi
                                | Role::SpiMiso
                                | Role::SpiCs
                                | Role::SpiDc
                                | Role::SpiData { .. }
                        );
                        if r.is_none() && decodable {
                            // Auto-detected UARTs follow rate changes from any start.
                            *r = Some(match s.role {
                                Role::Uart { .. } => Role::Uart { baud: 0 },
                                ref x => x.clone(),
                            });
                        }
                    }
                }
                // SPI with a D/C line drives a display controller: if its first
                // command bytes look like SSD1306 commands, decode them as such.
                let has_dc = self.roles.lock().unwrap().contains(&Some(Role::SpiDc));
                if has_dc && self.options.lock().unwrap().spi_protocol == SpiProtocol::Raw && self.looks_like_ssd1306() {
                    self.options.lock().unwrap().spi_protocol = SpiProtocol::Ssd1306 { width: 128, height: 64 };
                }
                // A UART whose first byte is an ATR start in 8E framing is a
                // smart card line.
                let has_uart = self.roles.lock().unwrap().iter().any(|r| matches!(r, Some(Role::Uart { .. })));
                if has_uart && self.options.lock().unwrap().uart_protocol == UartProtocol::Raw && self.looks_like_iso7816() {
                    self.options.lock().unwrap().uart_protocol = UartProtocol::Iso7816;
                }
                self.apply_roles();
                self.save_sidecar();
                Some(self.status_json())
            }
            "notes" => {
                *self.notes.lock().unwrap() = msg.get("text").and_then(Json::str).unwrap_or("").to_string();
                self.save_sidecar();
                Some(self.status_json())
            }
            "bookmark" => {
                let sample = msg.get("sample").and_then(Json::num)?.max(0.0) as u64;
                let label = msg.get("label").and_then(Json::str).unwrap_or("").to_string();
                {
                    let mut b = self.bookmarks.lock().unwrap();
                    b.retain(|m| m.sample != sample);
                    if msg.get("remove").and_then(Json::bool) != Some(true) {
                        b.push(Bookmark { sample, label });
                    }
                    b.sort_by_key(|m| m.sample);
                }
                self.save_sidecar();
                Some(self.status_json())
            }
            "display" => {
                // The reconstructed display at a moment: rows of hex, 1 bit
                // per pixel, MSB = leftmost.
                let store = self.store()?;
                let at = msg.get("at").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let (frame, count) = store.frame_at(at);
                let Some((sample, v)) = frame else {
                    return Some(format!(r#"{{"type":"display","frames":{count}}}"#));
                };
                let rows: Vec<String> = (0..v.height)
                    .map(|y| {
                        let mut row = String::with_capacity(v.width / 4);
                        for x in (0..v.width).step_by(4) {
                            let n = (0..4).fold(0u8, |n, k| n << 1 | (x + k < v.width && v.pixels[y * v.width + x + k]) as u8);
                            row.push(char::from_digit(n as u32, 16).unwrap());
                        }
                        jstr(&row)
                    })
                    .collect();
                Some(format!(
                    r#"{{"type":"display","frames":{count},"sample":{sample},"title":{},"width":{},"height":{},"on":{},"update":{},"rows":[{}]}}"#,
                    jstr(v.title),
                    v.width,
                    v.height,
                    v.on,
                    v.updates,
                    rows.join(",")
                ))
            }
            "event" => {
                // One event in full: text, payload and the surrounding events
                // of the same decoder.
                let store = self.store()?;
                let start = msg.get("start").and_then(Json::num)?.max(0.0) as u64;
                let ch = msg.get("channel").and_then(Json::num).unwrap_or(0.0) as u8;
                let end = msg.get("end").and_then(Json::num).map(|e| e.max(0.0) as u64);
                let Some((e, before, after)) = store.event_with_context(start, ch, end, 6) else {
                    return Some(r#"{"type":"error","message":"event not found"}"#.to_string());
                };
                let hex: String = e
                    .data
                    .as_deref()
                    .map(|d| d.iter().map(|b| format!("{b:02x}")).collect())
                    .unwrap_or_default();
                let list = |v: &[StoredEvent]| v.iter().map(event_json).collect::<Vec<_>>().join(",");
                Some(format!(
                    r#"{{"type":"event","ev":{},"data":{},"before":[{}],"after":[{}],"inspect":{}}}"#,
                    event_json(&e),
                    jstr(&hex),
                    list(&before),
                    list(&after),
                    inspect_json(&store, &e)
                ))
            }
            "seek" => {
                let store = self.store()?;
                let at = msg.get("at").and_then(Json::num).unwrap_or(0.0).max(0.0) as u64;
                let fwd = msg.get("forward").and_then(Json::bool).unwrap_or(true);
                let e = store.event_near(at, fwd)?;
                Some(format!(
                    r#"{{"type":"seek","start":{},"end":{},"text":{}}}"#,
                    e.start,
                    e.end,
                    jstr(&e.text)
                ))
            }
            _ => Some(format!(
                r#"{{"type":"error","message":{}}}"#,
                jstr(&format!("unknown command {cmd:?}"))
            )),
        }
    }
}

/// `[start, end, channel, source, text, payload bytes]`.
/// The lines an event was decoded from around it, and its bits and fields
/// (see `visgrok::inspect`).
fn inspect_json(store: &SampleStore, e: &StoredEvent) -> String {
    use visgrok::inspect::{self, Place, Request};
    const MAX_EDGES: usize = 200_000;
    let lines = inspect::lines(&e.source);
    if lines.is_empty() {
        return "null".into();
    }
    // UART timing as the decoder last reported it.
    let samplerate = store.info().samplerate as f64;
    let baud = store
        .last_event(e.start + 1, &e.source, 100_000, |t| t.starts_with("── baud rate "))
        .and_then(|b| b.text.trim_start_matches("── baud rate ").split(' ').next()?.parse::<f64>().ok());
    let format = store
        .last_event(e.start + 1, &e.source, 100_000, |t| t.starts_with("── frame format "))
        .and_then(|f| f.text.trim_start_matches("── frame format ").split(' ').next().map(str::to_string));
    let req = Request {
        source: &e.source,
        start: e.start,
        end: e.end,
        text: &e.text,
        data: e.data.as_deref(),
        bit_time: baud.filter(|&b| b > 0.0).map(|b| samplerate / b),
        format: format.as_deref(),
    };
    let (ws, we) = inspect::window(&req);
    let Ok(sig) = store.signal(ws, we, inspect::mask(&e.source)) else {
        return "null".into();
    };
    let fields = inspect::inspect(&req, &sig);
    let lines_json: Vec<String> = lines
        .iter()
        .map(|&(ch, role)| {
            let edges = sig.edges(ch, sig.start, sig.end);
            let cut = edges.len() > MAX_EDGES;
            let list: Vec<String> = edges.iter().take(MAX_EDGES).map(|x| x.0.to_string()).collect();
            format!(
                "[{ch},{},{},[{}],{cut}]",
                jstr(role),
                sig.level(ch, sig.start) as u8,
                list.join(",")
            )
        })
        .collect();
    let fields_json: Vec<String> = fields
        .iter()
        .map(|f| {
            let place = match f.place {
                Place::Line(ch) => ch.to_string(),
                Place::Row(r) => jstr(r),
            };
            format!(
                "[{},{},{place},{},{},{}]",
                f.start,
                f.end,
                jstr(&f.label),
                jstr(&f.detail),
                f.bad as u8
            )
        })
        .collect();
    format!(
        r#"{{"start":{},"end":{},"lines":[{}],"fields":[{}]}}"#,
        sig.start,
        sig.end,
        lines_json.join(","),
        fields_json.join(",")
    )
}

fn event_json(e: &StoredEvent) -> String {
    format!(
        r#"[{},{},{},{},{},{}]"#,
        e.start,
        e.end,
        e.channel,
        jstr(&e.source),
        jstr(&e.text),
        e.data.as_ref().map_or(0, |d| d.len())
    )
}

// ---------------------------------------------------------------- server

/// Runs the web server until the process is interrupted.
pub fn run(opts: WebOptions) -> io::Result<()> {
    let session = Arc::new(Session {
        dir: opts.dir,
        mode: Mutex::new(Mode::Idle),
        roles: Mutex::new(Vec::new()),
        options: Mutex::new(DecoderOptions::default()),
        names: Mutex::new(Vec::new()),
        error: Mutex::new(None),
        capture: Mutex::new(None),
        bookmarks: Mutex::new(Vec::new()),
        notes: Mutex::new(String::new()),
        recording: Mutex::new((None, None, None)),
        suggestions: Arc::new(Mutex::new(Vec::new())),
    });
    if let Some(p) = &opts.open
        && let Err(e) = session.open_path(p)
    {
        eprintln!("visgrok: {e}");
    }
    let listener = TcpListener::bind(&opts.listen)?;
    eprintln!(
        "visgrok web UI: http://{}/  (captures in {})",
        listener.local_addr()?,
        session.dir.display()
    );
    for conn in listener.incoming() {
        let Ok(stream) = conn else { continue };
        let s = session.clone();
        std::thread::spawn(move || {
            let _ = serve(stream, s);
        });
    }
    Ok(())
}

fn serve(stream: TcpStream, session: Arc<Session>) -> io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut line = String::new();
    reader.read_line(&mut line)?;
    let path = line.split_whitespace().nth(1).unwrap_or("/").to_string();
    let mut key = None;
    loop {
        let mut h = String::new();
        if reader.read_line(&mut h)? == 0 || h.trim().is_empty() {
            break;
        }
        if let Some((k, v)) = h.split_once(':')
            && k.trim().eq_ignore_ascii_case("sec-websocket-key")
        {
            key = Some(v.trim().to_string());
        }
    }
    let mut out = stream;
    match (path.as_str(), key) {
        ("/ws", Some(key)) => {
            let accept = base64(&sha1(format!("{key}258EAFA5-E914-47DA-95CA-C5AB0DC85B11").as_bytes()));
            write!(
                out,
                "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: {accept}\r\n\r\n"
            )?;
            websocket(reader, out, session)
        }
        ("/" | "/index.html", _) => {
            write!(
                out,
                "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
                PAGE.len()
            )?;
            out.write_all(PAGE.as_bytes())
        }
        _ => write!(out, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"),
    }
}

fn send_text(out: &Mutex<TcpStream>, text: &str) -> io::Result<()> {
    let b = text.as_bytes();
    let mut h = vec![0x81u8];
    match b.len() {
        n if n < 126 => h.push(n as u8),
        n if n < 65536 => {
            h.push(126);
            h.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            h.push(127);
            h.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    let mut o = out.lock().unwrap();
    o.write_all(&h)?;
    o.write_all(b)?;
    o.flush()
}

/// Reads one complete message (handling fragmentation, ping and close).
fn read_message(r: &mut impl Read, out: &Mutex<TcpStream>) -> io::Result<Option<Vec<u8>>> {
    let mut msg = Vec::new();
    loop {
        let mut h = [0u8; 2];
        r.read_exact(&mut h)?;
        let fin = h[0] & 0x80 != 0;
        let op = h[0] & 0x0f;
        let masked = h[1] & 0x80 != 0;
        let mut len = (h[1] & 0x7f) as u64;
        if len == 126 {
            let mut b = [0u8; 2];
            r.read_exact(&mut b)?;
            len = u16::from_be_bytes(b) as u64;
        } else if len == 127 {
            let mut b = [0u8; 8];
            r.read_exact(&mut b)?;
            len = u64::from_be_bytes(b);
        }
        if len > 16 << 20 {
            return Err(io::Error::other("message too large"));
        }
        let mut mask = [0u8; 4];
        if masked {
            r.read_exact(&mut mask)?;
        }
        let mut payload = vec![0u8; len as usize];
        r.read_exact(&mut payload)?;
        if masked {
            for (i, b) in payload.iter_mut().enumerate() {
                *b ^= mask[i % 4];
            }
        }
        match op {
            0x8 => return Ok(None),
            0x9 => {
                let mut o = out.lock().unwrap();
                o.write_all(&[0x8a, payload.len().min(125) as u8])?;
                o.write_all(&payload[..payload.len().min(125)])?;
                continue;
            }
            0xa => continue,
            _ => msg.extend_from_slice(&payload),
        }
        if fin {
            return Ok(Some(msg));
        }
    }
}

fn websocket(mut reader: BufReader<TcpStream>, out: TcpStream, session: Arc<Session>) -> io::Result<()> {
    let out = Arc::new(Mutex::new(out));
    let alive = Arc::new(AtomicBool::new(true));
    // Status pushes, so the page follows live captures without polling.
    {
        let (out, alive, session) = (out.clone(), alive.clone(), session.clone());
        std::thread::spawn(move || {
            while alive.load(Ordering::SeqCst) {
                if send_text(&out, &session.status_json()).is_err() {
                    break;
                }
                std::thread::sleep(Duration::from_millis(250));
            }
        });
    }
    send_text(&out, &session.files_json())?;
    let r = loop {
        let msg = match read_message(&mut reader, &out) {
            Ok(Some(m)) => m,
            Ok(None) => break Ok(()),
            Err(e) => break Err(e),
        };
        let Ok(text) = String::from_utf8(msg) else { continue };
        let Some(json) = Json::parse(&text) else { continue };
        if let Some(reply) = session.handle(&json)
            && let Err(e) = send_text(&out, &reply)
        {
            break Err(e);
        }
    };
    alive.store(false, Ordering::SeqCst);
    r
}

// ---------------------------------------------------------------- SHA-1, base64

fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476, 0xc3d2_e1f0];
    let mut msg = data.to_vec();
    let bits = (data.len() as u64) * 8;
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_be_bytes());
    for block in msg.as_chunks::<64>().0 {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes(block[4 * i..4 * i + 4].try_into().unwrap());
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (i, wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5a82_7999),
                20..=39 => (b ^ c ^ d, 0x6ed9_eba1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8f1b_bcdc),
                _ => (b ^ c ^ d, 0xca62_c1d6),
            };
            let t = a.rotate_left(5).wrapping_add(f).wrapping_add(e).wrapping_add(k).wrapping_add(*wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (x, v) in h.iter_mut().zip([a, b, c, d, e]) {
            *x = x.wrapping_add(v);
        }
    }
    let mut out = [0u8; 20];
    for (i, x) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&x.to_be_bytes());
    }
    out
}

fn base64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut o = String::new();
    for c in data.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for k in 0..4 {
            if k <= c.len() {
                o.push(T[(n >> (18 - 6 * k) & 63) as usize] as char);
            } else {
                o.push('=');
            }
        }
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handshake_vectors() {
        // RFC 6455 section 1.3 example.
        let k = "dGhlIHNhbXBsZSBub25jZQ==258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        assert_eq!(base64(&sha1(k.as_bytes())), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
        assert_eq!(base64(b"ab"), "YWI=");
    }

    #[test]
    fn json_roundtrip() {
        let j = Json::parse(r#"{"cmd":"view","start":12,"x":[1,"a\"b",true,null],"u":"été ok"}"#).unwrap();
        assert_eq!(j.get("cmd").and_then(Json::str), Some("view"));
        assert_eq!(j.get("start").and_then(Json::num), Some(12.0));
        assert_eq!(j.get("u").and_then(Json::str), Some("été ok"));
        assert_eq!(jstr("a\"b\n"), r#""a\"b\n""#);
    }
}
