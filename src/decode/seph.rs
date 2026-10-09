//! Ledger SEPROXYHAL: the packet protocol between the secure element (SE)
//! and the MCU of a Ledger device (Nano S, Nano X, ...), carried over their
//! ISO 7816 link once the SE's ATR and the PPS are done.
//!
//! Every packet is `tag (1) | length (2, big endian) | payload`. The MCU
//! sends **events** (tags below 0x30: ticker, buttons, USB and BLE traffic,
//! APDUs...); the SE answers with **commands** (0x30..0x5f: USB and BLE
//! transfers, display, power...) and ends its turn with a **status**
//! (0x60 and up, usually `GENERAL_STATUS`).
//!
//! On top of the packets, this decoder reassembles the APDUs exchanged with
//! the host, from USB HID reports (`USB_EP_XFER_EVENT` in,
//! `USB_EP_PREPARE` out) and BLE GATT writes and notifications (HCI packets
//! in `BLE_RECV_EVENT` / `BLE_SEND`), framed by Ledger's transport protocol
//! (`channel (USB only) | 0x05 | sequence | length (first chunk) | data`).
//!
//! Tag values and payload layouts follow Ledger's secure SDKs
//! (`seproxyhal_protocol.h`, current and Nano X/S versions).

use super::hci;
use super::{Annotation, Event};

pub(crate) const PROTO: &str = "SEPH";

/// Largest payload accepted; longer lengths mean the stream is out of sync.
const MAX_PAYLOAD: usize = 1024;

/// Bytes as hex, at most `max` of them (then an ellipsis and the count).
pub(crate) fn hex_trunc(b: &[u8], max: usize) -> String {
    let mut s: String = b.iter().take(max).map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ");
    if b.len() > max {
        s += &format!(" … ({} bytes)", b.len());
    }
    if s.is_empty() { "-".into() } else { s }
}

fn text(b: &[u8]) -> String {
    b.iter().map(|&c| if (0x20..0x7f).contains(&c) { c as char } else { '.' }).collect()
}

fn printable(b: &[u8]) -> bool {
    !b.is_empty() && b.iter().all(|&c| (0x20..0x7f).contains(&c))
}

fn be16(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_be_bytes([*b.get(at)?, *b.get(at + 1)?]))
}

fn be32(b: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_be_bytes(b.get(at..at + 4)?.try_into().ok()?))
}

/// Name of a tag, from both the current and the older SDKs.
pub fn tag_name(tag: u8) -> Option<&'static str> {
    Some(match tag {
        0x01 => "SESSION_START_EVENT",
        0x02 => "BLE_CONNECTION_EVENT",
        0x03 => "BLE_WRITE_REQUEST_EVENT",
        0x04 => "BLE_READ_REQUEST_EVENT",
        0x05 => "BUTTON_PUSH_EVENT",
        0x06 => "NFC_FIELD_DETECTION_EVENT",
        0x07 => "NFC_APDU_RECEIVED_EVENT",
        0x08 => "BATTERY_NOTIFICATION_EVENT",
        0x09 => "M24SR_GPO_CHANGE_EVENT",
        0x0a => "M24SR_RESPONSE_APDU_EVENT",
        0x0b => "BLE_NOTIFY_INDICATE_EVENT",
        0x0c => "FINGER_EVENT",
        0x0d => "DISPLAY_PROCESSED_EVENT",
        0x0e => "TICKER_EVENT",
        0x0f => "USB_EVENT",
        0x10 => "USB_EP_XFER_EVENT",
        0x11 => "BLE_CONNECTION_EVENT",
        0x12 => "UNSEC_CHUNK_EVENT",
        0x13 => "ACK_LINK_SPEED",
        0x14 => "BLUENRG_RECV_EVENT",
        0x15 => "STATUS_EVENT",
        0x16 => "CAPDU_EVENT",
        0x17 => "I2C_EVENT",
        0x18 => "BLE_RECV_EVENT",
        0x19 => "BOOTLOADER_RAPDU_EVENT",
        0x1a => "ITC_EVENT",
        0x1b => "POWER_BUTTON_EVENT",
        0x1c => "NFC_APDU_EVENT",
        0x1d => "QI_FLASH_CHECKSUM_EVENT",
        0x1e => "NFC_EVENT",
        0x1f => "SE_READY_TO_RECEIVE_START_SESSION_EVENT",
        0x31 => "MCU",
        0x32 => "UNSEC_CHUNK_READ",
        0x33 => "UNSEC_CHUNK_READ_EXT",
        0x34 => "NFC_POWER",
        0x38 => "BLE_SEND",
        0x3e => "SET_SCREEN_CONFIG",
        0x3f => "SET_LINK_PROP",
        0x40 => "BLUENRG_SEND",
        0x41 => "BLE_DEFINE_GENERIC_SETTING",
        0x42 => "BLE_DEFINE_SERVICE_SETTING",
        0x43 => "NFC_DEFINE_SERVICE_SETTING",
        0x44 => "BLE_RADIO_POWER",
        0x45 => "NFC_RADIO_POWER",
        0x46 => "SE_POWER_OFF",
        0x47 => "SPI_CS",
        0x48 => "BLE_SECURITY_DB",
        0x49 => "BATTERY_CHARGE",
        0x4a => "NFC_RAPDU",
        0x4b => "DEVICE_OFF",
        0x4c => "MORE_TIME",
        0x4d => "M24SR_C_APDU",
        0x4e => "SET_TICKER_INTERVAL",
        0x4f => "USB_CONFIG",
        0x50 => "USB_EP_PREPARE",
        0x51 => "SET_LED",
        0x52 => "REQUEST_STATUS",
        0x53 => "RAPDU",
        0x54 => "I2C_XFER",
        0x56 => "PLAY_TUNE",
        0x57 => "SET_SHIP_MODE",
        0x58 => "QI_FLASH",
        0x5a => "NBGL_SEND_SPECULOS_TEXT_LINE",
        0x5b => "SET_TOUCH_STATE",
        0x5c => "NBGL_SERIALIZED",
        0x5d => "ITC_CMD",
        0x5e => "DBG_SCREEN_DISPLAY_STATUS",
        0x5f => "PRINTF",
        0x60 => "GENERAL_STATUS",
        0x61 => "PAIRING_STATUS",
        0x62 => "BLE_READ_RESPONSE_STATUS",
        0x63 => "NFC_READ_RESPONSE_STATUS",
        0x64 => "BLE_NOTIFY_INDICATE_STATUS",
        0x65 => "SCREEN_DISPLAY_STATUS",
        0x66 => "PRINTF_STATUS",
        0x67 => "SET_LINK_SPEED",
        0x68 => "SCREEN_ANIMATION_STATUS",
        0x69 => "SCREEN_DISPLAY_RAW_STATUS",
        0x6a => "BOOTLOADER_CAPDU_STATUS",
        _ => return None,
    })
}

/// Sender of a packet with this tag.
pub fn direction(tag: u8) -> &'static str {
    if tag < 0x30 { "MCU→SE" } else { "SE→MCU" }
}

/// Meaning of an APDU status word.
pub fn status_word(sw: u16) -> Option<&'static str> {
    Some(match sw {
        0x9000 => "success",
        0x5515 => "device locked",
        0x6700 => "wrong length",
        0x6982 => "security status not satisfied",
        0x6985 => "conditions not satisfied (e.g. refused by the user)",
        0x6a80 => "invalid data",
        0x6a82 => "file or application not found",
        0x6a84 => "not enough memory",
        0x6a86 => "incorrect P1/P2",
        0x6b00 => "wrong parameters P1/P2",
        0x6d00 => "instruction not supported",
        0x6e00 => "class not supported",
        0x6f00 => "unknown error",
        _ => return None,
    })
}

/// One direction of an APDU transport being reassembled from chunks.
#[derive(Default)]
struct Reassembly {
    start: u64,
    expected: usize,
    next_seq: u16,
    data: Vec<u8>,
}

/// A transport the host's APDUs travel over.
#[derive(Clone, Copy, PartialEq)]
enum Transport {
    Usb(u8),
    Ble,
    /// Raw APDUs (CAPDU_EVENT / RAPDU, NFC).
    Raw,
}

impl Transport {
    fn label(self) -> String {
        match self {
            Transport::Usb(ep) => format!("USB ep{ep}"),
            Transport::Ble => "BLE".into(),
            Transport::Raw => "raw".into(),
        }
    }
}

/// Streaming SEPROXYHAL packet decoder fed with the link's characters.
#[derive(Default)]
pub struct Seph {
    /// Bytes of the packet being collected.
    buf: Vec<u8>,
    start: u64,
    last: u64,
    /// Bytes that don't start a packet (noise, resynchronization).
    junk: Vec<u8>,
    junk_span: (u64, u64),
    /// APDUs in progress, keyed by transport and direction (true = to the
    /// device).
    apdus: Vec<((Transport, bool), Reassembly)>,
    /// Last command per transport, to decode its response.
    last_cmd: Vec<(Transport, Vec<u8>)>,
    /// Packets decoded.
    packets: u64,
}

impl Seph {
    /// A decoder waiting for the first packet.
    pub fn new() -> Seph {
        Seph::default()
    }

    /// True when no packet is partly received.
    pub fn idle(&self) -> bool {
        self.buf.is_empty()
    }

    /// End of the last character received, if a packet is in progress.
    pub fn pending_since(&self) -> Option<u64> {
        (!self.buf.is_empty()).then_some(self.last)
    }

    /// Forgets partial packets and transfers (e.g. at a reset).
    pub fn reset(&mut self, out: &mut Vec<Annotation>) {
        self.flush(out);
        self.apdus.clear();
        self.last_cmd.clear();
    }

    /// Reports a partly received packet and pending noise.
    pub fn flush(&mut self, out: &mut Vec<Annotation>) {
        if !self.buf.is_empty() {
            let b = std::mem::take(&mut self.buf);
            let what = match b.len() {
                1 | 2 => format!("incomplete packet: {}", hex_trunc(&b, 16)),
                _ => format!(
                    "incomplete {} packet: {} of {} payload bytes",
                    tag_name(b[0]).unwrap_or("?"),
                    b.len() - 3,
                    be16(&b, 1).unwrap_or(0)
                ),
            };
            out.push(Annotation::new(self.start, self.last, Event::protocol(PROTO, what, Some(b.into()))));
        }
        self.flush_junk(out);
    }

    fn flush_junk(&mut self, out: &mut Vec<Annotation>) {
        if !self.junk.is_empty() {
            let j = std::mem::take(&mut self.junk);
            out.push(Annotation::new(
                self.junk_span.0,
                self.junk_span.1,
                Event::protocol(
                    PROTO,
                    format!("{} bytes outside packets: {}", j.len(), hex_trunc(&j, 16)),
                    Some(j.into()),
                ),
            ));
        }
    }

    /// Feeds one character spanning samples `start..end`.
    pub fn push(&mut self, b: u8, start: u64, end: u64, out: &mut Vec<Annotation>) {
        if self.buf.is_empty() {
            if tag_name(b).is_none() {
                if self.junk.is_empty() {
                    self.junk_span.0 = start;
                }
                self.junk.push(b);
                self.junk_span.1 = end;
                return;
            }
            self.flush_junk(out);
            self.start = start;
        }
        self.buf.push(b);
        self.last = end;
        if self.buf.len() < 3 {
            return;
        }
        let len = be16(&self.buf, 1).unwrap_or(0) as usize;
        if len > MAX_PAYLOAD {
            // Out of sync: the "tag" was noise. Keep the other bytes.
            let b = std::mem::take(&mut self.buf);
            self.junk_span.0 = self.start;
            self.junk.push(b[0]);
            self.junk_span.1 = self.start + 1;
            let span = (self.start, self.last);
            for &x in &b[1..] {
                self.push(x, span.0, span.1, out);
            }
            return;
        }
        if self.buf.len() == 3 + len {
            let p = std::mem::take(&mut self.buf);
            self.packet(&p, out);
        }
    }

    fn packet(&mut self, p: &[u8], out: &mut Vec<Annotation>) {
        self.packets += 1;
        let (tag, payload) = (p[0], &p[3..]);
        let (start, end) = (self.start, self.last);
        let mut apdus = Vec::new();
        let detail = self.describe(tag, payload, start, end, &mut apdus);
        let name = tag_name(tag).unwrap_or("?");
        let text = if detail.is_empty() {
            format!("{} {name}", direction(tag))
        } else {
            format!("{} {name}: {detail}", direction(tag))
        };
        out.push(Annotation::new(start, end, Event::protocol(PROTO, text, Some(p.into()))));
        out.extend(apdus);
    }

    /// Adds a chunk of Ledger transport framing to an APDU in progress;
    /// reports the APDU when complete.
    fn chunk(&mut self, t: Transport, to_device: bool, frame: &[u8], start: u64, end: u64, out: &mut Vec<Annotation>) -> String {
        // USB frames start with a channel id; BLE frames don't.
        let (chan, f) = match t {
            Transport::Usb(_) if frame.len() >= 2 => (Some(be16(frame, 0).unwrap_or(0)), &frame[2..]),
            _ => (None, frame),
        };
        let ch = chan.map(|c| format!("channel {c:#06x} ")).unwrap_or_default();
        let Some(&tag) = f.first() else {
            return "empty frame".into();
        };
        match tag {
            0x05 if f.len() >= 3 => {}
            0x00 => return format!("{ch}get protocol version"),
            0x01 => return format!("{ch}allocate channel"),
            0x02 => return format!("{ch}ping"),
            0x03 => return format!("{ch}abort"),
            0x08 => return format!("{ch}MTU {}", hex_trunc(&f[1..], 8)),
            _ => return format!("{ch}not Ledger APDU framing: {}", hex_trunc(frame, 16)),
        }
        let seq = be16(f, 1).unwrap_or(0);
        let key = (t, to_device);
        let i = match self.apdus.iter().position(|a| a.0 == key) {
            Some(i) => i,
            None => {
                self.apdus.push((key, Reassembly::default()));
                self.apdus.len() - 1
            }
        };
        let r = &mut self.apdus[i].1;
        let mut note = String::new();
        let body = if seq == 0 {
            if !r.data.is_empty() {
                note = format!(", previous APDU abandoned after {} of {} bytes", r.data.len(), r.expected);
            }
            let Some(total) = be16(f, 3) else {
                return format!("{ch}truncated first chunk");
            };
            *r = Reassembly {
                start,
                expected: total as usize,
                next_seq: 0,
                data: Vec::new(),
            };
            &f[5.min(f.len())..]
        } else {
            &f[3..]
        };
        if seq != r.next_seq {
            let m = format!("{ch}chunk {seq} out of sequence (expected {})", r.next_seq);
            r.data.clear();
            return m;
        }
        r.next_seq += 1;
        let want = r.expected - r.data.len();
        r.data.extend_from_slice(&body[..want.min(body.len())]);
        let summary = format!("{ch}APDU chunk {seq}, {}/{} bytes{note}", r.data.len(), r.expected);
        if r.data.len() == r.expected {
            let r = std::mem::take(&mut self.apdus[i].1);
            out.push(self.apdu(t, to_device, &r.data, r.start, end));
        }
        summary
    }

    fn describe(&mut self, tag: u8, p: &[u8], start: u64, end: u64, out: &mut Vec<Annotation>) -> String {
        match tag {
            0x01 => session_start(p),
            0x05 => p.first().map_or(String::new(), |&m| {
                let mut b = Vec::new();
                if m & 2 != 0 {
                    b.push("left");
                }
                if m & 4 != 0 {
                    b.push("right");
                }
                let which = if b.is_empty() { "released".to_string() } else { b.join("+") };
                format!("{which} (mask {m:#04x})")
            }),
            0x0c if p.len() >= 5 => format!(
                "{} at {},{}",
                match p[0] {
                    1 => "touch",
                    2 => "release",
                    _ => "finger",
                },
                be16(p, 1).unwrap_or(0),
                be16(p, 3).unwrap_or(0)
            ),
            0x0e => match be32(p, 0) {
                Some(ms) => format!("{} ms since power-on", ms),
                None => String::new(),
            },
            0x0f => p.first().map_or(String::new(), |&t| {
                match t {
                    1 => "reset",
                    2 => "start of frame",
                    4 => "suspended",
                    8 => "resumed",
                    _ => "?",
                }
                .to_string()
            }),
            0x10 => self.usb_xfer(p, start, end, out),
            0x12 => format!("{} bytes: {}", p.len(), hex_trunc(p, 24)),
            0x15 => status_event(p),
            0x16 => {
                out.push(self.apdu(Transport::Raw, true, p, start, end));
                format!("{} bytes", p.len())
            }
            0x18 => {
                let mut write = None;
                let s = hci::describe_packet(p, &mut write);
                match write {
                    Some((attr, _, data)) if data.first() == Some(&0x05) => {
                        let c = self.chunk(Transport::Ble, true, &data, start, end, out);
                        format!("{s} — {c} (attribute {attr:#06x})")
                    }
                    _ => s,
                }
            }
            0x1a => match p.split_first() {
                Some((&k, rest)) => format!(
                    "type {k:#04x}{}",
                    if rest.is_empty() {
                        String::new()
                    } else {
                        format!(", {}", hex_trunc(rest, 24))
                    }
                ),
                None => String::new(),
            },
            0x31 => mcu(p),
            0x32 | 0x33 => hex_trunc(p, 16),
            0x34 => p.first().map_or(String::new(), |&t| {
                match t {
                    0 => "off",
                    1 => "on (card emulation)",
                    2 => "on (reader)",
                    _ => "?",
                }
                .into()
            }),
            0x38 => {
                let mut notify = None;
                let s = hci::describe_command(p, &mut notify);
                match notify {
                    Some((_, value)) if value.first() == Some(&0x05) => {
                        let c = self.chunk(Transport::Ble, false, &value, start, end, out);
                        format!("{s} — {c}")
                    }
                    _ => s,
                }
            }
            0x3e if p.len() >= 2 => format!(
                "flags {:#04x}{}, brightness {}%",
                p[0],
                if p[0] & 0x80 != 0 { " (power on)" } else { "" },
                p[1]
            ),
            0x44 => p.first().map_or(String::new(), |&a| {
                let mut v = Vec::new();
                if a & 0x02 != 0 {
                    v.push("on");
                }
                if a & 0x04 != 0 {
                    v.push("wipe database");
                }
                if a & 0x40 != 0 {
                    v.push("factory test");
                }
                if v.is_empty() {
                    v.push("off");
                }
                format!("{} ({a:#04x})", v.join(", "))
            }),
            0x4b => p
                .first()
                .map_or(String::new(), |&c| if c != 0 { "critical battery".into() } else { String::new() }),
            0x4e => be16(p, 0).map_or(String::new(), |ms| format!("every {ms} ms")),
            0x4f => usb_config(p),
            0x50 => self.usb_prepare(p, start, end, out),
            0x53 => {
                out.push(self.apdu(Transport::Raw, false, p, start, end));
                format!("{} bytes", p.len())
            }
            0x56 => p.first().map_or(String::new(), |t| format!("tune {t}")),
            0x5b => p
                .first()
                .map_or(String::new(), |&e| if e != 0 { "enable".into() } else { "disable".into() }),
            0x5d => itc_cmd(p),
            0x5f => format!("\"{}\"", text(p).escape_debug()),
            0x60 => match be16(p, 0) {
                Some(0) => "last command".into(),
                Some(s) => format!("status {s:#06x}"),
                None => String::new(),
            },
            0x65 | 0x5e => bagl(p),
            0x67 if p.len() >= 2 => format!("{} MHz, etu {}", p[0], p[1]),
            _ if p.is_empty() => String::new(),
            _ => format!("{} bytes: {}", p.len(), hex_trunc(p, 24)),
        }
    }

    fn usb_xfer(&mut self, p: &[u8], start: u64, end: u64, out: &mut Vec<Annotation>) -> String {
        if p.len() < 3 {
            return hex_trunc(p, 16);
        }
        let (ep, kind, len) = (p[0], p[1], p[2] as usize);
        let data = &p[3..];
        let n = &data[..len.min(data.len())];
        match kind {
            0x01 => format!("ep{} SETUP {}", ep & 0x7f, setup(n)),
            0x02 => format!("ep{} IN done ({len} bytes)", ep & 0x7f),
            0x04 | 0x05 => {
                let s = format!("ep{} OUT {len} bytes", ep & 0x7f);
                if ep & 0x7f != 0 && len > 0 {
                    // OUT data on ep0 (control) has no Ledger framing.
                    let c = self.chunk(Transport::Usb(ep & 0x7f), true, data, start, end, out);
                    format!("{s} — {c}")
                } else if len > 0 {
                    format!("{s}: {}", hex_trunc(n, 16))
                } else {
                    s
                }
            }
            k => format!("ep{} kind {k:#04x}, {len} bytes: {}", ep & 0x7f, hex_trunc(n, 16)),
        }
    }

    fn usb_prepare(&mut self, p: &[u8], start: u64, end: u64, out: &mut Vec<Annotation>) -> String {
        if p.len() < 3 {
            return hex_trunc(p, 16);
        }
        let (ep, dir, len) = (p[0], p[1], p[2] as usize);
        let data = &p[3..];
        let epn = ep & 0x7f;
        match dir {
            0x10 => format!("ep{epn} expect SETUP"),
            0x20 | 0x21 => {
                let pad = if dir == 0x21 { " (zero padded to 64)" } else { "" };
                let s = format!("ep{epn} IN {len} bytes{pad}");
                if epn == 0 {
                    if data.is_empty() {
                        format!("{s} (status stage)")
                    } else {
                        format!("{s}: {}", descriptor(data))
                    }
                } else if !data.is_empty() {
                    let c = self.chunk(Transport::Usb(epn), false, data, start, end, out);
                    format!("{s} — {c}")
                } else {
                    s
                }
            }
            0x30 => format!("ep{epn} ready to receive {len} bytes"),
            0x40 => format!("ep{epn} stall"),
            0x80 => format!("ep{epn} unstall"),
            d => format!("ep{epn} direction {d:#04x}, {len} bytes: {}", hex_trunc(data, 16)),
        }
    }
}

impl Seph {
    /// An APDU annotation: a command to the device or its response.
    fn apdu(&mut self, t: Transport, to_device: bool, b: &[u8], start: u64, end: u64) -> Annotation {
        if to_device {
            self.last_cmd.retain(|c| c.0 != t);
            if b.len() >= 4 {
                self.last_cmd.push((t, b.to_vec()));
            }
            return apdu(t, true, b, None, start, end);
        }
        let cmd = self.last_cmd.iter().find(|c| c.0 == t).map(|c| c.1.as_slice());
        apdu(t, false, b, cmd, start, end)
    }
}

/// Known instructions: the OS's default class (SDK `sdk_apdu_commands.h`)
/// and the dashboard's (as `ledgerctl` uses them; apps may reuse CLA 0xE0
/// for their own instructions).
pub fn instruction(cla: u8, ins: u8) -> Option<&'static str> {
    Some(match (cla, ins) {
        (0xb0, 0x01) => "GET_APP_NAME_AND_VERSION",
        (0xb0, 0x02) => "GET_SEED_COOKIE",
        (0xb0, 0x06) => "LOAD_CERTIFICATE",
        (0xb0, 0x57) => "STACK_CONSUMPTION",
        (0xb0, 0xa7) => "APP_EXIT",
        (0xe0, 0x00) => "dashboard SECUINS (secure channel)",
        (0xe0, 0x01) => "dashboard GET_VERSION",
        (0xe0, 0x04) => "dashboard VALIDATE_TARGET_ID",
        (0xe0, 0x50) => "dashboard INITIALIZE_AUTHENTICATION",
        (0xe0, 0x51) => "dashboard VALIDATE_CERTIFICATE",
        (0xe0, 0x52) => "dashboard GET_CERTIFICATE",
        (0xe0, 0x53) => "dashboard MUTUAL_AUTHENTICATE",
        (0xe0, 0xc0) => "dashboard ENDORSE_SET_START",
        (0xe0, 0xc2) => "dashboard ENDORSE_SET_COMMIT",
        (0xe0, 0xd0) => "dashboard ONBOARD",
        (0xe0, 0xd8) => "dashboard RUN_APP",
        _ => return None,
    })
}

/// Device model from a target id (`ledgerctl`'s table).
pub fn target_name(id: u32) -> Option<&'static str> {
    Some(match id {
        0x3110_0002..=0x3110_0004 => "Nano S",
        0x3100_0002 | 0x3101_0004 => "Blue",
        0x3300_0004 => "Nano X",
        0x3310_0004 => "Nano S Plus",
        0x3320_0004 => "Stax",
        0x3330_0004 => "Flex",
        0x3340_0004 => "Apex P",
        0x3350_0004 => "Apex M",
        _ => return None,
    })
}

/// Length-prefixed strings from `b`, as far as they parse.
fn pascal_strings(mut b: &[u8]) -> (Vec<Vec<u8>>, &[u8]) {
    let mut v = Vec::new();
    while let Some((&n, r)) = b.split_first() {
        if r.len() < n as usize {
            break;
        }
        v.push(r[..n as usize].to_vec());
        b = &r[n as usize..];
    }
    (v, b)
}

/// A certificate of the dashboard's authentication (`ledgerctl`): length-
/// prefixed header (device certificate only), public key and signature.
fn certificate(fields: &[Vec<u8>]) -> String {
    let key = |k: &[u8]| {
        if k.len() == 65 && k[0] == 4 {
            format!("public key {}", hex_trunc(k, 65).replace(' ', ""))
        } else {
            format!("{}-byte key {}", k.len(), hex_trunc(k, 65).replace(' ', ""))
        }
    };
    let sig = |g: &[u8]| {
        let der = g.first() == Some(&0x30) && g.get(1).is_some_and(|&n| n as usize + 2 == g.len());
        format!("signature {} bytes{}", g.len(), if der { " (DER ECDSA)" } else { "" })
    };
    match fields {
        [h, k, g] => {
            let header = if h.is_empty() {
                "no header".to_string()
            } else {
                format!("header {}", hex_trunc(h, 32).replace(' ', ""))
            };
            format!("{header}, {}, {}", key(k), sig(g))
        }
        [k, g] => format!("{}, {}", key(k), sig(g)),
        _ => fields.iter().map(|f| hex_trunc(f, 16)).collect::<Vec<_>>().join(" | "),
    }
}

/// What a command's data says, for known commands.
fn command_meaning(c: &[u8]) -> Option<String> {
    let data = c.get(5..).unwrap_or(&[]);
    match (c[0], c[1]) {
        (0xe0, 0x04) if data.len() == 4 => {
            let id = be32(data, 0)?;
            Some(format!(
                "target {id:#010x}{}",
                target_name(id).map(|n| format!(" ({n})")).unwrap_or_default()
            ))
        }
        (0xe0, 0x50) if data.len() == 8 => Some(format!("server nonce {}", hex_trunc(data, 8).replace(' ', ""))),
        // The server's certificate chain, the last one (P1 0x80) being its
        // ephemeral key, signed over 0x11 ‖ server nonce ‖ device nonce ‖ key.
        (0xe0, 0x51) => {
            let (f, _) = pascal_strings(data);
            let what = if c[2] & 0x80 != 0 {
                "server ephemeral key (last certificate)"
            } else {
                "server certificate"
            };
            Some(format!("{what}: {}", certificate(&f)))
        }
        _ => None,
    }
}

/// What a response's data says, for known commands.
fn response_meaning(c: &[u8], d: &[u8]) -> Option<String> {
    match (c[0], c[1]) {
        (0xe0, 0x50) if d.len() >= 12 => Some(format!(
            "{}, device nonce {}",
            hex_trunc(&d[..4], 4).replace(' ', ""),
            hex_trunc(&d[4..12], 8).replace(' ', "")
        )),
        // The device's chain: its certificate (signed by Ledger over 0x02 ‖
        // header ‖ key), then its ephemeral key (signed by the device key
        // over 0x12 ‖ device nonce ‖ server nonce ‖ key).
        (0xe0, 0x52) => {
            let (f, _) = pascal_strings(d);
            let what = if c[2] & 0x80 != 0 {
                "device ephemeral key"
            } else {
                "device certificate"
            };
            Some(format!("{what}: {}", certificate(&f)))
        }
        (0xb0, 0x01) if d.first() == Some(&1) => {
            let (v, _) = pascal_strings(&d[1..]);
            let name = v.first().map(|x| text(x))?;
            let version = v.get(1).map(|x| text(x)).unwrap_or_default();
            Some(format!("running \"{name}\" {version}"))
        }
        // target id, SE version, flags (length 4, little endian), MCU
        // version, MCU bootloader version, ...
        (0xe0, 0x01) if d.len() >= 5 => {
            let id = be32(d, 0)?;
            let (v, _) = pascal_strings(&d[4..]);
            let mut s = format!(
                "target {id:#010x}{}",
                target_name(id).map(|n| format!(" ({n})")).unwrap_or_default()
            );
            if let Some(se) = v.first() {
                s += &format!(", SE {}", text(se));
            }
            if let Some(f) = v.get(1).filter(|f| f.len() == 4) {
                let flags = u32::from_le_bytes([f[0], f[1], f[2], f[3]]);
                let names: Vec<&str> = [
                    (1, "recovery mode"),
                    (2, "signed MCU"),
                    (4, "onboarded"),
                    (8, "trust issuer"),
                    (16, "trust custom CA"),
                    (32, "HSM initialized"),
                    (128, "PIN validated"),
                ]
                .iter()
                .filter(|(b, _)| flags & b != 0)
                .map(|(_, n)| *n)
                .collect();
                s += &format!(", flags {flags:#x} ({})", names.join(", "));
            }
            if let Some(m) = v.get(2) {
                s += &format!(", MCU {}", text(m));
            }
            if let Some(m) = v.get(3) {
                s += &format!(", MCU bootloader {}", text(m));
            }
            Some(s)
        }
        _ => None,
    }
}

/// An APDU annotation: a command to the device or its response (`cmd`:
/// the last command on that transport).
fn apdu(t: Transport, to_device: bool, b: &[u8], cmd: Option<&[u8]>, start: u64, end: u64) -> Annotation {
    let text = if to_device {
        match b {
            [cla, ins, p1, p2, rest @ ..] => {
                let mut s = format!("C-APDU ({}): CLA {cla:02x} INS {ins:02x} P1 {p1:02x} P2 {p2:02x}", t.label());
                if let Some(n) = instruction(*cla, *ins) {
                    s += &format!(" {n}");
                }
                match rest {
                    [] => {}
                    [le] => s += &format!(", Le {le}"),
                    [lc, data @ ..] => {
                        s += &format!(", Lc {lc}");
                        if !data.is_empty() {
                            s += &format!(": {}", hex_trunc(data, 32));
                        }
                    }
                }
                if let Some(m) = command_meaning(b) {
                    s += &format!(" — {m}");
                }
                s
            }
            _ => format!("C-APDU ({}): short, {}", t.label(), hex_trunc(b, 16)),
        }
    } else if b.len() >= 2 {
        let sw = be16(b, b.len() - 2).unwrap_or(0);
        let meaning = status_word(sw).map(|m| format!(" ({m})")).unwrap_or_default();
        let data = &b[..b.len() - 2];
        let to = cmd
            .and_then(|c| instruction(c[0], c[1]))
            .map(|n| format!(" to {n}"))
            .unwrap_or_default();
        if data.is_empty() {
            let done = if sw == 0x9000 && cmd.is_some_and(|c| c[..2] == [0xe0, 0x53]) {
                " — secure channel established"
            } else {
                ""
            };
            format!("R-APDU ({}){to}: SW {sw:04x}{meaning}{done}", t.label())
        } else {
            let txt = match cmd.and_then(|c| response_meaning(c, data)) {
                Some(m) => format!(" — {m}"),
                None if printable(data) => format!(" \"{}\"", text(data)),
                None => String::new(),
            };
            format!(
                "R-APDU ({}){to}: SW {sw:04x}{meaning}, {} bytes: {}{txt}",
                t.label(),
                data.len(),
                hex_trunc(data, 32)
            )
        }
    } else {
        format!("R-APDU ({}): short, {}", t.label(), hex_trunc(b, 16))
    };
    Annotation::new(start, end, Event::protocol("APDU", text, Some(b.into())))
}

fn session_start(p: &[u8]) -> String {
    let Some(&kind) = p.first() else { return String::new() };
    let mut s = match kind {
        0x00 => "normal".to_string(),
        k => {
            let mut v = Vec::new();
            if k & 0x02 != 0 {
                v.push("recovery");
            }
            if k & 0x04 != 0 {
                v.push("flashback");
            }
            if k & 0x08 != 0 {
                v.push("boot menu");
            }
            if v.is_empty() { format!("type {k:#04x}") } else { v.join("+") }
        }
    };
    let Some(f) = be32(p, 1) else { return s };
    let mut feats = Vec::new();
    for (bit, name) in [
        (0x01, "USB"),
        (0x02, "BLE"),
        (0x04, "touch"),
        (0x08, "battery"),
        (0x10, "button"),
        (0x1000_0000, "MCU secure instruction set"),
        (0x2000_0000, "MCU bootloader"),
    ] {
        if f & bit != 0 {
            feats.push(name.to_string());
        }
    }
    match f & 0xf00 {
        0 => {}
        0x100 => feats.push("big screen".into()),
        0x300 => feats.push("SSD1312 screen".into()),
        x => feats.push(format!("screen {x:#05x}")),
    }
    if f & 0xf000 != 0 {
        feats.push(format!("HW version {}", f >> 12 & 15));
    }
    s += &format!(", features {f:#010x} ({})", feats.join(", "));
    // Then length-prefixed fields: SEPROXYHAL version (the bootloader puts
    // its own version there), MCU bootloader load key id, and from the
    // firmware only, MCU bootloader version and MCU SEPH sign key id.
    // Newer firmware appends more.
    let bootloader = f & 0x2000_0000 != 0;
    let names: &[&str] = if bootloader {
        &["bootloader version", "load key id"]
    } else {
        &[
            "SEPROXYHAL version",
            "MCU bootloader load key id",
            "MCU bootloader version",
            "MCU SEPH sign key id",
        ]
    };
    let mut rest = &p[5..];
    let mut fields = Vec::new();
    let mut i = 0;
    while let Some((&n, r)) = rest.split_first() {
        if r.len() < n as usize {
            fields.push(format!("{} trailing bytes: {}", rest.len(), hex_trunc(rest, 24)));
            break;
        }
        let v = &r[..n as usize];
        let val = if v.is_empty() {
            "blank".to_string()
        } else if printable(v) {
            format!("\"{}\"", text(v))
        } else {
            hex_trunc(v, 24).replace(' ', "")
        };
        fields.push(match names.get(i) {
            Some(name) => format!("{name} {val}"),
            None => format!("extra field {val}"),
        });
        i += 1;
        rest = &r[n as usize..];
    }
    if !fields.is_empty() {
        s += &format!("; {}", fields.join(", "));
    }
    s
}

fn status_event(p: &[u8]) -> String {
    let Some(f) = be32(p, 0) else { return hex_trunc(p, 24) };
    let mut v = Vec::new();
    for (bit, name) in [
        (0x01, "charging"),
        (0x02, "USB on"),
        (0x04, "BLE on"),
        (0x08, "USB powered"),
        (0x10, "charging issue"),
        (0x20, "temperature issue"),
        (0x40, "battery issue"),
        (0x80, "gas gauge issue"),
    ] {
        if f & bit != 0 {
            v.push(name);
        }
    }
    let mut s = format!("flags {f:#010x} ({})", if v.is_empty() { "none".into() } else { v.join(", ") });
    // Layout documented by the Nano X SDK: flags, backlight %, LED ARGB,
    // battery mV, battery %. Firmware may append more.
    if p.len() >= 14 {
        s += &format!(
            ", backlight {}%, LED {:#010x}, battery {} mV {}%",
            p[4],
            be32(p, 5).unwrap_or(0),
            be32(p, 9).unwrap_or(0),
            p[13]
        );
        if p.len() > 14 {
            s += &format!(", then {}", hex_trunc(&p[14..], 16));
        }
    } else if p.len() > 4 {
        s += &format!(", {}", hex_trunc(&p[4..], 16));
    }
    s
}

fn mcu(p: &[u8]) -> String {
    match p.split_first() {
        Some((0, _)) => "go to bootloader".into(),
        Some((1, _)) => "lock".into(),
        Some((2, _)) => "protect".into(),
        Some((3, r)) => {
            // BD address and device name, each length-prefixed.
            let mut s = "set BLE identity".to_string();
            if let Some((&n, r)) = r.split_first()
                && r.len() >= n as usize
            {
                s += &format!(", address {}", hci::bd_addr(&r[..n as usize]));
                if let Some((&m, name)) = r[n as usize..].split_first() {
                    s += &format!(", name \"{}\"", text(&name[..(m as usize).min(name.len())]));
                }
            }
            s
        }
        Some((t, r)) => format!("type {t:#04x} {}", hex_trunc(r, 16)),
        None => String::new(),
    }
}

fn itc_cmd(p: &[u8]) -> String {
    let Some((&t, r)) = p.split_first() else { return String::new() };
    let name = match t {
        0x00 => "BLE stop",
        0x01 => "BLE start",
        0x02 => "BLE reset pairings",
        0x03 => "BLE name changed",
        0x10 => "NFC stop",
        0x11 => "NFC start (card emulation)",
        0x12 => "NFC start (reader)",
        0x20 => "UX redisplay",
        0x21 => "UX accept BLE pairing",
        0x22 => "UX ask BLE pairing",
        0x23 => "UX BLE pairing status",
        _ => return format!("type {t:#04x} {}", hex_trunc(r, 16)),
    };
    if r.is_empty() {
        name.into()
    } else {
        format!("{name} {}", hex_trunc(r, 16))
    }
}

fn usb_config(p: &[u8]) -> String {
    match p.split_first() {
        Some((1, _)) => "connect".into(),
        Some((2, _)) => "disconnect".into(),
        Some((3, r)) => format!("address {}", r.first().copied().unwrap_or(0)),
        Some((4, r)) => {
            let n = r.first().copied().unwrap_or(0) as usize;
            let eps: Vec<String> = r[1.min(r.len())..]
                .chunks(3)
                .take(n)
                .map(|e| {
                    let ty = match e.get(1) {
                        Some(0) => "disabled",
                        Some(1) => "control",
                        Some(2) => "interrupt",
                        Some(3) => "bulk",
                        Some(4) => "isochronous",
                        _ => "?",
                    };
                    let addr = e.first().copied().unwrap_or(0);
                    format!(
                        "ep{} {} {ty} {}",
                        addr & 0x7f,
                        if addr & 0x80 != 0 { "IN" } else { "OUT" },
                        e.get(2).copied().unwrap_or(0)
                    )
                })
                .collect();
            format!("endpoints: {}", eps.join(", "))
        }
        Some((5, r)) => format!(
            "APDU proxy on ep{} {}",
            r.first().map_or(0, |e| e & 0x7f),
            if r.get(1) == Some(&1) { "on" } else { "off" }
        ),
        Some((t, r)) => format!("type {t:#04x} {}", hex_trunc(r, 16)),
        None => String::new(),
    }
}

/// A USB SETUP packet.
fn setup(b: &[u8]) -> String {
    if b.len() < 8 {
        return hex_trunc(b, 16);
    }
    let (rt, req) = (b[0], b[1]);
    let value = u16::from_le_bytes([b[2], b[3]]);
    let index = u16::from_le_bytes([b[4], b[5]]);
    let len = u16::from_le_bytes([b[6], b[7]]);
    let dir = if rt & 0x80 != 0 { "IN" } else { "OUT" };
    let recipient = ["device", "interface", "endpoint", "other"].get((rt & 3) as usize).unwrap_or(&"?");
    let what = match (rt >> 5 & 3, req) {
        (0, 0) => "GET_STATUS".to_string(),
        (0, 1) => "CLEAR_FEATURE".into(),
        (0, 3) => "SET_FEATURE".into(),
        (0, 5) => return format!("SET_ADDRESS {value}"),
        (0, 6) => {
            let ty = match value >> 8 {
                1 => "device".to_string(),
                2 => "configuration".into(),
                3 => format!("string {}", value & 0xff),
                6 => "device qualifier".into(),
                0x0f => "BOS".into(),
                0x21 => "HID".into(),
                0x22 => "HID report".into(),
                t => format!("type {t:#04x}"),
            };
            return format!("GET_DESCRIPTOR {ty}, {len} bytes");
        }
        (0, 8) => "GET_CONFIGURATION".into(),
        (0, 9) => return format!("SET_CONFIGURATION {value}"),
        (0, 10) => "GET_INTERFACE".into(),
        (0, 11) => "SET_INTERFACE".into(),
        (1, 0x01) => "class GET_REPORT".into(),
        (1, 0x09) => "class SET_REPORT".into(),
        (1, 0x0a) => "class SET_IDLE".into(),
        (1, 0x0b) => "class SET_PROTOCOL".into(),
        (1, r) => format!("class request {r:#04x}"),
        (2, r) => format!("vendor request {r:#04x}"),
        (_, r) => format!("request {r:#04x}"),
    };
    format!("{what} ({dir}, {recipient}) value {value:#06x} index {index:#06x} length {len}")
}

/// A descriptor (or other data) sent on the control endpoint.
fn descriptor(d: &[u8]) -> String {
    match (d.first(), d.get(1)) {
        (Some(_), Some(1)) if d.len() >= 18 => format!(
            "device descriptor: USB {:x}.{:02x}, VID {:04x} PID {:04x}, {} configuration(s)",
            d[3],
            d[2],
            u16::from_le_bytes([d[8], d[9]]),
            u16::from_le_bytes([d[10], d[11]]),
            d[17]
        ),
        (Some(_), Some(2)) if d.len() >= 9 => format!(
            "configuration descriptor: {} bytes total, {} interface(s), max power {} mA",
            u16::from_le_bytes([d[2], d[3]]),
            d[4],
            d[8] as u32 * 2
        ),
        (Some(&n), Some(3)) => {
            let units: Vec<u16> = d[2..(n as usize).min(d.len())]
                .chunks(2)
                .filter(|c| c.len() == 2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect();
            if (n as usize) > d.len() {
                format!("string descriptor header ({n} bytes in all)")
            } else if d.len() == 4 && n == 4 {
                format!("string descriptor: language {:#06x}", units.first().unwrap_or(&0))
            } else {
                format!("string descriptor \"{}\"", String::from_utf16_lossy(&units))
            }
        }
        (Some(_), Some(0x0f)) if d.len() >= 5 => format!("BOS descriptor, {} capabilities", d[4]),
        _ => hex_trunc(d, 24),
    }
}

/// A BAGL display element (`SCREEN_DISPLAY_STATUS`, little endian).
fn bagl(p: &[u8]) -> String {
    if p.len() < 28 {
        return format!("{} bytes: {}", p.len(), hex_trunc(p, 24));
    }
    let le = |i: usize| i16::from_le_bytes([p[i], p[i + 1]]);
    let ty = match p[0] {
        0 => "none".to_string(),
        1 => "button".into(),
        2 => "label".into(),
        3 => "rectangle".into(),
        4 => "line".into(),
        5 => "icon".into(),
        6 => "circle".into(),
        7 => "label line".into(),
        t => format!("type {t}"),
    };
    let mut s = format!("{ty} id {} at {},{} size {}x{}", p[1], le(2), le(4), le(6), le(8));
    let txt = &p[28..];
    if printable(txt) {
        s += &format!(" \"{}\"", text(txt));
    } else if !txt.is_empty() {
        s += &format!(", {} bytes: {}", txt.len(), hex_trunc(txt, 16));
    }
    s
}

/// The text between `"` after `label`, e.g. `"2.39.0"` in
/// `SEPROXYHAL version "2.39.0"`.
fn quoted_after<'a>(text: &'a str, label: &str) -> Option<&'a str> {
    let rest = &text[text.find(label)? + label.len()..];
    let rest = rest.trim_start().strip_prefix('"')?;
    Some(&rest[..rest.find('"')?])
}

/// What an ATR's historical bytes say on a Ledger SE: its version and
/// target id, as length-prefixed fields.
fn atr_identity(text: &str) -> Option<String> {
    let hex = text.split("historical bytes ").nth(1)?.split(" '").next()?;
    let b: Vec<u8> = hex.split(' ').map(|h| u8::from_str_radix(h, 16).ok()).collect::<Option<_>>()?;
    let (f, _) = pascal_strings(&b);
    let ver = f.first().filter(|v| printable(v))?;
    let mut s = format!("SE {}", self::text(ver));
    if let Some(id) = f.get(1).filter(|i| i.len() == 4) {
        let id = u32::from_be_bytes([id[0], id[1], id[2], id[3]]);
        s += &format!(
            ", target {id:#010x}{}",
            target_name(id).map(|n| format!(" ({n})")).unwrap_or_default()
        );
    }
    Some(s)
}

/// A timeline entry: what happened from `start` (to `end` for a run of
/// events), in samples.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Milestone {
    /// First sample.
    pub start: u64,
    /// Start of the last event of a run.
    pub end: Option<u64>,
    /// Description.
    pub text: String,
}

/// A run of similar events summarized in one timeline entry.
struct Run {
    start: u64,
    end: u64,
    count: usize,
    bytes: usize,
    first: u32,
    last: u32,
}

/// Milestones of a SEPROXYHAL capture, from the decoder's events
/// (`(start sample, text, data)`, in order): SE boots and link rate, MCU
/// sessions and versions, SE commands to the MCU, power, USB and BLE state,
/// battery, lock state, app and versions reported to the host, the
/// authentication, secure channel transfers, progress reports, line
/// problems.
pub fn timeline<'a>(events: impl IntoIterator<Item = (u64, &'a str, Option<&'a [u8]>)>) -> Vec<Milestone> {
    let mut out: Vec<Milestone> = Vec::new();
    let mut boots = 0;
    let mut usb: Option<bool> = None;
    let mut ble_adv: Option<bool> = None;
    let mut locked = false;
    let mut app = String::new();
    let mut versions = String::new();
    let mut power = String::new();
    let mut secure: Option<Run> = None;
    let mut progress: Option<Run> = None;
    let flush_secure = |r: &mut Option<Run>, out: &mut Vec<Milestone>| {
        if let Some(r) = r.take() {
            out.push(Milestone {
                start: r.start,
                end: Some(r.end),
                text: format!("{} secure channel commands, {} bytes of data", r.count, r.bytes),
            });
        }
    };
    let flush_progress = |r: &mut Option<Run>, out: &mut Vec<Milestone>| {
        if let Some(r) = r.take() {
            out.push(Milestone {
                start: r.start,
                end: Some(r.end),
                text: format!("progress reports from the MCU, {} → {} ({} events)", r.first, r.last, r.count),
            });
        }
    };
    for (at, text, data) in events {
        let mut push = |text: String| {
            out.push(Milestone {
                start: at,
                end: None,
                text,
            })
        };
        if text.starts_with("ATR") {
            flush_secure(&mut secure, &mut out);
            flush_progress(&mut progress, &mut out);
            boots += 1;
            locked = false;
            let id = atr_identity(text).map(|s| format!(": {s}")).unwrap_or_default();
            out.push(Milestone {
                start: at,
                end: None,
                text: format!("SE boot {boots}{id}"),
            });
        } else if let Some(r) = text.strip_prefix("── new rate: ") {
            push(format!("link speed: {}", r.trim_end_matches(" ──")));
        } else if text.contains("SESSION_START_EVENT:") {
            if let Some(v) = quoted_after(text, "bootloader version") {
                if text.contains("SEPROXYHAL version") {
                    let sv = quoted_after(text, "SEPROXYHAL version").unwrap_or("?");
                    push(format!("MCU firmware session: SEPROXYHAL version {sv}, MCU bootloader {v}"));
                } else {
                    push(format!("MCU bootloader session: bootloader {v}"));
                }
            } else {
                push("MCU session start".into());
            }
        } else if let Some(r) = text.strip_prefix("SE→MCU MCU: ") {
            push(format!("SE asks the MCU: {r}"));
        } else if text.starts_with("SE→MCU SE_POWER_OFF") {
            flush_secure(&mut secure, &mut out);
            out.push(Milestone {
                start: at,
                end: None,
                text: "SE power off".into(),
            });
        } else if text.starts_with("SE→MCU DEVICE_OFF") {
            push("device off".into());
        } else if text.starts_with("SE→MCU USB_CONFIG: connect") || text.starts_with("SE→MCU USB_CONFIG: disconnect") {
            let on = text.ends_with("connect") && !text.ends_with("disconnect");
            if usb != Some(on) {
                usb = Some(on);
                push(format!("USB {}", if on { "connect" } else { "disconnect" }));
            }
        } else if text.contains("SETUP SET_CONFIGURATION") {
            push(format!(
                "USB enumerated by the host ({})",
                text.rsplit("SETUP ").next().unwrap_or("")
            ));
        } else if text.contains("HCI command aci_gap_set_discoverable") || text.contains("HCI command aci_gap_set_non_discoverable") {
            let on = text.contains("set_discoverable");
            if ble_adv != Some(on) {
                ble_adv = Some(on);
                push(format!("BLE advertising {}", if on { "on" } else { "off" }));
            }
        } else if text.contains("HCI LE meta: connection complete") || text.contains("HCI LE meta: enhanced connection complete") {
            push(format!("BLE connection: {}", text.split("complete ").nth(1).unwrap_or("")));
        } else if text.contains("HCI disconnection complete") {
            push("BLE disconnection".into());
        } else if text.starts_with("MCU→SE STATUS_EVENT:") {
            // Flags and battery, reported when the flags change.
            let flags = text.split(" (").nth(1).and_then(|f| f.split(')').next()).unwrap_or("");
            if flags != power {
                power = flags.to_string();
                // "battery 3817 mV 21%" (not the "battery issue" flag).
                let battery = text
                    .split(", battery ")
                    .skip(1)
                    .find(|b| b.starts_with(|c: char| c.is_ascii_digit()))
                    .and_then(|b| b.split(',').next())
                    .map(|b| format!("; battery {b}"))
                    .unwrap_or_default();
                push(format!("power: {flags}{battery}"));
            }
        } else if text.starts_with("MCU→SE ITC_EVENT: type 0xff, ") {
            let v = data.and_then(|d| d.get(4)).copied().unwrap_or(0) as u32;
            match progress.as_mut() {
                Some(r) if v >= r.last => {
                    r.last = v;
                    r.end = at;
                    r.count += 1;
                }
                _ => {
                    flush_progress(&mut progress, &mut out);
                    progress = Some(Run {
                        start: at,
                        end: at,
                        count: 1,
                        bytes: 0,
                        first: v,
                        last: v,
                    });
                }
            }
        } else if text.starts_with("C-APDU") {
            let secu = data.is_some_and(|d| d.len() >= 2 && d[0] == 0xe0 && d[1] == 0x00);
            if secu {
                let n = data.map_or(0, |d| d.len().saturating_sub(5));
                match secure.as_mut() {
                    Some(r) => {
                        r.count += 1;
                        r.bytes += n;
                        r.end = at;
                    }
                    None => {
                        secure = Some(Run {
                            start: at,
                            end: at,
                            count: 1,
                            bytes: n,
                            first: 0,
                            last: 0,
                        })
                    }
                }
            } else {
                flush_secure(&mut secure, &mut out);
            }
        } else if text.starts_with("R-APDU") {
            if text.contains("SW 5515") {
                if !locked {
                    locked = true;
                    push("host command refused: device locked".into());
                }
                continue;
            }
            if text.contains("SW 9000") {
                if locked {
                    locked = false;
                    push("device unlocked (commands accepted again)".into());
                }
            } else if !text.contains("SECUINS") {
                let sw = text.split("SW ").nth(1).unwrap_or("");
                let to = text.split(" to ").nth(1).and_then(|t| t.split(':').next()).unwrap_or("command");
                push(format!("host {to} failed: SW {sw}"));
            }
            if let Some(m) = text.split(" — ").nth(1) {
                if m.starts_with("running ") && m != app {
                    app = m.to_string();
                    push(format!("host sees: {m}"));
                } else if m.starts_with("target ") && m != versions {
                    versions = m.to_string();
                    push(format!("host reads versions: {m}"));
                } else if m == "secure channel established" {
                    push("host authenticated the device (genuine check); secure channel established".into());
                }
            }
        } else if text == "BREAK" {
            push("line held low (break)".into());
        } else if text.contains("outside packets") || text.starts_with("incomplete") {
            push(format!("line: {text}"));
        }
    }
    flush_secure(&mut secure, &mut out);
    flush_progress(&mut progress, &mut out);
    out.sort_by_key(|e| e.start);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(s: &mut Seph, bytes: &[u8]) -> Vec<String> {
        let mut out = Vec::new();
        for (i, &b) in bytes.iter().enumerate() {
            s.push(b, i as u64 * 10, i as u64 * 10 + 9, &mut out);
        }
        out.iter()
            .filter_map(|a| match &a.event {
                Event::Protocol { text, .. } => Some(text.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn packets_and_resync() {
        let mut s = Seph::new();
        let t = feed(
            &mut s,
            &[
                0x0e, 0x00, 0x04, 0x00, 0x00, 0x09, 0xe0, // ticker
                0x80, 0x3c, // noise
                0x4c, 0x00, 0x00, // more time
                0x60, 0x00, 0x02, 0x00, 0x00, // general status
            ],
        );
        assert_eq!(
            t,
            [
                "MCU→SE TICKER_EVENT: 2528 ms since power-on",
                "2 bytes outside packets: 80 3c",
                "SE→MCU MORE_TIME",
                "SE→MCU GENERAL_STATUS: last command",
            ]
        );
    }

    #[test]
    fn session_start_fields() {
        let p = [
            0x00, 0x08, 0x00, 0x23, 0x03, 0x06, b'2', b'.', b'3', b'9', b'.', b'0', 0x04, 0xf4, 0xd8, 0xaa, 0x43,
        ];
        assert_eq!(
            session_start(&p),
            "normal, features 0x08002303 (USB, BLE, SSD1312 screen, HW version 2); SEPROXYHAL version \"2.39.0\", MCU bootloader load key id f4d8aa43"
        );
    }

    #[test]
    fn session_start_bootloader() {
        let p = [
            0x00, 0x28, 0x00, 0x23, 0x00, 0x06, b'1', b'.', b'2', b'5', b'.', b'0', 0x04, 0xf4, 0xd8, 0xaa, 0x43,
        ];
        assert_eq!(
            session_start(&p),
            "normal, features 0x28002300 (MCU bootloader, SSD1312 screen, HW version 2); bootloader version \"1.25.0\", load key id f4d8aa43"
        );
    }

    #[test]
    fn usb_hid_apdu() {
        let mut s = Seph::new();
        let mut out = Vec::new();
        // Host → device: e0 01 00 00 00 in one HID report on ep2.
        let mut report = vec![0x01, 0x01, 0x05, 0x00, 0x00, 0x00, 0x05, 0xe0, 0x01, 0x00, 0x00, 0x00];
        report.resize(64, 0);
        let mut pk = vec![0x10, 0x00, 67, 0x82, 0x04, 64];
        pk.extend(&report);
        for (i, &b) in pk.iter().enumerate() {
            s.push(b, i as u64, i as u64 + 1, &mut out);
        }
        // Device → host: 9000.
        let mut resp = vec![0x01, 0x01, 0x05, 0x00, 0x00, 0x00, 0x02, 0x90, 0x00];
        resp.resize(64, 0);
        let mut pk = vec![0x50, 0x00, 67, 0x82, 0x20, 64];
        pk.extend(&resp);
        for (i, &b) in pk.iter().enumerate() {
            s.push(b, 1000 + i as u64, 1001 + i as u64, &mut out);
        }
        let t: Vec<String> = out.iter().map(|a| a.event.to_string()).collect();
        assert_eq!(
            t,
            [
                "MCU→SE USB_EP_XFER_EVENT: ep2 OUT 64 bytes — channel 0x0101 APDU chunk 0, 5/5 bytes",
                "C-APDU (USB ep2): CLA e0 INS 01 P1 00 P2 00 dashboard GET_VERSION, Le 0",
                "SE→MCU USB_EP_PREPARE: ep2 IN 64 bytes — channel 0x0101 APDU chunk 0, 2/2 bytes",
                "R-APDU (USB ep2) to dashboard GET_VERSION: SW 9000 (success)",
            ]
        );
    }

    #[test]
    fn usb_control() {
        assert_eq!(setup(&[0x00, 0x05, 0x01, 0x00, 0, 0, 0, 0]), "SET_ADDRESS 1");
        assert_eq!(setup(&[0x80, 0x06, 0x00, 0x01, 0, 0, 0x40, 0]), "GET_DESCRIPTOR device, 64 bytes");
        assert_eq!(
            descriptor(&[0x0e, 0x03, b'N', 0, b'a', 0, b'n', 0, b'o', 0, b' ', 0, b'X', 0]),
            "string descriptor \"Nano X\""
        );
    }

    #[test]
    fn ble_identity() {
        let p = [
            0x03, 0x06, 0xb3, 0x75, 0x0c, 0xbe, 0xf1, 0xde, 0x0b, b'N', b'a', b'n', b'o', b' ', b'X', b' ', b'A', b'9', b'F', b'0',
        ];
        assert_eq!(mcu(&p), "set BLE identity, address DE:F1:BE:0C:75:B3, name \"Nano X A9F0\"");
    }
}

#[cfg(test)]
mod response_tests {
    use super::*;

    #[test]
    fn versions() {
        let d = [0x01, 0x05, b'B', b'O', b'L', b'O', b'S', 0x05, b'2', b'.', b'6', b'.', b'0'];
        assert_eq!(response_meaning(&[0xb0, 0x01, 0, 0], &d).unwrap(), "running \"BOLOS\" 2.6.0");
        let d = [
            0x33, 0x00, 0x00, 0x04, 0x05, b'2', b'.', b'6', b'.', b'0', 0x04, 0xe6, 0x00, 0x00, 0x0b, 0x06, b'2', b'.', b'3', b'9', b'.',
            b'0', 0x06, b'1', b'.', b'2', b'5', b'.', b'0',
        ];
        assert_eq!(
            response_meaning(&[0xe0, 0x01, 0, 0], &d).unwrap(),
            "target 0x33000004 (Nano X), SE 2.6.0, flags 0xb0000e6 (signed MCU, onboarded, HSM initialized, PIN validated), MCU 2.39.0, MCU bootloader 1.25.0"
        );
    }
}

#[cfg(test)]
mod auth_tests {
    use super::*;

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// The handshake as captured from a Nano X.
    #[test]
    fn authentication() {
        let init = unhex("e05000000865fbf6224564be6d");
        assert_eq!(command_meaning(&init).unwrap(), "server nonce 65fbf6224564be6d");
        assert_eq!(
            response_meaning(&init, &unhex("0000000104250e5baed9eee6")).unwrap(),
            "00000001, device nonce 04250e5baed9eee6"
        );
        let cert = unhex(concat!(
            "070d8398393c74bc41043cf2f66ec69a0a34e6f4b1bbabc7042ca9a023b17ff7f0a2fc769651c1da56649d3e0a55a720b1a755",
            "0448fcb57a4d44c65ab7e42294be83c666ae85447ec23c463044022016909d3599f75346a56564c0f02f3bd708041ee7b6513b",
            "eccbee4a6bd5eceaaa02207f0e01434b55146c51f1a976b0b84f1fdcf1464324cbcaadd25d9cf2dc413802"
        ));
        assert_eq!(
            response_meaning(&unhex("e0520000"), &cert).unwrap(),
            "device certificate: header 0d8398393c74bc, public key 043cf2f66ec69a0a34e6f4b1bbabc7042ca9a023b17ff7f0a2fc769651c1da56649d3e0a55a720b1a7550448fcb57a4d44c65ab7e42294be83c666ae85447ec23c, signature 70 bytes (DER ECDSA)"
        );
    }
}

#[cfg(test)]
mod timeline_tests {
    use super::*;

    #[test]
    fn milestones() {
        let ev: Vec<(u64, &str, Option<&[u8]>)> = vec![
            (
                10,
                "ATR (direct convention), TA1=87 Fi index 8 Di index 7 (reserved values); protocols T=0; 11 historical bytes 05 32 2e 36 2e 30 04 33 00 00 04 '.2.6.0.3...'",
                None,
            ),
            (20, "R-APDU (USB ep2) to GET_APP_NAME_AND_VERSION: SW 5515 (device locked)", None),
            (21, "R-APDU (USB ep2) to GET_APP_NAME_AND_VERSION: SW 5515 (device locked)", None),
            (
                30,
                "C-APDU (USB ep2): CLA e0 INS 00 P1 00 P2 00 dashboard SECUINS (secure channel), Lc 3: 01 02 03",
                Some(&[0xe0, 0, 0, 0, 3, 1, 2, 3]),
            ),
            (
                31,
                "C-APDU (USB ep2): CLA e0 INS 00 P1 00 P2 00 dashboard SECUINS (secure channel), Lc 1: 01",
                Some(&[0xe0, 0, 0, 0, 1, 1]),
            ),
            (40, "SE→MCU SE_POWER_OFF", None),
        ];
        let t: Vec<(u64, Option<u64>, String)> = timeline(ev).into_iter().map(|m| (m.start, m.end, m.text)).collect();
        assert_eq!(
            t,
            [
                (10, None, "SE boot 1: SE 2.6.0, target 0x33000004 (Nano X)".to_string()),
                (20, None, "host command refused: device locked".into()),
                (30, Some(31), "2 secure channel commands, 4 bytes of data".into()),
                (40, None, "SE power off".into()),
            ]
        );
    }
}
