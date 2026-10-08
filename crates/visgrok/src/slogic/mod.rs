//! Driver for Sipeed SLogic logic analyzers.
//!
//! See `docs/PROTOCOL.md` for the protocol this implements. In short, the U3
//! models (SLogic16 U3, SLogic32 U3) are configured through 32-bit registers
//! accessed with vendor control requests, settings go through an "AUX"
//! mailbox, and samples stream unframed on bulk IN endpoint `0x82`. The
//! Combo 8 takes a single start command and streams on `0x81`.

mod stream;

use std::fmt;
use std::time::Duration;

use rawusb::{Context, Device, DeviceHandle};

pub use stream::{Capture, CaptureStats};

/// Sipeed's USB vendor ID.
pub const VID: u16 = 0x359f;

const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

/// Register addresses (U3 models).
mod reg {
    pub const CTRL: u16 = 0x0004;
    pub const AUX: u16 = 0x000c;
    pub const AUX_DATA: u16 = 0x0010;
}

/// CTRL register values.
mod ctrl {
    pub const STOP: u32 = 0;
    pub const RUN: u32 = 1;
    pub const RESET: u32 = 2;
}

/// AUX mailbox selectors.
mod aux {
    pub const CHANNELS: u32 = 1;
    pub const SAMPLERATE: u32 = 2;
    pub const THRESHOLD: u32 = 3;
    pub const PATTERN: u32 = 5;
}

/// Supported models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// SLogic16 U3: 16 channels, USB 3, up to 800 MHz (4 ch).
    SLogic16U3,
    /// SLogic32 U3: 32 channels, USB 3.2 Gen2, up to 1.4 GHz (4 ch).
    SLogic32U3,
    /// SLogic Combo 8: 8 channels, USB 2, up to 160 MHz (2 ch). Untested.
    Combo8,
}

impl Model {
    /// Identifies a model from its product ID.
    pub fn from_pid(pid: u16) -> Option<Model> {
        match pid {
            0x3031 => Some(Model::SLogic16U3),
            0x3032 => Some(Model::SLogic32U3),
            0x0300 => Some(Model::Combo8),
            _ => None,
        }
    }

    /// Product name.
    pub fn name(self) -> &'static str {
        match self {
            Model::SLogic16U3 => "SLogic16 U3",
            Model::SLogic32U3 => "SLogic32 U3",
            Model::Combo8 => "SLogic Combo 8",
        }
    }

    /// Bulk IN endpoint carrying samples.
    pub fn endpoint(self) -> u8 {
        match self {
            Model::Combo8 => 0x81,
            _ => 0x82,
        }
    }

    /// Channel counts supported by visgrok on this model.
    pub fn channel_modes(self) -> &'static [usize] {
        match self {
            Model::Combo8 => &[4, 8],
            Model::SLogic16U3 => &[4, 8, 16],
            Model::SLogic32U3 => &[4, 8, 16, 32],
        }
    }

    /// Maximum bandwidth, in channel·Hz.
    pub fn max_bandwidth(self) -> u64 {
        match self {
            Model::SLogic16U3 => 3_200_000_000,
            Model::SLogic32U3 => 6_400_000_000,
            Model::Combo8 => 320_000_000,
        }
    }

    /// Whether the model uses the register/AUX protocol.
    fn is_u3(self) -> bool {
        !matches!(self, Model::Combo8)
    }

    /// Sample rates the hardware can produce, highest first, ignoring the
    /// per-channel-count bandwidth limit.
    pub fn samplerates(self) -> Vec<u64> {
        match self {
            // base / (divm1 + 1), divm1 0..=255; keep integer-Hz rates. The
            // 16U3 has one 800 MHz base; the 32U3 has 1400 and 800 MHz.
            Model::SLogic16U3 | Model::SLogic32U3 => {
                let bases: &[u64] = if self == Model::SLogic32U3 {
                    &[1_400_000_000, 800_000_000]
                } else {
                    &[800_000_000]
                };
                let mut v: Vec<u64> = bases
                    .iter()
                    .flat_map(|&b| (1..=256u64).filter(move |n| b % n == 0).map(move |n| b / n))
                    .collect();
                v.sort_unstable_by(|a, b| b.cmp(a));
                v.dedup();
                v
            }
            Model::Combo8 => [160, 80, 40, 32, 20, 16, 10, 8, 5, 4, 2, 1].iter().map(|m| m * 1_000_000).collect(),
        }
    }

    /// Highest valid sample rate for `channels` channels.
    pub fn max_samplerate(self, channels: usize) -> u64 {
        let bw = self.max_bandwidth() / channels as u64;
        self.samplerates().into_iter().find(|&r| r <= bw).unwrap_or(0)
    }
}

/// A discovered device.
#[derive(Clone, Debug)]
pub struct Found {
    /// Model.
    pub model: Model,
    /// USB bus number.
    pub bus: u8,
    /// USB device address.
    pub address: u8,
    /// Serial number, when the OS knows it.
    pub serial: Option<String>,
    /// The device is in its bootloader (DFU) and cannot capture.
    pub bootloader: bool,
}

impl fmt::Display for Found {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} (bus {} addr {}", self.model.name(), self.bus, self.address)?;
        if let Some(s) = &self.serial {
            write!(f, ", serial {s}")?;
        }
        if self.bootloader {
            write!(f, ", BOOTLOADER")?;
        }
        write!(f, ")")
    }
}

/// Driver errors.
#[derive(Debug)]
pub enum Error {
    /// USB failure.
    Usb(rawusb::Error),
    /// No matching device.
    NotFound,
    /// The device answered something unexpected.
    Protocol(String),
    /// The requested configuration is not supported.
    Config(String),
    /// The host did not keep up and samples were lost.
    Overrun(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Usb(e) => write!(f, "USB: {e}"),
            Error::NotFound => write!(f, "no SLogic device found"),
            Error::Protocol(m) => write!(f, "protocol: {m}"),
            Error::Config(m) => write!(f, "configuration: {m}"),
            Error::Overrun(m) => write!(f, "overrun: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<rawusb::Error> for Error {
    fn from(e: rawusb::Error) -> Error {
        Error::Usb(e)
    }
}

impl From<Error> for std::io::Error {
    fn from(e: Error) -> std::io::Error {
        match e {
            Error::Usb(u) => u.into(),
            other => std::io::Error::other(other.to_string()),
        }
    }
}

/// Result alias for the driver.
pub type Result<T> = std::result::Result<T, Error>;

/// Device test-pattern modes (U3 models).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pattern {
    /// Real inputs.
    Normal = 0,
    /// Unpaced, flow-controlled counter (USB throughput test).
    UsbTest = 1,
    /// Paced synthetic pattern `(i & !7) | (7 - (i & 7))`.
    Emulation = 2,
}

/// How to convert a threshold voltage into a DAC code.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThresholdModel {
    /// Linear fit measured on real hardware: `V = 0.005166 * code + 0.4318`.
    Measured,
    /// Sipeed's nominal formula: `code = V / 6.66 * 1024`.
    Nominal,
}

/// Capture settings.
#[derive(Clone, Debug)]
pub struct Config {
    /// Channels to capture: 4, 8 or 16 (channels D0..D(n-1)).
    pub channels: usize,
    /// Sample rate in Hz; must be one of [`Model::samplerates`] and within
    /// the bandwidth limit for `channels`.
    pub samplerate: u64,
    /// Input threshold in volts (U3 models). `None` keeps the power-on default
    /// (about 2.0 V).
    pub threshold: Option<f64>,
    /// Voltage-to-code conversion.
    pub threshold_model: ThresholdModel,
    /// Test pattern.
    pub pattern: Pattern,
    /// Stop after this many samples.
    pub limit: Option<u64>,
    /// Verify the effective sample rate at start and restart when the device
    /// latched the previous rate (see PROTOCOL.md §6.3).
    pub verify_rate: bool,
    /// Bulk transfer size override, in bytes (rounded up to 32 KiB).
    pub transfer_size: Option<usize>,
    /// Number of transfers kept in flight (override).
    pub transfers: Option<usize>,
}

impl Config {
    /// 16 channels at `samplerate`, default threshold, normal mode.
    pub fn new(channels: usize, samplerate: u64) -> Config {
        Config {
            channels,
            samplerate,
            threshold: None,
            threshold_model: ThresholdModel::Measured,
            pattern: Pattern::Normal,
            limit: None,
            verify_rate: true,
            transfer_size: None,
            transfers: None,
        }
    }

    /// Bytes per second on the wire.
    pub fn byte_rate(&self) -> f64 {
        self.samplerate as f64 * self.channels as f64 / 8.0
    }
}

/// Converts a threshold voltage into a DAC code.
pub fn threshold_code(volts: f64, model: ThresholdModel) -> u32 {
    let code = match model {
        ThresholdModel::Measured => (volts - 0.4318) / 0.005166,
        ThresholdModel::Nominal => volts.clamp(0.0, 6.0) / 6.66 * 1024.0,
    };
    code.round().clamp(0.0, 1023.0) as u32
}

/// Converts a DAC code back into volts.
pub fn threshold_volts(code: u32, model: ThresholdModel) -> f64 {
    match model {
        ThresholdModel::Measured => 0.005166 * code as f64 + 0.4318,
        ThresholdModel::Nominal => code as f64 * 6.66 / 1024.0,
    }
}

/// Lists connected SLogic devices without opening them.
pub fn list() -> Result<Vec<Found>> {
    let ctx = Context::new()?;
    Ok(ctx.devices()?.iter().filter_map(found).collect())
}

fn found(d: &Device) -> Option<Found> {
    if d.vendor_id() != VID {
        return None;
    }
    let pid = d.product_id();
    let bootloader = matches!(pid, 0x30f1 | 0x30f2);
    let model = match pid {
        0x30f1 => Model::SLogic16U3,
        0x30f2 => Model::SLogic32U3,
        p => Model::from_pid(p)?,
    };
    Some(Found {
        model,
        bus: d.bus_number(),
        address: d.address(),
        // Asking a bootloader for strings is harmless but pointless.
        serial: if bootloader { None } else { d.serial_number().map(str::to_string) },
        bootloader,
    })
}

/// An open SLogic device.
pub struct SLogic {
    _ctx: Context,
    handle: DeviceHandle,
    model: Model,
    serial: Option<String>,
}

impl SLogic {
    /// Opens the first device, or the one with the given serial number.
    pub fn open(serial: Option<&str>) -> Result<SLogic> {
        let ctx = Context::new()?;
        let dev = ctx
            .devices()?
            .into_iter()
            .find(|d| found(d).is_some_and(|f| !f.bootloader && serial.is_none_or(|s| f.serial.as_deref() == Some(s))))
            .ok_or(Error::NotFound)?;
        let model = Model::from_pid(dev.product_id()).ok_or(Error::NotFound)?;
        let serial = dev.serial_number().map(str::to_string);
        let handle = dev.open()?;
        // The 16U3 can enumerate unconfigured (seen on macOS).
        if handle.active_configuration()? == 0 {
            handle.set_configuration(1)?;
        }
        handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(0)?;
        let s = SLogic {
            _ctx: ctx,
            handle,
            model,
            serial,
        };
        if model.is_u3() {
            // A previous session may have left the device in reset, in which
            // state the AUX mailbox does not answer.
            s.reset()?;
        }
        Ok(s)
    }

    /// Model of the device.
    pub fn model(&self) -> Model {
        self.model
    }

    /// Serial number, if known.
    pub fn serial(&self) -> Option<&str> {
        self.serial.as_deref()
    }

    pub(crate) fn handle(&self) -> &DeviceHandle {
        &self.handle
    }

    /// Reads a 32-bit register (U3 models).
    pub fn reg_read(&self, addr: u16) -> Result<u32> {
        let mut b = [0u8; 4];
        let n = self.handle.control_read(0xc0, 0x00, addr, 0, &mut b, CTRL_TIMEOUT)?;
        if n != 4 {
            return Err(Error::Protocol(format!("register {addr:#06x}: short read ({n} bytes)")));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Writes a 32-bit register (U3 models).
    pub fn reg_write(&self, addr: u16, value: u32) -> Result<()> {
        let n = self.handle.control_write(0x40, 0x01, addr, 0, &value.to_le_bytes(), CTRL_TIMEOUT)?;
        if n != 4 {
            return Err(Error::Protocol(format!("register {addr:#06x}: short write ({n} bytes)")));
        }
        Ok(())
    }

    /// Pulses reset, restoring all settings to their defaults.
    pub fn reset(&self) -> Result<()> {
        self.reg_write(reg::CTRL, ctrl::RESET)?;
        self.reg_write(reg::CTRL, ctrl::STOP)
    }

    pub(crate) fn run(&self, cfg: &Config) -> Result<()> {
        if self.model.is_u3() {
            self.reg_write(reg::CTRL, ctrl::RUN)
        } else {
            // Combo 8: [MHz u16 LE][channels u8][pad].
            let mhz = (cfg.samplerate / 1_000_000) as u16;
            let p = [mhz as u8, (mhz >> 8) as u8, cfg.channels as u8, 0];
            self.handle.control_write(0x40, 0xb1, 0, 0, &p, CTRL_TIMEOUT)?;
            Ok(())
        }
    }

    pub(crate) fn halt(&self) -> Result<()> {
        if self.model.is_u3() {
            self.reg_write(reg::CTRL, ctrl::STOP)
        } else {
            // The Combo 8's stop command is unreliable; the stream ends when
            // the host stops reading and drains the endpoint.
            Ok(())
        }
    }

    /// Runs one AUX transaction: selects `sel`, reads the payload, lets `f`
    /// modify it, writes it back and checks the read-back. Returns the final
    /// payload.
    fn aux(&self, sel: u32, f: impl FnOnce(&mut Vec<u32>) -> Result<()>) -> Result<Vec<u32>> {
        self.reg_write(reg::AUX, sel)?;
        let mut header = 0;
        for _ in 0..8 {
            header = self.reg_read(reg::AUX)?;
            if header >> 16 & 1 != 0 {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        if header >> 16 & 1 == 0 {
            return Err(Error::Protocol(format!("AUX {sel}: not ready (header {header:#010x})")));
        }
        if header & 0x1ff != sel {
            return Err(Error::Protocol(format!("AUX {sel}: header echoes selector {}", header & 0x1ff)));
        }
        let len = (header >> 9 & 0x7f) as usize;
        let words = len.div_ceil(4);
        if words == 0 {
            return Err(Error::Protocol(format!("AUX {sel}: empty payload")));
        }
        let read = |s: &SLogic| -> Result<Vec<u32>> { (0..words).map(|i| s.reg_read(reg::AUX_DATA + 4 * i as u16)).collect() };
        let mut payload = read(self)?;
        f(&mut payload)?;
        for (i, w) in payload.iter().enumerate() {
            self.reg_write(reg::AUX_DATA + 4 * i as u16, *w)?;
        }
        let back = read(self)?;
        // Only compare the bytes the payload actually has.
        let mask = |i: usize| -> u32 {
            let bytes = (len - 4 * i).min(4);
            if bytes == 4 { u32::MAX } else { (1u32 << (8 * bytes)) - 1 }
        };
        for i in 0..words {
            if (back[i] ^ payload[i]) & mask(i) != 0 {
                return Err(Error::Protocol(format!(
                    "AUX {sel}: word {i} reads back {:#010x}, wrote {:#010x}",
                    back[i], payload[i]
                )));
            }
        }
        Ok(back)
    }

    /// Checks a configuration against the model's capabilities.
    pub fn validate(&self, cfg: &Config) -> Result<()> {
        let m = self.model;
        if !m.channel_modes().contains(&cfg.channels) {
            return Err(Error::Config(format!("{} supports {:?} channels", m.name(), m.channel_modes())));
        }
        if !m.samplerates().contains(&cfg.samplerate) {
            return Err(Error::Config(format!("{} Hz is not a supported sample rate", cfg.samplerate)));
        }
        if cfg.samplerate * cfg.channels as u64 > m.max_bandwidth() {
            return Err(Error::Config(format!(
                "{} channels at {} Hz exceeds the bandwidth (max {} Hz)",
                cfg.channels,
                cfg.samplerate,
                m.max_samplerate(cfg.channels)
            )));
        }
        if !m.is_u3() && cfg.pattern != Pattern::Normal {
            return Err(Error::Config("test patterns need a U3 model".into()));
        }
        Ok(())
    }

    /// Stops, resets and programs the device for `cfg` (does not start it).
    pub(crate) fn configure(&self, cfg: &Config) -> Result<()> {
        self.validate(cfg)?;
        if !self.model.is_u3() {
            return Ok(());
        }
        self.reg_write(reg::CTRL, ctrl::STOP)?;
        self.reset()?;
        self.reg_write(reg::CTRL, ctrl::STOP)?;

        let mask = if cfg.channels >= 32 { u32::MAX } else { (1u32 << cfg.channels) - 1 };
        self.aux(aux::CHANNELS, |p| {
            p[0] = mask;
            Ok(())
        })?;

        let rate = cfg.samplerate;
        self.aux(aux::SAMPLERATE, |p| {
            if p.len() < 2 {
                return Err(Error::Protocol("samplerate payload too short".into()));
            }
            // Walk the base-clock table until a base divides the rate.
            for _ in 0..8 {
                let base = (p[0] >> 16) as u64 * 1_000_000;
                if base == 0 {
                    return Err(Error::Config(format!("no base clock for {rate} Hz")));
                }
                if base.is_multiple_of(rate) {
                    let div = base / rate;
                    if !(1..=256).contains(&div) {
                        return Err(Error::Config(format!("{rate} Hz needs divider {div} (max 256)")));
                    }
                    p[1] = (div - 1) as u32;
                    return Ok(());
                }
                let idx = (p[0] & 0xffff) + 1;
                self.reg_write(reg::AUX_DATA, idx | (p[0] & 0xffff_0000))?;
                p[0] = self.reg_read(reg::AUX_DATA)?;
            }
            Err(Error::Config(format!("no base clock for {rate} Hz")))
        })?;

        if let Some(v) = cfg.threshold {
            let code = threshold_code(v, cfg.threshold_model);
            self.aux(aux::THRESHOLD, |p| {
                p[0] = code;
                Ok(())
            })?;
        }

        let pattern = cfg.pattern as u32;
        self.aux(aux::PATTERN, |p| {
            p[0] = pattern;
            Ok(())
        })?;
        Ok(())
    }

    /// Reads the current threshold DAC code (U3 models).
    pub fn threshold_code(&self) -> Result<u32> {
        let p = self.aux(aux::THRESHOLD, |_| Ok(()))?;
        Ok(p[0] & 0xffff)
    }

    /// Starts a capture. The returned [`Capture`] is a [`crate::Source`].
    pub fn start(self, cfg: Config) -> Result<Capture> {
        Capture::start(self, cfg)
    }
}

impl Drop for SLogic {
    fn drop(&mut self) {
        let _ = self.halt();
        let _ = self.handle.release_interface(0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rates() {
        let r = Model::SLogic16U3.samplerates();
        assert_eq!(r[0], 800_000_000);
        assert!(r.contains(&3_125_000));
        assert!(!r.contains(&1_000_000));
        assert_eq!(Model::SLogic16U3.max_samplerate(16), 200_000_000);
        assert_eq!(Model::SLogic16U3.max_samplerate(8), 400_000_000);
        assert_eq!(Model::SLogic16U3.max_samplerate(4), 800_000_000);

        let r = Model::SLogic32U3.samplerates();
        assert_eq!(r[0], 1_400_000_000);
        assert!(r.contains(&700_000_000) && r.contains(&800_000_000) && r.contains(&350_000_000));
        assert_eq!(Model::SLogic32U3.max_samplerate(32), 200_000_000);
        assert_eq!(Model::SLogic32U3.max_samplerate(16), 400_000_000);
        assert_eq!(Model::SLogic32U3.max_samplerate(8), 800_000_000);
        assert_eq!(Model::SLogic32U3.max_samplerate(4), 1_400_000_000);
    }

    #[test]
    fn thresholds() {
        assert_eq!(threshold_code(1.6, ThresholdModel::Nominal), 246);
        let c = threshold_code(1.65, ThresholdModel::Measured);
        assert!((threshold_volts(c, ThresholdModel::Measured) - 1.65).abs() < 0.003);
    }
}
