//! BAGL: the display protocol of legacy Ledger devices (Nano S, Blue),
//! where the secure element sends each screen element to the MCU in a
//! SEPROXYHAL `SCREEN_DISPLAY_STATUS` packet and the MCU draws it.
//!
//! Every packet carries a `bagl_component_t` (28 bytes, little endian):
//!
//! | offset | field |
//! |---|---|
//! | 0 | type (`0x80`: touchable flag) |
//! | 1 | user id |
//! | 2, 4 | x, y (signed) |
//! | 6, 8 | width, height |
//! | 10, 11, 12 | stroke, radius, fill |
//! | 16, 20 | foreground, background color (`0x00RRGGBB`) |
//! | 24 | font id (low 12 bits) and text alignment (high 4 bits) |
//! | 26 | icon id (a glyph built into the MCU, 0: bitmap follows) |
//!
//! followed by the label text, or for icons without an id, the bitmap:
//! bits per pixel (1), the palette (4 bytes per color) and the pixels,
//! row by row, least significant bit first.
//!
//! [`Canvas`] redraws the elements following BAGL's own drawing rules
//! (`lib_bagl/src/bagl.c`). The MCU's fonts and glyphs are not in the
//! traffic: text is drawn with a built-in 5x7 font (so widths and exact
//! shapes differ from the device's Open Sans), a few common glyphs are
//! approximated, and other glyphs show as an outline of their box.

use super::DisplayView;

/// Size of `bagl_component_t` on the wire.
pub const COMPONENT_LEN: usize = 28;

const NOFILL: u8 = 0;
const FILL: u8 = 1;

/// A decoded `bagl_component_t`.
#[derive(Clone, Debug, PartialEq)]
pub struct Component {
    /// Type without the touchable flag.
    pub kind: u8,
    pub userid: u8,
    pub x: i32,
    pub y: i32,
    pub width: i32,
    pub height: i32,
    pub stroke: u8,
    pub radius: u8,
    pub fill: u8,
    pub fg: u32,
    pub bg: u32,
    pub font_id: u16,
    pub icon_id: u8,
}

impl Component {
    /// Parses the component at the start of a `SCREEN_DISPLAY_STATUS`
    /// payload; the rest of the payload is its text or bitmap.
    pub fn parse(p: &[u8]) -> Option<(Component, &[u8])> {
        if p.len() < COMPONENT_LEN {
            return None;
        }
        let i16le = |i: usize| i16::from_le_bytes([p[i], p[i + 1]]) as i32;
        let u16le = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let u32le = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        Some((
            Component {
                kind: p[0] & 0x7f,
                userid: p[1],
                x: i16le(2),
                y: i16le(4),
                width: u16le(6) as i32,
                height: u16le(8) as i32,
                stroke: p[10],
                radius: p[11],
                fill: p[12],
                fg: u32le(16),
                bg: u32le(20),
                font_id: u16le(24),
                icon_id: p[26],
            },
            &p[COMPONENT_LEN..],
        ))
    }
}

/// Name of a component type.
pub fn kind_name(kind: u8) -> Option<&'static str> {
    Some(match kind & 0x7f {
        0 => "none",
        1 => "button",
        2 => "label",
        3 => "rectangle",
        4 => "line",
        5 => "icon",
        6 => "circle",
        7 => "label line",
        _ => return None,
    })
}

/// Name of a glyph built into the Nano S / Blue MCU (`BAGL_GLYPH_*`).
pub fn glyph_name(id: u8) -> Option<&'static str> {
    const NAMES: [&str; 23] = [
        "none",
        "Ledger logo",
        "Ledger Blue logo",
        "gears",
        "clear",
        "backspace",
        "check",
        "cross",
        "check badge",
        "left",
        "right",
        "up",
        "down",
        "Ledger mini logo",
        "cross badge",
        "dashboard badge",
        "plus",
        "less",
        "toggle on",
        "toggle off",
        "loading badge",
        "cog badge",
        "warning badge",
    ];
    NAMES.get(id as usize).copied()
}

/// Built-in glyphs drawn here: (id, width, height, rows with bit 0 = left).
const GLYPHS: &[(u8, i32, i32, &[u8])] = &[
    // check, 8x6
    (6, 8, 6, &[0x80, 0x40, 0x21, 0x12, 0x0c, 0x00]),
    // cross, 7x7
    (7, 7, 7, &[0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41]),
    // left, 4x7
    (9, 4, 7, &[0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08]),
    // right, 4x7
    (10, 4, 7, &[0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01]),
    // up, 7x4
    (11, 7, 4, &[0x08, 0x14, 0x22, 0x41]),
    // down, 7x4
    (12, 7, 4, &[0x41, 0x22, 0x14, 0x08]),
    // plus, 5x5
    (16, 5, 5, &[0x04, 0x04, 0x1f, 0x04, 0x04]),
    // less, 5x1
    (17, 5, 1, &[0x1f]),
];

/// The 5x7 font for ASCII 0x20..0x7e: five columns per character, bit 0
/// = top row.
const FONT: [[u8; 5]; 95] = [
    [0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x00, 0x5f, 0x00, 0x00],
    [0x00, 0x07, 0x00, 0x07, 0x00],
    [0x14, 0x7f, 0x14, 0x7f, 0x14],
    [0x24, 0x2a, 0x7f, 0x2a, 0x12],
    [0x23, 0x13, 0x08, 0x64, 0x62],
    [0x36, 0x49, 0x55, 0x22, 0x50],
    [0x00, 0x05, 0x03, 0x00, 0x00],
    [0x00, 0x1c, 0x22, 0x41, 0x00],
    [0x00, 0x41, 0x22, 0x1c, 0x00],
    [0x08, 0x2a, 0x1c, 0x2a, 0x08],
    [0x08, 0x08, 0x3e, 0x08, 0x08],
    [0x00, 0x50, 0x30, 0x00, 0x00],
    [0x08, 0x08, 0x08, 0x08, 0x08],
    [0x00, 0x60, 0x60, 0x00, 0x00],
    [0x20, 0x10, 0x08, 0x04, 0x02],
    [0x3e, 0x51, 0x49, 0x45, 0x3e],
    [0x00, 0x42, 0x7f, 0x40, 0x00],
    [0x42, 0x61, 0x51, 0x49, 0x46],
    [0x21, 0x41, 0x45, 0x4b, 0x31],
    [0x18, 0x14, 0x12, 0x7f, 0x10],
    [0x27, 0x45, 0x45, 0x45, 0x39],
    [0x3c, 0x4a, 0x49, 0x49, 0x30],
    [0x01, 0x71, 0x09, 0x05, 0x03],
    [0x36, 0x49, 0x49, 0x49, 0x36],
    [0x06, 0x49, 0x49, 0x29, 0x1e],
    [0x00, 0x36, 0x36, 0x00, 0x00],
    [0x00, 0x56, 0x36, 0x00, 0x00],
    [0x08, 0x14, 0x22, 0x41, 0x00],
    [0x14, 0x14, 0x14, 0x14, 0x14],
    [0x00, 0x41, 0x22, 0x14, 0x08],
    [0x02, 0x01, 0x51, 0x09, 0x06],
    [0x32, 0x49, 0x79, 0x41, 0x3e],
    [0x7e, 0x11, 0x11, 0x11, 0x7e],
    [0x7f, 0x49, 0x49, 0x49, 0x36],
    [0x3e, 0x41, 0x41, 0x41, 0x22],
    [0x7f, 0x41, 0x41, 0x22, 0x1c],
    [0x7f, 0x49, 0x49, 0x49, 0x41],
    [0x7f, 0x09, 0x09, 0x09, 0x01],
    [0x3e, 0x41, 0x49, 0x49, 0x7a],
    [0x7f, 0x08, 0x08, 0x08, 0x7f],
    [0x00, 0x41, 0x7f, 0x41, 0x00],
    [0x20, 0x40, 0x41, 0x3f, 0x01],
    [0x7f, 0x08, 0x14, 0x22, 0x41],
    [0x7f, 0x40, 0x40, 0x40, 0x40],
    [0x7f, 0x02, 0x0c, 0x02, 0x7f],
    [0x7f, 0x04, 0x08, 0x10, 0x7f],
    [0x3e, 0x41, 0x41, 0x41, 0x3e],
    [0x7f, 0x09, 0x09, 0x09, 0x06],
    [0x3e, 0x41, 0x51, 0x21, 0x5e],
    [0x7f, 0x09, 0x19, 0x29, 0x46],
    [0x46, 0x49, 0x49, 0x49, 0x31],
    [0x01, 0x01, 0x7f, 0x01, 0x01],
    [0x3f, 0x40, 0x40, 0x40, 0x3f],
    [0x1f, 0x20, 0x40, 0x20, 0x1f],
    [0x3f, 0x40, 0x38, 0x40, 0x3f],
    [0x63, 0x14, 0x08, 0x14, 0x63],
    [0x07, 0x08, 0x70, 0x08, 0x07],
    [0x61, 0x51, 0x49, 0x45, 0x43],
    [0x00, 0x7f, 0x41, 0x41, 0x00],
    [0x02, 0x04, 0x08, 0x10, 0x20],
    [0x00, 0x41, 0x41, 0x7f, 0x00],
    [0x04, 0x02, 0x01, 0x02, 0x04],
    [0x40, 0x40, 0x40, 0x40, 0x40],
    [0x00, 0x01, 0x02, 0x04, 0x00],
    [0x20, 0x54, 0x54, 0x54, 0x78],
    [0x7f, 0x48, 0x44, 0x44, 0x38],
    [0x38, 0x44, 0x44, 0x44, 0x20],
    [0x38, 0x44, 0x44, 0x48, 0x7f],
    [0x38, 0x54, 0x54, 0x54, 0x18],
    [0x08, 0x7e, 0x09, 0x01, 0x02],
    [0x0c, 0x52, 0x52, 0x52, 0x3e],
    [0x7f, 0x08, 0x04, 0x04, 0x78],
    [0x00, 0x44, 0x7d, 0x40, 0x00],
    [0x20, 0x40, 0x44, 0x3d, 0x00],
    [0x7f, 0x10, 0x28, 0x44, 0x00],
    [0x00, 0x41, 0x7f, 0x40, 0x00],
    [0x7c, 0x04, 0x18, 0x04, 0x78],
    [0x7c, 0x08, 0x04, 0x04, 0x78],
    [0x38, 0x44, 0x44, 0x44, 0x38],
    [0x7c, 0x14, 0x14, 0x14, 0x08],
    [0x08, 0x14, 0x14, 0x18, 0x7c],
    [0x7c, 0x08, 0x04, 0x04, 0x08],
    [0x48, 0x54, 0x54, 0x54, 0x20],
    [0x04, 0x3f, 0x44, 0x40, 0x20],
    [0x3c, 0x40, 0x40, 0x20, 0x7c],
    [0x1c, 0x20, 0x40, 0x20, 0x1c],
    [0x3c, 0x40, 0x30, 0x40, 0x3c],
    [0x44, 0x28, 0x10, 0x28, 0x44],
    [0x0c, 0x50, 0x50, 0x50, 0x3c],
    [0x44, 0x64, 0x54, 0x4c, 0x44],
    [0x00, 0x08, 0x36, 0x41, 0x00],
    [0x00, 0x00, 0x7f, 0x00, 0x00],
    [0x00, 0x41, 0x36, 0x08, 0x00],
    [0x08, 0x04, 0x08, 0x10, 0x08],
];

/// Width of a character cell of the built-in font (5 columns + 1 space).
const CHAR_W: i32 = 6;
/// Height of the built-in font above its baseline.
const BASELINE: i32 = 7;

/// A monochrome screen drawn from BAGL components: any non-black color is
/// a lit pixel.
#[derive(Clone, Debug)]
pub struct Canvas {
    pub width: usize,
    pub height: usize,
    pixels: Vec<bool>,
}

impl Canvas {
    /// A blank screen.
    pub fn new(width: usize, height: usize) -> Canvas {
        Canvas {
            width,
            height,
            pixels: vec![false; width * height],
        }
    }

    /// Lit pixels, row by row.
    pub fn pixels(&self) -> &[bool] {
        &self.pixels
    }

    /// The screen as a display view.
    pub fn view(&self, title: &'static str, on: bool, updates: u64) -> DisplayView {
        let mut v = DisplayView::new(title, self.width, self.height);
        v.pixels = self.pixels.clone();
        v.on = on;
        v.updates = updates;
        v
    }

    fn set(&mut self, x: i32, y: i32, on: bool) {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            self.pixels[y as usize * self.width + x as usize] = on;
        }
    }

    fn rect(&mut self, color: u32, x: i32, y: i32, w: i32, h: i32) {
        for yy in y..y + h.max(0) {
            for xx in x..x + w.max(0) {
                self.set(xx, yy, color != 0);
            }
        }
    }

    /// A 1-bit-per-row-bit glyph from [`GLYPHS`].
    fn glyph(&mut self, color: u32, x: i32, y: i32, w: i32, rows: &[u8]) {
        for (r, &bits) in rows.iter().enumerate() {
            for c in 0..w {
                if bits >> c & 1 != 0 {
                    self.set(x + c, y + r as i32, color != 0);
                }
            }
        }
    }

    /// Text in the built-in font from `x, top`, clipped to `x..right`.
    fn text(&mut self, color: u32, x: i32, top: i32, right: i32, s: &[u8]) {
        for (i, &c) in s.iter().enumerate() {
            let cols = FONT.get(c.wrapping_sub(0x20) as usize).unwrap_or(&FONT['?' as usize - 0x20]);
            for (col, &bits) in cols.iter().enumerate() {
                let px = x + i as i32 * CHAR_W + col as i32;
                if px >= right {
                    return;
                }
                for row in 0..BASELINE {
                    if bits >> row & 1 != 0 {
                        self.set(px, top + row, color != 0);
                    }
                }
            }
        }
    }

    /// A bitmap: `bpp` bits per pixel, row by row, least significant bit
    /// first; lit where the palette color is not black.
    fn bitmap(&mut self, (x, y, w, h): (i32, i32, i32, i32), bpp: usize, colors: &[u32], bits: &[u8]) {
        for py in 0..h {
            for px in 0..w {
                let i = (py * w + px) as usize * bpp;
                let mut idx = 0;
                for b in 0..bpp {
                    let bit = i + b;
                    if bits.get(bit / 8).is_some_and(|v| v >> (bit % 8) & 1 != 0) {
                        idx |= 1 << b;
                    }
                }
                if let Some(&c) = colors.get(idx) {
                    self.set(x + px, y + py, c != 0);
                }
            }
        }
    }

    fn circle(&mut self, color: u32, cx: i32, cy: i32, r: i32, inner: i32, bg: u32) {
        for dy in -r..=r {
            for dx in -r..=r {
                let d = dx * dx + dy * dy;
                if d <= r * r {
                    let c = if inner > 0 && d < inner * inner { bg } else { color };
                    self.set(cx + dx, cy + dy, c != 0);
                }
            }
        }
    }

    /// Draws one component with its text or bitmap (`extra`).
    pub fn draw(&mut self, c: &Component, extra: &[u8]) {
        let text: &[u8] = {
            // Labels are sent without their terminating NUL, but be lenient.
            let end = extra.iter().position(|&b| b == 0).unwrap_or(extra.len());
            &extra[..end]
        };
        let (x, y, w, h) = (c.x, c.y, c.width, c.height);
        let radius = (c.radius as i32).min(w / 2).min(h / 2);
        let stroke = c.stroke as i32;
        match c.kind {
            // Line: a filled rectangle (a rounded one is drawn as a
            // rectangle would be).
            4 if c.radius == 0 => self.rect(c.fg, x, y, w, h),
            1..=4 | 7 => {
                let labeline = c.kind == 7;
                let label = c.kind == 2 || labeline;
                if label {
                    // Labels clear their box with the background first.
                    self.rect(c.bg, x, y - if labeline { BASELINE } else { 0 }, w, h);
                } else if c.fill != FILL {
                    // Inside with the background, then the outline.
                    self.rect(c.bg, x, y, w, h);
                    self.rect(c.fg, x + radius, y, w - 2 * radius, stroke);
                    self.rect(c.fg, x + radius, y + h - stroke, w - 2 * radius, stroke);
                    self.rect(c.fg, x, y + radius, stroke, h - 2 * radius);
                    self.rect(c.fg, x + w - stroke, y + radius, stroke, h - 2 * radius);
                } else {
                    self.rect(c.fg, x + radius, y, w - 2 * radius, h);
                    self.rect(c.fg, x, y + radius, radius, h - 2 * radius);
                    self.rect(c.fg, x + w - radius, y + radius, radius, h - 2 * radius);
                }
                if radius > 1 && !label {
                    let inner = if c.fill != FILL && stroke < radius { radius - stroke } else { 0 };
                    for (cx, cy) in [
                        (x + radius, y + radius),
                        (x + w - radius - 1, y + radius),
                        (x + radius, y + h - radius - 1),
                        (x + w - radius - 1, y + h - radius - 1),
                    ] {
                        self.circle(c.fg, cx, cy, radius, inner, c.bg);
                    }
                }
                if c.kind == 4 || text.is_empty() {
                    return;
                }
                // Filled buttons and rectangles show their text inverted.
                let fg = if matches!(c.kind, 1 | 3) && c.fill == FILL { c.bg } else { c.fg };
                let tw = text.len() as i32 * CHAR_W - 1;
                let ha = match c.font_id & 0xc000 {
                    0x4000 => (w - tw).max(0),
                    0x8000 => (w / 2 - tw / 2).max(0),
                    _ => 0,
                };
                let top = if labeline {
                    y - BASELINE
                } else {
                    y + match c.font_id & 0x3000 {
                        0x1000 => h - BASELINE,
                        0x2000 => h / 2 - BASELINE / 2 - 1,
                        _ => 0,
                    }
                };
                self.text(fg, x + ha, top, x + w, text);
            }
            5 => {
                if c.icon_id != 0 {
                    match GLYPHS.iter().find(|g| g.0 == c.icon_id) {
                        Some(&(_, gw, gh, rows)) => {
                            // Glyph colors may follow (4 bytes each, little
                            // endian); the last one draws the glyph.
                            let color = match extra.len() {
                                8 => u32::from_le_bytes([extra[4], extra[5], extra[6], extra[7]]),
                                _ => 0xffffff,
                            };
                            self.glyph(color, x + w / 2 - gw / 2, y + h / 2 - gh / 2, gw, rows);
                        }
                        // A glyph we don't have: its box.
                        None => {
                            self.rect(c.fg.max(1), x, y, w, 1);
                            self.rect(c.fg.max(1), x, y + h - 1, w, 1);
                            self.rect(c.fg.max(1), x, y, 1, h);
                            self.rect(c.fg.max(1), x + w - 1, y, 1, h);
                        }
                    }
                    return;
                }
                // <bpp> <colors, 4 bytes each> <bitmap>
                let Some((&bpp, rest)) = extra.split_first() else { return };
                if bpp == 0 || bpp > 2 {
                    return;
                }
                let n = 1usize << bpp;
                if rest.len() < n * 4 {
                    return;
                }
                let colors: Vec<u32> = rest[..n * 4]
                    .chunks(4)
                    .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .collect();
                self.bitmap((x, y, w, h), bpp as usize, &colors, &rest[n * 4..]);
            }
            6 => {
                let r = c.radius as i32;
                let inner = if c.fill != FILL && stroke < r { r - stroke } else { 0 };
                self.circle(c.fg, x + r, y + r, r, inner, c.bg);
            }
            _ => {}
        }
    }

    /// Draws a `SCREEN_DISPLAY_RAW_STATUS` start packet: `<0> <x> <y> <w>
    /// <h> (2 bytes each, big endian) <bpp> <colors> <bitmap>`. Returns
    /// where the bitmap continues, for the following packets.
    pub fn raw_start(&mut self, p: &[u8]) -> Option<RawState> {
        if p.len() < 10 || p[0] != 0 {
            return None;
        }
        let be = |i: usize| i16::from_be_bytes([p[i], p[i + 1]]) as i32;
        let (x, y, w, h, bpp) = (be(1), be(3), be(5), be(7), p[9] as usize);
        if bpp == 0 || bpp > 2 {
            return None;
        }
        let n = 1 << bpp;
        let colors: Vec<u32> = p
            .get(10..10 + n * 4)?
            .chunks(4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();
        let mut st = RawState {
            x,
            y,
            w,
            h,
            bpp,
            colors,
            bits: Vec::new(),
        };
        st.bits.extend_from_slice(&p[10 + n * 4..]);
        self.raw_draw(&st);
        Some(st)
    }

    /// Draws a raw bitmap as far as it was received.
    pub fn raw_draw(&mut self, st: &RawState) {
        self.bitmap((st.x, st.y, st.w, st.h), st.bpp, &st.colors, &st.bits);
    }
}

/// A raw bitmap being received in `SCREEN_DISPLAY_RAW_STATUS` packets.
#[derive(Clone, Debug)]
pub struct RawState {
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    bpp: usize,
    colors: Vec<u32>,
    /// Bitmap bytes received so far.
    pub bits: Vec<u8>,
}

/// One-line description of a `SCREEN_DISPLAY_STATUS` payload.
pub fn describe(p: &[u8]) -> String {
    let Some((c, extra)) = Component::parse(p) else {
        return format!("{} bytes: {}", p.len(), super::seph::hex_trunc(p, 24));
    };
    let ty = kind_name(c.kind).map_or_else(|| format!("type {}", c.kind), str::to_string);
    let mut s = format!("{ty} id {} at {},{} size {}x{}", c.userid, c.x, c.y, c.width, c.height);
    match c.kind {
        5 if c.icon_id != 0 => {
            s += &match glyph_name(c.icon_id) {
                Some(n) => format!(", glyph {} ({n})", c.icon_id),
                None => format!(", glyph {}", c.icon_id),
            };
        }
        5 if !extra.is_empty() => {
            s += &format!(", {} bpp bitmap", extra[0]);
        }
        1..=4 | 6 | 7 => {
            if matches!(c.kind, 1 | 3 | 6) {
                s += match c.fill {
                    FILL => ", filled",
                    NOFILL => "",
                    _ => ", outline",
                };
            }
            let text: Vec<u8> = extra.iter().copied().take_while(|&b| b != 0).collect();
            if !text.is_empty() {
                let t: String = text
                    .iter()
                    .map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' })
                    .collect();
                s += &format!(" \"{t}\"");
            }
        }
        _ if !extra.is_empty() => s += &format!(", {} bytes: {}", extra.len(), super::seph::hex_trunc(extra, 16)),
        _ => {}
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::too_many_arguments)]
    fn comp(kind: u8, x: i16, y: i16, w: u16, h: u16, fill: u8, fg: u32, bg: u32, font: u16, icon: u8) -> Vec<u8> {
        let mut p = vec![0u8; COMPONENT_LEN];
        p[0] = kind;
        p[2..4].copy_from_slice(&x.to_le_bytes());
        p[4..6].copy_from_slice(&y.to_le_bytes());
        p[6..8].copy_from_slice(&w.to_le_bytes());
        p[8..10].copy_from_slice(&h.to_le_bytes());
        p[12] = fill;
        p[16..20].copy_from_slice(&fg.to_le_bytes());
        p[20..24].copy_from_slice(&bg.to_le_bytes());
        p[24..26].copy_from_slice(&font.to_le_bytes());
        p[26] = icon;
        p
    }

    fn rows(c: &Canvas, y0: usize, n: usize, x0: usize, w: usize) -> Vec<String> {
        (y0..y0 + n)
            .map(|y| (x0..x0 + w).map(|x| if c.pixels[y * c.width + x] { '#' } else { '.' }).collect())
            .collect()
    }

    /// Captured from a Nano S PIN screen: a digit as a 1 bpp icon.
    #[test]
    fn icon_bitmap() {
        let mut p = comp(5, 25, 18, 8, 10, 0, 0xffffff, 0, 0, 0);
        p[1] = 17;
        p.extend([1, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0]);
        p.extend([0x3e, 0x02, 0x02, 0x02, 0x3e, 0x40, 0x40, 0x40, 0x40, 0x3e]);
        let (c, extra) = Component::parse(&p).unwrap();
        assert_eq!(describe(&p), "icon id 17 at 25,18 size 8x10, 1 bpp bitmap");
        let mut cv = Canvas::new(128, 32);
        cv.draw(&c, extra);
        assert_eq!(
            rows(&cv, 18, 10, 25, 8),
            [
                ".#####..", ".#......", ".#......", ".#......", ".#####..", "......#.", "......#.", "......#.", "......#.", ".#####..",
            ]
        );
    }

    #[test]
    fn filled_rectangle_and_glyph() {
        let mut cv = Canvas::new(128, 32);
        let p = comp(3, 0, 0, 128, 32, FILL, 0xffffff, 0, 0, 0);
        let (c, e) = Component::parse(&p).unwrap();
        cv.draw(&c, e);
        assert!(cv.pixels().iter().all(|&p| p));
        let p = comp(3, 0, 0, 128, 32, FILL, 0, 0xffffff, 0, 0);
        let (c, e) = Component::parse(&p).unwrap();
        cv.draw(&c, e);
        assert!(cv.pixels().iter().all(|&p| !p));
        // The PIN screen's "down" arrow.
        let p = comp(5, 3, 14, 7, 4, 0, 0xffffff, 0, 0, 12);
        assert_eq!(describe(&p), "icon id 0 at 3,14 size 7x4, glyph 12 (down)");
        let (c, e) = Component::parse(&p).unwrap();
        cv.draw(&c, e);
        assert_eq!(rows(&cv, 14, 4, 3, 7), ["#.....#", ".#...#.", "..#.#..", "...#..."]);
    }

    #[test]
    fn centered_label_line() {
        let mut p = comp(7, 0, 12, 128, 32, 0, 0xffffff, 0, 0x800a, 0);
        p.extend(b"I");
        assert_eq!(describe(&p), "label line id 0 at 0,12 size 128x32 \"I\"");
        let (c, e) = Component::parse(&p).unwrap();
        let mut cv = Canvas::new(128, 32);
        cv.draw(&c, e);
        // One 5-column character centered: starts at 64 - 5/2 = 62; the
        // baseline is row 11.
        assert_eq!(
            rows(&cv, 5, 7, 62, 5),
            [".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."]
        );
        assert!(!cv.pixels[12 * 128 + 64]);
    }
}
