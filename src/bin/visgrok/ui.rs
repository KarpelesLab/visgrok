//! Interactive terminal UI.

use std::io;
use std::time::{Duration, Instant};

use ratatui::crossterm::event::{self, Event as TermEvent, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell, Clear, List, ListItem, ListState, Paragraph, Row, Table, TableState};
use ratatui::{DefaultTerminal, Frame};
use visgrok::analyzer::{Analyzer, SpiProtocol};
use visgrok::decode::DisplayView;
use visgrok::roles::{BAUD_RATES, Role, Suggestion, fmt_hz};

#[cfg(test)]
use crate::pipeline::Setup;
use crate::pipeline::{Pipeline, format_annotation};

/// UI state that is not part of the pipeline.
struct Ui {
    table: TableState,
    /// Waveform window width, in samples.
    zoom: u64,
    /// Display frozen (capture continues).
    paused: bool,
    /// Snapshot shown while paused.
    frozen: Option<Snapshot>,
    /// Edge counts at the previous rate sample, for edges/s.
    last_edges: Vec<u64>,
    edge_rate: Vec<f64>,
    last_rate_at: Instant,
    auto: bool,
    auto_applied: bool,
    message: Option<(String, Instant)>,
    /// Role picker for the selected channel, when open.
    picker: Option<ListState>,
    /// Show every channel even when there are many.
    show_all: bool,
}

/// Channels shown in the table and waveform. Up to 16 channels: all of
/// them. Beyond that (SLogic32): those with activity, a role or a custom
/// name, unless the user asked for all.
fn visible(pipe: &Pipeline, ui: &Ui, snap: &Snapshot) -> Vec<usize> {
    let n = pipe.info.channels;
    if n <= 16 || ui.show_all {
        return (0..n).collect();
    }
    let roles = pipe.roles.lock().unwrap();
    let v: Vec<usize> = (0..n)
        .filter(|&i| {
            snap.channels.get(i).is_some_and(|c| c.edges > 0)
                || roles.get(i).is_some_and(|r| r.is_some())
                || pipe.names[i] != format!("D{i}")
        })
        .collect();
    if v.is_empty() { (0..8.min(n)).collect() } else { v }
}

/// Roles offered by the picker.
const PICKS: &[&str] = &[
    "(none)",
    "UART (auto baud, follows rate changes)",
    "SPI SCLK",
    "SPI MOSI / DI",
    "SPI MISO / DO",
    "SPI CS",
    "SPI D/C (data/command)",
    "I2C SCL",
    "I2C SDA",
    "SD CLK",
    "SD CMD",
    "SD DAT0",
    "SD DAT1",
    "SD DAT2",
    "SD DAT3",
    "idle (ignore)",
];

fn pick_role(i: usize, ch: usize, roles: &[Option<Role>]) -> Option<Role> {
    let find = |f: &dyn Fn(&Role) -> bool| {
        roles
            .iter()
            .enumerate()
            .find(|(j, r)| *j != ch && r.as_ref().is_some_and(f))
            .map(|(j, _)| j)
    };
    match i {
        1 => Some(Role::Uart { baud: 0 }),
        2 => Some(Role::SpiClk),
        3 => Some(Role::SpiMosi),
        4 => Some(Role::SpiMiso),
        5 => Some(Role::SpiCs),
        6 => Some(Role::SpiDc),
        7 => {
            let sda = find(&|r| matches!(r, Role::I2cSda { .. })).unwrap_or(ch + 1);
            Some(Role::I2cScl { sda: sda as u8 })
        }
        8 => {
            let scl = find(&|r| matches!(r, Role::I2cScl { .. })).unwrap_or(ch.saturating_sub(1));
            Some(Role::I2cSda { scl: scl as u8 })
        }
        9 => Some(Role::SdClk),
        10 => Some(Role::SdCmd),
        11..=14 => Some(Role::SdDat(i as u8 - 11)),
        15 => Some(Role::Idle),
        _ => None,
    }
}

/// Keeps I2C pairs pointing at each other after an assignment.
fn fix_i2c_pairs(roles: &mut [Option<Role>]) {
    let scl = roles.iter().position(|r| matches!(r, Some(Role::I2cScl { .. })));
    let sda = roles.iter().position(|r| matches!(r, Some(Role::I2cSda { .. })));
    if let (Some(c), Some(d)) = (scl, sda) {
        roles[c] = Some(Role::I2cScl { sda: d as u8 });
        roles[d] = Some(Role::I2cSda { scl: c as u8 });
    }
}

/// Everything needed to draw one frame, copied out of the analyzer so the
/// lock is held only briefly.
#[derive(Clone)]
struct Snapshot {
    samples: u64,
    state: Option<u32>,
    channels: Vec<ChannelRow>,
    suggestions: Vec<Suggestion>,
    wave: Vec<visgrok::Transition>,
    log: Vec<String>,
    gaps: u64,
    display: Option<DisplayView>,
}

#[derive(Clone)]
struct ChannelRow {
    edges: u64,
    period: Option<u64>,
    duty: Option<f64>,
    min_pulse: Option<u64>,
}

fn snapshot(a: &Analyzer, window: u64) -> Snapshot {
    let st = a.stats();
    let samples = st.samples;
    let from = samples.saturating_sub(window);
    // Include the transition just before the window, to know the initial level.
    let start = a.wave.partition_point(|t| t.at < from).saturating_sub(1);
    let wave: Vec<_> = a.wave.range(start..).copied().collect();
    let names: Vec<String> = a.decoders().iter().map(|d| d.name()).collect();
    let skip = a.annotations.len().saturating_sub(200);
    Snapshot {
        samples,
        state: a.state(),
        channels: st
            .channels
            .iter()
            .map(|c| ChannelRow {
                edges: c.edges(),
                period: c.median_period(),
                duty: c.duty(),
                min_pulse: Some(c.min_high.min(c.min_low)).filter(|&m| m != u64::MAX),
            })
            .collect(),
        suggestions: a.suggest(),
        wave,
        log: a
            .annotations
            .iter()
            .skip(skip)
            .filter(|t| !matches!(t.annotation.event, visgrok::decode::Event::Frame(_)))
            .map(|t| format_annotation(t, &names, a.samplerate()))
            .collect(),
        gaps: a.gaps,
        display: a.decoders().iter().find_map(|d| d.display()),
    }
}

/// Runs the UI until the user quits or the capture ends and is dismissed.
pub fn run(pipe: &Pipeline, auto: bool) -> io::Result<()> {
    let mut term = ratatui::init();
    let r = event_loop(&mut term, pipe, auto);
    ratatui::restore();
    r
}

fn event_loop(term: &mut DefaultTerminal, pipe: &Pipeline, auto: bool) -> io::Result<()> {
    let n = pipe.info.channels;
    let mut ui = Ui {
        table: TableState::default().with_selected(Some(0)),
        zoom: (pipe.info.samplerate / 1000).max(100), // 1 ms
        paused: false,
        frozen: None,
        last_edges: vec![0; n],
        edge_rate: vec![0.0; n],
        last_rate_at: Instant::now(),
        auto,
        auto_applied: false,
        message: None,
        picker: None,
        show_all: false,
    };
    loop {
        if ui.auto && !ui.auto_applied && pipe.seconds() >= 2.0 {
            pipe.apply_suggestions();
            ui.auto_applied = true;
            ui.flash("auto-detected roles applied");
        }
        let snap = match (&ui.frozen, ui.paused) {
            (Some(s), true) => s.clone(),
            _ => snapshot(&pipe.analyzer.lock().unwrap(), ui.zoom),
        };
        let dt = ui.last_rate_at.elapsed().as_secs_f64();
        if dt >= 1.0 {
            for (i, c) in snap.channels.iter().enumerate() {
                ui.edge_rate[i] = c.edges.saturating_sub(ui.last_edges[i]) as f64 / dt;
                ui.last_edges[i] = c.edges;
            }
            ui.last_rate_at = Instant::now();
        }
        term.draw(|f| draw(f, pipe, &mut ui, &snap))?;

        if !event::poll(Duration::from_millis(50))? {
            continue;
        }
        let TermEvent::Key(k) = event::read()? else { continue };
        if k.kind != KeyEventKind::Press {
            continue;
        }
        let vis = visible(pipe, &ui, &snap);
        let row = ui.table.selected().unwrap_or(0).min(vis.len().saturating_sub(1));
        let sel = vis.get(row).copied().unwrap_or(0);
        if let Some(p) = &mut ui.picker {
            let i = p.selected().unwrap_or(0);
            match k.code {
                KeyCode::Down | KeyCode::Char('j') => p.select(Some((i + 1).min(PICKS.len() - 1))),
                KeyCode::Up | KeyCode::Char('k') => p.select(Some(i.saturating_sub(1))),
                KeyCode::Enter => {
                    {
                        let mut roles = pipe.roles.lock().unwrap();
                        let role = pick_role(i, sel, &roles);
                        roles[sel] = role;
                        fix_i2c_pairs(&mut roles);
                    }
                    pipe.rebuild_decoders();
                    ui.flash(&format!("{}: {}", pipe.names[sel], PICKS[i]));
                    ui.picker = None;
                }
                KeyCode::Esc | KeyCode::Char('q') => ui.picker = None,
                _ => {}
            }
            continue;
        }
        match k.code {
            KeyCode::Enter => ui.picker = Some(ListState::default().with_selected(Some(0))),
            KeyCode::Char('p') => {
                let next = {
                    let mut o = pipe.options.lock().unwrap();
                    o.spi_protocol = match o.spi_protocol {
                        SpiProtocol::Raw => SpiProtocol::Ssd1306 { width: 128, height: 64 },
                        SpiProtocol::Ssd1306 { height: 64, .. } => SpiProtocol::Ssd1306 { width: 128, height: 32 },
                        _ => SpiProtocol::Raw,
                    };
                    o.spi_protocol
                };
                pipe.rebuild_decoders();
                ui.flash(&format!("SPI protocol: {next:?}"));
            }
            KeyCode::Char('m') => {
                let next = {
                    let mut o = pipe.options.lock().unwrap();
                    o.spi_mode = match o.spi_mode {
                        None => Some(0),
                        Some(3) => None,
                        Some(m) => Some(m + 1),
                    };
                    o.spi_mode
                };
                pipe.rebuild_decoders();
                ui.flash(&match next {
                    Some(m) => format!("SPI mode {m}"),
                    None => "SPI mode auto (CPOL from idle clock)".into(),
                });
            }
            KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
            KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL) => return Ok(()),
            KeyCode::Down | KeyCode::Char('j') => ui.table.select(Some((row + 1).min(vis.len().saturating_sub(1)))),
            KeyCode::Up | KeyCode::Char('k') => ui.table.select(Some(row.saturating_sub(1))),
            KeyCode::Char('v') => {
                ui.show_all = !ui.show_all;
                ui.flash(if ui.show_all {
                    "showing all channels"
                } else {
                    "showing active/assigned channels"
                });
            }
            KeyCode::Char('+') | KeyCode::Char('=') => ui.zoom = (ui.zoom / 2).max(16),
            KeyCode::Char('-') => ui.zoom = (ui.zoom * 2).min(pipe.info.samplerate * 10),
            KeyCode::Char(' ') => {
                ui.paused = !ui.paused;
                ui.frozen = ui.paused.then(|| snap.clone());
            }
            KeyCode::Char('a') => {
                pipe.apply_suggestions();
                ui.flash("suggested roles applied to unassigned channels");
            }
            KeyCode::Char('A') => {
                // Accept the suggestion for the selected channel only.
                if let Some(s) = snap.suggestions.get(sel) {
                    set_role(pipe, sel, Some(s.role.clone()));
                    ui.flash(&format!("ch{sel}: {}", s.role));
                }
            }
            KeyCode::Char('x') => {
                set_role(pipe, sel, None);
                ui.flash(&format!("ch{sel}: role cleared"));
            }
            KeyCode::Char('u') => {
                // UART; pressing again cycles through standard baud rates.
                let cur = pipe.roles.lock().unwrap()[sel].clone();
                let baud = match cur {
                    Some(Role::Uart { baud }) => match BAUD_RATES.iter().position(|&b| b == baud) {
                        Some(i) if i + 1 < BAUD_RATES.len() => BAUD_RATES[i + 1],
                        Some(_) => 0,
                        None => BAUD_RATES[0],
                    },
                    _ => 0,
                };
                set_role(pipe, sel, Some(Role::Uart { baud }));
                ui.flash(&if baud == 0 {
                    format!("ch{sel}: UART auto baud")
                } else {
                    format!("ch{sel}: UART starting at {baud}")
                });
            }
            KeyCode::Char('i') if sel + 1 < n => {
                {
                    let mut r = pipe.roles.lock().unwrap();
                    r[sel] = Some(Role::I2cScl { sda: (sel + 1) as u8 });
                    r[sel + 1] = Some(Role::I2cSda { scl: sel as u8 });
                }
                pipe.rebuild_decoders();
                ui.flash(&format!("I2C: SCL=ch{sel} SDA=ch{}", sel + 1));
            }
            KeyCode::Char('r') => {
                pipe.analyzer.lock().unwrap().reset_stats();
                ui.flash("statistics reset");
            }
            _ => {}
        }
    }
}

fn set_role(pipe: &Pipeline, ch: usize, role: Option<Role>) {
    pipe.roles.lock().unwrap()[ch] = role;
    pipe.rebuild_decoders();
}

impl Ui {
    fn flash(&mut self, msg: &str) {
        self.message = Some((msg.to_string(), Instant::now()));
    }
}

fn draw(f: &mut Frame, pipe: &Pipeline, ui: &mut Ui, snap: &Snapshot) {
    let vis = visible(pipe, ui, snap);
    let n = vis.len() as u16;
    let disp_rows = snap.display.as_ref().map_or(0, |d| d.height.div_ceil(4) as u16 + 2);
    let [header, middle, wave, bottom, help] = Layout::vertical([
        Constraint::Length(3),
        Constraint::Length(n + 3),
        Constraint::Length(n + 2),
        Constraint::Min(4.max(disp_rows)),
        Constraint::Length(1),
    ])
    .areas(f.area());
    let log = match &snap.display {
        Some(d) => {
            let w = d.width.div_ceil(2) as u16 + 2;
            let [log, panel] = Layout::horizontal([Constraint::Min(20), Constraint::Length(w)]).areas(bottom);
            draw_display(f, panel, d);
            log
        }
        None => bottom,
    };

    draw_header(f, header, pipe, ui, snap);
    draw_channels(f, middle, pipe, ui, snap, &vis);
    draw_wave(f, wave, pipe, ui, snap, &vis);

    let lines: Vec<Line> = snap
        .log
        .iter()
        .rev()
        .take(log.height.saturating_sub(2) as usize)
        .rev()
        .map(|l| Line::raw(l.as_str()))
        .collect();
    let title = format!(
        " Decoded ({} decoders) ",
        pipe.analyzer.lock().map(|a| a.decoders().len()).unwrap_or(0)
    );
    f.render_widget(
        Paragraph::new(lines).block(Block::default().borders(Borders::ALL).title(title)),
        log,
    );

    let mut help_spans = vec![Span::styled(
        " q quit  ↑↓ select  ⏎ set role  a auto-assign all  A accept  u UART  i I2C  x clear  p SPI proto  m SPI mode  v all ch  +/- zoom  space pause  r reset",
        Style::default().fg(Color::DarkGray),
    )];
    if let Some((m, at)) = &ui.message
        && at.elapsed() < Duration::from_secs(4)
    {
        help_spans = vec![Span::styled(format!(" {m}"), Style::default().fg(Color::Yellow))];
    }
    f.render_widget(Paragraph::new(Line::from(help_spans)), help);

    if let Some(p) = &mut ui.picker {
        let sel = vis.get(ui.table.selected().unwrap_or(0)).copied().unwrap_or(0);
        let area = f.area();
        let w = 46.min(area.width);
        let h = (PICKS.len() as u16 + 2).min(area.height);
        let r = Rect {
            x: area.x + (area.width - w) / 2,
            y: area.y + (area.height - h) / 2,
            width: w,
            height: h,
        };
        let items: Vec<ListItem> = PICKS.iter().map(|s| ListItem::new(*s)).collect();
        let list = List::new(items)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(format!(" Role for {} (D{sel}) ", pipe.names[sel])),
            )
            .highlight_style(Style::default().bg(Color::Blue).add_modifier(Modifier::BOLD));
        f.render_widget(Clear, r);
        f.render_stateful_widget(list, r, p);
    }
}

/// Renders a reconstructed display with braille dots (2×4 pixels per cell).
fn draw_display(f: &mut Frame, area: Rect, d: &DisplayView) {
    let title = format!(
        " {} {}×{} {} · {} updates ",
        d.title,
        d.width,
        d.height,
        if d.on { "on" } else { "off" },
        d.updates
    );
    let block = Block::default().borders(Borders::ALL).title(title);
    let inner = block.inner(area);
    f.render_widget(block, area);
    const BITS: [[u32; 4]; 2] = [[0x01, 0x02, 0x04, 0x40], [0x08, 0x10, 0x20, 0x80]];
    let mut lines = Vec::new();
    for cy in 0..d.height.div_ceil(4).min(inner.height as usize) {
        let mut row = String::new();
        for cx in 0..d.width.div_ceil(2).min(inner.width as usize) {
            let mut v = 0;
            for (dx, col) in BITS.iter().enumerate() {
                for (dy, bit) in col.iter().enumerate() {
                    let (x, y) = (cx * 2 + dx, cy * 4 + dy);
                    if x < d.width && y < d.height && d.pixels[y * d.width + x] {
                        v |= bit;
                    }
                }
            }
            row.push(char::from_u32(0x2800 + v).unwrap_or(' '));
        }
        lines.push(Line::raw(row));
    }
    let color = if d.on { Color::Cyan } else { Color::DarkGray };
    f.render_widget(Paragraph::new(lines).style(Style::default().fg(color)), inner);
}

fn draw_header(f: &mut Frame, area: Rect, pipe: &Pipeline, ui: &Ui, snap: &Snapshot) {
    let t = pipe.seconds();
    let samples = pipe.samples();
    let mut status = vec![
        Span::styled(format!(" {} ", pipe.info.device), Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(format!(
            "│ {} ch @ {} │ {:.1}s │ {} samples ({:.1} MS/s) ",
            pipe.info.channels,
            fmt_hz(pipe.info.samplerate as f64),
            t,
            samples,
            samples as f64 / t.max(1e-9) / 1e6
        )),
    ];
    if let Some(p) = &pipe.output {
        status.push(Span::raw(format!("│ {} → {} ", pipe.written_text(), p.display())));
    }
    if pipe.finished() {
        status.push(Span::styled("│ CAPTURE ENDED ", Style::default().fg(Color::Yellow)));
    }
    if ui.paused {
        status.push(Span::styled("│ PAUSED ", Style::default().fg(Color::Cyan)));
    }
    if let Some(e) = pipe.error() {
        status.push(Span::styled(format!("│ ERROR: {e} "), Style::default().fg(Color::Red)));
    }
    let skipped = pipe.blocks_skipped();
    if skipped > 0 || snap.gaps > 0 {
        status.push(Span::styled(
            format!("│ analysis behind: {skipped} blocks skipped "),
            Style::default().fg(Color::Magenta),
        ));
    }
    f.render_widget(
        Paragraph::new(Line::from(status)).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

fn draw_channels(f: &mut Frame, area: Rect, pipe: &Pipeline, ui: &mut Ui, snap: &Snapshot, vis: &[usize]) {
    let sr = pipe.info.samplerate as f64;
    let roles = pipe.roles.lock().unwrap().clone();
    let name_w = pipe.names.iter().map(|n| n.chars().count()).max().unwrap_or(2).max(2) as u16 + 1;
    let rows = vis.iter().map(|&i| {
        let c = &snap.channels[i];
        let level = snap.state.map(|s| s >> i & 1 != 0);
        let lvl = match level {
            Some(true) => Span::styled("HIGH", Style::default().fg(Color::Green)),
            Some(false) => Span::styled("low ", Style::default().fg(Color::DarkGray)),
            None => Span::raw("  - "),
        };
        let freq = c.period.map(|p| fmt_hz(sr / p as f64)).unwrap_or_default();
        let duty = c.duty.map(|d| format!("{:5.1}%", d * 100.0)).unwrap_or_default();
        let minp = c.min_pulse.map(|m| fmt_time(m as f64 / sr)).unwrap_or_default();
        let assigned = roles.get(i).cloned().flatten();
        let sugg = snap.suggestions.get(i);
        let role = match (&assigned, sugg) {
            (Some(r), _) => Span::styled(r.to_string(), Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
            (None, Some(s)) if s.role != Role::Unknown => Span::styled(
                format!("{}? ({:.0}%)", s.role, s.confidence * 100.0),
                Style::default().fg(Color::DarkGray),
            ),
            _ => Span::raw(""),
        };
        Row::new(vec![
            Cell::from(pipe.names[i].clone()),
            Cell::from(lvl),
            Cell::from(format!("{}", c.edges)),
            Cell::from(fmt_rate(ui.edge_rate[i])),
            Cell::from(freq),
            Cell::from(duty),
            Cell::from(minp),
            Cell::from(role),
        ])
    });
    let table = Table::new(
        rows,
        [
            Constraint::Length(name_w),
            Constraint::Length(5),
            Constraint::Length(12),
            Constraint::Length(10),
            Constraint::Length(13),
            Constraint::Length(7),
            Constraint::Length(10),
            Constraint::Min(20),
        ],
    )
    .header(
        Row::new(["Ch", "Lvl", "Edges", "Edges/s", "Frequency", "Duty", "Min pulse", "Role"])
            .style(Style::default().add_modifier(Modifier::BOLD)),
    )
    .row_highlight_style(Style::default().bg(Color::DarkGray))
    .block(Block::default().borders(Borders::ALL).title(if vis.len() < pipe.info.channels {
        format!(" Channels ({} of {} shown, v: all) ", vis.len(), pipe.info.channels)
    } else {
        " Channels ".to_string()
    }));
    f.render_stateful_widget(table, area, &mut ui.table);
}

fn draw_wave(f: &mut Frame, area: Rect, pipe: &Pipeline, ui: &Ui, snap: &Snapshot, vis: &[usize]) {
    let sr = pipe.info.samplerate as f64;
    let title = format!(" Waveform: last {} ", fmt_time(ui.zoom as f64 / sr));
    let block = Block::default().borders(Borders::ALL).title(title);
    let inner = block.inner(area);
    f.render_widget(block, area);
    let label_w = pipe.names.iter().map(|n| n.chars().count()).max().unwrap_or(2) as u16 + 1;
    let cols = inner.width.saturating_sub(label_w) as u64;
    if cols == 0 {
        return;
    }
    let end = snap.samples;
    let start = end.saturating_sub(ui.zoom);
    let span = (end - start).max(1);
    let mut lines = Vec::new();
    for &ch in vis {
        let bit = 1u32 << ch;
        // Level at window start.
        let mut idx = snap.wave.partition_point(|t| t.at <= start);
        let mut level = if idx > 0 {
            snap.wave[idx - 1].now & bit != 0
        } else {
            snap.wave
                .first()
                .map(|t| t.prev & bit != 0)
                .or(snap.state.map(|s| s & bit != 0))
                .unwrap_or(false)
        };
        let mut s = String::with_capacity(cols as usize * 3);
        for col in 0..cols {
            let a = start + span * col / cols;
            let b = start + span * (col + 1) / cols;
            let mut toggles = 0;
            while idx < snap.wave.len() && snap.wave[idx].at < b {
                if snap.wave[idx].changed() & bit != 0 {
                    toggles += 1;
                    level = snap.wave[idx].now & bit != 0;
                }
                idx += 1;
            }
            let _ = a;
            s.push(match toggles {
                0 if level => '▔',
                0 => '▁',
                1 => '│',
                _ => '█',
            });
        }
        let color = if snap.channels.get(ch).is_some_and(|c| c.edges > 0) {
            Color::Green
        } else {
            Color::DarkGray
        };
        lines.push(Line::from(vec![
            Span::styled(
                format!("{:<w$}", pipe.names[ch], w = label_w as usize),
                Style::default().fg(Color::Gray),
            ),
            Span::styled(s, Style::default().fg(color)),
        ]));
    }
    f.render_widget(Paragraph::new(lines), inner);
}

fn fmt_time(s: f64) -> String {
    if s >= 1.0 {
        format!("{s:.3} s")
    } else if s >= 1e-3 {
        format!("{:.3} ms", s * 1e3)
    } else if s >= 1e-6 {
        format!("{:.3} µs", s * 1e6)
    } else {
        format!("{:.1} ns", s * 1e9)
    }
}

fn fmt_rate(r: f64) -> String {
    if r >= 1e6 {
        format!("{:.2}M", r / 1e6)
    } else if r >= 1e3 {
        format!("{:.1}k", r / 1e3)
    } else {
        format!("{r:.0}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;
    use visgrok::synth::Synth;

    /// Renders one frame of a short synthetic capture; run with
    /// `--nocapture` to eyeball the layout.
    #[test]
    fn renders_demo() {
        let pipe = Pipeline::start(Box::new(Synth::new(20_000_000, Some(20_000_000))), None, Setup::default()).unwrap();
        while !pipe.finished() {
            std::thread::sleep(Duration::from_millis(50));
        }
        pipe.join();
        pipe.apply_suggestions();
        let n = pipe.info.channels;
        let mut ui = Ui {
            table: TableState::default().with_selected(Some(0)),
            zoom: 20_000,
            paused: false,
            frozen: None,
            last_edges: vec![0; n],
            edge_rate: vec![0.0; n],
            last_rate_at: Instant::now(),
            auto: false,
            auto_applied: false,
            message: None,
            picker: None,
            show_all: false,
        };
        let snap = snapshot(&pipe.analyzer.lock().unwrap(), ui.zoom);
        let mut term = Terminal::new(TestBackend::new(150, 40)).unwrap();
        term.draw(|f| draw(f, &pipe, &mut ui, &snap)).unwrap();
        let buf = term.backend().buffer();
        let mut out = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                out.push_str(buf[(x, y)].symbol());
            }
            out.push('\n');
        }
        println!("{out}");
        assert!(out.contains("UART 115200"));
    }

    /// The device demo moved to channels 24..28 of a 32-channel stream.
    struct High(Synth);

    impl visgrok::Source for High {
        fn info(&self) -> visgrok::CaptureInfo {
            let mut i = self.0.info();
            i.channels = 32;
            i.unit_size = 4;
            i
        }
        fn next_block(&mut self) -> std::io::Result<Option<visgrok::Block>> {
            Ok(self.0.next_block()?.map(|b| {
                let data = b.data.iter().flat_map(|&v| ((v as u32) << 24).to_le_bytes()).collect();
                visgrok::Block::new(b.start, 4, data)
            }))
        }
    }

    #[test]
    fn renders_32_channels() {
        use visgrok::analyzer::{DecoderOptions, SpiProtocol};
        let mut roles = vec![None; 32];
        roles[24] = Some(Role::Uart { baud: 0 });
        roles[25] = Some(Role::SpiClk);
        roles[26] = Some(Role::SpiMosi);
        roles[27] = Some(Role::SpiDc);
        roles[28] = Some(Role::SpiCs);
        let mut opts = DecoderOptions::default();
        opts.spi_protocol = SpiProtocol::Ssd1306 { width: 128, height: 64 };
        let pipe = Pipeline::start(
            Box::new(High(Synth::device(50_000_000, Some(15_000_000)))),
            None,
            Setup {
                roles,
                options: opts,
                ..Default::default()
            },
        )
        .unwrap();
        while !pipe.finished() {
            std::thread::sleep(Duration::from_millis(50));
        }
        pipe.join();
        let mut ui = Ui {
            table: TableState::default().with_selected(Some(0)),
            zoom: 50_000,
            paused: false,
            frozen: None,
            last_edges: vec![0; 32],
            edge_rate: vec![0.0; 32],
            last_rate_at: Instant::now(),
            auto: false,
            auto_applied: false,
            message: None,
            picker: None,
            show_all: false,
        };
        let snap = snapshot(&pipe.analyzer.lock().unwrap(), ui.zoom);
        let mut term = Terminal::new(TestBackend::new(160, 48)).unwrap();
        term.draw(|f| draw(f, &pipe, &mut ui, &snap)).unwrap();
        let buf = term.backend().buffer();
        let mut out = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                out.push_str(buf[(x, y)].symbol());
            }
            out.push('\n');
        }
        println!("{out}");
        assert!(out.contains("5 of 32 shown"), "{out}");
        assert!(out.contains("D28"));
        assert!(out.contains("SSD1306 128×64 on"));
        assert!(out.contains("baud rate"));
    }

    #[test]
    fn renders_device_demo_with_oled() {
        use visgrok::analyzer::{DecoderOptions, SpiProtocol};
        let roles = vec![
            Some(Role::Uart { baud: 0 }),
            Some(Role::SpiClk),
            Some(Role::SpiMosi),
            Some(Role::SpiDc),
            Some(Role::SpiCs),
        ];
        let mut opts = DecoderOptions::default();
        opts.spi_protocol = SpiProtocol::Ssd1306 { width: 128, height: 64 };
        let pipe = Pipeline::start(
            Box::new(Synth::device(50_000_000, Some(15_000_000))),
            None,
            Setup {
                roles,
                options: opts,
                ..Default::default()
            },
        )
        .unwrap();
        while !pipe.finished() {
            std::thread::sleep(Duration::from_millis(50));
        }
        pipe.join();
        let n = pipe.info.channels;
        let mut ui = Ui {
            table: TableState::default().with_selected(Some(0)),
            zoom: 50_000,
            paused: false,
            frozen: None,
            last_edges: vec![0; n],
            edge_rate: vec![0.0; n],
            last_rate_at: Instant::now(),
            auto: false,
            auto_applied: false,
            message: None,
            picker: None,
            show_all: false,
        };
        let snap = snapshot(&pipe.analyzer.lock().unwrap(), ui.zoom);
        let mut term = Terminal::new(TestBackend::new(160, 48)).unwrap();
        term.draw(|f| draw(f, &pipe, &mut ui, &snap)).unwrap();
        let buf = term.backend().buffer();
        let mut out = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                out.push_str(buf[(x, y)].symbol());
            }
            out.push('\n');
        }
        println!("{out}");
        assert!(out.contains("SSD1306 128×64 on"), "{out}");
        assert!(out.contains("baud rate"), "{out}");
    }
}
