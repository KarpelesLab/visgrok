//! Interactive terminal UI.

use std::io;
use std::time::{Duration, Instant};

use ratatui::crossterm::event::{self, Event as TermEvent, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, Table, TableState};
use ratatui::{DefaultTerminal, Frame};
use visgrok::analyzer::Analyzer;
use visgrok::roles::{BAUD_RATES, Role, Suggestion, fmt_hz};

use crate::pipeline::{Pipeline, fmt_bytes, format_annotation};

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
}

/// Everything needed to draw one frame, copied out of the analyzer so the
/// lock is held only briefly.
#[derive(Clone)]
struct Snapshot {
    samples: u64,
    state: Option<u16>,
    channels: Vec<ChannelRow>,
    suggestions: Vec<Suggestion>,
    wave: Vec<visgrok::Transition>,
    log: Vec<String>,
    gaps: u64,
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
        log: a.annotations.iter().skip(skip).map(|t| format_annotation(t, &names, a.samplerate())).collect(),
        gaps: a.gaps,
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
        let sel = ui.table.selected().unwrap_or(0);
        match k.code {
            KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
            KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL) => return Ok(()),
            KeyCode::Down | KeyCode::Char('j') => ui.table.select(Some((sel + 1).min(n - 1))),
            KeyCode::Up | KeyCode::Char('k') => ui.table.select(Some(sel.saturating_sub(1))),
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
                    Some(Role::Uart { baud }) => {
                        let i = BAUD_RATES.iter().position(|&b| b == baud).unwrap_or(0);
                        BAUD_RATES[(i + 1) % BAUD_RATES.len()]
                    }
                    _ => match snap.suggestions.get(sel).map(|s| &s.role) {
                        Some(Role::Uart { baud }) => *baud,
                        _ => 115_200,
                    },
                };
                set_role(pipe, sel, Some(Role::Uart { baud }));
                ui.flash(&format!("ch{sel}: UART {baud}"));
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
            KeyCode::Char('s') if sel + 3 < n => {
                {
                    let mut r = pipe.roles.lock().unwrap();
                    r[sel] = Some(Role::SpiClk);
                    r[sel + 1] = Some(Role::SpiData { clk: sel as u8 });
                    r[sel + 2] = Some(Role::SpiData { clk: sel as u8 });
                    r[sel + 3] = Some(Role::SpiCs);
                }
                pipe.rebuild_decoders();
                ui.flash(&format!("SPI: CLK/MOSI/MISO/CS = ch{}..ch{}", sel, sel + 3));
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
    let n = pipe.info.channels as u16;
    let [header, middle, wave, log, help] = Layout::vertical([
        Constraint::Length(3),
        Constraint::Length(n + 3),
        Constraint::Length(n + 2),
        Constraint::Min(4),
        Constraint::Length(1),
    ])
    .areas(f.area());

    draw_header(f, header, pipe, ui, snap);
    draw_channels(f, middle, pipe, ui, snap);
    draw_wave(f, wave, pipe, ui, snap);

    let lines: Vec<Line> = snap
        .log
        .iter()
        .rev()
        .take(log.height.saturating_sub(2) as usize)
        .rev()
        .map(|l| Line::raw(l.as_str()))
        .collect();
    let title = format!(" Decoded ({} decoders) ", pipe.analyzer.lock().map(|a| a.decoders().len()).unwrap_or(0));
    f.render_widget(Paragraph::new(lines).block(Block::default().borders(Borders::ALL).title(title)), log);

    let mut help_spans = vec![Span::styled(
        " q quit  ↑↓ select  a auto-assign all  A accept selected  u UART/baud  i I2C(sel,sel+1)  s SPI(sel..sel+3)  x clear  +/- zoom  space pause  r reset stats",
        Style::default().fg(Color::DarkGray),
    )];
    if let Some((m, at)) = &ui.message
        && at.elapsed() < Duration::from_secs(4)
    {
        help_spans = vec![Span::styled(format!(" {m}"), Style::default().fg(Color::Yellow))];
    }
    f.render_widget(Paragraph::new(Line::from(help_spans)), help);
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
        status.push(Span::raw(format!("│ {} → {} ", fmt_bytes(pipe.written()), p.display())));
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
    f.render_widget(Paragraph::new(Line::from(status)).block(Block::default().borders(Borders::ALL)), area);
}

fn draw_channels(f: &mut Frame, area: Rect, pipe: &Pipeline, ui: &mut Ui, snap: &Snapshot) {
    let sr = pipe.info.samplerate as f64;
    let roles = pipe.roles.lock().unwrap().clone();
    let rows = snap.channels.iter().enumerate().map(|(i, c)| {
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
            Cell::from(format!("D{i}")),
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
            Constraint::Length(4),
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
    .block(Block::default().borders(Borders::ALL).title(" Channels "));
    f.render_stateful_widget(table, area, &mut ui.table);
}

fn draw_wave(f: &mut Frame, area: Rect, pipe: &Pipeline, ui: &Ui, snap: &Snapshot) {
    let sr = pipe.info.samplerate as f64;
    let title = format!(" Waveform: last {} ", fmt_time(ui.zoom as f64 / sr));
    let block = Block::default().borders(Borders::ALL).title(title);
    let inner = block.inner(area);
    f.render_widget(block, area);
    let label_w = 4u16;
    let cols = inner.width.saturating_sub(label_w) as u64;
    if cols == 0 {
        return;
    }
    let end = snap.samples;
    let start = end.saturating_sub(ui.zoom);
    let span = (end - start).max(1);
    let mut lines = Vec::new();
    for ch in 0..pipe.info.channels {
        let bit = 1u16 << ch;
        // Level at window start.
        let mut idx = snap.wave.partition_point(|t| t.at <= start);
        let mut level = if idx > 0 {
            snap.wave[idx - 1].now & bit != 0
        } else {
            snap.wave.first().map(|t| t.prev & bit != 0).or(snap.state.map(|s| s & bit != 0)).unwrap_or(false)
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
        let color = if snap.channels.get(ch).is_some_and(|c| c.edges > 0) { Color::Green } else { Color::DarkGray };
        lines.push(Line::from(vec![
            Span::styled(format!("D{ch:<2} "), Style::default().fg(Color::Gray)),
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
        let pipe = Pipeline::start(Box::new(Synth::new(20_000_000, Some(20_000_000))), None).unwrap();
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
}
