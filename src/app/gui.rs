use crate::app::{AppError, ProbeChannel, Result};
use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor, WebColors},
    primitives::*,
    text::{Alignment, Text},
    transform::Transform,
    Drawable, Pixel,
};

use super::{
    unit::{TimeScale, VoltageScale},
    Panel, PanelStyle, RunningState, State, StateMarker, StateVec, WaveformStorage,
};

pub const TEXT_OFFSET: Point = Point::new(0, 2);
pub const SCREEN_WIDTH: u32 = 320;
pub const SCREEN_HEIGHT: u32 = 240;

type StateResult = Result<Option<&'static [StateMarker]>>;
pub trait Draw<D> {
    async fn draw(&self, display: &mut D, state: &mut State, vec: &mut StateVec) -> Result<()>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let state_mask = self.state_emit_mask();
        let do_update = if state_mask.len() != 0 {
            // if any of the state markers are not updated, start drawing
            state_mask.iter().any(|x| !vec.at(*x))
        } else {
            // if no state markers are specified, always draw
            true
        };
        if !do_update {
            return Ok(());
        }
        // crate::info!("calling draw for {:?}", state_mask);
        let updated = self.draw_state(display, state);
        let updated = match updated {
            Ok(x) => Ok(x),
            Err(_) => self.draw_state_vec(display, state, vec),
        }?;
        if let Some(updated) = updated {
            // modify the state vector
            for x in updated {
                match x {
                    StateMarker::AllFlush => {
                        for x in vec.iter_mut() {
                            *x = false;
                        }
                    }
                    _ => {
                        vec.confirm(*x);
                    }
                }
            }
        }
        // yeild for other tasks
        Timer::after_ticks(0).await;
        Ok(())
    }

    fn state_emit_mask(&self) -> &[StateMarker] {
        // nomask: &[]
        &[]
    }

    fn draw_state(&self, _display: &mut D, _state: &mut State) -> StateResult
    where
        D: DrawTarget<Color = Rgb565>,
    {
        Err(AppError::NotImplemented)
    }

    fn draw_state_vec(&self, _display: &mut D, _state: &mut State, _vec: &StateVec) -> StateResult
    where
        D: DrawTarget<Color = Rgb565>,
    {
        Err(AppError::NotImplemented)
    }
}

#[derive(Debug, Default, Clone)]
pub struct GUIInfo {
    pub(crate) size: Size,
    pub(crate) position: Point,
    pub(crate) color_primary: Rgb565,
    pub(crate) color_secondary: Rgb565,
}

impl GUIInfo {
    pub fn width(&self) -> i32 {
        self.size.width as i32
    }
    pub fn height(&self) -> i32 {
        self.size.height as i32
    }
    pub fn size_center(&self) -> Point {
        Point::new(self.size.width as i32 / 2, self.size.height as i32 / 2)
    }
    pub fn bottom_right(&self) -> Point {
        Point::new(
            self.position.x + self.size.width as i32,
            self.position.y + self.size.height as i32,
        )
    }
}

pub struct Waveform {
    pub(crate) info: GUIInfo,
}

impl Default for Waveform {
    fn default() -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(SCREEN_WIDTH - 48 - 4 - 1, SCREEN_HEIGHT - 12 - 12 - 1),
                position: Point::new(4, 12),
                color_primary: Rgb565::CSS_DARK_SLATE_GRAY,
                color_secondary: Rgb565::CSS_LIGHT_GRAY,
                ..Default::default()
            },
        }
    }
}

impl<D> Draw<D> for Waveform
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Waveform, StateMarker::WaveformData]
    }
    fn draw_state_vec(&self, display: &mut D, state: &mut State, vec: &StateVec) -> StateResult {
        let update_full = !vec.at(StateMarker::Waveform);
        let update_data = update_full || !vec.at(StateMarker::WaveformData);
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(self.info.color_secondary)
            .stroke_width(1)
            .fill_color(Rgb565::BLACK)
            .build();
        if update_full {
            Rectangle::new(self.info.position, self.info.size)
                .draw_styled(&style, display)
                .map_err(|_| AppError::DisplayError)?;
            let center = self.info.size_center();
            Line::new(
                Point::new(center.x, 0),
                Point::new(center.x, self.info.height() - 1),
            )
            .translate(self.info.position)
            .draw_styled(&style, display)
            .map_err(|_| AppError::DisplayError)?;
            Line::new(
                Point::new(0, center.y),
                Point::new(self.info.width() - 1, center.y),
            )
            .translate(self.info.position)
            .draw_styled(&style, display)
            .map_err(|_| AppError::DisplayError)?;

            let dl = 1;
            // Draw point grid
            for i in 0..(self.info.size.height as i32 / 40 + 1) {
                for j in 0..(self.info.size.width as i32 / 8) {
                    let x = j * 8 + 6 - 1;
                    let y = i * 40 + 28 - 1;
                    if x == center.x as i32 {
                        continue;
                    }
                    if y == center.y as i32 {
                        Line::new(Point::new(x, y - dl), Point::new(x, y + dl))
                            .translate(self.info.position)
                            .draw_styled(&style, display)
                            .map_err(|_| AppError::DisplayError)?;
                    } else {
                        let p = Point::new(x, y) + self.info.position;
                        Pixel(p, self.info.color_primary)
                            .draw(display)
                            .map_err(|_| AppError::DisplayError)?;
                    }
                }
            }
            for i in 0..(self.info.size.width as i32 / 40 + 1) {
                for j in 0..(self.info.size.height as i32 / 8 + 1) {
                    let x = i * 40 + 14 - 1;
                    let y = j * 8 + 4 - 1;
                    if y == center.y as i32 {
                        continue;
                    }
                    if x == center.x as i32 {
                        Line::new(Point::new(x - dl, y), Point::new(x + dl, y))
                            .translate(self.info.position)
                            .draw_styled(&style, display)
                            .map_err(|_| AppError::DisplayError)?;
                    } else {
                        let p = Point::new(x, y) + self.info.position;
                        Pixel(p, Rgb565::CSS_DARK_SLATE_GRAY)
                            .draw(display)
                            .map_err(|_| AppError::DisplayError)?;
                    }
                }
            }
        }

        if update_data {
            let update_only = state.waveform.linked.len() > 3;
            let color = ProbeChannel::from(state.channel_current).color();
            self.draw_values(display, &mut state.waveform, color, update_only)?;
        }
        if update_data && !update_full {
            Ok(Some(&[StateMarker::WaveformData]))
        } else {
            Ok(Some(&[StateMarker::Waveform, StateMarker::WaveformData]))
        }
    }
}

impl Waveform {
    fn draw_list_values_color<D>(&self, display: &mut D, data: &[f32], color: Rgb565) -> Result<()>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let screen_offset = self.info.position + Point::new(0, self.info.height() / 2);
        let mut pt_last = Point::new(0, 0);
        for (i, pt) in data.iter().enumerate() {
            let pt = Point::new(
                (i * (self.info.width() as usize) / data.len()) as i32,
                ((*pt) * (self.info.height() as f32) / 6.0) as i32,
            );
            if i != 0 {
                Line::new(pt_last, pt)
                    .translate(screen_offset)
                    .draw_styled(&PrimitiveStyle::with_stroke(color, 1), display)
                    .map_err(|_| AppError::DisplayError)?;
            }
            pt_last = pt;
        }
        Ok(())
    }

    fn draw_values<D>(
        &self,
        display: &mut D,
        storage: &mut WaveformStorage,
        color: Rgb565,
        update_only: bool,
    ) -> Result<()>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let color_secondary = Rgb565::new(color.r() / 2, color.g() / 2, color.b() / 2);
        if !update_only {
            for (idx, it) in storage.linked.iter().enumerate() {
                if !it.0 {
                    continue;
                }
                let data = &storage.data[it.1][..storage.len];
                let color = if idx == 0 { color } else { color_secondary };
                self.draw_list_values_color(display, data, color)?;
            }
        } else {
            // clear tail and draw head
            let tail = storage.linked.pop_back().ok_or(AppError::Unexpected)?;
            if tail.0 {
                let data = &storage.data[tail.1][..storage.len];
                self.draw_list_values_color(display, data, Rgb565::BLACK)?;
            }
            storage
                .linked
                .push_back(tail)
                .map_err(|_| AppError::Unexpected)?;
            let head = storage.linked.pop_front().ok_or(AppError::Unexpected)?;
            let head2 = storage.linked.pop_front().ok_or(AppError::Unexpected)?;
            if head2.0 {
                let data = &storage.data[head2.1][..storage.len];
                self.draw_list_values_color(display, data, color_secondary)?;
            }
            if head.0 {
                let data = &storage.data[head.1][..storage.len];
                self.draw_list_values_color(display, data, color)?;
            }
            storage
                .linked
                .push_front(head2)
                .map_err(|_| AppError::Unexpected)?;
            storage
                .linked
                .push_front(head)
                .map_err(|_| AppError::Unexpected)?;
        }
        Ok(())
    }
}

#[derive(Clone)]
struct LineDisp<'a> {
    pub(crate) info: GUIInfo,
    pub(crate) text: &'a str,
    pub(crate) font: MonoTextStyle<'static, Rgb565>,
}
impl<'a> Default for LineDisp<'a> {
    fn default() -> Self {
        Self {
            info: Default::default(),
            text: Default::default(),
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
        }
    }
}

impl<D> Draw<D> for LineDisp<'_>
where
    D: DrawTarget<Color = Rgb565>,
{
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        Rectangle::new(self.info.position, self.info.size)
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(self.info.color_primary)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        Text::with_alignment(
            self.text,
            self.info.size_center() + TEXT_OFFSET,
            self.font,
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}

#[derive(Default)]
pub struct RunningStateDisp;

impl RunningStateDisp {
    fn new_disp<'a>(text: &'a str, color: Rgb565) -> LineDisp<'a> {
        LineDisp {
            info: GUIInfo {
                size: Size::new(29, 10),
                position: Point::new(4, 0),
                color_primary: color,
                color_secondary: Rgb565::WHITE,
            },
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
            text,
            ..Default::default()
        }
    }
}
impl<D> Draw<D> for RunningStateDisp
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::RunningState]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let text: &str = state.running_state.into();
        let disp = Self::new_disp(
            text,
            if state.running_state == RunningState::Running {
                Rgb565::CSS_PURPLE
            } else {
                Rgb565::RED
            },
        );
        disp.draw_state(display, state)?;
        Ok(Some(&[StateMarker::RunningState]))
    }
}

#[derive(Default)]
pub struct TimeScaleDisp;

impl TimeScaleDisp {
    fn new_disp<'a>(text: &'a str) -> LineDisp<'a> {
        LineDisp {
            info: GUIInfo {
                size: Size::new(35, 10),
                position: Point::new(4 + 29 + 1, 0),
                color_primary: Rgb565::MAGENTA,
                color_secondary: Rgb565::WHITE,
            },
            text,
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
        }
    }
}
impl<D> Draw<D> for TimeScaleDisp
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::TimeScale]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let mut buf = [0u8; 16];
        let time_scale = TimeScale::from_ns(state.time_scale_ns);
        let text_unit: &str = time_scale.unit.into();
        let text = format_no_std::show(&mut buf, format_args!("{}{}", time_scale.time, text_unit))
            .map_err(|_| AppError::DataFormatError)?;
        let disp = Self::new_disp(text);
        disp.draw_state(display, state)?;
        Ok(Some(&[StateMarker::TimeScale]))
    }
}

pub struct ChannelSettingDisp(ProbeChannel, LineDisp<'static>);

impl ChannelSettingDisp {
    pub fn new(channel: ProbeChannel, info: GUIInfo) -> Self {
        Self(
            channel,
            LineDisp {
                info,
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
                ..Default::default()
            },
        )
    }
}
impl<D> Draw<D> for ChannelSettingDisp
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::ChannelSetting]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let idx: usize = self.0.into();
        let mut voltage_scale = VoltageScale::from_mv(state.channel_info[idx].voltage_scale_mv);
        let text = voltage_scale.str();
        let disp = LineDisp {
            text,
            ..self.1.clone()
        };
        disp.draw_state(display, state)?;
        // Ok(Some(&[StateMarker::ChannelSetting]))
        Ok(None)
    }
}

pub struct Overview {
    pub(crate) info: GUIInfo,
    text: &'static str,
}
impl Overview {
    pub fn new() -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(133, 10),
                position: Point::new(70, 0),
                color_primary: Rgb565::WHITE,
                color_secondary: Rgb565::BLACK,
            },
            text: "Roll Mode",
        }
    }
}
impl<D> Draw<D> for Overview
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Waveform]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        Rectangle::new(self.info.position, self.info.size)
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .stroke_color(self.info.color_primary)
                    .stroke_width(1)
                    .fill_color(self.info.color_secondary)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        Text::with_alignment(
            self.text,
            self.info.size_center() + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, self.info.color_primary),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}

pub struct Battery {
    pub(crate) info: GUIInfo,
    pub(crate) level: u8,
}

impl Battery {
    pub fn new(level: u8) -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(17, 8),
                position: Point::new(SCREEN_WIDTH as i32 - 17, 1),
                color_primary: Rgb565::WHITE,
                color_secondary: Rgb565::BLACK,
            },
            level,
        }
    }
}

impl<D> Draw<D> for Battery {
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Battery]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult
    where
        D: DrawTarget<Color = Rgb565>,
    {
        Rectangle::new(self.info.position, self.info.size - Size::new(2, 0))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .stroke_color(self.info.color_primary)
                    .stroke_width(1)
                    .fill_color(self.info.color_secondary)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        Rectangle::new(
            Point::new(
                self.info.size.width as i32 - 3,
                self.info.size_center().y / 2,
            ),
            Size::new(3, 4),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(self.info.color_primary)
                .stroke_width(1)
                .fill_color(self.info.color_secondary)
                .build(),
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        let level = self.level as i32 * (self.info.size.width as i32 - 2) / 100;
        Rectangle::new(
            Point::new(self.info.position.x + 1, self.info.position.y + 1),
            Size::new(level as u32, self.info.size.height - 2),
        )
        .into_styled(PrimitiveStyle::with_fill(self.info.color_primary))
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(Some(&[StateMarker::Battery]))
    }
}

pub struct Clock {
    pub(crate) info: GUIInfo,
    pub(crate) hour: u8,
    pub(crate) minute: u8,
}

impl Clock {
    pub fn new() -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(30, 10),
                position: Point::new(SCREEN_WIDTH as i32 - 48 - 1, 0),
                color_primary: Rgb565::WHITE,
                color_secondary: Rgb565::CSS_DARK_SLATE_GRAY,
            },
            hour: 12,
            minute: 30,
        }
    }
    pub fn set_time(&mut self, hour: u8, minute: u8) {
        self.hour = hour;
        self.minute = minute;
    }
}

impl<D> Draw<D> for Clock
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Clock]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        Rectangle::new(self.info.position, self.info.size)
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(self.info.color_secondary)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        let mut buf = [0u8; 6];
        let text = format_no_std::show(
            &mut buf,
            format_args!("{:02}:{:02}", self.hour, self.minute),
        )
        .map_err(|_| AppError::DataFormatError)?;
        Text::with_alignment(
            &text,
            self.info.size_center() + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, self.info.color_primary),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(Some(&[StateMarker::Clock]))
    }
}

#[derive(Default)]
pub struct PanelItem {
    pub(crate) info: GUIInfo,
    pub(crate) label: &'static str,
    pub(crate) text: &'static str,
    pub(crate) style: PanelStyle,
    pub(crate) panel: Panel,
}

impl PanelItem {
    pub fn new(panel: Panel, label: &'static str, text: &'static str, style: PanelStyle) -> Self {
        let index: usize = panel.into();
        Self {
            info: GUIInfo {
                size: Size::new(48 - 1, 26),
                position: Point::new(SCREEN_WIDTH as i32 - 48 + 1, 12 + (index % 8) as i32 * 27),
                color_primary: Rgb565::CSS_PURPLE,
                color_secondary: Rgb565::BLACK,
            },
            label,
            text,
            style,
            panel,
        }
    }
}

impl<D> Draw<D> for PanelItem
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::PanelPage]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let color_main = if self.style == PanelStyle::ChannelColor {
            state.channel_current.color()
        } else {
            self.info.color_primary
        };
        let size_half = Size::new(self.info.size.width, self.info.size.height / 2);
        let color_bg = if let Some(focused) = state.panel_focused {
            if focused == Panel::from(self.panel as usize) {
                Rgb565::WHITE
            } else {
                Rgb565::BLACK
            }
        } else {
            Rgb565::BLACK
        };
        Rectangle::new(
            self.info.position + Point::new(0, size_half.height as i32),
            size_half,
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(color_main)
                .stroke_width(1)
                .fill_color(color_bg)
                .build(),
        )
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Rectangle::new(self.info.position, size_half)
            .into_styled(PrimitiveStyleBuilder::new().fill_color(color_main).build())
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        let mut buf = [0u8; 2];
        let index: usize = self.panel.into();
        let index_disp = (index % 8) + 1;
        let text_index = format_no_std::show(&mut buf, format_args!("{}", index_disp))
            .map_err(|_| AppError::DataFormatError)?;
        let color_label = if self.style == PanelStyle::ChannelColor {
            Rgb565::BLACK
        } else {
            Rgb565::WHITE
        };
        let color_index = if self.style == PanelStyle::Normal {
            Rgb565::CYAN
        } else {
            Rgb565::CSS_DARK_RED
        };
        Text::with_alignment(
            text_index,
            Point::new(1, self.info.size.height as i32 * 1 / 4) + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, color_index),
            Alignment::Left,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        if index_disp == 8 {
            Text::with_alignment(
                "0",
                Point::new(
                    self.info.size.width as i32 - 7,
                    self.info.size.height as i32 * 1 / 4,
                ) + TEXT_OFFSET,
                MonoTextStyle::new(&FONT_6X9, color_index),
                Alignment::Left,
            )
            .translate(self.info.position)
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        }
        Text::with_alignment(
            self.label,
            Point::new(
                self.info.size.width as i32 / 2,
                self.info.size.height as i32 * 1 / 4,
            ) + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, color_label),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        let color_text = if let Some(focused) = state.panel_focused {
            if focused == Panel::from(self.panel as usize) {
                Rgb565::BLACK
            } else {
                Rgb565::WHITE
            }
        } else {
            Rgb565::WHITE
        };
        let mut _voltage_scale = Default::default();
        let text = match self.panel {
            Panel::Channel => state.channel_current.into(),
            Panel::VoltageScale => {
                let idx: usize = state.channel_current.into();
                _voltage_scale = VoltageScale::from_mv(state.channel_info[idx].voltage_scale_mv);
                _voltage_scale.str()
            }
            _ => self.text,
        };
        Text::with_alignment(
            text,
            Point::new(
                self.info.size.width as i32 / 2,
                self.info.size.height as i32 * 3 / 4,
            ) + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, color_text),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}

pub struct MeasureItem {
    pub(crate) info: GUIInfo,
    pub(crate) channel: ProbeChannel,
    pub(crate) label: &'static str,
    pub(crate) text: &'static str,
    pub(crate) enabled: bool,
}

impl MeasureItem {
    pub fn new(
        index: u8,
        channel: ProbeChannel,
        label: &'static str,
        text: &'static str,
        enabled: bool,
    ) -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(66, 10),
                position: Point::new(67 * index as i32 + 4, SCREEN_HEIGHT as i32 - 11),
                color_primary: Rgb565::YELLOW,
                color_secondary: Rgb565::GREEN,
            },
            channel,
            label,
            text,
            enabled,
        }
    }
}

impl<D> Draw<D> for MeasureItem
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Measures]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        let color_main = self.channel.color();
        Rectangle::new(self.info.position, self.info.size)
            .into_styled(PrimitiveStyle::with_fill(color_main))
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        if !self.enabled {
            Text::with_alignment(
                "--",
                self.info.size_center() + TEXT_OFFSET,
                MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
                Alignment::Center,
            )
            .translate(self.info.position)
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
            return Ok(None);
        }
        let mut buf = [0u8; 16];
        let text = format_no_std::show(&mut buf, format_args!("{}:{}", self.label, self.text))
            .map_err(|_| AppError::DataFormatError)?;
        Text::with_alignment(
            text,
            self.info.size_center() + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}

pub struct Generator(LineDisp<'static>);

impl Generator {
    pub fn new(text: &'static str) -> Self {
        Self(LineDisp {
            info: GUIInfo {
                size: Size::new(48 - 1, 10),
                position: Point::new(SCREEN_WIDTH as i32 - 48 + 1, SCREEN_HEIGHT as i32 - 11),
                color_primary: Rgb565::CSS_ORANGE_RED,
                color_secondary: Rgb565::WHITE,
            },
            text,
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
        })
    }
}

impl<D> Draw<D> for Generator
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Generator]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        self.0.draw_state(display, _state)?;
        Ok(Some(&[StateMarker::Generator]))
    }
}

pub struct SelectItem {
    pub(crate) info: GUIInfo,
    pub(crate) items: &'static [&'static [&'static str]],
}

impl<D> Draw<D> for SelectItem
where
    D: DrawTarget<Color = Rgb565>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::SettingValueContent]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let columns = self.items.len();
        let box_size = Size::new(self.info.size.width / columns as u32, self.info.size.height);
        for col in 0..columns {
            let box_pos = self.info.position + Point::new(box_size.width as i32 * col as i32, 0);
            let text_height = 11;
            for idx in 0..self.items[col].len() {
                let text = self.items[col][idx];
                // crate::info!(
                //     "{}:{} {}, selected {}:{}",
                //     col,
                //     idx,
                //     text,
                //     state.setting_select_col,
                //     state.setting_select_idx[col]
                // );
                let text_box_pos = box_pos + Point::new(1, (text_height * idx) as i32);
                let text_pos_center =
                    text_box_pos + Point::new(box_size.width as i32 / 2, (text_height / 2) as i32);
                let selected_col = idx == state.setting_select_idx[col] as usize;
                let selected_item = selected_col && col == state.setting_select_col as usize;
                let color_text = if selected_col {
                    if selected_item {
                        Rgb565::WHITE
                    } else {
                        self.info.color_primary
                    }
                } else {
                    Rgb565::BLACK
                };
                let color_bg = if selected_col {
                    Rgb565::BLACK
                } else {
                    self.info.color_primary
                };
                Rectangle::new(
                    text_box_pos,
                    Size::new(box_size.width - 2, text_height as u32 - 1),
                )
                .into_styled(PrimitiveStyle::with_fill(color_bg))
                .draw(display)
                .map_err(|_| AppError::DisplayError)?;
                Text::with_alignment(
                    text,
                    text_pos_center + TEXT_OFFSET,
                    MonoTextStyle::new(&FONT_6X9, color_text),
                    Alignment::Center,
                )
                .draw(display)
                .map_err(|_| AppError::DisplayError)?;
            }
        }
        Ok(Some(&[StateMarker::SettingValueContent]))
    }
}
