#![allow(static_mut_refs)]

use super::{
    unit::{TimeScale, VoltageScale},
    Panel, PanelStyle, RunningState, State, StateMarker, StateVec,
};
use crate::app::{AppError, ProbeChannel, Result};
use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::*, MonoTextStyle},
    primitives::*,
    text::{Alignment, Text},
    transform::Transform,
    Drawable,
};

mod color;
mod menu;
mod overview;
mod waveform;
pub use color::*;
pub use menu::*;
pub use overview::*;
pub use waveform::*;

pub const TEXT_OFFSET: Point = Point::new(0, 2);
#[cfg(feature = "screen-320x240")]
mod screen_size {
    pub const SCREEN_WIDTH: u32 = 320;
    pub const SCREEN_HEIGHT: u32 = 240;
}
#[cfg(feature = "screen-160x80")]
mod screen_size {
    pub const SCREEN_WIDTH: u32 = 160;
    pub const SCREEN_HEIGHT: u32 = 80;
}
#[cfg(feature = "screen-256x128")]
mod screen_size {
    pub const SCREEN_WIDTH: u32 = 256;
    pub const SCREEN_HEIGHT: u32 = 128;
}
pub use screen_size::*;

type StateResult = Result<Option<&'static [StateMarker]>>;
pub trait Draw<D> {
    fn enabled(&self) -> bool {
        true
    }
    async fn draw(&self, display: &mut D, state: &mut State, vec: &mut StateVec) -> Result<()>
    where
        D: DrawTarget<Color = GuiColor>,
    {
        if !self.enabled() {
            return Ok(());
        }
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
        D: DrawTarget<Color = GuiColor>,
    {
        Err(AppError::NotImplemented)
    }

    fn draw_state_vec(&self, _display: &mut D, _state: &mut State, _vec: &StateVec) -> StateResult
    where
        D: DrawTarget<Color = GuiColor>,
    {
        Err(AppError::NotImplemented)
    }
}

#[derive(Debug, Default, Clone)]
pub struct GUIInfo {
    pub(crate) size: Size,
    pub(crate) position: Point,
    pub(crate) color_primary: GuiColor,
    pub(crate) color_secondary: GuiColor,
    pub(crate) disabled: bool,
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
    pub fn disable(&mut self) {
        self.disabled = true;
    }
    pub fn enabled(&self) -> bool {
        !self.disabled
    }
}

#[derive(Clone)]
struct LineDisp<'a> {
    pub(crate) info: GUIInfo,
    pub(crate) text: &'a str,
    pub(crate) font: MonoTextStyle<'static, GuiColor>,
}
impl<'a> Default for LineDisp<'a> {
    fn default() -> Self {
        Self {
            info: Default::default(),
            text: Default::default(),
            font: MonoTextStyle::new(&FONT_6X9, gui_color(15)),
        }
    }
}

impl<D> Draw<D> for LineDisp<'_>
where
    D: DrawTarget<Color = GuiColor>,
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
    fn new_disp<'a>(text: &'a str, color: GuiColor) -> LineDisp<'a> {
        LineDisp {
            info: GUIInfo {
                size: Size::new(29, 10),
                position: Point::new(4, 0),
                color_primary: color,
                color_secondary: gui_color(15),
                ..Default::default()
            },
            font: MonoTextStyle::new(&FONT_6X9, gui_color(15)),
            text,
            ..Default::default()
        }
    }
}
impl<D> Draw<D> for RunningStateDisp
where
    D: DrawTarget<Color = GuiColor>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::RunningState]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let text: &str = state.running_state.into();
        let disp = Self::new_disp(
            text,
            if state.running_state == RunningState::Running {
                gui_color(8)
            } else {
                gui_color(4)
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
                color_primary: gui_color(5),
                color_secondary: gui_color(15),
                ..Default::default()
            },
            text,
            font: MonoTextStyle::new(&FONT_6X9, gui_color(15)),
        }
    }
}
impl<D> Draw<D> for TimeScaleDisp
where
    D: DrawTarget<Color = GuiColor>,
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
                font: MonoTextStyle::new(&FONT_6X9, gui_color(0)),
                ..Default::default()
            },
        )
    }
}
impl<D> Draw<D> for ChannelSettingDisp
where
    D: DrawTarget<Color = GuiColor>,
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
                color_primary: gui_color(15),
                color_secondary: gui_color(0),
                ..Default::default()
            },
            level,
        }
    }
    pub fn set_level(&mut self, level: u8) {
        self.level = level;
    }
}

impl<D> Draw<D> for Battery {
    fn enabled(&self) -> bool {
        self.info.enabled()
    }
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Battery]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult
    where
        D: DrawTarget<Color = GuiColor>,
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
                color_primary: gui_color(15),
                color_secondary: gui_color(1),
                ..Default::default()
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
    D: DrawTarget<Color = GuiColor>,
{
    fn enabled(&self) -> bool {
        self.info.enabled()
    }
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
    pub(crate) show_index: bool,
    pub(crate) per_page: usize,
}

impl PanelItem {
    pub fn new(
        panel: Panel,
        label: &'static str,
        text: &'static str,
        style: PanelStyle,
        show_index: bool,
        per_page: usize,
    ) -> Self {
        let index: usize = panel.into();
        Self {
            info: GUIInfo {
                size: Size::new(48 - 1, 26),
                position: Point::new(
                    SCREEN_WIDTH as i32 - 48 + 1,
                    12 + (index % per_page) as i32 * 27,
                ),
                color_primary: gui_color(8),
                color_secondary: gui_color(0),
                ..Default::default()
            },
            label,
            text,
            style,
            panel,
            show_index,
            per_page,
        }
    }
}

pub const DRAW_BOARDER_ONLY: bool = cfg!(feature = "color-binary");
impl<D> Draw<D> for PanelItem
where
    D: DrawTarget<Color = GuiColor>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::PanelPage]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let color_main = if self.style == PanelStyle::ChannelColor {
            state.channel_current.color()
        } else {
            if DRAW_BOARDER_ONLY {
                gui_color(15)
            } else {
                self.info.color_primary
            }
        };
        let size_half = Size::new(self.info.size.width, self.info.size.height / 2);
        let color_bg = if let Some(focused) = state.panel_focused {
            if focused == Panel::from(self.panel as usize) {
                gui_color(15)
            } else {
                gui_color(0)
            }
        } else {
            gui_color(0)
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
        if !DRAW_BOARDER_ONLY || self.style == PanelStyle::ChannelColor {
            Rectangle::new(self.info.position, size_half)
                .into_styled(PrimitiveStyleBuilder::new().fill_color(color_main).build())
                .draw(display)
                .map_err(|_| AppError::DisplayError)?;
        } else if DRAW_BOARDER_ONLY {
            Rectangle::new(self.info.position, size_half)
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .stroke_color(color_main)
                        .stroke_width(1)
                        .build(),
                )
                .draw(display)
                .map_err(|_| AppError::DisplayError)?;
        }
        let mut buf = [0u8; 2];
        let index: usize = self.panel.into();
        let index_disp = (index % 8) + 1;
        let color_label = if self.style == PanelStyle::ChannelColor {
            gui_color(0)
        } else {
            gui_color(15)
        };
        if self.show_index {
            let text_index = format_no_std::show(&mut buf, format_args!("{}", index_disp))
                .map_err(|_| AppError::DataFormatError)?;
            let color_index = if self.style == PanelStyle::Normal {
                gui_color(6)
            } else {
                gui_color(10)
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
            if index_disp == self.per_page {
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
                gui_color(0)
            } else {
                gui_color(15)
            }
        } else {
            gui_color(15)
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
    pub(crate) flexible: bool,
}

impl MeasureItem {
    pub fn new(
        index: u8,
        channel: ProbeChannel,
        label: &'static str,
        text: &'static str,
        enabled: bool,
        flexible: bool,
    ) -> Self {
        let short = label.is_empty() && flexible;
        Self {
            info: GUIInfo {
                size: Size::new(if short { 32 } else { 66 }, 10),
                position: Point::new(67 * index as i32 + 4, SCREEN_HEIGHT as i32 - 11),
                color_primary: gui_color(2),
                color_secondary: gui_color(3),
                ..Default::default()
            },
            channel,
            label,
            text,
            enabled,
            flexible,
        }
    }
}

impl<D> Draw<D> for MeasureItem
where
    D: DrawTarget<Color = GuiColor>,
{
    fn enabled(&self) -> bool {
        self.info.enabled()
    }
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
                MonoTextStyle::new(&FONT_6X9, gui_color(0)),
                Alignment::Center,
            )
            .translate(self.info.position)
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
            return Ok(None);
        }
        let mut buf = [0u8; 16];
        let text = if self.label.is_empty() {
            self.text
        } else {
            format_no_std::show(&mut buf, format_args!("{}:{}", self.label, self.text))
                .map_err(|_| AppError::DataFormatError)?
        };
        Text::with_alignment(
            text,
            self.info.size_center() + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, gui_color(0)),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}

pub struct Generator {
    pub(crate) info: GUIInfo,
    pub(crate) text: &'static str,
}

impl Generator {
    pub fn new(text: &'static str) -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(48 - 1, 10),
                position: Point::new(SCREEN_WIDTH as i32 - 48 + 1, SCREEN_HEIGHT as i32 - 11),
                color_primary: gui_color(9),
                color_secondary: gui_color(15),
                ..Default::default()
            },
            text,
        }
    }
}

impl<D> Draw<D> for Generator
where
    D: DrawTarget<Color = GuiColor>,
{
    fn enabled(&self) -> bool {
        self.info.enabled()
    }
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Generator]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        let disp = LineDisp {
            info: self.info.clone(),
            text: self.text,
            font: MonoTextStyle::new(&FONT_6X9, gui_color(15)),
        };
        disp.draw_state(display, _state)?;
        Ok(Some(&[StateMarker::Generator]))
    }
}

pub struct SelectItem {
    pub(crate) info: GUIInfo,
    pub(crate) items: &'static [&'static [&'static str]],
}

impl<D> Draw<D> for SelectItem
where
    D: DrawTarget<Color = GuiColor>,
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
                        gui_color(15)
                    } else {
                        self.info.color_primary
                    }
                } else {
                    gui_color(0)
                };
                let color_bg = if selected_col {
                    gui_color(0)
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

#[derive(Default, Debug)]
pub struct TriggerLevelDisp;
impl<D> Draw<D> for TriggerLevelDisp
where
    D: DrawTarget<Color = GuiColor>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::TriggerLevel]
    }
    fn draw_state(&self, display: &mut D, state: &mut State) -> StateResult {
        let center = Point::new(3, 12 + WF_WIDTH_HEIGHT as i32 / 2);
        Rectangle::with_center(center, Size::new(6, WF_WIDTH_HEIGHT))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(GUI_BG_COLOR)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        let trigger_level_pixel_offset =
            -((state.trigger_level_mv as i32) * (WF_WIDTH_HEIGHT as i32) / 2 / 3000);
        let center = center + Point::new(0, trigger_level_pixel_offset);
        Rectangle::with_center(center, Size::new(5, 8))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(state.trigger_channel.color())
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        Text::with_alignment(
            "T",
            center + Point::new(0, 2),
            MonoTextStyle::new(&FONT_4X6, gui_color(0)),
            Alignment::Center,
        )
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(Some(&[StateMarker::TriggerLevel]))
    }
}
