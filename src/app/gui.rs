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
    RunningState, State, StateMarker, StateVec,
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
        let updated = self.draw_state(display, state)?;
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
                        vec.set(*x, true);
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

    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult
    where
        D: DrawTarget<Color = Rgb565>;
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
        &[StateMarker::Waveform]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(self.info.color_secondary)
            .stroke_width(1)
            .fill_color(Rgb565::BLACK)
            .build();
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

        Ok(Some(&[StateMarker::Waveform]))
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

pub struct ChannelSettingDisp(LineDisp<'static>);

impl ChannelSettingDisp {
    pub fn new(info: GUIInfo) -> Self {
        Self(LineDisp {
            info,
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
            ..Default::default()
        })
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
        let mut voltage_scale = VoltageScale::from_mv(state.channel_info);
        let text = voltage_scale.str();
        let disp = LineDisp {
            text,
            ..self.0.clone()
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelStyle {
    Normal,
    ChannelColor,
}

pub struct PanelItem {
    pub(crate) info: GUIInfo,
    pub(crate) label: &'static str,
    pub(crate) text: &'static str,
    pub(crate) style: PanelStyle,
    pub(crate) index: u8,
}

impl PanelItem {
    pub fn new(index: u8, label: &'static str, text: &'static str, style: PanelStyle) -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(48 - 1, 26),
                position: Point::new(SCREEN_WIDTH as i32 - 48 + 1, 12 + (index - 1) as i32 * 27),
                color_primary: Rgb565::CSS_PURPLE,
                color_secondary: Rgb565::BLACK,
            },
            label,
            text,
            style,
            index,
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
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        let color_main = if self.style == PanelStyle::ChannelColor {
            Rgb565::YELLOW
        } else {
            self.info.color_primary
        };
        let size_half = Size::new(self.info.size.width, self.info.size.height / 2);
        Rectangle::new(
            self.info.position + Point::new(0, size_half.height as i32),
            size_half,
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(color_main)
                .stroke_width(1)
                .fill_color(self.info.color_secondary)
                .build(),
        )
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Rectangle::new(self.info.position, size_half)
            .into_styled(PrimitiveStyleBuilder::new().fill_color(color_main).build())
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        let mut buf = [0u8; 2];
        let text_index = format_no_std::show(&mut buf, format_args!("{}", self.index))
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
        if self.index == 8 {
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
        Text::with_alignment(
            self.text,
            Point::new(
                self.info.size.width as i32 / 2,
                self.info.size.height as i32 * 3 / 4,
            ) + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
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
        let color_main = if self.channel == ProbeChannel::A {
            self.info.color_primary
        } else {
            self.info.color_secondary
        };
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
