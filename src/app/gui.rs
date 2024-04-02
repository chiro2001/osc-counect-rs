use crate::app::{AppError, Channel, Result};
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

use super::{State, StateMarker};

pub const TEXT_OFFSET: Point = Point::new(0, 2);
pub const SCREEN_WIDTH: u32 = 320;
pub const SCREEN_HEIGHT: u32 = 240;

pub trait Draw<D> {
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()>
    where
        D: DrawTarget<Color = Rgb565>;
}

#[derive(Debug, Default)]
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
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()> {
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

        Ok(())
    }
}

pub struct LineDisp<'a> {
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
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()> {
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
        Ok(())
    }
}

pub struct RunningStateDisp(pub LineDisp<'static>);

impl RunningStateDisp {
    pub fn new() -> Self {
        Self(LineDisp {
            info: GUIInfo {
                size: Size::new(29, 10),
                position: Point::new(4, 0),
                color_primary: Rgb565::RED,
                color_secondary: Rgb565::WHITE,
            },
            font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
            ..Default::default()
        })
    }
}
impl Default for RunningStateDisp {
    fn default() -> Self {
        Self::new()
    }
}
impl<D> Draw<D> for RunningStateDisp
where
    D: DrawTarget<Color = Rgb565>,
{
    fn draw(&mut self, display: &mut D, state: &mut State) -> Result<()> {
        let idx_updated: usize = StateMarker::RunningState.into();
        if state.updated[idx_updated] {
            return Ok(());
        }
        self.0.text = state.running_state.into();
        self.0.draw(display, state)?;
        state.updated[idx_updated] = true;
        Ok(())
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
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()> {
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
        Ok(())
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
    fn draw(&mut self, display: &mut D, state: &mut State) -> Result<()>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let idx_updated: usize = StateMarker::Battery.into();
        if state.updated[idx_updated] {
            return Ok(());
        }
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
        state.updated[idx_updated] = true;
        Ok(())
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
    fn draw(&mut self, display: &mut D, state: &mut State) -> Result<()> {
        let idx_updated: usize = StateMarker::Clock.into();
        if state.updated[idx_updated] {
            return Ok(());
        }
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
        state.updated[idx_updated] = true;
        Ok(())
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
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()> {
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
        Ok(())
    }
}

pub struct MeasureItem {
    pub(crate) info: GUIInfo,
    pub(crate) channel: Channel,
    pub(crate) label: &'static str,
    pub(crate) text: &'static str,
    pub(crate) enabled: bool,
}

impl MeasureItem {
    pub fn new(
        index: u8,
        channel: Channel,
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
    fn draw(&mut self, display: &mut D, _state: &mut State) -> Result<()> {
        let color_main = if self.channel == Channel::A {
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
            return Ok(());
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
        Ok(())
    }
}

pub struct Generator(pub LineDisp<'static>);

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
