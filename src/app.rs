#![allow(dead_code)]

use crate::info;
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

#[derive(Default, Debug)]
struct State {}

const TEXT_OFFSET: Point = Point::new(0, 2);

#[derive(Debug)]
pub enum AppError {
    DisplayError,
}

type Result<T, E = AppError> = core::result::Result<T, E>;

pub trait Draw<DISPLAY> {
    fn draw(&self, display: &mut DISPLAY) -> Result<()>;
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
                size: Size::new(320 - 48 - 4, 240 - 12 - 12),
                position: Point::new(4, 12),
                color_primary: Rgb565::CSS_DARK_SLATE_GRAY,
                color_secondary: Rgb565::CSS_LIGHT_GRAY,
                ..Default::default()
            },
        }
    }
}

impl<DISPLAY> Draw<DISPLAY> for Waveform
where
    DISPLAY: DrawTarget<Color = Rgb565>,
{
    fn draw(&self, display: &mut DISPLAY) -> Result<()> {
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

        // let style2 = PrimitiveStyleBuilder::new()
        //     .stroke_color(self.info.color_primary)
        //     .stroke_width(1)
        //     .build();

        let dl = 2;
        // Draw point grid
        for i in 0..(self.info.size.height as i32 / 40 + 1) {
            for j in 0..(self.info.size.width as i32 / 8) {
                let x = j * 8 + 6;
                let y = i * 40 + 28;
                if y == center.y as i32 {
                    Line::new(Point::new(x, y - dl), Point::new(x, y + dl))
                        .translate(self.info.position)
                        .draw_styled(&style, display)
                        .map_err(|_| AppError::DisplayError)?;
                } else {
                    let p = Point::new(x, y) + self.info.position;
                    Pixel(p, self.info.color_primary)
                        .draw(&mut *display)
                        .map_err(|_| AppError::DisplayError)?;
                }
            }
        }
        for i in 0..(self.info.size.width as i32 / 40 + 1) {
            for j in 0..(self.info.size.height as i32 / 8) {
                let x = i * 40 + 14;
                let y = j * 8 + 4;
                if x == center.x as i32 {
                    Line::new(Point::new(x - dl, y), Point::new(x + dl, y))
                        .translate(self.info.position)
                        .draw_styled(&style, display)
                        .map_err(|_| AppError::DisplayError)?;
                } else {
                    let p = Point::new(x, y) + self.info.position;
                    Pixel(p, Rgb565::CSS_DARK_SLATE_GRAY)
                        .draw(&mut *display)
                        .map_err(|_| AppError::DisplayError)?;
                }
            }
        }

        Ok(())
    }
}

pub struct LineDisp<'a> {
    pub(crate) info: GUIInfo,
    text: &'a str,
    font: MonoTextStyle<'static, Rgb565>,
}

impl<DISPLAY> Draw<DISPLAY> for LineDisp<'_>
where
    DISPLAY: DrawTarget<Color = Rgb565>,
{
    fn draw(&self, display: &mut DISPLAY) -> Result<()> {
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
        .draw(&mut *display)
        .map_err(|_| AppError::DisplayError)?;
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
                size: Size::new(24 * 3, 10),
                position: Point::new(84, 0),
                color_primary: Rgb565::WHITE,
                color_secondary: Rgb565::BLACK,
            },
            text: "Roll Mode",
        }
    }
}
impl<DISPLAY> Draw<DISPLAY> for Overview
where
    DISPLAY: DrawTarget<Color = Rgb565>,
{
    fn draw(&self, display: &mut DISPLAY) -> Result<()> {
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
        .draw(&mut *display)
        .map_err(|_| AppError::DisplayError);
        Ok(())
    }
}

pub struct App<DISPLAY> {
    pub(crate) state: State,
    pub display: DISPLAY,
    waveform: Waveform,
    run_stop: LineDisp<'static>,
    time_scale: LineDisp<'static>,
    overview: Overview,
}

impl<DISPLAY> App<DISPLAY>
where
    DISPLAY: DrawTarget<Color = Rgb565> + 'static,
{
    pub fn new(display: DISPLAY) -> Self {
        Self {
            state: Default::default(),
            display,
            waveform: Default::default(),
            run_stop: LineDisp {
                info: GUIInfo {
                    size: Size::new(29, 10),
                    position: Point::new(4, 0),
                    color_primary: Rgb565::RED,
                    color_secondary: Rgb565::WHITE,
                },
                text: "STOP",
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
            },
            time_scale: LineDisp {
                info: GUIInfo {
                    size: Size::new(35, 10),
                    position: Point::new(4 + 29 + 1, 0),
                    color_primary: Rgb565::MAGENTA,
                    color_secondary: Rgb565::WHITE,
                },
                text: "H 2us",
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
            },
            overview: Overview::new(),
        }
    }

    pub async fn draw(&mut self) -> Result<()> {
        // Create a new character style
        // let style = MonoTextStyle::new(&FONT_4X6, Rgb565::RED);

        self.display
            .clear(Rgb565::CSS_DARK_SLATE_GRAY)
            .map_err(|_| AppError::DisplayError)?;

        self.waveform.draw(&mut self.display)?;

        self.run_stop.draw(&mut self.display)?;
        self.time_scale.draw(&mut self.display)?;

        self.overview.draw(&mut self.display)?;

        // Create a text at position (20, 30) and draw it using the previously defined style
        // Text::with_alignment(
        //     "First line\nSecond line",
        //     Point::new(20, 30),
        //     style,
        //     Alignment::Left,
        // )
        // .draw(&mut self.display)
        // .map_err(|_| AppError::DisplayError)?;

        Ok(())
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task]
pub async fn main_loop(display: impl DrawTarget<Color = Rgb565> + 'static) {
    let mut app = App::new(display);
    loop {
        info!("Hello, world!");
        app.draw().await.unwrap();
        Timer::after_millis(10000).await;
    }
}

#[cfg(feature = "simulator")]
// #[embassy_executor::task]
pub async fn main_loop(
    display: embedded_graphics_simulator::SimulatorDisplay<Rgb565>,
    mut window: embedded_graphics_simulator::Window,
) {
    let mut app = App::new(display);
    'running: loop {
        app.draw().await.unwrap();

        window.update(&app.display);
        for event in window.events() {
            if event == embedded_graphics_simulator::SimulatorEvent::Quit {
                break 'running;
            }
        }
        Timer::after_millis(1).await;
    }
}
