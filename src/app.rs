use defmt::*;
use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor, WebColors},
    primitives::{Line, PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    text::{Alignment, Text},
    transform::Transform,
    Drawable, Pixel,
};

#[derive(Default, Debug)]
struct State {}

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
    pub(crate) rect: Rectangle,
    pub(crate) color_primary: Rgb565,
    pub(crate) color_secondary: Rgb565,
    pub(crate) text: &'static str,
    pub(crate) translate: Point,
}

#[derive(Debug)]
pub struct Waveform {
    pub(crate) info: GUIInfo,
}

impl Default for Waveform {
    fn default() -> Self {
        Self {
            info: GUIInfo {
                rect: Rectangle::new(Default::default(), Size::new(320 - 48 - 4, 240 - 12 - 12)),
                translate: Point::new(4, 12),
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
            .stroke_color(Rgb565::CSS_LIGHT_GRAY)
            .stroke_width(1)
            .fill_color(Rgb565::BLACK)
            .build();
        self.info
            .rect
            .translate(self.info.translate)
            .draw_styled(&style, display)
            .map_err(|_| AppError::DisplayError)?;
        let center = self.info.rect.center();
        let bottom_right = self.info.rect.bottom_right().unwrap();
        Line::new(
            Point::new(center.x, 0),
            Point::new(center.x, bottom_right.y),
        )
        .translate(self.info.translate)
        .draw_styled(&style, display)
        .map_err(|_| AppError::DisplayError)?;
        Line::new(
            Point::new(0, center.y),
            Point::new(bottom_right.x, center.y),
        )
        .translate(self.info.translate)
        .draw_styled(&style, display)
        .map_err(|_| AppError::DisplayError)?;

        let style2 = PrimitiveStyleBuilder::new()
            .stroke_color(Rgb565::CSS_DARK_SLATE_GRAY)
            .stroke_width(1)
            .fill_color(Rgb565::BLACK)
            .build();

        // Draw point grid
        for i in 0..(self.info.rect.size.height as i32 / 40 + 1) {
            for j in 0..(self.info.rect.size.width as i32 / 8 + 1) {
                let p = Point::new(j * 8 + 5, i * 40 + 27) + self.info.translate;
                Pixel(p, Rgb565::CSS_DARK_SLATE_GRAY)
                    .draw(&mut *display)
                    .map_err(|_| AppError::DisplayError)?;
            }
        }
        for i in 0..(self.info.rect.size.width as i32 / 40 + 1) {
            for j in 0..(self.info.rect.size.height as i32 / 8) {
                let p = Point::new(i * 40 + 13, j * 8 + 11) + self.info.translate;
                Pixel(p, Rgb565::CSS_DARK_SLATE_GRAY)
                    .draw(&mut *display)
                    .map_err(|_| AppError::DisplayError)?;
            }
        }

        Ok(())
    }
}

pub struct App<DISPLAY> {
    pub(crate) state: State,
    display: DISPLAY,
    waveform: Waveform,
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
        }
    }

    pub async fn draw(&mut self) -> Result<()> {
        // Create a new character style
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

        self.display
            .clear(Rgb565::CSS_DARK_SLATE_GRAY)
            .map_err(|_| AppError::DisplayError)?;

        self.waveform.draw(&mut self.display)?;

        // Create a text at position (20, 30) and draw it using the previously defined style
        Text::with_alignment(
            "First line\nSecond line",
            Point::new(20, 30),
            style,
            Alignment::Left,
        )
        .draw(&mut self.display)
        .map_err(|_| AppError::DisplayError)?;

        Ok(())
    }
}

#[embassy_executor::task]
pub async fn main_loop(display: impl DrawTarget<Color = Rgb565> + 'static) {
    let mut app = App::new(display);
    loop {
        info!("Hello, world!");
        app.draw().await.unwrap();
        Timer::after_millis(10000).await;
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use embedded_graphics_simulator::{
//         BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, Window,
//     };

//     #[test]
//     fn test_draw() {
//         let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(320, 240));
//     }
// }
