use defmt::*;
use std::borrow::Borrow;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};

mod app;

// type DisplayType = Mutex<ThreadModeRawMutex, Option<SimulatorDisplay>>;
// type DisplayType = app::DISPLAY;
// static DISPLAY: DisplayType = Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) -> () {
    env_logger::init();
    info!("Simulator started");

    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(320, 240));
    let line_style = PrimitiveStyle::with_stroke(Rgb565::GREEN, 1);
    let output_settings = OutputSettingsBuilder::new().scale(4).build();
    let mut window = Window::new("osc", &output_settings);

    Circle::new(Point::new(72, 8), 48)
        .into_styled(line_style)
        .draw(&mut display)
        .unwrap();

    // Line::new(Point::new(48, 16), Point::new(8, 16))
    //     .into_styled(line_style)
    //     .draw(&mut display)?;

    // Line::new(Point::new(48, 16), Point::new(64, 32))
    //     .into_styled(line_style)
    //     .draw(&mut display)?;

    // Rectangle::new(Point::new(79, 15), Size::new(34, 34))
    //     .into_styled(line_style)
    //     .draw(&mut display)?;

    // Text::new("Hello World!", Point::new(5, 5), text_style).draw(&mut display)?;

    // spawner.spawn(app::main_loop(display, window)).unwrap();
    app::main_loop(display, window).await;

    std::process::exit(0);
}
