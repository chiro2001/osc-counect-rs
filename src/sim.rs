use std::borrow::Borrow;
use defmt::*;

use embassy_executor::Spawner;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) -> () {
    env_logger::init();

    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(320, 240));
    // let line_style = PrimitiveStyle::with_stroke(Rgb565::GREEN, 1);
    // let text_style = MonoTextStyle::new(&FONT_6X9, Rgb565::RED);
    let output_settings = OutputSettingsBuilder::new().scale(4).build();
    let mut window = Window::new("osc", &output_settings);

    // Circle::new(Point::new(72, 8), 48)
    //     .into_styled(line_style)
    //     .draw(&mut display)?;

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

    // spawner.spawn(app::main_loop(display)).unwrap();

    info!("loop start");
    'running: loop {
        window.update(&display);
        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }
    }
    info!("exit");
}
