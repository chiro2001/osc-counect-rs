use defmt::*;

use embassy_executor::Spawner;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
};
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, Window,
};

mod app;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> () {
    env_logger::init();
    info!("Simulator started");

    let display = SimulatorDisplay::<Rgb565>::new(Size::new(320, 240));
    let output_settings = OutputSettingsBuilder::new().scale(4).build();
    let window = Window::new("osc", &output_settings);

    app::main_loop(display, window).await;

    info!("Simulator stopped");
    std::process::exit(0);
}
