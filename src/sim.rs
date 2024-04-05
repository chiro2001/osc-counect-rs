use defmt::*;

use embassy_executor::Spawner;
use embedded_graphics::
    prelude::*
;
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, Window,
};

use crate::app::GuiColor;

mod app;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> () {
    env_logger::init();
    info!("Simulator started");

    let display = SimulatorDisplay::<GuiColor>::new(Size::new(320, 240));
    let output_settings = OutputSettingsBuilder::new().scale(3).build();
    let window = Window::new("osc simulator", &output_settings);

    app::main_loop(display, window).await;

    info!("Simulator stopped");
    std::process::exit(0);
}
