use std::sync::{Arc, Mutex};

use app::input::{KeyboardDevice, Keys};
use defmt::*;

use embassy_executor::Spawner;
use embedded_graphics::prelude::*;
use embedded_graphics_simulator::{OutputSettingsBuilder, SimulatorDisplay, Window};

use crate::app::{input::DummyAdcDevice, GuiColor};

mod app;

struct SDLKeyboardDeriver {
    window: Arc<Mutex<Window>>,
}
impl KeyboardDevice for SDLKeyboardDeriver {
    fn read_key(&mut self) -> Keys {
        use embedded_graphics_simulator::SimulatorEvent;
        for event in self.window.lock().unwrap().events() {
            let k = match event {
                SimulatorEvent::Quit => {
                    info!("Simulator quitting");
                    std::process::exit(0);
                    // Keys::None
                }
                SimulatorEvent::KeyUp {
                    keycode,
                    keymod: _,
                    repeat,
                } => {
                    if !repeat {
                        let key = Keys::from(keycode);
                        info!("KeyUp: {:?} sdl code {:?}", key, keycode);
                        // app.input_key_event(key).unwrap();
                        key
                    } else {
                        Keys::None
                    }
                }
                _ => Keys::None,
            };
            if k != Keys::None {
                return k;
            }
        }
        Keys::None
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> () {
    env_logger::init();
    info!("Simulator started");

    let display = SimulatorDisplay::<GuiColor>::new(Size::new(320, 240));
    let output_settings = OutputSettingsBuilder::new().scale(3).build();
    let window = Window::new("osc simulator", &output_settings);
    let window = Arc::new(Mutex::new(window));
    let adc_device = DummyAdcDevice {};
    let kbd_device = SDLKeyboardDeriver {
        window: window.clone(),
    };

    app::main_loop(spawner, display.clone(), kbd_device, adc_device, |d| {
        window.lock().unwrap().update(d);
    })
    .await;
    info!("Simulator stopped");
    std::process::exit(0);
}
