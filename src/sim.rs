#![feature(type_alias_impl_trait)]

use std::sync::{mpsc::Receiver, Arc, Mutex};

use app::devices::{AdcDevice, AdcReadOptions, KeyboardDevice, Keys};
use defmt::*;

use embassy_executor::Spawner;
use embedded_graphics::prelude::*;
use embedded_graphics_simulator::{OutputSettingsBuilder, SimulatorDisplay, Window};

use crate::app::{devices::{DummyBoardDevice, DummyBuzzerDevice}, GuiColor};

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
    // let adc_device = DummyAdcDevice {};
    let adc_device = AudioAdcDevice::new();
    let kbd_device = SDLKeyboardDeriver {
        window: window.clone(),
    };

    let board = DummyBoardDevice {};
    let buzzer = DummyBuzzerDevice {};
    app::main_loop(
        spawner,
        display.clone(),
        board,
        buzzer,
        kbd_device,
        adc_device,
        |d| {
            window.lock().unwrap().update(d);
        },
    )
    .await;
    info!("Simulator stopped");
    std::process::exit(0);
}

struct AudioAdcDevice {
    _stream: cpal::Stream,
    receiver: Receiver<f32>,
}
impl AudioAdcDevice {
    fn new() -> Self {
        use cpal::traits::DeviceTrait;
        use cpal::traits::HostTrait;
        let host = cpal::default_host();
        use std::sync::mpsc::channel;
        let (sender, receiver) = channel();
        let device = host
            .default_input_device()
            .expect("no input device available");
        device
            .supported_input_configs()
            .unwrap()
            .for_each(|config| {
                info!("supported input config: {:?}", config);
            });
        let stream = device
            .build_input_stream(
                &cpal::StreamConfig {
                    channels: 1,
                    sample_rate: cpal::SampleRate(44100),
                    buffer_size: cpal::BufferSize::Fixed(1024 * 64),
                },
                move |data: &[f32], _: &_| {
                    // let average: f32 = data.iter().sum::<f32>() / data.len() as f32;
                    // info!("average: {}", average);
                    for sample in data.iter() {
                        // sender.send(*sample * 2.0 / average).unwrap();
                        sender.send(*sample * 2.0).unwrap();
                    }
                },
                move |err| {
                    // An error occurred on stream
                    error!("an error occurred on stream: {}", err);
                },
                None,
            )
            .unwrap();
        Self {
            _stream: stream,
            receiver,
        }
    }
}

impl AdcDevice for AudioAdcDevice {
    async fn read(&mut self, _options: AdcReadOptions, buf: &mut [f32]) -> app::Result<usize> {
        let mut i = 0;
        while i < buf.len() {
            match self.receiver.recv() {
                Ok(sample) => {
                    buf[i] = sample;
                    i += 1;
                }
                Err(_) => {
                    warn!("no data! return what we have now: {}", i);
                    return Ok(i);
                }
            }
        }
        Ok(i)
    }
}
