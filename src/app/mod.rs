#![allow(dead_code)]

mod gui;
pub mod input;
mod unit;

use gui::*;
use input::*;

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum RunningState {
    #[default]
    Stopped,
    Running,
}
// use core::fmt::{self, Display, Formatter};
// impl Display for RunningState {
//     fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
//         match self {
//             RunningState::Running => write!(f, "RUN"),
//             RunningState::Stopped => write!(f, "STOP"),
//         }
//     }
// }
impl Into<&'static str> for RunningState {
    fn into(self) -> &'static str {
        match self {
            RunningState::Running => "RUN",
            RunningState::Stopped => "STOP",
        }
    }
}

#[derive(IntoPrimitive, Clone, Copy, PartialEq, PartialOrd)]
#[repr(usize)]
pub enum StateMarker {
    PanelPage,
    RunningState,
    Battery,
    Clock,
    Window,
    Waveform,
    TimeScale,
    ChannelSetting,
    Measures,
    Generator,
    Endding,
    All,
    AllFlush,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum Window {
    #[default]
    Main,
    SetValue,
    Settings,
}

#[derive(Debug)]
pub struct State {
    pub panel_page: u8,
    pub running_state: RunningState,
    pub battery: u8,
    pub clock: [char; 5],
    pub window: Window,
    // TODO: waveform data
    pub waveform: u8,
    pub time_scale_ns: u64,
    // TODO: channel setting
    pub channel_info: u64,
    // TODO: measures
    pub measures: u64,
    // TODO: generator setting
    pub generator: u64,
}

#[derive(Debug, Default)]
pub struct StateVec([bool; StateMarker::Endding as usize]);
impl StateVec {
    pub fn clear(&mut self) {
        for x in self.iter_mut() {
            *x = false;
        }
    }
    pub fn iter(&self) -> core::slice::Iter<bool> {
        self.0.iter()
    }
    pub fn iter_mut(&mut self) -> core::slice::IterMut<bool> {
        self.0.iter_mut()
    }
    pub fn at(&self, index: StateMarker) -> bool {
        let idx: usize = index.into();
        self.0[idx]
    }
    pub fn set(&mut self, index: StateMarker, value: bool) {
        let idx: usize = index.into();
        self.0[idx] = value;
    }
}

impl Default for State {
    fn default() -> Self {
        Self {
            panel_page: 0,
            running_state: Default::default(),
            battery: 50,
            clock: ['1', '2', ':', '4', '5'],
            window: Default::default(),
            waveform: Default::default(),
            time_scale_ns: 100_000,
            channel_info: 1250,
            measures: Default::default(),
            generator: Default::default(),
        }
    }
}

#[derive(Debug)]
pub enum AppError {
    DisplayError,
    DataFormatError,
}

pub type Result<T, E = AppError> = core::result::Result<T, E>;

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub enum Channel {
    A,
    B,
}

use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor, WebColors},
    primitives::{Primitive, PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
    Drawable,
};
use num_enum::IntoPrimitive;

pub struct App<D> {
    pub state: State,
    pub updated: StateVec,
    pub display: D,

    // widgets of main window
    waveform: Waveform,
    running_state: RunningStateDisp,
    time_scale: TimeScaleDisp,
    overview: Overview,
    channel_info1: ChannelSettingDisp,
    channel_info2: ChannelSettingDisp,
    battery: Battery,
    clock: Clock,
    panel_items: [PanelItem; 8 + 6],
    measure_items: [MeasureItem; 4],
    generator: Generator,
}

impl<D> App<D>
where
    D: DrawTarget<Color = Rgb565> + 'static,
{
    pub fn new(display: D) -> Self {
        Self {
            state: Default::default(),
            updated: Default::default(),
            display,
            waveform: Default::default(),
            running_state: Default::default(),
            time_scale: Default::default(),
            overview: Overview::new(),
            channel_info1: ChannelSettingDisp::new(GUIInfo {
                size: Size::new(33, 10),
                position: Point::new(204, 0),
                color_primary: Rgb565::YELLOW,
                color_secondary: Rgb565::BLACK,
            }),
            channel_info2: ChannelSettingDisp::new(GUIInfo {
                size: Size::new(33, 10),
                position: Point::new(205 + 33, 0),
                color_primary: Rgb565::GREEN,
                color_secondary: Rgb565::BLACK,
            }),
            battery: Battery::new(50),
            clock: Clock::new(),
            panel_items: [
                PanelItem::new(1, "Chann", "CHA", PanelStyle::ChannelColor),
                PanelItem::new(2, "T-Sca", "100ms", PanelStyle::Normal),
                PanelItem::new(3, "V-Sca", "20mV", PanelStyle::ChannelColor),
                PanelItem::new(4, "Xpos", "0.0ns", PanelStyle::Normal),
                PanelItem::new(5, "Ypos", "0.0ns", PanelStyle::ChannelColor),
                PanelItem::new(6, "T-thr", "-3.03mV", PanelStyle::Normal),
                PanelItem::new(7, "Coup", "DC", PanelStyle::ChannelColor),
                PanelItem::new(8, "T-Typ", "CHA-U", PanelStyle::Normal),
                PanelItem::new(1, "Probe", "X2", PanelStyle::ChannelColor),
                PanelItem::new(2, "H-Me1", "Freq", PanelStyle::ChannelColor),
                PanelItem::new(3, "V-Me1", "Vp-p", PanelStyle::ChannelColor),
                PanelItem::new(4, "H-Me2", "--", PanelStyle::ChannelColor),
                PanelItem::new(5, "V-Me2", "Vrms", PanelStyle::ChannelColor),
                PanelItem::new(6, "Sweep", "AUTO", PanelStyle::Normal),
            ],
            measure_items: [
                MeasureItem::new(0, Channel::A, "Freq", "34kHz", true),
                MeasureItem::new(1, Channel::A, "Vp-p", "2.3mV", false),
                MeasureItem::new(2, Channel::B, "Freq", "--", true),
                MeasureItem::new(3, Channel::B, "Vrms", "430uV", true),
            ],
            generator: Generator::new("Sin 10k"),
        }
    }

    async fn draw_main_window(&mut self) -> Result<()> {
        // if self.updated.iter().all(|&x| x) {
        //     return Ok(());
        // }
        if self.updated.iter().all(|&x| !x) {
            // clear screen at the first time
            self.display
                .clear(Rgb565::CSS_DARK_SLATE_GRAY)
                .map_err(|_| AppError::DisplayError)?;
        }

        self.running_state
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;
        self.time_scale
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;
        self.overview
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;

        if !self.updated.at(StateMarker::ChannelSetting) {
            self.channel_info1
                .draw(&mut self.display, &mut self.state, &mut self.updated)?;
            self.channel_info2
                .draw(&mut self.display, &mut self.state, &mut self.updated)?;
            self.updated.set(StateMarker::ChannelSetting, true);
        }

        self.battery
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;
        self.clock
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;
        self.waveform
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;

        if !self.updated.at(StateMarker::PanelPage) {
            Rectangle::new(
                self.panel_items[0].info.position,
                Size::new(
                    self.panel_items[0].info.size.width,
                    (self.panel_items[0].info.size.height + 1) * 8,
                ),
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_DARK_SLATE_GRAY))
            .draw(&mut self.display)
            .map_err(|_| AppError::DisplayError)?;
            let mut drawed_panel_items = 0;
            for item in self
                .panel_items
                .iter_mut()
                .skip(self.state.panel_page as usize * 8)
                .take(8)
            {
                item.draw(&mut self.display, &mut self.state, &mut self.updated)?;
                drawed_panel_items += 1;
            }
            if drawed_panel_items < 8 {
                // add info: 0 to switch page
                Text::with_alignment(
                    "0:Page",
                    Point::new(SCREEN_WIDTH as i32 - 24, SCREEN_HEIGHT as i32 - 11 * 3 + 5)
                        + TEXT_OFFSET,
                    MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
                    Alignment::Center,
                )
                .draw(&mut self.display)
                .map_err(|_| AppError::DisplayError)?;
            }
            self.updated.set(StateMarker::PanelPage, true);
        }
        if !self.updated.at(StateMarker::Measures) {
            for item in self.measure_items.iter_mut() {
                item.draw(&mut self.display, &mut self.state, &mut self.updated)?;
            }
            self.updated.set(StateMarker::Measures, true);
        }
        self.generator
            .draw(&mut self.display, &mut self.state, &mut self.updated)?;

        // self.updated[StateMarker::RunningState as usize] = false;
        // if self.state.running_state == RunningState::Running {
        //     self.state.running_state = RunningState::Stopped;
        // } else {
        //     self.state.running_state = RunningState::Running;
        // }

        Ok(())
    }

    async fn draw_set_value_window(&mut self) -> Result<()> {
        self.display
            .clear(Rgb565::GREEN)
            .map_err(|_| AppError::DisplayError)?;
        Ok(())
    }

    async fn draw_settings_window(&mut self) -> Result<()> {
        self.display
            .clear(Rgb565::CSS_ALICE_BLUE)
            .map_err(|_| AppError::DisplayError)?;
        Ok(())
    }

    pub async fn draw(&mut self) -> Result<()> {
        match self.state.window {
            Window::Main => self.draw_main_window().await,
            Window::SetValue => self.draw_set_value_window().await,
            Window::Settings => self.draw_settings_window().await,
        }
    }
}

impl<D> App<D> {
    fn clear_update_state(&mut self) {
        for x in self.updated.iter_mut() {
            *x = false;
        }
    }
    pub fn input_key_event(&mut self, key: Keys) -> Result<()> {
        match self.state.window {
            Window::Main => {
                let mut flush = false;
                match key {
                    Keys::Sharp => {
                        flush = true;
                    }
                    Keys::Key0 => {
                        self.state.panel_page = if self.state.panel_page >= 1 {
                            0
                        } else {
                            self.state.panel_page + 1
                        };
                        self.updated.set(StateMarker::PanelPage, false);
                    }
                    _ => {}
                }
                if flush {
                    self.updated.clear();
                }
                Ok(())
            }
            Window::SetValue => Ok(()),
            Window::Settings => Ok(()),
        }
    }
}

#[cfg(feature = "embedded")]
// #[embassy_executor::task]
pub async fn main_loop<D, K>(display: D, mut keyboard: K)
where
    D: DrawTarget<Color = Rgb565> + 'static,
    K: KeyboardDevice,
{
    let mut app = App::new(display);
    loop {
        use crate::info;
        // info!("Hello, world!");
        app.draw().await.unwrap();
        // Timer::after_millis(10000).await;
        let key = keyboard.read_key();
        if key != Keys::None {
            let s: &str = key.into();
            info!("Key: {:?}", s);
            app.input_key_event(key).unwrap();
        }
        Timer::after_millis(10).await;
    }
}

#[cfg(feature = "simulator")]
// #[embassy_executor::task]
pub async fn main_loop(
    display: embedded_graphics_simulator::SimulatorDisplay<Rgb565>,
    mut window: embedded_graphics_simulator::Window,
) {
    use defmt::*;
    let mut app = App::new(display);
    'running: loop {
        app.draw().await.unwrap();

        window.update(&app.display);
        use embedded_graphics_simulator::SimulatorEvent;
        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                SimulatorEvent::KeyUp {
                    keycode,
                    keymod: _,
                    repeat,
                } => {
                    if !repeat {
                        let key = Keys::from(keycode);
                        info!("KeyUp: {:?} sdl code {:?}", key, keycode);
                        app.input_key_event(key).unwrap();
                    }
                }
                _ => {}
            }
        }
        Timer::after_millis(20).await;
    }
}
