#![allow(dead_code)]

mod gui;

use gui::*;

#[derive(Debug, Default, Clone, Copy)]
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

#[derive(IntoPrimitive)]
#[repr(usize)]
pub enum StateMarker {
    PanelPage,
    RunningState,
    Battery,
    Clock,
    Endding,
}

#[derive(Debug)]
pub struct State {
    pub panel_page: u8,
    pub running_state: RunningState,
    pub battery: u8,
    pub clock: [char; 5],
    pub updated: [bool; StateMarker::Endding as usize],
}

impl Default for State {
    fn default() -> Self {
        Self {
            panel_page: 0,
            running_state: Default::default(),
            battery: 50,
            clock: ['1', '2', ':', '4', '5'],
            updated: [false; StateMarker::Endding as usize],
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
    text::{Alignment, Text},
    Drawable,
};
use num_enum::IntoPrimitive;

pub struct App<D> {
    pub state: State,
    pub display: D,
    waveform: Waveform,
    running_state: RunningStateDisp,
    time_scale: LineDisp<'static>,
    overview: Overview,
    channel_info1: LineDisp<'static>,
    channel_info2: LineDisp<'static>,
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
            display,
            waveform: Default::default(),
            running_state: Default::default(),
            time_scale: LineDisp {
                info: GUIInfo {
                    size: Size::new(35, 10),
                    position: Point::new(4 + 29 + 1, 0),
                    color_primary: Rgb565::MAGENTA,
                    color_secondary: Rgb565::WHITE,
                },
                text: "500ms",
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE),
            },
            overview: Overview::new(),
            channel_info1: LineDisp {
                info: GUIInfo {
                    size: Size::new(33, 10),
                    position: Point::new(204, 0),
                    color_primary: Rgb565::YELLOW,
                    color_secondary: Rgb565::BLACK,
                },
                text: "20mV",
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
            },
            channel_info2: LineDisp {
                info: GUIInfo {
                    size: Size::new(33, 10),
                    position: Point::new(205 + 33, 0),
                    color_primary: Rgb565::GREEN,
                    color_secondary: Rgb565::BLACK,
                },
                text: "500mV",
                font: MonoTextStyle::new(&FONT_6X9, Rgb565::BLACK),
            },
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

    pub async fn draw(&mut self) -> Result<()> {
        if self.state.updated.iter().all(|&x| x) {
            return Ok(());
        }
        if self.state.updated.iter().all(|&x| !x) {
            self.display
                .clear(Rgb565::CSS_DARK_SLATE_GRAY)
                .map_err(|_| AppError::DisplayError)?;
        }

        self.waveform.draw(&mut self.display, &mut self.state)?;
        self.running_state
            .draw(&mut self.display, &mut self.state)?;
        self.time_scale.draw(&mut self.display, &mut self.state)?;
        self.overview.draw(&mut self.display, &mut self.state)?;
        self.channel_info1
            .draw(&mut self.display, &mut self.state)?;
        self.channel_info2
            .draw(&mut self.display, &mut self.state)?;
        self.battery.draw(&mut self.display, &mut self.state)?;
        self.clock.draw(&mut self.display, &mut self.state)?;

        let mut drawed_panel_items = 0;
        for item in self
            .panel_items
            .iter_mut()
            .skip(self.state.panel_page as usize * 8)
            .take(8)
        {
            item.draw(&mut self.display, &mut self.state)?;
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
        for item in self.measure_items.iter_mut() {
            item.draw(&mut self.display, &mut self.state)?;
        }
        self.generator.0.draw(&mut self.display, &mut self.state)?;

        Ok(())
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task]
pub async fn main_loop(display: impl DrawTarget<Color = Rgb565> + 'static) {
    let mut app = App::new(display);
    loop {
        use crate::info;
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
        Timer::after_millis(20).await;
    }
}
