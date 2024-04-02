#![allow(dead_code)]

mod gui;
pub mod input;
mod unit;

use core::ops::Range;

use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Sender},
};
use gui::*;
use input::*;

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum RunningState {
    #[default]
    Stopped,
    Running,
}
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

#[derive(FromPrimitive, Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[repr(usize)]
pub enum Panel {
    #[default]
    Channel,
    TimeScale,
    VoltageScale,
    Xpos,
    Ypos,
    TrigerLevel,
    Couple,
    TrigerType,
    Probe,
    HorMeasure1,
    VerMeasure1,
    HorMeasure2,
    VerMeasure2,
    Sweep,
    Endding,
}
impl Panel {
    pub fn style(&self) -> PanelStyle {
        match self {
            Panel::Channel => PanelStyle::ChannelColor,
            Panel::VoltageScale => PanelStyle::ChannelColor,
            Panel::Ypos => PanelStyle::ChannelColor,
            Panel::Couple => PanelStyle::ChannelColor,
            Panel::Probe => PanelStyle::ChannelColor,
            Panel::HorMeasure1 => PanelStyle::ChannelColor,
            Panel::VerMeasure1 => PanelStyle::ChannelColor,
            Panel::HorMeasure2 => PanelStyle::ChannelColor,
            Panel::VerMeasure2 => PanelStyle::ChannelColor,
            _ => PanelStyle::Normal,
        }
    }
}
impl Into<&'static str> for Panel {
    fn into(self) -> &'static str {
        match self {
            Panel::Channel => "Chann",
            Panel::TimeScale => "T-Sca",
            Panel::VoltageScale => "V-Sca",
            Panel::Xpos => "Xpos",
            Panel::Ypos => "Ypos",
            Panel::TrigerLevel => "T-thr",
            Panel::Couple => "Coup",
            Panel::TrigerType => "T-Typ",
            Panel::Probe => "Probe",
            Panel::HorMeasure1 => "H-Me1",
            Panel::VerMeasure1 => "V-Me1",
            Panel::HorMeasure2 => "H-Me2",
            Panel::VerMeasure2 => "V-Me2",
            Panel::Sweep => "Sweep",
            Panel::Endding => "Endding",
        }
    }
}

#[derive(Debug)]
pub struct State {
    pub panel_page: u8,
    pub panel_focused: Option<u8>,
    pub running_state: RunningState,
    pub battery: u8,
    pub clock: [char; 5],
    pub window: Window,
    pub window_next: Option<Window>,
    // TODO: waveform data
    pub waveform: u8,
    pub time_scale_ns: u64,
    // TODO: channel setting
    pub channel_info: u64,
    // TODO: measures
    pub measures: u64,
    // TODO: generator setting
    pub generator: u64,

    // used in setting value window
    pub setting_index: u8,
    pub setting_inited: bool,
    pub setting_time_scale: TimeScale,
    pub setting_voltage_scale: VoltageScale,
    pub setting_select_idx: [u8; 3],
    pub setting_select_sel: u8,
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
            panel_focused: Default::default(),
            running_state: Default::default(),
            battery: 50,
            clock: ['1', '2', ':', '4', '5'],
            window: Default::default(),
            window_next: Default::default(),
            waveform: Default::default(),
            time_scale_ns: 100_000,
            channel_info: 500,
            measures: Default::default(),
            generator: Default::default(),
            setting_index: Default::default(),
            setting_inited: Default::default(),
            setting_time_scale: Default::default(),
            setting_voltage_scale: Default::default(),
            setting_select_idx: Default::default(),
            setting_select_sel: Default::default(),
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
pub enum ProbeChannel {
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
    transform::Transform,
    Drawable,
};
use num_enum::{FromPrimitive, IntoPrimitive};

use self::unit::{TimeScale, VoltageScale};

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
    panel_items: [PanelItem; Panel::Endding as usize],
    measure_items: [MeasureItem; 4],
    generator: Generator,
}

impl<D> App<D>
where
    D: DrawTarget<Color = Rgb565> + 'static,
{
    pub fn new(display: D) -> Self {
        let panel_items = core::array::from_fn(|i| {
            let p = Panel::from(i);
            PanelItem::new(i as u8, p.into(), "--", p.style())
        });
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
            // panel_items: [
            //     PanelItem::new(0, "Chann", "CHA", PanelStyle::ChannelColor),
            //     PanelItem::new(1, "T-Sca", "100ms", PanelStyle::Normal),
            //     PanelItem::new(2, "V-Sca", "20mV", PanelStyle::ChannelColor),
            //     PanelItem::new(3, "Xpos", "0.0ns", PanelStyle::Normal),
            //     PanelItem::new(4, "Ypos", "0.0ns", PanelStyle::ChannelColor),
            //     PanelItem::new(5, "T-thr", "-3.03mV", PanelStyle::Normal),
            //     PanelItem::new(6, "Coup", "DC", PanelStyle::ChannelColor),
            //     PanelItem::new(7, "T-Typ", "CHA-U", PanelStyle::Normal),
            //     PanelItem::new(8, "Probe", "X2", PanelStyle::ChannelColor),
            //     PanelItem::new(9, "H-Me1", "Freq", PanelStyle::ChannelColor),
            //     PanelItem::new(10, "V-Me1", "Vp-p", PanelStyle::ChannelColor),
            //     PanelItem::new(11, "H-Me2", "--", PanelStyle::ChannelColor),
            //     PanelItem::new(12, "V-Me2", "Vrms", PanelStyle::ChannelColor),
            //     PanelItem::new(13, "Sweep", "AUTO", PanelStyle::Normal),
            // ],
            panel_items,
            measure_items: [
                MeasureItem::new(0, ProbeChannel::A, "Freq", "34kHz", true),
                MeasureItem::new(1, ProbeChannel::A, "Vp-p", "2.3mV", false),
                MeasureItem::new(2, ProbeChannel::B, "Freq", "--", true),
                MeasureItem::new(3, ProbeChannel::B, "Vrms", "430uV", true),
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
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
        self.time_scale
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
        self.overview
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;

        if !self.updated.at(StateMarker::ChannelSetting) {
            self.channel_info1
                .draw(&mut self.display, &mut self.state, &mut self.updated)
                .await?;
            self.channel_info2
                .draw(&mut self.display, &mut self.state, &mut self.updated)
                .await?;
            self.updated.set(StateMarker::ChannelSetting, true);
        }

        self.battery
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
        self.clock
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
        self.waveform
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;

        if !self.updated.at(StateMarker::PanelPage) {
            // Rectangle::new(
            //     self.panel_items[0].info.position,
            //     Size::new(
            //         self.panel_items[0].info.size.width,
            //         (self.panel_items[0].info.size.height + 1) * 8,
            //     ),
            // )
            // .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_DARK_SLATE_GRAY))
            // .draw(&mut self.display)
            // .map_err(|_| AppError::DisplayError)?;
            // let mut drawed_panel_items = 0;
            let drawed_panel_items = if self.state.panel_page == 0 {
                8
            } else {
                self.panel_items.len() - 8
            };
            for item in self
                .panel_items
                .iter_mut()
                .skip(self.state.panel_page as usize * 8)
                .take(8)
            {
                item.draw(&mut self.display, &mut self.state, &mut self.updated)
                    .await?;
                // drawed_panel_items += 1;
            }
            if drawed_panel_items < 8 {
                // clear some items
                let range = Range {
                    start: drawed_panel_items,
                    end: 8,
                };
                for i in range {
                    Rectangle::new(
                        self.panel_items[i].info.position,
                        self.panel_items[i].info.size,
                    )
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_DARK_SLATE_GRAY))
                    .draw(&mut self.display)
                    .map_err(|_| AppError::DisplayError)?;
                }
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
                item.draw(&mut self.display, &mut self.state, &mut self.updated)
                    .await?;
            }
            self.updated.set(StateMarker::Measures, true);
        }
        self.generator
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;

        // self.updated[StateMarker::RunningState as usize] = false;
        // if self.state.running_state == RunningState::Running {
        //     self.state.running_state = RunningState::Stopped;
        // } else {
        //     self.state.running_state = RunningState::Running;
        // }

        Ok(())
    }

    async fn draw_set_value_window(&mut self) -> Result<()> {
        let window_size = Size::new(128, 84);
        let window_rect = Rectangle::with_center(
            self.waveform.info.position
                + Point::new(
                    self.waveform.info.size.width as i32 / 2,
                    self.waveform.info.size.height as i32 / 2,
                ),
            window_size,
        );
        if !self.state.setting_inited {
            window_rect
                .clone()
                .into_styled(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
                .draw(&mut self.display)
                .map_err(|_| AppError::DisplayError)?;
            return Ok(());
        }
        let pannel = &self.panel_items[self.state.setting_index as usize];
        Text::with_alignment(
            pannel.label,
            Point::new(window_size.width as i32 / 2, 10),
            MonoTextStyle::new(&FONT_7X13_BOLD, Rgb565::BLACK),
            Alignment::Center,
        )
        .translate(window_rect.top_left)
        .draw(&mut self.display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(())
    }

    async fn draw_settings_window(&mut self) -> Result<()> {
        self.display
            .clear(Rgb565::GREEN)
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

    pub async fn value_init(&mut self) -> Result<()> {
        match self.state.window {
            Window::SetValue => {
                if !self.state.setting_inited {
                    // TODO: Read setting value from state
                    self.state.setting_inited = true;
                }
            }
            _ => {}
        }
        Ok(())
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
                    Keys::Key1
                    | Keys::Key2
                    | Keys::Key3
                    | Keys::Key4
                    | Keys::Key5
                    | Keys::Key6
                    | Keys::Key7
                    | Keys::Key8 => {
                        let idx = (key.digital_value() - 1) + self.state.panel_page * 8;
                        if idx < self.panel_items.len() as u8 {
                            self.state.setting_inited = false;
                            self.state.window_next = Some(Window::SetValue);
                            self.state.setting_index = idx;
                            self.state.panel_focused = Some(idx);
                            self.updated.set(StateMarker::PanelPage, false);
                        }
                    }
                    _ => {}
                }
                if flush {
                    self.updated.clear();
                }
                Ok(())
            }
            Window::SetValue => {
                match key {
                    Keys::Ok => {
                        // reset states
                        self.state.setting_inited = false;

                        // self.state.window_next = Some(Window::Main);
                        self.state.window = Window::Main;
                        self.state.panel_focused = None;
                        // self.updated.clear();
                        self.updated.set(StateMarker::PanelPage, false);
                        self.updated.set(StateMarker::Waveform, false);
                    }
                    _ => {}
                }
                Ok(())
            }
            Window::Settings => Ok(()),
        }
    }
}

static KBD_CHANNEL: Channel<ThreadModeRawMutex, Keys, 16> = Channel::new();

#[embassy_executor::task]
async fn keyboad_task(
    sender: Sender<'static, ThreadModeRawMutex, Keys, 16>,
    mut keyboard: impl KeyboardDevice + 'static,
) {
    loop {
        let key = keyboard.read_key();
        if key != Keys::None {
            // let s: &str = key.into();
            // crate::debug!("send Key: {:?}", s);
            sender.send(key).await;
        } else {
            // crate::debug!("send no Key");
        }
        Timer::after_millis(10).await;
    }
}

#[cfg(feature = "embedded")]
// #[embassy_executor::task]
pub async fn main_loop<D, K>(spawner: embassy_executor::Spawner, display: D, keyboard: K)
where
    D: DrawTarget<Color = Rgb565> + 'static,
    K: KeyboardDevice + 'static,
{
    let mut app = App::new(display);
    spawner
        .spawn(keyboad_task(KBD_CHANNEL.sender(), keyboard))
        .unwrap();
    loop {
        app.draw().await.unwrap();
        app.value_init().await.unwrap();
        if let Some(window_next) = app.state.window_next {
            app.state.window = window_next;
        }
        app.state.window_next = None;
        match KBD_CHANNEL.try_receive() {
            Ok(key) => {
                let s: &str = key.into();
                crate::info!("recv Key: {:?}", s);
                app.input_key_event(key).unwrap();
            }
            Err(_) => {}
        }
        Timer::after_millis(50).await;
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
        app.value_init().await.unwrap();

        if let Some(window_next) = app.state.window_next {
            app.state.window = window_next;
        }
        app.state.window_next = None;

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
