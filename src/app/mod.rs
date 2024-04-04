#![allow(dead_code)]

mod gui;
pub mod input;
mod misc;
mod state;
mod unit;

use core::ops::Range;

use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Sender},
};
use gui::*;
use input::*;
use misc::*;
use state::*;
use unit::*;

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

    // widgets of setting value window
    select_items: Option<SelectItem>,
}

impl<D> App<D>
where
    D: DrawTarget<Color = Rgb565> + 'static,
{
    pub fn new(display: D) -> Self {
        let panel_items = core::array::from_fn(|i| {
            let p = Panel::from(i);
            PanelItem::new(p, p.into(), "--", p.style())
        });
        Self {
            state: Default::default(),
            updated: Default::default(),
            display,
            waveform: Default::default(),
            running_state: Default::default(),
            time_scale: Default::default(),
            overview: Overview::new(),
            channel_info1: ChannelSettingDisp::new(
                ProbeChannel::A,
                GUIInfo {
                    size: Size::new(33, 10),
                    position: Point::new(204, 0),
                    color_primary: ProbeChannel::A.color(),
                    color_secondary: Rgb565::BLACK,
                },
            ),
            channel_info2: ChannelSettingDisp::new(
                ProbeChannel::B,
                GUIInfo {
                    size: Size::new(33, 10),
                    position: Point::new(205 + 33, 0),
                    color_primary: ProbeChannel::B.color(),
                    color_secondary: Rgb565::BLACK,
                },
            ),
            battery: Battery::new(50),
            clock: Clock::new(),
            panel_items,
            measure_items: [
                MeasureItem::new(0, ProbeChannel::A, "Freq", "34kHz", true),
                MeasureItem::new(1, ProbeChannel::A, "Vp-p", "2.3mV", false),
                MeasureItem::new(2, ProbeChannel::B, "Freq", "--", true),
                MeasureItem::new(3, ProbeChannel::B, "Vrms", "430uV", true),
            ],
            generator: Generator::new("Sin 10k"),
            select_items: Default::default(),
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
            self.updated.confirm(StateMarker::ChannelSetting);
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
            self.updated.confirm(StateMarker::PanelPage);
        }
        if !self.updated.at(StateMarker::Measures) {
            for item in self.measure_items.iter_mut() {
                item.draw(&mut self.display, &mut self.state, &mut self.updated)
                    .await?;
            }
            self.updated.confirm(StateMarker::Measures);
        }
        self.generator
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
        Ok(())
    }

    async fn draw_set_value_window(&mut self) -> Result<()> {
        let title_height = 14;
        let item_height = 11;
        let window_height_min = 44 + title_height;
        let panel = Panel::from(self.state.setting_index as usize);
        let window_height_max = item_height * item_height + title_height;
        let window_height = if let Some(items) = panel.select_items() {
            items[0].len() as i32 * item_height + title_height
        } else {
            window_height_min
        } as u32;
        let window_height = window_height.clamp(window_height_min as u32, window_height_max as u32);
        let window_size = Size::new(128, window_height);
        let window_rect = Rectangle::with_center(
            self.waveform.info.position
                + Point::new(
                    self.waveform.info.size.width as i32 / 2,
                    self.waveform.info.size.height as i32 / 2,
                ),
            window_size,
        );
        if self.select_items.is_none() {
            if let Some(items) = panel.select_items() {
                let select_item = SelectItem {
                    info: GUIInfo {
                        size: Size::new(128, 84),
                        position: Point::new(0, 14) + window_rect.top_left,
                        color_primary: Rgb565::MAGENTA,
                        color_secondary: Rgb565::BLACK,
                    },
                    items,
                };
                self.select_items = Some(select_item);
            }
        }
        if !self.state.setting_inited {
            window_rect
                .clone()
                .into_styled(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
                .draw(&mut self.display)
                .map_err(|_| AppError::DisplayError)?;
            return Ok(());
        }
        if !self.updated.at(StateMarker::SettingValueTitle) {
            let _pannel_item = &self.panel_items[self.state.setting_index as usize];
            let mut buf = [0u8; 16];
            let title = format_no_std::show(&mut buf, format_args!("{}", panel.str())).unwrap();
            Text::with_alignment(
                title,
                Point::new(window_size.width as i32 / 2, 10),
                MonoTextStyle::new(&FONT_7X13_BOLD, Rgb565::BLACK),
                Alignment::Center,
            )
            .translate(window_rect.top_left)
            .draw(&mut self.display)
            .map_err(|_| AppError::DisplayError)?;

            self.updated.confirm(StateMarker::SettingValueTitle);
        }
        if let Some(select_items) = &self.select_items {
            select_items
                .draw(&mut self.display, &mut self.state, &mut self.updated)
                .await?;
        }
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
                    self.state.setting_select_col = 0;
                    self.state
                        .setting_select_idx
                        .iter_mut()
                        .for_each(|x| *x = 0);
                    // Read setting value from state
                    let panel = Panel::from(self.state.setting_index as usize);
                    match panel {
                        Panel::Channel => {
                            let idx: usize = self.state.channel_current.into();
                            self.state.setting_select_idx[0] = idx as u8;
                        }
                        Panel::VoltageScale => {
                            let idx_channel: usize = self.state.channel_current.into();
                            let mv = self.state.channel_info[idx_channel].voltage_scale_mv;
                            let voltage_scale = VoltageScale::from_mv(mv);
                            self.state.setting_voltage_scale = voltage_scale;
                            self.state.setting_select_idx[0] =
                                match self.state.setting_voltage_scale.voltage {
                                    1 => 0,
                                    2 => 1,
                                    5 => 2,
                                    10 => 3,
                                    20 => 4,
                                    50 => 5,
                                    100 => 6,
                                    200 => 7,
                                    500 => 8,
                                    _ => 0,
                                };
                            self.state.setting_select_idx[1] =
                                match self.state.setting_voltage_scale.unit {
                                    VoltageUnit::MilliVolt => 0,
                                    VoltageUnit::Volt => 1,
                                };
                        }
                        _ => {
                            // crate::warn!("not emplemented: {:?}", panel);
                        }
                    }
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
                        self.updated.request(StateMarker::PanelPage);
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
                            self.state.panel_focused = Some(Panel::from(idx as usize));
                            self.updated.request(StateMarker::PanelPage);
                        }
                    }
                    Keys::Lock => {
                        self.state.running_state = match self.state.running_state {
                            RunningState::Running => RunningState::Stopped,
                            RunningState::Stopped => RunningState::Running,
                        };
                        self.updated.request(StateMarker::RunningState);
                    }
                    _ => {}
                }
                if flush {
                    self.updated.clear();
                }
                Ok(())
            }
            Window::SetValue => {
                let panel = Panel::from(self.state.setting_index as usize);
                match key {
                    Keys::Left => {
                        if let Some(items) = panel.select_items() {
                            self.state.setting_select_col = if self.state.setting_select_col >= 1 {
                                self.state.setting_select_col - 1
                            } else {
                                // 0
                                items.len() as u8 - 1
                            };
                            self.updated.request(StateMarker::SettingValueContent);
                        }
                        self.updated.request(StateMarker::SettingValueContent);
                    }
                    Keys::Right => {
                        if let Some(items) = panel.select_items() {
                            self.state.setting_select_col =
                                if self.state.setting_select_col < items.len() as u8 - 1 {
                                    self.state.setting_select_col + 1
                                } else {
                                    // items.len() as u8 - 1
                                    0
                                };
                            self.updated.request(StateMarker::SettingValueContent);
                        }
                        self.updated.request(StateMarker::SettingValueContent);
                    }
                    Keys::Up => {
                        if let Some(items) = panel.select_items() {
                            self.state.setting_select_idx[self.state.setting_select_col as usize] =
                                if self.state.setting_select_idx
                                    [self.state.setting_select_col as usize]
                                    >= 1
                                {
                                    self.state.setting_select_idx
                                        [self.state.setting_select_col as usize]
                                        - 1
                                } else {
                                    // 0
                                    (items[self.state.setting_select_col as usize].len() - 1) as u8
                                };
                            self.updated.request(StateMarker::SettingValueContent);
                        }
                    }
                    Keys::Down => {
                        if let Some(items) = panel.select_items() {
                            self.state.setting_select_idx[self.state.setting_select_col as usize] =
                                if self.state.setting_select_idx
                                    [self.state.setting_select_col as usize]
                                    < items[self.state.setting_select_col as usize].len() as u8 - 1
                                {
                                    let v = self.state.setting_select_idx
                                        [self.state.setting_select_col as usize]
                                        + 1;
                                    // crate::info!(
                                    //     "next value: {}, items: {:?}",
                                    //     v,
                                    //     items[self.state.setting_select_col as usize]
                                    // );
                                    v
                                } else {
                                    // (items[self.state.setting_select_col as usize].len() - 1) as u8
                                    0
                                };
                            self.updated.request(StateMarker::SettingValueContent);
                        }
                    }
                    Keys::Ok => {
                        let ok = {
                            let panel = Panel::from(self.state.setting_index as usize);
                            // FIXME: when not setting panel data?
                            match panel {
                                Panel::Channel => {
                                    let idx = self.state.setting_select_idx[0] as usize;
                                    let ch = match idx {
                                        0 => ProbeChannel::A,
                                        1 => ProbeChannel::B,
                                        _ => ProbeChannel::A,
                                    };
                                    self.state.channel_current = ch;
                                    true
                                }
                                Panel::VoltageScale => {
                                    let idx = self.state.setting_select_idx[0] as usize;
                                    let n = match idx {
                                        0 => 1,
                                        1 => 2,
                                        2 => 5,
                                        3 => 10,
                                        4 => 20,
                                        5 => 50,
                                        6 => 100,
                                        7 => 200,
                                        8 => 500,
                                        _ => panic!("invalid voltage scale index"),
                                    };
                                    let unit = match self.state.setting_select_idx[1] {
                                        0 => VoltageUnit::MilliVolt,
                                        1 => VoltageUnit::Volt,
                                        _ => panic!("invalid voltage unit index"),
                                    };
                                    self.state.setting_voltage_scale = Default::default();
                                    let voltage_scale = VoltageScale::new(n, unit);
                                    let idx_channel: usize = self.state.channel_current.into();
                                    self.state.channel_info[idx_channel].voltage_scale_mv =
                                        voltage_scale.to_mv();
                                    self.updated.request(StateMarker::ChannelSetting);
                                    true
                                }
                                _ => {
                                    // crate::warn!("not emplemented: {:?}", panel);
                                    true
                                }
                            }
                        };

                        if ok {
                            // reset states
                            self.state.setting_inited = false;
                            self.select_items = None;

                            // self.state.window_next = Some(Window::Main);
                            self.state.window = Window::Main;
                            self.state.panel_focused = None;
                            // self.updated.clear();
                            self.updated.requests(&[
                                StateMarker::SettingValueTitle,
                                StateMarker::SettingValueContent,
                                StateMarker::PanelPage,
                                StateMarker::Waveform,
                            ]);
                        } else {
                            crate::warn!("invalid setting value");
                        }
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
