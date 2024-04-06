#![allow(dead_code)]

mod gui;
pub mod input;
mod misc;
mod state;
mod unit;

use core::ops::Range;

use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use gui::*;
use input::*;
use misc::*;
use state::*;
use unit::*;

pub use gui::GuiColor;
pub use misc::Result;

use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::*, MonoTextStyle},
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
    trigger_level: TriggerLevelDisp,

    // widgets of setting value window
    select_items: Option<SelectItem>,
}

impl<D> App<D>
where
    D: DrawTarget<Color = GuiColor> + 'static,
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
                    color_secondary: gui_color(0),
                },
            ),
            channel_info2: ChannelSettingDisp::new(
                ProbeChannel::B,
                GUIInfo {
                    size: Size::new(33, 10),
                    position: Point::new(205 + 33, 0),
                    color_primary: ProbeChannel::B.color(),
                    color_secondary: gui_color(0),
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
            trigger_level: Default::default(),
            select_items: Default::default(),
        }
    }

    async fn draw_main_window(&mut self) -> Result<()> {
        let display = &mut self.display;
        if self.updated.iter().all(|&x| !x) {
            // clear screen at the first time
            display
                .clear(GUI_BG_COLOR)
                .map_err(|_| AppError::DisplayError)?;
        }

        self.running_state
            .draw(display, &mut self.state, &mut self.updated)
            .await?;
        self.time_scale
            .draw(display, &mut self.state, &mut self.updated)
            .await?;
        self.overview
            .draw(display, &mut self.state, &mut self.updated)
            .await?;

        if !self.updated.at(StateMarker::ChannelSetting) {
            self.channel_info1
                .draw(display, &mut self.state, &mut self.updated)
                .await?;
            self.channel_info2
                .draw(display, &mut self.state, &mut self.updated)
                .await?;
            self.updated.confirm(StateMarker::ChannelSetting);
        }

        self.battery
            .draw(display, &mut self.state, &mut self.updated)
            .await?;
        self.clock
            .draw(display, &mut self.state, &mut self.updated)
            .await?;
        self.trigger_level
            .draw(display, &mut self.state, &mut self.updated)
            .await?;
        self.waveform
            .draw(display, &mut self.state, &mut self.updated)
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
                item.draw(display, &mut self.state, &mut self.updated)
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
                    .into_styled(PrimitiveStyle::with_fill(GUI_BG_COLOR))
                    .draw(display)
                    .map_err(|_| AppError::DisplayError)?;
                }
                // add info: 0 to switch page
                Text::with_alignment(
                    "0:Page",
                    Point::new(SCREEN_WIDTH as i32 - 24, SCREEN_HEIGHT as i32 - 11 * 3 + 5)
                        + TEXT_OFFSET,
                    MonoTextStyle::new(&FONT_6X9, gui_color(15)),
                    Alignment::Center,
                )
                .draw(display)
                .map_err(|_| AppError::DisplayError)?;
            }
            self.updated.confirm(StateMarker::PanelPage);
        }
        if !self.updated.at(StateMarker::Measures) {
            for item in self.measure_items.iter_mut() {
                item.draw(display, &mut self.state, &mut self.updated)
                    .await?;
            }
            self.updated.confirm(StateMarker::Measures);
        }
        self.generator
            .draw(display, &mut self.state, &mut self.updated)
            .await?;

        // // generate random data for testing
        // static mut OFFSET: f64 = 0.0;
        // use libm::*;
        // for channel in (ProbeChannel::B as usize)..(ProbeChannel::Endding as usize) {
        //     let data_new = (0..self.state.waveform[channel].len).map(|i| {
        //         (sin(i as f64 * 0.3 + unsafe { OFFSET })
        //             * 2.0
        //             * (if channel != 0 { 1.0 } else { -1.0 })) as f32
        //     });
        //     self.state.waveform[channel].append_iter(data_new)?;
        // }
        // unsafe {
        //     OFFSET += 0.1;
        //     if OFFSET >= 2.0 * 3.14159265 {
        //         OFFSET = 0.0;
        //         self.updated.request(StateMarker::Waveform);
        //     }
        // }
        // self.updated.request(StateMarker::WaveformData);
        Ok(())
    }

    async fn draw_set_value_window(&mut self) -> Result<()> {
        let display = &mut self.display;
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
                        color_primary: gui_color(5),
                        color_secondary: gui_color(0),
                    },
                    items,
                };
                self.select_items = Some(select_item);
            }
        }
        if !self.state.setting_inited {
            window_rect
                .clone()
                .into_styled(PrimitiveStyle::with_fill(gui_color(5)))
                .draw(display)
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
                MonoTextStyle::new(&FONT_7X13_BOLD, gui_color(0)),
                Alignment::Center,
            )
            .translate(window_rect.top_left)
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;

            self.updated.confirm(StateMarker::SettingValueTitle);
        }
        if let Some(select_items) = &self.select_items {
            select_items
                .draw(display, &mut self.state, &mut self.updated)
                .await?;
        }
        Ok(())
    }

    async fn draw_settings_window(&mut self) -> Result<()> {
        let display = &mut self.display;
        display
            .clear(gui_color(3))
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
                match key {
                    Keys::Sharp => {
                        self.updated.clear();
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
                    Keys::X => {
                        // change timebase mode
                        self.state.timebase_mode = match self.state.timebase_mode {
                            TimebaseMode::Normal => TimebaseMode::Rolling,
                            // TimebaseMode::Rolling => TimebaseMode::XY,
                            // TimebaseMode::XY => TimebaseMode::Normal,
                            _ => TimebaseMode::Normal,
                        };
                        self.updated.request(StateMarker::Waveform);
                        for w in self.state.waveform.iter_mut() {
                            w.clear();
                        }
                    }
                    _ => {}
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

    async fn find_triggered_offset(&self, data: &[f32]) -> Option<i32> {
        let length = data.len() as i32;
        let mut offset_result = length / 2;
        let mut offset_min_abs_mid = length / 2;
        let mut triggered = false;
        let mut triggered_once = false;
        let level_v = self.state.trigger_level_mv as f32 / 1000.0;
        for (i, &x) in data.iter().enumerate() {
            if x > level_v {
                triggered = true;
                if !triggered_once {
                    // defmt::info!("triggered at: {}", i);
                    let offset = i as i32;
                    let offset_mid = offset - length / 2;
                    if offset_mid.abs() <= offset_min_abs_mid {
                        // defmt::info!(
                        //     "selecting triggered offset: {}, last offset_result: {}",
                        //     offset_mid,
                        //     offset_result
                        // );
                        offset_min_abs_mid = offset_mid.abs();
                        offset_result = offset_mid;
                    } else {
                        break;
                    }
                    triggered_once = true;
                }
            } else {
                triggered_once = false;
            }
        }
        if !triggered {
            None
        } else {
            // defmt::info!(
            //     "find triggered offset: {}, data len: {}",
            //     offset_result,
            //     length
            // );
            Some(offset_result)
        }
    }

    pub async fn data_input(&mut self, data: &[f32], channel: ProbeChannel) -> Result<()> {
        // let s: &str = channel.into();
        // crate::info!("data input: {}, len {}", s, data.len());
        let idx: usize = channel.into();
        let offset = if self.state.timebase_mode == TimebaseMode::Normal {
            self.find_triggered_offset(data).await.unwrap_or(0)
        } else {
            0
        };
        let waveform = &mut self.state.waveform[idx];
        if self.state.timebase_mode == TimebaseMode::Normal {
            waveform.append_frame_iter(data.iter().copied(), offset)?;
        } else {
            waveform.append_rolling_data(data)?;
        }
        self.updated.request(StateMarker::WaveformData);
        Ok(())
    }
}

static KBD_CHANNEL: Channel<ThreadModeRawMutex, Keys, 16> = Channel::new();
static ADC_CHANNEL: Channel<ThreadModeRawMutex, usize, 1> = Channel::new();
static ADC_REQ_CHANNEL: Channel<ThreadModeRawMutex, AdcReadOptions, 8> = Channel::new();
static mut ADC_DATA: [f32; ADC_BUF_SZ] = [0.0; ADC_BUF_SZ];

#[embassy_executor::task]
async fn adc_task(
    receiver: Receiver<'static, ThreadModeRawMutex, AdcReadOptions, 8>,
    sender: Sender<'static, ThreadModeRawMutex, usize, 1>,
    mut adc: impl AdcDevice + 'static,
) {
    loop {
        let options = receiver.receive().await;
        let mut length = options.length;
        while length > 0 {
            let pos = options.length - length;
            let count = adc
                .read(AdcReadOptions { pos, ..options }, unsafe {
                    &mut ADC_DATA[pos..options.length]
                })
                .await
                .unwrap();
            sender.send(count).await;
            assert!(
                count <= length,
                "invalid count: {}, length: {}",
                count,
                length
            );
            length -= count;
        }
        // Timer::after_millis(1000).await;
    }
}

// struct KeepMutex;
// impl Drop for KeepMutex {
//     fn drop(&mut self) {
//         defmt::info!("KeepMutex drop");
//     }
// }
// static DEV_MUTEX: Mutex<ThreadModeRawMutex, KeepMutex> = Mutex::new(KeepMutex {});
#[embassy_executor::task]
async fn keyboad_task(
    sender: Sender<'static, ThreadModeRawMutex, Keys, 16>,
    mut keyboard: impl KeyboardDevice + 'static,
) {
    loop {
        let key = {
            // DEV_MUTEX.lock().await;
            keyboard.read_key()
        };
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

// #[cfg(feature = "embedded")]
// #[embassy_executor::task]
pub async fn main_loop<D, K, A, F>(
    spawner: embassy_executor::Spawner,
    display: D,
    keyboard: K,
    adc: A,
    loop_start: F,
) where
    D: DrawTarget<Color = GuiColor> + 'static,
    K: KeyboardDevice + 'static,
    A: AdcDevice + 'static,
    F: Fn(&D) -> (),
{
    let mut app = App::new(display);
    spawner
        .spawn(keyboad_task(KBD_CHANNEL.sender(), keyboard))
        .unwrap();
    spawner
        .spawn(adc_task(
            ADC_REQ_CHANNEL.receiver(),
            ADC_CHANNEL.sender(),
            adc,
        ))
        .unwrap();
    let mut send_req = true;
    loop {
        loop_start(&app.display);
        if send_req {
            let request_length = match app.state.timebase_mode {
                TimebaseMode::Normal => WAVEFORM_LEN,
                TimebaseMode::Rolling => (WAVEFORM_LEN / 128).max(1),
                TimebaseMode::XY => WAVEFORM_LEN,
            };
            ADC_REQ_CHANNEL
                .send(AdcReadOptions::new(ProbeChannel::A, request_length, 1000))
                .await;
            send_req = false;
        }
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
        match ADC_CHANNEL.try_receive() {
            Ok(sz) => {
                // defmt::info!("recv ADC data: {}", sz);
                let data = unsafe { core::slice::from_raw_parts(ADC_DATA.as_ptr(), sz) };
                app.data_input(data, ProbeChannel::A).await.unwrap();
                send_req = true;
            }
            Err(_) => {}
        }
        Timer::after_millis(5).await;
    }
}
