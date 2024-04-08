#![allow(dead_code)]

pub mod devices;
mod gui;
mod misc;
mod state;
mod unit;

use core::ops::Range;

use devices::*;
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use gui::*;
use misc::*;
use state::*;
use unit::*;

pub use gui::GuiColor;
pub use misc::{AppError, ProbeChannel, Result};
pub use state::State;

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
pub struct App<D, B, Z> {
    pub state: State,
    pub updated: StateVec,
    pub display: D,
    pub board: B,
    pub buzzer: Z,

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

    // setting menu
    menu: Menu<MenuId>,
}

const STATE_OFFSET: u32 = 0;
impl<D, B, Z> App<D, B, Z>
where
    D: DrawTarget<Color = GuiColor> + 'static,
    B: BoardDevice + NvmDevice,
    Z: BuzzerDevice,
{
    async fn load_state(&mut self) -> Result<()> {
        // read state magic
        let magic: u64 = 0;
        self.board
            .read(STATE_OFFSET, magic.to_ne_bytes().as_mut())?;
        if magic == STATE_MAGIC {
            defmt::info!("state magic matched");
            // read version
            let version: u64 = 0;
            self.board
                .read(STATE_OFFSET + 8, version.to_ne_bytes().as_mut())?;
            if version == self.state.version {
                defmt::info!("state version matched");
                // read state
                let state: State = Default::default();
                let ptr = &state as *const State as *mut u8;
                self.board.read(STATE_OFFSET, unsafe {
                    core::slice::from_raw_parts_mut(ptr, core::mem::size_of::<State>())
                })?;
                self.state = state;
                Ok(())
            } else {
                defmt::info!("state version mismatched");
                Err(AppError::StateFormatError)
            }
        } else {
            defmt::info!(
                "state magic mismatched, read: {:x} expected: {:x}",
                magic,
                STATE_MAGIC
            );
            Err(AppError::StateFormatError)
        }
    }
    async fn save_state(&mut self) -> Result<()> {
        // write state
        let ptr = &self.state as *const State as *const u8;
        self.board
            .erase(STATE_OFFSET, core::mem::size_of::<State>())?;
        defmt::info!(
            "saving state: {=[u8]:x} at ptr {} or {}",
            unsafe { core::slice::from_raw_parts(ptr, 16) },
            ptr,
            &self.state.magic as *const u64 as *const u8
        );
        defmt::info!("state erased");
        self.board.write(STATE_OFFSET, unsafe {
            core::slice::from_raw_parts(ptr, core::mem::size_of::<State>())
        })?;
        embassy_time::Timer::after_millis(100).await;
        // check written state
        let state_magic_written: u64 = 0;
        self.board
            .read(STATE_OFFSET, state_magic_written.to_ne_bytes().as_mut())?;
        if state_magic_written == STATE_MAGIC {
            defmt::info!("state saved, magic matched");
        } else {
            defmt::warn!(
                "state saved, magic mismatched, read: {:x}",
                state_magic_written
            );
        }
        Ok(())
    }
    async fn init(mut self) -> Self {
        match self.load_state().await {
            Ok(_) => {
                defmt::info!("state loaded");
            }
            Err(_) => {
                defmt::info!("state not loaded, save default");
                match self.save_state().await {
                    Ok(_) => {
                        defmt::info!("state saved");
                    }
                    Err(_) => {
                        defmt::warn!("state not saved");
                    }
                }
            }
        }
        self
    }
    pub async fn new(display: D, board: B, buzzer: Z) -> Self {
        let panel_items = core::array::from_fn(|i| {
            let p = Panel::from(i);
            PanelItem::new(p, p.into(), "--", p.style())
        });
        Self {
            state: Default::default(),
            updated: Default::default(),
            display,
            board,
            buzzer,
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
            menu: Menu::new(&MENU),
        }
        .init()
        .await
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
        if !self.updated.at(StateMarker::SettingsMenu) {
            // redraw background
            let _ = self.waveform.draw_state_vec(
                &mut self.display,
                &mut self.state,
                &mut self.updated,
            )?;
        }
        self.menu
            .draw(&mut self.display, &mut self.state, &mut self.updated)
            .await?;
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

impl<D, B, Z> App<D, B, Z>
where
    B: BoardDevice,
    Z: BuzzerDevice,
{
    fn clear_update_state(&mut self) {
        for x in self.updated.iter_mut() {
            *x = false;
        }
    }
    pub async fn input_key_event(&mut self, key: Keys) -> Result<()> {
        // beep if volume is not 0
        if self.state.volume != 0 {
            self.buzzer.beep(523, 10).await;
        }
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
                    Keys::Ok => {
                        // go to settings
                        self.state.setting_inited = false;
                        self.state.menu_idx_l1 = 0;
                        self.state.menu_idx_l1_last = None;
                        self.state.menu_idx_l2 = None;
                        self.state.menu_idx_l2_last = None;
                        self.state.window_next = Some(Window::Settings);
                        self.updated.request(StateMarker::SettingsMenu);
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
            Window::Settings => {
                match key {
                    Keys::X => {
                        // cancel
                        self.state.window = Window::Main;
                        self.updated.clear();
                    }
                    Keys::Down => {
                        if let Some(idx) = self.state.menu_idx_l2 {
                            // in level 2 menu
                            let len = self.menu.items[self.state.menu_idx_l1].2.len();
                            self.state.menu_idx_l2_last = Some(idx);
                            self.state.menu_idx_l2 = Some((idx + 1) % len);
                            self.updated.request(StateMarker::SettingsMenuMoveL2);
                        } else {
                            // in level 1 menu
                            self.state.menu_idx_l1_last = Some(self.state.menu_idx_l1);
                            self.state.menu_idx_l1 =
                                (self.state.menu_idx_l1 + 1) % self.menu.items.len();
                            self.updated.request(StateMarker::SettingsMenuMoveL1);
                        }
                    }
                    Keys::Up => {
                        if let Some(idx) = self.state.menu_idx_l2 {
                            // in level 2 menu
                            let len = self.menu.items[self.state.menu_idx_l1].2.len();
                            self.state.menu_idx_l2_last = Some(idx);
                            self.state.menu_idx_l2 = Some((idx + len - 1) % len);
                            self.updated.request(StateMarker::SettingsMenuMoveL2);
                        } else {
                            // in level 1 menu
                            self.state.menu_idx_l1_last = Some(self.state.menu_idx_l1);
                            self.state.menu_idx_l1 =
                                (self.state.menu_idx_l1 + self.menu.items.len() - 1)
                                    % self.menu.items.len();
                            self.updated.request(StateMarker::SettingsMenuMoveL1);
                        }
                    }
                    Keys::Right | Keys::Ok => {
                        if let Some(idx_level2) = self.state.menu_idx_l2 {
                            // select level 2 item
                            let menu_id = self.menu.items[self.state.menu_idx_l1].2[idx_level2].0;
                            if let Some(menu_id) = menu_id {
                                self.handle_menu_exit(Some(menu_id)).await?;
                            } else {
                                let menu_id = self.menu.items[self.state.menu_idx_l1].0;
                                self.handle_menu_exit(menu_id).await?;
                            }
                        } else {
                            // into level 2 menu
                            let len = self.menu.items[self.state.menu_idx_l1].2.len();
                            if len > 0 {
                                match self.before_handle_menu() {
                                    Err(_) => self.state.menu_idx_l2 = Some(0),
                                    _ => {}
                                }
                                self.updated.request(StateMarker::SettingsMenu);
                            } else {
                                // select level 1 item
                                self.handle_menu_exit(self.menu.items[self.state.menu_idx_l1].0)
                                    .await?;
                            }
                        }
                    }
                    Keys::Left => {
                        // back to level 1 menu
                        if self.state.menu_idx_l2.is_some() {
                            self.state.menu_idx_l2 = None;
                            self.updated.request(StateMarker::SettingsMenu);
                        } else {
                            // back to main window
                            self.state.window = Window::Main;
                            self.updated.clear();
                        }
                    }
                    _ => {}
                }
                Ok(())
            }
        }
    }

    fn before_handle_menu(&mut self) -> Result<()> {
        // to select initial value of menu_idx_l2
        let menu_id = self.menu.items[self.state.menu_idx_l1].0;
        if let Some(menu_id) = menu_id {
            match menu_id {
                MenuId::Backlight => {
                    let bl = self.state.backlight;
                    let idx = match bl {
                        0..=10 => 0,
                        11..=20 => 1,
                        21..=40 => 2,
                        41..=60 => 3,
                        61..=80 => 4,
                        _ => 5,
                    };
                    self.state.menu_idx_l2 = Some(idx);
                    Ok(())
                }
                MenuId::Volume => {
                    let idx = if self.state.volume == 0 { 0 } else { 1 };
                    self.state.menu_idx_l2 = Some(idx);
                    Ok(())
                }
                _ => Err(AppError::NotImplemented),
            }
        } else {
            Err(AppError::NotImplemented)
        }
    }

    async fn handle_menu(&mut self, menu_id: MenuId) -> Result<()> {
        match menu_id {
            MenuId::Backlight => {
                let bl = match self.state.menu_idx_l2 {
                    Some(0) => 10,
                    Some(1) => 20,
                    Some(2) => 40,
                    Some(3) => 60,
                    Some(4) => 80,
                    Some(5) => 100,
                    _ => 100,
                };
                self.board.set_brightness(bl);
                Ok(())
            }
            MenuId::Volume => {
                let volume = match self.state.menu_idx_l2 {
                    Some(0) => 0,
                    Some(1) => 100,
                    _ => 100,
                };
                self.state.volume = volume;
                Ok(())
            }
            _ => Err(AppError::NotImplemented),
        }
    }

    async fn handle_menu_exit(&mut self, menu_id: Option<MenuId>) -> Result<()> {
        if let Some(menu_id) = menu_id {
            match self.handle_menu(menu_id).await {
                Ok(_) => {
                    if self.state.window == Window::Settings && self.state.window_next.is_none() {
                        self.state.window = Window::Main;
                        self.updated.clear();
                    }
                    Ok(())
                }
                Err(e) => {
                    crate::warn!("handle menu error: {}", e.str());
                    Ok(())
                }
            }
        } else {
            Ok(())
        }
    }

    async fn find_triggered_offset(&self, data: &[f32]) -> Option<i32> {
        let length = data.len() as i32;
        if length == 0 {
            return None;
        }
        let mut offset_result = length / 2;
        let mut offset_min_abs_mid = length / 2;
        let mut triggered = false;
        let level_v = self.state.trigger_level_mv as f32 / 1000.0;
        let mut triggered_once = data[0] > level_v;
        for (i, &x) in data.iter().enumerate() {
            if x > level_v {
                if !triggered_once {
                    triggered = true;
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

#[derive(Default, Debug, Copy, Clone)]
#[repr(usize)]
pub enum MenuId {
    #[default]
    About,
    Backlight,
    Volume,
}
static MENU: MenuItems<3, MenuId> = [
    (Some(MenuId::About), "About", &[]),
    (
        Some(MenuId::Backlight),
        "Backlight",
        &[
            (None, "10%"),
            (None, "20%"),
            (None, "40%"),
            (None, "60%"),
            (None, "80%"),
            (None, "100%"),
        ],
    ),
    (
        Some(MenuId::Volume),
        "Volume",
        &[(None, "OFF"), (None, "ON")],
    ),
];

static KBD_CHANNEL: Channel<ThreadModeRawMutex, Keys, 16> = Channel::new();
static BUZZER_CHANNEL: Channel<ThreadModeRawMutex, (u32, u32), 8> = Channel::new();
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

#[embassy_executor::task]
async fn buzzer_task(
    receiver: Receiver<'static, ThreadModeRawMutex, (u32, u32), 8>,
    mut buzzer: impl BuzzerDevice + 'static,
) {
    loop {
        let (freq, duration) = receiver.receive().await;
        buzzer.beep(freq, duration).await;
    }
}

struct ChanneledBuzzer {
    sender: Sender<'static, ThreadModeRawMutex, (u32, u32), 8>,
}
impl BuzzerDevice for ChanneledBuzzer {
    async fn beep(&mut self, freq: u32, duration: u32) {
        self.sender.send((freq, duration)).await;
    }
}

pub async fn main_loop<D, B, Z, K, A, F>(
    spawner: embassy_executor::Spawner,
    display: D,
    board: B,
    buzzer: Z,
    keyboard: K,
    adc: A,
    loop_start: F,
) where
    D: DrawTarget<Color = GuiColor> + 'static,
    B: BoardDevice + NvmDevice + 'static,
    Z: BuzzerDevice + 'static,
    K: KeyboardDevice + 'static,
    A: AdcDevice + 'static,
    F: Fn(&D) -> (),
{
    spawner
        .spawn(keyboad_task(KBD_CHANNEL.sender(), keyboard))
        .unwrap();
    spawner
        .spawn(buzzer_task(BUZZER_CHANNEL.receiver(), buzzer))
        .unwrap();
    spawner
        .spawn(adc_task(
            ADC_REQ_CHANNEL.receiver(),
            ADC_CHANNEL.sender(),
            adc,
        ))
        .unwrap();
    let buzzer_device = ChanneledBuzzer {
        sender: BUZZER_CHANNEL.sender(),
    };
    let mut app = App::new(display, board, buzzer_device).await;
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
                app.input_key_event(key).await.unwrap();
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
        // Timer::after_millis(5).await;
        Timer::after_micros(10).await;
    }
}
