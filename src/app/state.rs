use core::mem::MaybeUninit;

use num_enum::IntoPrimitive;

use super::{
    unit::{TimeScale, VoltageScale},
    ChannelInfo, Panel, ProbeChannel, RunningState, TimebaseMode, WaveformStorage, Window,
};

#[derive(IntoPrimitive, Debug, Clone, Copy, PartialEq, PartialOrd)]
#[repr(usize)]
pub enum StateMarker {
    PanelPage,
    RunningState,
    Battery,
    Clock,
    Window,
    Waveform,
    WaveformData,
    TimeScale,
    ChannelSetting,
    Measures,
    Generator,
    TriggerLevel,
    SettingValueTitle,
    SettingValueContent,
    SettingsMenu,
    SettingsMenuMoveL1,
    SettingsMenuMoveL2,
    Endding,
    All,
    AllFlush,
}

#[derive(Debug)]
pub struct State {
    pub panel_page: u8,
    pub panel_focused: Option<Panel>,
    pub running_state: RunningState,
    pub battery: u8,
    pub clock: [char; 5],
    pub window: Window,
    pub window_next: Option<Window>,
    // waveform data
    pub waveform: &'static mut [WaveformStorage; ProbeChannel::Endding as usize],
    pub timebase_mode: TimebaseMode,
    pub time_scale_ns: u64,
    // channel settings
    pub channel_info: [ChannelInfo; ProbeChannel::Endding as usize],
    pub channel_current: ProbeChannel,
    // TODO: measures
    pub measures: u64,
    // TODO: waveform generator setting
    pub generator: u64,
    pub trigger_channel: ProbeChannel,
    pub trigger_level_mv: u64,

    // used in setting value window
    pub setting_index: u8,
    pub setting_inited: bool,
    pub setting_time_scale: TimeScale,
    pub setting_voltage_scale: VoltageScale,
    pub setting_select_idx: [u8; 3],
    pub setting_select_col: u8,

    // used in menu
    pub menu_idx_l1: usize,
    pub menu_idx_l1_last: Option<usize>,
    pub menu_idx_l2: Option<usize>,
    pub menu_idx_l2_last: Option<usize>,

    // settings
    pub backlight: u8,
    pub volume: u8,
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
    pub fn confirm(&mut self, index: StateMarker) {
        // crate::info!("confirm: {:?}", index);
        self.set(index, true);
    }
    pub fn request(&mut self, index: StateMarker) {
        // crate::info!("request: {:?}", index);
        self.set(index, false);
    }
    pub fn requests(&mut self, indexes: &[StateMarker]) {
        for index in indexes {
            self.request(*index);
        }
    }
}

// maybe not initialized
static mut WAVEFORM_DATA: MaybeUninit<[WaveformStorage; ProbeChannel::Endding as usize]> =
    MaybeUninit::uninit();

impl Default for State {
    fn default() -> Self {
        // initialize waveform data
        unsafe {
            for i in 0..ProbeChannel::Endding as usize {
                core::ptr::write(
                    &mut WAVEFORM_DATA.assume_init_mut()[i],
                    WaveformStorage::default(),
                );
            }
        }
        Self {
            panel_page: 0,
            panel_focused: Default::default(),
            running_state: Default::default(),
            battery: 50,
            clock: ['1', '2', ':', '4', '5'],
            window: Default::default(),
            window_next: Default::default(),
            waveform: unsafe { WAVEFORM_DATA.assume_init_mut() },
            timebase_mode: Default::default(),
            time_scale_ns: 100_000,
            channel_info: core::array::from_fn(|_| Default::default()),
            channel_current: Default::default(),
            measures: Default::default(),
            generator: Default::default(),
            trigger_channel: Default::default(),
            trigger_level_mv: 1000,
            setting_index: Default::default(),
            setting_inited: Default::default(),
            setting_time_scale: Default::default(),
            setting_voltage_scale: Default::default(),
            setting_select_idx: Default::default(),
            setting_select_col: Default::default(),
            menu_idx_l1: Default::default(),
            menu_idx_l1_last: Default::default(),
            menu_idx_l2: Default::default(),
            menu_idx_l2_last: Default::default(),
            backlight: 50,
            volume: 0,
        }
    }
}
