use num_enum::IntoPrimitive;

use super::{unit::{TimeScale, VoltageScale}, ChannelInfo, Panel, ProbeChannel, Window};

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

#[derive(IntoPrimitive, Debug, Clone, Copy, PartialEq, PartialOrd)]
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
    SettingValueTitle,
    SettingValueContent,
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
    // TODO: waveform data
    pub waveform: u8,
    pub time_scale_ns: u64,
    // channel settings
    pub channel_info: [ChannelInfo; ProbeChannel::Endding as usize],
    pub channel_current: ProbeChannel,
    // TODO: measures
    pub measures: u64,
    // TODO: waveform generator setting
    pub generator: u64,

    // used in setting value window
    pub setting_index: u8,
    pub setting_inited: bool,
    pub setting_time_scale: TimeScale,
    pub setting_voltage_scale: VoltageScale,
    pub setting_select_idx: [u8; 3],
    pub setting_select_col: u8,
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
            channel_info: core::array::from_fn(|_| Default::default()),
            channel_current: Default::default(),
            measures: Default::default(),
            generator: Default::default(),
            setting_index: Default::default(),
            setting_inited: Default::default(),
            setting_time_scale: Default::default(),
            setting_voltage_scale: Default::default(),
            setting_select_idx: Default::default(),
            setting_select_col: Default::default(),
        }
    }
}

