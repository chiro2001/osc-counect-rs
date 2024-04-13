use num_enum::{FromPrimitive, IntoPrimitive};

use super::{gui_color, waveform_color, GuiColor, VoltageUnit, WaveformColor};

pub type Result<T, E = AppError> = core::result::Result<T, E>;
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AppError {
    DisplayError,
    DataFormatError,
    NotImplemented,
    Unexpected,
    LimitExceeded,
    StorageIOError,
    StateFormatError,
}
impl AppError {
    pub fn str(&self) -> &'static str {
        match self {
            AppError::DisplayError => "Display Error",
            AppError::DataFormatError => "Data Format Error",
            AppError::NotImplemented => "Not Implemented",
            AppError::Unexpected => "Unexpected",
            AppError::LimitExceeded => "Limit Exceeded",
            AppError::StorageIOError => "Storage IO Error",
            AppError::StateFormatError => "State Format Error",
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum Window {
    #[default]
    Main,
    SetValue,
    Settings,
    MusicBoard,
}

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

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum SettingValueMode {
    ItemSelect,
    TextInput,
    Invalid,
}

#[derive(FromPrimitive, IntoPrimitive, Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
#[repr(usize)]
pub enum Panel {
    #[default]
    Channel = 0,
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
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelStyle {
    #[default]
    Normal,
    ChannelColor,
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
    pub fn select_items(&self) -> Option<&'static [&'static [&'static str]]> {
        match self {
            Panel::Channel => Some(&[&["CHA", "CHB"]]),
            Panel::VoltageScale => Some(&[
                &["1", "2", "5", "10", "20", "50", "100", "200", "500"],
                &["mV", "V"],
            ]),
            _ => None,
        }
    }
    pub fn get_setting_mode(&self) -> SettingValueMode {
        match self {
            Panel::Channel => SettingValueMode::ItemSelect,
            Panel::VoltageScale => SettingValueMode::ItemSelect,
            _ => SettingValueMode::Invalid,
        }
    }
    pub fn str_short(&self) -> &'static str {
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
            Panel::Endding => "--",
        }
    }
    pub fn str(&self) -> &'static str {
        match self {
            Panel::Channel => "Channel",
            Panel::TimeScale => "Time Scale",
            Panel::VoltageScale => "Voltage Scale",
            Panel::Xpos => "X pos offset",
            Panel::Ypos => "Y pos offset",
            Panel::TrigerLevel => "Triger Level",
            Panel::Couple => "Couple",
            Panel::TrigerType => "Triger Type",
            Panel::Probe => "Probe",
            Panel::HorMeasure1 => "Hor Measure 1",
            Panel::VerMeasure1 => "Ver Measure 1",
            Panel::HorMeasure2 => "Hor Measure 2",
            Panel::VerMeasure2 => "Ver Measure 2",
            Panel::Sweep => "Sweep",
            Panel::Endding => "--",
        }
    }
}

impl Into<&'static str> for Panel {
    fn into(self) -> &'static str {
        self.str_short()
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum Coupling {
    #[default]
    AC,
    DC,
    OFF,
}

#[derive(Debug)]
pub struct ChannelInfo {
    pub voltage_scale_mv: u64,
    pub coupling: Coupling,
    pub probe: u8,
}

impl Default for ChannelInfo {
    fn default() -> Self {
        Self {
            voltage_scale_mv: 500,
            coupling: Default::default(),
            probe: 1,
        }
    }
}

#[derive(IntoPrimitive, FromPrimitive, Debug, Default, PartialEq, PartialOrd, Clone, Copy)]
#[repr(usize)]
pub enum ProbeChannel {
    #[default]
    A,
    B,
    Endding,
}
impl Into<&'static str> for ProbeChannel {
    fn into(self) -> &'static str {
        match self {
            ProbeChannel::A => "CHA",
            ProbeChannel::B => "CHB",
            _ => "CH?",
        }
    }
}
impl ProbeChannel {
    pub fn color(&self) -> GuiColor {
        match self {
            ProbeChannel::A => gui_color(2),
            ProbeChannel::B => gui_color(3),
            _ => gui_color(4),
        }
    }
    pub fn color_waveform(&self) -> WaveformColor {
        match self {
            ProbeChannel::A => waveform_color(2),
            ProbeChannel::B => waveform_color(3),
            _ => waveform_color(0),
        }
    }
}

#[cfg(feature = "embedded")]
pub const WAVEFORM_LEN: usize = 64;
#[cfg(not(feature = "embedded"))]
pub const WAVEFORM_LEN: usize = 128;
pub const WAVEFORM_HISTORY_LEN: usize = 5;
#[derive(Debug)]
pub struct WaveformStorage {
    pub linked: heapless::Deque<(bool, usize), WAVEFORM_HISTORY_LEN>,
    // when in rolling mode, only draw data[0]
    pub data: [[f32; WAVEFORM_LEN]; WAVEFORM_HISTORY_LEN],
    pub offset: [i32; WAVEFORM_HISTORY_LEN],
    pub rolling_offset: usize,
    pub len: usize,
    pub unit: VoltageUnit,
}

impl Default for WaveformStorage {
    fn default() -> Self {
        let mut linked = heapless::Deque::new();
        for i in 0..WAVEFORM_HISTORY_LEN {
            linked.push_back((false, i)).unwrap();
        }
        Self {
            linked,
            data: core::array::from_fn(|_| [0.0; WAVEFORM_LEN]),
            offset: Default::default(),
            rolling_offset: 0,
            len: WAVEFORM_LEN,
            unit: VoltageUnit::MilliVolt,
        }
    }
}

impl WaveformStorage {
    pub fn clear(&mut self) {
        self.linked.clear();
        for i in 0..WAVEFORM_HISTORY_LEN {
            self.linked.push_back((false, i)).unwrap();
        }
        self.data.iter_mut().for_each(|x| x.fill(0.0));
        self.offset.fill(0);
        self.rolling_offset = 0;
    }
    pub fn append_frame(&mut self, data: &[f32], offset: i32) -> Result<()> {
        self.append_frame_iter(data.iter().cloned(), offset)
    }
    pub fn append_frame_iter<I>(&mut self, data: I, offset: i32) -> Result<()>
    where
        I: IntoIterator<Item = f32>,
    {
        // find a free slot, if full, auto drop
        let back = self.linked.pop_back().ok_or(AppError::Unexpected)?;
        let (_, slot) = back;
        // copy data
        self.data[slot]
            .iter_mut()
            .zip(data.into_iter())
            .for_each(|(x, y)| *x = y);
        self.offset[slot] = offset;
        // link data
        self.linked
            .push_front((true, slot))
            .map_err(|_| AppError::LimitExceeded)?;
        Ok(())
    }
    pub fn pop(&mut self) -> Option<&[f32]> {
        let slot = self.linked.pop_front().and_then(
            |(avaliable, x)| {
                if avaliable {
                    Some(x)
                } else {
                    None
                }
            },
        );
        if let Some(slot) = slot {
            self.linked.push_back((false, slot)).unwrap();
            Some(&self.data[slot])
        } else {
            None
        }
    }
    pub fn append_rolling_data(&mut self, data: &[f32]) -> Result<()> {
        if data.len() > WAVEFORM_LEN {
            // copy to data[0] start
            self.data[0].copy_from_slice(&data[..WAVEFORM_LEN]);
            self.rolling_offset = 0;
        } else {
            // append to ring buf self.data
            let offset = self.rolling_offset as usize;
            if offset + data.len() <= WAVEFORM_LEN {
                self.data[0][offset..offset + data.len()].copy_from_slice(data);
                self.rolling_offset += data.len();
            } else {
                let len = WAVEFORM_LEN - offset;
                self.data[0][offset..].copy_from_slice(&data[..len]);
                self.data[0][..data.len() - len].copy_from_slice(&data[len..]);
                self.rolling_offset = data.len() - len;
            }
            // self.rolling_offset = (offset + data.len()) % WAVEFORM_LEN;
        }
        Ok(())
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimebaseMode {
    #[default]
    Normal,
    Rolling,
    XY,
}
impl TimebaseMode {
    pub fn str(&self) -> &'static str {
        match self {
            TimebaseMode::Normal => "Normal",
            TimebaseMode::Rolling => "Rolling",
            TimebaseMode::XY => "XY",
        }
    }
}
