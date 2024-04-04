use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use num_enum::{FromPrimitive, IntoPrimitive};

pub type Result<T, E = AppError> = core::result::Result<T, E>;
#[derive(Debug)]
pub enum AppError {
    DisplayError,
    DataFormatError,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum Window {
    #[default]
    Main,
    SetValue,
    Settings,
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

#[derive(IntoPrimitive, Debug, Default, PartialEq, PartialOrd, Clone, Copy)]
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
    pub fn color(&self) -> Rgb565 {
        match self {
            ProbeChannel::A => Rgb565::YELLOW,
            ProbeChannel::B => Rgb565::GREEN,
            _ => Rgb565::RED,
        }
    }
}
