use embedded_graphics::pixelcolor::{raw::RawU16, Rgb565, RgbColor, WebColors};
use num_enum::{FromPrimitive, IntoPrimitive};

use super::{VoltageUnit, WaveformColor};

pub type Result<T, E = AppError> = core::result::Result<T, E>;
#[derive(Debug)]
pub enum AppError {
    DisplayError,
    DataFormatError,
    NotImplemented,
    Unexpected,
    LimitExceeded,
}

const COLOR_HALF_YELLOW: Rgb565 = Rgb565::new(Rgb565::MAX_R / 2, Rgb565::MAX_G / 2, 0);
const COLOR_HALF_GREEN: Rgb565 = Rgb565::new(0, Rgb565::MAX_G / 2, 0);
pub const GUI_COLOR_LUT_16: [Rgb565; 16] = [
    Rgb565::BLACK,               // 0
    Rgb565::CSS_DARK_SLATE_GRAY, // 1
    Rgb565::YELLOW,              // 2
    Rgb565::GREEN,               // 3
    Rgb565::RED,                 // 4
    Rgb565::MAGENTA,             // 5
    Rgb565::CYAN,                // 6
    Rgb565::CSS_LIGHT_GRAY,      // 7
    Rgb565::CSS_PURPLE,          // 8
    Rgb565::CSS_ORANGE_RED,      // 9
    Rgb565::CSS_DARK_RED,        // 10
    COLOR_HALF_YELLOW,           // 11
    COLOR_HALF_GREEN,            // 12
    Rgb565::WHITE,               // 13
    Rgb565::WHITE,               // 14
    Rgb565::WHITE,               // 15
];
pub const GUI_COLOR_LUT_4: [Rgb565; 4] = [
    Rgb565::BLACK,          // 0
    Rgb565::CSS_LIGHT_GRAY, // 1
    Rgb565::YELLOW,         // 2
    Rgb565::GREEN,          // 3
];
// pub type GuiColor = Gray4;
// pub const fn gui_color(r: u8) -> GuiColor {
//     Gray4::new(r)
// }
pub type GuiColor = Rgb565;
pub type GuiColorRaw = RawU16;
pub const fn gui_color(r: u16) -> GuiColor {
    GUI_COLOR_LUT_16[r as usize]
}
pub const GUI_BG_COLOR: GuiColor = gui_color(1);

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
            ProbeChannel::A => WaveformColor::new(2),
            ProbeChannel::B => WaveformColor::new(3),
            _ => WaveformColor::new(0),
        }
    }
}

pub const WAVEFORM_LEN: usize = 16;
pub const WAVEFORM_HISTORY_LEN: usize = 10;
#[derive(Debug)]
pub struct WaveformStorage {
    pub linked: heapless::Deque<(bool, usize), WAVEFORM_HISTORY_LEN>,
    pub data: [[f32; WAVEFORM_LEN]; WAVEFORM_HISTORY_LEN],
    pub offset: [i32; WAVEFORM_HISTORY_LEN],
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
            len: WAVEFORM_LEN,
            unit: VoltageUnit::MilliVolt,
        }
    }
}

impl WaveformStorage {
    pub fn clear(&mut self) {
        self.linked.clear();
    }
    pub fn append(&mut self, data: &[f32], offset: i32) -> Result<()> {
        self.append_iter(data.iter().cloned(), offset)
    }
    pub fn append_iter<I>(&mut self, data: I, offset: i32) -> Result<()>
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
    // pub fn iter<'a, T>(&'a self) -> IterWaveform<'a, T> {
    //     let it = self.linked.iter();
    //     IterWaveform { it, parent: self }
    // }
}

// pub struct IterWaveform<'a, T> {
//     it: T,
//     parent: &'a WaveformStorage,
// }

// impl<'a, T> Iterator for IterWaveform<'a, T>
// where
//     T: Iterator<Item = &'a (bool, usize)>,
// {
//     type Item = &'a [f32];
//     fn next(&mut self) -> Option<Self::Item> {
//         let next = self.it.next();
//         self.it = next;
//         match next {
//             Some((_, x)) => Some(&self.parent.data[*x]),
//             None => None,
//         }
//     }
// }
