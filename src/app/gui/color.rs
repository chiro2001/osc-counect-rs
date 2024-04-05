use embedded_graphics::pixelcolor::{
    raw::{RawData, RawU1, RawU16, RawU2},
    GrayColor, PixelColor, Rgb565, WebColors,
};
use embedded_graphics::prelude::RgbColor;

pub const COLOR_HALF_YELLOW: Rgb565 = Rgb565::new(Rgb565::MAX_R / 2, Rgb565::MAX_G / 2, 0);
pub const COLOR_HALF_GREEN: Rgb565 = Rgb565::new(0, Rgb565::MAX_G / 2, 0);
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

pub type WaveformColorRaw = RawU2;
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct WaveformColor(WaveformColorRaw);
pub type WaveformColorExRaw = RawU1;
#[cfg(not(feature = "waveform_3bit"))]
pub type WaveformColorEx = WaveformColor;
#[cfg(feature = "waveform_3bit")]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct WaveformColorEx(embedded_graphics::pixelcolor::BinaryColor);
#[cfg(feature = "waveform_3bit")]
impl WaveformColorEx {
    pub const fn new(luma: u8) -> Self {
        Self(if luma != 0 {
            embedded_graphics::pixelcolor::BinaryColor::On
        } else {
            embedded_graphics::pixelcolor::BinaryColor::Off
        })
    }
}
#[cfg(feature = "waveform_3bit")]
impl PixelColor for WaveformColorEx {
    type Raw = WaveformColorExRaw;
}
#[cfg(feature = "waveform_3bit")]
impl From<u8> for WaveformColorEx {
    fn from(data: u8) -> Self {
        Self::new(data)
    }
}
#[cfg(feature = "waveform_3bit")]
impl From<RawU1> for WaveformColorEx {
    fn from(data: RawU1) -> Self {
        Self::new(data.into_inner())
    }
}
#[cfg(feature = "waveform_3bit")]
impl From<WaveformColorEx> for RawU1 {
    fn from(color: WaveformColorEx) -> Self {
        use embedded_graphics::pixelcolor::IntoStorage;
        RawU1::new(color.0.into_storage())
    }
}

impl WaveformColor {
    pub const fn new(luma: u8) -> Self {
        Self(WaveformColorRaw::new(luma))
    }
}
impl PixelColor for WaveformColor {
    type Raw = WaveformColorRaw;
}
impl GrayColor for WaveformColor {
    fn luma(&self) -> u8 {
        self.0.into_inner()
    }
    const BLACK: Self = Self::new(0);
    const WHITE: Self = Self::new(1);
}
impl From<WaveformColorRaw> for WaveformColor {
    fn from(data: WaveformColorRaw) -> Self {
        Self(data)
    }
}
impl From<WaveformColor> for WaveformColorRaw {
    fn from(color: WaveformColor) -> Self {
        color.0
    }
}
impl From<u8> for WaveformColor {
    fn from(data: u8) -> Self {
        Self::new(data)
    }
}
impl From<WaveformColor> for Rgb565 {
    fn from(color: WaveformColor) -> Self {
        let luma = color.luma();
        GUI_COLOR_LUT_4[luma as usize]
    }
}

type WaveformCombinedColorRaw = RawU16;
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct WaveformCombinedColor(WaveformColorRaw, WaveformColorExRaw);
impl WaveformCombinedColor {
    pub fn new(color: u8, color_ex: u8) -> Self {
        Self(color.into(), color_ex.into())
    }
}
impl PixelColor for WaveformCombinedColor {
    type Raw = WaveformCombinedColorRaw;
}
impl From<WaveformCombinedColorRaw> for WaveformCombinedColor {
    fn from(data: WaveformCombinedColorRaw) -> Self {
        Self(
            (((data.into_inner() >> 8) & 0xff) as u8).into(),
            ((data.into_inner() & 0xff) as u8).into(),
        )
    }
}
impl From<WaveformCombinedColor> for WaveformCombinedColorRaw {
    fn from(color: WaveformCombinedColor) -> Self {
        RawU16::new((color.0.into_inner() as u16) << 8 | color.1.into_inner() as u16)
    }
}
impl From<WaveformCombinedColor> for Rgb565 {
    fn from(color: WaveformCombinedColor) -> Self {
        let (color, color_ex) = (color.0.into_inner(), color.1.into_inner());
        if color_ex != 0 {
            GUI_COLOR_LUT_4[1]
        } else {
            GUI_COLOR_LUT_4[color as usize]
        }
    }
}

pub const fn waveform_color(r: u8) -> WaveformColor {
    WaveformColor::new(r)
}