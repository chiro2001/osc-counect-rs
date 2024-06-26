use embedded_graphics::pixelcolor::{
    raw::{RawData, RawU1, RawU16},
    WebColors,
};
use embedded_graphics::prelude::RgbColor;

// pub type GuiColor = Gray4;
#[cfg(feature = "color-bgr565")]
pub type GuiColor = embedded_graphics::pixelcolor::Bgr565;
#[cfg(feature = "color-rgb565")]
pub type GuiColor = embedded_graphics::pixelcolor::Rgb565;
#[cfg(feature = "color-binary")]
pub type GuiColor = embedded_graphics::pixelcolor::BinaryColor;

#[cfg(any(feature = "color-rgb565", feature = "color-bgr565"))]
mod colors {
    use super::*;
    pub const COLOR_HALF_YELLOW: GuiColor =
        GuiColor::new(GuiColor::MAX_R / 2, GuiColor::MAX_G / 2, 0);
    pub const COLOR_HALF_GREEN: GuiColor = GuiColor::new(0, GuiColor::MAX_G / 2, 0);
    pub const GUI_COLOR_LUT_16: [GuiColor; 16] = [
        GuiColor::BLACK,               // 0
        GuiColor::CSS_DARK_SLATE_GRAY, // 1
        GuiColor::YELLOW,              // 2
        GuiColor::GREEN,               // 3
        GuiColor::RED,                 // 4
        GuiColor::MAGENTA,             // 5
        GuiColor::CYAN,                // 6
        GuiColor::CSS_LIGHT_GRAY,      // 7
        GuiColor::CSS_PURPLE,          // 8
        GuiColor::CSS_ORANGE_RED,      // 9
        GuiColor::CSS_DARK_RED,        // 10
        COLOR_HALF_YELLOW,             // 11
        COLOR_HALF_GREEN,              // 12
        GuiColor::WHITE,               // 13
        GuiColor::WHITE,               // 14
        GuiColor::WHITE,               // 15
    ];
    pub const GUI_COLOR_LUT_4: [GuiColor; 4] = [
        GuiColor::BLACK,          // 0
        GuiColor::CSS_LIGHT_GRAY, // 1
        GuiColor::YELLOW,         // 2
        GuiColor::GREEN,          // 3
    ];
}
#[cfg(not(feature = "color-binary"))]
pub use colors::*;
// pub const fn gui_color(r: u8) -> GuiColor {
//     Gray4::new(r)
// }
pub type GuiColorRaw = RawU16;
#[cfg(not(feature = "color-binary"))]
pub const fn gui_color(r: u16) -> GuiColor {
    GUI_COLOR_LUT_16[r as usize]
}
#[cfg(feature = "color-binary")]
pub const fn gui_color(r: u16) -> GuiColor {
    // if r == 0 {
    //     GuiColor::Off
    // } else {
    //     GuiColor::On
    // }
    match r {
        0 | 1 | 4 | 5 | 6 | 7 | 8 | 9 | 10 => GuiColor::Off,
        _ => GuiColor::On,
    }
}
pub const GUI_BG_COLOR: GuiColor = gui_color(1);

#[cfg(not(any(feature = "waveform-16bit", feature = "waveform-1bit")))]
pub type WaveformColorRaw = embedded_graphics_core::pixelcolor::raw::RawU2;
#[cfg(feature = "waveform-1bit")]
pub type WaveformColorRaw = embedded_graphics_core::pixelcolor::raw::RawU1;
#[cfg(feature = "waveform-16bit")]
pub type WaveformColorRaw = embedded_graphics_core::pixelcolor::raw::RawU16;
#[cfg(feature = "waveform-16bit")]
pub type WaveformColor = GuiColor;
#[cfg(not(feature = "waveform-16bit"))]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct WaveformColor(WaveformColorRaw);
pub type WaveformColorExRaw = RawU1;
#[cfg(not(feature = "waveform-3bit"))]
pub type WaveformColorEx = WaveformColor;
#[cfg(feature = "waveform-3bit")]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default)]
pub struct WaveformColorEx(embedded_graphics::pixelcolor::BinaryColor);

#[cfg(feature = "waveform-16bit")]
pub fn waveform_color_ex(r: u8) -> WaveformColorEx {
    let r = match r {
        1 => 7,
        r => r,
    } as usize;
    GUI_COLOR_LUT_16[r].into()
}
#[cfg(not(feature = "waveform-16bit"))]
pub fn waveform_color_ex(r: u8) -> WaveformColorEx {
    WaveformColorEx::new(r)
}

#[cfg(feature = "waveform-3bit")]
impl WaveformColorEx {
    pub const fn new(luma: u8) -> Self {
        Self(if luma != 0 {
            embedded_graphics::pixelcolor::BinaryColor::On
        } else {
            embedded_graphics::pixelcolor::BinaryColor::Off
        })
    }
}
#[cfg(feature = "waveform-3bit")]
impl embedded_graphics::pixelcolor::PixelColor for WaveformColorEx {
    type Raw = WaveformColorExRaw;
}
#[cfg(feature = "waveform-3bit")]
impl From<u8> for WaveformColorEx {
    fn from(data: u8) -> Self {
        Self::new(data)
    }
}
#[cfg(feature = "waveform-3bit")]
impl From<RawU1> for WaveformColorEx {
    fn from(data: RawU1) -> Self {
        Self::new(data.into_inner())
    }
}
#[cfg(feature = "waveform-3bit")]
impl From<WaveformColorEx> for RawU1 {
    fn from(color: WaveformColorEx) -> Self {
        use embedded_graphics::pixelcolor::IntoStorage;
        RawU1::new(color.0.into_storage())
    }
}

#[cfg(not(feature = "waveform-16bit"))]
impl WaveformColor {
    pub const fn new(luma: u8) -> Self {
        Self(WaveformColorRaw::new(luma))
    }
}
#[cfg(not(feature = "waveform-16bit"))]
impl embedded_graphics::pixelcolor::PixelColor for WaveformColor {
    type Raw = WaveformColorRaw;
}
#[cfg(not(feature = "waveform-16bit"))]
impl embedded_graphics::pixelcolor::GrayColor for WaveformColor {
    fn luma(&self) -> u8 {
        use embedded_graphics_core::prelude::RawData;
        self.0.into_inner()
    }
    const BLACK: Self = Self::new(0);
    const WHITE: Self = Self::new(1);
}
#[cfg(not(feature = "waveform-16bit"))]
impl From<WaveformColorRaw> for WaveformColor {
    fn from(data: WaveformColorRaw) -> Self {
        Self(data)
    }
}
#[cfg(not(feature = "waveform-16bit"))]
impl From<WaveformColor> for WaveformColorRaw {
    fn from(color: WaveformColor) -> Self {
        color.0
    }
}
#[cfg(not(feature = "waveform-16bit"))]
impl From<u8> for WaveformColor {
    fn from(data: u8) -> Self {
        Self::new(data)
    }
}
#[cfg(not(any(feature = "waveform-16bit", feature = "color-binary")))]
impl From<WaveformColor> for GuiColor {
    fn from(color: WaveformColor) -> Self {
        use embedded_graphics_core::pixelcolor::GrayColor;
        let luma = color.luma();
        GUI_COLOR_LUT_4[luma as usize]
    }
}
#[cfg(feature = "color-binary")]
impl From<WaveformColor> for GuiColor {
    fn from(color: WaveformColor) -> Self {
        if color.0.into_inner() == 0 {
            GuiColor::Off
        } else {
            GuiColor::On
        }
    }
}

#[cfg(feature = "waveform-3bit")]
mod combined_color {
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
    impl From<WaveformCombinedColor> for GuiColor {
        fn from(color: WaveformCombinedColor) -> Self {
            let (color, color_ex) = (color.0.into_inner(), color.1.into_inner());
            if color_ex != 0 {
                GUI_COLOR_LUT_4[1]
            } else {
                GUI_COLOR_LUT_4[color as usize]
            }
        }
    }
}
#[cfg(feature = "waveform-3bit")]
pub use combined_color::*;

pub const fn waveform_color(r: u8) -> WaveformColor {
    #[cfg(not(any(feature = "waveform-16bit", feature = "waveform-1bit")))]
    let c = WaveformColor::new(r);
    #[cfg(feature = "waveform-1bit")]
    let c = WaveformColor::new(if r > 0 { 1 } else { 0 });
    #[cfg(feature = "waveform-16bit")]
    let c = GUI_COLOR_LUT_16[r as usize];
    c
}
