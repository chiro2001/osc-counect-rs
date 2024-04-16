use num_enum::{IntoPrimitive, TryFromPrimitive};

use super::ProbeChannel;
use super::Result;
use super::WAVEFORM_LEN;

/// TM1668
/// Key Map 4x5
/// F1 [13]   F2 [12]   F3 [1]     F4    [0]
/// 1  [11]    2 [10]    3 [3]     Lock  [2]
/// 4  [15]    5 [14]    6 [5]     X     [4]
/// 7  [17]    8 [16]    9 [7]     <     [6]
/// *  [19]    0 [18]    # [9]     OK    [8]

#[derive(IntoPrimitive, TryFromPrimitive, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[repr(usize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[rustfmt::skip]
pub enum Keys {
    None  = 30,
    Left  = 13,  Up   = 12,  Down  = 1,  Right = 0,
    Key1  = 11,  Key2 = 10,  Key3  = 3,  Lock  = 2,
    Key4  = 15,  Key5 = 14,  Key6  = 5,  X     = 4,
    Key7  = 17,  Key8 = 16,  Key9  = 7,  Back  = 6,
    Star  = 19,  Key0 = 18,  Sharp = 9,  Ok    = 8,
    Power = 31,
}
impl Keys {
    pub fn is_digit(&self) -> bool {
        match self {
            Keys::Key0
            | Keys::Key1
            | Keys::Key2
            | Keys::Key3
            | Keys::Key4
            | Keys::Key5
            | Keys::Key6
            | Keys::Key7
            | Keys::Key8
            | Keys::Key9 => true,
            _ => false,
        }
    }
    pub fn digital_value(&self) -> u8 {
        match self {
            Keys::Key0 => 0,
            Keys::Key1 => 1,
            Keys::Key2 => 2,
            Keys::Key3 => 3,
            Keys::Key4 => 4,
            Keys::Key5 => 5,
            Keys::Key6 => 6,
            Keys::Key7 => 7,
            Keys::Key8 => 8,
            Keys::Key9 => 9,
            _ => 0,
        }
    }
}
impl Into<&'static str> for Keys {
    fn into(self) -> &'static str {
        match self {
            Keys::None => "None",
            Keys::Left => "Left",
            Keys::Up => "Up",
            Keys::Down => "Down",
            Keys::Right => "Right",
            Keys::Key1 => "Key1",
            Keys::Key2 => "Key2",
            Keys::Key3 => "Key3",
            Keys::Lock => "Lock",
            Keys::Key4 => "Key4",
            Keys::Key5 => "Key5",
            Keys::Key6 => "Key6",
            Keys::X => "X",
            Keys::Key7 => "Key7",
            Keys::Key8 => "Key8",
            Keys::Key9 => "Key9",
            Keys::Back => "Back",
            Keys::Star => "Star",
            Keys::Key0 => "Key0",
            Keys::Sharp => "Sharp",
            Keys::Ok => "Ok",
            Keys::Power => "Power",
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputEvent {
    KeyPressed(Keys),
    KeyReleased(Keys),
    None,
}

pub trait KeyboardDevice {
    fn read_key(&mut self) -> Keys;
    fn read_key_event(&mut self) -> InputEvent {
        let key = self.read_key();
        match key {
            Keys::None => InputEvent::None,
            _ => InputEvent::KeyReleased(key),
        }
    }
}

pub struct DummyKeyboardDevice;
impl KeyboardDevice for DummyKeyboardDevice {
    fn read_key(&mut self) -> Keys {
        Keys::None
    }
}

#[cfg(feature = "simulator")]
impl From<embedded_graphics_simulator::sdl2::Keycode> for Keys {
    fn from(value: embedded_graphics_simulator::sdl2::Keycode) -> Self {
        use embedded_graphics_simulator::sdl2::Keycode;
        match value {
            Keycode::Backspace => Keys::Back,
            Keycode::Return => Keys::Ok,
            Keycode::Up => Keys::Up,
            Keycode::Down => Keys::Down,
            Keycode::Left => Keys::Left,
            Keycode::Right => Keys::Right,
            Keycode::Num1 => Keys::Key1,
            Keycode::Num2 => Keys::Key2,
            Keycode::Num3 => Keys::Key3,
            Keycode::Num4 => Keys::Key4,
            Keycode::Num5 => Keys::Key5,
            Keycode::Num6 => Keys::Key6,
            Keycode::Num7 => Keys::Key7,
            Keycode::Num8 => Keys::Key8,
            Keycode::Num9 => Keys::Key9,
            Keycode::Num0 => Keys::Key0,
            Keycode::Space => Keys::Lock,
            Keycode::X => Keys::X,
            // Not sure about these
            Keycode::Minus => Keys::Star,
            Keycode::Equals => Keys::Sharp,
            Keycode::Delete => Keys::Power,
            // Alternatives
            Keycode::F1 => Keys::Left,
            Keycode::F2 => Keys::Up,
            Keycode::F3 => Keys::Down,
            Keycode::F4 => Keys::Right,
            _ => Keys::None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AdcReadOptions {
    pub channel: ProbeChannel,
    pub length: usize,
    pub pos: usize,
    pub frequency: u64,
}
impl AdcReadOptions {
    pub fn new(channel: ProbeChannel, length: usize, frequency: u64) -> Self {
        Self {
            channel,
            length,
            pos: 0,
            frequency,
        }
    }
}
// pub const ADC_BUF_SZ: usize = 32;
pub const ADC_BUF_SZ: usize = WAVEFORM_LEN * 1;
pub trait AdcDevice {
    async fn read(&mut self, options: AdcReadOptions, buf: &mut [f32]) -> Result<usize>;
}

pub struct DummyAdcDevice;
impl AdcDevice for DummyAdcDevice {
    async fn read(&mut self, _options: AdcReadOptions, buf: &mut [f32]) -> Result<usize> {
        let count = buf
            .into_iter()
            .zip(
                [
                    -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
                    1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7,
                    2.8,
                ]
                .into_iter(),
            )
            .map(|(a, b)| *a = b as f32)
            .count();
        Ok(count)
    }
}

pub trait BuzzerDevice {
    async fn beep(&mut self, frequency: u32, duration_ms: u32);
    async fn beep_on(&mut self, frequency: u32) {
        self.beep(frequency, 0).await;
    }
    async fn beep_off(&mut self) {
        self.beep(0, 0).await;
    }
}

pub struct DummyBuzzerDevice;
impl BuzzerDevice for DummyBuzzerDevice {
    async fn beep(&mut self, _frequency: u32, _duration_ms: u32) {}
}

pub trait BoardDevice {
    /// Set the brightness of the backlight.
    /// The brightness is a value between 0 and 100.
    fn set_brightness(&mut self, brightness: u8);

    fn set_power_on(&mut self, _on: bool) {}
    fn read_power_key(&mut self) -> bool {
        false
    }

    fn has_keypad(&self) -> bool {
        false
    }

    fn has_clock(&self) -> bool {
        false
    }

    fn has_battery(&self) -> bool {
        false
    }
    fn get_battery_percentage(&self) -> u8 {
        0
    }
}

pub trait NvmDevice {
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<()>;
    fn write(&mut self, address: u32, buf: &[u8]) -> Result<()>;
    fn erase(&mut self, address: u32, len: usize) -> Result<()>;
}

pub struct DummyBoardDevice;
impl BoardDevice for DummyBoardDevice {
    fn set_brightness(&mut self, _brightness: u8) {}
}

impl NvmDevice for DummyBoardDevice {
    fn read(&mut self, _address: u32, _buf: &mut [u8]) -> Result<()> {
        Err(super::AppError::NotImplemented)
    }
    fn write(&mut self, _address: u32, _buf: &[u8]) -> Result<()> {
        Err(super::AppError::NotImplemented)
    }
    fn erase(&mut self, _address: u32, _len: usize) -> Result<()> {
        Err(super::AppError::NotImplemented)
    }
}

pub trait DisplayFlushable {
    fn do_display_flush(&mut self) -> Result<()> {
        Ok(())
    }
}
