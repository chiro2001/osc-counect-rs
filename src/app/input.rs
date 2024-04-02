use num_enum::{IntoPrimitive, TryFromPrimitive};

/// TM1668
/// Key Map 4x5
/// F1 [13]   F2 [12]   F3 [1]     F4    [0]
/// 1  [11]    2 [10]    3 [3]     Lock  [2]
/// 4  [15]    5 [14]    6 [5]     X     [4]
/// 7  [17]    8 [16]    9 [7]     <     [6]
/// *  [19]    0 [18]    # [9]     OK    [8]

#[derive(IntoPrimitive, TryFromPrimitive, Clone, Copy, PartialEq, PartialOrd, Debug)]
#[repr(usize)]
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

pub trait KeyboardDevice {
    fn read_key(&mut self) -> Keys;
}

#[cfg(feature = "simulator")]
impl From<embedded_graphics_simulator::sdl2::Keycode> for Keys {
    fn from(value: Keycode) -> Self {
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
