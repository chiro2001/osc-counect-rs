#![no_std]
#![allow(unused_imports)]

extern crate alloc;

#[cfg(feature = "lvgl")]
pub mod group;

use alloc::boxed::Box;
use core::{convert::Infallible, mem::MaybeUninit};
use defmt::*;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
#[cfg(feature = "lvgl")]
use lvgl::{
    input_device::{pointer::PointerInputData, BufferStatus, Data, InputDriver},
    LvError,
};

/// TM1668
/// Key Map 4x5
/// F1 [13]   F2 [12]   F3 [1]     F4    [0]
/// 1  [11]    2 [10]    3 [3]     Lock  [2]
/// 4  [15]    5 [14]    6 [5]     X     [4]
/// 7  [17]    8 [16]    9 [7]     <     [6]
/// *  [19]    0 [18]    # [9]     OK    [8]

const KEY_MAP: [&str; 20] = [
    "F4", "F3", "Lock", "3", "X", "6", "<", "9", "OK", "#", "2", "1", "F2", "F1", "5", "4", "8",
    "7", "0", "*",
];

pub trait InoutPin {
    fn set_input(&mut self);
    fn set_output(&mut self);
    fn set_high(&mut self);
    fn set_low(&mut self);
    fn is_high(&self) -> bool;
    fn is_low(&self) -> bool;
}

pub struct TM1668<'d, S, C, D, DELAY> {
    stb: S,
    clk: C,
    dio: D,
    delay: &'d mut DELAY,
}

impl<'d, S, C, D, DELAY> TM1668<'d, S, C, D, DELAY>
where
    S: OutputPin<Error = Infallible>,
    C: OutputPin<Error = Infallible>,
    D: InoutPin,
    DELAY: DelayNs,
{
    pub fn new(stb: S, clk: C, dio: D, delay: &'d mut DELAY) -> Self {
        Self {
            stb,
            clk,
            dio,
            delay,
        }
    }

    fn read_keys(&mut self, data: &mut [u8; 5]) {
        self.stb.set_low().unwrap();
        // self.delay.delay_us(1);
        // command: scan key and read
        self.write(0b0100_0010);
        self.dio.set_input();
        // delay
        self.delay.delay_us(1);
        self.read(data);
        self.stb.set_high().unwrap();
    }

    /// decoded: { KS1_K1, KS1_K2, KS2_K1, KS2_k2... }
    pub fn read_decode_keys(&mut self, decoded: &mut [bool; 20]) {
        let mut data = [0u8; 5];
        self.read_keys(&mut data);
        for i in 0..5 {
            let d = data[i];
            let k11 = d & (1 << 0) != 0;
            let k21 = d & (1 << 1) != 0;
            let k12 = d & (1 << 3) != 0;
            let k22 = d & (1 << 4) != 0;
            decoded[i * 4 + 0] = k11;
            decoded[i * 4 + 1] = k21;
            decoded[i * 4 + 2] = k12;
            decoded[i * 4 + 3] = k22;
        }
    }

    pub fn code_to_key(&self, code: usize) -> &'static str {
        if code as usize >= KEY_MAP.len() {
            return "";
        }
        KEY_MAP[code]
    }

    fn write(&mut self, data: u8) {
        self.dio.set_output();
        self.send_byte(data);
    }

    fn read(&mut self, data: &mut [u8; 5]) {
        self.dio.set_input();
        // self.delay.delay_us(2);
        for k in 0..5 {
            data[k] = self.read_byte();
        }
        self.dio.set_output();
    }

    fn send_byte(&mut self, data: u8) {
        for i in 0..8 {
            self.clk.set_low().unwrap();
            // LSB
            if data & (1 << i) != 0 {
                self.dio.set_high();
            } else {
                self.dio.set_low();
            }
            self.delay.delay_us(1);
            self.clk.set_high().unwrap();
            // self.delay.delay_us(1);
        }
        // self.dio.set_low();
    }

    fn read_byte(&mut self) -> u8 {
        let mut data = 0u8;
        for i in 0..8 {
            self.clk.set_low().unwrap();
            self.delay.delay_us(1);
            if self.dio.is_high() {
                data |= 1 << i;
            }
            self.clk.set_high().unwrap();
            // self.delay.delay_us(1);
        }
        data
    }
}

#[cfg(feature = "lvgl")]
pub struct KeypadDriver {
    pub(crate) driver: Box<lvgl_sys::lv_indev_drv_t>,
    pub(crate) descriptor: Option<*mut lvgl_sys::lv_indev_t>,
}

#[cfg(feature = "lvgl")]
unsafe extern "C" fn read_input<F>(
    indev_drv: *mut lvgl_sys::lv_indev_drv_t,
    data: *mut lvgl_sys::lv_indev_data_t,
) where
    F: Fn() -> BufferStatus,
{
    // convert user data to function
    let user_closure = &mut *((*indev_drv).user_data as *mut F);
    // call user data
    let info: BufferStatus = user_closure();
    (*data).continue_reading = match info {
        BufferStatus::Once(input) => {
            let (valid, pressed, key) = match input {
                lvgl::input_device::InputState::Released(Data::Pointer(PointerInputData::Key(
                    key,
                ))) => {
                    if key < 20 {
                        info!("Released key {} code {}", key, KEY_MAP[key as usize]);
                        (true, false, key)
                    } else {
                        (false, false, 0)
                    }
                }
                lvgl::input_device::InputState::Pressed(Data::Pointer(PointerInputData::Key(
                    key,
                ))) => {
                    if key < 20 {
                        info!("Pressed key {} code {}", key, KEY_MAP[key as usize]);
                        (true, true, key)
                    } else {
                        (false, false, 0)
                    }
                }
                _ => (false, false, 0),
            };
            if valid && key < 20 {
                let code = KEY_MAP[key as usize];
                let act_key = match code {
                    "F1" => Some(lvgl_sys::LV_KEY_PREV),
                    "F2" => Some(lvgl_sys::LV_KEY_UP),
                    "F3" => Some(lvgl_sys::LV_KEY_DOWN),
                    "F4" => Some(lvgl_sys::LV_KEY_NEXT),
                    "OK" => Some(lvgl_sys::LV_KEY_ENTER),
                    _ => None,
                };
                if let Some(k) = act_key {
                    (*data).key = k;
                    (*data).state = if pressed {
                        lvgl_sys::lv_indev_state_t_LV_INDEV_STATE_PRESSED
                    } else {
                        lvgl_sys::lv_indev_state_t_LV_INDEV_STATE_RELEASED
                    };
                } else {
                    warn!("Invalid key {} code {}", key, code);
                }
            }
            false
        }
        BufferStatus::Buffered(_input) => {
            info!("Buffered");
            true
        }
    };
}

#[cfg(feature = "lvgl")]
impl KeypadDriver {
    pub fn get_descriptor(&self) -> Option<*mut lvgl_sys::lv_indev_t> {
        self.descriptor
    }
}

#[cfg(feature = "lvgl")]
impl InputDriver<KeypadDriver> for KeypadDriver {
    fn register<F>(handler: F, _display: &lvgl::Display) -> lvgl::LvResult<Self>
    where
        F: Fn() -> lvgl::input_device::BufferStatus,
    {
        let driver = unsafe {
            let mut indev_drv = MaybeUninit::uninit();
            lvgl_sys::lv_indev_drv_init(indev_drv.as_mut_ptr());
            let mut indev_drv = Box::new(indev_drv.assume_init());
            indev_drv.type_ = lvgl_sys::lv_indev_type_t_LV_INDEV_TYPE_KEYPAD;
            indev_drv.read_cb = Some(read_input::<F>);
            indev_drv.user_data = Box::into_raw(Box::new(handler)) as *mut _;
            indev_drv
        };
        let mut dev = Self {
            driver,
            descriptor: None,
        };

        match unsafe {
            let descr = lvgl_sys::lv_indev_drv_register(dev.get_driver() as *mut _);
            if descr.is_null() {
                return Err(LvError::LvOOMemory);
            }
            dev.set_descriptor(descr)
        } {
            Ok(()) => Ok(dev),
            Err(e) => Err(e),
        }
    }

    fn get_driver(&mut self) -> &mut lvgl_sys::lv_indev_drv_t {
        self.driver.as_mut()
    }

    unsafe fn new_raw(
        read_cb: Option<
            unsafe extern "C" fn(*mut lvgl_sys::lv_indev_drv_t, *mut lvgl_sys::lv_indev_data_t),
        >,
        feedback_cb: Option<unsafe extern "C" fn(*mut lvgl_sys::lv_indev_drv_t, u8)>,
        _display: &lvgl::Display,
    ) -> lvgl::LvResult<KeypadDriver> {
        let driver = unsafe {
            let mut indev_drv = MaybeUninit::uninit();
            lvgl_sys::lv_indev_drv_init(indev_drv.as_mut_ptr());
            let mut indev_drv = Box::new(indev_drv.assume_init());
            indev_drv.type_ = lvgl_sys::lv_indev_type_t_LV_INDEV_TYPE_KEYPAD;
            indev_drv.read_cb = read_cb;
            indev_drv.feedback_cb = feedback_cb;
            indev_drv
        };
        let mut dev = Self {
            driver,
            descriptor: None,
        };

        match unsafe {
            let descr = lvgl_sys::lv_indev_drv_register(dev.get_driver() as *mut _);
            if descr.is_null() {
                return Err(LvError::LvOOMemory);
            }
            dev.set_descriptor(descr)
        } {
            Ok(()) => Ok(dev),
            Err(e) => Err(e),
        }
    }

    unsafe fn set_descriptor(
        &mut self,
        descriptor: *mut lvgl_sys::lv_indev_t,
    ) -> lvgl::LvResult<()> {
        if self.descriptor.is_none() {
            self.descriptor = Some(descriptor);
        } else {
            return Err(LvError::AlreadyInUse);
        }
        Ok(())
    }
}
