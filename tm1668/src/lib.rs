#![no_std]

use core::convert::Infallible;
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

/// TM1668
/// Key Map 4x5
/// F1 [13]   F2 [12]   F3 [1]     F4    [0]
/// 1  [11]    2 [10]    3 [3]     Lock  [2]
/// 4  [15]    5 [14]    6 [5]     X     [4]
/// 7  [17]    8 [16]    9 [7]     <     [6]
/// *  [19]    0 [18]    # [9]     OK    [8]

const KEY_MAP: [&str; 20] = [
    "F4", "F3", "Lock", "3", "X", "6", "<", "9", "OK", "#", "2", "1", "F2", "F1", "5", "4", "8", "7",
    "0", "*",
];

pub trait InoutPin {
    fn set_input(&mut self);
    fn set_output(&mut self);
    fn set_high(&mut self);
    fn set_low(&mut self);
    fn is_high(&self) -> bool;
    fn is_low(&self) -> bool;
}

pub struct TM1668<
    'd,
    S: OutputPin<Error = Infallible>,
    C: OutputPin<Error = Infallible>,
    D: InoutPin,
    DELAY,
> {
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
    DELAY: DelayUs<u16>,
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
