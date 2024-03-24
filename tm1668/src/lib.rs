#![no_std]

use core::convert::Infallible;
use defmt::*;
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

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

    pub fn read_keys(&mut self, data: &mut [u8; 5]) {
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

    pub fn write(&mut self, data: u8) {
        self.dio.set_output();
        self.send_byte(data);
    }

    pub fn read(&mut self, data: &mut [u8; 5]) {
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
                info!("read 1 at bit {}", i);
                data |= 1 << i;
            }
            self.clk.set_high().unwrap();
            // self.delay.delay_us(1);
        }
        data
    }
}
