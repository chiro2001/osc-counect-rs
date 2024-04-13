#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embedded_graphics::draw_target::DrawTargetExt;
use embedded_graphics::pixelcolor::{IntoStorage, Rgb565};
use embedded_graphics::{geometry::Point, pixelcolor::RgbColor};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    embassy,
    gpio::IO,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, LowSpeed, LEDC,
    },
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::TimerGroup,
};

use crate::app::devices::{
    DummyAdcDevice, DummyBoardDevice, DummyBuzzerDevice, DummyKeyboardDevice,
};

mod app;

/*
ST7789 <-> ESP32C3
VCC    <-> 3.3V
GND    <-> GND

SCK    <-> IO2
SDA    <-> IO3
RES    <-> IO10
DC     <-> IO6
CS     <-> IO7
BL     <-> IO11

BTN0   <-> IO4
BTN1   <-> IO5
BTN2   <-> IO8
BTN3   <-> IO9
BTN    <-> IO13
*/

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let res = io.pins.gpio10.into_push_pull_output();
    let dc = io.pins.gpio6.into_push_pull_output();
    let cs = io.pins.gpio7.into_push_pull_output();
    // espefuse.py -p /dev/ttyACM0 burn_efuse VDD_SPI_AS_GPIO 1
    let bl = io.pins.gpio11.into_push_pull_output();
    let bl2 = io.pins.gpio0.into_push_pull_output();

    let mut ledc = LEDC::new(peripherals.LEDC, &clocks);

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 24.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, bl);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 100,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    channel0.set_duty(15).unwrap();
    // channel0.start_duty_fade(0, 20, 200).unwrap();

    let spi = Spi::new(peripherals.SPI2, 80.MHz(), SpiMode::Mode0, &clocks)
        .with_sck(sclk)
        .with_mosi(mosi);
    let spi_mutex = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_mutex, cs);
    let interface = display_interface_spi::SPIInterface::new(spi_device, dc);
    let mut lcd = st7789::ST7789::new(interface, Some(res), Some(bl2), 160, 80);
    let mut delay = Delay::new(&clocks);
    lcd.init(&mut delay).unwrap();
    lcd.set_orientation(st7789::Orientation::Landscape).unwrap();
    lcd.set_pixels(
        0,
        0,
        160,
        80,
        core::iter::repeat(Rgb565::CYAN.into_storage()).take(160 * 80),
    )
    .unwrap();
    let display = lcd.translated(Point::new(1, 26));

    let adc_device = DummyAdcDevice {};
    let kbd_device = DummyKeyboardDevice {};
    let board = DummyBoardDevice {};
    let buzzer = DummyBuzzerDevice {};
    app::main_loop(
        spawner,
        display,
        board,
        buzzer,
        kbd_device,
        adc_device,
        |_| {},
    )
    .await;
    defmt::panic!("Simulator stopped");
}
