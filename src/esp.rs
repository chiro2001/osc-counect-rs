#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use app::devices::{BoardDevice, KeyboardDevice, Keys, NvmDevice};
use defmt::*;
use display_interface::DataFormat;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{CriticalSectionMutex, NoopMutex};
use embassy_time::Timer;
use embedded_graphics::draw_target::DrawTargetExt;
use embedded_graphics::pixelcolor::IntoStorage;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Drawable;
use embedded_graphics::{geometry::Point, pixelcolor::RgbColor};
use embedded_hal::digital::InputPin;
use esp_backtrace as _;
use esp_hal::clock::Clocks;
use esp_hal::gpio::OutputPin;
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::timer::TimerSpeed;
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
use gc9a01::display::{DisplayDefinition, NewZeroed};
use gc9a01::rotation::DisplayRotation;
use static_cell::make_static;

use rtt_target::rtt_init_print;

use crate::app::devices::{DummyAdcDevice, DummyBuzzerDevice};
use crate::app::Result;

mod app;

#[cfg(feature = "psram")]
extern crate alloc;
#[cfg(feature = "psram")]
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
#[cfg(feature = "psram")]
fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(
            esp_hal::psram::psram_vaddr_start() as *mut u8,
            esp_hal::psram::PSRAM_BYTES,
        );
    }
}

#[cfg(feature = "display-gc9a01")]
#[derive(Debug, Copy, Clone)]
struct DisplayResolution240x240;

#[cfg(feature = "display-gc9a01")]
struct MyBuffer<const N: usize>(alloc::boxed::Box<[u16; N]>);

#[cfg(feature = "display-gc9a01")]
impl DisplayDefinition for DisplayResolution240x240 {
    const WIDTH: u16 = 240;
    const HEIGHT: u16 = 240;

    type Buffer = MyBuffer<{ Self::WIDTH as usize * Self::HEIGHT as usize }>;
}

#[cfg(feature = "display-gc9a01")]
impl<const N: usize> NewZeroed for MyBuffer<N> {
    fn new_zeroed() -> Self {
        MyBuffer(alloc::boxed::Box::new([0; N]))
    }
}

#[cfg(feature = "display-gc9a01")]
impl<const N: usize> AsMut<[u16]> for MyBuffer<N> {
    fn as_mut(&mut self) -> &mut [u16] {
        &mut self.0[..]
    }
}

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
    static mut CLOCKS: Option<Clocks<'static>> = None;
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = unsafe {
        CLOCKS.replace(clocks);
        CLOCKS.as_ref().unwrap()
    };

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, clocks);
    embassy::init(&clocks, timg0);

    #[cfg(feature = "psram")]
    {
        esp_hal::psram::init_psram(peripherals.PSRAM);
        init_psram_heap();
    }

    rtt_init_print!();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(feature = "display-st7789")]
    let (sclk, mosi, res, dc, cs, bl, bl2) = {
        let sclk = io.pins.gpio2.into_push_pull_output();
        let mosi = io.pins.gpio3.into_push_pull_output();
        let res = io.pins.gpio10.into_push_pull_output();
        let dc = io.pins.gpio6.into_push_pull_output();
        let cs = io.pins.gpio7.into_push_pull_output();
        // espefuse.py -p /dev/ttyACM0 burn_efuse VDD_SPI_AS_GPIO 1
        let bl = io.pins.gpio11.into_push_pull_output();
        let bl2 = io.pins.gpio0.into_push_pull_output();
        (sclk, mosi, res, dc, cs, bl, bl2)
    };
    #[cfg(feature = "display-st7789-a")]
    let (sclk, mosi, res, dc, cs, bl, bl2) = {
        let sclk = io.pins.gpio18.into_push_pull_output();
        let mosi = io.pins.gpio17.into_push_pull_output();
        let res = io.pins.gpio4.into_push_pull_output();
        let dc = io.pins.gpio5.into_push_pull_output();
        let cs = io.pins.gpio8.into_push_pull_output();
        let bl = io.pins.gpio6.into_push_pull_output();
        let bl2 = io.pins.gpio0.into_push_pull_output();
        (sclk, mosi, res, dc, cs, bl, bl2)
    };
    #[cfg(feature = "display-gc9a01")]
    let (sclk, mosi, res, dc, cs, bl, bl2) = {
        let sclk = io.pins.gpio18.into_push_pull_output();
        let mosi = io.pins.gpio17.into_push_pull_output();
        let res = io.pins.gpio4.into_push_pull_output();
        let dc = io.pins.gpio5.into_push_pull_output();
        let cs = io.pins.gpio11.into_push_pull_output();
        let bl = io.pins.gpio6.into_push_pull_output();
        let bl2 = io.pins.gpio0.into_push_pull_output();
        (sclk, mosi, res, dc, cs, bl, bl2)
    };

    let ledc = make_static!(LEDC::new(peripherals.LEDC, clocks));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let lstimer0 = make_static!(ledc.get_timer::<LowSpeed>(timer::Number::Timer0));
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
            timer: lstimer0,
            duty_pct: 100,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    channel0.set_duty(50).unwrap();
    // channel0.start_duty_fade(0, 20, 200).unwrap();

    let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, clocks)
        .with_sck(sclk)
        .with_mosi(mosi);
    let spi_mutex = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_mutex, cs);
    let mut interface = display_interface_spi::SPIInterface::new(spi_device, dc);
    let mut delay = Delay::new(clocks);

    #[cfg(any(feature = "display-st7789", feature = "display-st7789-a"))]
    let mut display = {
        let mut lcd = st7789::ST7789::new(interface, Some(res), Some(bl2), 160, 80);
        lcd.init(&mut delay).unwrap();
        lcd.set_orientation(st7789::Orientation::Landscape).unwrap();
        // let mut display = lcd;
        // // let mut display = display.color_converted::<app::GuiColor>();
        // display.translated(Point::new(1, 26))
        lcd
    };

    #[cfg(feature = "display-gc9a01")]
    let mut display = {
        let lcd = gc9a01::Gc9a01::new(
            interface,
            DisplayResolution240x240 {},
            DisplayRotation::Rotate0,
        );
        lcd.into_buffered_graphics()
    };

    #[cfg(not(feature = "display-gc9a01"))]
    loop {
        Rectangle::new(
            Point::new(0, 0),
            embedded_graphics::geometry::Size::new(160, 80),
        )
        .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
            app::GuiColor::RED,
        ))
        .draw(&mut display)
        .unwrap();
        Rectangle::new(
            Point::new(0, 0),
            embedded_graphics::geometry::Size::new(160, 80),
        )
        .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
            app::GuiColor::BLUE,
        ))
        .draw(&mut display)
        .unwrap();
    }

    #[cfg(feature = "display-gc9a01")]
    loop {
        display.fill(app::GuiColor::RED.into_storage());
        display.flush().unwrap();
        display.fill(app::GuiColor::BLUE.into_storage());
        display.flush().unwrap();
    }

    Rectangle::new(
        Point::new(0, 0),
        embedded_graphics::geometry::Size::new(160, 80),
    )
    .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
        app::GuiColor::RED,
    ))
    .draw(&mut display)
    .unwrap();

    // let left = io.pins.gpio5.into_pull_up_input();
    // let right = io.pins.gpio9.into_pull_up_input();
    // let up = io.pins.gpio8.into_pull_up_input();
    // let down = io.pins.gpio13.into_pull_up_input();
    // let enter = io.pins.gpio4.into_pull_up_input();
    // let kbd_device = KeysInputDriver::new(left, right, up, down, enter);
    let kbd_device = app::devices::DummyKeyboardDevice {};

    let adc_device = DummyAdcDevice {};
    let board = BoardDriver {
        backlight: channel0,
    };
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
    defmt::panic!("stopped");
}

struct KeysInputDriver<L, R, U, D, E> {
    left: L,
    right: R,
    up: U,
    down: D,
    enter: E,
    state: [bool; 5],
}

impl<L, R, U, D, E> KeysInputDriver<L, R, U, D, E> {
    pub fn new(left: L, right: R, up: U, down: D, enter: E) -> Self {
        Self {
            left,
            right,
            up,
            down,
            enter,
            state: [false; 5],
        }
    }
}

impl<L, R, U, D, E> KeyboardDevice for KeysInputDriver<L, R, U, D, E>
where
    L: InputPin,
    R: InputPin,
    U: InputPin,
    D: InputPin,
    E: InputPin,
{
    fn read_key(&mut self) -> Keys {
        let state = [
            self.left.is_low().unwrap(),
            self.right.is_low().unwrap(),
            self.up.is_low().unwrap(),
            self.down.is_low().unwrap(),
            self.enter.is_low().unwrap(),
        ];
        for i in 0..5 {
            if state[i] && !self.state[i] {
                self.state[i] = true;
                return match i {
                    0 => Keys::Left,
                    1 => Keys::Right,
                    2 => Keys::Up,
                    3 => Keys::Down,
                    4 => Keys::Ok,
                    _ => Keys::None,
                };
            } else if !state[i] {
                self.state[i] = false;
            }
        }
        Keys::None
    }
}

struct BoardDriver<'a, S: TimerSpeed, O: OutputPin> {
    backlight: Channel<'a, S, O>,
}

impl<'a, S: TimerSpeed, O: OutputPin> BoardDevice for BoardDriver<'a, S, O> {
    fn set_brightness(&mut self, brightness: u8) {
        self.backlight.set_duty(brightness).unwrap();
    }
}

impl<'a, S: TimerSpeed, O: OutputPin> NvmDevice for BoardDriver<'a, S, O> {
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }

    fn write(&mut self, address: u32, buf: &[u8]) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }

    fn erase(&mut self, address: u32, len: usize) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }
}
