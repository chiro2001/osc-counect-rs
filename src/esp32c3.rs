#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use app::devices::{AdcDevice, BoardDevice, KeyboardDevice, Keys, NvmDevice};
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embedded_graphics::draw_target::DrawTargetExt;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Drawable;
use embedded_graphics::{geometry::Point, pixelcolor::RgbColor};
use embedded_hal::digital::InputPin;
use esp_backtrace as _;
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
use static_cell::make_static;

use rtt_target::rtt_init_print;

use crate::app::devices::DummyBuzzerDevice;
use crate::app::Result;

mod app;
mod common;

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
    let clocks = &*make_static!(ClockControl::max(system.clock_control).freeze());

    let timg0 = TimerGroup::new(peripherals.TIMG0, clocks);
    embassy::init(clocks, timg0);

    rtt_init_print!();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let res = io.pins.gpio10.into_push_pull_output();
    let dc = io.pins.gpio6.into_push_pull_output();
    let cs = io.pins.gpio7.into_push_pull_output();
    // espefuse.py -p /dev/ttyACM0 burn_efuse VDD_SPI_AS_GPIO 1
    let bl = io.pins.gpio11.into_push_pull_output();
    let bl2 = io.pins.gpio12.into_push_pull_output();

    let ledc = make_static!(LEDC::new(peripherals.LEDC, clocks));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let lstimer0 = make_static!(ledc.get_timer::<LowSpeed>(timer::Number::Timer0));
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 24u32.kHz(),
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
    channel0.set_duty(15).unwrap();
    // channel0.start_duty_fade(0, 20, 200).unwrap();

    let spi = Spi::new(peripherals.SPI2, 80u32.MHz(), SpiMode::Mode0, clocks)
        .with_sck(sclk)
        .with_mosi(mosi);
    let spi_mutex = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_mutex, cs);
    let interface = display_interface_spi::SPIInterface::new(spi_device, dc);
    let mut lcd = st7789::ST7789::new(interface, Some(res), Some(bl2), 160, 80);
    let mut delay = Delay::new(clocks);
    lcd.init(&mut delay).unwrap();
    lcd.set_orientation(st7789::Orientation::Landscape).unwrap();
    let mut display = lcd;
    // let mut display = display.color_converted::<app::GuiColor>();
    let mut display = display.translated(Point::new(1, 26));

    Rectangle::new(
        Point::new(0, 0),
        embedded_graphics::geometry::Size::new(160, 80),
    )
    .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
        app::GuiColor::RED,
    ))
    .draw(&mut display)
    .unwrap();

    // Create ADC instances
    // You can try any of the following calibration methods by uncommenting
    // them. Note that only AdcCalLine returns readings in mV; the other two
    // return raw readings in some unspecified scale.
    //
    // type AdcCal = ();
    // type AdcCal = esp_hal::adc::AdcCalBasic<ADC1>;
    // type AdcCal = esp_hal::adc::AdcCalLine<ADC1>;
    type AdcCal = esp_hal::adc::AdcCalCurve<esp_hal::peripherals::ADC1>;

    let analog_pin = io.pins.gpio0.into_analog();

    let mut adc1_config = esp_hal::adc::AdcConfig::new();
    let adc1_pin = adc1_config
        .enable_pin_with_cal::<_, AdcCal>(analog_pin, esp_hal::adc::Attenuation::Attenuation11dB);
    let adc1 = esp_hal::adc::ADC::<esp_hal::peripherals::ADC1>::new(peripherals.ADC1, adc1_config);
    let adc_device = AdcDriver::new(adc1, adc1_pin);

    let left = io.pins.gpio5.into_pull_up_input();
    let right = io.pins.gpio9.into_pull_up_input();
    let up = io.pins.gpio8.into_pull_up_input();
    let down = io.pins.gpio13.into_pull_up_input();
    let enter = io.pins.gpio4.into_pull_up_input();
    let kbd_device = KeysInputDriver::new(left, right, up, down, enter);

    // let adc_device = DummyAdcDevice {};
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
    defmt::panic!("Simulator stopped");
}

struct AdcDriver<'d, A, P, C> {
    adc: esp_hal::adc::ADC<'d, A>,
    pin: esp_hal::adc::AdcPin<P, A, C>,
}
impl<'d, A, P, C> AdcDriver<'d, A, P, C> {
    pub fn new(adc: esp_hal::adc::ADC<'d, A>, pin: esp_hal::adc::AdcPin<P, A, C>) -> Self {
        Self { adc, pin }
    }
}
impl<'d, A, P, C> AdcDevice for AdcDriver<'d, A, P, C>
where
    A: esp_hal::adc::RegisterAccess,
    P: embedded_hal_02::adc::Channel<A, ID = u8>,
    C: esp_hal::adc::AdcCalScheme<A>,
    esp_hal::adc::ADC<'d, A>: embedded_hal_02::adc::OneShot<A, u16, esp_hal::adc::AdcPin<P, A, C>>,
{
    async fn read(
        &mut self,
        _options: app::devices::AdcReadOptions,
        buf: &mut [f32],
    ) -> Result<usize> {
        let mut count = 0;
        let mut it = buf.iter_mut();
        while let Some(data) = it.next() {
            let mv = nb::block!(self.adc.read(&mut self.pin))
                .map_err(|_| app::AppError::AdcReadError)?;
            // defmt::info!("ADC: {} mv", mv);
            *data = mv as f32 / 1000.0;
            count += 1;
        }
        Ok(count)
    }
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

impl<'a, S: TimerSpeed, O: OutputPin> BoardDevice for BoardDriver<'a, S, O>
where
    Channel<'a, S, O>: esp_hal::ledc::channel::ChannelHW<O>,
{
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
