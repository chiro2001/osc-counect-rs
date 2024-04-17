#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(dead_code)]

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
use esp_hal::dma::TxPrivate;
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

use esp32c3 as pac;

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
    type AdcCal = esp_hal::adc::AdcCalLine<esp_hal::peripherals::ADC1>;
    // type AdcCal = esp_hal::adc::AdcCalCurve<esp_hal::peripherals::ADC1>;

    let analog_pin = io.pins.gpio0.into_analog();

    let mut adc1_config = esp_hal::adc::AdcConfig::new();
    let adc1_pin = adc1_config
        .enable_pin_with_cal::<_, AdcCal>(analog_pin, esp_hal::adc::Attenuation::Attenuation11dB);
    let adc1 = esp_hal::adc::ADC::<esp_hal::peripherals::ADC1>::new(peripherals.ADC1, adc1_config);

    const DR_REG_SYSTEM_BASE: u32 = 0x600c0000;
    const SYSTEM_PERIP_CLK_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x10;
    const SYSTEM_PERIP_RST_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x18;
    const APB_SARADC_CLK_EN_M: u32 = 0x00000001 << 28;
    const DR_REG_APB_SARADC_BASE: u32 = 0x60040000;
    const APB_SARADC_APB_ADC_CLKM_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x54;
    const APB_SARADC_REG_CLK_SEL_V: u32 = 0x00000003;
    const APB_SARADC_REG_CLK_SEL_S: u32 = 21;
    const APB_SARADC_CTRL_REG: u32 = DR_REG_APB_SARADC_BASE;
    const APB_SARADC_SAR_PATT_P_CLEAR_M: u32 = 0x00000001 << 23;
    const APB_SARADC_SAR_PATT_LEN_V: u32 = 0x00000007;
    const APB_SARADC_SAR_PATT_LEN_S: u32 = 15;
    const APB_SARADC_SAR_CLK_GATED_M: u32 = 0x00000001 << 6;
    const APB_SARADC_XPD_SAR_FORCE_V: u32 = 0x00000003;
    const APB_SARADC_XPD_SAR_FORCE_S: u32 = 27;
    const APB_SARADC_SAR_CLK_DIV_V: u32 = 0x000000FF;
    const APB_SARADC_SAR_CLK_DIV_S: u32 = 7;
    const APB_SARADC_SAR_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x18;
    const APB_SARADC_SAR_PATT_TAB2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x1c;
    const APB_SARADC_SAR_PATT_TAB1_V: u32 = 0x00FFFFFF;
    const APB_SARADC_SAR_PATT_TAB1_S: u32 = 0;
    const APB_SARADC_SAR_PATT_TAB2_V: u32 = 0x00FFFFFF;
    const APB_SARADC_SAR_PATT_TAB2_S: u32 = 0;
    const APB_SARADC_CTRL2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x4;
    const APB_SARADC_TIMER_TARGET_V: u32 = 0x00000FFF;
    const APB_SARADC_TIMER_TARGET_S: u32 = 12;
    const APB_SARADC_REG_CLKM_DIV_NUM_V: u32 = 0x000000FF;
    const APB_SARADC_REG_CLKM_DIV_NUM_S: u32 = 0;
    const APB_SARADC_MEAS_NUM_LIMIT: u32 = 1 << 0;
    const APB_SARADC_DMA_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x50;
    const APB_SARADC_APB_ADC_TRANS_M: u32 = 0x00000001 << 31;
    const APB_SARADC_TIMER_EN: u32 = 1 << 24;
    const APB_SARADC_FSM_WAIT_REG: u32 = DR_REG_APB_SARADC_BASE + 0xc;
    const APB_SARADC_RSTB_WAIT_V: u32 = 0x000000FF;
    const APB_SARADC_RSTB_WAIT_S: u32 = 8;
    const APB_SARADC_XPD_WAIT_V: u32 = 0x000000FF;
    const APB_SARADC_XPD_WAIT_S: u32 = 0;
    const APB_SARADC_STANDBY_WAIT_V: u32 = 0x000000FF;
    const APB_SARADC_STANDBY_WAIT_S: u32 = 16;
    const SYSTEM_APB_SARADC_RST_M: u32 = 0x00000001 << 28;
    const SYSTEM_APB_SARADC_CLK_EN_M: u32 = 0x00000001 << 28;

    const ADC_LL_CLKM_DIV_NUM_DEFAULT: u32 = 15;
    const ADC_LL_CLKM_DIV_B_DEFAULT: u32 = 1;
    const ADC_LL_CLKM_DIV_A_DEFAULT: u32 = 0;

    let sample_freq_hz = 80_000;
    let clk_src_freq_hz = 5_000_000;
    let interval = clk_src_freq_hz
        / (ADC_LL_CLKM_DIV_NUM_DEFAULT + ADC_LL_CLKM_DIV_A_DEFAULT / ADC_LL_CLKM_DIV_B_DEFAULT + 1)
        / 2
        / sample_freq_hz;

    info!("interval: {}", interval);
    // reg_set_field(
    //     APB_SARADC_CTRL2_REG,
    //     APB_SARADC_TIMER_TARGET_V,
    //     APB_SARADC_TIMER_TARGET_S,
    //     interval,
    // );
    // reg_set_field(
    //     APB_SARADC_APB_ADC_CLKM_CONF_REG,
    //     APB_SARADC_REG_CLKM_DIV_NUM_V,
    //     APB_SARADC_REG_CLKM_DIV_NUM_S,
    //     1,
    // );

    let saradc = &*unsafe { pac::APB_SARADC::steal() };
    // esp_println::println!("saradc: {:#?}", saradc);
    saradc.onetime_sample().modify(|_, w| {
        w.saradc1_onetime_sample().clear_bit();
        w.saradc2_onetime_sample().clear_bit();
        w
    });
    // typedef struct {
    //     uint8_t atten;      ///< Attenuation of this ADC channel
    //     uint8_t channel;    ///< ADC channel
    //     uint8_t unit;       ///< ADC unit
    //     uint8_t bit_width;  ///< ADC output bit width
    // } adc_digi_pattern_config_t;
    let pattern = [0u32, 0, 0, 12];
    // pattern.val = (table.atten & 0x3) | ((table.channel & 0x7) << 2) | ((table.unit & 0x1) << 5);
    let val = (pattern[0] & 0x3) | ((pattern[1] & 0x7) << 2) | ((pattern[2] & 0x1) << 5);
    saradc
        .sar_patt_tab1()
        .modify(|_, w| unsafe { w.saradc_sar_patt_tab1().bits(val) });
    saradc.ctrl2().modify(|_, w| {
        w.saradc_meas_num_limit().set_bit();
        w.saradc_max_meas_num().variant(10);
        w
    });
    // saradc.arb_ctrl().modify(|_, w| w.adc_arb_apb_priority());
    // stop adc
    saradc
        .ctrl2()
        .modify(|_, w| w.saradc_timer_en().clear_bit());
    let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) =
        esp_hal::dma_buffers!(1024);
    // let dma_channel = esp_hal::dma::Channel0 {};
    // let dma = esp_hal::dma::ChannelRx::new(rx_descriptors, dma_channel, false);
    // esp_hal::dma::ChannelCreator0::configure(self, burst_mode, tx_descriptors, rx_descriptors, priority)
    let dma = esp_hal::dma::Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        esp_hal::dma::DmaPriority::Priority0,
    );
    dma_channel.tx.is_done();
    // loop {}

    // let adc_device = AdcDriver::new(adc1, adc1_pin);
    let adc_device = AdcDmaDriver::new(adc1, adc1_pin);

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
struct AdcDmaDriver<'d, A, P, C> {
    adc: esp_hal::adc::ADC<'d, A>,
    pin: esp_hal::adc::AdcPin<P, A, C>,
}
impl<'d, A, P, C> AdcDmaDriver<'d, A, P, C> {
    pub fn new(adc: esp_hal::adc::ADC<'d, A>, pin: esp_hal::adc::AdcPin<P, A, C>) -> Self {
        Self { adc, pin }
    }
}
impl<'d, A, P, C> AdcDevice for AdcDmaDriver<'d, A, P, C>
where
    A: esp_hal::adc::RegisterAccess,
    P: embedded_hal_02::adc::Channel<A, ID = u8>,
{
    async fn read(
        &mut self,
        _options: app::devices::AdcReadOptions,
        buf: &mut [f32],
    ) -> Result<usize> {
        let mut count = 0;
        let mut it = buf.iter_mut();
        while let Some(data) = it.next() {
            let mv = 0;
            *data = mv as f32 / 1000.0;
            count += 1;
        }
        Ok(count)
    }
}
fn reg_set_field(reg: u32, field_v: u32, field_s: u32, value: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(
            ((reg as *mut u32).read_volatile() & !(field_v << field_s))
                | ((value & field_v) << field_s),
        )
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
    fn read(&mut self, _address: u32, _buf: &mut [u8]) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }

    fn write(&mut self, _address: u32, _buf: &[u8]) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }

    fn erase(&mut self, _address: u32, _len: usize) -> Result<()> {
        Err(app::AppError::NotImplemented)
    }
}
