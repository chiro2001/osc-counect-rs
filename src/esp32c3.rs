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
use embedded_hal::{delay::DelayNs, digital::InputPin};
use esp_backtrace as _;
use esp_hal::dma::{RegisterAccess, TxPrivate};
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

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, clocks);
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
    // type AdcCal = esp_hal::analog::adc::AdcCalBasic<ADC1>;
    type AdcCal = esp_hal::analog::adc::AdcCalLine<esp_hal::peripherals::ADC1>;
    // type AdcCal = esp_hal::analog::adc::AdcCalCurve<esp_hal::peripherals::ADC1>;

    let analog_pin = io.pins.gpio0.into_analog();

    let mut adc1_config = esp_hal::analog::adc::AdcConfig::new();
    let _adc1_pin = adc1_config.enable_pin_with_cal::<_, AdcCal>(
        analog_pin,
        esp_hal::analog::adc::Attenuation::Attenuation11dB,
    );
    // to enable clocks
    let _adc1 =
        esp_hal::analog::adc::ADC::<esp_hal::peripherals::ADC1>::new(peripherals.ADC1, adc1_config);

    const ADC_LL_CLKM_DIV_NUM_DEFAULT: u8 = 15;
    const ADC_LL_CLKM_DIV_B_DEFAULT: u8 = 1;
    const ADC_LL_CLKM_DIV_A_DEFAULT: u8 = 0;

    // let sample_freq_hz = 80_000;
    // let clk_src_freq_hz = 5_000_000;
    // let interval = clk_src_freq_hz
    //     / (ADC_LL_CLKM_DIV_NUM_DEFAULT + ADC_LL_CLKM_DIV_A_DEFAULT / ADC_LL_CLKM_DIV_B_DEFAULT + 1)
    //     / 2
    //     / sample_freq_hz;

    // info!("interval: {}", interval);

    let saradc = &*unsafe { pac::APB_SARADC::steal() };
    // stop adc
    saradc
        .ctrl2()
        .modify(|_, w| w.saradc_timer_en().clear_bit());
    saradc.ctrl().modify(|_, w| {
        w.saradc_start_force()
            .clear_bit()
            .saradc_start()
            .clear_bit()
    });
    // setup clocks
    saradc.clkm_conf().modify(|_, w| unsafe {
        w.clkm_div_a()
            .bits(ADC_LL_CLKM_DIV_A_DEFAULT)
            .clkm_div_b()
            .bits(ADC_LL_CLKM_DIV_B_DEFAULT)
            .clkm_div_num()
            .bits(ADC_LL_CLKM_DIV_NUM_DEFAULT)
            .clk_en()
            .set_bit()
    });
    saradc.onetime_sample().modify(|_, w| {
        w.saradc1_onetime_sample().clear_bit();
        w.saradc2_onetime_sample().clear_bit();
        w
    });
    // clear pattern
    saradc
        .ctrl()
        .modify(|_, w| w.saradc_sar_patt_p_clear().set_bit());
    saradc
        .ctrl()
        .modify(|_, w| w.saradc_sar_patt_p_clear().clear_bit());
    let pattern_len = 1;
    saradc
        .ctrl()
        .modify(|_, w| unsafe { w.saradc_sar_patt_len().bits(pattern_len as u8 - 1) });
    {
        // setup patterns
        // typedef struct {
        //     uint8_t atten;      ///< Attenuation of this ADC channel
        //     uint8_t channel;    ///< ADC channel
        //     uint8_t unit;       ///< ADC unit
        //     uint8_t bit_width;  ///< ADC output bit width
        // } adc_digi_pattern_config_t;
        // typedef struct  {
        //     union {
        //         struct {
        //             uint8_t atten:      2;
        //             uint8_t channel:    3;
        //             uint8_t unit:       1;
        //             uint8_t reserved:   2;
        //         };
        //         uint8_t val;
        //     };
        // } __attribute__((packed)) adc_ll_digi_pattern_table_t;
        let pattern = [3u32, 0, 0, 12];
        let pattern_val =
            (pattern[0] & 0x3) | ((pattern[1] & 0x7) << 2) | ((pattern[2] & 0x1) << 5);
        let pattern_index = 0;
        let tab = saradc.sar_patt_tab1().read().bits();
        defmt::info!("read tab {:x}", tab);
        let _index = pattern_index / 4;
        let offset = (pattern_index % 4) * 6;
        let mut tab = tab;
        tab &= !(0xFC0000 >> offset);
        tab |= ((pattern_val & 0x3F) << 18) >> offset;
        saradc
            .sar_patt_tab1()
            .write(|w| unsafe { w.saradc_sar_patt_tab1().bits(tab) });
        defmt::info!("set tab to {:x}", tab);
    }
    saradc.ctrl2().modify(|_, w| unsafe {
        // w.saradc_meas_num_limit().set_bit();
        w.saradc_meas_num_limit().clear_bit();
        w.saradc_max_meas_num().bits(10);
        // dump
        w.saradc_timer_target().bits(0b111111111011);
        w
    });
    // // set sample cycle
    // const I2C_SAR_ADC: u8 = 0x69;
    // const I2C_SAR_ADC_HOSTID: u8 = 0;
    // const ADC_SAR1_SAMPLE_CYCLE_ADDR: u8 = 0x2;
    // const ADC_SAR1_SAMPLE_CYCLE_ADDR_MSB: u8 = 0x2;
    // const ADC_SAR1_SAMPLE_CYCLE_ADDR_LSB: u8 = 0x0;
    // const ADC_LL_SAMPLE_CYCLE_DEFAULT: u8 = 2;
    // regi2c_write_mask(
    //     I2C_SAR_ADC,
    //     I2C_SAR_ADC_HOSTID,
    //     ADC_SAR1_SAMPLE_CYCLE_ADDR,
    //     ADC_SAR1_SAMPLE_CYCLE_ADDR_MSB,
    //     ADC_SAR1_SAMPLE_CYCLE_ADDR_LSB,
    //     ADC_LL_SAMPLE_CYCLE_DEFAULT,
    // );
    const BUFFER_LEN: usize = 1024;
    let (_tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) =
        esp_hal::dma_buffers!(BUFFER_LEN);
    rx_buffer.iter_mut().for_each(|x| *x = 0);
    let dma = esp_hal::dma::Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        esp_hal::dma::DmaPriority::Priority0,
    );
    use esp_hal::dma::RxPrivate;
    let mut rx = dma_channel.rx;
    // rx.is_done();
    // let dma_reg = &*unsafe { pac::DMA::steal() };
    // reset adc digital controller
    saradc
        .dma_conf()
        .modify(|_, w| w.apb_adc_reset_fsm().set_bit());
    saradc
        .dma_conf()
        .modify(|_, w| w.apb_adc_reset_fsm().clear_bit());
    // set adc eof
    const SOC_ADC_DIGI_DATA_BYTES_PER_CONV: u16 = 4;
    let eof: u16 = BUFFER_LEN as u16;
    saradc.dma_conf().modify(|_, w| unsafe {
        w.apb_adc_eof_num()
            .bits(eof / SOC_ADC_DIGI_DATA_BYTES_PER_CONV)
    });
    // start dma
    rx.prepare_transfer_without_start(
        false,
        esp_hal::dma::DmaPeripheral::Adc,
        rx_buffer.as_mut_ptr(),
        BUFFER_LEN,
    )
    .unwrap();
    rx.listen_eof();
    rx.start_transfer().unwrap();
    // connect DMA and peripheral
    saradc.dma_conf().modify(|_, w| w.apb_adc_trans().set_bit());
    // start ADC
    saradc.ctrl2().modify(|_, w| w.saradc_timer_en().set_bit());

    // esp_println::println!("after config, dma: {:#?}; \nsaradc: {:#?}", dma_reg, saradc);
    esp_hal::dma::Channel0::start_in();
    // esp_println::println!("in_link_ch0: {:#?}", dma_reg.in_link_ch(0).read());

    while !rx.is_done() {
        // let available = rx.available();
        // info!(
        //     "waiting... available = {}, buffer[0] is {:02x}",
        //     available, rx_buffer[0]
        // );
        delay.delay_ms(1u32);
    }
    info!("DMA done");
    // for i in 0..BUFFER_LEN {
    //     info!("rx_buffer[{}] = {:02x}", i, rx_buffer[i]);
    // }
    for i in 0..(BUFFER_LEN / 4) {
        let t = AdcDigiOutputData(u32::from_le_bytes([
            rx_buffer[i * 4],
            rx_buffer[i * 4 + 1],
            rx_buffer[i * 4 + 2],
            rx_buffer[i * 4 + 3],
        ]));
        info!(
            "data[{}]: data = {}, channel = {}, unit = {}",
            i / 4,
            t.data(),
            t.channel(),
            t.unit()
        );
    }

    loop {}

    // let adc_device = AdcDriver::new(adc1, adc1_pin);
    let adc_device = AdcDmaDriver::new(_adc1, _adc1_pin);

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

// fn regi2c_write_mask(block: u8, host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
//     unsafe {
//         esp_hal::rom::rom_i2c_writeReg_Mask(
//             block as _,
//             host_id as _,
//             reg_add as _,
//             msb as _,
//             lsb as _,
//             data as _,
//         );
//     }
// }
extern "C" {
    pub(crate) fn rom_i2c_writeReg_Mask(
        block: u32,
        block_hostid: u32,
        reg_add: u32,
        reg_add_msb: u32,
        reg_add_lsb: u32,
        indata: u32,
    );
}
fn regi2c_write_mask(block: u8, host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    unsafe {
        rom_i2c_writeReg_Mask(
            block as _,
            host_id as _,
            reg_add as _,
            msb as _,
            lsb as _,
            data as _,
        );
    }
}

// typedef struct {
//     union {
//         struct {
//             uint32_t data:          12; /*!<ADC real output data info. Resolution: 12 bit. */
//             uint32_t reserved12:    1;  /*!<Reserved12. */
//             uint32_t channel:       3;  /*!<ADC channel index info.
//                                             If (channel < ADC_CHANNEL_MAX), The data is valid.
//                                             If (channel > ADC_CHANNEL_MAX), The data is invalid. */
//             uint32_t unit:          1;  /*!<ADC unit index info. 0: ADC1; 1: ADC2.  */
//             uint32_t reserved17_31: 15; /*!<Reserved17. */
//         } type2;                        /*!<When the configured output format is 12bit. */
//         uint32_t val;                   /*!<Raw data value */
//     };
// } adc_digi_output_data_t;
#[derive(Clone, Copy)]
pub struct AdcDigiOutputData(u32);
impl AdcDigiOutputData {
    pub fn data(&self) -> u16 {
        (self.0 & 0xFFF) as u16
    }
    pub fn channel(&self) -> u8 {
        ((self.0 >> 13) & 0x7) as u8
    }
    pub fn unit(&self) -> u8 {
        ((self.0 >> 16) & 0x1) as u8
    }
}

struct AdcDriver<'d, A, P, C> {
    adc: esp_hal::analog::adc::ADC<'d, A>,
    pin: esp_hal::analog::adc::AdcPin<P, A, C>,
}
impl<'d, A, P, C> AdcDriver<'d, A, P, C> {
    pub fn new(
        adc: esp_hal::analog::adc::ADC<'d, A>,
        pin: esp_hal::analog::adc::AdcPin<P, A, C>,
    ) -> Self {
        Self { adc, pin }
    }
}
impl<'d, A, P, C> AdcDevice for AdcDriver<'d, A, P, C>
where
    A: esp_hal::analog::adc::RegisterAccess,
    P: embedded_hal_02::adc::Channel<A, ID = u8> + esp_hal::analog::adc::AdcChannel,
    C: esp_hal::analog::adc::AdcCalScheme<A>,
    esp_hal::analog::adc::ADC<'d, A>:
        embedded_hal_02::adc::OneShot<A, u16, esp_hal::analog::adc::AdcPin<P, A, C>>,
{
    async fn read(
        &mut self,
        _options: app::devices::AdcReadOptions,
        buf: &mut [f32],
    ) -> Result<usize> {
        let mut count = 0;
        let mut it = buf.iter_mut();
        while let Some(data) = it.next() {
            let mv = nb::block!(self.adc.read_oneshot(&mut self.pin))
                .map_err(|_| app::AppError::AdcReadError)?;
            // defmt::info!("ADC: {} mv", mv);
            *data = mv as f32 / 1000.0;
            count += 1;
        }
        Ok(count)
    }
}
struct AdcDmaDriver<'d, A, P, C> {
    adc: esp_hal::analog::adc::ADC<'d, A>,
    pin: esp_hal::analog::adc::AdcPin<P, A, C>,
}
impl<'d, A, P, C> AdcDmaDriver<'d, A, P, C> {
    pub fn new(
        adc: esp_hal::analog::adc::ADC<'d, A>,
        pin: esp_hal::analog::adc::AdcPin<P, A, C>,
    ) -> Self {
        Self { adc, pin }
    }
}
impl<'d, A, P, C> AdcDevice for AdcDmaDriver<'d, A, P, C>
where
    A: esp_hal::analog::adc::RegisterAccess,
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
