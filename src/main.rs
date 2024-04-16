#![allow(clippy::empty_loop)]
#![allow(static_mut_refs)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]
#![allow(dead_code)]

extern crate alloc;

use core::convert::Infallible;

use app::devices::{BoardDevice, BuzzerDevice, NvmDevice};
use defmt::*;
use embedded_graphics::{draw_target::DrawTargetExt, geometry::Point};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use static_cell::make_static;

use {defmt_rtt as _, panic_probe as _};

use ili9341::{DisplaySize240x320, Ili9341 as Ili9327, Orientation};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    dma::NoDma,
    gpio::{Flex, Input, OutputType, Pull},
    peripherals::ADC1,
    spi::Spi,
    time::Hertz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        CaptureCompare16bitInstance, Channel,
    },
    Peripheral, PeripheralRef,
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_stm32::{pac, Config};
use embassy_time::{Delay, Timer};

use display_interface_fsmc as fsmc;

use tm1668::InoutPin;

mod app;

#[cfg(feature = "stm32f103vc")]
bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<ADC1>;
});

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();
struct DioPin<'d, T: embassy_stm32::gpio::Pin> {
    pin: Flex<'d, T>,
}

impl<'d, T> InoutPin for DioPin<'d, T>
where
    T: embassy_stm32::gpio::Pin,
{
    fn set_input(&mut self) {
        self.pin.set_as_input(Pull::None);
    }

    fn set_output(&mut self) {
        self.pin.set_as_output(Speed::Low);
    }

    fn set_high(&mut self) {
        self.pin.set_high();
    }

    fn set_low(&mut self) {
        self.pin.set_low();
    }

    fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    fn is_low(&self) -> bool {
        self.pin.is_low()
    }
}

pub struct KeyboardDriver<'d, S, C, D, DELAY> {
    driver: tm1668::TM1668<'d, S, C, D, DELAY>,
    keys: [bool; 20],
}
impl<'d, S, C, D, DELAY> KeyboardDriver<'d, S, C, D, DELAY>
where
    S: OutputPin<Error = Infallible>,
    C: OutputPin<Error = Infallible>,
    D: InoutPin,
    DELAY: DelayNs,
{
    pub fn new(driver: tm1668::TM1668<'d, S, C, D, DELAY>) -> Self {
        Self {
            driver,
            keys: [false; 20],
        }
    }
}
impl<'d, S, C, D, DELAY> app::devices::KeyboardDevice for KeyboardDriver<'d, S, C, D, DELAY>
where
    S: OutputPin<Error = Infallible>,
    C: OutputPin<Error = Infallible>,
    D: InoutPin,
    DELAY: DelayNs,
{
    fn read_key(&mut self) -> app::devices::Keys {
        let mut keys = [false; 20];
        self.driver.read_decode_keys(&mut keys);
        let mut r = app::devices::Keys::None;
        for (i, k) in keys.iter().enumerate() {
            // read key up
            if !*k && self.keys[i] {
                r = app::devices::Keys::try_from(i).unwrap();
                break;
            }
        }
        self.keys = keys;
        r
    }
    fn read_key_event(&mut self) -> app::devices::InputEvent {
        let mut keys = [false; 20];
        self.driver.read_decode_keys(&mut keys);
        let mut r = app::devices::InputEvent::None;
        for (i, k) in keys.iter().enumerate() {
            // read key up
            if !*k && self.keys[i] {
                r = app::devices::InputEvent::KeyReleased(app::devices::Keys::try_from(i).unwrap());
                self.keys[i] = false;
                break;
            }
            // read key down
            if *k && !self.keys[i] {
                r = app::devices::InputEvent::KeyPressed(app::devices::Keys::try_from(i).unwrap());
                self.keys[i] = true;
                break;
            }
        }
        r
    }
}

pub struct OutputInversePin<T> {
    pin: T,
}
impl<T: embedded_hal::digital::OutputPin> OutputInversePin<T> {
    pub fn new(pin: T) -> Self {
        Self { pin }
    }
}
impl<T: embedded_hal::digital::OutputPin> embedded_hal::digital::ErrorType for OutputInversePin<T> {
    type Error = T::Error;
}
impl<T: embedded_hal::digital::OutputPin> embedded_hal::digital::OutputPin for OutputInversePin<T> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // #[cfg(feature = "custom-alloc")]
    // heap_init!(32 * 1024);
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        #[cfg(feature = "stm32h743vi")]
        {
            config.rcc.hse = Some(Hse {
                freq: Hertz(25_000_000),
                mode: HseMode::Oscillator,
            });
            config.rcc.pll1 = Some(Pll {
                source: PllSource::HSE,
                prediv: PllPreDiv::DIV5,
                mul: PllMul::MUL192,
                divp: Some(PllDiv::DIV2),
                divq: None,
                divr: None,
            });
            config.rcc.sys = Sysclk::PLL1_P;
        }
        // #[cfg(feature = "stm32f103vc")]
        // {
        //     config.rcc.hse = Some(Hse {
        //         freq: Hertz(8_000_000),
        //         mode: HseMode::Oscillator,
        //     });
        //     #[cfg(not(feature = "overclocking"))]
        //     {
        //         config.rcc.pll = Some(Pll {
        //             src: PllSource::HSE,
        //             prediv: PllPreDiv::DIV1,
        //             mul: PllMul::MUL9,
        //         });
        //     }
        //     #[cfg(feature = "overclocking")]
        //     {
        //         config.rcc.pll = Some(Pll {
        //             src: PllSource::HSE,
        //             prediv: PllPreDiv::DIV1,
        //             // overclocking to 8 * 16 = 128 MHz
        //             mul: PllMul::MUL16,
        //         });
        //     }
        // }
        // config.rcc.sys = Sysclk::PLL1_P;
        // #[cfg(feature = "stm32f103vc")]
        // {
        //     config.rcc.apb2_pre = APBPrescaler::DIV1;
        //     #[cfg(feature = "overclocking")]
        //     {
        //         config.rcc.ahb_pre = AHBPrescaler::DIV2;
        //         config.rcc.apb1_pre = APBPrescaler::DIV2;
        //         // overclocking ADC 16MHz, note that the max ADC clock is 14MHz
        //         config.rcc.adc_pre = ADCPrescaler::DIV4;
        //     }
        //     #[cfg(not(feature = "overclocking"))]
        //     {
        //         config.rcc.ahb_pre = AHBPrescaler::DIV1;
        //         config.rcc.apb1_pre = APBPrescaler::DIV2;
        //         // ADC 72 / 6 = 12 MHz
        //         config.rcc.adc_pre = ADCPrescaler::DIV6;
        //     }
        // }
        #[cfg(feature = "stm32f103vc")]
        {
            config.rcc.hse = Some(Hertz(8_000_000));
            config.rcc.sys_ck = Some(Hertz(72_000_000));
            config.rcc.hclk = Some(Hertz(72_000_000));
            config.rcc.pclk1 = Some(Hertz(36_000_000));
            config.rcc.pclk2 = Some(Hertz(72_000_000));
            config.rcc.adcclk = Some(Hertz(12_000_000));
        }

        #[cfg(feature = "stm32h743vi")]
        {
            config.rcc.d1c_pre = AHBPrescaler::DIV1;
            config.rcc.ahb_pre = AHBPrescaler::DIV2;
            config.rcc.apb1_pre = APBPrescaler::DIV2;
            config.rcc.apb2_pre = APBPrescaler::DIV2;
            config.rcc.apb3_pre = APBPrescaler::DIV2;
            config.rcc.apb4_pre = APBPrescaler::DIV2;
        }
    }
    let p = embassy_stm32::init(config);
    // let p = embassy_stm32::init(Default::default());

    let delay = make_static!(Delay {});
    // static mut DELAY: Delay = Delay {};

    info!("System launched!");

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 6 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    #[cfg(feature = "display-ili9327")]
    let display = {
        let init = fsmc::FsmcNorsramInitTypeDef {
            ns_bank: 0,
            data_address_mux: 0,
            memory_type: 0,
            memory_data_width: 0x10,
            burst_access_mode: 0,
            wait_signal_polarity: 0,
            wrap_mode: 0,
            wait_signal_active: 0,
            write_operation: 0x1000,
            wait_signal: 0,
            extended_mode: 0x4000,
            asynchronous_wait: 0,
            write_burst: 0,
            page_size: 0,
        };
        let timing = fsmc::FsmcNorsramTimingTypeDef {
            address_setup_time: 0,
            address_hold_time: 1,
            data_setup_time: 1,
            bus_turn_around_duration: 0,
            clk_division: 1,
            data_latency: 2,
            access_mode: 0,
        };
        let ext_timing = fsmc::FsmcNorsramTimingTypeDef {
            address_setup_time: 0,
            address_hold_time: 1,
            data_setup_time: 1,
            bus_turn_around_duration: 0,
            clk_division: 1,
            data_latency: 2,
            access_mode: 0,
        };
        let hsram = fsmc::SramHandleTypeDef::new(init, timing, ext_timing);
        pac::GPIOD
            .cr(0)
            .write_value(pac::gpio::regs::Cr(0xB4BB44BB));
        pac::GPIOD
            .cr(1)
            .write_value(pac::gpio::regs::Cr(0xBB44BBBB));
        pac::GPIOE
            .cr(0)
            .write_value(pac::gpio::regs::Cr(0xB4444444));
        pac::GPIOE
            .cr(1)
            .write_value(pac::gpio::regs::Cr(0xBBBBBBBB));
        pac::RCC.ahbenr().modify(|w| w.set_fsmcen(true));
        let interface = fsmc::FsmcInterface::new(hsram, PeripheralRef::new(p.DMA1_CH4));
        let rst = Output::new(p.PC9, Level::Low, Speed::Low);
        let mut lcd = Ili9327::new(
            interface,
            rst,
            delay,
            Orientation::LandscapeFlipped,
            // Orientation::PortraitFlipped,
            DisplaySize240x320,
        )
        // .await
        .unwrap();
        lcd.clear_screen(0).unwrap();
        lcd
    };
    #[cfg(any(feature = "display-st7789-1", feature = "display-st7789-2"))]
    let display = {
        let dc = Output::new(p.PE13, Level::Low, Speed::Low);
        let bl2 = OutputInversePin::new(Output::new(p.PE9, Level::Low, Speed::Low));
        let cs = Output::new(p.PE11, Level::High, Speed::Low);
        let reset = Output::new(p.PE6, Level::High, Speed::Low);
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.frequency = Hertz(60_000_000);
        let spi = Spi::new(p.SPI4, p.PE12, p.PE14, p.PE5, NoDma, NoDma, spi_config);
        let spi_mutex = &*static_cell::make_static!(embassy_sync::blocking_mutex::NoopMutex::new(
            core::cell::RefCell::new(spi)
        ));
        let spi_device =
            embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(spi_mutex, cs);
        let interface = display_interface_spi::SPIInterface::new(spi_device, dc);
        let display = make_static!(st7789::ST7789::new(
            interface,
            Some(reset),
            Some(bl2),
            160,
            80
        ));
        lcd.init(delay).unwrap();
        lcd.set_orientation(st7789::Orientation::LandscapeSwapped)
            .unwrap();
        lcd.translated(Point::new(1, 26))
    };
    #[cfg(feature = "display-gu256x128c")]
    let mut display = {
        let cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);
        let latch = Output::new(p.PB6, Level::Low, Speed::VeryHigh);
        let ready = Input::new(p.PB4, Pull::None);
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.frequency = Hertz(15_000_000);
        // spi_config.frequency = Hertz(1_000_000);
        // spi_config.frequency = Hertz(1_875_000);
        spi_config.mode = embedded_hal_02::spi::MODE_2;
        let spi = Spi::new(p.SPI6, p.PB3, p.PB5, p.PA6, NoDma, NoDma, spi_config);
        let interface = gu256x128c::Spi74hc595::new(spi, latch, cs, ready, delay);
        let mut vfd = gu256x128c::Gu256x128c::new(interface);
        vfd.init().unwrap();
        vfd
    };
    info!("Display OK!");

    let mut delay = Delay {};
    loop {
        let s = "TEST...\r\n";
        display.write_str(s).unwrap();
        delay.delay_ms(100);
    }

    #[cfg(feature = "stm32f103vc")]
    let (bl, bl_channel) = {
        let mut bl = SimplePwm::new(
            // Warning: TIM3 channel 3 not usable
            p.TIM8,
            None,
            None,
            Some(PwmPin::new_ch3(p.PC8, OutputType::PushPull)),
            None,
            Hertz::khz(2),
            Default::default(),
        );
        bl.enable(Channel::Ch3);
        bl.set_duty(Channel::Ch3, bl.get_max_duty() / 2);
        (bl, Channel::Ch3)
    };
    #[cfg(feature = "stm32h743vi")]
    let (bl, bl_channel) = {
        let mut bl = ComplementaryPwm::new(
            p.TIM1,
            None,
            None,
            None,
            Some(ComplementaryPwmPin::new_ch2(p.PE10, OutputType::PushPull)),
            None,
            None,
            None,
            None,
            Hertz::khz(2),
            Default::default(),
        );
        bl.enable(Channel::Ch2);
        bl.set_duty(Channel::Ch2, bl.get_max_duty() / 2);
        (bl, Channel::Ch2)
    };
    let beep = SimplePwm::new(
        p.TIM3,
        None,
        None,
        None,
        Some(PwmPin::new_ch4(p.PB1, OutputType::PushPull)),
        Hertz::hz(523),
        Default::default(),
    );
    // let mut buzzer = Buzzer::new(beep, Channel::Ch4, 50);
    // buzzer.beep().await;

    // init keyboard
    #[cfg(feature = "stm32f103vc")]
    let kbd_drv = {
        let stb = Output::new(p.PE2, Level::Low, Speed::Low);
        let dio = DioPin {
            pin: Flex::new(p.PE4),
        };
        let clk = Output::new(p.PE3, Level::Low, Speed::Low);
        let kbd = tm1668::TM1668::new(stb, clk, dio, delay);
        let kbd_drv = KeyboardDriver::new(kbd);
        kbd_drv
    };
    #[cfg(feature = "stm32h743vi")]
    let kbd_drv = app::devices::DummyKeyboardDevice {};

    // for region in embassy_stm32::flash::get_flash_regions() {
    //     defmt::info!("region: {:?}", region);
    // }
    // let region = embassy_stm32::flash::get_flash_regions()[0];
    // let state_max = 4096;
    // defmt::assert!(core::mem::size_of::<app::State>() <= state_max);
    // let offset = region.size - state_max as u32;
    // // let offset = 1024 * 64;
    // let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH)
    //     .into_blocking_regions()
    //     .bank1_region;

    // let adc_device = app::devices::DummyAdcDevice {};
    // // let adc_device = SimpleAdcDevice::new(PeripheralRef::new(p.ADC1), p.PA1, p.PA2);
    // let mut power_key_test = Flex::new(p.PE1);
    // power_key_test.set_as_input(Pull::Down);
    // let power_on = Output::new(p.PE0, Level::High, Speed::Low);
    // let board_device = BoardDriver::new(bl, bl_channel, flash, offset, power_key_test, power_on);
    // let buzzer_device = BuzzerDriver::new(beep, Channel::Ch4);
    // app::main_loop(
    //     spawner,
    //     lcd,
    //     board_device,
    //     buzzer_device,
    //     kbd_drv,
    //     adc_device,
    //     |_| {},
    // )
    // .await;
    // defmt::panic!("unreachable");
}

struct SimpleAdcDevice<ADC, A, B> {
    adc: ADC,
    channels: (A, B),
}

impl<ADC, A, B> SimpleAdcDevice<ADC, A, B> {
    fn new(adc: ADC, channel_a: A, channel_b: B) -> Self {
        Self {
            adc,
            channels: (channel_a, channel_b),
        }
    }
}

#[cfg(feature = "stm32f103vc")]
impl<ADC, A, B> app::devices::AdcDevice for SimpleAdcDevice<ADC, A, B>
where
    ADC: Peripheral,
    A: embassy_stm32::adc::AdcPin<ADC::P>,
    B: embassy_stm32::adc::AdcPin<ADC::P>,
    <ADC as Peripheral>::P: embassy_stm32::adc::Instance,
{
    async fn read(
        &mut self,
        options: app::devices::AdcReadOptions,
        buf: &mut [f32],
    ) -> app::Result<usize> {
        // let length = app::input::ADC_BUF_SZ.min(options.length - options.pos);
        let mut adc = embassy_stm32::adc::Adc::new(&mut self.adc, &mut Delay);
        let mut vrefint = adc.enable_vref(&mut Delay);
        let vrefint_sample = adc.read(&mut vrefint).await;
        let convert_to_millivolts = |sample| {
            // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
            // 5.3.4 Embedded reference voltage
            const VREFINT_MV: u32 = 1200; // mV

            (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
        };
        // let convert_to_millivolts = |sample| sample as u16;
        let mut it = buf.iter_mut();
        // for i in 0..length {
        //     let v = adc.read(&mut self.channels.0).await;
        //     let v = convert_to_millivolts(v);
        //     data[i] = v as f32 / 1000.0;
        //     // defmt::info!("{}: {}", i, v);
        //     Timer::after_micros(1).await;
        // }
        // Ok(data)
        let mut count = 0usize;
        while let Some(data) = it.next() {
            let v = if options.channel == app::ProbeChannel::A {
                adc.read(&mut self.channels.0).await
            } else {
                adc.read(&mut self.channels.1).await
            };
            let v = convert_to_millivolts(v);
            *data = v as f32 / 1000.0;
            // Timer::after_micros(1).await;
            count += 1;
        }
        Ok(count)
    }
}

struct BuzzerDriver<'d, T> {
    pwm: SimplePwm<'d, T>,
    channel: Channel,
}
impl<'d, T> BuzzerDriver<'d, T>
where
    T: CaptureCompare16bitInstance,
{
    fn init(mut self) -> Self {
        self.pwm.set_duty(self.channel, self.pwm.get_max_duty() / 2);
        self
    }
    pub fn new(pwm: SimplePwm<'d, T>, channel: Channel) -> Self {
        Self { pwm, channel }.init()
    }
}
impl<'d, T> BuzzerDevice for BuzzerDriver<'d, T>
where
    T: CaptureCompare16bitInstance,
{
    async fn beep(&mut self, frequency: u32, duration_ms: u32) {
        if frequency == 0 {
            self.pwm.disable(self.channel);
        } else {
            self.pwm.set_frequency(Hertz::hz(frequency));
            self.pwm.enable(self.channel);
            if duration_ms != 0 {
                Timer::after(embassy_time::Duration::from_millis(duration_ms as u64)).await;
                self.pwm.disable(self.channel);
            }
        }
    }
}

struct BoardDriver<PWM, C, F, P, W> {
    backlight: PWM,
    backlight_channel: C,
    flash: F,
    flash_offset: u32,
    power_key_test: P,
    power_on: W,
}

impl<PWM, F, P, W> BoardDriver<PWM, Channel, F, P, W>
where
    PWM: embedded_hal_02::Pwm,
{
    fn new(
        backlight: PWM,
        backlight_channel: Channel,
        flash: F,
        flash_offset: u32,
        power_key_test: P,
        power_on: W,
    ) -> Self {
        Self {
            backlight,
            backlight_channel,
            flash,
            flash_offset,
            power_key_test,
            power_on,
        }
    }
}

impl<PWM, F, P, W> BoardDevice for BoardDriver<PWM, Channel, F, P, W>
where
    PWM: embedded_hal_02::Pwm<Channel = Channel, Duty = u16>,
    P: InputPin,
    W: OutputPin,
{
    fn set_brightness(&mut self, brightness: u8) {
        self.backlight.set_duty(
            self.backlight_channel,
            brightness.min(100) as u16 * self.backlight.get_max_duty() / 100,
        );
    }
    fn set_power_on(&mut self, on: bool) {
        if on {
            self.power_on.set_high().unwrap();
        } else {
            self.power_on.set_low().unwrap();
        }
    }
    fn read_power_key(&mut self) -> bool {
        self.power_key_test.is_high().unwrap()
    }
    #[cfg(feature = "stm32f103vc")]
    fn has_battery(&self) -> bool {
        true
    }
    #[cfg(feature = "stm32f103vc")]
    fn has_clock(&self) -> bool {
        true
    }
    #[cfg(feature = "stm32f103vc")]
    fn has_keypad(&self) -> bool {
        true
    }
    #[cfg(feature = "stm32f103vc")]
    fn get_battery_percentage(&self) -> u8 {
        50
    }
}

impl<PWM, F, P, W> NvmDevice for BoardDriver<PWM, Channel, F, P, W>
where
    F: NorFlash + ReadNorFlash,
    <F as embedded_storage::nor_flash::ErrorType>::Error: defmt::Format,
{
    fn read(&mut self, address: u32, buf: &mut [u8]) -> app::Result<()> {
        defmt::info!("read at {:x}", address + self.flash_offset);
        self.flash
            .read(address + self.flash_offset, buf)
            .map_err(|_| app::AppError::StorageIOError)
    }

    fn write(&mut self, address: u32, buf: &[u8]) -> app::Result<()> {
        let len = buf.len();
        let blksz = 2048;
        let aligned = ((len + blksz - 1) / blksz) * blksz;
        defmt::info!("write at {:x}, len {}", address + self.flash_offset, len);
        if len != aligned {
            defmt::info!("write at {:x}, len {}", address + self.flash_offset, len);
            let mut buf_vec = alloc::vec::Vec::from(buf);
            buf_vec.resize(aligned, 0);
            defmt::info!("data[0]: {:x}, buf[0]: {:x}", buf[0], buf_vec[0]);
            self.flash
                .write(address + self.flash_offset, &buf_vec)
                .map_err(|_| app::AppError::StorageIOError)
        } else {
            defmt::info!("write at {:x}, len {}", address + self.flash_offset, len);
            self.flash
                .write(address + self.flash_offset, buf)
                .map_err(|_| app::AppError::StorageIOError)
        }
        // self.flash
        //     .write(address + self.flash_offset, buf)
        //     .map_err(|_| app::AppError::StorageIOError)
    }

    fn erase(&mut self, address: u32, len: usize) -> app::Result<()> {
        let blksz = 2048;
        // let aligned = len / blksz * blksz;
        let aligned = ((len + blksz - 1) / blksz) * blksz;
        defmt::info!("erase at {:x}, len {}", address + self.flash_offset, len);
        let from = address + self.flash_offset;
        let to = from + aligned as u32;
        self.flash.erase(from, to).map_err(|e| {
            defmt::info!("erase error: {:?}", e);
            app::AppError::StorageIOError
        })
    }
}
