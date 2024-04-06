#![allow(clippy::empty_loop)]
#![allow(static_mut_refs)]
#![no_main]
#![no_std]

extern crate alloc;

use core::convert::Infallible;

use defmt::*;
use embedded_hal::{delay::DelayNs, digital::OutputPin};

use {defmt_rtt as _, panic_probe as _};

use ili9341::{DisplaySize240x320, Ili9341 as Ili9327, Orientation};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Flex, OutputType, Pull},
    peripherals::ADC1,
    time::Hertz,
    timer::{Channel, GeneralInstance4Channel},
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

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<ADC1>;
});

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();
struct DioPin<'d> {
    pin: Flex<'d>,
}

impl<'d> InoutPin for DioPin<'d> {
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

struct Buzzer<'d, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'d, T>,
    channel: Channel,
    delay_ms: u64,
    freqs: [Hertz; 3],
}
impl<'d, T> Buzzer<'d, T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'d, T>, channel: Channel, delay_ms: u64) -> Self {
        Self {
            pwm,
            channel,
            delay_ms,
            freqs: [Hertz::hz(523), Hertz::hz(659), Hertz::hz(784)],
        }
    }

    pub async fn beep(&mut self) {
        // self.pwm.set_duty(self.channel, self.pwm.get_max_duty() / 2);
        self.pwm.set_duty(self.channel, 0);
        self.pwm.enable(self.channel);
        for f in self.freqs.iter() {
            self.pwm.set_frequency(*f);
            Timer::after_millis(self.delay_ms).await;
        }
        self.pwm.disable(self.channel);
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
impl<'d, S, C, D, DELAY> app::input::KeyboardDevice for KeyboardDriver<'d, S, C, D, DELAY>
where
    S: OutputPin<Error = Infallible>,
    C: OutputPin<Error = Infallible>,
    D: InoutPin,
    DELAY: DelayNs,
{
    fn read_key(&mut self) -> app::input::Keys {
        let mut keys = [false; 20];
        self.driver.read_decode_keys(&mut keys);
        let mut r = app::input::Keys::None;
        for (i, k) in keys.iter().enumerate() {
            // read key up
            if !*k && self.keys[i] {
                r = app::input::Keys::try_from(i).unwrap();
                break;
            }
        }
        self.keys = keys;
        r
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
                source: PllSource::HSI,
                prediv: PllPreDiv::DIV4,
                mul: PllMul::MUL60,
                divp: Some(PllDiv::DIV2),
                divq: None,
                divr: None,
            });
        }
        #[cfg(feature = "stm32f103vc")]
        {
            config.rcc.hse = Some(Hse {
                freq: Hertz(8_000_000),
                mode: HseMode::Oscillator,
            });
            #[cfg(not(feature = "overclocking"))]
            {
                config.rcc.pll = Some(Pll {
                    src: PllSource::HSE,
                    prediv: PllPreDiv::DIV1,
                    mul: PllMul::MUL9,
                });
            }
            #[cfg(feature = "overclocking")]
            {
                config.rcc.pll = Some(Pll {
                    src: PllSource::HSE,
                    prediv: PllPreDiv::DIV1,
                    // overclocking to 8 * 16 = 128 MHz
                    mul: PllMul::MUL16,
                });
            }
        }
        config.rcc.sys = Sysclk::PLL1_P;
        #[cfg(feature = "stm32f103vc")]
        {
            config.rcc.apb2_pre = APBPrescaler::DIV1;
            #[cfg(feature = "overclocking")]
            {
                config.rcc.ahb_pre = AHBPrescaler::DIV2;
                config.rcc.apb1_pre = APBPrescaler::DIV2;
                // overclocking ADC 16MHz, note that the max ADC clock is 14MHz
                config.rcc.adc_pre = ADCPrescaler::DIV4;
            }
            #[cfg(not(feature = "overclocking"))]
            {
                config.rcc.ahb_pre = AHBPrescaler::DIV1;
                config.rcc.apb1_pre = APBPrescaler::DIV2;
                // ADC 72 / 6 = 12 MHz
                config.rcc.adc_pre = ADCPrescaler::DIV6;
            }
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

    // let mut delay = Delay {};
    static mut DELAY: Delay = Delay {};

    info!("System launched!");

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 6 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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
        unsafe { &mut DELAY },
        Orientation::LandscapeFlipped,
        // Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
    // .await
    .unwrap();
    lcd.clear_screen(0).unwrap();
    info!("OK!");

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
    let beep = SimplePwm::new(
        p.TIM3,
        None,
        None,
        None,
        Some(PwmPin::new_ch4(p.PB1, OutputType::PushPull)),
        Hertz::hz(523),
        Default::default(),
    );
    let mut buzzer = Buzzer::new(beep, Channel::Ch4, 50);
    buzzer.beep().await;

    // init keyboard
    let stb = Output::new(p.PE2, Level::Low, Speed::Low);
    let dio = DioPin {
        pin: Flex::new(p.PE4),
    };
    let clk = Output::new(p.PE3, Level::Low, Speed::Low);

    let kbd = tm1668::TM1668::new(stb, clk, dio, unsafe { &mut DELAY });
    let kbd_drv = KeyboardDriver::new(kbd);

    // spawner.spawn(app::main_loop(lcd, kbd_drv)).unwrap();

    // pac::RCC.ahbenr().modify(|w| {
    //     w.set_dma1en(true);
    //     w.set_dma2en(true);
    // });
    // pac::RCC.apb2enr().modify(|w| {
    //     w.set_adc1en(true);
    //     w.set_adc2en(true);
    // });

    // let mut adc = embassy_stm32::adc::Adc::new(p.ADC1, &mut Delay);
    // let mut pin = p.PA1;

    // let mut vrefint = adc.enable_vref(&mut Delay);
    // let vrefint_sample = adc.read(&mut vrefint).await;
    // let convert_to_millivolts = |sample: u16| {
    //     // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
    //     // 5.3.4 Embedded reference voltage
    //     const VREFINT_MV: u32 = 1200; // mV

    //     (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
    // };
    // let v = adc.read(&mut pin).await;
    // info!("--> {} - {} mV", v, convert_to_millivolts(v));

    // use pac::adc::regs::*;
    // pac::ADC1.sr().write_value(Sr(0x00000010));
    // pac::ADC1.cr1().write_value(Cr1(0x00000000));
    // // ADC1, CR2, 0x001E0103, ADON: 0b1; CONT: 0b1; CAL: 0b0; RSTCAL: 0b0; DMA: 0b1; ALIGN: 0b0; JEXTSEL: 0b000; JEXTTRIG: 0b0; EXTSEL: 0b111; EXTTRIG: 0b1; JSWSTART: 0b0; SWSTART: 0b0; TSVREFE: 0b0
    // // pac::ADC1.cr2().write_value(Cr2(0x001E0103));
    // pac::ADC1.cr2().write(|w| {
    //     w.set_adon(false);
    //     w.set_cont(true);
    //     w.set_cal(false);
    //     w.set_rstcal(false);
    //     w.set_dma(false);
    //     w.set_align(false);
    //     w.set_jextsel(0);
    //     w.set_jexttrig(false);
    //     w.set_extsel(0b111);
    //     w.set_exttrig(true);
    //     w.set_jswstart(false);
    //     w.set_swstart(false);
    //     w.set_tsvrefe(false);
    // });
    // pac::ADC1.smpr1().write_value(Smpr1(0x00000000));
    // pac::ADC1.smpr2().write_value(Smpr2(0x00000000));
    // pac::ADC1.htr().write_value(Htr(0x00000FFF));
    // pac::ADC1.ltr().write_value(Ltr(0x00000000));
    // pac::ADC1.sqr1().write_value(Sqr1(0x00000000));
    // pac::ADC1.sqr2().write_value(Sqr2(0x00000000));
    // pac::ADC1.sqr3().write_value(Sqr3(0x00000001));
    // pac::ADC1.jsqr().write_value(Jsqr(0x00000000));

    // // use continuous conversion mode
    // // pac::ADC1.cr2().modify(|w| w.set_adon(false));
    // // pac::ADC1.cr1().modify(|w| {
    // //     w.set_scan(false);
    // //     w.set_discen(false);
    // // });
    // // pac::ADC1.cr2().modify(|w| {
    // //     w.set_cont(true);
    // //     w.set_align(false);
    // // });
    // // pac::ADC1.cr2().modify(|w| w.set_adon(true));
    // // pac::ADC1.sr().write_value(Sr(0x00000010));
    // let mut values = [0u16; 12];
    // // let mut options: embassy_stm32::dma::TransferOptions = Default::default();
    // // options.circular = true;
    // // options.half_transfer_ir = true;
    // // options.priority = embassy_stm32::dma::Priority::High;
    // // let transfer = unsafe {
    // //     Transfer::new_read_raw(
    // //         p.DMA1_CH1,
    // //         (),
    // //         pac::ADC1.dr().as_ptr() as *mut u16,
    // //         &mut values as *mut [u16],
    // //         options,
    // //     )
    // // };

    // pac::ADC1.cr2().modify(|w| {
    //     w.set_adon(true);
    //     w.set_dma(true);
    // });

    // // DMA1, ISR, 0x00000007, GIF1: 0b1; TCIF1: 0b1; HTIF1: 0b1; TEIF1: 0b0; GIF2: 0b0; TCIF2: 0b0; HTIF2: 0b0; TEIF2: 0b0; GIF3: 0b0; TCIF3: 0b0; HTIF3: 0b0; TEIF3: 0b0; GIF4: 0b0; TCIF4: 0b0; HTIF4: 0b0; TEIF4: 0b0; GIF5: 0b0; TCIF5: 0b0; HTIF5: 0b0; TEIF5: 0b0; GIF6: 0b0; TCIF6: 0b0; HTIF6: 0b0; TEIF6: 0b0; GIF7: 0b0; TCIF7: 0b0; HTIF7: 0b0; TEIF7: 0b0
    // // DMA1, IFCR, <Write-Only>, CGIF1: -; CTCIF1: -; CHTIF1: -; CTEIF1: -; CGIF2: -; CTCIF2: -; CHTIF2: -; CTEIF2: -; CGIF3: -; CTCIF3: -; CHTIF3: -; CTEIF3: -; CGIF4: -; CTCIF4: -; CHTIF4: -; CTEIF4: -; CGIF5: -; CTCIF5: -; CHTIF5: -; CTEIF5: -; CGIF6: -; CTCIF6: -; CHTIF6: -; CTEIF6: -; CGIF7: -; CTCIF7: -; CHTIF7: -; CTEIF7: -
    // // DMA1, CCR1, 0x000025AF, EN: 0b1; TCIE: 0b1; HTIE: 0b1; TEIE: 0b1; DIR: 0b0; CIRC: 0b1; PINC: 0b0; MINC: 0b1; PSIZE: 0b01; MSIZE: 0b01; PL: 0b10; MEM2MEM: 0b0
    // // DMA1, CNDTR1, 0x0000004B, NDT: 0b0000000001001011
    // // DMA1, CPAR1, 0x4001244C, PA: 0b01000000000000010010010001001100
    // // DMA1, CMAR1, 0x2000BF30, MA: 0b00100000000000001011111100110000

    // pac::DMA1.ch(0).cr().write(|w| {
    //     w.set_en(false);
    //     w.set_tcie(true);
    //     w.set_htie(true);
    //     w.set_teie(true);
    //     w.set_dir(pac::bdma::vals::Dir::FROMPERIPHERAL);
    //     w.set_circ(true);
    //     w.set_pinc(false);
    //     w.set_minc(true);
    //     w.set_psize(pac::bdma::vals::Size::BITS16);
    //     w.set_msize(pac::bdma::vals::Size::BITS16);
    //     w.set_pl(pac::bdma::vals::Pl::HIGH);
    //     w.set_mem2mem(false);
    // });
    // pac::DMA1
    //     .ch(0)
    //     .ndtr()
    //     .write(|w| w.set_ndt(values.len() as _));
    // pac::DMA1.ch(0).par().write_value(0x4001244C);
    // pac::DMA1
    //     .ch(0)
    //     .mar()
    //     .write_value(&values as *const u16 as u32);
    // pac::DMA1.ch(0).cr().modify(|w| w.set_en(true));
    // info!("start continuous conversion");
    // // transfer.await;
    // // info!("done, values: {:?}", values);
    // Timer::after_millis(1000).await;
    // for v in values {
    //     info!("value: {:x}", v);
    // }

    // loop {}

    // let adc_device = DummyAdcDevice {};
    let adc_device = SimpleAdcDevice::new(PeripheralRef::new(p.ADC1), p.PA1, p.PA2);
    // let adc_device = app::input::DummyAdcDevice {};
    app::main_loop(spawner, lcd, kbd_drv, adc_device, |_| {}).await;
    defmt::panic!("unreachable");

    // loop {
    //     Timer::after_millis(10000).await;
    //     // debug!("main loop");
    // }
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

impl<ADC, A, B> app::input::AdcDevice for SimpleAdcDevice<ADC, A, B>
where
    ADC: Peripheral<P: embassy_stm32::adc::Instance>,
    A: embassy_stm32::adc::AdcPin<ADC::P>,
    B: embassy_stm32::adc::AdcPin<ADC::P>,
{
    async fn read(
        &mut self,
        _options: app::input::AdcReadOptions,
        buf: &mut [f32],
    ) -> app::Result<usize> {
        // let length = app::input::ADC_BUF_SZ.min(options.length - options.pos);
        let mut adc = embassy_stm32::adc::Adc::new(&self.adc, &mut Delay);
        let mut vrefint = adc.enable_vref(&mut Delay);
        let vrefint_sample = adc.read(&mut vrefint).await;
        let convert_to_millivolts = |sample| {
            // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
            // 5.3.4 Embedded reference voltage
            const VREFINT_MV: u32 = 1200; // mV

            (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
        };
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
            let v = adc.read(&mut self.channels.0).await;
            let v = convert_to_millivolts(v);
            *data = v as f32 / 1000.0;
            // Timer::after_micros(1).await;
            count += 1;
        }
        Ok(count)
    }
}
