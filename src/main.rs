//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate alloc;

use core::mem::size_of;

use defmt::*;
use display_interface::DataFormat;
use {defmt_rtt as _, panic_probe as _};

use ili9341::{Command, DisplaySize240x320, Ili9341Async as Ili9327, Orientation};

use embassy_executor::Spawner;
use embassy_stm32::{
    dma::{Transfer, TransferOptions},
    gpio::{Flex, OutputType, Pull},
    pac::bdma::vals::{Dir, Pl, Size},
    time::Hertz,
    timer::{CaptureCompare16bitInstance, Channel},
    PeripheralRef,
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_stm32::{pac, Config};
use embassy_time::{Delay, Timer};

use display_interface_fsmc as fsmc;

use tm1668::InoutPin;

mod osc;

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

struct Buzzer<'d, T> {
    pwm: SimplePwm<'d, T>,
    channel: Channel,
    delay_ms: u64,
    freqs: [Hertz; 3],
}
impl<'d, T> Buzzer<'d, T>
where
    T: CaptureCompare16bitInstance,
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // #[cfg(feature = "custom-alloc")]
    // heap_init!(32 * 1024);
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // Oscillator for bluepill, Bypass for nucleos.
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);
    // let p = embassy_stm32::init(Default::default());

    let mut delay = Delay {};

    info!("System launched!");

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 16 * 1024;
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
        &mut delay,
        Orientation::LandscapeFlipped,
        // Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
    .await
    .unwrap();
    // lcd.brightness(0).unwrap();
    lcd.clear_screen(0).await.unwrap();
    // info!("Clearing...");
    // lcd.clear(Rgb565::new(0, 0, 0)).unwrap();
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

    let _kbd = tm1668::TM1668::new(stb, clk, dio, &mut delay);

    let mut color = 0u16;
    lcd.set_window(0, 0, 320, 240).await.unwrap();
    const SZ: usize = 20 * 320;
    const BUF: [u16; SZ] = [0x5a; SZ];
    static mut DEST: [u16; SZ] = [1u16; SZ];
    unsafe {
        DEST[0] = 0x5a;
    }
    // let mut dma = PeripheralRef::new(p.DMA1_CH1);
    // // let ch = &mut dma;
    // // let req = unsafe { embassy_stm32::peripherals::DMA1_CH1::steal() }.request();
    // let mut option = TransferOptions::default();
    // option.complete_transfer_ir = true;
    // option.priority = embassy_stm32::dma::Priority::Medium;

    // let mut transfer = unsafe {
    //     Transfer::new_write(
    //         &mut dma,
    //         // &mut p.DMA2_CH2,
    //         (),
    //         &BUF,
    //         DEST.as_mut_ptr(),
    //         option,
    //     )
    // };
    // info!("dma starting");
    // transfer.await;
    // transfer.blocking_wait();
    // while transfer.is_running() {
    //     info!("dma running, remaining: {}", transfer.get_remaining_transfers());
    //     Timer::after_millis(100).await;
    // }

    const LCD_FSMC_NEX: u32 = 1;
    const LCD_FSMC_AX: u32 = 16;
    let lcd_base: u32 =
        (0x60000000u32 + (0x4000000u32 * (LCD_FSMC_NEX - 1))) | (((1 << LCD_FSMC_AX) * 2) - 2);
    // pac::DMA2.ch(4).mar().write(|w| *w = 0x2000_0000);
    // pac::DMA2.ch(4).par().write(|w| *w = lcd_base);
    pac::DMA2.ch(4).par().write(|w| {});
    pac::DMA2
        .ch(4)
        .par()
        .modify(|w| *w = unsafe { DEST }.as_ptr() as u32);
    // pac::DMA2.ch(4).ndtr().write(|w| w.set_ndt(BUF.len() as _));
    pac::DMA2.ch(4).mar().modify(|w| *w = BUF.as_ptr() as u32);
    pac::DMA2
        .ch(4)
        .ndtr()
        .modify(|w| w.set_ndt((BUF.len() * size_of::<u16>()) as _));
    pac::DMA2.ch(4).cr().modify(|w| {
        w.set_pl(Pl::HIGH);
    });
    pac::DMA2.ch(4).cr().modify(|w| {
        w.set_dir(Dir::FROMMEMORY);
        w.set_mem2mem(true);
        w.set_msize(Size::BITS16);
        w.set_psize(Size::BITS16);
        w.set_minc(true);
        w.set_pinc(true);

        // w.set_teie(true);
        // w.set_htie(true);
        // w.set_tcie(true);
    });
    info!("dma starting");
    // while pac::DMA2.isr().read().tcif(4) {}
    pac::DMA2.ch(4).cr().modify(|w| w.set_en(true));
    while !pac::DMA2.isr().read().tcif(4) {}
    Timer::after_millis(100).await;
    let isr = pac::DMA2.isr().read();
    info!(
        "dma done, isr gif:{} tcif:{} teif:{} htif:{}",
        isr.gif(4),
        isr.tcif(4),
        isr.teif(4),
        isr.htif(4)
    );
    pac::DMA2.ifcr().write(|w| w.set_tcif(4, false));
    pac::DMA2.ch(4).cr().modify(|w| w.set_en(false));
    info!("dma pass, DEST[0]: {:x}", unsafe { DEST[0] });
    for i in 0..SZ {
        if unsafe { DEST[i] } != BUF[i] {
            defmt::panic!(
                "DMA data error at {}, expected {:x} now {:x}",
                i,
                BUF[i],
                unsafe { DEST[i] }
            );
        }
    }
    info!("dma data checked");
    loop {
        lcd.set_window(0, 0, 320, 240).await.unwrap();
        lcd.command(Command::MemoryWrite, &[]).await.unwrap();
        // for _ in 0..(248 * 320 * 2 / SZ) {
        //     // unsafe { lcd.write_slice(&DEST) }.await.unwrap();
        //     use display_interface::AsyncWriteOnlyDataCommand;
        //     unsafe {
        //         lcd.interface.send_data(DataFormat::U16(&DEST)).await.unwrap();
        //     }
        // }

        pac::DMA2
            .ch(4)
            .mar()
            // .write(|w| *w = (&color) as *const u16 as u32);
            .write(|w| *w = unsafe { DEST }.as_ptr() as u32);
        pac::DMA2.ch(4).par().write(|w| *w = lcd_base as u32);
        pac::DMA2
            .ch(4)
            .ndtr()
            .write(|w| w.set_ndt((320 * 240 / 2 * 1) as u16));
        pac::DMA2.ch(4).cr().write(|w| {
            w.set_pl(Pl::HIGH);
            w.set_dir(Dir::FROMMEMORY);
            // w.set_mem2mem(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_minc(false);
            w.set_pinc(false);
        });
        info!("dma starting");
        pac::DMA2.ch(4).cr().modify(|w| w.set_en(true));
        while !pac::DMA2.isr().read().tcif(4) {
            let remains = pac::DMA2.ch(4).ndtr().read();
            info!("dma running, remaining: {}", remains.ndt());
            // Timer::after_millis(100).await;
        }
        pac::DMA2.ifcr().write(|w| w.set_tcif(4, false));
        pac::DMA2.ch(4).cr().modify(|w| w.set_en(false));

        info!("pass, color={:x}", color);
        color += 1;
        if color > 0xFFF {
            color = 0;
        }
        unsafe {
            for i in 0..SZ {
                DEST[i] = color;
            }
        }
        // info!("Color: {}", color);

        // lcd.clear_screen(color).await.unwrap();

        // Timer::after_millis(100).await;
    }
}
