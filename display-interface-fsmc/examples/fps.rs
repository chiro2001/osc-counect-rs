//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::Drawable;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::prelude::*;
use embedded_graphics_core::primitives::Rectangle;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use stm32f1xx_hal::rcc::Enable;
use stm32f1xx_hal::{pac, prelude::*, rcc};
use display_interface_fsmc as fsmc;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let _cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    pac::GPIOE::enable(&dp.RCC);
    pac::GPIOD::enable(&dp.RCC);
    pac::GPIOC::enable(&dp.RCC);
    pac::GPIOA::enable(&dp.RCC);
    pac::FSMC::enable(&dp.RCC);
    pac::TIM1::enable(&dp.RCC);
    pac::TIM2::enable(&dp.RCC);
    pac::TIM3::enable(&dp.RCC);

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    pac::NVIC::unpend(pac::Interrupt::FSMC);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // Alternative configuration using dividers and multipliers directly
    let clocks = rcc.cfgr.freeze_with_config(
        rcc::Config {
            hse: Some(8_000_000),
            pllmul: Some(7),
            hpre: rcc::HPre::Div1,
            ppre1: rcc::PPre::Div4,
            ppre2: rcc::PPre::Div1,
            usbpre: rcc::UsbPre::Div15,
            adcpre: rcc::AdcPre::Div6,
        },
        &mut flash.acr,
    );

    // Configure the syst timer to trigger an update every second
    let mut timer = dp.TIM3.counter_ms(&clocks);

    // test gpio led
    let mut gpioc = dp.GPIOC.split();
    // Configure gpio C pin 8 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut bl = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);

    bl.set_high();

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
        address_hold_time: 15,
        data_setup_time: 15,
        bus_turn_around_duration: 0,
        clk_division: 16,
        data_latency: 17,
        access_mode: 0,
    };
    let ext_timing = fsmc::FsmcNorsramTimingTypeDef {
        address_setup_time: 0,
        address_hold_time: 15,
        data_setup_time: 1,
        bus_turn_around_duration: 0,
        clk_division: 16,
        data_latency: 17,
        access_mode: 0,
    };
    let hsram = fsmc::SramHandleTypeDef {
        device: &dp.FSMC,
        init,
        timing,
        ext_timing,
    };
    let interface = fsmc::FsmcInterface::new(hsram, dp.GPIOE, dp.GPIOD);
    let mut afio = dp.AFIO.constrain();
    afio.mapr2.mapr2().modify(|_, w| w.fsmc_nadv().set_bit());
    let rst = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);
    let mut delay = dp.TIM2.delay_us(&clocks);
    let mut lcd = Ili9341::new(
        interface,
        rst,
        &mut delay,
        // Orientation::LandscapeFlipped,
        Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
        .unwrap();
    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X9, Rgb565::new(0, 255, 255));

    let mut cnt = 0;
    let cnt_time = 2000;
    timer.start(cnt_time.millis()).unwrap();
    lcd.clear(Rgb565::new(0, 0, 0)).unwrap();
    let mut last = 0xffffffu32;
    let font_height = FONT_6X9.character_size.height;
    let update_rect = Rectangle::new(
        Point::new(0, font_height as i32),
        Size::new(lcd.width() as u32, lcd.height() as u32 - font_height),
    );
    let text_rect = Rectangle::new(Point::new(0, 0), Size::new(lcd.width() as u32, font_height));

    loop {
        lcd.fill_solid(
            &update_rect,
            Rgb565::new(0, 0, if cnt % 2 == 0 { 0xff } else { 0 }),
        )
            .unwrap();
        cnt += 1;
        let now = timer.now().ticks();
        if now < last {
            let mut buf = [0u8; 64];
            let s = format_no_std::show(&mut buf, format_args!("Hello Rust! fps={}", cnt * 1000 / cnt_time)).unwrap();
            cnt = 0;
            let text = Text::with_alignment(
                &s,
                Point::new(0, (font_height / 2 + 1) as i32),
                style,
                Alignment::Left,
            );
            lcd.fill_solid(&text_rect, Rgb565::new(0, 0, 0)).unwrap();
            text.draw(&mut lcd).unwrap();
        }
        last = now;
    }
}
