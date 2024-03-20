//! CRC calculation

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

mod fsmc;

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::text::{Alignment, Text};
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // test gpio led
    let mut gpioc = dp.GPIOC.split();
    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);

    led.set_high();

    // let mut peripherals = stm32::Peripherals::take().unwrap();
    // let fsmc: &stm32f1::stm32f103::FSMC = &peripherals.FSMC;
    let fsmc = &dp.FSMC;
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
        extended_mode: 0x400,
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
        device: fsmc,
        init,
        timing,
        ext_timing,
    };
    let interface = fsmc::FsmcInterface::new(hsram, dp.GPIOE, dp.GPIOD);
    let mut rst = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);
    let mut delay = dp.TIM2.delay_us(&clocks);
    rst.set_low();
    delay.delay_ms(100u16);
    rst.set_high();
    let mut lcd = Ili9341::new(
        interface,
        rst,
        &mut delay,
        Orientation::PortraitFlipped,
        DisplaySize240x320,
    ).unwrap();
    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::new(1, 1, 1));

    // Create a text at position (20, 30) and draw it using the previously defined style
    Text::with_alignment(
        "First line\nSecond line",
        Point::new(20, 30),
        style,
        Alignment::Center,
    )
        .draw(&mut lcd)
        .unwrap();
    loop {}
}
