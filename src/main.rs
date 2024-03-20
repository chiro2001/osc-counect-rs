//! CRC calculation

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

mod fsmc;

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1::stm32f103 as stm32;
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
    let mut hsram = fsmc::SramHandleTypeDef { device: fsmc, init };
    fsmc::hal_sram_init(&mut hsram, &timing, &ext_timing);
    loop {}
}
