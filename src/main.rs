//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate alloc;

use panic_halt as _;

use core::time::Duration;
use cortex_m_rt::entry;
use cstr_core::CString;
use embedded_graphics_core::prelude::*;
use embedded_graphics_core::primitives::Rectangle;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use lvgl::{Align, Animation, Color, Display, DrawBuffer, Event, Part, Widget};
use lvgl::style::Style;
use lvgl::widgets::{Bar, Label};
use stm32f1xx_hal::{pac, prelude::*, rcc};
use stm32f1xx_hal::rcc::Enable;

use display_interface_fsmc as fsmc;
#[cfg(feature = "custom-alloc")]
use embedded_alloc_c::*;

#[entry]
fn main() -> ! {
    #[cfg(feature = "custom-alloc")]
    heap_init!(32 * 1024);
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

    // // Initialize the allocator BEFORE you use it
    // {
    //     use core::mem::MaybeUninit;
    //     const HEAP_SIZE: usize = 8 * 1024;
    //     // const HEAP_SIZE: usize = (320 * 240 * 2 / 4 / 8);
    //     static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    //     unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    // }

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
        Orientation::LandscapeFlipped,
        // Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
    .unwrap();
    // Create a new character style
    // let style = MonoTextStyle::new(&FONT_6X9, Rgb565::new(0, 255, 255));

    let mut cnt = 0u32;
    let cnt_time = 2000;
    timer.start(cnt_time.millis()).unwrap();
    // lcd.clear(Rgb565::new(0, 0, 0)).unwrap();
    let mut last = 0xffffffu32;
    // let font_height = FONT_6X9.character_size.height;
    // let update_rect = Rectangle::new(
    //     Point::new(0, font_height as i32),
    //     Size::new(lcd.width() as u32, lcd.height() as u32 - font_height),
    // );
    // let text_rect = Rectangle::new(Point::new(0, 0), Size::new(lcd.width() as u32, font_height));

    unsafe {
        lvgl_sys::lv_init();
    }

    const BUFFER_SZ: usize = 320 * 10;

    let buffer = DrawBuffer::<BUFFER_SZ>::default();
    let display = Display::register(buffer, lcd.width() as u32, lcd.height() as u32, |refresh| {
        let area = &refresh.area;
        let rc = Rectangle::new(
            Point::new(area.x1 as i32, area.y1 as i32),
            Size::new(
                (area.x2 - area.x1 + 1) as u32,
                (area.y2 - area.y1 + 1) as u32,
            ),
        );
        lcd.fill_contiguous(&rc, refresh.colors.into_iter().map(|p| p.into()))
            .unwrap();
    })
    .unwrap();

    // loop {
    //     lcd.fill_solid(
    //         &update_rect,
    //         Rgb565::new(0, 0, if cnt % 2 == 0 { 0xff } else { 0 }),
    //     )
    //     .unwrap();
    //     cnt += 1;
    //     let now = timer.now().ticks();
    //     if now < last {
    //         let mut buf = [0u8; 64];
    //         let s = format_no_std::show(
    //             &mut buf,
    //             format_args!("Hello Rust! fps={}", cnt * 1000 / cnt_time),
    //         )
    //         .unwrap();
    //         cnt = 0;
    //         let text = Text::with_alignment(
    //             &s,
    //             Point::new(0, (font_height / 2 + 1) as i32),
    //             style,
    //             Alignment::Left,
    //         );
    //         lcd.fill_solid(&text_rect, Rgb565::new(0, 0, 0)).unwrap();
    //         text.draw(&mut lcd).unwrap();
    //     }
    //     last = now;
    // }

    let mut screen = display.get_scr_act().unwrap();

    let mut screen_style = Style::default();
    screen_style.set_bg_color(Color::from_rgb((255, 255, 255)));
    screen_style.set_radius(0);
    screen.add_style(Part::Main, &mut screen_style).unwrap();

    // Create the bar object
    let mut bar = Bar::create(&mut screen).unwrap();
    bar.set_size(175, 20).unwrap();
    bar.set_align(Align::Center, 0, 0).unwrap();
    bar.set_range(0, 100).unwrap();
    bar.on_event(|_b, _e| {
        // println!("Completed!");
        // lcd.clear(Rgb565::new(255, 255, 0)).unwrap();
    })
    .unwrap();

    // Set the indicator style for the bar object
    let mut ind_style = Style::default();
    ind_style.set_bg_color(Color::from_rgb((100, 245, 100)));
    bar.add_style(Part::Any, &mut ind_style).unwrap();

    let mut loading_lbl = Label::create(&mut screen).unwrap();
    loading_lbl
        .set_text(CString::new("Testing bar...").unwrap().as_c_str())
        .unwrap();
    loading_lbl.set_align(Align::OutTopMid, 0, 20).unwrap();

    let mut loading_style = Style::default();
    loading_style.set_text_color(Color::from_rgb((0, 0, 0)));
    loading_lbl
        .add_style(Part::Main, &mut loading_style)
        .unwrap();

    let mut i = 0;
    loop {
        // lcd.clear(Rgb565::new(0, 255, 0)).unwrap();
        // let start = Instant::now();
        let start = timer.now().ticks();
        if i > 100 {
            i = 0;
            lvgl::event_send(&mut bar, Event::Clicked).unwrap();
        }
        bar.set_value(i, Animation::ON).unwrap();
        i += 1;
        cnt += 1;

        lvgl::task_handler();
        // window.update(&shared_native_display.borrow());
        //
        // for event in window.events() {
        //     match event {
        //         SimulatorEvent::Quit => break 'running,
        //         _ => {}
        //     }
        // }
        // sleep(Duration::from_millis(15));
        // delay.delay_ms(8u16);
        let now = timer.now().ticks();
        let duration = if now >= start {
            now - start
            // lvgl::tick_inc(Duration::from_millis((now - start) as u64));
        } else {
            // overflow
            // lvgl::tick_inc(Duration::from_millis((start + cnt_time - now) as u64));
            start + cnt_time - now
        };
        lvgl::tick_inc(Duration::from_millis(duration as u64));
        // lvgl::tick_inc(Duration::from_millis(15));

        if now < last {
            let mut buf = [0u8; 64];
            let s = format_no_std::show(
                &mut buf,
                format_args!("Hello lv_binding_rust! fps={}", cnt * 1000 / cnt_time),
            )
                .unwrap();
            cnt = 0;
            loading_lbl.set_text(CString::new(s).unwrap().as_c_str()).unwrap();
        }
        last = now;
    }
}
