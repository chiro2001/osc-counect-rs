//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// extern crate alloc;

use core::time::Duration;
use defmt::*;
use embedded_graphics::pixelcolor::Rgb565;
use {defmt_rtt as _, panic_probe as _};

use cstr_core::CString;
use embedded_graphics_core::prelude::*;
use embedded_graphics_core::primitives::Rectangle;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use lvgl::style::Style;
use lvgl::widgets::{Bar, Label};
use lvgl::{Align, Animation, Color, Display, DrawBuffer, Event, Part, Widget};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{pac, Config};
use embassy_time::{Delay, Instant, Timer};

use display_interface_fsmc as fsmc;

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

    info!("System launched!");

    let mut bl_pin = Output::new(p.PC8, Level::High, Speed::Low);
    bl_pin.set_high();

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
    let interface = fsmc::FsmcInterface::new(hsram);
    let rst = Output::new(p.PC9, Level::Low, Speed::Low);
    let mut delay = Delay {};
    let mut lcd = Ili9341::new(
        interface,
        rst,
        &mut delay,
        Orientation::LandscapeFlipped,
        // Orientation::PortraitFlipped,
        DisplaySize240x320,
    )
    .unwrap();
    info!("Clearing...");
    lcd.clear(Rgb565::new(0, 0, 0)).unwrap();
    info!("OK!");

    let mut cnt = 0u32;
    let cnt_time = 2000;

    unsafe {
        lvgl_sys::lv_init();
    }

    const BUFFER_SZ: usize = 320 * 6;

    let buffer = DrawBuffer::<BUFFER_SZ>::default();
    let display = Display::register(buffer, lcd.width() as u32, lcd.height() as u32, |refresh| {
        let area = &refresh.area;

        defmt::assert!(area.x2 > area.x1);
        defmt::assert!(area.y2 > area.y1);
        let rc = Rectangle::new(
            Point::new(area.x1 as i32, area.y1 as i32),
            Size::new(
                (area.x2 - area.x1 + 1) as u32,
                (area.y2 - area.y1 + 1) as u32,
            ),
        );
        // lcd.fill_contiguous(&rc, refresh.colors.into_iter().map(|p| p.into()))
        //     .unwrap();
        let len = refresh.colors.len();
        let data_ptr = refresh.colors.as_ptr() as *const u16;
        lcd.fill_slice(&rc, unsafe { core::slice::from_raw_parts(data_ptr, len) })
            .unwrap();

        // let len = refresh.colors.len();
        // let data_ptr = refresh.colors.as_ptr() as *const u16;
        // lcd.draw_raw_slice(
        //     area.x1 as u16,
        //     area.y1 as u16,
        //     area.x2 as u16,
        //     area.y2 as u16,
        //     unsafe { core::slice::from_raw_parts(data_ptr, len) },
        // )
        // .unwrap();

        // lcd.draw_raw_iter(
        //     area.x1 as u16,
        //     area.y1 as u16,
        //     area.x2 as u16,
        //     area.y2 as u16,
        //     refresh.colors.into_iter().map(|p| {
        //         let b = Rgb565::from(p).to_be_bytes();
        //         b[1] as u16 | ((b[0] as u16) << 8u16)
        //     }),
        // )
        // .unwrap();
    })
    .unwrap();

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
        // info!("Completed!");
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
    let mut last = Instant::now().as_ticks();
    loop {
        let start = Instant::now().as_millis();
        if i > 100 {
            i = 0;
            lvgl::event_send(&mut bar, Event::Clicked).unwrap();
        }
        bar.set_value(i, Animation::ON).unwrap();
        i += 1;
        cnt += 1;

        lvgl::task_handler();
        // Timer::after_millis(1).await;
        let now = Instant::now().as_millis();
        let duration = now - start;
        lvgl::tick_inc(Duration::from_millis(duration as u64));
        debug!("duration: {}, now: {}, last: {}", duration, now, last);

        if now >= last {
            if now - last > cnt_time {
                let mut buf = [0u8; 64];
                let fps = (cnt * 1000) as u64 / cnt_time;
                let s = format_no_std::show(
                    &mut buf,
                    format_args!("Hello lv_binding_rust! fps={}", fps),
                )
                .unwrap();
                info!("fps {}, output: {}", fps, s);
                cnt = 0;
                loading_lbl
                    .set_text(CString::new(s).unwrap().as_c_str())
                    .unwrap();
                last = now;
            }
        } else {
            last = now;
        }
    }
}
