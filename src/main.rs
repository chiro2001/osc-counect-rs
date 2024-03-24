//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate alloc;

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

use alloc::vec::Vec;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Flex, OutputType, Pull},
    time::Hertz,
    timer::{CaptureCompare16bitInstance, Channel},
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_stm32::{pac, Config};
use embassy_time::{Delay, Instant, Timer};

use display_interface_fsmc as fsmc;

use tm1668::InoutPin;

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

    // let mut bl_pin = Output::new(p.PC8, Level::High, Speed::Low);
    // bl_pin.set_high();
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
    // beep.set_duty(Channel::Ch4, beep.get_max_duty() / 2);
    // let do_beep = || {
    //     beep.enable(Channel::Ch4);
    //     beep.set_frequency(Hertz::hz(523));
    //     Timer::after_millis(50).await;
    //     beep.set_frequency(Hertz::hz(659));
    //     Timer::after_millis(50).await;
    //     beep.set_frequency(Hertz::hz(784));
    //     Timer::after_millis(50).await;
    //     beep.disable(Channel::Ch4);
    // };
    // do_beep();
    let mut buzzer = Buzzer::new(beep, Channel::Ch4, 50);
    buzzer.beep().await;

    // init keyboard
    let stb = Output::new(p.PE2, Level::Low, Speed::Low);
    let dio = DioPin {
        pin: Flex::new(p.PE4),
    };
    let clk = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut kbd = tm1668::TM1668::new(stb, clk, dio, &mut delay);
    let mut kbd_decoded = [false; 20];

    let mut cnt = 0u32;
    let cnt_time = 2000;

    unsafe {
        lvgl_sys::lv_init();
    }

    const BUFFER_SZ: usize = 320 * 8;

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

    let mut keys_lbl = Label::create(&mut screen).unwrap();
    keys_lbl
        .set_text(CString::new("Testing key...").unwrap().as_c_str())
        .unwrap();
    keys_lbl.set_align(Align::OutTopMid, 0, 40).unwrap();

    let mut style = Style::default();
    style.set_text_color(Color::from_rgb((0, 0, 0)));
    loading_lbl.add_style(Part::Main, &mut style).unwrap();
    let mut last_keys = Vec::new();

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

        kbd.read_decode_keys(&mut kbd_decoded);
        let mut keys = Vec::new();
        let mut has_keys = false;
        for k in 0..kbd_decoded.len() {
            if kbd_decoded[k] {
                // info!("Key {} [{}] pressed", kbd.code_to_key(k), k);
                has_keys = true;
                keys.push(kbd.code_to_key(k));
            }
        }
        let mut buf = [0u8; 64];
        if has_keys {
            if keys != last_keys {
                buzzer.beep().await;
                let s = format_no_std::show(&mut buf, format_args!("keys: {:?}", keys)).unwrap();
                keys_lbl
                    .set_text(CString::new(s).unwrap().as_c_str())
                    .unwrap();
                let s = format_no_std::show(
                    &mut buf,
                    format_args!("keys: {:?}, last_keys: {:?}", keys, last_keys),
                )
                .unwrap();
                info!("{}", s);
                last_keys = keys;
            }
        } else {
            keys_lbl
                .set_text(CString::new("No keys pressed").unwrap().as_c_str())
                .unwrap();
            last_keys.clear();
        }

        lvgl::task_handler();
        Timer::after_millis(100).await;
        let now = Instant::now().as_millis();
        let duration = now - start;
        lvgl::tick_inc(Duration::from_millis(duration as u64));
        debug!("duration: {}, now: {}, last: {}", duration, now, last);

        if now >= last {
            if now - last > cnt_time {
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
