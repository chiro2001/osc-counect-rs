//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate alloc;

use cortex_m as _;
use panic_halt as _;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::RefCell;
use core::ops::{Deref, Range};
use core::time::Duration;

use cortex_m_rt::entry;
use embedded_alloc::Heap;
use embedded_graphics::Drawable;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::prelude::*;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use slint::platform::{Platform, software_renderer as renderer, WindowAdapter};
use renderer::Rgb565Pixel;
use slint::PlatformError;
use stm32f1xx_hal::{pac, prelude::*, rcc};
use stm32f1xx_hal::rcc::Enable;
use stm32f1xx_hal::timer::CounterMs;

use display_interface_fsmc as fsmc;

slint::include_modules!();

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

macro_rules! heap_init {
    ($SZ:expr) => {
        static mut HEAP_MEM: [core::mem::MaybeUninit<u8>; $SZ] =
            [core::mem::MaybeUninit::uninit(); $SZ];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, $SZ) }
    };
}

pub type TargetPixel = Rgb565Pixel;
// pub type TargetPixel = u16;

struct DrawBuffer<Display> {
    pub display: Display,
    pub buffer: &'static mut [TargetPixel],
}

impl<R> renderer::LineBufferProvider for &mut DrawBuffer<Ili9341<fsmc::FsmcInterface, R>> {
    type TargetPixel = TargetPixel;

    fn process_line(
        &mut self,
        line: usize,
        range: Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        render_fn(&mut self.buffer[range.clone()]);
        self.display
            .draw_raw_iter(
                range.start as u16,
                line as u16,
                range.end as u16,
                line as u16 + 1,
                self.buffer[range.clone()].into_iter().map(|x| x.0),
            )
            .unwrap();
    }
}

struct McuBackend<DrawBuffer, TIM> {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
    buffer_provider: RefCell<DrawBuffer>,
    timer: CounterMs<TIM>,
}

impl<R, TIM> Platform for McuBackend<DrawBuffer<Ili9341<fsmc::FsmcInterface, R>>, TIM>
where
    TIM: stm32f1xx_hal::timer::Instance,
{
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
        let window =
            renderer::MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);
        self.window.replace(Some(window.clone()));
        // self.timer.start(2000.millis()).unwrap();
        Ok(window)
    }

    fn run_event_loop(&self) -> Result<(), PlatformError> {
        self.window
            .borrow()
            .as_ref()
            .unwrap()
            .set_size(slint::PhysicalSize::new(
                self.buffer_provider.borrow().display.width() as _,
                self.buffer_provider.borrow().display.height() as _,
            ));
        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = self.buffer_provider.borrow_mut();
                    renderer.render_by_line(&mut *buffer_provider);
                });
            }
        }
    }

    fn duration_since_start(&self) -> Duration {
        let counter = self.timer.now().ticks();
        core::time::Duration::from_micros(counter as u64)
    }
}

// slint::include_modules!();

// slint::slint! {
//     import { HelloWorld } from "ui/simple.slint";
// }

#[entry]
fn main() -> ! {
    heap_init!(52 * 1024);
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
        device: dp.FSMC,
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

    // timer.start(cnt_time.millis()).unwrap();
    timer.start(2000.millis()).unwrap();
    lcd.clear(Rgb565::new(0, 0, 0)).unwrap();

    let width = lcd.width();
    let buffer_provider = DrawBuffer {
        display: lcd,
        buffer: vec![Default::default(); width as _].leak(),
    };
    slint::platform::set_platform(Box::new(McuBackend {
        window: Default::default(),
        buffer_provider: buffer_provider.into(),
        timer,
    }))
    .unwrap();
    // let window = HelloWorld::new().unwrap();
    // let window = MainWindow::new().unwrap();
    let window = Counter::new().unwrap();
    window.run().unwrap();
    loop {}
}
