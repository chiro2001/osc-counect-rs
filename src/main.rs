//! FSMC interface test for ILI9341 display

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate alloc;

use defmt::*;
use embassy_futures::join::join;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    Builder,
};
use embassy_usb_dfu::consts::DfuAttributes;
use embassy_usb_driver::EndpointError;
use embedded_graphics::pixelcolor::Rgb565;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics_core::prelude::*;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Flex, OutputType, Pull},
    peripherals,
    time::Hertz,
    timer::{CaptureCompare16bitInstance, Channel},
    usb::{self, Driver},
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

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => usb::InterruptHandler<peripherals::USB>;
});

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

    let kbd = tm1668::TM1668::new(stb, clk, dio, &mut delay);

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // Create embassy-usb Config
    let config = embassy_usb::Config::new(0xc0de, 0xcafe);
    //config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 7];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;

    // embassy_usb_dfu::usb_dfu(
    //     &mut builder,
    //     &mut embassy_usb_dfu::Control::new(
    //         embassy_boot::BlockingFirmwareUpdater::new(),
    //         DfuAttributes::CAN_UPLOAD | DfuAttributes::CAN_DOWNLOAD,
    //     ),
    // );
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: usb::Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
