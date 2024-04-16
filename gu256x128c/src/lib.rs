#![no_std]

pub type DisplayError = display_interface::DisplayError;

pub struct Spi74hc595<S, L, C, R, D> {
    spi: S,
    latch: L,
    cs: C,
    ready: R,
    delay: D,
}
impl<S, L, C, R, D> Spi74hc595<S, L, C, R, D>
where
    S: embedded_hal::spi::SpiBus,
    L: embedded_hal::digital::OutputPin,
    C: embedded_hal::digital::OutputPin,
    R: embedded_hal::digital::InputPin,
    D: embedded_hal::delay::DelayNs,
{
    pub fn new(spi: S, latch: L, cs: C, ready: R, delay: D) -> Self {
        Spi74hc595 {
            spi,
            latch,
            cs,
            ready,
            delay,
        }
    }
    pub fn write_u8(&mut self, data: u8) -> Result<(), DisplayError> {
        // defmt::info!("write_u8({:02x})", data);
        while self.ready.is_low().map_err(|_| DisplayError::DCError)? {
            // self.delay.delay_us(1);
            self.delay.delay_ns(100);
            // self.delay.delay_ms(1000);
            // defmt::info!("waiting for ready");
        }
        self.cs.set_low().map_err(|_| DisplayError::CSError)?;
        self.spi
            .write(&[data])
            .map_err(|_| DisplayError::BusWriteError)?;
        // defmt::info!("spi write!");
        self.delay.delay_us(2);
        self.latch.set_low().map_err(|_| DisplayError::DCError)?;
        self.delay.delay_us(2);
        self.latch.set_high().map_err(|_| DisplayError::DCError)?;
        self.cs.set_high().map_err(|_| DisplayError::CSError)?;
        Ok(())
    }
}
impl<S, L, C, R, D> display_interface::WriteOnlyDataCommand for Spi74hc595<S, L, C, R, D>
where
    S: embedded_hal::spi::SpiBus,
    L: embedded_hal::digital::OutputPin,
    C: embedded_hal::digital::OutputPin,
    R: embedded_hal::digital::InputPin,
    D: embedded_hal::delay::DelayNs,
{
    fn send_commands(
        &mut self,
        cmds: display_interface::DataFormat<'_>,
    ) -> Result<(), DisplayError> {
        self.send_data(cmds)
    }
    fn send_data(&mut self, buf: display_interface::DataFormat<'_>) -> Result<(), DisplayError> {
        match buf {
            display_interface::DataFormat::U8(data) => {
                for d in data {
                    self.write_u8(*d)?;
                }
            }
            display_interface::DataFormat::U8Iter(data) => {
                for d in data {
                    self.write_u8(d)?;
                }
            }
            _ => return Err(DisplayError::DataFormatNotImplemented),
        }
        Ok(())
    }
}

pub struct Gu256x128c<I> {
    interface: I,
}
impl<I: display_interface::WriteOnlyDataCommand> Gu256x128c<I> {
    pub fn new(interface: I) -> Self {
        Gu256x128c { interface }
    }
    pub fn init(&mut self) -> Result<(), DisplayError> {
        self.interface
            .send_commands(display_interface::DataFormat::U8(&[0x1b]))?;
        self.interface
            .send_commands(display_interface::DataFormat::U8(&[0x40]))?;
        Ok(())
    }
    pub fn write_str(&mut self, s: &str) -> Result<(), DisplayError> {
        self.interface
            .send_data(display_interface::DataFormat::U8Iter(&mut s.bytes()))
    }
}
