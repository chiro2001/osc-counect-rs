#![no_std]

use embedded_graphics_core::{draw_target::DrawTarget, geometry::Dimensions};

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
        while self.ready.is_low().map_err(|_| DisplayError::DCError)? {
            self.delay.delay_ns(100);
        }
        self.cs.set_low().map_err(|_| DisplayError::CSError)?;
        self.spi
            .write(&[data])
            .map_err(|_| DisplayError::BusWriteError)?;
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
    buffer: [u8; 256 * 128 / 8],
}
impl<I: display_interface::WriteOnlyDataCommand> Gu256x128c<I> {
    pub fn new(interface: I) -> Self {
        Gu256x128c {
            interface,
            buffer: [0; 256 * 128 / 8],
        }
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
    pub fn set_pixel(&mut self, x: u16, y: u16, color: bool) -> Result<(), DisplayError> {
        if x >= 256 || y >= 128 {
            return Ok(());
        }
        let idx = (x as usize) * self.bounding_box().size.height as usize / 8 + y as usize / 8;
        let bit = 7 - (y % 8) as u8;
        if idx >= self.buffer.len() {
            return Err(DisplayError::OutOfBoundsError);
        }
        let d = &mut self.buffer[idx];
        *d = *d & !(1 << bit) | ((color as u8) << bit);
        Ok(())
    }
    pub fn flush(&mut self) -> Result<(), DisplayError> {
        // Bit image write: 02h,44h,DAD,46h, aL,aH,sL,sH, d(1)...d(s)
        let s: u16 = 128 * 256 / 8;
        let sel = [
            0x02,
            0x44,
            DAD,
            0x46,
            0,
            0,
            (s & 0xff) as u8,
            ((s >> 8) & 0xff) as u8,
        ];
        self.interface
            .send_data(display_interface::DataFormat::U8Iter(
                &mut sel.iter().cloned().chain(self.buffer.iter().cloned()),
            ))
    }
}

impl<I> Dimensions for Gu256x128c<I> {
    fn bounding_box(&self) -> embedded_graphics_core::primitives::Rectangle {
        embedded_graphics_core::primitives::Rectangle::new(
            embedded_graphics_core::geometry::Point::zero(),
            embedded_graphics_core::geometry::Size::new(256, 128),
        )
    }
}

// ignored
const DAD: u8 = 0x00;

impl<T: display_interface::WriteOnlyDataCommand> DrawTarget for Gu256x128c<T> {
    type Color = embedded_graphics_core::pixelcolor::BinaryColor;

    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics_core::prelude::Pixel<Self::Color>>,
    {
        for pixel in pixels.into_iter() {
            let x = pixel.0.x as u16;
            let y = pixel.0.y as u16;
            let d = pixel.1.is_on();
            self.set_pixel(x, y, d)?;
        }
        self.flush()?;
        Ok(())
    }
    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics_core::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        for (i, color) in colors
            .into_iter()
            .take((area.size.width * area.size.height) as usize)
            .enumerate()
        {
            let x = (i % area.size.width as usize) as u16 + area.top_left.x as u16;
            let y = (i / area.size.width as usize) as u16 + area.top_left.y as u16;
            self.set_pixel(x, y, color.is_on())?;
        }
        self.flush()?;
        Ok(())
    }
}
