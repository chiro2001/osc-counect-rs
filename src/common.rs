use embedded_graphics::draw_target::{DrawTarget, Translated};

use crate::app::devices::DisplayFlushable;

#[cfg(feature = "display-gu256x128c")]
impl<I: display_interface::WriteOnlyDataCommand> DisplayFlushable for gu256x128c::Gu256x128c<I> {
    fn do_display_flush(&mut self) -> crate::app::Result<()> {
        self.flush().map_err(|_| crate::app::AppError::DisplayError)
    }
}
#[cfg(feature = "display-ili9327")]
impl<I, R> DisplayFlushable for ili9341::Ili9341<I, R> {}
#[cfg(any(feature = "display-st7789-1", feature = "display-st7789-2"))]
impl<DI, RST, BL, C> DisplayFlushable for st7789::ST7789<DI, RST, BL, C>
where
    DI: display_interface::WriteOnlyDataCommand,
    RST: embedded_hal::digital::OutputPin,
    BL: embedded_hal::digital::OutputPin,
{
}
impl<'a, I> DisplayFlushable for Translated<'a, I> where I: DisplayFlushable + DrawTarget {}
