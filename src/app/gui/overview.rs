use embedded_graphics::Drawable;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    primitives::{Primitive, PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, Text},
    transform::Transform,
};

use crate::app::{AppError, State, StateMarker};

use super::{gui_color, Draw, GUIInfo, GuiColor, StateResult, TEXT_OFFSET};

pub struct Overview {
    pub(crate) info: GUIInfo,
    text: &'static str,
}
impl Overview {
    pub fn new() -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(133, 10),
                position: Point::new(70, 0),
                color_primary: gui_color(15),
                color_secondary: gui_color(0),
            },
            text: "Roll Mode",
        }
    }
}
impl<D> Draw<D> for Overview
where
    D: DrawTarget<Color = GuiColor>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Waveform]
    }
    fn draw_state(&self, display: &mut D, _state: &mut State) -> StateResult {
        Rectangle::new(self.info.position, self.info.size)
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .stroke_color(self.info.color_primary)
                    .stroke_width(1)
                    .fill_color(self.info.color_secondary)
                    .build(),
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        Text::with_alignment(
            self.text,
            self.info.size_center() + TEXT_OFFSET,
            MonoTextStyle::new(&FONT_6X9, self.info.color_primary),
            Alignment::Center,
        )
        .translate(self.info.position)
        .draw(display)
        .map_err(|_| AppError::DisplayError)?;
        Ok(None)
    }
}
