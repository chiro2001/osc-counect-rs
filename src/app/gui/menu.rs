use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    primitives::*,
    text::{Alignment, Text},
    Drawable,
};

use crate::app::{AppError, StateMarker};

use super::{gui_color, Draw, GuiColor, TEXT_OFFSET};

pub type MenuItems<const N: usize, I> = [(
    Option<I>,
    &'static str,
    &'static [(Option<I>, &'static str)],
); N];
pub type MenuItemsRef<I> = &'static [(
    Option<I>,
    &'static str,
    &'static [(Option<I>, &'static str)],
)];

pub struct Menu<I: 'static> {
    pub items: MenuItemsRef<I>,
}
impl<I> Menu<I> {
    pub fn new(items: MenuItemsRef<I>) -> Self {
        Self { items }
    }
}

impl<D, I> Draw<D> for Menu<I> {
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::SettingsMenu]
    }

    fn draw_state(&self, display: &mut D, state: &mut crate::app::State) -> super::StateResult
    where
        D: DrawTarget<Color = GuiColor>,
    {
        let offset_l1 = Point::new(48, 14);
        self.draw_level(
            display,
            state,
            offset_l1,
            self.items.iter().map(|x| x.1),
            self.items.len(),
            state.menu_idx_l1,
        )?;
        if let Some(idx) = state.menu_idx_l2 {
            let offset_l2 = Point::new(48 + 74, 14 + 12 * state.menu_idx_l1 as i32);
            self.draw_level(
                display,
                state,
                offset_l2,
                self.items[state.menu_idx_l1].2.iter().map(|x| x.1),
                self.items[state.menu_idx_l1].2.len(),
                idx,
            )?;
        }
        Ok(Some(&[StateMarker::SettingsMenu]))
    }
}

impl<I> Menu<I> {
    fn draw_level<D, L>(
        &self,
        display: &mut D,
        _state: &mut crate::app::State,
        offset: Point,
        list: L,
        length: usize,
        focursed: usize,
    ) -> super::Result<()>
    where
        D: DrawTarget<Color = GuiColor>,
        L: IntoIterator<Item = &'static str>,
    {
        let item_height = 12u32;
        let item_width = 72u32;
        Rectangle::new(offset, Size::new(item_width, item_height * length as u32))
            .draw_styled(&PrimitiveStyle::with_fill(gui_color(5)), display)
            .map_err(|_| AppError::DisplayError)?;
        Rectangle::new(
            offset + Point::new(1, item_height as i32 * focursed as i32 + 1),
            Size::new(item_width - 2, item_height - 2),
        )
        .draw_styled(&PrimitiveStyle::with_fill(gui_color(0)), display)
        .map_err(|_| AppError::DisplayError)?;
        for (i, s) in list.into_iter().enumerate() {
            let text_color = if i == focursed {
                gui_color(15)
            } else {
                gui_color(0)
            };
            Text::with_alignment(
                s,
                offset
                    + Point::new(2, i as i32 * item_height as i32 + item_height as i32 / 2)
                    + TEXT_OFFSET,
                MonoTextStyle::new(&FONT_6X9, text_color),
                Alignment::Left,
            )
            .draw(display)
            .map_err(|_| AppError::DisplayError)?;
        }
        Ok(())
    }
}
