use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    primitives::*,
    text::{Alignment, Text},
    Drawable,
};

use crate::app::{AppError, StateMarker, StateVec};

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
        &[
            StateMarker::SettingsMenu,
            StateMarker::SettingsMenuMoveL1,
            StateMarker::SettingsMenuMoveL2,
        ]
    }

    fn draw_state_vec(
        &self,
        display: &mut D,
        state: &mut crate::app::State,
        vec: &StateVec,
    ) -> super::StateResult
    where
        D: DrawTarget<Color = GuiColor>,
    {
        let offset_l1 = Point::new(48, 14);
        if !vec.at(StateMarker::SettingsMenuMoveL1) || !vec.at(StateMarker::SettingsMenu) {
            self.draw_level(
                display,
                state,
                offset_l1,
                self.items.iter().map(|x| (x.1, x.2.len() > 0)),
                self.items.len(),
                state.menu_idx_l1,
                state.menu_idx_l1_last,
                vec.at(StateMarker::SettingsMenu),
            )?;
        }
        if let Some(idx) = state.menu_idx_l2 {
            if !vec.at(StateMarker::SettingsMenuMoveL2) || !vec.at(StateMarker::SettingsMenu) {
                let offset_l2 = Point::new(48 + 74, 14 + 12 * state.menu_idx_l1 as i32);
                self.draw_level(
                    display,
                    state,
                    offset_l2,
                    self.items[state.menu_idx_l1].2.iter().map(|x| (x.1, false)),
                    self.items[state.menu_idx_l1].2.len(),
                    idx,
                    state.menu_idx_l2_last,
                    vec.at(StateMarker::SettingsMenu),
                )?;
            }
        }
        // }
        Ok(Some(&[
            StateMarker::SettingsMenu,
            StateMarker::SettingsMenuMoveL1,
            StateMarker::SettingsMenuMoveL2,
        ]))
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
        focursed_last: Option<usize>,
        partial_update: bool,
    ) -> super::Result<()>
    where
        D: DrawTarget<Color = GuiColor>,
        L: IntoIterator<Item = (&'static str, bool)>,
    {
        let item_height = 12u32;
        let item_width = 72u32;
        if !partial_update {
            Rectangle::new(offset, Size::new(item_width, item_height * length as u32))
                .draw_styled(&PrimitiveStyle::with_fill(gui_color(5)), display)
                .map_err(|_| AppError::DisplayError)?;
        }
        let mut fill_item = |i, color, text, fill_rect, mark| {
            if fill_rect {
                Rectangle::new(
                    offset + Point::new(1, item_height as i32 * i as i32 + 1),
                    Size::new(item_width - 2, item_height - 2),
                )
                .draw_styled(&PrimitiveStyle::with_fill(color), display)
                .map_err(|_| AppError::DisplayError)
            } else {
                let pos = offset
                    + Point::new(2, i as i32 * item_height as i32 + item_height as i32 / 2)
                    + TEXT_OFFSET;
                let r = Text::with_alignment(
                    text,
                    pos,
                    MonoTextStyle::new(&FONT_6X9, color),
                    Alignment::Left,
                )
                .draw(display)
                .map_err(|_| AppError::DisplayError)
                .map(|_| ());
                match r {
                    Ok(_) => {
                        if mark {
                            Text::with_alignment(
                                ">",
                                pos + Point::new(item_width as i32 - 9, 0),
                                MonoTextStyle::new(&FONT_6X9, color),
                                Alignment::Left,
                            )
                            .draw(display)
                            .map_err(|_| AppError::DisplayError)
                            .map(|_| ())
                        } else {
                            Ok(())
                        }
                    }
                    Err(e) => Err(e),
                }
            }
        };
        if let Some(idx) = focursed_last {
            fill_item(idx, gui_color(5), "", true, false)?;
        }
        fill_item(focursed, gui_color(0), "", true, false)?;
        if !partial_update {
            for (i, (s, mark)) in list.into_iter().enumerate() {
                let text_color = if i == focursed {
                    gui_color(15)
                } else {
                    gui_color(0)
                };
                fill_item(i, text_color, s, false, mark)?;
            }
        } else {
            for (i, (s, mark)) in list
                .into_iter()
                .enumerate()
                .filter(|(i, _)| *i == focursed || *i == focursed_last.unwrap_or(0))
                .take(2)
            {
                let text_color = if i == focursed {
                    gui_color(15)
                } else {
                    gui_color(0)
                };
                fill_item(i, text_color, s, false, mark)?;
            }
        }
        Ok(())
    }
}
