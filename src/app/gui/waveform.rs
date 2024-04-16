use embedded_graphics::{
    draw_target::{DrawTarget, DrawTargetExt},
    framebuffer::Framebuffer,
    geometry::{Point, Size},
    pixelcolor::raw::LittleEndian,
    primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    transform::Transform,
    Drawable, Pixel,
};
use itertools::Itertools;

use crate::app::{
    waveform_color, AppError, ProbeChannel, Result, State, StateMarker, StateVec, TimebaseMode,
    WaveformStorage,
};

use super::{
    gui_color, waveform_color_ex, Draw, GUIInfo, GuiColor, StateResult, WaveformColor,
    WaveformColorRaw, SCREEN_HEIGHT, SCREEN_WIDTH,
};

#[cfg(feature = "waveform-3bit")]
use super::WaveformColorEx;

pub struct Waveform {
    pub(crate) info: GUIInfo,
}
pub const WF_SCALING: u32 = 1;
pub const WF_WIDTH_WIDTH: u32 = (SCREEN_WIDTH - 48 - 8) / WF_SCALING / 8 * 8;
pub const WF_WIDTH_HEIGHT: u32 = (SCREEN_HEIGHT - 12 - 12) / WF_SCALING / 8 * 8;
impl Default for Waveform {
    fn default() -> Self {
        Self {
            info: GUIInfo {
                size: Size::new(WF_WIDTH_WIDTH, WF_WIDTH_HEIGHT),
                position: Point::new(7, 12),
                color_primary: gui_color(1),
                color_secondary: gui_color(7),
                ..Default::default()
            },
        }
    }
}

#[cfg(not(feature = "waveform-nobuffer"))]
static mut WF_FRAME_BUFFER: Framebuffer<
    WaveformColor,
    WaveformColorRaw,
    LittleEndian,
    { WF_WIDTH_WIDTH as usize },
    { WF_WIDTH_HEIGHT as usize },
    {
        embedded_graphics::framebuffer::buffer_size::<WaveformColor>(
            WF_WIDTH_WIDTH as usize,
            WF_WIDTH_HEIGHT as usize,
        )
    },
> = Framebuffer::<
    WaveformColor,
    WaveformColorRaw,
    LittleEndian,
    { WF_WIDTH_WIDTH as usize },
    { WF_WIDTH_HEIGHT as usize },
    {
        embedded_graphics::framebuffer::buffer_size::<WaveformColor>(
            WF_WIDTH_WIDTH as usize,
            WF_WIDTH_HEIGHT as usize,
        )
    },
>::new();
#[cfg(feature = "waveform-3bit")]
static mut WF_FRAME_BUFFER_EX: Framebuffer<
    WaveformColorEx,
    WaveformColorExRaw,
    LittleEndian,
    { WF_WIDTH_WIDTH as usize },
    { WF_WIDTH_HEIGHT as usize },
    {
        embedded_graphics::framebuffer::buffer_size::<WaveformColorEx>(
            WF_WIDTH_WIDTH as usize,
            WF_WIDTH_HEIGHT as usize,
        )
    },
> = Framebuffer::<
    WaveformColorEx,
    WaveformColorExRaw,
    LittleEndian,
    { WF_WIDTH_WIDTH as usize },
    { WF_WIDTH_HEIGHT as usize },
    {
        embedded_graphics::framebuffer::buffer_size::<WaveformColorEx>(
            WF_WIDTH_WIDTH as usize,
            WF_WIDTH_HEIGHT as usize,
        )
    },
>::new();

impl<D> Draw<D> for Waveform
where
    D: DrawTarget<Color = GuiColor>,
{
    fn state_emit_mask(&self) -> &[StateMarker] {
        &[StateMarker::Waveform, StateMarker::WaveformData]
    }
    fn draw_state_vec(&self, display: &mut D, state: &mut State, vec: &StateVec) -> StateResult {
        #[cfg(not(feature = "waveform-nobuffer"))]
        let fb = unsafe { &mut WF_FRAME_BUFFER };
        #[cfg(not(feature = "waveform-nobuffer"))]
        let display_target = display;
        #[cfg(not(feature = "waveform-nobuffer"))]
        let display = fb;
        #[cfg(feature = "waveform-3bit")]
        let fb_ex = unsafe { &mut WF_FRAME_BUFFER_EX };
        #[cfg(feature = "waveform-3bit")]
        let display_ex = fb_ex;
        #[cfg(not(any(feature = "waveform-3bit", feature = "waveform-nobuffer")))]
        let display_ex = display;
        #[cfg(feature = "waveform-nobuffer")]
        let mut display = display.translated(self.info.position);
        #[cfg(feature = "waveform-nobuffer")]
        let mut display_ex = display.color_converted();
        #[cfg(feature = "waveform-nobuffer")]
        let display_ex = &mut display_ex;
        let update_full = !vec.at(StateMarker::Waveform) || crate::app::WAVEFORM_HISTORY_LEN <= 3;
        let update_data = update_full || !vec.at(StateMarker::WaveformData);
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(waveform_color_ex(1))
            .stroke_width(1)
            .fill_color(waveform_color_ex(0))
            .build();
        // let trans = self.info.position;
        let trans = Point::zero();
        if update_full || state.timebase_mode != TimebaseMode::Normal {
            Rectangle::new(trans, self.info.size)
                .draw_styled(&style, display_ex)
                .map_err(|_| AppError::DisplayError)?;
        }
        let center = self.info.size_center();
        Line::new(
            Point::new(center.x, 0),
            Point::new(center.x, self.info.height() - 1),
        )
        .translate(trans)
        .draw_styled(&style, display_ex)
        .map_err(|_| AppError::DisplayError)?;
        Line::new(
            Point::new(0, center.y),
            Point::new(self.info.width() - 1, center.y),
        )
        .translate(trans)
        .draw_styled(&style, display_ex)
        .map_err(|_| AppError::DisplayError)?;

        let dl = 1;
        // Draw point grid
        // for i in 0..(self.info.size.height as i32 / 40 + 1) {
        //     for j in 0..(self.info.size.width as i32 / 8) {
        //         let x = j * 8 + 4;
        //         let y = i * 40 + 28;
        //         if x == center.x as i32 {
        //             continue;
        //         }
        //         if y == center.y as i32 {
        //             Line::new(Point::new(x, y - dl), Point::new(x, y + dl))
        //                 .translate(trans)
        //                 .draw_styled(&style, display_ex)
        //                 .map_err(|_| AppError::DisplayError)?;
        //         } else {
        //             let p = Point::new(x, y) + trans;
        //             Pixel(p, waveform_color_ex(1))
        //                 .draw(display_ex)
        //                 .map_err(|_| AppError::DisplayError)?;
        //         }
        //     }
        // }
        // for i in 0..(self.info.size.width as i32 / 40 + 1) {
        //     for j in 0..(self.info.size.height as i32 / 8 + 1) {
        //         let x = i * 40 + 12;
        //         let y = j * 8 + 4;
        //         if y == center.y as i32 {
        //             continue;
        //         }
        //         if x == center.x as i32 {
        //             Line::new(Point::new(x - dl, y), Point::new(x + dl, y))
        //                 .translate(trans)
        //                 .draw_styled(&style, display_ex)
        //                 .map_err(|_| AppError::DisplayError)?;
        //         } else {
        //             let p = Point::new(x, y) + trans;
        //             Pixel(p, waveform_color_ex(1))
        //                 .draw(display_ex)
        //                 .map_err(|_| AppError::DisplayError)?;
        //         }
        //     }
        // }
        let dd = self.info.size.height as i32 / 8;
        for k in 0..2 {
            let dir_x = k == 0;
            let mut x = center.x as i32;
            while x > 0 && x < self.info.width() as i32 {
                for l in 0..2 {
                    let dir_y = l == 0;
                    let mut y = center.y as i32;
                    while y > 0 && y < self.info.height() as i32 {
                        if y == center.y as i32 && dir_y {
                            let (p1, p2) = (Point::new(x, y - dl), Point::new(x, y + dl));
                            Line::new(p1, p2)
                                .translate(trans)
                                .draw_styled(&style, display_ex)
                                .map_err(|_| AppError::DisplayError)?;
                        } else {
                            Pixel(Point::new(x, y), waveform_color_ex(1))
                                .draw(display_ex)
                                .map_err(|_| AppError::DisplayError)?;
                        }
                        if dir_y {
                            y += dd;
                        } else {
                            y -= dd;
                        }
                    }
                }
                if dir_x {
                    x += dd;
                } else {
                    x -= dd;
                }
            }
            let x = center.x as i32;
            let mut y = center.y as i32;
            while y > 0 && y < self.info.height() as i32 {
                let (p1, p2) = (Point::new(x - dl, y), Point::new(x + dl, y));
                Line::new(p1, p2)
                    .translate(trans)
                    .draw_styled(&style, display_ex)
                    .map_err(|_| AppError::DisplayError)?;
                if dir_x {
                    y += dd;
                } else {
                    y -= dd;
                }
            }
        }
        #[cfg(not(feature = "waveform-3bit"))]
        let display = display_ex;

        // draw trigger level
        let trigger_color = state.trigger_channel.color_waveform();
        // FIXME: normalize
        let trigger_level_pixel =
            center.y - ((state.trigger_level_mv as i32) * (WF_WIDTH_HEIGHT as i32) / 2 / 3000);
        Line::new(
            Point::new(0, trigger_level_pixel),
            Point::new(self.info.width() as i32, trigger_level_pixel),
        )
        .draw_styled(&PrimitiveStyle::with_stroke(trigger_color, 1), display)
        .map_err(|_| AppError::DisplayError)?;

        if update_data {
            match state.timebase_mode {
                TimebaseMode::Normal | TimebaseMode::Rolling => {
                    for channel in 0..(ProbeChannel::Endding as usize) {
                        let update_only = state.waveform[channel].linked.len() > 3;
                        let color = ProbeChannel::from(channel).color_waveform();
                        self.draw_channel(
                            display,
                            &mut state.waveform[channel],
                            color,
                            update_only,
                            state.timebase_mode == TimebaseMode::Rolling,
                        )?;
                    }
                }
                TimebaseMode::XY => {}
            }
        }
        // #[cfg(feature = "waveform-nobuffer")]
        // let display_target = display_ex;
        #[cfg(not(feature = "waveform-nobuffer"))]
        let mut display_translated = display_target.translated(self.info.position);
        #[cfg(not(feature = "waveform-nobuffer"))]
        let mut display_converted = display_translated.color_converted();
        #[cfg(not(any(
            feature = "waveform-16bit",
            feature = "waveform-1bit",
            feature = "waveform-nobuffer"
        )))]
        let data = display.data();
        #[cfg(feature = "waveform-3bit")]
        let data_ex = display_ex.data();
        #[cfg(feature = "waveform-3bit")]
        let contiguous = data
            .chunks(WF_WIDTH_WIDTH as usize / (8 / WaveformColorRaw::BITS_PER_PIXEL))
            .zip(data_ex.chunks(WF_WIDTH_WIDTH as usize / (8 / WaveformColorExRaw::BITS_PER_PIXEL)))
            // copy lines
            .flat_map(|(row, row_ex)| core::iter::repeat((row, row_ex)).take(WF_SCALING as usize))
            .flat_map(|(row, row_ex)| {
                row.iter()
                    .zip(row_ex.iter().flat_map(|x| {
                        // match bits in u8
                        core::iter::repeat(x).take(
                            WaveformColorRaw::BITS_PER_PIXEL / WaveformColorExRaw::BITS_PER_PIXEL,
                        )
                    }))
                    .enumerate()
                    .flat_map(|(i, (&d, &d_ex))| {
                        (0..4)
                            .map(move |j| {
                                let d = (d >> ((3 - j) * 2)) & 0b11;
                                let d_ex = (d_ex >> ((3 - j) + ((!(i & 0b1)) << 2))) & 0b1;
                                WaveformCombinedColor::new(d, d_ex)
                            })
                            .flat_map(|color| {
                                // copy pixels
                                core::iter::repeat(color).take(WF_SCALING as usize)
                            })
                    })
            });
        #[cfg(not(any(
            feature = "waveform-3bit",
            feature = "waveform-16bit",
            feature = "waveform-1bit",
            feature = "waveform-nobuffer"
        )))]
        let contiguous = {
            use embedded_graphics_core::prelude::RawData;
            data.chunks(WF_WIDTH_WIDTH as usize / (8 / WaveformColorRaw::BITS_PER_PIXEL))
                // copy lines
                .flat_map(|row| core::iter::repeat(row).take(WF_SCALING as usize))
                .flat_map(|row| {
                    row.iter().flat_map(|&d| {
                        (0..4)
                            .map(move |j| {
                                let d = (d >> ((3 - j) * 2)) & 0b11;
                                waveform_color(d)
                            })
                            .flat_map(|color| {
                                // copy pixels
                                core::iter::repeat(color).take(WF_SCALING as usize)
                            })
                    })
                })
        };
        #[cfg(any(feature = "waveform-16bit", feature = "waveform-1bit"))]
        {
            let im = display.as_image();
            let image = embedded_graphics::image::Image::new(&im, Point::zero());
            image
                .draw(&mut display_converted)
                .map_err(|_| AppError::DisplayError)?;
        }
        #[cfg(not(any(
            feature = "waveform-16bit",
            feature = "waveform-1bit",
            feature = "waveform-nobuffer"
        )))]
        display_converted
            .fill_contiguous(
                &Rectangle::new(
                    Point::zero(),
                    Size::new(
                        self.info.size.width * WF_SCALING,
                        self.info.size.height * WF_SCALING,
                    ),
                ),
                contiguous,
            )
            .map_err(|_| AppError::DisplayError)?;

        if update_data && !update_full {
            Ok(Some(&[StateMarker::WaveformData]))
        } else {
            Ok(Some(&[StateMarker::Waveform, StateMarker::WaveformData]))
        }
    }
}

impl Waveform {
    fn draw_list_values_color<D>(
        &self,
        display: &mut D,
        data: &[f32],
        offset_idx: i32,
        color: WaveformColor,
        interpolate: bool,
        rolling_mode: bool,
    ) -> Result<()>
    where
        D: DrawTarget<Color = WaveformColor>,
    {
        let screen_offset = Point::new(2, self.info.height() / 2);
        let style = PrimitiveStyle::with_stroke(color, 1);
        let mut pt_last = Point::new(0, 0);
        let fill = (-offset_idx.min(0)) as usize;
        let skip = offset_idx.max(0) as usize;
        let mut paint_point = |(i, v, width_caling)| -> Result<()> {
            // defmt::info!("drawing: {} {}", i, v);
            let idx = i + fill;
            let clamp = |x: i32| {
                x.min(self.info.height() / 2 as i32 - 1)
                    .max(-self.info.height() / 2 + 1)
            };
            let pt = Point::new(
                (idx * (self.info.width() as usize) * width_caling / data.len()) as i32,
                clamp((v * -1.0 * (self.info.height() as f32) / 6.0) as i32),
            );
            if i != 0 {
                Line::new(pt_last, pt)
                    .translate(screen_offset)
                    .draw_styled(&style, display)
                    .map_err(|_| AppError::DisplayError)?;
            }
            pt_last = pt;
            Ok(())
        };
        if interpolate && !rolling_mode {
            let data_it = data.iter().skip(skip);
            let data_it_len = data_it.len();
            // use Lagrange interpolation
            let l_funcs = data_it.clone().enumerate().map(|(i, d)| {
                // input: f(0), f(2), f(4), f(6)... target: f(1), f(3), f(5), f(7)...
                let idx = i * 2;
                let indexes = (0..data_it_len).filter(move |x| *x != i);
                let l_base = indexes
                    .clone()
                    .map(|x| (idx as f32 - x as f32 * 2.0))
                    .tree_fold1(|a, b| a * b)
                    .unwrap();
                move |x| {
                    indexes
                        .map(|j| (x - (j * 2) as f32) as f32)
                        .tree_fold1(|a, b| a * b)
                        .unwrap()
                        * d
                        / l_base
                }
            });
            let lagrange = |x: f32| {
                l_funcs
                    .clone()
                    .zip(data_it.clone())
                    .map(|(l, y)| l(x) * *y)
                    .sum::<f32>()
            };
            let interpolated = (0..data_it_len)
                .map(|x| x * 2 + 1)
                .map(|x| lagrange(x as f32));
            let it = data_it
                .clone()
                .zip(interpolated)
                .flat_map(|(x1, x2)| [*x1, x2]);
            for (i, v) in it.enumerate() {
                paint_point((i, v, 2))?;
            }
        } else {
            if !rolling_mode {
                // defmt::info!("len/2: {}, skip: {}", data.len() / 2, skip);
                let it = data.iter().skip(skip);
                for (i, v) in it.enumerate() {
                    paint_point((i, *v, 1))?;
                }
            } else {
                let it = data.iter().skip(skip).chain(data.iter().take(skip));
                // let it = data.iter();
                for (i, v) in it.enumerate() {
                    paint_point((i, *v, 1))?;
                }
            }
        };
        Ok(())
    }

    fn draw_channel<D>(
        &self,
        display: &mut D,
        storage: &mut WaveformStorage,
        color: WaveformColor,
        update_only: bool,
        rolling_mode: bool,
    ) -> Result<()>
    where
        D: DrawTarget<Color = WaveformColor>,
    {
        // let color_secondary = GuiColor::new(color.r() / 2, color.g() / 2, color.b() / 2);
        // let color_secondary = gui_color(GuiColorRaw::from(color).into_inner() + 9);
        use waveform_color as gui_color;
        // let color_secondary = color;
        let color_secondary = gui_color(1);
        let interpolate = false;
        // let interpolate = true;
        if rolling_mode {
            let data = &storage.data[0][..storage.len];
            let color = color;
            self.draw_list_values_color(
                display,
                data,
                storage.rolling_offset as i32,
                color,
                interpolate,
                true,
            )?;
        } else {
            if !update_only {
                for (idx, it) in storage.linked.iter().enumerate() {
                    if !it.0 {
                        continue;
                    }
                    let data = &storage.data[it.1][..storage.len];
                    let color = if crate::app::WAVEFORM_HISTORY_LEN <= 3 {
                        color
                    } else {
                        if idx == 0 {
                            color
                        } else {
                            color_secondary
                        }
                    };
                    self.draw_list_values_color(
                        display,
                        data,
                        storage.offset[it.1],
                        color,
                        interpolate,
                        false,
                    )?;
                }
            } else {
                // clear tail and draw head
                let tail = storage.linked.pop_back().ok_or(AppError::Unexpected)?;
                if tail.0 {
                    let data = &storage.data[tail.1][..storage.len];
                    self.draw_list_values_color(
                        display,
                        data,
                        storage.offset[tail.1],
                        gui_color(0),
                        interpolate,
                        false,
                    )?;
                }
                storage
                    .linked
                    .push_back(tail)
                    .map_err(|_| AppError::Unexpected)?;
                let head = storage.linked.pop_front().ok_or(AppError::Unexpected)?;
                let head2 = storage.linked.pop_front().ok_or(AppError::Unexpected)?;
                if head2.0 {
                    let data = &storage.data[head2.1][..storage.len];
                    self.draw_list_values_color(
                        display,
                        data,
                        storage.offset[head2.1],
                        color_secondary,
                        interpolate,
                        false,
                    )?;
                }
                if head.0 {
                    let data = &storage.data[head.1][..storage.len];
                    self.draw_list_values_color(
                        display,
                        data,
                        storage.offset[head.1],
                        color,
                        interpolate,
                        false,
                    )?;
                }
                storage
                    .linked
                    .push_front(head2)
                    .map_err(|_| AppError::Unexpected)?;
                storage
                    .linked
                    .push_front(head)
                    .map_err(|_| AppError::Unexpected)?;
            }
        }
        Ok(())
    }
}
