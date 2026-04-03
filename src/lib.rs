#![no_std]

mod spline;

use embassy_futures::yield_now;
use embassy_time::{Duration, Instant};
use embedded_graphics::primitives::{
    Circle, Line, PrimitiveStyle, Rectangle, Styled, StyledPixels, Triangle,
};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal_async::i2c::I2c;
use panic_probe as _;

use crate::spline::{SplineRasterIter, compute_steps};

pub const DISPLAY_SIZE_X: usize = 128;
pub const DISPLAY_SIZE_Y: usize = 64;
pub const PAGE_SIZE: usize = 128;
pub const NUM_PAGES: usize = DISPLAY_SIZE_X * DISPLAY_SIZE_Y / 8 / PAGE_SIZE;

static FONT_10X20_IMAGE: &[u8] = include_bytes!("../fonts/font_10x20.raw");
static FONT_8X13_IMAGE: &[u8] = include_bytes!("../fonts/font_8x13.raw");
static FONT_8X13_BOLD_IMAGE: &[u8] = include_bytes!("../fonts/font_8x13_bold.raw");
static FONT_6X10_IMAGE: &[u8] = include_bytes!("../fonts/font_6x10.raw");

static FONT_10X20: Font = Font {
    image: FONT_10X20_IMAGE,
    width: 10,
    height: 20,
};

static FONT_8X13: Font = Font {
    image: FONT_8X13_IMAGE,
    width: 8,
    height: 13,
};

static FONT_8X13_BOLD: Font = Font {
    image: FONT_8X13_BOLD_IMAGE,
    width: 8,
    height: 13,
};

static FONT_6X10: Font = Font {
    image: FONT_6X10_IMAGE,
    width: 6,
    height: 10,
};

/// Minimal SSD1306 driver with:
/// - internal framebuffer
/// - dirty page tracking
/// - cooperative async yielding
///
/// This driver is designed for deterministic behavior in async environments
/// (e.g. Embassy), ensuring long operations periodically yield execution.
///
/// The display is assumed to be **128x64** in page addressing mode with I2C interface.
///
/// The frame buffer can be updated directly using methods set_pixel(),
/// unset_pixel(), or update_pixel().  Location of a pixel should be specified
/// by a coordinate. The top-left corner is the origin. The position goes right
/// as x grows while it goes down as y grows.
///
/// There are also severl high-level drawing function that helps following use cases:
/// - printing text
/// - drawing embedded-graphics styled
/// - rendering image
/// - drawing arc
///
/// The image on the frame buffer is sent to the display by method flush()
/// or flush_full().
pub struct Ssd1306Lite<I2C> {
    i2c: I2C,
    addr: u8,
    framebuffer: [u8; 1024],
    dirty: [DirtyPage; 8],

    yield_interval: Duration,
}

/// Supported font sizes.
///
/// Note: `Medium` uses a bold variant automatically when drawing with `BinaryColor::Off`.
pub enum FontSize {
    Medium,
    Large,
    Small,
}

#[derive(Clone, Copy)]
struct DirtyPage {
    min_col: u8,
    max_col: u8,
    dirty: bool,
}

struct Font {
    pub image: &'static [u8],
    pub width: usize,
    pub height: usize,
}

impl<I2C> Ssd1306Lite<I2C>
where
    I2C: I2c,
{
    /// Creates a new driver instance with default configuration.
    ///
    /// - I2C address defaults to `0x3C`
    /// - Yield interval defaults to 25µs
    pub fn new(i2c: I2C) -> Self {
        let dirty = [DirtyPage {
            min_col: 127,
            max_col: 0,
            dirty: false,
        }; 8];
        Self {
            i2c,
            addr: 0x3c,
            framebuffer: [0u8; 1024],
            dirty,
            yield_interval: Duration::from_micros(25),
        }
    }

    /// Sets how often long-running operations yield to the executor.
    ///
    /// Smaller values improve responsiveness but reduce throughput.
    pub fn set_yield_interval(&mut self, interval: Duration) {
        self.yield_interval = interval;
    }

    /// Initializes the SSD1306 controller.
    ///
    /// Configures:
    /// - page addressing mode
    /// - display orientation
    /// - contrast and timing parameters
    pub async fn initialize(&mut self) {
        #[rustfmt::skip]
        let init_cmds = [
            0xAE,       // display off
            0x20, 0x02, // page addressing mode
            0xB0,
            0xC8,
            0x00,
            0x10,
            0x40,
            0x81, 0x7F,
            0xA1,
            0xA6,
            0xA8, 0x3F,
            0xA4,
            0xD3, 0x00,
            0xD5, 0x80,
            0xD9, 0xF1,
            0xDA, 0x12,
            0xDB, 0x40,
            0x8D, 0x14,
            0xAF, // display on
        ];

        Self::write_commands(&mut self.i2c, self.addr, &init_cmds).await;
    }

    /// Flushes the entire framebuffer to the display.
    ///
    /// This ignores dirty tracking and always updates all pages.
    pub async fn flush_full(&mut self) {
        for page in 0..NUM_PAGES {
            let base = page * PAGE_SIZE;

            Self::write_commands(&mut self.i2c, self.addr, &[0xB0 + page as u8, 0x00, 0x10]).await;

            Self::write_data(
                &mut self.i2c,
                self.addr,
                &self.framebuffer[base..base + PAGE_SIZE],
            )
            .await;
        }
    }

    /// Flushes only modified regions using dirty page tracking.
    ///
    /// Each page (8 vertical pixels) tracks min/max modified columns.
    /// Only those regions are transmitted over I2C.
    pub async fn flush(&mut self) {
        let mut temp = [0u8; PAGE_SIZE]; // one page max width

        for page in 0..NUM_PAGES {
            // ---- Step 1: copy dirty info out ----
            let (dirty, start, end) = {
                let d = &self.dirty[page];
                (d.dirty, d.min_col, d.max_col)
            };

            if !dirty {
                continue;
            }

            let start = start.min((PAGE_SIZE - 1) as u8);
            let end = end.min((PAGE_SIZE - 1) as u8);

            let len = (end - start + 1) as usize;
            let base = page * DISPLAY_SIZE_X;

            // ---- Step 2: copy framebuffer slice (break borrow) ----
            temp[..len]
                .copy_from_slice(&self.framebuffer[base + start as usize..=base + end as usize]);

            yield_now().await;

            // ---- Step 3: send commands ----
            Self::write_commands(
                &mut self.i2c,
                self.addr,
                &[
                    0xB0 + page as u8,     // set page
                    0x00 + (start & 0x0F), // lower column
                    0x10 + (start >> 4),   // upper column
                ],
            )
            .await;

            // ---- Step 4: send data ----
            Self::write_data(&mut self.i2c, self.addr, &temp[..len]).await;

            // ---- Step 5: clear dirty ----
            self.dirty[page] = DirtyPage {
                dirty: false,
                min_col: (PAGE_SIZE - 1) as u8,
                max_col: 0,
            };
        }
    }

    /// Clears the framebuffer with the specified color.
    ///
    /// Marks all pages as dirty.
    pub async fn clear(&mut self, color: BinaryColor) {
        let fill = match color {
            BinaryColor::On => 0xff,
            BinaryColor::Off => 0x00,
        };

        // Fill framebuffer
        self.framebuffer.fill(fill);

        self.dirty = [DirtyPage {
            min_col: 0,
            max_col: (PAGE_SIZE - 1) as u8,
            dirty: true,
        }; 8];
    }

    /// Draws an embedded-graphics styled primitive.
    ///
    /// This method yields periodically to maintain responsiveness.
    pub async fn draw_styled<P>(&mut self, styled: Styled<P, PrimitiveStyle<BinaryColor>>)
    where
        P: StyledPixels<PrimitiveStyle<BinaryColor>>,
        <P as StyledPixels<PrimitiveStyle<BinaryColor>>>::Iter: Iterator<Item = Pixel<BinaryColor>>,
    {
        let mut last_yield = Instant::now();

        for Pixel(point, color) in styled.pixels() {
            let (index, bit) = self.map_pixel_mut(point.x as u32, point.y as u32);
            match color {
                BinaryColor::On => {
                    self.framebuffer[index] |= 1 << bit;
                }
                BinaryColor::Off => {
                    self.framebuffer[index] &= !(1 << bit);
                }
            };
            if last_yield.elapsed() > self.yield_interval {
                yield_now().await;
                last_yield = Instant::now();
            }
        }
    }

    #[inline(always)]
    pub async fn draw_line(
        &mut self,
        start: (i32, i32),
        end: (i32, i32),
        style: PrimitiveStyle<BinaryColor>,
    ) {
        self.draw_styled(
            Line::new(Point::new(start.0, start.1), Point::new(end.0, end.1)).into_styled(style),
        )
        .await;
    }

    #[inline(always)]
    pub async fn draw_circle(
        &mut self,
        top_left: (i32, i32),
        diameter: u32,
        style: PrimitiveStyle<BinaryColor>,
    ) {
        self.draw_styled(
            Circle::new(Point::new(top_left.0, top_left.1), diameter).into_styled(style),
        )
        .await;
    }

    #[inline(always)]
    pub async fn draw_rectangle(
        &mut self,
        top_left: (i32, i32),
        width: u32,
        height: u32,
        style: PrimitiveStyle<BinaryColor>,
    ) {
        self.draw_styled(
            Rectangle::new(Point::new(top_left.0, top_left.1), Size::new(width, height))
                .into_styled(style),
        )
        .await;
    }

    #[inline(always)]
    pub async fn draw_triangle(
        &mut self,
        vertex1: (i32, i32),
        vertex2: (i32, i32),
        vertex3: (i32, i32),
        style: PrimitiveStyle<BinaryColor>,
    ) {
        self.draw_styled(
            Triangle::new(
                Point::new(vertex1.0, vertex1.1),
                Point::new(vertex2.0, vertex2.1),
                Point::new(vertex3.0, vertex3.1),
            )
            .into_styled(style),
        )
        .await;
    }

    /// Sets a pixel to ON
    #[inline]
    pub fn set_pixel(&mut self, x: i32, y: i32) {
        let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
        self.framebuffer[index] |= 1 << bit;
    }

    /// Sets a pixel to OFF.
    #[inline]
    pub fn unset_pixel(&mut self, x: i32, y: i32) {
        let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
        self.framebuffer[index] &= !(1 << bit);
    }

    /// Updates a pixel with a boolean value.
    #[inline]
    pub fn update_pixel(&mut self, x: i32, y: i32, color: bool) {
        let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
        if color {
            self.framebuffer[index] |= 1 << bit;
        } else {
            self.framebuffer[index] &= !(1 << bit);
        }
    }

    /// Draws a bitmap image.
    ///
    /// This method yields periodically.
    pub async fn draw_bitmap(
        &mut self,
        top_left_x: i32,
        top_left_y: i32,
        bitmap: &[u8],
        width: u32,
        height: u32,
        color: BinaryColor,
    ) {
        match color {
            BinaryColor::On => {
                self.draw_bitmap_core(
                    top_left_x,
                    top_left_y,
                    bitmap,
                    width,
                    height,
                    |data, bit| *data |= 1 << bit,
                )
                .await
            }
            BinaryColor::Off => {
                self.draw_bitmap_core(
                    top_left_x,
                    top_left_y,
                    bitmap,
                    width,
                    height,
                    |data, bit| *data &= !(1 << bit),
                )
                .await
            }
        };
    }

    async fn draw_bitmap_core<F>(
        &mut self,
        top_left_x: i32,
        top_left_y: i32,
        bitmap: &[u8],
        width: u32,
        height: u32,
        plot: F,
    ) where
        F: Fn(&mut u8, usize),
    {
        let mut last_yield = Instant::now();

        let byte_width = (width as usize + 7) / 8;
        for j in 0..height as usize {
            let mut byte = 0u8;
            let y = top_left_y + j as i32;
            for i in 0..width {
                let x = top_left_x + i as i32;
                if (i & 7) != 0 {
                    byte <<= 1;
                } else {
                    byte = bitmap[y as usize * byte_width + i as usize / 8];
                }

                let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
                if (byte & 0x80) != 0 {
                    plot(&mut self.framebuffer[index], bit);
                }
            }
            if last_yield.elapsed() > self.yield_interval {
                yield_now().await;
                last_yield = Instant::now();
            }
        }
    }

    async fn draw_char_at(
        &mut self,
        ch: char,
        x0: usize,
        y0: usize,
        font: &Font,
        color: BinaryColor,
        last_yield: &mut Instant,
    ) -> usize {
        let width = font.width;
        let height = font.height;
        let image = font.image;

        let byte_width = 16 * width / 8;

        // Map ASCII → atlas index
        let ch_u32 = ch as u32;
        let index = if ch_u32 < 0x20 || ch_u32 > 0x7e {
            0 // fallback to space
        } else {
            (ch_u32 - 0x20) as usize
        };

        let column = index % 16;
        let row = index / 16;

        for i in 0..width {
            let x = column * width + i;

            for j in 0..height {
                let y = row * height + j;

                let font_image_index = x / 8 + byte_width * y;
                let byte = image[font_image_index];
                let value = (byte >> (7 - x % 8)) & 0x1;

                let (data_index, data_bit) = self.map_pixel_mut((x0 + i) as u32, (y0 + j) as u32);

                match color {
                    BinaryColor::On => {
                        if value == 1 {
                            self.framebuffer[data_index] |= 1 << data_bit;
                        }
                    }
                    BinaryColor::Off => {
                        // Inverted rendering
                        if value == 1 {
                            self.framebuffer[data_index] &= !(1 << data_bit);
                        }
                    }
                }
                if last_yield.elapsed() > self.yield_interval {
                    yield_now().await;
                    *last_yield = Instant::now();
                }
            }
        }

        x0 + width
    }

    /// Draws a string at the given position.
    ///
    /// Rendering proceeds left-to-right.
    /// This method yields periodically.
    pub async fn draw_string(&mut self, text: &str, text_box: TextBox, font_size: FontSize) {
        let mut last_yield = Instant::now();

        let color = text_box.fg_color;

        let font = match font_size {
            FontSize::Large => &FONT_10X20,
            FontSize::Medium => match color {
                BinaryColor::On => &FONT_8X13,
                BinaryColor::Off => &FONT_8X13_BOLD,
            },
            FontSize::Small => &FONT_6X10,
        };

        let mut x = Self::position_text(text, &text_box, &font);
        let y = Self::v_position_text(&text_box, &font);

        for ch in text.chars() {
            x = self
                .draw_char_at(ch, x, y, &font, color, &mut last_yield)
                .await;
        }
    }

    fn position_text(string: &str, text_box: &TextBox, font: &Font) -> usize {
        match text_box.alignment {
            Alignment::Left => text_box.top_left_x,
            _ => {
                let text_width = font.width * string.len();
                let mut margin = if text_box.width > text_width {
                    text_box.width - text_width
                } else {
                    0
                };
                if matches!(text_box.alignment, Alignment::Center) {
                    margin /= 2;
                }
                text_box.top_left_x + margin
            }
        }
    }

    fn v_position_text(text_box: &TextBox, font: &Font) -> usize {
        match text_box.v_alignment {
            VAlignment::Top => text_box.top_left_y,
            _ => {
                let text_height = font.height;
                let mut margin = if text_box.height > text_height {
                    text_box.height - text_height
                } else {
                    0
                };
                if matches!(text_box.v_alignment, VAlignment::Center) {
                    margin /= 2;
                }
                text_box.top_left_y + margin
            }
        }
    }

    fn map_pixel_mut(&mut self, mut x: u32, mut y: u32) -> (usize, usize) {
        // limit the position within the display window
        x &= (DISPLAY_SIZE_X - 1) as u32;
        y &= (DISPLAY_SIZE_Y - 1) as u32;

        let page = (y / 8) as usize;

        let entry = &mut self.dirty[page];
        entry.dirty = true;
        entry.min_col = entry.min_col.min(x as u8);
        entry.max_col = entry.max_col.max(x as u8);

        let index = x as usize + DISPLAY_SIZE_X * page;
        let bit = y % 8;

        (index, bit as usize)
    }

    async fn write_commands(i2c: &mut I2C, addr: u8, cmds: &[u8]) {
        let mut buf = [0u8; 32];
        buf[0] = 0x00; // command

        let len = cmds.len();
        buf[1..1 + len].copy_from_slice(cmds);

        i2c.write(addr, &buf[..1 + len]).await.unwrap();
    }

    async fn write_data(i2c: &mut I2C, addr: u8, data: &[u8]) {
        let mut buf = [0u8; 129];
        buf[0] = 0x40;

        let mut offset = 0;
        while offset < data.len() {
            let chunk = (data.len() - offset).min(DISPLAY_SIZE_X);

            buf[1..1 + chunk].copy_from_slice(&data[offset..offset + chunk]);

            i2c.write(addr, &buf[..1 + chunk]).await.unwrap();

            offset += chunk;
        }
    }

    /// draws an arc
    pub async fn draw_arc(
        &mut self,
        cx: i32,
        cy: i32,
        radius: i32,
        start: Angle,
        end: Angle,
        color: bool,
    ) {
        let (sx, sy) = (start.x, start.y);
        let (ex, ey) = (end.x, end.y);

        let mut x = radius;
        let mut y = 0;
        let mut err = 1 - x;

        let mut last_yield = Instant::now();
        while x >= y {
            // 8 octants
            self.plot_arc_points(cx, cy, x, y, sx, sy, ex, ey, color, &mut last_yield)
                .await;
            self.plot_arc_points(cx, cy, y, x, sx, sy, ex, ey, color, &mut last_yield)
                .await;

            y += 1;

            if err < 0 {
                err += 2 * y + 1;
            } else {
                x -= 1;
                err += 2 * (y - x) + 1;
            }
        }
    }

    async fn plot_arc_points(
        &mut self,
        cx: i32,
        cy: i32,
        dx: i32,
        dy: i32,
        sx: i32,
        sy: i32,
        ex: i32,
        ey: i32,
        color: bool,
        last_yield: &mut Instant,
    ) {
        let candidates = [(dx, dy), (-dx, dy), (dx, -dy), (-dx, -dy)];

        for (px, py) in candidates {
            if in_arc(px, py, sx, sy, ex, ey) {
                self.update_pixel(cx + px, cy + py, color);
            }
            if last_yield.elapsed() > self.yield_interval {
                yield_now().await;
                *last_yield = Instant::now();
            }
        }
    }

    pub async fn draw_spline<'a>(
        &mut self,
        points: &'a [(i32, i32)],
        steps_per_segment: u32,
        color: BinaryColor,
    ) {
        match color {
            BinaryColor::On => {
                self.draw_spline_core(points, steps_per_segment, |data, bit| *data |= 1 << bit)
                    .await
            }
            BinaryColor::Off => {
                self.draw_spline_core(points, steps_per_segment, |data, bit| *data &= !(1 << bit))
                    .await
            }
        }
    }

    async fn draw_spline_core<'a, F>(
        &mut self,
        points: &'a [(i32, i32)],
        steps_per_segment: u32,
        plot: F,
    ) where
        F: Fn(&mut u8, usize),
    {
        let mut last_yield = Instant::now();
        let iter = SplineRasterIter::new(points, steps_per_segment);
        for (x, y) in iter {
            let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
            plot(&mut self.framebuffer[index], bit);
            if last_yield.elapsed() > self.yield_interval {
                yield_now().await;
                last_yield = Instant::now();
            }
        }
    }

    pub async fn draw_3p_curve(
        &mut self,
        p0: (i32, i32),
        p1: (i32, i32),
        p2: (i32, i32),
        color: BinaryColor,
    ) {
        let steps = compute_steps(p0, p1, p2);
        let points = [p0, p1, p2];
        self.draw_spline(&points, steps, color).await;
    }
}

/// Represents an angle as a direction vector `(cos, sin)`.
///
/// Values are scaled integers (≈1024 max).
/// This avoids floating-point usage.
#[non_exhaustive]
pub struct Angle {
    pub x: i32,
    pub y: i32,
}

const SIN_TABLE: [i16; 91] = [
    0, 18, 36, 55, 73, 91, 109, 127, 145, 163, 181, 198, 216, 233, 251, 268, 285, 302, 319, 336,
    353, 369, 386, 402, 418, 434, 450, 466, 481, 497, 512, 527, 542, 557, 572, 587, 601, 615, 629,
    643, 657, 670, 684, 697, 710, 723, 735, 748, 760, 772, 784, 796, 807, 819, 830, 841, 852, 862,
    873, 883, 893, 903, 912, 922, 931, 940, 949, 957, 966, 974, 981, 989, 996, 1003, 1010, 1017,
    1023, 1029, 1035, 1041, 1046, 1052, 1057, 1062, 1066, 1071, 1075, 1079, 1083, 1086, 1090,
];

impl Angle {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Creates an Angle instance from degrees.
    pub fn from_degrees(degrees: i32) -> Self {
        Self {
            x: Self::cos_degree(degrees),
            y: Self::sin_degree(degrees),
        }
    }

    fn sin_degree(degrees: i32) -> i32 {
        let mut d = degrees % 360;
        if d < 0 {
            d += 360
        };

        match d {
            0..=90 => SIN_TABLE[d as usize] as i32,
            91..=180 => SIN_TABLE[(180 - d) as usize] as i32,
            181..=270 => -(SIN_TABLE[(d - 180) as usize] as i32),
            _ => -(SIN_TABLE[(360 - d) as usize] as i32),
        }
    }

    fn cos_degree(degrees: i32) -> i32 {
        Self::sin_degree(degrees + 90)
    }
}

fn in_arc(dx: i32, dy: i32, sx: i32, sy: i32, ex: i32, ey: i32) -> bool {
    let c1 = cross(sx, sy, dx, dy);
    let c2 = cross(dx, dy, ex, ey);

    if cross(sx, sy, ex, ey) >= 0 {
        // normal case
        c1 >= 0 && c2 >= 0
    } else {
        // wrapped around 360°
        c1 >= 0 || c2 >= 0
    }
}

#[inline]
fn cross(ax: i32, ay: i32, bx: i32, by: i32) -> i32 {
    ax * by - ay * bx
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Error(&'static str);

impl Error {
    pub const fn new(msg: &'static str) -> Self {
        Self(msg)
    }

    pub const fn msg(&self) -> &'static str {
        self.0
    }
}

#[non_exhaustive]
#[derive(Clone)]
pub struct TextBox {
    pub top_left_x: usize,
    pub top_left_y: usize,
    pub width: usize,
    pub height: usize,
    pub fg_color: BinaryColor,
    pub alignment: Alignment,
    pub v_alignment: VAlignment,
}

#[derive(Clone)]
pub enum Alignment {
    Left,
    Center,
    Right,
}

#[derive(Clone)]
pub enum VAlignment {
    Top,
    Center,
    Bottom,
}

#[non_exhaustive]
pub struct TextBoxBuilder {
    text_box: TextBox,
}

impl TextBox {
    pub fn simple(top_left_x: usize, top_left_y: usize, fg_color: BinaryColor) -> Self {
        let top_left_x = top_left_x % DISPLAY_SIZE_X;
        let top_left_y = top_left_y % DISPLAY_SIZE_Y;

        Self {
            top_left_x,
            top_left_y,
            width: DISPLAY_SIZE_X - top_left_x,
            height: DISPLAY_SIZE_Y - top_left_y,
            fg_color,
            alignment: Alignment::Left,
            v_alignment: VAlignment::Top,
        }
    }

    pub fn builder(top_left_x: usize, top_left_y: usize) -> TextBoxBuilder {
        TextBoxBuilder {
            text_box: TextBox::simple(top_left_x, top_left_y, BinaryColor::On),
        }
    }

    pub fn top_center() -> TextBoxBuilder {
        let mut builder = Self::builder(0, 0);
        builder.align(Alignment::Center);
        builder
    }

    pub fn center() -> TextBoxBuilder {
        let mut builder = Self::builder(0, 0);
        builder.align(Alignment::Center).valign(VAlignment::Center);
        builder
    }

    #[inline]
    pub fn is_color_reverse(&self) -> bool {
        matches!(self.fg_color, BinaryColor::Off)
    }
}

impl TextBoxBuilder {
    pub fn fg_color(&mut self, color: BinaryColor) -> &mut Self {
        self.text_box.fg_color = color;
        self
    }

    pub fn width(&mut self, width: usize) -> &mut Self {
        self.text_box.width = width.min(DISPLAY_SIZE_X - self.text_box.top_left_x);
        self
    }

    pub fn height(&mut self, height: usize) -> &mut Self {
        self.text_box.height = height.min(DISPLAY_SIZE_Y - self.text_box.top_left_y);
        self
    }

    pub fn align(&mut self, align: Alignment) -> &mut Self {
        self.text_box.alignment = align;
        self
    }

    pub fn valign(&mut self, align: VAlignment) -> &mut Self {
        self.text_box.v_alignment = align;
        self
    }

    pub fn build(&mut self) -> TextBox {
        self.text_box.clone()
    }
}
