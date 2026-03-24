#![no_std]

use embassy_futures::yield_now;
use embassy_time::{Duration, Instant};
use embedded_graphics::primitives::{PrimitiveStyle, Styled, StyledPixels};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal_async::i2c::I2c;
use panic_probe as _;

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

    font_large: Font,
    font_medium: Font,
    font_medium_bold: Font,
    font_small: Font,
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
        let image_10x20 = include_bytes!("../fonts/font_10x20.raw");
        let font_10x20 = Font {
            image: image_10x20,
            width: 10,
            height: 20,
        };
        let image_8x13 = include_bytes!("../fonts/font_8x13.raw");
        let font_8x13 = Font {
            image: image_8x13,
            width: 8,
            height: 13,
        };
        let image_8x13_bold = include_bytes!("../fonts/font_8x13_bold.raw");
        let font_8x13_bold = Font {
            image: image_8x13_bold,
            width: 8,
            height: 13,
        };
        let image_6x10 = include_bytes!("../fonts/font_6x10.raw");
        let font_6x10 = Font {
            image: image_6x10,
            width: 6,
            height: 10,
        };
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
            font_large: font_10x20,
            font_medium: font_8x13,
            font_medium_bold: font_8x13_bold,
            font_small: font_6x10,
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
        for page in 0..8 {
            let base = page * 128;

            Self::write_commands(&mut self.i2c, self.addr, &[0xB0 + page as u8, 0x00, 0x10]).await;

            Self::write_data(
                &mut self.i2c,
                self.addr,
                &self.framebuffer[base..base + 128],
            )
            .await;
        }
    }

    /// Flushes only modified regions using dirty page tracking.
    ///
    /// Each page (8 vertical pixels) tracks min/max modified columns.
    /// Only those regions are transmitted over I2C.
    pub async fn flush(&mut self) {
        let mut temp = [0u8; 128]; // one page max width

        for page in 0..8 {
            // ---- Step 1: copy dirty info out ----
            let (dirty, start, end) = {
                let d = &self.dirty[page];
                (d.dirty, d.min_col, d.max_col)
            };

            if !dirty {
                continue;
            }

            let start = start.min(127);
            let end = end.min(127);

            let len = (end - start + 1) as usize;
            let base = page * 128;

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
                min_col: 127,
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
            max_col: 127,
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

    /// Sets a pixel to ON
    #[inline]
    pub fn set_pixel(&mut self, x: u32, y: u32) {
        let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
        self.framebuffer[index] |= 1 << bit;
    }

    /// Sets a pixel to OFF.
    #[inline]
    pub fn unset_pixel(&mut self, x: u32, y: u32) {
        let (index, bit) = self.map_pixel_mut(x as u32, y as u32);
        self.framebuffer[index] &= !(1 << bit);
    }

    /// Updates a pixel with a boolean value.
    #[inline]
    pub fn update_pixel(&mut self, x: u32, y: u32, color: bool) {
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
        top_left_x: u32,
        top_left_y: u32,
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
        top_left_x: u32,
        top_left_y: u32,
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
            let y = top_left_y + j as u32;
            for i in 0..width {
                let x = top_left_x + i;
                if (i & 7) != 0 {
                    byte <<= 1;
                } else {
                    byte = bitmap[y as usize * byte_width + i as usize / 8];
                }

                let (index, bit) = self.map_pixel_mut(x, y);
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
        font_size: &FontSize,
        color: BinaryColor,
        last_yield: &mut Instant,
    ) -> usize {
        let font = match font_size {
            FontSize::Large => &self.font_large,
            FontSize::Medium => match color {
                BinaryColor::On => &self.font_medium,
                BinaryColor::Off => &self.font_medium_bold,
            },
            FontSize::Small => &self.font_small,
        };

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
                        } else {
                            self.framebuffer[data_index] &= !(1 << data_bit);
                        }
                    }
                    BinaryColor::Off => {
                        // Inverted rendering
                        if value == 1 {
                            self.framebuffer[data_index] &= !(1 << data_bit);
                        } else {
                            self.framebuffer[data_index] |= 1 << data_bit;
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
    pub async fn draw_string(
        &mut self,
        s: &str,
        mut x: usize,
        y: usize,
        font_size: FontSize,
        color: BinaryColor,
    ) {
        let mut last_yield = Instant::now();
        for ch in s.chars() {
            x = self
                .draw_char_at(ch, x, y, &font_size, color, &mut last_yield)
                .await;
        }
    }

    fn map_pixel_mut(&mut self, x: u32, y: u32) -> (usize, usize) {
        let page = (y / 8) as usize;

        let entry = &mut self.dirty[page];
        entry.dirty = true;
        entry.min_col = entry.min_col.min(x as u8);
        entry.max_col = entry.max_col.max(x as u8);

        let index = x as usize + 128 * page;
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
            let chunk = (data.len() - offset).min(128);

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
                self.update_pixel((cx + px) as u32, (cy + py) as u32, color);
            }
            if last_yield.elapsed() > self.yield_interval {
                yield_now().await;
                *last_yield = Instant::now();
            }
        }
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
