//! Matrix digital rain screensaver for the PIO display.
//!
//! Each column displays a falling stream of glyphs from `matrix_symbols`.
//! The head of each stream is drawn at full brightness (level 5) and the
//! trail fades through levels 4→1.
//!
//! Movement is pixel-smooth: drops advance by 1–3 pixels per frame
//! instead of jumping a full glyph row at a time.
//!
//! At Bpp1 the brightness manifests as fewer lit pixels per glyph at lower
//! levels (the palette data naturally thins out). At Bpp2/Bpp4 the pixel
//! grey value also decreases for dimmer trail positions, giving a smooth
//! fade even when the glyph shape doesn't change much.

use crate::matrix_symbols::{
    BRIGHTNESS_LEVELS, GLYPHS, GLYPH_COUNT, GLYPH_HEIGHT, GLYPH_WIDTH, PALETTE_24BIT,
};
use pico_display::{ColorMode, DisplayAttributes, PicoDisplay};
use pio_programs::DisplayMode;

/// Green channel threshold: palette entries with green ≥ this are "on".
const GREEN_THRESHOLD: u8 = 70;

/// Length of the visible trail behind the head (including the head).
const TRAIL_LEN: usize = BRIGHTNESS_LEVELS; // 6

// ── Pre-thresholded glyph bitmaps ──────────────────────────────────────────

/// One row of a glyph packed into 14 bits of a u16 (bit 0 = leftmost pixel).
type GlyphMasks = [[[u16; GLYPH_HEIGHT]; GLYPH_COUNT]; BRIGHTNESS_LEVELS];

/// Build the pre-thresholded glyph bitmaps from `GLYPHS` + `PALETTE_24BIT`.
/// Precomputed into RAM so there are no flash reads in the hot loop.
fn build_glyph_masks() -> GlyphMasks {
    let mut masks = [[[0u16; GLYPH_HEIGHT]; GLYPH_COUNT]; BRIGHTNESS_LEVELS];
    let mut brightness = 0;
    while brightness < BRIGHTNESS_LEVELS {
        let mut glyph = 0;
        while glyph < GLYPH_COUNT {
            let mut y = 0;
            while y < GLYPH_HEIGHT {
                let mut row: u16 = 0;
                let mut x = 0;
                while x < GLYPH_WIDTH {
                    let palette_idx = GLYPHS[brightness][glyph][y][x] as usize;
                    let green = PALETTE_24BIT[palette_idx][1];
                    if green >= GREEN_THRESHOLD {
                        row |= 1 << x;
                    }
                    x += 1;
                }
                masks[brightness][glyph][y] = row;
                y += 1;
            }
            glyph += 1;
        }
        brightness += 1;
    }
    masks
}

// ── Drop / Matrix state ────────────────────────────────────────────────────

/// A single falling column of glyphs.
struct Drop {
    /// Pixel-Y of the top edge of the head (bottom-most) glyph.
    head_y: i16,
    /// Pixels to advance per frame (1–3).
    speed: u8,
    /// Sub-glyph accumulator: counts pixels within the current glyph cell.
    sub_y: u8,
    /// Glyph index for each visible trail position (head at [0]).
    glyphs: [u8; TRAIL_LEN],
}

pub struct Matrix {
    drops: [Drop; Self::COLUMNS],
    masks: GlyphMasks,
    rng: u32,
    rgb565_palette: [u16; 256],
    rgb444_palette: [u16; 256],
    /// Display width/height captured once from type params.
    width: u16,
    height: u16,
}

/// Map a brightness level (0–5) to a pixel grey value for the given mode.
const fn pixel_for_brightness(mode: DisplayMode) -> [u8; BRIGHTNESS_LEVELS] {
    let bpp = match mode {
        DisplayMode::Mono(c) => c.bits_per_pixel(),
        DisplayMode::Rgb444 | DisplayMode::Rgb565 => 1, // placeholder, not used at runtime
    };
    let max = (1u16 << bpp) - 1;
    let mut table = [0u8; BRIGHTNESS_LEVELS];
    if bpp == 1 {
        table[1] = 1;
        table[2] = 1;
        table[3] = 1;
        table[4] = 1;
        table[5] = 1;
    } else {
        let mut i = 1;
        while i < BRIGHTNESS_LEVELS {
            table[i] = ((i as u16 * max + 2) / 5) as u8;
            i += 1;
        }
    }
    table
}

impl Matrix {
    /// Number of glyph columns — derived from the display width at the call site.
    const COLUMNS: usize = 320 / GLYPH_WIDTH; // max possible; actual used is ≤ this

    pub fn new(seed: u32) -> Self {
        // Precompute glyph masks and palettes into RAM —
        // no flash reads in the hot loop.
        let masks = build_glyph_masks();

        let mut rgb565_palette = [0u16; 256];
        let mut rgb444_palette = [0u16; 256];
        let mut i = 0;
        while i < 256 {
            let r = PALETTE_24BIT[i][0] as u16;
            let g = PALETTE_24BIT[i][1] as u16;
            let b = PALETTE_24BIT[i][2] as u16;
            rgb565_palette[i] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            rgb444_palette[i] = ((r >> 4) << 8) | ((g >> 4) << 4) | (b >> 4);
            i += 1;
        }

        let mut m = Matrix {
            drops: core::array::from_fn(|_| Drop {
                head_y: 0,
                speed: 1,
                sub_y: 0,
                glyphs: [0; TRAIL_LEN],
            }),
            masks,
            rng: if seed == 0 { 1 } else { seed },
            rgb565_palette,
            rgb444_palette,
            width: 0,
            height: 0,
        };
        // Initial stagger — uses max dimensions; step() will clip to actual.
        let screen_h = 240i16;
        let trail_px = (TRAIL_LEN * GLYPH_HEIGHT) as i16;
        for col in 0..Self::COLUMNS {
            let offset = (m.rand() % (screen_h as u32 + trail_px as u32)) as i16;
            m.drops[col].head_y = -offset;
            m.drops[col].speed = 1 + (m.rand() % 3) as u8;
            m.drops[col].sub_y = 0;
            for i in 0..TRAIL_LEN {
                m.drops[col].glyphs[i] = (m.rand() % GLYPH_COUNT as u32) as u8;
            }
        }
        m
    }

    fn rand(&mut self) -> u32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        self.rng
    }

    pub fn step<TDispAttr: DisplayAttributes, TColor: ColorMode>(
        &mut self,
        display: &mut PicoDisplay<TDispAttr, TColor>,
    ) {
        let mode = TColor::MODE;
        let w = TDispAttr::WIDTH;
        let h = TDispAttr::HEIGHT;
        self.width = w;
        self.height = h;

        let columns = w as usize / GLYPH_WIDTH;
        let x_offset = ((w as usize - columns * GLYPH_WIDTH) / 2) as u16;
        let screen_h = h as i16;
        let gh = GLYPH_HEIGHT as i16;
        let trail_px = (TRAIL_LEN * GLYPH_HEIGHT) as i16;
        let pfb = pixel_for_brightness(mode);

        display.clear();

        for col in 0..columns {
            let drop = &self.drops[col];
            let px = x_offset + (col as u16) * (GLYPH_WIDTH as u16);

            for t in 0..TRAIL_LEN {
                let py = drop.head_y - (t as i16) * gh;
                if py >= screen_h || py + gh <= 0 {
                    continue;
                }
                let brightness = (BRIGHTNESS_LEVELS - 1) - t;
                let glyph_idx = drop.glyphs[t] as usize;

                match mode {
                    DisplayMode::Rgb565 => {
                        blit_glyph_rgb565(
                            display,
                            px,
                            py,
                            screen_h,
                            &self.masks[brightness][glyph_idx],
                            brightness,
                            glyph_idx,
                            &self.rgb565_palette,
                        );
                    }
                    DisplayMode::Rgb444 => {
                        blit_glyph_rgb444(
                            display,
                            px,
                            py,
                            screen_h,
                            &self.masks[brightness][glyph_idx],
                            brightness,
                            glyph_idx,
                            &self.rgb444_palette,
                        );
                    }
                    DisplayMode::Mono(_) => {
                        let pixel_val = pfb[brightness];
                        if pixel_val > 0 {
                            blit_glyph_mono(
                                display,
                                px,
                                py,
                                screen_h,
                                &self.masks[brightness][glyph_idx],
                                pixel_val,
                            );
                        }
                    }
                }
            }
        }

        // Advance drops.
        for col in 0..columns {
            let speed = self.drops[col].speed;
            self.drops[col].head_y += speed as i16;
            self.drops[col].sub_y += speed;

            if self.drops[col].sub_y >= GLYPH_HEIGHT as u8 {
                self.drops[col].sub_y -= GLYPH_HEIGHT as u8;
                let new_glyph = (self.rand() % GLYPH_COUNT as u32) as u8;
                self.drops[col].glyphs.rotate_right(1);
                self.drops[col].glyphs[0] = new_glyph;
            }

            let tail_top = self.drops[col].head_y - (TRAIL_LEN as i16 - 1) * gh;
            if tail_top >= screen_h {
                let offset = (self.rand() % (trail_px as u32)) as i16;
                let speed = 1 + (self.rand() % 3) as u8;
                let mut glyphs = [0u8; TRAIL_LEN];
                for g in &mut glyphs {
                    *g = (self.rand() % GLYPH_COUNT as u32) as u8;
                }
                self.drops[col].head_y = -offset;
                self.drops[col].speed = speed;
                self.drops[col].sub_y = 0;
                self.drops[col].glyphs = glyphs;
            }
        }
    }
}

// ── Blit helpers ───────────────────────────────────────────────────────────
// blit_glyph_mono: no flash reads in the hot loop (uses precomputed RAM masks only).
// blit_glyph_rgb565/rgb444: still read GLYPHS from flash for palette index lookup.

#[inline(never)]
fn blit_glyph_mono<TDispAttr: DisplayAttributes, TColor: ColorMode>(
    display: &mut PicoDisplay<TDispAttr, TColor>,
    px: u16,
    py: i16,
    screen_h: i16,
    row_masks: &[u16; GLYPH_HEIGHT],
    pixel_val: u8,
) {
    for y in 0..GLYPH_HEIGHT as i16 {
        let sy = py + y;
        if sy < 0 {
            continue;
        }
        if sy >= screen_h {
            break;
        }
        let mut bits = row_masks[y as usize];
        while bits != 0 {
            let x = bits.trailing_zeros() as u16;
            display.set_pixel_value(px + x, sy as u16, pixel_val);
            bits &= bits - 1;
        }
    }
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn blit_glyph_rgb565<TDispAttr: DisplayAttributes, TColor: ColorMode>(
    display: &mut PicoDisplay<TDispAttr, TColor>,
    px: u16,
    py: i16,
    screen_h: i16,
    row_masks: &[u16; GLYPH_HEIGHT],
    brightness: usize,
    glyph_idx: usize,
    palette: &[u16; 256],
) {
    for y in 0..GLYPH_HEIGHT as i16 {
        let sy = py + y;
        if sy < 0 {
            continue;
        }
        if sy >= screen_h {
            break;
        }
        let mut bits = row_masks[y as usize];
        while bits != 0 {
            let x = bits.trailing_zeros() as u16;
            let pal_idx = GLYPHS[brightness][glyph_idx][y as usize][x as usize];
            let color = palette[pal_idx as usize];
            display.set_pixel_rgb565(px + x, sy as u16, color);
            bits &= bits - 1;
        }
    }
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn blit_glyph_rgb444<TDispAttr: DisplayAttributes, TColor: ColorMode>(
    display: &mut PicoDisplay<TDispAttr, TColor>,
    px: u16,
    py: i16,
    screen_h: i16,
    row_masks: &[u16; GLYPH_HEIGHT],
    brightness: usize,
    glyph_idx: usize,
    palette: &[u16; 256],
) {
    for y in 0..GLYPH_HEIGHT as i16 {
        let sy = py + y;
        if sy < 0 {
            continue;
        }
        if sy >= screen_h {
            break;
        }
        let mut bits = row_masks[y as usize];
        while bits != 0 {
            let x = bits.trailing_zeros() as u16;
            let pal_idx = GLYPHS[brightness][glyph_idx][y as usize][x as usize];
            let color = palette[pal_idx as usize];
            display.set_pixel_rgb444(px + x, sy as u16, color);
            bits &= bits - 1;
        }
    }
}
