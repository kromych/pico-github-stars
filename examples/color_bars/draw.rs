use pico_display::{Display2_8, PicoDisplay, Rgb565};

pub type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

/// Convert 8-bit RGB to RGB565.
const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

/// HSV to RGB565. `h` is 0..360, `s` and `v` are 0..255.
fn hsv_to_rgb565(h: u16, s: u8, v: u8) -> u16 {
    if s == 0 {
        return rgb565(v, v, v);
    }

    let region = h / 60;
    let remainder = ((h % 60) as u32 * 255) / 60;

    let p = ((v as u32 * (255 - s as u32)) / 255) as u8;
    let q = ((v as u32 * (255 - (s as u32 * remainder) / 255)) / 255) as u8;
    let t = ((v as u32 * (255 - (s as u32 * (255 - remainder)) / 255)) / 255) as u8;

    match region {
        0 => rgb565(v, t, p),
        1 => rgb565(q, v, p),
        2 => rgb565(p, v, t),
        3 => rgb565(p, q, v),
        4 => rgb565(t, p, v),
        _ => rgb565(v, p, q),
    }
}

/// Pre-compute one full hue cycle (360 entries) at full saturation and value.
pub fn build_hue_lut(lut: &mut [u16; 360]) {
    let mut h = 0u16;
    while h < 360 {
        lut[h as usize] = hsv_to_rgb565(h, 255, 255);
        h += 1;
    }
}

pub fn draw_frame(display: &mut MyDisplay, hue_lut: &[u16; 360], offset: u16) {
    let w = display.width();
    let h = display.height();

    display.clear();

    for x in 0..w {
        let hue = ((x + offset) % 360) as usize;
        let color = hue_lut[hue];
        for y in 0..h {
            display.set_pixel_rgb565(x, y, color);
        }
    }
}
