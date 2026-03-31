//! Animated color bar pattern in RGB565.
//!
//! Vertical rainbow bars scroll horizontally across the display.
//! Each column's hue is derived from its position plus a per-frame
//! offset, producing a smooth, continuously moving spectrum.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Display2_8, PicoDisplay, Rgb565};
use rp2040_hal::rom_data;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp2040_hal::binary_info::EntryAddr; 4] = [
    rp2040_hal::binary_info::rp_program_name!(c"ColorBars"),
    rp2040_hal::binary_info::rp_program_description!(c"Animated rainbow color bars"),
    rp2040_hal::binary_info::rp_program_build_attribute!(),
    rp2040_hal::binary_info::rp_cargo_version!(),
];

type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

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
fn build_hue_lut(lut: &mut [u16; 360]) {
    let mut h = 0u16;
    while h < 360 {
        lut[h as usize] = hsv_to_rgb565(h, 255, 255);
        h += 1;
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM version {:x}",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
    );

    let mut display = pico_display::pico_display_new!(MyDisplay);

    let w = display.width();
    let h = display.height();

    // No flash reads in the hot loop — precompute palette into RAM.
    let mut hue_lut = [0u16; 360];
    build_hue_lut(&mut hue_lut);

    let mut offset: u16 = 0;

    defmt::info!("Color bars started ({}x{})", w, h);

    loop {
        display.clear();

        for x in 0..w {
            let hue = ((x + offset) % 360) as usize;
            let color = hue_lut[hue];
            for y in 0..h {
                display.set_pixel_rgb565(x, y, color);
            }
        }

        display.flush();

        offset = (offset + 1) % 360;
    }
}
