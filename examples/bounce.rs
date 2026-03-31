//! Bouncing ball in 1-bpp monochrome.
//!
//! A filled circle bounces around the screen, reversing direction when
//! it hits an edge. Demonstrates the double-buffered Bpp1 mode and
//! basic frame-rate animation.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Bpp1, Display2_8, PicoDisplay};
use rp2040_hal::rom_data;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp2040_hal::binary_info::EntryAddr; 4] = [
    rp2040_hal::binary_info::rp_program_name!(c"Bounce"),
    rp2040_hal::binary_info::rp_program_description!(c"Bouncing ball demo (Bpp1)"),
    rp2040_hal::binary_info::rp_program_build_attribute!(),
    rp2040_hal::binary_info::rp_cargo_version!(),
];

type MyDisplay = PicoDisplay<Display2_8, Bpp1>;

const RADIUS: i16 = 16;

/// Draw a filled circle using the midpoint algorithm.
fn fill_circle(display: &mut MyDisplay, cx: i16, cy: i16, r: i16) {
    let w = display.width() as i16;
    let h = display.height() as i16;

    for dy in -r..=r {
        let sy = cy + dy;
        if sy < 0 || sy >= h {
            continue;
        }
        // Integer sqrt: dx² + dy² ≤ r²
        let dx_max = isqrt((r * r - dy * dy) as u32) as i16;
        let x0 = (cx - dx_max).max(0) as u16;
        let x1 = ((cx + dx_max).min(w - 1)) as u16;
        for x in x0..=x1 {
            display.set_pixel(x, sy as u16, true);
        }
    }
}

/// Integer square root.
fn isqrt(n: u32) -> u32 {
    if n == 0 {
        return 0;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
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

    let w = display.width() as i16;
    let h = display.height() as i16;

    let mut cx: i16 = w / 4;
    let mut cy: i16 = h / 3;
    let mut dx: i16 = 2;
    let mut dy: i16 = 1;

    defmt::info!("Bounce demo started ({}x{}, Bpp1)", w, h);

    loop {
        display.clear();
        fill_circle(&mut display, cx, cy, RADIUS);
        display.flush();

        cx += dx;
        cy += dy;

        if cx - RADIUS <= 0 || cx + RADIUS >= w {
            dx = -dx;
            cx += dx;
        }
        if cy - RADIUS <= 0 || cy + RADIUS >= h {
            dy = -dy;
            cy += dy;
        }
    }
}
