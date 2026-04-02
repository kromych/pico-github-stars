//! Animated color bar pattern in RGB565.
//!
//! Vertical rainbow bars scroll horizontally across the display.
//! Each column's hue is derived from its position plus a per-frame
//! offset, producing a smooth, continuously moving spectrum.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod draw;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    defmt::info!("Color bars starting");

    let mut display = pico_display::pico_display_new!(draw::MyDisplay);

    let mut hue_lut = [0u16; 360];
    draw::build_hue_lut(&mut hue_lut);

    let w = display.width();
    let h = display.height();
    let mut offset: u16 = 0;

    defmt::info!("Color bars started ({}x{})", w, h);

    loop {
        draw::draw_frame(&mut display, &hue_lut, offset);
        display.flush();
        offset = (offset + 1) % 360;
    }
}
