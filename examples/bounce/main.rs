//! Bouncing ball in 1-bpp monochrome.
//!
//! A filled circle bounces around the screen, reversing direction when
//! it hits an edge. Demonstrates the double-buffered Bpp1 mode and
//! basic frame-rate animation.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod draw;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    defmt::info!("Bounce demo starting");

    let mut display = pico_display::pico_display_new!(draw::MyDisplay);
    let mut state = draw::BounceState::new(&display);

    defmt::info!(
        "Bounce demo started ({}x{}, Bpp1)",
        state.w,
        state.h
    );

    loop {
        state.draw_and_advance(&mut display);
        display.flush();
    }
}
