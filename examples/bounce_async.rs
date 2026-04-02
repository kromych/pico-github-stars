//! Bouncing ball in 1-bpp monochrome (async version).
//!
//! Same as `bounce` but uses `flush_async()` to yield to the executor
//! between frames, allowing other embassy tasks to run.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[path = "bounce/draw.rs"]
mod draw;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    defmt::info!("Bounce async demo starting");

    let mut display = pico_display::pico_display_new!(draw::MyDisplay);
    let mut state = draw::BounceState::new(&display);

    defmt::info!(
        "Bounce async demo started ({}x{}, Bpp1)",
        state.w,
        state.h
    );

    loop {
        state.draw_and_advance(&mut display);
        display.flush_async().await;
    }
}
