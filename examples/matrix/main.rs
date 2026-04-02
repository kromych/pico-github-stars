//! Matrix digital rain screensaver.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Display2_8, PicoDisplay, Rgb565};

mod matrix;
mod matrix_symbols;

type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

fn time_us() -> u32 {
    pico_display::timer().timerawl().read()
}

fn time_us64() -> u64 {
    let t = pico_display::timer();
    t.timelr().read() as u64 | ((t.timehr().read() as u64) << 32)
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    let mut display = pico_display::pico_display_new!(MyDisplay);

    defmt::info!("Matrix screensaver, time {} us", time_us64());

    let seed = time_us();
    let mut matrix = matrix::Matrix::new(seed);

    defmt::info!("Matrix screensaver started, seed={}", seed);

    loop {
        matrix.step(&mut display);
        display.flush();
    }
}
