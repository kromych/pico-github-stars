#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Display2_8, PicoDisplay, Rgb565};

type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    let mut display = pico_display::pico_display_new!(MyDisplay);

    defmt::info!("PicoDisplay started");

    display.clear();
    display.flush();

    loop {
        cortex_m::asm::wfi();
    }
}
