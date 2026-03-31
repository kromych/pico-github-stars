//! Matrix digital rain screensaver.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Display2_8, PicoDisplay, Rgb565};
use rp2040_hal::rom_data;

mod matrix;
mod matrix_symbols;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp2040_hal::binary_info::EntryAddr; 4] = [
    rp2040_hal::binary_info::rp_program_name!(c"Matrix"),
    rp2040_hal::binary_info::rp_program_description!(c"Matrix digital rain screensaver"),
    rp2040_hal::binary_info::rp_program_build_attribute!(),
    rp2040_hal::binary_info::rp_cargo_version!(),
];

type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

fn time_us() -> u32 {
    unsafe { (*rp2040_pac::TIMER::PTR).timerawl().read().bits() }
}

fn time_us64() -> u64 {
    unsafe {
        (*rp2040_pac::TIMER::PTR).timelr().read().bits() as u64
            | (((*rp2040_pac::TIMER::PTR).timehr().read().bits() as u64) << 32)
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut display = pico_display::pico_display_new!(MyDisplay);

    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}, time {:x} us",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
        time_us64()
    );

    let seed = time_us();
    let mut matrix = matrix::Matrix::new(seed);

    defmt::info!("Matrix screensaver started, seed={}", seed);

    loop {
        matrix.step(&mut display);
        display.flush();
    }
}
