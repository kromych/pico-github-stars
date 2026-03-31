#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::{Display2_8, PicoDisplay, Rgb565};
use rp2040_hal::rom_data;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp2040_hal::binary_info::EntryAddr; 4] = [
    rp2040_hal::binary_info::rp_program_name!(c"PicoDisplay"),
    rp2040_hal::binary_info::rp_program_description!(c"PicoDisplay experiments"),
    rp2040_hal::binary_info::rp_program_build_attribute!(),
    rp2040_hal::binary_info::rp_cargo_version!(),
];

type MyDisplay = PicoDisplay<Display2_8, Rgb565>;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut display = pico_display::pico_display_new!(MyDisplay);

    defmt::info!(
        "Board {}, git revision {:x}",
        rom_data::copyright_string(),
        rom_data::git_revision(),
    );

    display.clear();
    display.flush();

    loop {
        cortex_m::asm::wfi();
    }
}
