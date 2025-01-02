#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use pico_display_pimoroni::MonochromeColor;
use pico_display_pimoroni::MonochromeDisplayBuffer;
use pico_display_pimoroni::PicoDisplay2_8;
use rp2040_hal::rom_data;

mod lax_dma;
mod pico_display_pimoroni;
mod rng;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

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

#[allow(dead_code)]
mod time {
    pub fn time_us() -> u32 {
        unsafe { (*rp2040_pac::TIMER::PTR).timerawl().read().bits() }
    }

    pub fn time_us64() -> u64 {
        unsafe {
            (*rp2040_pac::TIMER::PTR).timelr().read().bits() as u64
                | (((*rp2040_pac::TIMER::PTR).timehr().read().bits() as u64) << 32)
        }
    }
}

const DISPLAY_WIDTH: u16 = 320;
const DISPLAY_HEIGHT: u16 = 240;
const DISPLAY_COLOR: MonochromeColor = MonochromeColor::Bpp1;
const DISPLAY_BUFFER_SIZE: usize =
    (DISPLAY_WIDTH as usize) * (DISPLAY_HEIGHT as usize) / DISPLAY_COLOR.pixel_per_byte() as usize;

#[rp2040_hal::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}, time {:x} us",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
        time::time_us64()
    );

    defmt::info!("Display size: {}x{}", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    defmt::info!("Display buffer size: {}", DISPLAY_BUFFER_SIZE);
    let buffer = [0u8; DISPLAY_BUFFER_SIZE];
    let mut fb =
        MonochromeDisplayBuffer::new(DISPLAY_WIDTH, DISPLAY_HEIGHT, buffer, DISPLAY_COLOR).unwrap();

    let red_rgb565 = 0xF800u16;
    let green_rgb565 = 0x07E0u16;
    let blue_rgb565 = 0x001Fu16;
    let color_be = blue_rgb565.to_be();
    let mut buffer = [color_be; 100 * 100];
    let mut display = PicoDisplay2_8::new();
    display.flush(&mut buffer, 120, 120, 200 - 1, 200 - 1);

    loop {
        cortex_m::asm::wfe();
        defmt::info!("WFE time: {:x}", time::time_us64());
    }
}
