//! On-device DMA test suite.
//!
//! Exercises the `LaxDmaWrite` driver with various word sizes, byte-swap
//! modes, and PIO pipelines. Run with `cargo run --release --examples`,
//! results are printed over defmt RTT.

#![no_std]
#![no_main]
#![allow(dead_code)]

use defmt_rtt as _;
use panic_probe as _;
use pico_display::lax_dma::{self, Config, Destination, LaxDmaWrite, Source, TxReq, TxSize};
use pico_display::MonochromeColor;
use rp2040_hal::dma;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::rom_data;

use pico_display::XOSC_CRYSTAL_FREQ;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp2040_hal::binary_info::EntryAddr; 4] = [
    rp2040_hal::binary_info::rp_program_name!(c"DmaTests"),
    rp2040_hal::binary_info::rp_program_description!(c"On-device DMA test suite"),
    rp2040_hal::binary_info::rp_program_build_attribute!(),
    rp2040_hal::binary_info::rp_cargo_version!(),
];

// ── Memory-to-memory DMA tests ────────────────────────────────────────────

struct TestConfig {
    src: &'static mut [u8; 4],
    dst: &'static mut [u8; 4],
    expected: [u8; 4],
    word_size: lax_dma::TxSize,
    byte_swap: bool,
    increment_src: bool,
    increment_dst: bool,
    test_name: &'static str,
}

fn run_dma_test<CHID: dma::ChannelIndex>(config: TestConfig) {
    let TestConfig {
        src,
        dst,
        expected,
        word_size,
        byte_swap,
        increment_src,
        increment_dst,
        test_name,
    } = config;

    defmt::info!("*** Running DMA test {}, channel {}", test_name, CHID::id());

    let tx_count = match word_size {
        lax_dma::TxSize::_8bit => dst.len() as u32,
        lax_dma::TxSize::_16bit => dst.len() as u32 / core::mem::size_of::<u16>() as u32,
        lax_dma::TxSize::_32bit => dst.len() as u32 / core::mem::size_of::<u32>() as u32,
    };

    let dma_config = Config {
        high_priority: false,
        word_size,
        source: Source {
            address: src.as_ptr(),
            increment: increment_src,
        },
        destination: Destination {
            address: dst.as_mut_ptr(),
            increment: increment_dst,
        },
        tx_count,
        tx_req: TxReq::Permanent,
        byte_swap,
        start: false,
    };

    let dma = LaxDmaWrite::new::<CHID>(dma_config);

    defmt::debug!("DMA source addr: {:x}", src.as_ptr() as usize);
    defmt::debug!("DMA dest addr: {:x}", dst.as_ptr() as usize);
    defmt::debug!("src: {:?}", src);
    defmt::debug!("dst: {:?}", dst);

    defmt::debug!("Starting DMA");
    dma.trigger();
    dma.wait();
    defmt::debug!("DMA done");

    defmt::debug!("src: {:?}", src);
    defmt::debug!("dst: {:?}", dst);

    defmt::debug!("DMA read error: {:?}", dma.read_error());
    defmt::debug!("DMA write error: {:?}", dma.write_error());
    defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr() as usize);
    defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr() as usize);
    defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

    if dst != &expected {
        defmt::error!(
            "!!! {} failed! Expected: {:?}, got: {:?}",
            test_name,
            expected,
            dst
        );
    } else {
        defmt::info!(
            "*** {} passed. Expected: {:?}, got: {:?}",
            test_name,
            expected,
            dst
        );
    }
}

fn run_mem_to_mem_tests() {
    let tests = [
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_8bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_8bit",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_16bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_16bit",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_32bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_32bit",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_8bit,
            byte_swap: true,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_8bit_byte_swap",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [43, 42, 45, 44],
            word_size: TxSize::_16bit,
            byte_swap: true,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_16bit_byte_swap",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [45, 44, 43, 42],
            word_size: TxSize::_32bit,
            byte_swap: true,
            increment_src: true,
            increment_dst: true,
            test_name: "dma_test_32bit_byte_swap",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 42, 42, 42],
            word_size: TxSize::_8bit,
            byte_swap: false,
            increment_src: false,
            increment_dst: true,
            test_name: "dma_test_8bit_fill",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 42, 43],
            word_size: TxSize::_16bit,
            byte_swap: false,
            increment_src: false,
            increment_dst: true,
            test_name: "dma_test_16bit_fill",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_32bit,
            byte_swap: false,
            increment_src: false,
            increment_dst: true,
            test_name: "dma_test_32bit_fill",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [45, 0, 0, 0],
            word_size: TxSize::_8bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: false,
            test_name: "dma_test_8bit_dst_fixed",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [44, 45, 0, 0],
            word_size: TxSize::_16bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: false,
            test_name: "dma_test_16bit_dst_fixed",
        },
        TestConfig {
            src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
            dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
            expected: [42, 43, 44, 45],
            word_size: TxSize::_32bit,
            byte_swap: false,
            increment_src: true,
            increment_dst: false,
            test_name: "dma_test_32bit_dst_fixed",
        },
    ];

    for test in tests.into_iter() {
        run_dma_test::<dma::CH5>(test);
    }
}

// ── PIO pipeline tests ────────────────────────────────────────────────────

fn test_with_pio_invert_twice() {
    const SIZE: usize = 32;
    let input_buffer = [0x55u8; SIZE];
    let mut output_buffer = [0u8; SIZE];
    let input_buffer_addr = [input_buffer.as_ptr() as u32];

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let invert_pio = pio_programs::invert_pio_program();
    let invert_pio_again = pio_programs::invert_pio_again_program();

    let _dma = pac.DMA.split(&mut pac.RESETS);
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (sm0, rx0, tx0) =
        rp2040_hal::pio::PIOBuilder::from_installed_program(pio.install(&invert_pio).unwrap())
            .autopull(false)
            .autopush(false)
            .build(sm0);
    sm0.start();

    let (sm1, rx1, tx1) = rp2040_hal::pio::PIOBuilder::from_installed_program(
        pio.install(&invert_pio_again).unwrap(),
    )
    .autopull(false)
    .autopush(false)
    .build(sm1);
    sm1.start();

    let txf0 = tx0.fifo_address();
    let rxf0 = rx0.fifo_address();

    let txf1 = tx1.fifo_address();
    let rxf1 = rx1.fifo_address();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    let dma3 = LaxDmaWrite::new::<dma::CH3>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: rxf1.cast(),
            increment: false,
        },
        destination: Destination {
            address: output_buffer.as_mut_ptr(),
            increment: true,
        },
        tx_count: SIZE as u32 / 4,
        tx_req: TxReq::Pio0Rx1,
        byte_swap: false,
        start: true,
    });

    let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: rxf0.cast(),
            increment: false,
        },
        destination: Destination {
            address: txf1.cast_mut().cast(),
            increment: false,
        },
        tx_count: SIZE as u32 / 4,
        tx_req: TxReq::Pio0Tx1,
        byte_swap: false,
        start: true,
    });

    let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: core::ptr::null(),
            increment: true,
        },
        destination: Destination {
            address: txf0.cast_mut().cast(),
            increment: false,
        },
        tx_count: SIZE as u32 / 4,
        tx_req: TxReq::Pio0Tx0,
        byte_swap: false,
        start: false,
    });

    let dma0 = LaxDmaWrite::new::<dma::CH0>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: input_buffer_addr.as_ptr().cast(),
            increment: false,
        },
        destination: Destination {
            address: dma1.read_trig_addr().cast_mut().cast(),
            increment: false,
        },
        tx_count: 1,
        tx_req: TxReq::Permanent,
        byte_swap: false,
        start: false,
    });

    dma0.trigger();

    dma0.wait();
    dma2.wait();
    dma3.wait();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);
}

fn test_with_pio_expand_12times() {
    const SIZE: usize = 4;
    let input_buffer = [0x5au8; SIZE];
    let mut output_buffer = [0u8; 12 * SIZE];

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let expand_times12_pio = pio_programs::gen_monochrome_expand_program(MonochromeColor::Bpp1);

    let _dma = pac.DMA.split(&mut pac.RESETS);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let installed_pio = pio.install(&expand_times12_pio).unwrap();
    let (sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed_pio)
        .autopull(true)
        .autopush(true)
        .build(sm0);
    sm.start();

    let txf = tx.fifo_address();
    let rxf = rx.fifo_address();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: input_buffer.as_ptr(),
            increment: true,
        },
        destination: Destination {
            address: txf.cast_mut().cast(),
            increment: false,
        },
        tx_count: SIZE as u32 / 4,
        tx_req: TxReq::Pio0Tx0,
        byte_swap: false,
        start: false,
    });

    let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: rxf.cast(),
            increment: false,
        },
        destination: Destination {
            address: output_buffer.as_mut_ptr(),
            increment: true,
        },
        tx_count: 12 * SIZE as u32 / 4,
        tx_req: TxReq::Pio0Rx0,
        byte_swap: false,
        start: false,
    });

    dma1.trigger();
    dma2.trigger();

    dma1.wait();
    dma2.wait();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);
}

fn test_with_pio_expand_dynamic(color: MonochromeColor) {
    const RGB_BPP: u8 = 12;
    let bpp = RGB_BPP / color as u8;

    const SIZE: usize = 8;
    let input_buffer: [u8; SIZE] = [0xaa; SIZE];
    let mut output_buffer: [u8; 12 * SIZE] = [0u8; 12 * SIZE];

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let _dma = pac.DMA.split(&mut pac.RESETS);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let expand_pio = pio_programs::gen_monochrome_expand_program(color);
    let installed_pio = pio.install(&expand_pio).unwrap();
    let (sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed_pio)
        .autopull(true)
        .autopush(true)
        .build(sm0);
    sm.start();

    let txf = tx.fifo_address();
    let rxf = rx.fifo_address();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: input_buffer.as_ptr(),
            increment: true,
        },
        destination: Destination {
            address: txf.cast_mut().cast(),
            increment: false,
        },
        tx_count: SIZE as u32 / 4,
        tx_req: TxReq::Pio0Tx0,
        byte_swap: false,
        start: false,
    });

    let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        source: Source {
            address: rxf.cast(),
            increment: false,
        },
        destination: Destination {
            address: output_buffer.as_mut_ptr(),
            increment: true,
        },
        tx_count: bpp as u32 * SIZE as u32 / 4,
        tx_req: TxReq::Pio0Rx0,
        byte_swap: false,
        start: false,
    });

    dma1.trigger();
    dma2.trigger();

    dma1.wait();
    dma2.wait();

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);
}

// ── Entry point ───────────────────────────────────────────────────────────

#[rp2040_hal::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM version {:x}",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
    );

    defmt::info!("=== Memory-to-memory DMA tests ===");
    run_mem_to_mem_tests();

    // NOTE: the PIO tests each call Peripherals::take() and therefore
    // cannot run in the same binary as each other or after
    // run_mem_to_mem_tests (which does not take peripherals).
    // Uncomment ONE of the following to run it:
    //
    // test_with_pio_invert_twice();
    // test_with_pio_expand_12times();
    // test_with_pio_expand_dynamic(MonochromeColor::Bpp1);

    defmt::info!("=== All tests done ===");

    loop {
        cortex_m::asm::wfi();
    }
}
