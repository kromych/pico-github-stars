//! On-device DMA test suite.
//!
//! Exercises the `LaxDmaWrite` driver with various word sizes, byte-swap
//! modes, and PIO pipelines. Run with `cargo run --release --examples`,
//! results are printed over defmt RTT.

#![no_std]
#![no_main]
#![allow(dead_code)]

use defmt_rtt as _;
use embassy_rp::pac;
use panic_probe as _;
use pico_display::lax_dma::{self, Config, LaxDmaWrite, TreqSel, TxSize};
use pico_display::MonochromeColor;

// ── PIO program loading helper ─────────────────────────────────────────────

/// Load a PIO program into PIO0 instruction memory at the given offset.
/// Returns the number of instructions written.
fn load_pio_program(
    program: &pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>,
    offset: usize,
) -> usize {
    let pio0 = pac::PIO0;
    for (i, &instr) in program.code.iter().enumerate() {
        pio0.instr_mem(offset + i).write(|w| w.set_instr_mem(instr));
    }
    program.code.len()
}

/// Reset PIO0: disable all SMs, restart them, clear FIFOs and instruction memory.
fn reset_pio0() {
    let pio0 = pac::PIO0;

    // Disable all state machines
    pio0.ctrl().write(|w| w.set_sm_enable(0));

    // Restart all SMs (clears internal state)
    pio0.ctrl().write(|w| {
        w.set_sm_restart(0xF);
        w.set_clkdiv_restart(0xF);
    });

    // Clear instruction memory
    for i in 0..32 {
        pio0.instr_mem(i).write(|w| w.set_instr_mem(0));
    }

    // Reset SM configs
    for sm_idx in 0..4 {
        let sm = pio0.sm(sm_idx);
        sm.clkdiv().write(|w| {
            w.set_int(1);
            w.set_frac(0);
        });
        sm.shiftctrl().write(|_| {});
        sm.execctrl().write(|_| {});
        sm.pinctrl().write(|_| {});
    }
}

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

fn run_dma_test(ch_id: u8, config: TestConfig) {
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

    defmt::info!("*** Running DMA test {}, channel {}", test_name, ch_id);

    let tx_count = match word_size {
        lax_dma::TxSize::_8bit => dst.len() as u32,
        lax_dma::TxSize::_16bit => dst.len() as u32 / core::mem::size_of::<u16>() as u32,
        lax_dma::TxSize::_32bit => dst.len() as u32 / core::mem::size_of::<u32>() as u32,
    };

    let dma_config = Config {
        high_priority: false,
        word_size,
        src_addr: src.as_ptr() as u32,
        src_incr: increment_src,
        dest_addr: dst.as_mut_ptr() as u32,
        dest_incr: increment_dst,
        tx_count,
        treq_sel: TreqSel::PERMANENT,
        byte_swap,
        start: false,
    };

    let dma = LaxDmaWrite::new(ch_id, dma_config);

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
        run_dma_test(5, test);
    }
}

// ── PIO pipeline tests ────────────────────────────────────────────────────

fn test_with_pio_invert_twice() {
    const SIZE: usize = 32;
    let input_buffer = [0x55u8; SIZE];
    let mut output_buffer = [0u8; SIZE];
    let input_buffer_addr = [input_buffer.as_ptr() as u32];

    reset_pio0();

    let pio0 = pac::PIO0;

    // Load programs into instruction memory
    let invert_pio = pio_programs::invert_pio_program();
    let invert_pio_again = pio_programs::invert_pio_again_program();
    let prog0_len = load_pio_program(&invert_pio, 0);
    load_pio_program(&invert_pio_again, prog0_len);

    // Configure SM0: invert program (no autopull, no autopush)
    let sm0 = pio0.sm(0);
    sm0.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm0.shiftctrl().write(|w| {
        w.set_autopull(false);
        w.set_autopush(false);
    });
    sm0.execctrl().write(|w| {
        w.set_wrap_top(invert_pio.wrap.source);
        w.set_wrap_bottom(invert_pio.wrap.target);
    });
    // JMP to program start
    sm0.instr().write(|w| w.set_instr(0x0000));

    // Configure SM1: invert_again program (no autopull, no autopush)
    let sm1 = pio0.sm(1);
    sm1.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm1.shiftctrl().write(|w| {
        w.set_autopull(false);
        w.set_autopush(false);
    });
    sm1.execctrl().write(|w| {
        w.set_wrap_top(prog0_len as u8 + invert_pio_again.wrap.source);
        w.set_wrap_bottom(prog0_len as u8 + invert_pio_again.wrap.target);
    });
    // JMP to program start
    sm1.instr().write(|w| w.set_instr(prog0_len as u16));

    // Enable both SMs
    pio0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() | 0b11);
    });

    // FIFO addresses
    let txf0_addr = pio0.txf(0).as_ptr() as u32;
    let rxf0_addr = pio0.rxf(0).as_ptr() as u32;
    let txf1_addr = pio0.txf(1).as_ptr() as u32;
    let rxf1_addr = pio0.rxf(1).as_ptr() as u32;

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    // DMA3: SM1 RX → output buffer
    let dma3 = LaxDmaWrite::new(3, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: rxf1_addr,
        src_incr: false,
        dest_addr: output_buffer.as_mut_ptr() as u32,
        dest_incr: true,
        tx_count: SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_RX1,
        byte_swap: false,
        start: true,
    });

    // DMA2: SM0 RX → SM1 TX
    let dma2 = LaxDmaWrite::new(2, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: rxf0_addr,
        src_incr: false,
        dest_addr: txf1_addr,
        dest_incr: false,
        tx_count: SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_TX1,
        byte_swap: false,
        start: true,
    });

    // DMA1: input buffer → SM0 TX (triggered by DMA0)
    let dma1 = LaxDmaWrite::new(1, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: 0,
        src_incr: true,
        dest_addr: txf0_addr,
        dest_incr: false,
        tx_count: SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_TX0,
        byte_swap: false,
        start: false,
    });

    // DMA0: write input_buffer address → DMA1 read_addr_trig (triggers DMA1)
    let dma0 = LaxDmaWrite::new(0, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: input_buffer_addr.as_ptr() as u32,
        src_incr: false,
        dest_addr: dma1.read_addr_trig_register_addr(),
        dest_incr: false,
        tx_count: 1,
        treq_sel: TreqSel::PERMANENT,
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

    reset_pio0();

    let pio0 = pac::PIO0;
    let expand_pio = pio_programs::gen_monochrome_expand_program(MonochromeColor::Bpp1);
    load_pio_program(&expand_pio, 0);

    // Configure SM0: autopull, autopush, push_thresh=12
    let sm0 = pio0.sm(0);
    sm0.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm0.shiftctrl().write(|w| {
        w.set_autopull(true);
        w.set_autopush(true);
        w.set_push_thresh(12);
        w.set_pull_thresh(0); // 0 = 32 bits
        w.set_in_shiftdir(false); // left
        w.set_out_shiftdir(true); // right
    });
    sm0.execctrl().write(|w| {
        w.set_wrap_top(expand_pio.wrap.source);
        w.set_wrap_bottom(expand_pio.wrap.target);
    });
    sm0.instr().write(|w| w.set_instr(0x0000));

    // Enable SM0
    pio0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() | 0b01);
    });

    let txf_addr = pio0.txf(0).as_ptr() as u32;
    let rxf_addr = pio0.rxf(0).as_ptr() as u32;

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    // DMA1: input buffer → SM0 TX
    let dma1 = LaxDmaWrite::new(1, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: input_buffer.as_ptr() as u32,
        src_incr: true,
        dest_addr: txf_addr,
        dest_incr: false,
        tx_count: SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_TX0,
        byte_swap: false,
        start: false,
    });

    // DMA2: SM0 RX → output buffer
    let dma2 = LaxDmaWrite::new(2, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: rxf_addr,
        src_incr: false,
        dest_addr: output_buffer.as_mut_ptr() as u32,
        dest_incr: true,
        tx_count: 12 * SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_RX0,
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

    reset_pio0();

    let pio0 = pac::PIO0;
    let expand_pio = pio_programs::gen_monochrome_expand_program(color);
    load_pio_program(&expand_pio, 0);

    // Configure SM0: autopull, autopush, push_thresh=12
    let sm0 = pio0.sm(0);
    sm0.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm0.shiftctrl().write(|w| {
        w.set_autopull(true);
        w.set_autopush(true);
        w.set_push_thresh(12);
        w.set_pull_thresh(0); // 0 = 32 bits
        w.set_in_shiftdir(false); // left
        w.set_out_shiftdir(true); // right
    });
    sm0.execctrl().write(|w| {
        w.set_wrap_top(expand_pio.wrap.source);
        w.set_wrap_bottom(expand_pio.wrap.target);
    });
    sm0.instr().write(|w| w.set_instr(0x0000));

    // Enable SM0
    pio0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() | 0b01);
    });

    let txf_addr = pio0.txf(0).as_ptr() as u32;
    let rxf_addr = pio0.rxf(0).as_ptr() as u32;

    defmt::info!("input_buffer: {:08b}", input_buffer);
    defmt::info!("output_buffer: {:08b}", output_buffer);

    // DMA1: input buffer → SM0 TX
    let dma1 = LaxDmaWrite::new(1, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: input_buffer.as_ptr() as u32,
        src_incr: true,
        dest_addr: txf_addr,
        dest_incr: false,
        tx_count: SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_TX0,
        byte_swap: false,
        start: false,
    });

    // DMA2: SM0 RX → output buffer
    let dma2 = LaxDmaWrite::new(2, Config {
        high_priority: false,
        word_size: TxSize::_32bit,
        src_addr: rxf_addr,
        src_incr: false,
        dest_addr: output_buffer.as_mut_ptr() as u32,
        dest_incr: true,
        tx_count: bpp as u32 * SIZE as u32 / 4,
        treq_sel: TreqSel::PIO0_RX0,
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

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = embassy_rp::init(Default::default());

    defmt::info!("DMA test suite starting");

    defmt::info!("=== Memory-to-memory DMA tests ===");
    run_mem_to_mem_tests();

    defmt::info!("=== PIO invert twice test ===");
    test_with_pio_invert_twice();

    defmt::info!("=== PIO expand 12× test ===");
    test_with_pio_expand_12times();

    defmt::info!("=== PIO expand dynamic (Bpp1) test ===");
    test_with_pio_expand_dynamic(MonochromeColor::Bpp1);

    defmt::info!("=== All tests done ===");

    loop {
        cortex_m::asm::wfi();
    }
}
