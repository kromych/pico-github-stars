//! Very unsafe DMA driver for experimental purposes.

use rp2040_hal::dma;

#[allow(dead_code)]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum TxSize {
    _8bit = 0,
    _16bit = 1,
    _32bit = 2,
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum TxReq {
    Pio0Tx0 = 0,
    Pio0Tx1 = 1,
    Pio0Tx2 = 2,
    Pio0Tx3 = 3,
    Pio0Rx0 = 4,
    Pio0Rx1 = 5,
    Pio0Rx2 = 6,
    Pio0Rx3 = 7,
    Pio1Tx0 = 8,
    Pio1Tx1 = 9,
    Pio1Tx2 = 10,
    Pio1Tx3 = 11,
    Pio1Rx0 = 12,
    Pio1Rx1 = 13,
    Pio1Rx2 = 14,
    Pio1Rx3 = 15,
    Spi0Tx = 16,
    Spi0Rx = 17,
    Spi1Tx = 18,
    Spi1Rx = 19,
    Uart0Tx = 20,
    Uart0Rx = 21,
    Uart1Tx = 22,
    Uart1Rx = 23,
    PwmWrap0 = 24,
    PwmWrap1 = 25,
    PwmWrap2 = 26,
    PwmWrap3 = 27,
    PwmWrap4 = 28,
    PwmWrap5 = 29,
    PwmWrap6 = 30,
    PwmWrap7 = 31,
    I2C0Tx = 32,
    I2C0Rx = 33,
    I2C1Tx = 34,
    I2C1Rx = 35,
    Adc = 36,
    XipStream = 37,
    XipSsitx = 38,
    XipSsirx = 39,
    Timer0 = 59,
    Timer1 = 60,
    Timer2 = 61,
    Timer3 = 62,
    Permanent = 63,
}

impl From<u8> for TxReq {
    fn from(val: u8) -> Self {
        match val {
            0 => TxReq::Pio0Tx0,
            1 => TxReq::Pio0Tx1,
            2 => TxReq::Pio0Tx2,
            3 => TxReq::Pio0Tx3,
            4 => TxReq::Pio0Rx0,
            5 => TxReq::Pio0Rx1,
            6 => TxReq::Pio0Rx2,
            7 => TxReq::Pio0Rx3,
            8 => TxReq::Pio1Tx0,
            9 => TxReq::Pio1Tx1,
            10 => TxReq::Pio1Tx2,
            11 => TxReq::Pio1Tx3,
            12 => TxReq::Pio1Rx0,
            13 => TxReq::Pio1Rx1,
            14 => TxReq::Pio1Rx2,
            15 => TxReq::Pio1Rx3,
            16 => TxReq::Spi0Tx,
            17 => TxReq::Spi0Rx,
            18 => TxReq::Spi1Tx,
            19 => TxReq::Spi1Rx,
            20 => TxReq::Uart0Tx,
            21 => TxReq::Uart0Rx,
            22 => TxReq::Uart1Tx,
            23 => TxReq::Uart1Rx,
            24 => TxReq::PwmWrap0,
            25 => TxReq::PwmWrap1,
            26 => TxReq::PwmWrap2,
            27 => TxReq::PwmWrap3,
            28 => TxReq::PwmWrap4,
            29 => TxReq::PwmWrap5,
            30 => TxReq::PwmWrap6,
            31 => TxReq::PwmWrap7,
            32 => TxReq::I2C0Tx,
            33 => TxReq::I2C0Rx,
            34 => TxReq::I2C1Tx,
            35 => TxReq::I2C1Rx,
            36 => TxReq::Adc,
            37 => TxReq::XipStream,
            38 => TxReq::XipSsitx,
            39 => TxReq::XipSsirx,
            59 => TxReq::Timer0,
            60 => TxReq::Timer1,
            61 => TxReq::Timer2,
            62 => TxReq::Timer3,
            63 => TxReq::Permanent,
            _ => panic!("Invalid TxReq value"),
        }
    }
}

#[derive(Copy, Clone)]

pub struct Source {
    pub address: *const u8,
    pub increment: bool,
}

#[derive(Copy, Clone)]

pub struct Destination {
    pub address: *mut u8,
    pub increment: bool,
}
#[derive(Copy, Clone)]

pub struct Config {
    pub word_size: TxSize,
    pub source: Source,
    pub destination: Destination,
    pub tx_count: u32,
    pub tx_req: TxReq,
    pub byte_swap: bool,
    pub start: bool,
}

pub struct LaxDmaWrite {
    ch_id: u8,
    ch_id_chain: u8,
    ch: &'static rp2040_pac::dma::ch::CH,
}

/// Create a new DMA channel with the given configuration.
/// NOTE: be sure to reset the DMA system before using this function.
/// ```ignore
/// let dma = pac.DMA.split(&mut pac.RESETS);
/// ```
impl LaxDmaWrite {
    pub fn new<CHID: dma::ChannelIndex>(config: Config) -> Self {
        LaxDmaWrite::new_chained::<CHID, CHID>(config)
    }

    pub fn new_chained<CHID: dma::ChannelIndex, CHIDCHAIN: dma::ChannelIndex>(
        config: Config,
    ) -> Self {
        let ch = unsafe { (*rp2040_pac::DMA::PTR).ch(CHID::id() as usize) };

        let (src, src_incr) = (config.source.address, config.source.increment);
        let (dest, dest_incr) = (config.destination.address, config.destination.increment);

        cortex_m::asm::dsb();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        ch.ch_al1_ctrl().reset();
        ch.ch_al1_ctrl().write(|w| unsafe {
            w.data_size().bits(config.word_size as u8);
            w.incr_read().bit(src_incr);
            w.incr_write().bit(dest_incr);
            w.treq_sel().bits(config.tx_req as u8);
            w.bswap().bit(config.byte_swap);
            w.chain_to().bits(CHIDCHAIN::id());
            w.en().bit(true);
            w
        });
        ch.ch_read_addr().write(|w| unsafe { w.bits(src as u32) });
        ch.ch_trans_count()
            .write(|w| unsafe { w.bits(config.tx_count) });
        if config.start {
            ch.ch_al2_write_addr_trig()
                .write(|w| unsafe { w.bits(dest as u32) });
        } else {
            ch.ch_write_addr().write(|w| unsafe { w.bits(dest as u32) });
        }

        Self {
            ch_id: CHID::id(),
            ch_id_chain: CHIDCHAIN::id(),
            ch,
        }
    }

    pub fn trigger(&self) {
        let channel_flags = 1 << self.ch_id | 1 << self.ch_id_chain;
        unsafe { &*rp2040_pac::DMA::ptr() }
            .multi_chan_trigger()
            .write(|w| unsafe { w.bits(channel_flags) });
    }

    pub fn is_done(&self) -> bool {
        !self.ch.ch_al1_ctrl().read().busy().bit_is_set()
    }

    pub fn wait(&self) {
        while !self.is_done() {}

        cortex_m::asm::dsb();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    pub fn read_error(&self) -> bool {
        self.ch.ch_al1_ctrl().read().read_error().bit_is_set()
    }

    pub fn last_read_addr(&self) -> u32 {
        self.ch.ch_read_addr().read().bits()
    }

    pub fn write_error(&self) -> bool {
        self.ch.ch_al1_ctrl().read().write_error().bit_is_set()
    }

    pub fn last_write_addr(&self) -> u32 {
        self.ch.ch_write_addr().read().bits()
    }

    pub fn tx_count_remaining(&self) -> u32 {
        self.ch.ch_trans_count().read().bits()
    }

    pub fn read_trig_addr(&self) -> *const u8 {
        self.ch.ch_al3_read_addr_trig().as_ptr() as *const u8
    }
}

impl Drop for LaxDmaWrite {
    fn drop(&mut self) {
        self.wait();
        self.ch.ch_al1_ctrl().reset();
    }
}

#[allow(dead_code)]
pub mod tests {
    use crate::lax_dma;
    use crate::lax_dma::Config;
    use crate::lax_dma::Destination;
    use crate::lax_dma::LaxDmaWrite;
    use crate::lax_dma::Source;
    use crate::lax_dma::TxReq;
    use crate::lax_dma::TxSize;
    use crate::pico_display_pimoroni::MonochromeColor;
    use rp2040_hal::dma;
    use rp2040_hal::dma::DMAExt;
    use rp2040_hal::pio::PIOExt;

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

        // Calculate the transaction count based on the word size
        let tx_count = match word_size {
            lax_dma::TxSize::_8bit => dst.len() as u32,
            lax_dma::TxSize::_16bit => dst.len() as u32 / core::mem::size_of::<u16>() as u32,
            lax_dma::TxSize::_32bit => dst.len() as u32 / core::mem::size_of::<u32>() as u32,
        };

        // Configure the DMA transfer
        let dma_config = lax_dma::Config {
            word_size,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: increment_src,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: increment_dst,
            },
            tx_count,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap,
            start: false,
        };

        let dma = lax_dma::LaxDmaWrite::new::<CHID>(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr() as usize);
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr() as usize);
        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);

        // Start the DMA transfer
        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        // Log final state
        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);

        // Check for DMA errors
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr() as usize);
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr() as usize);
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        // Validate the result
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

    pub fn run_dma_tests() {
        // Define the test configurations
        let tests = [
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_8bit,
                byte_swap: false,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_8bit",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_16bit,
                byte_swap: false,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_16bit",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_32bit,
                byte_swap: false,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_32bit",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_8bit,
                byte_swap: true,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_8bit_byte_swap",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [43, 42, 45, 44],
                word_size: lax_dma::TxSize::_16bit,
                byte_swap: true,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_16bit_byte_swap",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [45, 44, 43, 42],
                word_size: lax_dma::TxSize::_32bit,
                byte_swap: true,
                increment_src: true,
                increment_dst: true,
                test_name: "dma_test_32bit_byte_swap",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 42, 42, 42],
                word_size: lax_dma::TxSize::_8bit,
                byte_swap: false,
                increment_src: false,
                increment_dst: true,
                test_name: "dma_test_8bit_fill",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 42, 43],
                word_size: lax_dma::TxSize::_16bit,
                byte_swap: false,
                increment_src: false,
                increment_dst: true,
                test_name: "dma_test_16bit_fill",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_32bit,
                byte_swap: false,
                increment_src: false,
                increment_dst: true,
                test_name: "dma_test_32bit_fill",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [45, 0, 0, 0],
                word_size: lax_dma::TxSize::_8bit,
                byte_swap: false,
                increment_src: true,
                increment_dst: false,
                test_name: "dma_test_8bit_dst_fixed",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [44, 45, 0, 0],
                word_size: lax_dma::TxSize::_16bit,
                byte_swap: false,
                increment_src: true,
                increment_dst: false,
                test_name: "dma_test_16bit_dst_fixed",
            },
            TestConfig {
                src: cortex_m::singleton!(: [u8; 4] = [42, 43, 44, 45]).unwrap(),
                dst: cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap(),
                expected: [42, 43, 44, 45],
                word_size: lax_dma::TxSize::_32bit,
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

    pub fn test_with_pio_invert_twice() {
        // | DMA Channel | Source (Read Address)      | Destination (Write Address) | FIFO Connection           | Shift Register              |
        // |-------------|----------------------------|-----------------------------|---------------------------|-----------------------------|
        // | DMA 1 (TX)  | RAM Buffer                 | PIO TX FIFO (PIO0_TXF_SM0)  | TX FIFO feeds OSR         | OSR (Output Shift Register) |
        // | DMA 2 (RX)  | PIO RX FIFO (PIO0_RXF_SM0) | RAM Buffer                  | RX FIFO receives from ISR | ISR (Input Shift Register)  |

        const SIZE: usize = 32;
        let input_buffer = [0x55u8; SIZE];
        let mut output_buffer = [0u8; SIZE];
        let input_buffer_addr = [input_buffer.as_ptr() as u32];

        let mut pac = rp2040_pac::Peripherals::take().unwrap();
        let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

        let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
            crate::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let invert_pio = pio_proc::pio_asm!(
            "more:",
            "       pull", // PIO TX FIFO -> OSR (no need if `autopull` is true)
            "       mov     x, osr", // OSR -> x (same as `out x, 32` for shifting 32 bits from OSR)
            "       mov     x, ~x", // ~x -> x (bitwise invert)
            "       mov     isr, x", // x -> ISR (same as `in x, 32` as shifting 32 bits into ISR)
            "       push", // ISR -> PIO TX FIFO (no need if `autopush` is true)
            "       irq     wait 4",
            "       jmp     !osre, more",
        );

        let invert_pio_again = pio_proc::pio_asm!(
            "more:",
            "       wait    1 irq 4",
            "       pull", // PIO TX FIFO -> OSR (no need if `autopull` is true)
            "       mov     x, osr", // OSR -> x (same as `out x, 32` for shifting 32 bits from OSR)
            "       mov     x, ~x", // ~x -> x (bitwise invert)
            "       mov     isr, x", // x -> ISR (same as `in x, 32` as shifting 32 bits into ISR)
            "       push", // ISR -> PIO TX FIFO (no need if `autopush` is true)
            "       jmp     !osre, more",
        );

        // Reset DMA
        let _dma = pac.DMA.split(&mut pac.RESETS);
        // Reset PIO
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let (sm0, rx0, tx0) = rp2040_hal::pio::PIOBuilder::from_installed_program(
            pio.install(&invert_pio.program).unwrap(),
        )
        .autopull(false)
        .autopush(false)
        .build(sm0);
        sm0.start();

        let (sm1, rx1, tx1) = rp2040_hal::pio::PIOBuilder::from_installed_program(
            pio.install(&invert_pio_again.program).unwrap(),
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

        // This DMA channel transfers data from the PIO state machine's
        // RX FIFO to the output buffer. It will be stalled until the
        // next DMA channel is started and feeds the PIO TX FIFO.
        let dma3 = LaxDmaWrite::new::<dma::CH3>(Config {
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

        // This DMA channel transfers data from the PIO state machine's
        // RX FIFO to the output buffer. It will be stalled until the
        // next DMA channel is started and feeds the PIO TX FIFO.
        let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
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

        // This DMA channel transfers data from the input buffer to the PIO state machine's TX FIFO.
        // If this one is chained to dma0 (that writes to this channel's read trigger address),
        // the two will be res-starting together, running in the ping-pong mode.
        let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
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

        // This DMA channel is used to configure the next one by writing to
        // the channel read address trigger register.
        let dma0 = LaxDmaWrite::new::<dma::CH0>(Config {
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

        // Start the DMA transfers
        dma0.trigger();

        // Wait for the DMA transfers to complete
        dma2.wait();
        dma3.wait();

        defmt::info!("input_buffer: {:08b}", input_buffer);
        defmt::info!("output_buffer: {:08b}", output_buffer);
    }

    pub fn test_with_pio_expand_12times() {
        // | DMA Channel | Source (Read Address)      | Destination (Write Address) | FIFO Connection           | Shift Register              |
        // |-------------|----------------------------|-----------------------------|---------------------------|-----------------------------|
        // | DMA 1 (TX)  | RAM Buffer                 | PIO TX FIFO (PIO0_TXF_SM0)  | TX FIFO feeds OSR         | OSR (Output Shift Register) |
        // | DMA 2 (RX)  | PIO RX FIFO (PIO0_RXF_SM0) | RAM Buffer                  | RX FIFO receives from ISR | ISR (Input Shift Register)  |

        const SIZE: usize = 4;
        let input_buffer = [0x5au8; SIZE];
        let mut output_buffer = [0u8; 12 * SIZE]; // bpp = 1; 12 /bpp

        let mut pac = rp2040_pac::Peripherals::take().unwrap();
        let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

        let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
            crate::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // bpp = 1, greyscale (effectively BW) so R == G == B, each
        // repeating 12 times within RGB444.
        // If a pixel == 1, produce twelve 1's,
        // if a pixel == 0, produce twelve 0's.
        let expand_times12_pio = pio_proc::pio_asm!(
            ".wrap_target",
            "           out     x, 1",  // bpp
            "           set     y, 11", // 12/bpp - 1
            "repeat:",
            "           in      x, 1", // bpp
            "           jmp     y--, repeat",
            ".wrap"
        );

        // Reset DMA
        let _dma = pac.DMA.split(&mut pac.RESETS);
        // Reset PIO
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let installed_pio = pio.install(&expand_times12_pio.program).unwrap();
        let (sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed_pio)
            .autopull(true)
            .autopush(true)
            .build(sm0);
        sm.start();

        let txf = tx.fifo_address();
        let rxf = rx.fifo_address();

        defmt::info!("input_buffer: {:08b}", input_buffer);
        defmt::info!("output_buffer: {:08b}", output_buffer);

        // This DMA channel transfers data from the input buffer to the PIO state machine's TX FIFO.
        let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
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

        // This DMA channel transfers data from the PIO state machine's
        // RX FIFO to the output buffer
        let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
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

        // Start the DMA transfers
        dma1.trigger();
        dma2.trigger();

        // Wait for the DMA transfers to complete
        dma1.wait();
        dma2.wait();

        defmt::info!("input_buffer: {:08b}", input_buffer);
        defmt::info!("output_buffer: {:08b}", output_buffer);
    }

    /// Generates a PIO program to produce greyscale color encoded as RGB444
    /// physically. Each pixel may have 2, 4, or 16 greyscale levels (1, 2, or 4 bpp).
    fn greyscale_pio(color: MonochromeColor) -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
        let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new();

        const RGB_BPP: u8 = 12;
        let bpp = color as u8;

        let mut repeat = a.label();

        // Pull `bpp` bits (1, 2, or 4) from the TX FIFO into OSR
        a.out(pio::OutDestination::X, bpp);

        // Loop counter in `Y` to repeat `bpp` as many times as need
        // to fill RGB444 for the greyscale color.
        a.set(pio::SetDestination::Y, RGB_BPP / bpp - 1);
        a.bind(&mut repeat);
        // Push the bits into ISR which goes into RX FIFO.
        a.r#in(pio::InSource::X, bpp);
        // Repeat
        a.jmp(pio::JmpCondition::YDecNonZero, &mut repeat);

        a.assemble_program()
    }

    pub fn test_with_pio_expand_dynamic(color: MonochromeColor) {
        // | DMA Channel | Source (Read Address)      | Destination (Write Address) | FIFO Connection           | Shift Register              |
        // |-------------|----------------------------|-----------------------------|---------------------------|-----------------------------|
        // | DMA 1 (TX)  | RAM Buffer                 | PIO TX FIFO (PIO0_TXF_SM0)  | TX FIFO feeds OSR         | OSR (Output Shift Register) |
        // | DMA 2 (RX)  | PIO RX FIFO (PIO0_RXF_SM0) | RAM Buffer                  | RX FIFO receives from ISR | ISR (Input Shift Register)  |

        const RGB_BPP: u8 = 12;
        let bpp = RGB_BPP / color as u8;

        const SIZE: usize = 4;
        let input_buffer: [u8; SIZE] = [0xaa; SIZE];
        let mut output_buffer: [u8; 12 * SIZE] = [0u8; 12 * SIZE]; // Max output size, each input bit repeated 12 times (greyscale RGB444)

        let mut pac = rp2040_pac::Peripherals::take().unwrap();
        let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

        let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
            crate::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Reset DMA
        let _dma = pac.DMA.split(&mut pac.RESETS);
        // Reset PIO
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let greyscale_pio = greyscale_pio(color);
        let installed_pio = pio.install(&greyscale_pio).unwrap();
        let (sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed_pio)
            .autopull(true)
            .autopush(true)
            .build(sm0);
        sm.start();

        let txf = tx.fifo_address();
        let rxf = rx.fifo_address();

        defmt::info!("input_buffer: {:08b}", input_buffer);
        defmt::info!("output_buffer: {:08b}", output_buffer);

        // This DMA channel transfers data from the input buffer to the PIO state machine's TX FIFO.
        let dma1 = LaxDmaWrite::new::<dma::CH1>(Config {
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

        // This DMA channel transfers data from the PIO state machine's
        // RX FIFO to the output buffer.
        let dma2 = LaxDmaWrite::new::<dma::CH2>(Config {
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

        // Start the DMA transfers
        dma1.trigger();
        dma2.trigger();

        // Wait for the DMA transfers to complete
        dma1.wait();
        dma2.wait();

        defmt::info!("input_buffer: {:08b}", input_buffer);
        defmt::info!("output_buffer: {:08b}", output_buffer);
    }
}
