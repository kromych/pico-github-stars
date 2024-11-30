//! Very unsafe DMA driver for experimental purposes.

use core::marker::PhantomData;
use rp2040_hal::dma;

#[allow(dead_code)]
#[repr(u8)]
pub enum TxSize {
    _8Bit = 0,
    _16bit = 1,
    _32bit = 2,
}

#[allow(dead_code)]
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

pub struct Source {
    pub address: *const u8,
    pub increment: bool,
}

pub struct Destination {
    pub address: *mut u8,
    pub increment: bool,
}

pub struct Config {
    pub word_size: TxSize,
    pub source: Source,
    pub destination: Destination,
    pub tx_count: u32,
    pub tx_req: TxReq,
    pub byte_swap: bool,
    pub start: bool,
}

pub struct LaxDmaWrite<CHID: dma::ChannelIndex, CHIDCHAIN: dma::ChannelIndex = CHID> {
    _ch_id: PhantomData<CHID>,
    _ch_id_chain: PhantomData<CHIDCHAIN>,
    ch: &'static rp2040_pac::dma::ch::CH,
}

impl<CHID: dma::ChannelIndex, CHIDCHAIN: dma::ChannelIndex> LaxDmaWrite<CHID, CHIDCHAIN> {
    pub fn new(config: Config) -> Self {
        let ch = unsafe { (*rp2040_pac::DMA::PTR).ch(CHID::id() as usize) };

        let (src, src_incr) = (config.source.address, config.source.increment);
        let (dest, dest_incr) = (config.destination.address, config.destination.increment);

        cortex_m::asm::dsb();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

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
            _ch_id: PhantomData,
            _ch_id_chain: PhantomData,
            ch,
        }
    }

    pub fn trigger(&self) {
        let channel_flags = 1 << CHID::id() | 1 << CHIDCHAIN::id();
        unsafe { &*rp2040_pac::DMA::ptr() }
            .multi_chan_trigger()
            .write(|w| unsafe { w.bits(channel_flags) });
    }

    pub fn is_done(&self) -> bool {
        !self.ch.ch_ctrl_trig().read().busy().bit_is_set()
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
}

pub mod tests {
    use crate::lax_dma;
    use rp2040_hal::dma;

    macro_rules! function {
        () => {{
            fn f() {}
            fn type_name_of<T>(_: T) -> &'static str {
                core::any::type_name::<T>()
            }
            let name = type_name_of(f);
            name.strip_suffix("::f").unwrap()
        }};
    }

    fn dma_test_8bit(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_8Bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_16bit(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_16bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u16>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_32bit(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_32bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u32>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_8bit_byte_swap(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_8Bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: true,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_16bit_byte_swap(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_16bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u16>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: true,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_32bit_byte_swap(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_32bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u32>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: true,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_8bit_fill(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_8Bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: false,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_16bit_fill(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_16bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: false,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u16>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    fn dma_test_32bit_fill(src_bytes: [u8; 4], expected: [u8; 4]) {
        defmt::info!("*** Running DMA test {}", function!());

        let src = cortex_m::singleton!(: [u8; 4] = src_bytes).unwrap();
        let dst = cortex_m::singleton!(: [u8; 4] = [0, 0, 0, 0]).unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_32bit,
            source: lax_dma::Source {
                address: src.as_ptr(),
                increment: false,
            },
            destination: lax_dma::Destination {
                address: dst.as_mut_ptr(),
                increment: true,
            },
            tx_count: dst.len() as u32 / core::mem::size_of::<u32>() as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: false,
        };

        let dma: lax_dma::LaxDmaWrite<dma::CH5> = lax_dma::LaxDmaWrite::new(dma_config);

        defmt::debug!("DMA source addr: {:x}", src.as_ptr());
        defmt::debug!("DMA dest addr: {:x}", dst.as_ptr());

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        defmt::debug!("Starting DMA");
        dma.trigger();
        dma.wait();
        defmt::debug!("DMA done");

        defmt::debug!("src: {:?}", src);
        defmt::debug!("dst: {:?}", dst);
        defmt::debug!("DMA read error: {:?}", dma.read_error());
        defmt::debug!("DMA write error: {:?}", dma.write_error());
        defmt::debug!("DMA last read addr: {:x}", dma.last_read_addr());
        defmt::debug!("DMA last write addr: {:x}", dma.last_write_addr());
        defmt::debug!("DMA tx count remaining: {:?}", dma.tx_count_remaining());

        if dst != &expected {
            defmt::error!(
                "!!! {} failed! Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        } else {
            defmt::info!(
                "*** {} passed. Expected: {:?}, got: {:?}",
                function!(),
                expected,
                dst
            );
        }
    }

    pub fn run_dma_tests() {
        dma_test_8bit([42, 43, 44, 45], [42, 43, 44, 45]);
        dma_test_16bit([42, 43, 44, 45], [42, 43, 44, 45]);
        dma_test_32bit([42, 43, 44, 45], [42, 43, 44, 45]);
        dma_test_8bit_byte_swap([42, 43, 44, 45], [42, 43, 44, 45]);
        dma_test_16bit_byte_swap([42, 43, 44, 45], [43, 42, 45, 44]);
        dma_test_32bit_byte_swap([42, 43, 44, 45], [45, 44, 43, 42]);
        dma_test_8bit_fill([42, 43, 44, 45], [42, 42, 42, 42]);
        dma_test_16bit_fill([42, 43, 44, 45], [42, 43, 42, 43]);
        dma_test_32bit_fill([42, 43, 44, 45], [42, 43, 44, 45]);
    }
}
