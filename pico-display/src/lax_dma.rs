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
    pub high_priority: bool,
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
            w.high_priority().bit(config.high_priority);
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

    /// Set the transfer count without triggering.
    pub fn set_transfer_count(&self, tx_count: u32) {
        self.ch
            .ch_trans_count()
            .write(|w| unsafe { w.bits(tx_count) });
    }

    /// Re-arm the channel with a new transfer count and trigger it.
    /// The read/write addresses and control register are left unchanged.
    pub fn restart(&self, tx_count: u32) {
        self.ch
            .ch_trans_count()
            .write(|w| unsafe { w.bits(tx_count) });
        self.trigger();
    }
}

impl Drop for LaxDmaWrite {
    fn drop(&mut self) {
        self.wait();
        self.ch.ch_al1_ctrl().reset();
    }
}
