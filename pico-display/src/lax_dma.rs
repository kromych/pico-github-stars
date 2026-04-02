//! Very unsafe DMA driver for experimental purposes.

use embassy_rp::pac;

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
pub struct Config {
    pub word_size: TxSize,
    pub src_addr: u32,
    pub src_incr: bool,
    pub dest_addr: u32,
    pub dest_incr: bool,
    pub tx_count: u32,
    pub tx_req: TxReq,
    pub byte_swap: bool,
    pub high_priority: bool,
    pub start: bool,
}

pub struct LaxDmaWrite {
    ch_id: u8,
    ch_id_chain: u8,
}

impl LaxDmaWrite {
    fn ch(&self) -> pac::dma::Channel {
        pac::DMA.ch(self.ch_id as usize)
    }

    /// Create a new DMA channel with the given configuration.
    /// NOTE: the DMA peripheral must be initialized before using this
    /// (embassy_rp::init handles this).
    pub fn new(ch_id: u8, config: Config) -> Self {
        LaxDmaWrite::new_chained(ch_id, ch_id, config)
    }

    pub fn new_chained(ch_id: u8, ch_id_chain: u8, config: Config) -> Self {
        let ch = pac::DMA.ch(ch_id as usize);

        cortex_m::asm::dsb();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        ch.read_addr().write_value(config.src_addr);
        ch.write_addr().write_value(config.dest_addr);
        ch.trans_count().write_value(config.tx_count);

        // Build the control register value using the typed CtrlTrig helper,
        // then write it via al1_ctrl (non-triggering alias at offset 0x10).
        // Writing ctrl_trig (offset 0x0C) would auto-start the channel.
        let mut ctrl = pac::dma::regs::CtrlTrig(0);
        ctrl.set_data_size(match config.word_size {
            TxSize::_8bit => pac::dma::vals::DataSize::SIZE_BYTE,
            TxSize::_16bit => pac::dma::vals::DataSize::SIZE_HALFWORD,
            TxSize::_32bit => pac::dma::vals::DataSize::SIZE_WORD,
        });
        ctrl.set_incr_read(config.src_incr);
        ctrl.set_incr_write(config.dest_incr);
        ctrl.set_treq_sel(pac::dma::vals::TreqSel::from_bits(config.tx_req as u8));
        ctrl.set_bswap(config.byte_swap);
        ctrl.set_chain_to(ch_id_chain);
        ctrl.set_high_priority(config.high_priority);
        ctrl.set_en(true);
        ch.al1_ctrl().write_value(ctrl.0);

        if config.start {
            // Write dest address via trigger alias to start the transfer.
            ch.al2_write_addr_trig().write_value(config.dest_addr);
        }

        Self { ch_id, ch_id_chain }
    }

    pub fn trigger(&self) {
        let channel_flags = (1u16 << self.ch_id) | (1u16 << self.ch_id_chain);
        pac::DMA.multi_chan_trigger().write(|w| {
            w.set_multi_chan_trigger(channel_flags);
        });
    }

    pub fn is_done(&self) -> bool {
        !self.ch().ctrl_trig().read().busy()
    }

    pub fn wait(&self) {
        while !self.is_done() {}

        cortex_m::asm::dsb();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    pub fn read_error(&self) -> bool {
        self.ch().ctrl_trig().read().read_error()
    }

    pub fn last_read_addr(&self) -> u32 {
        self.ch().read_addr().read()
    }

    pub fn write_error(&self) -> bool {
        self.ch().ctrl_trig().read().write_error()
    }

    pub fn last_write_addr(&self) -> u32 {
        self.ch().write_addr().read()
    }

    pub fn tx_count_remaining(&self) -> u32 {
        self.ch().trans_count().read()
    }

    /// Set the read address and trigger the transfer.
    pub fn set_read_addr_trigger(&self, addr: u32) {
        self.ch().al3_read_addr_trig().write_value(addr);
    }

    /// Return the address of the al3_read_addr_trig register (for DMA chaining).
    pub fn read_addr_trig_register_addr(&self) -> u32 {
        self.ch().al3_read_addr_trig().as_ptr() as u32
    }

    /// Set the transfer count without triggering.
    pub fn set_transfer_count(&self, tx_count: u32) {
        self.ch().trans_count().write_value(tx_count);
    }

    /// Re-arm the channel with a new transfer count and trigger it.
    /// The read/write addresses and control register are left unchanged.
    pub fn restart(&self, tx_count: u32) {
        self.ch().trans_count().write_value(tx_count);
        self.trigger();
    }
}

impl Drop for LaxDmaWrite {
    fn drop(&mut self) {
        self.wait();
        // Disable the channel via non-triggering alias
        self.ch().al1_ctrl().write_value(0);
    }
}
