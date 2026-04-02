//! Very unsafe DMA driver for experimental purposes.

use embassy_rp::pac;

pub use pac::dma::vals::TreqSel;

#[allow(dead_code)]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum TxSize {
    _8bit = 0,
    _16bit = 1,
    _32bit = 2,
}

#[derive(Copy, Clone)]
pub struct Config {
    pub word_size: TxSize,
    pub src_addr: u32,
    pub src_incr: bool,
    pub dest_addr: u32,
    pub dest_incr: bool,
    pub tx_count: u32,
    pub treq_sel: TreqSel,
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
        ctrl.set_treq_sel(config.treq_sel);
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
