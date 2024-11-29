#![no_std]
#![no_main]

use core::convert::TryInto;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};
use embedded_hal::digital::{InputPin, OutputPin};
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::gpio::Pins;
use rp2040_hal::gpio::{
    FunctionSioInput, FunctionSioOutput, FunctionSpi, Pin, PinId, PullDown, PullNone,
};
use rp2040_hal::pac;
use rp2040_hal::spi::Spi;
use rp2040_hal::{self, Clock};

use defmt_rtt as _;
use embedded_hal::spi::SpiBus;
use panic_probe as _;
use rp2040_pac::RESETS;

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

mod dma {
    use rp2040_pac::dma::ch::ch_ctrl_trig::CH_CTRL_TRIG_SPEC as CtrlReg;
    use rp2040_pac::dma::ch::ch_ctrl_trig::W as CtrlWriter;
    use rp2040_pac::dma::CH;
    use rp2040_pac::generic::W;

    pub const CHANNEL_FRAMEBUFFER: usize = 0;
    pub const CHANNEL_TILE0: usize = 1;
    pub const CHANNEL_TILE1: usize = 2;

    pub struct DmaChannel {
        pub channel: usize,
        pub ch: &'static CH,
    }

    impl DmaChannel {
        pub unsafe fn new(channel: usize) -> Self {
            DmaChannel {
                channel,
                ch: &(*rp2040_pac::DMA::PTR).ch(channel),
            }
        }

        pub unsafe fn set_src(&mut self, src: u32) {
            self.ch.ch_read_addr().write(|w| w.bits(src));
        }

        pub unsafe fn set_dst(&mut self, dst: u32) {
            self.ch.ch_write_addr().write(|w| w.bits(dst));
        }

        pub unsafe fn set_count(&mut self, count: u32) {
            self.ch.ch_trans_count().write(|w| w.bits(count));
        }

        pub unsafe fn set_ctrl_and_trigger<F>(&mut self, f: F)
        where
            F: FnOnce(&mut CtrlWriter) -> &mut W<CtrlReg>,
        {
            self.ch.ch_ctrl_trig().write(f);
        }

        pub fn wait(&self) {
            while self.ch.ch_trans_count().read().bits() > 0 {}
        }

        pub fn get_src(&self) -> u32 {
            self.ch.ch_read_addr().read().bits()
        }

        pub fn get_dst(&self) -> u32 {
            self.ch.ch_write_addr().read().bits()
        }

        pub fn get_count(&self) -> u32 {
            self.ch.ch_trans_count().read().bits()
        }
    }

    fn wordsize(elem_size: u32) -> u32 {
        match elem_size {
            1 => 0,
            2 => 1,
            4 => 2,
            _ => panic!("invalid DMA element size"),
        }
    }

    pub unsafe fn start_set_mem(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        let channel = dma_channel.channel;
        dma_channel.set_src(src);
        dma_channel.set_dst(dst);
        dma_channel.set_count(count);
        dma_channel.set_ctrl_and_trigger(|w| {
            w.treq_sel().permanent();
            w.chain_to().bits(channel as u8);
            w.incr_write().set_bit();
            w.data_size().bits(wordsize(elem_size) as u8);
            w.en().set_bit();
            w
        });
    }

    pub unsafe fn set_mem(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        start_set_mem(dma_channel, src, dst, elem_size, count);
        dma_channel.wait();
    }

    pub unsafe fn start_copy_mem(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        let channel = dma_channel.channel;
        dma_channel.set_src(src);
        dma_channel.set_dst(dst);
        dma_channel.set_count(count);
        dma_channel.set_ctrl_and_trigger(|w| {
            w.treq_sel().permanent();
            w.chain_to().bits(channel as u8);
            w.incr_write().set_bit();
            w.incr_read().set_bit();
            w.data_size().bits(wordsize(elem_size) as u8);
            w.en().set_bit();
            w
        });
    }

    pub unsafe fn copy_mem(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        start_copy_mem(dma_channel, src, dst, elem_size, count);
        dma_channel.wait();
    }

    pub unsafe fn start_copy_mem_bswap(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        let channel = dma_channel.channel;
        dma_channel.set_src(src);
        dma_channel.set_dst(dst);
        dma_channel.set_count(count);
        dma_channel.set_ctrl_and_trigger(|w| {
            w.bswap().set_bit();
            w.treq_sel().permanent();
            w.chain_to().bits(channel as u8);
            w.incr_write().set_bit();
            w.incr_read().set_bit();
            w.data_size().bits(wordsize(elem_size) as u8);
            w.en().set_bit();
            w
        });
    }

    pub unsafe fn copy_mem_bswap(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        start_copy_mem_bswap(dma_channel, src, dst, elem_size, count);
        dma_channel.wait();
    }

    pub unsafe fn copy_flash_to_mem(dma_channel: &mut DmaChannel, src: u32, dst: u32, count: u32) {
        // Flush XIP FIFO.
        let xip_ctrl = &*rp2040_pac::XIP_CTRL::PTR;
        while xip_ctrl.stat().read().fifo_empty().bit_is_clear() {
            defmt::info!("XIP FIFO not empty");
            cortex_m::asm::nop();
        }
        xip_ctrl.stream_addr().write(|w| w.bits(src));
        xip_ctrl.stream_ctr().write(|w| w.bits(count));

        let channel = dma_channel.channel;
        dma_channel.set_src(0x50400000); // XIP_AUX_BASE
        dma_channel.set_dst(dst);
        dma_channel.set_count(count);
        dma_channel.set_ctrl_and_trigger(|w| {
            w.treq_sel().bits(37); // DREQ_XIP_STREAM
            w.chain_to().bits(channel as u8);
            w.incr_write().set_bit();
            w.data_size().bits(2); // 4 bytes
            w.en().set_bit();
            w
        });
        dma_channel.wait();

        while xip_ctrl.stat().read().fifo_empty().bit_is_clear() {
            defmt::info!("XIP FIFO not empty");
            cortex_m::asm::nop();
        }
    }

    pub(crate) unsafe fn start_copy_to_spi(
        dma_channel: &mut DmaChannel,
        src: u32,
        dst: u32,
        elem_size: u32,
        count: u32,
    ) {
        defmt::info!("start_copy_to_spi");
        let channel = dma_channel.channel;
        dma_channel.set_src(src);
        dma_channel.set_dst(dst);
        dma_channel.set_count(count);
        dma_channel.set_ctrl_and_trigger(|w| {
            w.treq_sel().bits(16); // SPI0 TX
            w.chain_to().bits(channel as u8);
            w.incr_read().set_bit();
            w.data_size().bits(wordsize(elem_size) as u8);
            w.en().set_bit();
            w
        });
    }
}

pub const WIDTH: usize = 240;
pub const HEIGHT: usize = 320;

pub fn framebuffer() -> &'static mut [u16] {
    static mut FRAMEBUFFER: [u16; WIDTH * HEIGHT] = [0; WIDTH * HEIGHT]; // TODO: Use StaticCell
    unsafe { core::slice::from_raw_parts_mut(FRAMEBUFFER.as_ptr() as *mut u16, WIDTH * HEIGHT) }
}

#[allow(clippy::upper_case_acronyms, dead_code)]
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
/// ST7789 commands
enum Command {
    NOP = 0x00,
    SWRESET = 0x01,
    RDDID = 0x04,
    RDDST = 0x09,
    SLPIN = 0x10,
    SLPOUT = 0x11,
    PTLON = 0x12,
    NORON = 0x13,
    INVOFF = 0x20,
    INVON = 0x21,
    GAMSET = 0x26,
    DISPOFF = 0x28,
    DISPON = 0x29,
    CASET = 0x2A,
    RASET = 0x2B,
    RAMWR = 0x2C,
    RAMRD = 0x2E,
    PTLAR = 0x30,
    VSCRDER = 0x33,
    TEOFF = 0x34,
    TEON = 0x35,
    MADCTL = 0x36,
    VSCAD = 0x37,
    COLMOD = 0x3A,
    PORCTRL = 0xB2,
    GCTRL = 0xB7,
    VCOMS = 0xBB,
    LCMCTRL = 0xC0,
    VDVVRHEN = 0xC2,
    VRHS = 0xC3,
    VDVS = 0xC4,
    VCMOFSET = 0xC5,
    FRCTRL2 = 0xC6,
    PWMFRSEL = 0xCC,
    PWCTRL1 = 0xD0,
    _D6 = 0xD6,
    GMCTRP1 = 0xE0,
    GMCTRN1 = 0xE1,
}

pub struct Display<BL, DC, CS, VSYNC, SPIDEV, SPIPINOUT>
where
    BL: PinId,
    DC: PinId,
    CS: PinId,
    VSYNC: PinId,
    SPIDEV: rp2040_hal::spi::SpiDevice,
    SPIPINOUT: rp2040_hal::spi::ValidSpiPinout<SPIDEV>,
{
    backlight_pin: Pin<BL, FunctionSioOutput, PullDown>,
    dc_pin: Pin<DC, FunctionSioOutput, PullDown>,
    cs_pin: Pin<CS, FunctionSioOutput, PullDown>,
    vsync_pin: Pin<VSYNC, FunctionSioInput, PullNone>,
    spi_device: Spi<rp2040_hal::spi::Enabled, SPIDEV, SPIPINOUT, 8>,
    dma_channel: dma::DmaChannel,
    last_vsync_time: u32,
}

impl<
        BL: PinId,
        DC: PinId,
        CS: PinId,
        VSYNC: PinId,
        SPIDEV: rp2040_hal::spi::SpiDevice,
        SPIPINOUT: rp2040_hal::spi::ValidSpiPinout<SPIDEV>,
    > Display<BL, DC, CS, VSYNC, SPIDEV, SPIPINOUT>
{
    #[allow(clippy::too_many_arguments)]
    pub fn new<F, B>(
        backlight_pin: Pin<BL, FunctionSioOutput, PullDown>,
        dc_pin: Pin<DC, FunctionSioOutput, PullDown>,
        cs_pin: Pin<CS, FunctionSioOutput, PullDown>,
        vsync_pin: Pin<VSYNC, FunctionSioInput, PullNone>,
        spi_device: SPIDEV,
        spi_pinout: SPIPINOUT,
        delay_source: &mut cortex_m::delay::Delay,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        dma_channel: dma::DmaChannel,
    ) -> Self
    where
        F: Into<HertzU32>,
        B: Into<HertzU32>,
    {
        //let sspdr = spi_device.sspdr(); // For DMA
        let spi_device = Spi::<_, _, _, 8>::new(spi_device, spi_pinout);
        let spi_device = spi_device.init(
            resets,
            peri_frequency.into(),
            baudrate.into(),
            embedded_hal::spi::MODE_0,
        );
        let mut display = Self {
            backlight_pin,
            dc_pin,
            cs_pin,
            vsync_pin,
            spi_device,
            dma_channel,
            last_vsync_time: 0,
        };

        display.backlight_pin.set_low().unwrap();
        delay_source.delay_us(10_000);

        display.write_command(Command::SWRESET); // reset display
        delay_source.delay_us(150_000);
        display.write_command(Command::SLPOUT); // turn off sleep
        delay_source.delay_us(10_000);
        display.write_command(Command::INVOFF); // turn off invert
        display.write_command(Command::VSCRDER); // vertical scroll definition
        display.write_data(&[0u8, 0u8, 0x14u8, 0u8, 0u8, 0u8]); // 0 TSA, 320 VSA, 0 BSA
        display.write_command(Command::MADCTL); // left -> right, bottom -> top RGB
        display.write_data(&[0b0000_0000]);
        display.write_command(Command::COLMOD); // 16bit 65k colors
        display.write_data(&[0b0101_0101]);
        display.write_command(Command::INVON); // hack?
        delay_source.delay_us(10_000);
        display.write_command(Command::NORON); // turn on display
        delay_source.delay_us(10_000);
        display.write_command(Command::DISPON); // turn on display
        delay_source.delay_us(10_000);

        display.write_command(Command::TEON); // tear effect on
        display.write_data(&[0]); // Horizontal blanking
        delay_source.delay_us(10_000);

        display.backlight_pin.set_high().unwrap();
        delay_source.delay_us(10_000);

        display
    }

    #[inline(always)]
    fn write_command(&mut self, command: Command) {
        self.cs_pin.set_low().unwrap();

        self.dc_pin.set_low().unwrap();
        self.spi_device.write(&[command as u8]).unwrap(); // TODO: Handle error
        self.cs_pin.set_high().unwrap();

        // defmt::info!("Command 0x{:x}", command as u8);
    }

    #[inline(always)]
    fn write_data(&mut self, data: &[u8]) {
        self.cs_pin.set_low().unwrap();

        self.dc_pin.set_high().unwrap();
        self.spi_device.write(data).unwrap(); // TODO: Handle error
        self.cs_pin.set_high().unwrap();

        // defmt::info!("Command 0x{:x}", command as u8);
    }

    fn set_address_window(&mut self, sx: u16, sy: u16, ex: u16, ey: u16) {
        self.write_command(Command::CASET);
        self.write_data(&sx.to_be_bytes());
        self.write_data(&ex.to_be_bytes());
        self.write_command(Command::RASET);
        self.write_data(&sy.to_be_bytes());
        self.write_data(&ey.to_be_bytes());
    }

    pub fn set_pixel(&mut self, x: u16, y: u16, color: u16) {
        self.set_address_window(x, y, x, y);
        self.write_command(Command::RAMWR);
        self.write_data(&color.to_be_bytes());
    }

    pub fn set_pixels<T>(&mut self, sx: u16, sy: u16, ex: u16, ey: u16, colors: T)
    where
        T: IntoIterator<Item = u16>,
    {
        self.set_address_window(sx, sy, ex, ey);
        self.write_command(Command::RAMWR);
        for color in colors {
            self.write_data(&color.to_be_bytes());
        }
    }

    pub fn fill(&mut self, color: u16) {
        self.set_address_window(0, 0, WIDTH as u16 - 1, HEIGHT as u16 - 1);
        self.write_command(Command::RAMWR);
        for _ in 0..WIDTH * HEIGHT {
            self.write_data(&color.to_be_bytes());
        }
    }

    fn start_flush(&mut self) {
        unsafe {
            dma::start_copy_to_spi(
                &mut self.dma_channel,
                framebuffer().as_ptr() as u32,
                (*pac::SPI0::PTR).sspdr().as_ptr() as u32, // TODO: SPI0 TX FIFO, hardcoded for now
                1,
                (WIDTH * HEIGHT * 2) as u32,
            );
        }
    }

    fn wait_for_flush(&mut self) {
        self.dma_channel.wait();
    }

    pub fn flush(&mut self) {
        defmt::info!("flush");
        defmt::info!("wait for vsync");
        self.wait_for_vsync();
        defmt::info!("start flush");
        self.start_flush();
        defmt::info!("wait for flush");
        self.wait_for_flush();
        defmt::info!("flush done");
    }

    pub fn draw(&mut self, func: impl FnOnce(&mut Self)) {
        self.wait_for_flush();
        func(self);
        self.wait_for_vsync();
        self.start_flush();
    }

    pub fn enable_backlight(&mut self) {
        self.backlight_pin.set_high().unwrap();
    }

    pub fn disable_backlight(&mut self) {
        self.backlight_pin.set_low().unwrap();
    }

    pub fn wait_for_vsync(&mut self) {
        /*         if self.last_vsync_time != 0 && time::time_us() - self.last_vsync_time > 16_000 {
            defmt::info!("Missed vsync");
        } */
        // defmt::info!("frametime {0}",time::time_us() - self.last_vsync_time);
        while self.vsync_pin.is_high().unwrap() {}
        while self.vsync_pin.is_low().unwrap() {}
        self.last_vsync_time = time::time_us();
    }

    pub fn flush_progress(&self) -> usize {
        if self.dma_channel.get_count() == 0 {
            return WIDTH * HEIGHT;
        }
        (self.dma_channel.get_src() as usize - framebuffer().as_ptr() as usize) / 2
    }
}

impl<
        BL: PinId,
        DC: PinId,
        CS: PinId,
        VSYNC: PinId,
        SPIDEV: rp2040_hal::spi::SpiDevice,
        SPIPINOUT: rp2040_hal::spi::ValidSpiPinout<SPIDEV>,
    > OriginDimensions for Display<BL, DC, CS, VSYNC, SPIDEV, SPIPINOUT>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl<
        BL: PinId,
        DC: PinId,
        CS: PinId,
        VSYNC: PinId,
        SPIDEV: rp2040_hal::spi::SpiDevice,
        SPIPINOUT: rp2040_hal::spi::ValidSpiPinout<SPIDEV>,
    > DrawTarget for Display<BL, DC, CS, VSYNC, SPIDEV, SPIPINOUT>
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        const M: u32 = WIDTH as u32 - 1;
        const N: u32 = HEIGHT as u32 - 1;
        let fb = framebuffer();
        for Pixel(coord, color) in pixels.into_iter() {
            if let Ok((x @ 0..=M, y @ 0..=N)) = coord.try_into() {
                let index: u32 = x + y * WIDTH as u32;
                let color = RawU16::from(color).into_inner();
                fb[index as usize] = color.to_be();
            }
        }
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let clipped_area = area.intersection(&self.bounding_box());
        if area.bottom_right().is_none() || clipped_area.bottom_right().is_none() {
            return Ok(());
        }

        let skip_top_left = clipped_area.top_left - area.top_left;
        let skip_bottom_right = area.bottom_right().unwrap() - clipped_area.bottom_right().unwrap();

        let fb = framebuffer();
        let mut colors = colors.into_iter();

        for _ in 0..skip_top_left.y {
            for _ in 0..area.size.width {
                colors.next();
            }
        }

        for y in 0..clipped_area.size.height as i32 {
            for _ in 0..skip_top_left.x {
                colors.next();
            }

            let mut index = clipped_area.top_left.x + (clipped_area.top_left.y + y) * WIDTH as i32;
            for _ in 0..clipped_area.size.width {
                let color = colors.next().unwrap_or(Rgb565::RED);
                let color = RawU16::from(color).into_inner();
                fb[index as usize] = color.to_be();
                index += 1;
            }

            for _ in 0..skip_bottom_right.x {
                colors.next();
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let color = RawU16::from(color).into_inner().to_be();
        unsafe {
            dma::set_mem(
                &mut self.dma_channel,
                &color as *const u16 as u32,
                framebuffer().as_ptr() as u32,
                2,
                (WIDTH * HEIGHT) as u32,
            );
        }
        if framebuffer()[0] != color {
            defmt::info!(
                "incorrect framebuffer[0], expected {} got {}",
                color,
                framebuffer()[0]
            );
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
    }
}

#[rp_pico::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}",
        rp2040_hal::rom_data::copyright_string(),
        rp2040_hal::rom_data::git_revision(),
        rp2040_hal::rom_data::rom_version_number(),
    );

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = rp2040_hal::sio::Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut red_led_pin = pins.gpio26.into_push_pull_output();
    let mut green_led_pin = pins.gpio27.into_push_pull_output();
    let mut blue_led_pin = pins.gpio28.into_push_pull_output();

    red_led_pin.set_high().unwrap();
    green_led_pin.set_high().unwrap();
    blue_led_pin.set_high().unwrap();

    let backlight_pin = pins.gpio20.into_push_pull_output();
    let dc_pin = pins.gpio16.into_push_pull_output();
    let cs_pin = pins.gpio17.into_push_pull_output();
    let sck_pin = pins.gpio18.into_function::<FunctionSpi>();
    let mosi_pin = pins.gpio19.into_function::<FunctionSpi>();
    let vsync_pin = pins.gpio21.into_floating_input();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut display = Display::new(
        backlight_pin,
        dc_pin,
        cs_pin,
        vsync_pin,
        pac.SPI0,
        (mosi_pin, sck_pin),
        &mut delay,
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500.kHz(),
        unsafe { dma::DmaChannel::new(dma::CHANNEL_FRAMEBUFFER) },
    );
    display.fill(Rgb565::BLACK.into_storage());
    delay.delay_ms(1000);
    display.clear(Rgb565::BLUE).unwrap();
    display.flush();

    loop {
        cortex_m::asm::wfe();
    }
}
