#![no_std]
#![no_main]

use core::convert::TryInto;
use display_interface_spi::SPIInterface;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::spi::{MODE_0, MODE_3};
use fugit::RateExtU32;
use hal::gpio::Pins;
use hal::pac;
use hal::spi::Spi;
use rp2040_hal::clocks::{ClocksManager, InitError};
use rp2040_hal::gpio::{DynFunction, DynPin, DynPinMode};
use rp2040_hal::pll::PLLConfig;
use rp2040_hal::{self as hal, Clock, Watchdog};
use st7789::{TearingEffect, ST7789};

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

mod time {
    pub fn time_us() -> u32 {
        unsafe { (*rp2040_pac::TIMER::PTR).timerawl.read().bits() }
    }

    pub fn time_us64() -> u64 {
        unsafe {
            (*rp2040_pac::TIMER::PTR).timelr.read().bits() as u64
                | (((*rp2040_pac::TIMER::PTR).timehr.read().bits() as u64) << 32)
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
                ch: &(*rp2040_pac::DMA::PTR).ch[channel],
            }
        }

        pub unsafe fn set_src(&mut self, src: u32) {
            self.ch.ch_read_addr.write(|w| w.bits(src));
        }

        pub unsafe fn set_dst(&mut self, dst: u32) {
            self.ch.ch_write_addr.write(|w| w.bits(dst));
        }

        pub unsafe fn set_count(&mut self, count: u32) {
            self.ch.ch_trans_count.write(|w| w.bits(count));
        }

        pub unsafe fn set_ctrl_and_trigger<F>(&mut self, f: F)
        where
            F: FnOnce(&mut CtrlWriter) -> &mut W<CtrlReg>,
        {
            self.ch.ch_ctrl_trig.write(f);
        }

        pub fn wait(&self) {
            while self.ch.ch_trans_count.read().bits() > 0 {}
        }

        pub fn get_src(&self) -> u32 {
            self.ch.ch_read_addr.read().bits()
        }

        pub fn get_dst(&self) -> u32 {
            self.ch.ch_write_addr.read().bits()
        }

        pub fn get_count(&self) -> u32 {
            self.ch.ch_trans_count.read().bits()
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
        while xip_ctrl.stat.read().fifo_empty().bit_is_clear() {
            defmt::info!("XIP FIFO not empty");
            cortex_m::asm::nop();
        }
        xip_ctrl.stream_addr.write(|w| w.bits(src));
        xip_ctrl.stream_ctr.write(|w| w.bits(count));

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

        while xip_ctrl.stat.read().fifo_empty().bit_is_clear() {
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
    static mut FRAMEBUFFER: [u16; WIDTH * HEIGHT] = [0; WIDTH * HEIGHT];
    unsafe { core::slice::from_raw_parts_mut(FRAMEBUFFER.as_ptr() as *mut u16, WIDTH * HEIGHT) }
}

pub type RealDisplay = st7789::ST7789<
    SPIInterface<Spi<hal::spi::Enabled, pac::SPI0, 8>, DynPin, DynPin>,
    DynPin,
    DynPin,
>;

pub struct Display {
    st7789: RealDisplay,
    lcd_vsync_pin: DynPin,
    dma_channel: dma::DmaChannel,
    last_vsync_time: u32,
}

/*
    let spi_screen =
        Spi::<_, _, 8>::new(hw.SPI0).init( p.RESETS, 125u32.MHz(), 16u32.MHz(), &MODE_0);
    let spii_screen = SPIInterface::new(spi_screen, hw.lcd_dc_pin, hw.lcd_cs_pin);
    let mut display = mipidsi::Builder::st7789(spii_screen)
        .with_display_size(240, 240)
        .with_framebuffer_size(240, 240)
        .init(&mut delay, Some(DummyPin))
        .unwrap();

*/

impl Display {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mut backlight_pin: DynPin,
        mut lcd_dc_pin: DynPin,
        mut lcd_cs_pin: DynPin,
        mut lcd_sck_pin: DynPin,
        mut lcd_mosi_pin: DynPin,
        mut lcd_vsync_pin: DynPin,
        spi_device: pac::SPI0,
        resets: &mut pac::RESETS,
        delay_source: &mut impl DelayUs<u32>,
        dma_channel: dma::DmaChannel,
    ) -> Display {
        info!("Initializing display");
        backlight_pin.into_push_pull_output();
        lcd_dc_pin.into_push_pull_output();
        lcd_cs_pin.into_push_pull_output();
        lcd_cs_pin.set_low().unwrap();
        lcd_sck_pin
            .try_into_mode(DynPinMode::Function(DynFunction::Spi))
            .unwrap();
        lcd_mosi_pin
            .try_into_mode(DynPinMode::Function(DynFunction::Spi))
            .unwrap();
        lcd_vsync_pin.into_floating_input();
        let spi =
            Spi::<_, _, 8>::new(spi_device).init(resets, 125.MHz(), 62_500_000u32.Hz(), &MODE_0);
        let di = SPIInterface::new(spi, lcd_dc_pin, lcd_cs_pin);
        let mut st7789 = ST7789::new(di, None, Some(backlight_pin), WIDTH as u16, HEIGHT as u16);
        st7789.init(delay_source).unwrap();
        st7789.set_tearing_effect(TearingEffect::Vertical).unwrap();
        let mut display = Display {
            st7789,
            dma_channel,
            lcd_vsync_pin,
            last_vsync_time: 0,
        };
        // A single clear occasionally fails to clear the screen.
        for _ in 0..2 {
            // let colors =
            // core::iter::repeat(RawU16::from(Rgb565::BLACK).into_inner()).take(WIDTH * HEIGHT);
            let colors = core::iter::repeat(Rgb565::BLACK.into_storage()).take(WIDTH * HEIGHT);
            display
                .st7789
                .set_pixels(0, 0, (WIDTH - 1) as u16, (HEIGHT - 1) as u16, colors)
                .unwrap();
        }
        display
            .st7789
            .set_pixel(2, 2, Rgb565::BLUE.into_storage())
            .unwrap();
        display.enable_backlight(delay_source);
        display
    }

    fn start_flush(&mut self) {
        unsafe {
            dma::start_copy_to_spi(
                &mut self.dma_channel,
                framebuffer().as_ptr() as u32,
                (*pac::SPI0::PTR).sspdr.as_ptr() as u32,
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

    pub fn enable_backlight(&mut self, delay_source: &mut impl DelayUs<u32>) {
        self.st7789
            .set_backlight(st7789::BacklightState::On, delay_source)
            .unwrap();
    }

    pub fn disable_backlight(&mut self, delay_source: &mut impl DelayUs<u32>) {
        self.st7789
            .set_backlight(st7789::BacklightState::Off, delay_source)
            .unwrap();
    }

    pub fn wait_for_vsync(&mut self) {
        /*         if self.last_vsync_time != 0 && time::time_us() - self.last_vsync_time > 16_000 {
            defmt::info!("Missed vsync");
        } */
        // defmt::info!("frametime {0}",time::time_us() - self.last_vsync_time);
        while self.lcd_vsync_pin.is_high().unwrap() {}
        while self.lcd_vsync_pin.is_low().unwrap() {}
        self.last_vsync_time = time::time_us();
    }

    pub fn flush_progress(&self) -> usize {
        if self.dma_channel.get_count() == 0 {
            return WIDTH * HEIGHT;
        }
        (self.dma_channel.get_src() as usize - framebuffer().as_ptr() as usize) / 2
    }
}

impl DrawTarget for Display {
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

impl OriginDimensions for Display {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

pub struct XorDisplay<'a> {
    display: &'a mut Display,
}

impl<'a> XorDisplay<'a> {
    pub fn new(display: &'a mut Display) -> XorDisplay {
        XorDisplay { display }
    }
}

impl<'a> DrawTarget for XorDisplay<'a> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        const M: u32 = WIDTH as u32 - 1;
        let fb = framebuffer();
        for Pixel(coord, color) in pixels.into_iter() {
            if let Ok((x @ 0..=M, y @ 0..=M)) = coord.try_into() {
                let index: u32 = x + y * WIDTH as u32;
                let color = RawU16::from(color).into_inner();
                fb[index as usize] ^= color.to_be();
            }
        }

        Ok(())
    }
}

impl<'a> OriginDimensions for XorDisplay<'a> {
    fn size(&self) -> Size {
        self.display.size()
    }
}

#[rp_pico::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}",
        hal::rom_data::copyright_string(),
        hal::rom_data::git_revision(),
        hal::rom_data::rom_version_number(),
    );

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // The default is to generate a 125 MHz system clock
    let clocks = init_clocks_and_plls(
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::sio::Sio::new(pac.SIO);
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

    let backlight_pin = pins.gpio20.into();
    let lcd_dc_pin = pins.gpio16.into();
    let lcd_cs_pin = pins.gpio17.into();
    let lcd_sck_pin = pins.gpio18.into();
    let lcd_mosi_pin = pins.gpio19.into();
    let lcd_vsync_pin = pins.gpio21.into();

    let mut display = Display::new(
        backlight_pin,
        lcd_dc_pin,
        lcd_cs_pin,
        lcd_sck_pin,
        lcd_mosi_pin,
        lcd_vsync_pin,
        pac.SPI0,
        &mut pac.RESETS,
        &mut delay,
        unsafe { dma::DmaChannel::new(dma::CHANNEL_FRAMEBUFFER) },
    );
    display.clear(Rgb565::BLUE).unwrap();
    display.flush();

    loop {
        cortex_m::asm::wfe();
    }
}

// Copied and modified from rp2040_hal crate.
fn init_clocks_and_plls(
    xosc_crystal_freq: u32,
    xosc_dev: pac::XOSC,
    clocks_dev: pac::CLOCKS,
    pll_sys_dev: pac::PLL_SYS,
    pll_usb_dev: pac::PLL_USB,
    resets: &mut pac::RESETS,
    watchdog: &mut Watchdog,
) -> Result<ClocksManager, InitError> {
    let xosc = rp2040_hal::xosc::setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz())
        .map_err(InitError::XoscErr)?;

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys_180mhz: PLLConfig = PLLConfig {
        vco_freq: 716.MHz(),
        refdiv: 1,
        post_div1: 4,
        post_div2: 1,
    };

    let pll_sys = rp2040_hal::pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        pll_sys_180mhz,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;
    let pll_usb = rp2040_hal::pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        rp2040_hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(InitError::ClockError)?;
    Ok(clocks)
}
