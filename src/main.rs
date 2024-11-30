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
use embedded_hal::pwm::SetDutyCycle;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::{dma, gpio, pwm, rom_data, spi, Clock};
use spi::Spi;

use core::marker::PhantomData;
use defmt_rtt as _;
use embedded_hal::spi::SpiBus;
use panic_probe as _;

mod float;
mod lax_dma;

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

pub const WIDTH: usize = 240;
pub const HEIGHT: usize = 320;

static mut FRAMEBUFFER: [u16; WIDTH * HEIGHT] = [0; WIDTH * HEIGHT]; // TODO: Use StaticCell

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

pub struct Display<DC, CS, VSYNC, SPIDEV, SPIPINOUT, PWMSLICE, PWMCHAN, DMAX, DMAY>
where
    DC: gpio::PinId,
    CS: gpio::PinId,
    VSYNC: gpio::PinId,
    SPIDEV: spi::SpiDevice,
    SPIPINOUT: spi::ValidSpiPinout<SPIDEV>,
    PWMSLICE: pwm::AnySlice,
    PWMCHAN: pwm::ChannelId,
    DMAX: dma::ChannelIndex,
    DMAY: dma::ChannelIndex,
    pwm::Channel<PWMSLICE, PWMCHAN>: SetDutyCycle,
{
    backlight_pwm: pwm::Channel<PWMSLICE, PWMCHAN>,
    dc_pin: gpio::Pin<DC, gpio::FunctionSioOutput, gpio::PullDown>,
    cs_pin: gpio::Pin<CS, gpio::FunctionSioOutput, gpio::PullDown>,
    vsync_pin: gpio::Pin<VSYNC, gpio::FunctionSioInput, gpio::PullNone>,
    spi_device: Spi<spi::Enabled, SPIDEV, SPIPINOUT, 8>,
    dma_channel_x: PhantomData<DMAX>,
    dma_channel_y: PhantomData<DMAY>,
    sspdr: *mut u32,
    last_vsync_time: u32,
}

impl<
        DC: gpio::PinId,
        CS: gpio::PinId,
        VSYNC: gpio::PinId,
        SPIDEV: spi::SpiDevice,
        SPIPINOUT: spi::ValidSpiPinout<SPIDEV>,
        PWMSLICE: pwm::AnySlice,
        PWMCHAN: pwm::ChannelId,
        DMAX: dma::ChannelIndex,
        DMAY: dma::ChannelIndex,
    > Display<DC, CS, VSYNC, SPIDEV, SPIPINOUT, PWMSLICE, PWMCHAN, DMAX, DMAY>
where
    pwm::Channel<PWMSLICE, PWMCHAN>: SetDutyCycle,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new<F, B>(
        backlight_pwm: pwm::Channel<PWMSLICE, PWMCHAN>,
        dc_pin: gpio::Pin<DC, gpio::FunctionSioOutput, gpio::PullDown>,
        cs_pin: gpio::Pin<CS, gpio::FunctionSioOutput, gpio::PullDown>,
        vsync_pin: gpio::Pin<VSYNC, gpio::FunctionSioInput, gpio::PullNone>,
        spi_device: SPIDEV,
        spi_pinout: SPIPINOUT,
        _dma_channel_x: dma::Channel<DMAX>,
        _dma_channel_y: dma::Channel<DMAY>,
        delay_source: &mut cortex_m::delay::Delay,
        resets: &mut rp2040_pac::RESETS,
        peri_frequency: F,
        baudrate: B,
    ) -> Self
    where
        F: Into<HertzU32>,
        B: Into<HertzU32>,
    {
        let sspdr = spi_device.sspdr().as_ptr(); // For DMA
        let spi_device = Spi::<_, _, _, 8>::new(spi_device, spi_pinout);
        let spi_device = spi_device.init(
            resets,
            peri_frequency.into(),
            baudrate.into(),
            embedded_hal::spi::MODE_0,
        );
        let mut display = Self {
            backlight_pwm,
            dc_pin,
            cs_pin,
            vsync_pin,
            spi_device,
            sspdr,
            dma_channel_x: PhantomData,
            dma_channel_y: PhantomData,
            last_vsync_time: 0,
        };

        display.no_backlight();
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

        display.set_backlight(20);
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

    pub fn flush(&mut self) {
        self.set_address_window(0, 0, HEIGHT as u16 - 1, WIDTH as u16 - 1);
        self.write_command(Command::RAMWR);

        self.cs_pin.set_low().unwrap();
        self.dc_pin.set_high().unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_16bit,
            source: lax_dma::Source {
                address: unsafe { FRAMEBUFFER.as_ptr().cast() },
                increment: true,
            },
            destination: lax_dma::Destination {
                address: self.sspdr.cast(),
                increment: false,
            },
            tx_count: WIDTH as u32 * HEIGHT as u32,
            tx_req: lax_dma::TxReq::Spi0Tx, // TODO: hardcoded
            byte_swap: true,
            start: true,
        };

        let dma: lax_dma::LaxDmaWrite<DMAY> = lax_dma::LaxDmaWrite::new(dma_config);
        //dma.trigger();
        dma.wait();

        self.cs_pin.set_high().unwrap();

        //defmt::info!("flush done, time: {:x}", time::time_us());
    }

    pub fn set_backlight(&mut self, value: u8) {
        const GAMMA: f32 = 2.8;
        let pwm = (float::rom_instrinsics::powf32((value as f32) / 255.0f32, GAMMA) * 65535.0f32
            + 0.5f32) as u16;

        // defmt::info!("ROM computed backlight setting: {}", pwm);
        // let pwm = (float::handrolled::powf32((value as f32) / 255.0f32, GAMMA) * 65535.0f32
        //     + 0.5f32) as u16;
        defmt::info!("Setting backlight to {}", pwm);
        self.backlight_pwm
            .set_duty_cycle_fraction(value as u16, 255)
            .unwrap();
    }

    pub fn full_backlight(&mut self) {
        defmt::info!("Enabling backlight fully");
        self.backlight_pwm.set_duty_cycle_fully_on().unwrap();
    }

    pub fn no_backlight(&mut self) {
        defmt::info!("Disabling backlight");
        self.backlight_pwm.set_duty_cycle_fully_off().unwrap();
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
        0
    }
}

impl<
        DC: gpio::PinId,
        CS: gpio::PinId,
        VSYNC: gpio::PinId,
        SPIDEV: spi::SpiDevice,
        SPIPINOUT: spi::ValidSpiPinout<SPIDEV>,
        PWMSLICE: pwm::AnySlice,
        PWMCHAN: pwm::ChannelId,
        DMAX: dma::ChannelIndex,
        DMAY: dma::ChannelIndex,
    > OriginDimensions for Display<DC, CS, VSYNC, SPIDEV, SPIPINOUT, PWMSLICE, PWMCHAN, DMAX, DMAY>
where
    pwm::Channel<PWMSLICE, PWMCHAN>: SetDutyCycle,
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl<
        DC: gpio::PinId,
        CS: gpio::PinId,
        VSYNC: gpio::PinId,
        SPIDEV: spi::SpiDevice,
        SPIPINOUT: spi::ValidSpiPinout<SPIDEV>,
        PWMSLICE: pwm::AnySlice,
        PWMCHAN: pwm::ChannelId,
        DMAX: dma::ChannelIndex,
        DMAY: dma::ChannelIndex,
    > DrawTarget for Display<DC, CS, VSYNC, SPIDEV, SPIPINOUT, PWMSLICE, PWMCHAN, DMAX, DMAY>
where
    pwm::Channel<PWMSLICE, PWMCHAN>: SetDutyCycle,
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        const M: u32 = WIDTH as u32 - 1;
        const N: u32 = HEIGHT as u32 - 1;
        for Pixel(coord, color) in pixels.into_iter() {
            if let Ok((x @ 0..=M, y @ 0..=N)) = coord.try_into() {
                let index: u32 = x + y * WIDTH as u32;
                let color = RawU16::from(color).into_inner();
                unsafe { FRAMEBUFFER[index as usize] = color.to_be() };
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
                unsafe { FRAMEBUFFER[index as usize] = color.to_be() };
                index += 1;
            }

            for _ in 0..skip_bottom_right.x {
                colors.next();
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        static mut SOURCE: [u16; 1] = [0];
        unsafe { SOURCE[0] = color.into_storage() };

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_16bit,
            source: lax_dma::Source {
                address: unsafe { SOURCE.as_ptr().cast() },
                increment: false,
            },
            destination: lax_dma::Destination {
                address: unsafe { FRAMEBUFFER.as_mut_ptr().cast() },
                increment: true,
            },
            tx_count: WIDTH as u32 * HEIGHT as u32,
            tx_req: lax_dma::TxReq::Permanent,
            byte_swap: false,
            start: true,
        };

        let dma: lax_dma::LaxDmaWrite<DMAX> = lax_dma::LaxDmaWrite::new(dma_config);
        //dma.trigger();
        dma.wait();
        // defmt::info!("DMA done, first word: {:?}", unsafe { FRAMEBUFFER[0] });
        // defmt::info!("DMA done, last word: {:?}", unsafe {
        //     FRAMEBUFFER[WIDTH * HEIGHT - 1]
        // });

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
    }
}

#[rp_pico::entry]
fn main() -> ! {
    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}, time {:x} us",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
        time::time_us64()
    );

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let core = rp2040_pac::CorePeripherals::take().unwrap();
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
    let pins = gpio::Pins::new(
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

    let backlight_pin = pins.gpio20.into_function::<gpio::FunctionPwm>();
    let dc_pin = pins.gpio16.into_push_pull_output();
    let cs_pin = pins.gpio17.into_push_pull_output();
    let sck_pin = pins.gpio18.into_function::<gpio::FunctionSpi>();
    let mosi_pin = pins.gpio19.into_function::<gpio::FunctionSpi>();
    let vsync_pin = pins.gpio21.into_floating_input();

    let dma = pac.DMA.split(&mut pac.RESETS);
    //lax_dma::tests::run_dma_tests();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut backlight_pwm = pwm_slices.pwm2;
    backlight_pwm.set_ph_correct();
    backlight_pwm.enable();
    backlight_pwm.channel_a.output_to(backlight_pin);
    backlight_pwm.channel_a.set_duty_cycle(0).unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut display = Display::new(
        backlight_pwm.channel_a,
        dc_pin,
        cs_pin,
        vsync_pin,
        pac.SPI0,
        (mosi_pin, sck_pin),
        dma.ch0,
        dma.ch1,
        &mut delay,
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500.kHz(),
    );
    display.set_backlight(40);
    display.fill(Rgb565::GREEN.into_storage());

    display.clear(Rgb565::RED).unwrap();
    display.flush();

    loop {
        cortex_m::asm::wfe();
        defmt::info!("WFE time: {:x}", time::time_us64());
    }
}
