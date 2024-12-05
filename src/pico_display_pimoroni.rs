//! Pimoroni Pico Display driver for the Raspberry Pi Pico
//!
//! Display variants supported:
//!
//! * Pico Display Pack: https://shop.pimoroni.com/products/pico-display-pack
//! * Pimoroni Pico Display 2.0: https://shop.pimoroni.com/products/pico-display-pack-2-0
//! * Pimoroni Pico Display 2.8: https://shop.pimoroni.com/products/pico-display-pack-2-8
//!
//! The initialization code taken from the Pimoroni C++ library:
//!
//! https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp
//!
//! The `DrawTarget` implementation is based on the `st7789` crate.
//!
//#![no_std]

#![allow(dead_code)]

use crate::lax_dma;
use core::marker::PhantomData;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::Dimensions;
use embedded_graphics::prelude::DrawTarget;
use embedded_graphics::prelude::IntoStorage;
use embedded_graphics::prelude::OriginDimensions;
use embedded_graphics::prelude::RawData;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::prelude::Size;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Pixel;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::spi::SpiBus;
use fugit::HertzU32;
use rp2040_hal::dma;
use rp2040_hal::gpio;
use rp2040_hal::pwm;
use rp2040_hal::spi;
use rp2040_hal::Spi;

#[derive(Copy, Clone, PartialEq, Eq)]
#[allow(dead_code)]
pub enum DisplayKind {
    PicoDisplay1_14,
    PicoDisplaySquare,
    PicoDisplay2_0,
    PicoDisplay2_8,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum DisplayRotation {
    Rotate0,
    Rotate90,
    Rotate180,
    Rotate270,
}

#[derive(Copy, Clone, PartialEq)]
pub enum TearingEffect {
    Off,
    Vertical,
    HorizontalAndVertical,
}

const DISPLAY_KIND: DisplayKind = DisplayKind::PicoDisplay2_8;
const DISPLAY_ROTATION: DisplayRotation = DisplayRotation::Rotate0;

const fn get_display_dimensions(kind: DisplayKind) -> (u16, u16) {
    match kind {
        DisplayKind::PicoDisplay1_14 => (240, 135),
        DisplayKind::PicoDisplaySquare => (240, 240),
        DisplayKind::PicoDisplay2_0 => (320, 240),
        DisplayKind::PicoDisplay2_8 => (320, 240),
    }
}

pub const WIDTH: usize = get_display_dimensions(DISPLAY_KIND).0 as usize;
pub const HEIGHT: usize = get_display_dimensions(DISPLAY_KIND).1 as usize;

const MADCTL_ROW_ORDER: u8 = 0b10000000;
const MADCTL_COL_ORDER: u8 = 0b01000000;
const MADCTL_SWAP_XY: u8 = 0b00100000; // AKA "MV"
const MADCTL_SCAN_ORDER: u8 = 0b00010000;
const MADCTL_RGB_BGR: u8 = 0b00001000;
const MADCTL_HORIZ_ORDER: u8 = 0b00000100;

// TODO: move outside of the module, pass a reference to the framebuffer, or the block of memory
// to draw on and send to the display.
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
    kind: DisplayKind,
    rotation: DisplayRotation,
    pixel_count: u32,
    backlight_pwm: pwm::Channel<PWMSLICE, PWMCHAN>,
    dc_pin: gpio::Pin<DC, gpio::FunctionSioOutput, gpio::PullDown>,
    cs_pin: gpio::Pin<CS, gpio::FunctionSioOutput, gpio::PullDown>,
    vsync_pin: gpio::Pin<VSYNC, gpio::FunctionSioInput, gpio::PullNone>,
    spi_device: Spi<spi::Enabled, SPIDEV, SPIPINOUT, 8>,
    dma_channel_x: PhantomData<DMAX>,
    dma_channel_y: PhantomData<DMAY>,
    sspdr: *mut u32,
    tearing_effect: TearingEffect,
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
            kind: DISPLAY_KIND,
            rotation: DISPLAY_ROTATION,
            pixel_count: WIDTH as u32 * HEIGHT as u32,
            tearing_effect: TearingEffect::Off,
        };

        display.no_backlight();
        display.write_command(Command::SWRESET); // software reset

        delay_source.delay_ms(150);

        display.write_command_with_data(Command::COLMOD, &[0x55]); // 16 bits per pixel, RGB565

        display.write_command_with_data(Command::PORCTRL, &[0x0c, 0x0c, 0x00, 0x33, 0x33]);
        display.write_command_with_data(Command::LCMCTRL, &[0x2c]);
        display.write_command_with_data(Command::VDVVRHEN, &[0x01]);
        display.write_command_with_data(Command::VRHS, &[0x12]);
        display.write_command_with_data(Command::VDVS, &[0x20]);
        display.write_command_with_data(Command::PWCTRL1, &[0xa4, 0xa1]);
        /*
           Frame rate:
           0x00 = 119Hz, 0x10 = 58Hz,
           0x01 = 111Hz, 0x11 = 57Hz,
           0x02 = 105Hz, 0x12 = 55Hz,
           0x03 = 99Hz, 0x13 = 53Hz,
           0x04 = 94Hz, 0x14 = 52Hz,
           0x05 = 90Hz, 0x15 = 50Hz,
           0x06 = 86Hz, 0x16 = 49Hz,
           0x07 = 82Hz, 0x17 = 48Hz,
           0x08 = 78Hz, 0x18 = 46Hz,
           0x09 = 75Hz, 0x19 = 45Hz,
           0x0A = 72Hz, 0x1A = 44Hz,
           0x0B = 69Hz, 0x1B = 43Hz,
           0x0C = 67Hz, 0x1C = 42Hz,
           0x0D = 64Hz, 0x1D = 41Hz,
           0x0E = 62Hz, 0x1E = 40Hz,
           0x0F = 60Hz, 0x1F = 39Hz
        */
        display.write_command_with_data(Command::FRCTRL2, &[0x1f]);

        match display.kind {
            DisplayKind::PicoDisplaySquare => {
                display.write_command_with_data(Command::GCTRL, &[0x14]);
                display.write_command_with_data(Command::VCOMS, &[0x37]);
                display.write_command_with_data(
                    Command::GMCTRP1,
                    &[
                        0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B,
                        0x1F, 0x23,
                    ],
                );
                display.write_command_with_data(
                    Command::GMCTRN1,
                    &[
                        0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F,
                        0x20, 0x23,
                    ],
                );
            }

            DisplayKind::PicoDisplay2_0 | DisplayKind::PicoDisplay2_8 => {
                display.write_command_with_data(Command::GCTRL, &[0x35]);
                display.write_command_with_data(Command::VCOMS, &[0x1f]);
                display.write_command_with_data(
                    Command::GMCTRP1,
                    &[
                        0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14,
                        0x29, 0x2D,
                    ],
                );
                display.write_command_with_data(
                    Command::GMCTRN1,
                    &[
                        0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14,
                        0x2F, 0x31,
                    ],
                );
            }

            DisplayKind::PicoDisplay1_14 => {
                display.write_command_with_data(Command::VRHS, &[0x00]); // VRH Voltage setting
                display.write_command_with_data(Command::GCTRL, &[0x75]); // VGH and VGL voltages
                display.write_command_with_data(Command::VCOMS, &[0x3D]); // VCOM voltage
                display.write_command_with_data(Command::_D6, &[0xa1]); // ???
                display.write_command_with_data(
                    Command::GMCTRP1,
                    &[
                        0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2A, 0x33, 0x41, 0x07, 0x13, 0x13,
                        0x29, 0x2f,
                    ],
                );
                display.write_command_with_data(
                    Command::GMCTRN1,
                    &[
                        0x70, 0x03, 0x09, 0x0A, 0x09, 0x06, 0x2B, 0x34, 0x41, 0x07, 0x12, 0x14,
                        0x28, 0x2E,
                    ],
                );
            }
        }

        display.write_command(Command::INVON); // set inversion mode
        delay_source.delay_ms(10);
        display.write_command(Command::SLPOUT); // leave sleep mode
        delay_source.delay_ms(10);
        display.write_command(Command::NORON); // leave sleep mode
        delay_source.delay_ms(10);
        display.write_command(Command::DISPON); // turn display on
        delay_source.delay_ms(10);

        display.configure_display();
        display.set_tearing_effect(TearingEffect::HorizontalAndVertical);

        display.set_backlight(40);
        delay_source.delay_ms(10);

        display
    }

    fn configure_display(&mut self) {
        let round = false;
        let kind = self.kind;
        let rotation = self.rotation;
        let (mut width, mut height) = get_display_dimensions(kind);

        if rotation == DisplayRotation::Rotate90 || rotation == DisplayRotation::Rotate270 {
            core::mem::swap(&mut width, &mut height);
        }

        let mut caset = [0u16; 2];
        let mut raset = [0u16; 2];
        let mut madctl: u8 = 0;

        // 240x240 Square and Round LCD Breakouts
        if width == 240 && height == 240 {
            let mut row_offset = if round { 40 } else { 80 };
            let col_offset = 0;

            match rotation {
                DisplayRotation::Rotate90 => {
                    if !round {
                        row_offset = 0;
                    }
                    caset[0] = row_offset;
                    caset[1] = width + row_offset - 1;
                    raset[0] = col_offset;
                    raset[1] = width + col_offset - 1;

                    madctl = MADCTL_HORIZ_ORDER | MADCTL_COL_ORDER | MADCTL_SWAP_XY;
                }
                DisplayRotation::Rotate180 => {
                    caset[0] = col_offset;
                    caset[1] = width + col_offset - 1;
                    raset[0] = row_offset;
                    raset[1] = width + row_offset - 1;

                    madctl = MADCTL_HORIZ_ORDER | MADCTL_COL_ORDER | MADCTL_ROW_ORDER;
                }
                DisplayRotation::Rotate270 => {
                    caset[0] = row_offset;
                    caset[1] = width + row_offset - 1;
                    raset[0] = col_offset;
                    raset[1] = width + col_offset - 1;

                    madctl = MADCTL_ROW_ORDER | MADCTL_SWAP_XY;
                }
                _ => {
                    // Default to Rotate0
                    if !round {
                        row_offset = 0;
                    }
                    caset[0] = col_offset;
                    caset[1] = width + col_offset - 1;
                    raset[0] = row_offset;
                    raset[1] = width + row_offset - 1;

                    madctl = MADCTL_HORIZ_ORDER;
                }
            }
        }
        // Pico Display
        else if width == 240 && height == 135 {
            caset[0] = 40; // 240 columns
            caset[1] = 40 + width - 1;
            raset[0] = 52; // 135 rows
            raset[1] = 52 + height - 1;

            if rotation == DisplayRotation::Rotate0 {
                raset[0] += 1;
                raset[1] += 1;
            }

            madctl = if rotation == DisplayRotation::Rotate180 {
                MADCTL_ROW_ORDER
            } else {
                MADCTL_COL_ORDER
            };
            madctl |= MADCTL_SWAP_XY | MADCTL_SCAN_ORDER;
        }
        // Pico Display at 90 degree rotation
        else if width == 135 && height == 240 {
            caset[0] = 52; // 135 columns
            caset[1] = 52 + width - 1;
            raset[0] = 40; // 240 rows
            raset[1] = 40 + height - 1;

            madctl = if rotation == DisplayRotation::Rotate90 {
                caset[0] += 1;
                caset[1] += 1;
                MADCTL_COL_ORDER | MADCTL_ROW_ORDER
            } else {
                0
            };
        }
        // Pico Display 2.0 and 2.8
        else if width == 320 && height == 240 {
            caset[0] = 0;
            caset[1] = 319;
            raset[0] = 0;
            raset[1] = 239;

            madctl = if rotation == DisplayRotation::Rotate180
                || rotation == DisplayRotation::Rotate90
            {
                MADCTL_ROW_ORDER
            } else {
                MADCTL_COL_ORDER
            };
            madctl |= MADCTL_SWAP_XY | MADCTL_SCAN_ORDER;
        }
        // Pico Display 2.0 at 90 degree rotation
        else if width == 240 && height == 320 {
            caset[0] = 0;
            caset[1] = 239;
            raset[0] = 0;
            raset[1] = 319;

            madctl = if rotation == DisplayRotation::Rotate180
                || rotation == DisplayRotation::Rotate90
            {
                MADCTL_COL_ORDER | MADCTL_ROW_ORDER
            } else {
                0
            };
        }

        self.set_address_window(caset[0], raset[0], caset[1], raset[1]);

        self.write_command(Command::MADCTL);
        self.write_data(&[madctl]);
    }

    #[inline(always)]
    fn write_command(&mut self, command: Command) {
        self.dc_pin.set_low().unwrap();

        self.cs_pin.set_low().unwrap();
        self.spi_device.write(&[command as u8]).unwrap(); // TODO: Handle error
        self.cs_pin.set_high().unwrap();

        // defmt::info!("Command 0x{:x}", command as u8);
    }

    #[inline(always)]
    fn write_data(&mut self, data: &[u8]) {
        self.dc_pin.set_high().unwrap();

        self.cs_pin.set_low().unwrap();
        for byte in data {
            self.spi_device.write(&[*byte]).unwrap(); // TODO: Handle error
        }
        self.cs_pin.set_high().unwrap();

        // defmt::info!("Command 0x{:x}", command as u8);
    }

    #[inline(always)]
    fn write_command_with_data(&mut self, command: Command, data: &[u8]) {
        self.write_command(command);
        self.write_data(data);
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

    fn write_framebuffer(&mut self) {
        self.set_address_window(0, 0, WIDTH as u16 - 1, HEIGHT as u16 - 1);
        self.write_command(Command::RAMWR);

        self.cs_pin.set_low().unwrap();
        self.dc_pin.set_high().unwrap();

        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_8bit,
            source: lax_dma::Source {
                address: unsafe { FRAMEBUFFER.as_ptr().cast() },
                increment: true,
            },
            destination: lax_dma::Destination {
                address: self.sspdr.cast(),
                increment: false,
            },
            tx_count: 2 * WIDTH as u32 * HEIGHT as u32,
            tx_req: lax_dma::TxReq::Spi0Tx, // TODO: hardcoded
            byte_swap: false,
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
        let pwm = (crate::float::rom_instrinsics::powf32((value as f32) / 255.0f32, GAMMA)
            * 65535.0f32
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

    #[inline(always)]
    pub fn render_frame(&mut self, render_func: impl FnOnce(&mut Self)) {
        render_func(self);
        self.wait_for_vsync();
        self.write_framebuffer();
    }

    pub fn set_tearing_effect(&mut self, tearing_effect: TearingEffect) {
        self.tearing_effect = tearing_effect;
        match self.tearing_effect {
            TearingEffect::Off => self.write_command(Command::TEOFF),
            TearingEffect::Vertical => self.write_command_with_data(Command::TEON, &[0]),
            TearingEffect::HorizontalAndVertical => {
                self.write_command_with_data(Command::TEON, &[1])
            }
        };
    }

    #[inline(always)]
    fn wait_for_vsync(&mut self) {
        if self.tearing_effect == TearingEffect::Off {
            return;
        }

        while self.vsync_pin.is_high().unwrap() {}
        // while self.vsync_pin.is_low().unwrap() {}
        // self.last_vsync_time = crate::time::time_us();
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
