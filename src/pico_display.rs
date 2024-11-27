//! Pico Display driver
//!
//! Display variants supported:
//!
//! * Pico Display Pack: https://shop.pimoroni.com/products/pico-display-pack
//! * Pimoroni Pico Display 2.0: https://shop.pimoroni.com/products/pico-display-pack-2-0
//! * Pimoroni Pico Display 2.8: https://shop.pimoroni.com/products/pico-display-pack-2-8
//!
//! Mainly ported from the Pimoroni C++ library:
//!
//! https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp
//!
//#![no_std]

use crate::float;
use bitfield_struct::bitfield;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_rp::gpio::Output;
use embassy_rp::pwm::Pwm;
use embassy_rp::spi;
use embassy_rp::spi::Blocking;
use embassy_rp::spi::Spi;
use embassy_rp::Peripheral;

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

fn get_display_dimensions(kind: DisplayKind) -> (u16, u16) {
    match kind {
        DisplayKind::PicoDisplay1_14 => (240, 135),
        DisplayKind::PicoDisplaySquare => (240, 240),
        DisplayKind::PicoDisplay2_0 => (320, 240),
        DisplayKind::PicoDisplay2_8 => (320, 240),
    }
}

pub fn delay_ms(ms: u32) {
    let mut delay = embassy_time::Delay;
    delay.delay_ms(ms);
}

pub struct PicoDisplay<T: spi::Instance + 'static> {
    kind: DisplayKind,
    rotation: DisplayRotation,
    frame_size: u32,
    spi: Spi<'static, T, Blocking>,
    dc: Output<'static>,
    cs: Output<'static>,
    pwm: Pwm<'static>,
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

const MADCTL_ROW_ORDER: u8 = 0b10000000;
const MADCTL_COL_ORDER: u8 = 0b01000000;
const MADCTL_SWAP_XY: u8 = 0b00100000; // AKA "MV"
const MADCTL_SCAN_ORDER: u8 = 0b00010000;
const _MADCTL_RGB_BGR: u8 = 0b00001000;
const MADCTL_HORIZ_ORDER: u8 = 0b00000100;

#[bitfield(u16)]
pub struct RGB565 {
    #[bits(5)]
    r: u16,
    #[bits(6)]
    g: u16,
    #[bits(5)]
    b: u16,
}

impl RGB565 {
    pub fn white() -> Self {
        Self::new().with_r(0x1F).with_g(0x3F).with_b(0x1F)
    }

    pub fn black() -> Self {
        Self::new().with_r(0x00).with_g(0x00).with_b(0x00)
    }

    pub fn red() -> Self {
        Self::new().with_r(0x1F).with_g(0x00).with_b(0x00)
    }

    pub fn green() -> Self {
        Self::new().with_r(0x00).with_g(0x3F).with_b(0x00)
    }

    pub fn blue() -> Self {
        Self::new().with_r(0x00).with_g(0x00).with_b(0x1F)
    }

    pub fn from_rgb(r: u8, g: u8, b: u8) -> Self {
        Self::new()
            .with_r((r >> 3) as u16)
            .with_g((g >> 2) as u16)
            .with_b((b >> 3) as u16)
    }
}

impl<T: spi::Instance> PicoDisplay<T> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        kind: DisplayKind,
        rotation: DisplayRotation,
        spi: impl Peripheral<P = T> + 'static,
        clk: impl Peripheral<P = impl spi::ClkPin<T>> + 'static,
        mosi: impl Peripheral<P = impl spi::MosiPin<T>> + 'static,
        dc: Output<'static>,
        cs: Output<'static>,
        pwm: Pwm<'static>,
    ) -> Self {
        let mut spi_config = spi::Config::default();
        // The ST7789 requires 16 ns between SPI rising edges.
        // 16 ns = 62,500,000 Hz
        spi_config.frequency = 62_500_000;
        spi_config.polarity = spi::Polarity::IdleHigh;
        spi_config.phase = spi::Phase::CaptureOnSecondTransition;
        let spi: Spi<'static, T, Blocking> = Spi::new_blocking_txonly(spi, clk, mosi, spi_config);

        // Assuming 16-bit color depth (RGB565)
        let frame_size =
            get_display_dimensions(kind).0 as u32 * get_display_dimensions(kind).1 as u32 * 2;
        let mut display = Self {
            kind,
            rotation,
            frame_size,
            spi,
            dc,
            cs,
            pwm,
        };

        display.set_backlight(0); // Turn backlight off to avoid surprises
        display.init_display();
        delay_ms(100);
        display.configure_display();
        delay_ms(50); // Wait for the update to apply
        display.set_backlight(175); // Turn backlight on now surprises have passed

        display
    }

    fn command(&mut self, command: Command, data: &[u8]) {
        self.dc.set_low();

        self.cs.set_low();
        self.spi.blocking_write(&[command as u8]).unwrap();
        self.cs.set_high();

        if !data.is_empty() {
            self.dc.set_high();
            self.cs.set_low();
            self.spi.blocking_write(data).unwrap();
            self.cs.set_high();

            if data.len() < 16 {
                defmt::info!("Command 0x{:x}, data: 0x{:x}", command as u8, data);
            } else {
                defmt::info!("Command 0x{:x}, {} data bytes", command as u8, data.len());
            }
        } else {
            defmt::info!("Command {:x}", command as u8);
        }
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
        // Pico Display 2.0
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

        self.command(
            Command::CASET,
            &[
                (caset[0] >> 8) as u8,
                caset[0] as u8,
                (caset[1] >> 8) as u8,
                caset[1] as u8,
            ],
        );
        self.command(
            Command::RASET,
            &[
                (raset[0] >> 8) as u8,
                raset[0] as u8,
                (raset[1] >> 8) as u8,
                raset[1] as u8,
            ],
        );
        self.command(Command::MADCTL, &[madctl]);
    }

    fn init_display(&mut self) {
        self.command(Command::SWRESET, &[]);

        delay_ms(150);

        self.command(Command::TEON, &[]); // enable frame sync signal if used
        self.command(Command::COLMOD, &[0x05]); // 16 bits per pixel, RGB565

        self.command(Command::PORCTRL, &[0x0c, 0x0c, 0x00, 0x33, 0x33]);
        self.command(Command::LCMCTRL, &[0x2c]);
        self.command(Command::VDVVRHEN, &[0x01]);
        self.command(Command::VRHS, &[0x12]);
        self.command(Command::VDVS, &[0x20]);
        self.command(Command::PWCTRL1, &[0xa4, 0xa1]);
        self.command(Command::FRCTRL2, &[0x0f]);

        match self.kind {
            DisplayKind::PicoDisplaySquare => {
                self.command(Command::GCTRL, &[0x14]);
                self.command(Command::VCOMS, &[0x37]);
                self.command(
                    Command::GMCTRP1,
                    &[
                        0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B,
                        0x1F, 0x23,
                    ],
                );
                self.command(
                    Command::GMCTRN1,
                    &[
                        0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F,
                        0x20, 0x23,
                    ],
                );
            }

            DisplayKind::PicoDisplay2_0 | DisplayKind::PicoDisplay2_8 => {
                self.command(Command::GCTRL, &[0x35]);
                self.command(Command::VCOMS, &[0x1f]);
                self.command(
                    Command::GMCTRP1,
                    &[
                        0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14,
                        0x29, 0x2D,
                    ],
                );
                self.command(
                    Command::GMCTRN1,
                    &[
                        0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14,
                        0x2F, 0x31,
                    ],
                );
            }

            DisplayKind::PicoDisplay1_14 => {
                self.command(Command::VRHS, &[0x00]); // VRH Voltage setting
                self.command(Command::GCTRL, &[0x75]); // VGH and VGL voltages
                self.command(Command::VCOMS, &[0x3D]); // VCOM voltage
                self.command(Command::_D6, &[0xa1]); // ???
                self.command(
                    Command::GMCTRP1,
                    &[
                        0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2A, 0x33, 0x41, 0x07, 0x13, 0x13,
                        0x29, 0x2f,
                    ],
                );
                self.command(
                    Command::GMCTRN1,
                    &[
                        0x70, 0x03, 0x09, 0x0A, 0x09, 0x06, 0x2B, 0x34, 0x41, 0x07, 0x12, 0x14,
                        0x28, 0x2E,
                    ],
                );
            }
        }

        self.command(Command::INVON, &[]); // set inversion mode
        self.command(Command::SLPOUT, &[]); // leave sleep mode
        self.command(Command::DISPON, &[]); // turn display on
    }

    pub fn set_backlight(&mut self, value: u8) {
        let gamma = 2.8;
        let value = (value as f32) / 255.0f32;
        let value = float::powf32(value, gamma);
        let value = (value * 65535.0f32 + 0.5f32) as u16;
        defmt::info!("Setting backlight to {}", value);

        let mut pwm_config = embassy_rp::pwm::Config::default();
        pwm_config.compare_a = value;
        self.pwm.set_config(&pwm_config);
    }

    fn write_frame(&mut self, frame: &[RGB565]) {
        if frame.len() as u32 != self.frame_size / 2 {
            defmt::warn!("Frame size mismatch");
            return;
        }

        // Represnt as bytes and write to display
        let bytes = unsafe {
            core::slice::from_raw_parts(frame.as_ptr() as *const u8, self.frame_size as usize)
        };
        defmt::info!("Writing frame, size: {}", bytes.len());
        self.command(Command::RAMWR, bytes);
    }

    pub fn clear(&mut self, color: RGB565) {
        let frame = [color; 320 * 240];
        self.write_frame(&frame);
    }
}
