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
//! The code uses the `powf` function. It comes from the `libm` crate, that is as much accurate as
//! the hard float implementation of the `powf` function is, and is a dozen times faster than the
//! float functions in the Pico ROM. That is, at the expense of using a bit more flash memory.
//!
//! There is also the `micromath` crate that provides a `powf` function that is around 30% faster
//! than the `libm` crate, but it is less accurate. Again, that costs a bit more flash memory.
//!
//! Acuuracy is not a big concern here, as the `powf` function is used to compute the gamma
//! correction for the backlight PWM, and the difference in the gamma correction value is not
//! noticeable. Still, the `libm` crate is used here for the sake of accuracy.
//!
//! NOTE: Some places use `u16`. Do pay attention to the potential overflows. Running the debug
//! build is recommended.
//!
//#![no_std]

#![allow(dead_code)]

use crate::lax_dma;
use core::marker::PhantomData;
use core::usize;
use cortex_m::asm::delay;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::spi::SpiBus;
use fugit::RateExtU32;
use rp2040_hal::dma;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::dma::SingleChannel;
use rp2040_hal::gpio;
use rp2040_hal::gpio::bank0::*;
use rp2040_hal::gpio::FunctionSioInput;
use rp2040_hal::gpio::FunctionSioOutput;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PinId;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::gpio::*;
use rp2040_hal::pwm;
use rp2040_hal::spi;
use rp2040_hal::Clock;
use rp2040_hal::Spi;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[allow(dead_code)]
pub enum DisplayError {
    Spi,
    Dma,
    FramebufferSizeMismatch,
    BufferSizeMismatch,
    BufferRectSizeMismatch,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Display1_14;
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct DisplaySquare;
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Display2_0;
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Display2_8;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum DisplayKind {
    Display1_14,
    DisplaySquare,
    Display2_0,
    Display2_8,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum DisplayRotation {
    Rotate0,
    Rotate90,
    Rotate180,
    Rotate270,
}

pub trait DisplayAttributes {
    fn width() -> u16;
    fn height() -> u16;
    fn kind() -> DisplayKind;
    fn rotation() -> DisplayRotation;
}

impl DisplayAttributes for Display1_14 {
    fn width() -> u16 {
        240
    }

    fn height() -> u16 {
        135
    }

    fn kind() -> DisplayKind {
        DisplayKind::Display1_14
    }

    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for DisplaySquare {
    fn width() -> u16 {
        240
    }

    fn height() -> u16 {
        240
    }

    fn kind() -> DisplayKind {
        DisplayKind::DisplaySquare
    }

    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for Display2_0 {
    fn width() -> u16 {
        320
    }

    fn height() -> u16 {
        240
    }

    fn kind() -> DisplayKind {
        DisplayKind::Display2_0
    }

    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for Display2_8 {
    fn width() -> u16 {
        320
    }

    fn height() -> u16 {
        240
    }

    fn kind() -> DisplayKind {
        DisplayKind::Display2_8
    }

    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum TearingEffect {
    Off,
    Vertical,
    HorizontalAndVertical,
}

const MADCTL_ROW_ORDER: u8 = 0b10000000;
const MADCTL_COL_ORDER: u8 = 0b01000000;
const MADCTL_SWAP_XY: u8 = 0b00100000; // AKA "MV"
const MADCTL_SCAN_ORDER: u8 = 0b00010000;
const MADCTL_RGB_BGR: u8 = 0b00001000;
const MADCTL_HORIZ_ORDER: u8 = 0b00000100;

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

#[derive(Copy, Clone, Debug)]
pub enum MonochromeColor {
    Bpp1,
    Bpp2,
    Bpp4,
}

impl MonochromeColor {
    pub const fn bits_per_pixel(&self) -> u8 {
        match self {
            MonochromeColor::Bpp1 => 1,
            MonochromeColor::Bpp2 => 2,
            MonochromeColor::Bpp4 => 4,
        }
    }

    pub const fn pixel_per_byte(&self) -> u8 {
        8 / self.bits_per_pixel()
    }
}

pub struct MonochromeDisplayBuffer<const N: usize> {
    width: u16,
    height: u16,
    buffer: [u8; N],
    color: MonochromeColor,
}

impl<const N: usize> MonochromeDisplayBuffer<N> {
    pub const fn new(
        width: u16,
        height: u16,
        buffer: [u8; N],
        color: MonochromeColor,
    ) -> Result<Self, DisplayError> {
        if N != (width as usize * height as usize) / color.pixel_per_byte() as usize {
            Err(DisplayError::FramebufferSizeMismatch)
        } else {
            Ok(Self {
                width,
                height,
                buffer,
                color,
            })
        }
    }
}

/// A manual SPI implementation for the Pico Display
pub struct ManualDisplaySpi<MOSI, CLK, CS, DC>
where
    MOSI: PinId,
    CLK: PinId,
    CS: PinId,
    DC: PinId,
{
    mosi: Pin<MOSI, FunctionSioOutput, PullDown>,
    clk: Pin<CLK, FunctionSioOutput, PullDown>,
    cs: Pin<CS, FunctionSioOutput, PullDown>,
    dc: Pin<DC, FunctionSioOutput, PullDown>,
}

impl<MOSI, CLK, CS, DC> ManualDisplaySpi<MOSI, CLK, CS, DC>
where
    MOSI: PinId,
    CLK: PinId,
    CS: PinId,
    DC: PinId,
{
    pub fn new(
        mosi: Pin<MOSI, FunctionSioOutput, PullDown>,
        clk: Pin<CLK, FunctionSioOutput, PullDown>,
        cs: Pin<CS, FunctionSioOutput, PullDown>,
        dc: Pin<DC, FunctionSioOutput, PullDown>,
    ) -> Self {
        Self { mosi, clk, cs, dc }
    }

    #[inline(always)]
    fn spi_bit(&mut self, bit: bool) {
        const CYCLES_DELAY: u32 = 4;

        if bit {
            self.mosi.set_high().unwrap();
        } else {
            self.mosi.set_low().unwrap();
        }

        self.clk.set_high().unwrap();
        delay(CYCLES_DELAY);

        //let response = self.miso.as_mut().map(|miso| miso.is_high().unwrap());

        self.clk.set_low().unwrap();
        delay(CYCLES_DELAY);
    }

    #[inline(always)]
    fn spi_byte(&mut self, mut byte: u8) {
        for _ in 0..8 {
            self.spi_bit((byte & 0x80) != 0);
            byte <<= 1;
        }
    }

    fn write_byte(&mut self, val: u8) {
        self.cs.set_low().unwrap(); // Chip select active
        self.spi_byte(val);
        self.cs.set_high().unwrap(); // Chip select inactive
    }

    fn write_command(&mut self, cmd: Command) {
        self.dc.set_low().unwrap(); // Data/Command low for command
        self.write_byte(cmd as u8);
    }

    fn write_data(&mut self, val: &[u8]) {
        for byte in val {
            self.dc.set_high().unwrap(); // Data/Command high for data
            self.write_byte(*byte);
        }
    }

    fn write_command_with_data(&mut self, cmd: Command, val: &[u8]) {
        self.write_command(cmd);
        self.write_data(val);
    }

    fn release(
        self,
    ) -> (
        Pin<MOSI, FunctionSioOutput, PullDown>,
        Pin<CLK, FunctionSioOutput, PullDown>,
        Pin<CS, FunctionSioOutput, PullDown>,
        Pin<DC, FunctionSioOutput, PullDown>,
    ) {
        (self.mosi, self.clk, self.cs, self.dc)
    }
}

pub struct Display<TDispAttr>
where
    TDispAttr: DisplayAttributes,
{
    display_attr: PhantomData<TDispAttr>,

    red_led_pin: Pin<Gpio26, FunctionSioOutput, PullDown>,
    green_led_pin: Pin<Gpio27, FunctionSioOutput, PullDown>,
    blue_led_pin: Pin<Gpio28, FunctionSioOutput, PullDown>,

    backlight_pwm: pwm::Channel<pwm::Slice<pwm::Pwm2, pwm::FreeRunning>, pwm::A>,

    dc_pin: Pin<Gpio16, FunctionSioOutput, PullDown>,
    cs_pin: Pin<Gpio17, FunctionSioOutput, PullDown>,
    spi_device: Spi<
        spi::Enabled,
        rp2040_pac::SPI0,
        (
            Pin<Gpio19, FunctionSpi, PullDown>,
            Pin<Gpio18, FunctionSpi, PullDown>,
        ),
    >,

    dma0: u8,
    dma1: u8,

    vsync_pin: Pin<Gpio21, FunctionSioInput, PullNone>,

    width: u16,
    height: u16,
    pixel_count: u32,
    sspdr: *mut u32,
    tearing_effect: TearingEffect,
    last_vsync_time: u32,
}

pub type PicoDisplay2_8<'a> = Display<Display2_8>;
pub type PicoDisplay2_0<'a> = Display<Display2_0>;
pub type PicoDisplay1_14<'a> = Display<Display1_14>;
pub type PicoDisplaySquare<'a> = Display<DisplaySquare>;

impl<TDispAttr> Display<TDispAttr>
where
    TDispAttr: DisplayAttributes,
{
    pub fn new() -> Self {
        let display_kind = TDispAttr::kind();
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
        let sspdr = pac.SPI0.sspdr().as_ptr(); // For DMA, we need the address of the register

        let sio = rp2040_hal::sio::Sio::new(pac.SIO);
        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mut red_led_pin: Pin<Gpio26, FunctionSio<SioOutput>, PullDown> =
            pins.gpio26.into_push_pull_output();
        let mut green_led_pin: Pin<Gpio27, FunctionSio<SioOutput>, PullDown> =
            pins.gpio27.into_push_pull_output();
        let mut blue_led_pin: Pin<Gpio28, FunctionSio<SioOutput>, PullDown> =
            pins.gpio28.into_push_pull_output();

        red_led_pin.set_high().unwrap();
        green_led_pin.set_high().unwrap();
        blue_led_pin.set_high().unwrap();

        let backlight_pin = pins.gpio20.into_function::<gpio::FunctionPwm>();
        let dc_pin = pins.gpio16.into_push_pull_output();
        let cs_pin = pins.gpio17.into_push_pull_output();
        let sck_pin = pins.gpio18.into_push_pull_output();
        let mosi_pin = pins.gpio19.into_push_pull_output();
        let vsync_pin = pins.gpio21.into_floating_input();

        let dma = pac.DMA.split(&mut pac.RESETS);
        //lax_dma::tests::run_dma_tests();

        let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut backlight_pwm = pwm_slices.pwm2;
        backlight_pwm.set_ph_correct();
        backlight_pwm.enable();
        backlight_pwm.channel_a.output_to(backlight_pin);
        backlight_pwm.channel_a.set_duty_cycle(0).unwrap();
        let mut backlight_pwm = backlight_pwm.channel_a;
        backlight_pwm.set_duty_cycle_fully_off().unwrap();

        let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        let mut display = ManualDisplaySpi::new(mosi_pin, sck_pin, cs_pin, dc_pin);

        // Initialize display hardware

        {
            display.write_command(Command::SWRESET); // software reset

            delay.delay_ms(150);

            // 0x03: 12-bit/pixel RGB 4-4-4
            // 0x05: 16-bit/pixel RGB 5-6-5
            display.write_command_with_data(Command::COLMOD, &[0x03]);

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

            match display_kind {
                DisplayKind::DisplaySquare => {
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

                DisplayKind::Display2_0 | DisplayKind::Display2_8 => {
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

                DisplayKind::Display1_14 => {
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
            delay.delay_ms(10);
            display.write_command(Command::SLPOUT); // leave sleep mode
            delay.delay_ms(10);
            display.write_command(Command::NORON); // leave sleep mode
            delay.delay_ms(10);
            display.write_command(Command::DISPON); // turn display on
            delay.delay_ms(10);
        }

        // Configure display

        {
            let round = false;
            let rotation = TDispAttr::rotation();
            let (mut width, mut height) = (TDispAttr::width(), TDispAttr::height());

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

            display.write_command(Command::CASET);
            display.write_data(&caset[0].to_be_bytes());
            display.write_data(&caset[1].to_be_bytes());
            display.write_command(Command::RASET);
            display.write_data(&raset[0].to_be_bytes());
            display.write_data(&raset[1].to_be_bytes());

            display.write_command(Command::MADCTL);
            display.write_data(&[madctl]);
        }

        let (mosi_pin, sck_pin, cs_pin, dc_pin) = display.release();

        // Serious SPI speed

        let sck_pin = sck_pin.into_function::<gpio::FunctionSpi>();
        let mosi_pin = mosi_pin.into_function::<gpio::FunctionSpi>();

        let spi_device = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi_pin, sck_pin));
        let spi_device = spi_device.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            62_500_u32.kHz(),
            embedded_hal::spi::MODE_0,
        );

        let mut display = Display {
            display_attr: PhantomData,
            width: TDispAttr::width(),
            height: TDispAttr::height(),
            pixel_count: TDispAttr::width() as u32 * TDispAttr::height() as u32,
            sspdr,
            tearing_effect: TearingEffect::Off,
            last_vsync_time: 0,
            red_led_pin,
            green_led_pin,
            blue_led_pin,
            backlight_pwm,
            dc_pin,
            cs_pin,
            vsync_pin,
            spi_device,
            dma0: dma.ch0.id(),
            dma1: dma.ch1.id(),
        };

        display.set_tearing_effect(TearingEffect::HorizontalAndVertical);

        display.set_backlight(40);
        delay.delay_ms(10);

        display
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

        for byte in data {
            self.cs_pin.set_low().unwrap();
            self.spi_device.write(&[*byte]).unwrap(); // TODO: Handle error
            self.cs_pin.set_high().unwrap();
        }

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

    pub fn flush<'a>(&mut self, buffer: &'a mut [u16], sx: u16, sy: u16, ex: u16, ey: u16) {
        let tx_req = lax_dma::TxReq::Spi0Tx;
        let dma_config = lax_dma::Config {
            word_size: lax_dma::TxSize::_8bit,
            source: lax_dma::Source {
                address: buffer.as_ptr().cast(),
                increment: true,
            },
            destination: lax_dma::Destination {
                address: self.sspdr.cast(),
                increment: false,
            },
            tx_count: 2 * buffer.len() as u32,
            tx_req,
            byte_swap: false,
            start: true,
        };

        self.set_address_window(sx, sy, ex, ey);
        self.write_command(Command::RAMWR);

        self.dc_pin.set_high().unwrap();
        self.cs_pin.set_low().unwrap();

        let dma = lax_dma::LaxDmaWrite::new::<dma::CH0>(dma_config);
        //dma.trigger();
        dma.wait();

        self.cs_pin.set_high().unwrap();

        //defmt::info!("flush done, time: {:x}", time::time_us());
    }

    pub fn set_backlight(&mut self, value: u8) {
        const GAMMA: f32 = 2.8;
        let pwm = (libm::powf((value as f32) / 255.0f32, GAMMA) * 65535.0f32 + 0.5f32) as u16;

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
