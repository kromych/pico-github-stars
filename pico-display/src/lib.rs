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
#![no_std]

#![allow(dead_code)]

pub mod lax_dma;

use crate::lax_dma::Config;
use crate::lax_dma::Destination;
use crate::lax_dma::LaxDmaWrite;
use crate::lax_dma::Source;
use crate::lax_dma::TxReq;
use crate::lax_dma::TxSize;
use core::marker::PhantomData;
use cortex_m::asm::delay;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use rp2040_hal::dma;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio;
use rp2040_hal::gpio::bank0::*;
use rp2040_hal::gpio::FunctionSioInput;
use rp2040_hal::gpio::FunctionSioOutput;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::gpio::*;
use rp2040_hal::pio::PIOBuilder;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::pio::PinDir;
use rp2040_hal::pio::Running;
use rp2040_hal::pio::StateMachine;
use rp2040_hal::pio::SM0;
use rp2040_hal::pio::SM1;
use rp2040_hal::pll::PLLConfig;
use rp2040_hal::pwm;
use rp2040_hal::Clock;
use rp2040_pac::PIO0;

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
    const WIDTH: u16;
    const HEIGHT: u16;
    fn kind() -> DisplayKind;
    fn rotation() -> DisplayRotation;
}

impl DisplayAttributes for Display1_14 {
    const WIDTH: u16 = 240;
    const HEIGHT: u16 = 135;
    fn kind() -> DisplayKind {
        DisplayKind::Display1_14
    }
    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for DisplaySquare {
    const WIDTH: u16 = 240;
    const HEIGHT: u16 = 240;
    fn kind() -> DisplayKind {
        DisplayKind::DisplaySquare
    }
    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for Display2_0 {
    const WIDTH: u16 = 320;
    const HEIGHT: u16 = 240;
    fn kind() -> DisplayKind {
        DisplayKind::Display2_0
    }
    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

impl DisplayAttributes for Display2_8 {
    const WIDTH: u16 = 320;
    const HEIGHT: u16 = 240;
    fn kind() -> DisplayKind {
        DisplayKind::Display2_8
    }
    fn rotation() -> DisplayRotation {
        DisplayRotation::Rotate0
    }
}

// ── Color mode marker types ────────────────────────────────────────────────

pub use pio_programs::DisplayMode;
pub use pio_programs::MonochromeColor;

/// Trait implemented by color mode marker types.
/// The associated `MODE` constant selects the PIO pipeline and framebuffer
/// layout at compile time.
pub trait ColorMode {
    const MODE: DisplayMode;
}

/// 16-bit RGB (5-6-5) — highest color depth, single-buffered on 320×240.
pub struct Rgb565;
impl ColorMode for Rgb565 {
    const MODE: DisplayMode = DisplayMode::Rgb565;
}

/// 12-bit RGB (4-4-4) — faster PIO throughput than RGB565, single-buffered
/// on 320×240.
pub struct Rgb444;
impl ColorMode for Rgb444 {
    const MODE: DisplayMode = DisplayMode::Rgb444;
}

/// 1-bit monochrome — smallest buffer, double-buffered.
pub struct Bpp1;
impl ColorMode for Bpp1 {
    const MODE: DisplayMode = DisplayMode::Mono(MonochromeColor::Bpp1);
}

/// 2-bit greyscale (4 levels) — double-buffered on 320×240.
pub struct Bpp2;
impl ColorMode for Bpp2 {
    const MODE: DisplayMode = DisplayMode::Mono(MonochromeColor::Bpp2);
}

/// 4-bit greyscale (16 levels) — double-buffered on 320×240.
pub struct Bpp4;
impl ColorMode for Bpp4 {
    const MODE: DisplayMode = DisplayMode::Mono(MonochromeColor::Bpp4);
}

/// Compute the total framebuffer size in `u32` words for a given display
/// and color mode. Use this to size the static buffer passed to
/// [`PicoDisplay::from_slice`].
pub const fn buf_words(width: u16, height: u16, mode: DisplayMode) -> usize {
    let frame = mode.frame_words(width, height);
    let bufs = if mode.double_buffered_for(width, height) {
        2
    } else {
        1
    };
    frame * bufs
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

struct FrameBuffer {
    data: &'static mut [u32],
    /// Index of the buffer the CPU is currently drawing into (0 or 1).
    /// Always 0 for single-buffered modes.
    draw_idx: u8,
}

struct Display<TDispAttr, TColor>
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode,
{
    display_attr: PhantomData<TDispAttr>,
    color_mode: PhantomData<TColor>,
    fb: FrameBuffer,
    /// True after the first flush — subsequent flushes must wait for the
    /// previous frame's PIO/DMA pipeline to finish before swapping.
    frame_in_flight: bool,

    red_led_pin: Pin<Gpio26, FunctionSioOutput, PullDown>,
    green_led_pin: Pin<Gpio27, FunctionSioOutput, PullDown>,
    blue_led_pin: Pin<Gpio28, FunctionSioOutput, PullDown>,

    backlight_pwm: pwm::Channel<pwm::Slice<pwm::Pwm2, pwm::FreeRunning>, pwm::A>,

    dc_pin: Pin<Gpio16, FunctionSioOutput, PullDown>,
    cs_pin: Pin<Gpio17, FunctionPio0, PullDown>,
    sck_pin: Pin<Gpio18, FunctionPio0, PullDown>,
    mosi_pin: Pin<Gpio19, FunctionPio0, PullDown>,
    vsync_pin: Pin<Gpio21, FunctionSioInput, PullUp>,

    color_expand_sm_dma: LaxDmaWrite,
    display_sm_dma: LaxDmaWrite,
    dma_trig_addr: *mut u32,
    /// SM1 TX FIFO address — used to write the pixel count before each frame.
    display_spi_tx_fifo: *mut u32,
    color_expand_sm: StateMachine<(PIO0, SM0), Running>,
    display_spi_sm: StateMachine<(PIO0, SM1), Running>,

    width: u16,
    height: u16,
    pixel_count: u32,
    /// ST7789 column/row address offsets (from CASET/RASET during init).
    col_offset: u16,
    row_offset: u16,
    last_vsync_time: u32,
}

use pio_programs::gen_monochrome_pio_program;
use pio_programs::rgb444_pio_program;
use pio_programs::rgb565_spi_program;

fn _set_vreg_voltage(pac: &rp2040_pac::Peripherals) {
    /// Possible voltage values
    const VREG_VOLTAGE_0_85: u8 = 6; // 0.85V
    const VREG_VOLTAGE_0_90: u8 = 7; // 0.90V
    const VREG_VOLTAGE_0_95: u8 = 8; // 0.95V
    const VREG_VOLTAGE_1_00: u8 = 9; // 1.00V
    const VREG_VOLTAGE_1_05: u8 = 10; // 1.05V
    const VREG_VOLTAGE_1_10: u8 = 11; // 1.10V *default state
    const VREG_VOLTAGE_1_15: u8 = 12; // 1.15V
    const VREG_VOLTAGE_1_20: u8 = 13; // 1.20V
    const VREG_VOLTAGE_1_25: u8 = 14; // 1.25V
    const VREG_VOLTAGE_1_30: u8 = 15; // 1.30V
    const VREG_VOLTAGE_MIN: u8 = VREG_VOLTAGE_0_85; // minimum voltage
    const VREG_VOLTAGE_DEF: u8 = VREG_VOLTAGE_1_10; // default voltage after power up
    const VREG_VOLTAGE_MAX: u8 = VREG_VOLTAGE_1_30; // maximum voltage

    // A voltmod might be required for a stable 250MHz operation
    unsafe {
        pac.VREG_AND_CHIP_RESET
            .vreg()
            .modify(|_, w| w.vsel().bits(VREG_VOLTAGE_1_10));
    }
    // Delay for the voltage to stabilize
    cortex_m::asm::delay(10_000);
}

const _PLL_250MHZ: PLLConfig = pico_pll_config::pll_config!(250_000).unwrap();

impl<TDispAttr, TColor> Display<TDispAttr, TColor>
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode,
{
    const FRAME_WORDS: usize = TColor::MODE.frame_words(TDispAttr::WIDTH, TDispAttr::HEIGHT);
    const NUM_BUFS: usize = if TColor::MODE.double_buffered_for(TDispAttr::WIDTH, TDispAttr::HEIGHT) { 2 } else { 1 };

    fn new(fb_data: &'static mut [u32], xosc_crystal_freq: u32) -> Self {
        assert!(fb_data.len() >= Self::FRAME_WORDS * Self::NUM_BUFS);
        let display_kind = TDispAttr::kind();
        let mut pac = rp2040_pac::Peripherals::take().unwrap();
        let core = rp2040_pac::CorePeripherals::take().unwrap();

        // Give more priority to the DMA peripheral
        pac.BUSCTRL.bus_priority().write(|w| {
            w.dma_r().set_bit();
            w.dma_w().set_bit()
        });

        let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

        let clocks = rp2040_hal::clocks::init_clocks_and_plls(
            xosc_crystal_freq,
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

        let backlight_pin = pins.gpio20.into_function::<gpio::FunctionPwm>();
        let dc_pin = pins.gpio16.into_push_pull_output();
        let cs_pin = pins.gpio17.into_push_pull_output();
        let sck_pin = pins.gpio18.into_push_pull_output();
        let mosi_pin = pins.gpio19.into_push_pull_output();
        let vsync_pin = pins.gpio21.into_pull_up_input();

        // Reset the DMA peripheral and split it into channels
        let _dma = pac.DMA.split(&mut pac.RESETS);
        // Reset the PIO peripheral and split it into state machines
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut backlight_pwm = pwm_slices.pwm2;
        backlight_pwm.set_ph_correct();
        backlight_pwm.enable();
        backlight_pwm.channel_a.output_to(backlight_pin);
        backlight_pwm.channel_a.set_duty_cycle(0).unwrap();
        let mut backlight_pwm = backlight_pwm.channel_a;
        backlight_pwm.set_duty_cycle_fully_off().unwrap();

        let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        // Initialize display hardware (bit-bang SPI — pins are SIO GPIO outputs)

        {
            Self::bb_write_command(Command::SWRESET); // software reset

            delay.delay_ms(150);

            // 0x03: 12-bit/pixel RGB 4-4-4
            // 0x05: 16-bit/pixel RGB 5-6-5
            Self::bb_write_command_with_data(Command::COLMOD, &[TColor::MODE.colmod()]);

            Self::bb_write_command_with_data(Command::PORCTRL, &[0x0c, 0x0c, 0x00, 0x33, 0x33]);
            Self::bb_write_command_with_data(Command::LCMCTRL, &[0x2c]);
            Self::bb_write_command_with_data(Command::VDVVRHEN, &[0x01]);
            Self::bb_write_command_with_data(Command::VRHS, &[0x12]);
            Self::bb_write_command_with_data(Command::VDVS, &[0x20]);
            Self::bb_write_command_with_data(Command::PWCTRL1, &[0xa4, 0xa1]);
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
            Self::bb_write_command_with_data(Command::FRCTRL2, &[0x1f]);

            match display_kind {
                DisplayKind::DisplaySquare => {
                    Self::bb_write_command_with_data(Command::GCTRL, &[0x14]);
                    Self::bb_write_command_with_data(Command::VCOMS, &[0x37]);
                    Self::bb_write_command_with_data(
                        Command::GMCTRP1,
                        &[
                            0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B,
                            0x1F, 0x23,
                        ],
                    );
                    Self::bb_write_command_with_data(
                        Command::GMCTRN1,
                        &[
                            0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F,
                            0x20, 0x23,
                        ],
                    );
                }

                DisplayKind::Display2_0 | DisplayKind::Display2_8 => {
                    Self::bb_write_command_with_data(Command::GCTRL, &[0x35]);
                    Self::bb_write_command_with_data(Command::VCOMS, &[0x1f]);
                    Self::bb_write_command_with_data(
                        Command::GMCTRP1,
                        &[
                            0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14,
                            0x29, 0x2D,
                        ],
                    );
                    Self::bb_write_command_with_data(
                        Command::GMCTRN1,
                        &[
                            0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14,
                            0x2F, 0x31,
                        ],
                    );
                }

                DisplayKind::Display1_14 => {
                    Self::bb_write_command_with_data(Command::VRHS, &[0x00]); // VRH Voltage setting
                    Self::bb_write_command_with_data(Command::GCTRL, &[0x75]); // VGH and VGL voltages
                    Self::bb_write_command_with_data(Command::VCOMS, &[0x3D]); // VCOM voltage
                    Self::bb_write_command_with_data(Command::_D6, &[0xa1]); // ???
                    Self::bb_write_command_with_data(
                        Command::GMCTRP1,
                        &[
                            0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2A, 0x33, 0x41, 0x07, 0x13, 0x13,
                            0x29, 0x2f,
                        ],
                    );
                    Self::bb_write_command_with_data(
                        Command::GMCTRN1,
                        &[
                            0x70, 0x03, 0x09, 0x0A, 0x09, 0x06, 0x2B, 0x34, 0x41, 0x07, 0x12, 0x14,
                            0x28, 0x2E,
                        ],
                    );
                }
            }

            Self::bb_write_command(Command::INVON); // set inversion mode
            delay.delay_ms(10);
            Self::bb_write_command(Command::SLPOUT); // leave sleep mode
            delay.delay_ms(10);
            Self::bb_write_command(Command::NORON); // normal mode
            delay.delay_ms(10);
            Self::bb_write_command(Command::DISPON); // turn display on
            delay.delay_ms(10);
        }

        // Configure display

        let mut caset = [0u16; 2];
        let mut raset = [0u16; 2];
        {
            let round = false;
            let rotation = TDispAttr::rotation();
            let (mut width, mut height) = (TDispAttr::WIDTH, TDispAttr::HEIGHT);

            if rotation == DisplayRotation::Rotate90 || rotation == DisplayRotation::Rotate270 {
                core::mem::swap(&mut width, &mut height);
            }
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

            Self::bb_write_command_with_data(
                Command::CASET,
                &[
                    (caset[0] >> 8) as u8,
                    caset[0] as u8,
                    (caset[1] >> 8) as u8,
                    caset[1] as u8,
                ],
            );
            Self::bb_write_command_with_data(
                Command::RASET,
                &[
                    (raset[0] >> 8) as u8,
                    raset[0] as u8,
                    (raset[1] >> 8) as u8,
                    raset[1] as u8,
                ],
            );

            Self::bb_write_command_with_data(Command::MADCTL, &[madctl]);
        }
        let col_offset = caset[0];
        let row_offset = raset[0];

        // Send RAMWR and leave DC high, CS low — ready to receive pixel data.
        Self::bb_write_command(Command::RAMWR);
        Self::sio_pin_set(Self::PIN_DC, true);
        Self::sio_pin_set(Self::PIN_CS, false);

        // Zero-fill the caller-provided buffer.
        fb_data.fill(0);
        let fb = FrameBuffer {
            data: fb_data,
            draw_idx: 0,
        };

        // SM0: monochrome color expansion (stalls harmlessly in direct modes).
        let mono_color = match TColor::MODE {
            DisplayMode::Mono(c) => c,
            DisplayMode::Rgb444 | DisplayMode::Rgb565 => MonochromeColor::Bpp1, // dummy, SM0 never fed
        };
        let (color_expand_sm, color_expand_rx, color_expand_tx) =
            PIOBuilder::from_installed_program(
                pio.install(&gen_monochrome_pio_program(mono_color))
                    .unwrap(),
            )
            .autopull(true)
            .autopush(true)
            .push_threshold(12)
            .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);

        // Single-cycle I/O output pin
        let base_pin = Self::PIN_MOSI;
        let pin_count = 1;
        let mosi_pin = mosi_pin.into_function::<FunctionPio0>(); // Pin 19

        // Side-set pins for the PIO state machine
        let sck_pin = sck_pin.into_function::<FunctionPio0>(); // Pin 18
        let cs_pin = cs_pin.into_function::<FunctionPio0>(); // Pin 17

        // SM1: SPI output.
        //   RGB565: 16-bit program, left shift, autopull 32.
        //   RGB444 direct: 12-bit program, left shift, autopull 12.
        //   Mono: 12-bit program, right shift (matches SM0's in_shift Left), autopull 12.
        let (mut display_spi_sm, _display_spi_rx, display_spi_tx) = if TColor::MODE.is_rgb565() {
            PIOBuilder::from_installed_program(pio.install(&rgb565_spi_program()).unwrap())
                .autopull(true)
                .pull_threshold(32)
                .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(1, 0)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .set_pins(base_pin, pin_count)
                .out_pins(base_pin, pin_count)
                .side_set_pin_base(Self::PIN_CS)
                .build(sm1)
        } else if TColor::MODE.is_rgb444() {
            // RGB444 direct: DMA → SM1 (no SM0 expansion).
            // Left shift = MSB first on the wire, matching SPI convention.
            PIOBuilder::from_installed_program(pio.install(&rgb444_pio_program()).unwrap())
                .autopull(true)
                .pull_threshold(12)
                .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(1, 0)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .set_pins(base_pin, pin_count)
                .out_pins(base_pin, pin_count)
                .side_set_pin_base(Self::PIN_CS)
                .build(sm1)
        } else {
            // Mono: SM0 expands bpp→12-bit, pushes to SM1 via DMA CH2.
            // Right shift matches SM0's in_shift Left.
            PIOBuilder::from_installed_program(pio.install(&rgb444_pio_program()).unwrap())
                .autopull(true)
                .pull_threshold(12)
                .clock_divisor_fixed_point(1, 0)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .set_pins(base_pin, pin_count)
                .out_pins(base_pin, pin_count)
                .side_set_pin_base(Self::PIN_CS)
                .build(sm1)
        };
        display_spi_sm.set_pindirs([
            (Self::PIN_MOSI, PinDir::Output),
            (Self::PIN_SCK, PinDir::Output),
            (Self::PIN_CS, PinDir::Output),
        ]);

        let color_expand_sm = color_expand_sm.start();
        let display_spi_sm = display_spi_sm.start();

        // DMA CH2: SM0 RX FIFO → SM1 TX FIFO.
        //
        // With per-pixel autopush (threshold=12) and the IRQ 4 handshake,
        // each pixel is one 32-bit FIFO word. The transfer count equals the
        // pixel count for the frame.
        //
        // DREQ is Pio0Rx0 (SM0 RX has data). The IRQ handshake guarantees
        // SM0 never gets more than one pixel ahead of SM1, so SM1's TX FIFO
        // will always have space when DMA fires — preventing silent drops.
        //
        // Without the handshake, only Pio0Tx1 was safe (and only when SM0
        // was faster than SM1). Reading from an empty SM0 RX FIFO returns
        // undefined data; writing to a full SM1 TX FIFO silently drops the
        // word. The single-DREQ RP2040 DMA can only protect one side per
        // channel. The handshake makes both sides safe with either DREQ.
        let pixel_count = TDispAttr::WIDTH as u32 * TDispAttr::HEIGHT as u32;
        let display_sm_dma = LaxDmaWrite::new::<dma::CH2>(Config {
            high_priority: true,
            word_size: TxSize::_32bit,
            source: Source {
                address: color_expand_rx.fifo_address().cast(),
                increment: false,
            },
            destination: Destination {
                address: display_spi_tx.fifo_address().cast_mut().cast(),
                increment: false,
            },
            tx_count: pixel_count,
            tx_req: TxReq::Pio0Rx0,
            byte_swap: false,
            start: true,
        });

        // DMA CH1: RAM → PIO TX FIFO.
        // Direct modes (RGB565/RGB444): feeds SM1 TX, DREQ = Pio0Tx1.
        // Mono: feeds SM0 TX (color expansion), DREQ = Pio0Tx0.
        let (ch1_dest, ch1_dreq) = if TColor::MODE.is_direct() {
            (
                display_spi_tx.fifo_address().cast_mut().cast::<u8>(),
                TxReq::Pio0Tx1,
            )
        } else {
            (
                color_expand_tx.fifo_address().cast_mut().cast::<u8>(),
                TxReq::Pio0Tx0,
            )
        };
        let color_expand_sm_dma = LaxDmaWrite::new::<dma::CH1>(Config {
            high_priority: true,
            word_size: TxSize::_32bit,
            source: Source {
                address: core::ptr::null(),
                increment: true,
            },
            destination: Destination {
                address: ch1_dest,
                increment: false,
            },
            tx_count: Self::FRAME_WORDS as u32,
            tx_req: ch1_dreq,
            byte_swap: false,
            start: false,
        });
        let dma_trig_addr: *mut u32 = color_expand_sm_dma.read_trig_addr().cast_mut().cast();
        let display_spi_tx_fifo: *mut u32 = display_spi_tx.fifo_address().cast_mut().cast();

        let mut display = Display {
            fb,
            frame_in_flight: false,
            display_attr: PhantomData,
            color_mode: PhantomData,
            width: TDispAttr::WIDTH,
            height: TDispAttr::HEIGHT,
            pixel_count: TDispAttr::WIDTH as u32 * TDispAttr::HEIGHT as u32,
            col_offset,
            row_offset,
            last_vsync_time: 0,
            red_led_pin,
            green_led_pin,
            blue_led_pin,
            backlight_pwm,
            dc_pin,
            cs_pin,
            sck_pin,
            mosi_pin,
            vsync_pin,
            color_expand_sm_dma,
            display_sm_dma,
            dma_trig_addr,
            display_spi_tx_fifo,
            color_expand_sm,
            display_spi_sm,
        };

        display.set_backlight(40);
        delay.delay_ms(10);

        display
    }

    /// Clear PIO0 IRQ flag 0 (frame-done). Can also be called from a
    /// `PIO0_IRQ_0` interrupt handler if NVIC interrupts are enabled.
    pub fn clear_frame_done_irq(&self) {
        let pio0 = unsafe { &*rp2040_pac::PIO0::PTR };
        pio0.irq().write(|w| unsafe { w.irq().bits(1 << 0) });
    }

    /// Returns true if PIO0 IRQ flag 0 (frame-done) is set.
    pub fn is_frame_done(&self) -> bool {
        let pio0 = unsafe { &*rp2040_pac::PIO0::PTR };
        pio0.irq().read().irq().bits() & (1 << 0) != 0
    }

    /// Submit the current draw buffer to the PIO/DMA pipeline and swap
    /// buffers so the CPU can immediately start drawing the next frame.
    ///
    /// If a previous frame is still in flight, blocks until it finishes.
    /// Returns immediately after kicking off DMA — the CPU is free to
    /// draw into the new back buffer while the PIO ships the old one.
    pub fn flush(&mut self) {
        // Wait for the previous frame's pipeline to complete.
        if self.frame_in_flight {
            while !self.is_frame_done() {}
            self.frame_in_flight = false;
        }

        self.wait_for_vsync();
        self.clear_frame_done_irq();

        unsafe { *self.display_spi_tx_fifo = self.pixel_count - 1 };

        // Always set CH1 transfer count (blit() may have changed it).
        self.color_expand_sm_dma
            .set_transfer_count(Self::FRAME_WORDS as u32);

        if Self::NUM_BUFS == 1 {
            // Single buffer: trigger DMA and block until frame completes.
            unsafe { *self.dma_trig_addr = self.fb.data.as_ptr() as u32 };
            while !self.is_frame_done() {}
        } else {
            // Double buffer: kick off DMA and return immediately.
            let ship_start = self.fb.draw_idx as usize * Self::FRAME_WORDS;
            self.display_sm_dma.restart(self.pixel_count);
            unsafe { *self.dma_trig_addr = self.fb.data[ship_start..].as_ptr() as u32 };

            // Swap: CPU now draws into the other buffer.
            self.fb.draw_idx ^= 1;
            self.frame_in_flight = true;
        }
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

    #[inline(always)]
    fn wait_for_vsync(&mut self) {
        while self.vsync_pin.is_high().unwrap() {}
        // while self.vsync_pin.is_low().unwrap() {}
        self.last_vsync_time = unsafe { (*rp2040_pac::TIMER::PTR).timerawl().read().bits() };
    }

    // ── ST7789 viewport (CASET / RASET / RAMWR) ─────────────────────────

    // Pin numbers for the SPI bus.
    const PIN_DC: u8 = 16;
    const PIN_CS: u8 = 17;
    const PIN_SCK: u8 = 18;
    const PIN_MOSI: u8 = 19;
    const FUNCSEL_SIO: u8 = 5;
    const FUNCSEL_PIO0: u8 = 6;

    /// Switch a GPIO pin between peripherals (SIO=5, PIO0=6).
    fn set_pin_funcsel(pin: u8, funcsel: u8) {
        let io_bank0 = unsafe { &*rp2040_pac::IO_BANK0::PTR };
        io_bank0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(funcsel) });
    }

    /// Drive a GPIO pin via SIO.
    fn sio_pin_set(pin: u8, high: bool) {
        let sio = unsafe { &*rp2040_pac::SIO::PTR };
        if high {
            sio.gpio_out_set().write(|w| unsafe { w.bits(1 << pin) });
        } else {
            sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << pin) });
        }
    }

    /// Temporarily reclaim MOSI/SCK/CS from PIO so we can bit-bang
    /// ST7789 commands. Must be called while PIO is idle (stalled on pull).
    fn acquire_spi_pins(&self) {
        // Pre-set SIO outputs to match PIO idle state (CS low, SCK low)
        // to avoid glitches on the switchover.
        Self::sio_pin_set(Self::PIN_CS, false);
        Self::sio_pin_set(Self::PIN_SCK, false);
        let sio = unsafe { &*rp2040_pac::SIO::PTR };
        sio.gpio_oe_set().write(|w| unsafe {
            w.bits((1 << Self::PIN_MOSI) | (1 << Self::PIN_SCK) | (1 << Self::PIN_CS))
        });
        Self::set_pin_funcsel(Self::PIN_MOSI, Self::FUNCSEL_SIO);
        Self::set_pin_funcsel(Self::PIN_SCK, Self::FUNCSEL_SIO);
        Self::set_pin_funcsel(Self::PIN_CS, Self::FUNCSEL_SIO);
    }

    /// Hand MOSI/SCK/CS back to PIO. Leaves DC high (data mode) and
    /// CS low (selected) — ready for the PIO data transfer.
    fn release_spi_pins(&self) {
        Self::sio_pin_set(Self::PIN_DC, true); // data mode
        Self::sio_pin_set(Self::PIN_CS, false); // selected
        Self::set_pin_funcsel(Self::PIN_MOSI, Self::FUNCSEL_PIO0);
        Self::set_pin_funcsel(Self::PIN_SCK, Self::FUNCSEL_PIO0);
        Self::set_pin_funcsel(Self::PIN_CS, Self::FUNCSEL_PIO0);
    }

    /// Bit-bang one SPI byte (MSB first, mode 0) via SIO GPIO.
    fn bb_spi_byte(mut byte: u8) {
        for _ in 0..8 {
            Self::sio_pin_set(Self::PIN_MOSI, (byte & 0x80) != 0);
            Self::sio_pin_set(Self::PIN_SCK, true);
            delay(4);
            Self::sio_pin_set(Self::PIN_SCK, false);
            delay(4);
            byte <<= 1;
        }
    }

    /// Send a command byte (DC low, CS low → byte → CS high).
    fn bb_write_command(cmd: Command) {
        Self::sio_pin_set(Self::PIN_DC, false);
        Self::sio_pin_set(Self::PIN_CS, false);
        Self::bb_spi_byte(cmd as u8);
        Self::sio_pin_set(Self::PIN_CS, true);
    }

    /// Send a data byte (DC high, CS low → byte → CS high).
    fn bb_write_data_byte(data: u8) {
        Self::sio_pin_set(Self::PIN_DC, true);
        Self::sio_pin_set(Self::PIN_CS, false);
        Self::bb_spi_byte(data);
        Self::sio_pin_set(Self::PIN_CS, true);
    }

    /// Send a slice of data bytes.
    fn bb_write_data(val: &[u8]) {
        for &byte in val {
            Self::bb_write_data_byte(byte);
        }
    }

    /// Send a command followed by data bytes.
    fn bb_write_command_with_data(cmd: Command, val: &[u8]) {
        Self::bb_write_command(cmd);
        Self::bb_write_data(val);
    }

    /// Set the ST7789 address window for subsequent pixel writes.
    ///
    /// Coordinates are in display-local space (0-based, inclusive):
    /// the rectangle covers columns `x0..=x1` and rows `y0..=y1`.
    ///
    /// After calling this, use [`blit`] to send pixel data for the
    /// region, or call [`reset_viewport`] before the next [`flush`].
    pub fn set_viewport(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        // Ensure no transfer is in progress.
        if self.frame_in_flight {
            while !self.is_frame_done() {}
            self.frame_in_flight = false;
        }

        let ax0 = self.col_offset + x0;
        let ax1 = self.col_offset + x1;
        let ay0 = self.row_offset + y0;
        let ay1 = self.row_offset + y1;

        self.acquire_spi_pins();

        Self::bb_write_command(Command::CASET);
        Self::bb_write_data_byte((ax0 >> 8) as u8);
        Self::bb_write_data_byte(ax0 as u8);
        Self::bb_write_data_byte((ax1 >> 8) as u8);
        Self::bb_write_data_byte(ax1 as u8);

        Self::bb_write_command(Command::RASET);
        Self::bb_write_data_byte((ay0 >> 8) as u8);
        Self::bb_write_data_byte(ay0 as u8);
        Self::bb_write_data_byte((ay1 >> 8) as u8);
        Self::bb_write_data_byte(ay1 as u8);

        Self::bb_write_command(Command::RAMWR);

        self.release_spi_pins();
    }

    /// Reset the viewport to the full display area.
    pub fn reset_viewport(&mut self) {
        self.set_viewport(0, 0, self.width - 1, self.height - 1);
    }

    /// Blit pixel data into the current viewport via PIO/DMA.
    ///
    /// `pixel_count` is the number of pixels to send (must match the
    /// viewport dimensions: `(x1-x0+1) * (y1-y0+1)`).
    ///
    /// `data` must hold enough u32 words for the pixels (packing depends
    /// on the display mode — bpp-packed for mono, two pixels per word for
    /// RGB565).
    ///
    /// Always blocks until the transfer is complete.
    pub fn blit(&mut self, data: &[u32], pixel_count: u32) {
        self.clear_frame_done_irq();

        unsafe { *self.display_spi_tx_fifo = pixel_count - 1 };

        if TColor::MODE.is_direct() {
            // RGB565 or RGB444: 2 pixels per word, DMA → SM1 directly.
            let word_count = pixel_count.div_ceil(2);
            self.color_expand_sm_dma.set_transfer_count(word_count);
            unsafe { *self.dma_trig_addr = data.as_ptr() as u32 };
        } else {
            // Mono: bpp-packed words → SM0 (expand) → SM1 (SPI).
            let bpp = match TColor::MODE {
                DisplayMode::Mono(c) => c.bits_per_pixel() as u32,
                _ => unreachable!(),
            };
            let word_count = (pixel_count * bpp).div_ceil(32);
            self.display_sm_dma.restart(pixel_count);
            self.color_expand_sm_dma.set_transfer_count(word_count);
            unsafe { *self.dma_trig_addr = data.as_ptr() as u32 };
        }

        while !self.is_frame_done() {}
    }
}

impl<TDispAttr, TColor> Display<TDispAttr, TColor>
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode,
{
    /// Return a mutable reference to the current draw buffer.
    #[inline(always)]
    fn draw_buf(&mut self) -> &mut [u32] {
        let start = self.fb.draw_idx as usize * Self::FRAME_WORDS;
        &mut self.fb.data[start..start + Self::FRAME_WORDS]
    }

    /// Set a pixel to a multi-bit grey value (monochrome modes only).
    /// For Bpp1 max=1, Bpp2 max=3, Bpp4 max=15.
    #[inline(always)]
    pub fn set_pixel_value(&mut self, x: u16, y: u16, value: u8) {
        let bpp = match TColor::MODE {
            DisplayMode::Mono(c) => c.bits_per_pixel() as usize,
            _ => unreachable!(),
        };
        let bit_index = (y as usize * self.width as usize + x as usize) * bpp;
        let word = bit_index / 32;
        let bit = bit_index % 32;
        let mask = ((1u32 << bpp) - 1) << bit;
        let val = (value as u32 & ((1u32 << bpp) - 1)) << bit;
        let buf = self.draw_buf();
        buf[word] = (buf[word] & !mask) | val;
    }

    /// Set a pixel to a native RGB565 value (RGB565 mode only).
    ///
    /// Even pixels occupy bits [31:16] of each u32 word (shifted out first
    /// by the shift-left OSR); odd pixels occupy bits [15:0].
    #[inline(always)]
    pub fn set_pixel_rgb565(&mut self, x: u16, y: u16, rgb565: u16) {
        let pixel_idx = y as usize * self.width as usize + x as usize;
        let word_idx = pixel_idx / 2;
        let buf = self.draw_buf();
        if pixel_idx & 1 == 0 {
            buf[word_idx] = (buf[word_idx] & 0x0000_FFFF) | ((rgb565 as u32) << 16);
        } else {
            buf[word_idx] = (buf[word_idx] & 0xFFFF_0000) | (rgb565 as u32);
        }
    }

    /// Set a pixel to a native RGB444 value (RGB444 mode only).
    ///
    /// With left-shift OSR: even pixels occupy bits [31:20], odd pixels
    /// bits [19:8]. Bits [7:0] are discarded by autopull.
    #[inline(always)]
    pub fn set_pixel_rgb444(&mut self, x: u16, y: u16, rgb444: u16) {
        let pixel_idx = y as usize * self.width as usize + x as usize;
        let word_idx = pixel_idx / 2;
        let buf = self.draw_buf();
        if pixel_idx & 1 == 0 {
            buf[word_idx] = (buf[word_idx] & 0x000F_FFFF) | ((rgb444 as u32 & 0xFFF) << 20);
        } else {
            buf[word_idx] = (buf[word_idx] & 0xFFF0_00FF) | ((rgb444 as u32 & 0xFFF) << 8);
        }
    }

    /// Convenience: set a pixel on (max brightness) or off (black).
    #[inline(always)]
    pub fn set_pixel(&mut self, x: u16, y: u16, on: bool) {
        match TColor::MODE {
            DisplayMode::Mono(c) => {
                let max = (1u8 << c.bits_per_pixel()) - 1;
                self.set_pixel_value(x, y, if on { max } else { 0 });
            }
            DisplayMode::Rgb444 => {
                self.set_pixel_rgb444(x, y, if on { 0xFFF } else { 0 });
            }
            DisplayMode::Rgb565 => {
                self.set_pixel_rgb565(x, y, if on { 0xFFFF } else { 0 });
            }
        }
    }

    pub fn clear(&mut self) {
        self.draw_buf().fill(0);
    }

    pub fn width(&self) -> u16 {
        self.width
    }

    pub fn height(&self) -> u16 {
        self.height
    }
}

/// DMA-aligned framebuffer storage.
#[repr(C, align(512))]
pub struct AlignedBuf<const N: usize>(pub [u32; N]);

/// Standard crystal frequency for Raspberry Pi Pico boards (12 MHz).
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub struct PicoDisplay<TDispAttr, TColor>(Display<TDispAttr, TColor>)
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode;

impl<TDispAttr, TColor> PicoDisplay<TDispAttr, TColor>
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode,
{
    /// Total framebuffer size in `u32` words.
    pub const BUF_WORDS: usize = buf_words(TDispAttr::WIDTH, TDispAttr::HEIGHT, TColor::MODE);

    /// The display mode for this configuration.
    pub const MODE: DisplayMode = TColor::MODE;

    /// Create from a pre-allocated `&'static mut [u32]` slice.
    /// Prefer the [`pico_display_new!`] macro which handles buffer
    /// allocation automatically.
    pub fn from_slice(fb_data: &'static mut [u32]) -> Self {
        Self(Display::<TDispAttr, TColor>::new(fb_data, XOSC_CRYSTAL_FREQ))
    }

    pub fn flush(&mut self) {
        self.0.flush();
    }

    #[inline(always)]
    pub fn set_pixel_value(&mut self, x: u16, y: u16, value: u8) {
        self.0.set_pixel_value(x, y, value);
    }

    #[inline(always)]
    pub fn set_pixel_rgb565(&mut self, x: u16, y: u16, rgb565: u16) {
        self.0.set_pixel_rgb565(x, y, rgb565);
    }

    #[inline(always)]
    pub fn set_pixel_rgb444(&mut self, x: u16, y: u16, rgb444: u16) {
        self.0.set_pixel_rgb444(x, y, rgb444);
    }

    #[inline(always)]
    pub fn set_pixel(&mut self, x: u16, y: u16, on: bool) {
        self.0.set_pixel(x, y, on);
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn width(&self) -> u16 {
        self.0.width()
    }

    pub fn height(&self) -> u16 {
        self.0.height()
    }

    pub fn set_backlight(&mut self, value: u8) {
        self.0.set_backlight(value);
    }

    pub fn set_viewport(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        self.0.set_viewport(x0, y0, x1, y1);
    }

    pub fn reset_viewport(&mut self) {
        self.0.reset_viewport();
    }

    pub fn blit(&mut self, data: &[u32], pixel_count: u32) {
        self.0.blit(data, pixel_count);
    }
}

/// Create a [`PicoDisplay`] with an internally-allocated, DMA-aligned
/// framebuffer. The buffer is placed in a `static` so no `unsafe` is
/// needed at the call site.
///
/// # Example
/// ```ignore
/// type MyDisplay = PicoDisplay<Display2_8, Rgb565>;
/// let mut display = pico_display::pico_display_new!(MyDisplay);
/// ```
#[macro_export]
macro_rules! pico_display_new {
    ($ty:ty) => {{
        #[repr(C, align(512))]
        struct Buf([u32; <$ty>::BUF_WORDS]);
        static mut FB: Buf = Buf([0; <$ty>::BUF_WORDS]);
        // SAFETY: this path can only execute once because `Display::new`
        // calls `Peripherals::take()` which panics on a second call.
        let fb: &'static mut [u32] = unsafe { &mut *core::ptr::addr_of_mut!(FB.0) };
        <$ty>::from_slice(fb)
    }};
}
