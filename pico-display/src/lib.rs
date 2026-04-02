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
//! NOTE: embassy_rp::init() must be called before creating a display instance.

#![no_std]
#![allow(dead_code)]

pub mod lax_dma;

use crate::lax_dma::Config;
use crate::lax_dma::LaxDmaWrite;
use crate::lax_dma::TxReq;
use crate::lax_dma::TxSize;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_rp::pac;

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

    color_expand_sm_dma: LaxDmaWrite,
    display_sm_dma: LaxDmaWrite,
    /// SM1 TX FIFO register — used to write the pixel count before each frame.
    sm1_tx_fifo: pac::common::Reg<u32, pac::common::RW>,

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

// ── Low-level GPIO helpers (raw PAC) ────────────────────────────────────────

// Pin numbers for the SPI bus and peripherals.
const PIN_DC: u8 = 16;
const PIN_CS: u8 = 17;
const PIN_SCK: u8 = 18;
const PIN_MOSI: u8 = 19;
const PIN_BACKLIGHT: u8 = 20;
const PIN_VSYNC: u8 = 21;
const PIN_LED_R: u8 = 26;
const PIN_LED_G: u8 = 27;
const PIN_LED_B: u8 = 28;
const FUNCSEL_SIO: u8 = 5;
const FUNCSEL_PWM: u8 = 4;
const FUNCSEL_PIO0: u8 = 6;

/// Delay for approximately `ms` milliseconds (blocking, 125 MHz clock).
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(125_000 * ms);
}

/// Set GPIO pin function select.
fn set_pin_funcsel(pin: u8, funcsel: u8) {
    pac::IO_BANK0
        .gpio(pin as usize)
        .ctrl()
        .write(|w| w.set_funcsel(funcsel));
}

/// Initialize a GPIO pin as SIO output with initial value.
fn init_gpio_output(pin: u8, high: bool) {
    if high {
        pac::SIO.gpio_out(0).value_set().write_value(1 << pin);
    } else {
        pac::SIO.gpio_out(0).value_clr().write_value(1 << pin);
    }
    pac::SIO.gpio_oe(0).value_set().write_value(1 << pin);
    set_pin_funcsel(pin, FUNCSEL_SIO);
}

/// Initialize a GPIO pin as SIO input with pull-up.
fn init_gpio_input_pullup(pin: u8) {
    pac::SIO.gpio_oe(0).value_clr().write_value(1 << pin);
    pac::PADS_BANK0.gpio(pin as usize).modify(|w| {
        w.set_pue(true);
        w.set_pde(false);
    });
    set_pin_funcsel(pin, FUNCSEL_SIO);
}

/// Drive a GPIO pin via SIO.
fn sio_pin_set(pin: u8, high: bool) {
    if high {
        pac::SIO.gpio_out(0).value_set().write_value(1 << pin);
    } else {
        pac::SIO.gpio_out(0).value_clr().write_value(1 << pin);
    }
}

// ── ST7789 bit-bang SPI helpers ─────────────────────────────────────────────

/// Bit-bang one SPI byte (MSB first, mode 0) via SIO GPIO.
fn bb_spi_byte(mut byte: u8) {
    for _ in 0..8 {
        sio_pin_set(PIN_MOSI, (byte & 0x80) != 0);
        sio_pin_set(PIN_SCK, true);
        cortex_m::asm::delay(4);
        sio_pin_set(PIN_SCK, false);
        cortex_m::asm::delay(4);
        byte <<= 1;
    }
}

/// Send a command byte (DC low, CS low → byte → CS high).
fn bb_write_command(cmd: Command) {
    sio_pin_set(PIN_DC, false);
    sio_pin_set(PIN_CS, false);
    bb_spi_byte(cmd as u8);
    sio_pin_set(PIN_CS, true);
}

/// Send a data byte (DC high, CS low → byte → CS high).
fn bb_write_data_byte(data: u8) {
    sio_pin_set(PIN_DC, true);
    sio_pin_set(PIN_CS, false);
    bb_spi_byte(data);
    sio_pin_set(PIN_CS, true);
}

/// Send a slice of data bytes.
fn bb_write_data(val: &[u8]) {
    for &byte in val {
        bb_write_data_byte(byte);
    }
}

/// Send a command followed by data bytes.
fn bb_write_command_with_data(cmd: Command, val: &[u8]) {
    bb_write_command(cmd);
    bb_write_data(val);
}

/// Temporarily reclaim MOSI/SCK/CS from PIO so we can bit-bang
/// ST7789 commands. Must be called while PIO is idle (stalled on pull).
fn acquire_spi_pins() {
    // Pre-set SIO outputs to match PIO idle state (CS low, SCK low)
    sio_pin_set(PIN_CS, false);
    sio_pin_set(PIN_SCK, false);
    pac::SIO
        .gpio_oe(0)
        .value_set()
        .write_value((1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_CS));
    set_pin_funcsel(PIN_MOSI, FUNCSEL_SIO);
    set_pin_funcsel(PIN_SCK, FUNCSEL_SIO);
    set_pin_funcsel(PIN_CS, FUNCSEL_SIO);
}

/// Hand MOSI/SCK/CS back to PIO. Leaves DC high (data mode) and
/// CS low (selected) — ready for the PIO data transfer.
fn release_spi_pins() {
    sio_pin_set(PIN_DC, true); // data mode
    sio_pin_set(PIN_CS, false); // selected
    set_pin_funcsel(PIN_MOSI, FUNCSEL_PIO0);
    set_pin_funcsel(PIN_SCK, FUNCSEL_PIO0);
    set_pin_funcsel(PIN_CS, FUNCSEL_PIO0);
}

// ── PIO program loading helper ──────────────────────────────────────────────

/// Load a PIO program into instruction memory at the given offset.
/// JMP target addresses are relocated by `offset` so that programs
/// loaded after offset 0 jump to the correct instruction memory locations.
/// Returns the number of instructions written.
fn load_pio_program(program: &pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>, offset: usize) -> usize {
    let pio0 = pac::PIO0;
    for (i, &instr) in program.code.iter().enumerate() {
        let relocated = if offset > 0 && (instr >> 13) == 0b000 {
            // JMP instruction: relocate target address in bits [4:0]
            let target = (instr & 0x1F) + offset as u16;
            (instr & !0x1F) | (target & 0x1F)
        } else {
            instr
        };
        pio0.instr_mem(offset + i).write(|w| w.set_instr_mem(relocated));
    }
    program.code.len()
}

// ── Display implementation ──────────────────────────────────────────────────

/// Guard to prevent double-initialization.
static DISPLAY_INITIALIZED: AtomicBool = AtomicBool::new(false);

impl<TDispAttr, TColor> Display<TDispAttr, TColor>
where
    TDispAttr: DisplayAttributes,
    TColor: ColorMode,
{
    const FRAME_WORDS: usize = TColor::MODE.frame_words(TDispAttr::WIDTH, TDispAttr::HEIGHT);
    const NUM_BUFS: usize = if TColor::MODE.double_buffered_for(TDispAttr::WIDTH, TDispAttr::HEIGHT) { 2 } else { 1 };

    fn new(fb_data: &'static mut [u32]) -> Self {
        assert!(
            !DISPLAY_INITIALIZED.load(Ordering::SeqCst),
            "Display already initialized"
        );
        DISPLAY_INITIALIZED.store(true, Ordering::SeqCst);
        assert!(fb_data.len() >= Self::FRAME_WORDS * Self::NUM_BUFS);
        let display_kind = TDispAttr::kind();

        // Give more priority to the DMA peripheral
        pac::BUSCTRL.bus_priority().write(|w| {
            w.set_dma_r(true);
            w.set_dma_w(true);
        });

        // Ensure PIO0 and PWM are out of reset
        pac::RESETS.reset().modify(|w| {
            w.set_pio0(false);
            w.set_pwm(false);
        });
        while !pac::RESETS.reset_done().read().pio0() {}
        while !pac::RESETS.reset_done().read().pwm() {}

        // ── GPIO setup ──────────────────────────────────────────────────

        init_gpio_output(PIN_LED_R, true);  // LEDs off (active low)
        init_gpio_output(PIN_LED_G, true);
        init_gpio_output(PIN_LED_B, true);
        init_gpio_output(PIN_DC, false);    // command mode initially
        init_gpio_output(PIN_CS, true);     // deselected
        init_gpio_output(PIN_SCK, false);
        init_gpio_output(PIN_MOSI, false);
        init_gpio_input_pullup(PIN_VSYNC);

        // ── PWM backlight (slice 2, channel A, GPIO 20) ────────────────

        set_pin_funcsel(PIN_BACKLIGHT, FUNCSEL_PWM);
        let pwm_ch = pac::PWM.ch(2);
        pwm_ch.csr().write(|w| {
            w.set_ph_correct(true);
            w.set_en(true);
        });
        pwm_ch.cc().write(|w| w.set_a(0)); // backlight off initially

        // ── ST7789 initialization (bit-bang SPI) ────────────────────────

        bb_write_command(Command::SWRESET);
        delay_ms(150);

        bb_write_command_with_data(Command::COLMOD, &[TColor::MODE.colmod()]);
        bb_write_command_with_data(Command::PORCTRL, &[0x0c, 0x0c, 0x00, 0x33, 0x33]);
        bb_write_command_with_data(Command::LCMCTRL, &[0x2c]);
        bb_write_command_with_data(Command::VDVVRHEN, &[0x01]);
        bb_write_command_with_data(Command::VRHS, &[0x12]);
        bb_write_command_with_data(Command::VDVS, &[0x20]);
        bb_write_command_with_data(Command::PWCTRL1, &[0xa4, 0xa1]);
        bb_write_command_with_data(Command::FRCTRL2, &[0x1f]);

        match display_kind {
            DisplayKind::DisplaySquare => {
                bb_write_command_with_data(Command::GCTRL, &[0x14]);
                bb_write_command_with_data(Command::VCOMS, &[0x37]);
                bb_write_command_with_data(
                    Command::GMCTRP1,
                    &[0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23],
                );
                bb_write_command_with_data(
                    Command::GMCTRN1,
                    &[0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23],
                );
            }
            DisplayKind::Display2_0 | DisplayKind::Display2_8 => {
                bb_write_command_with_data(Command::GCTRL, &[0x35]);
                bb_write_command_with_data(Command::VCOMS, &[0x1f]);
                bb_write_command_with_data(
                    Command::GMCTRP1,
                    &[0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D],
                );
                bb_write_command_with_data(
                    Command::GMCTRN1,
                    &[0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31],
                );
            }
            DisplayKind::Display1_14 => {
                bb_write_command_with_data(Command::VRHS, &[0x00]);
                bb_write_command_with_data(Command::GCTRL, &[0x75]);
                bb_write_command_with_data(Command::VCOMS, &[0x3D]);
                bb_write_command_with_data(Command::_D6, &[0xa1]);
                bb_write_command_with_data(
                    Command::GMCTRP1,
                    &[0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2A, 0x33, 0x41, 0x07, 0x13, 0x13, 0x29, 0x2f],
                );
                bb_write_command_with_data(
                    Command::GMCTRN1,
                    &[0x70, 0x03, 0x09, 0x0A, 0x09, 0x06, 0x2B, 0x34, 0x41, 0x07, 0x12, 0x14, 0x28, 0x2E],
                );
            }
        }

        bb_write_command(Command::INVON);
        delay_ms(10);
        bb_write_command(Command::SLPOUT);
        delay_ms(10);
        bb_write_command(Command::NORON);
        delay_ms(10);
        bb_write_command(Command::DISPON);
        delay_ms(10);

        // Enable Tearing Effect output on the TE/VSYNC pin.
        // Without this, the ST7789 does not drive the TE pin and
        // wait_for_vsync() hangs because the pull-up holds GPIO high.
        // Parameter 0x00 = V-Blanking only (mode 0).
        bb_write_command_with_data(Command::TEON, &[0x00]);
        delay_ms(10);

        // ── Display configuration (CASET / RASET / MADCTL) ──────────────

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
                caset[0] = 40;
                caset[1] = 40 + width - 1;
                raset[0] = 52;
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
                caset[0] = 52;
                caset[1] = 52 + width - 1;
                raset[0] = 40;
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

            bb_write_command_with_data(
                Command::CASET,
                &[
                    (caset[0] >> 8) as u8, caset[0] as u8,
                    (caset[1] >> 8) as u8, caset[1] as u8,
                ],
            );
            bb_write_command_with_data(
                Command::RASET,
                &[
                    (raset[0] >> 8) as u8, raset[0] as u8,
                    (raset[1] >> 8) as u8, raset[1] as u8,
                ],
            );
            bb_write_command_with_data(Command::MADCTL, &[madctl]);
        }
        let col_offset = caset[0];
        let row_offset = raset[0];

        // Send RAMWR and leave DC high, CS low — ready to receive pixel data.
        bb_write_command(Command::RAMWR);
        sio_pin_set(PIN_DC, true);
        sio_pin_set(PIN_CS, false);

        // Zero-fill the caller-provided buffer.
        fb_data.fill(0);
        let fb = FrameBuffer {
            data: fb_data,
            draw_idx: 0,
        };

        // ── PIO setup (raw PAC) ─────────────────────────────────────────

        let pio0 = pac::PIO0;

        // SM0: monochrome color expansion (stalls harmlessly in direct modes).
        let mono_color = match TColor::MODE {
            DisplayMode::Mono(c) => c,
            DisplayMode::Rgb444 | DisplayMode::Rgb565 => MonochromeColor::Bpp1,
        };
        let sm0_prog = gen_monochrome_pio_program(mono_color);
        let sm0_len = load_pio_program(&sm0_prog, 0);

        // SM1: SPI output
        let sm1_prog = if TColor::MODE.is_rgb565() {
            rgb565_spi_program()
        } else {
            rgb444_pio_program()
        };
        let sm1_offset = sm0_len;
        load_pio_program(&sm1_prog, sm1_offset);

        // Configure SM0
        let sm0 = pio0.sm(0);
        sm0.clkdiv().write(|w| {
            w.set_int(1);
            w.set_frac(0);
        });
        sm0.shiftctrl().write(|w| {
            w.set_autopull(true);
            w.set_autopush(true);
            w.set_push_thresh(12);
            w.set_pull_thresh(0); // 0 = 32 bits
            w.set_in_shiftdir(false); // false = shift left
            w.set_out_shiftdir(true); // true = shift right (default)
        });
        sm0.execctrl().write(|w| {
            w.set_wrap_top(sm0_prog.wrap.source);
            w.set_wrap_bottom(sm0_prog.wrap.target);
        });

        // Configure SM1
        let sm1 = pio0.sm(1);
        sm1.clkdiv().write(|w| {
            w.set_int(1);
            w.set_frac(0);
        });

        // SM1 shift config
        sm1.shiftctrl().write(|w| {
            w.set_autopull(true);
            if TColor::MODE.is_rgb565() {
                w.set_pull_thresh(0); // 0 = 32 bits
                w.set_out_shiftdir(false); // left
            } else if TColor::MODE.is_rgb444() {
                w.set_pull_thresh(12);
                w.set_out_shiftdir(false); // left
            } else {
                // Mono: right shift matches SM0's in_shift Left
                w.set_pull_thresh(12);
                w.set_out_shiftdir(true); // right
            }
            w.set_fjoin_tx(true); // OnlyTx: double TX FIFO depth
        });

        // SM1 execution config
        let sideset_count = sm1_prog.side_set.bits() + sm1_prog.side_set.optional() as u8;
        sm1.execctrl().write(|w| {
            w.set_wrap_top(sm1_offset as u8 + sm1_prog.wrap.source);
            w.set_wrap_bottom(sm1_offset as u8 + sm1_prog.wrap.target);
            w.set_side_en(sm1_prog.side_set.optional());
            w.set_side_pindir(sm1_prog.side_set.pindirs());
        });

        // SM1 pin config
        sm1.pinctrl().write(|w| {
            w.set_out_base(PIN_MOSI);
            w.set_out_count(1);
            w.set_set_base(PIN_MOSI);
            w.set_set_count(1);
            w.set_sideset_base(PIN_CS);
            w.set_sideset_count(sideset_count);
        });

        // Switch SPI pins to PIO0 function
        set_pin_funcsel(PIN_MOSI, FUNCSEL_PIO0);
        set_pin_funcsel(PIN_SCK, FUNCSEL_PIO0);
        set_pin_funcsel(PIN_CS, FUNCSEL_PIO0);

        // Set pin directions for SM1: MOSI, SCK, CS as outputs.
        // Temporarily configure SET to cover all 3 pins for the PINDIRS instruction.
        sm1.pinctrl().modify(|w| {
            w.set_set_base(PIN_CS); // 17
            w.set_set_count(3);     // CS(17), SCK(18), MOSI(19)
        });
        // Execute: SET PINDIRS, 7 (all three pins = output)
        // Instruction encoding: 111_00000_100_00111 = 0xE087
        sm1.instr().write(|w| w.set_instr(0xE087));
        // Restore SET config
        sm1.pinctrl().modify(|w| {
            w.set_set_base(PIN_MOSI);
            w.set_set_count(1);
        });

        // Set initial PC for both SMs
        // Execute JMP to program start address
        sm0.instr().write(|w| w.set_instr(0x0000)); // JMP 0
        sm1.instr().write(|w| w.set_instr(sm1_offset as u16)); // JMP sm1_offset

        // Enable both state machines simultaneously
        pio0.ctrl().modify(|w| {
            w.set_sm_enable(w.sm_enable() | 0b11); // SM0 and SM1
        });

        // ── DMA setup ───────────────────────────────────────────────────

        // FIFO register addresses (as u32 for DMA source/destination config)
        let sm0_rx_fifo_addr = pio0.rxf(0).as_ptr() as u32;
        let sm0_tx_fifo_addr = pio0.txf(0).as_ptr() as u32;
        let sm1_tx_fifo_addr = pio0.txf(1).as_ptr() as u32;
        let sm1_tx_fifo = pio0.txf(1);

        // DMA CH2: SM0 RX FIFO → SM1 TX FIFO (monochrome pipeline).
        let pixel_count = TDispAttr::WIDTH as u32 * TDispAttr::HEIGHT as u32;
        let display_sm_dma = LaxDmaWrite::new(2, Config {
            high_priority: true,
            word_size: TxSize::_32bit,
            src_addr: sm0_rx_fifo_addr,
            src_incr: false,
            dest_addr: sm1_tx_fifo_addr,
            dest_incr: false,
            tx_count: pixel_count,
            tx_req: TxReq::Pio0Rx0,
            byte_swap: false,
            start: true,
        });

        // DMA CH1: RAM → PIO TX FIFO.
        // Direct modes (RGB565/RGB444): feeds SM1 TX, DREQ = Pio0Tx1.
        // Mono: feeds SM0 TX (color expansion), DREQ = Pio0Tx0.
        let (ch1_dest, ch1_dreq) = if TColor::MODE.is_direct() {
            (sm1_tx_fifo_addr, TxReq::Pio0Tx1)
        } else {
            (sm0_tx_fifo_addr, TxReq::Pio0Tx0)
        };
        let color_expand_sm_dma = LaxDmaWrite::new(1, Config {
            high_priority: true,
            word_size: TxSize::_32bit,
            src_addr: 0,
            src_incr: true,
            dest_addr: ch1_dest,
            dest_incr: false,
            tx_count: Self::FRAME_WORDS as u32,
            tx_req: ch1_dreq,
            byte_swap: false,
            start: false,
        });

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
            color_expand_sm_dma,
            display_sm_dma,
            sm1_tx_fifo,
        };

        display.set_backlight(40);
        delay_ms(10);

        display
    }

    /// Clear PIO0 IRQ flag 0 (frame-done).
    pub fn clear_frame_done_irq(&self) {
        pac::PIO0.irq().write(|w| w.set_irq(1 << 0));
    }

    /// Returns true if PIO0 IRQ flag 0 (frame-done) is set.
    pub fn is_frame_done(&self) -> bool {
        pac::PIO0.irq().read().irq() & (1 << 0) != 0
    }

    /// Submit the current draw buffer to the PIO/DMA pipeline and swap
    /// buffers so the CPU can immediately start drawing the next frame.
    pub fn flush(&mut self) {
        if self.frame_in_flight {
            while !self.is_frame_done() {}
            self.frame_in_flight = false;
        }

        self.wait_for_vsync();
        self.clear_frame_done_irq();

        self.sm1_tx_fifo.write_value(self.pixel_count - 1);

        self.color_expand_sm_dma
            .set_transfer_count(Self::FRAME_WORDS as u32);

        if Self::NUM_BUFS == 1 {
            self.color_expand_sm_dma.set_read_addr_trigger(self.fb.data.as_ptr() as u32);
            while !self.is_frame_done() {}
        } else {
            let ship_start = self.fb.draw_idx as usize * Self::FRAME_WORDS;
            self.display_sm_dma.restart(self.pixel_count);
            self.color_expand_sm_dma.set_read_addr_trigger(self.fb.data[ship_start..].as_ptr() as u32);
            self.fb.draw_idx ^= 1;
            self.frame_in_flight = true;
        }
    }

    /// Async version of [`flush`](Self::flush). Yields to the executor
    /// while waiting for the previous frame to finish and for VSYNC,
    /// allowing other tasks to run.
    pub async fn flush_async(&mut self) {
        use core::future::poll_fn;
        use core::task::Poll;

        if self.frame_in_flight {
            poll_fn(|cx| {
                if self.is_frame_done() {
                    Poll::Ready(())
                } else {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            })
            .await;
            self.frame_in_flight = false;
        }

        self.wait_for_vsync_async().await;
        self.clear_frame_done_irq();

        self.sm1_tx_fifo.write_value(self.pixel_count - 1);

        self.color_expand_sm_dma
            .set_transfer_count(Self::FRAME_WORDS as u32);

        if Self::NUM_BUFS == 1 {
            self.color_expand_sm_dma.set_read_addr_trigger(self.fb.data.as_ptr() as u32);
            poll_fn(|cx| {
                if self.is_frame_done() {
                    Poll::Ready(())
                } else {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            })
            .await;
        } else {
            let ship_start = self.fb.draw_idx as usize * Self::FRAME_WORDS;
            self.display_sm_dma.restart(self.pixel_count);
            self.color_expand_sm_dma.set_read_addr_trigger(self.fb.data[ship_start..].as_ptr() as u32);
            self.fb.draw_idx ^= 1;
            self.frame_in_flight = true;
        }
    }

    pub fn set_backlight(&mut self, value: u8) {
        let duty = (value as u32 * 65536 / 255) as u16;
        defmt::info!("Setting backlight to {}", duty);
        pac::PWM.ch(2).cc().modify(|w| w.set_a(duty));
    }

    pub fn full_backlight(&mut self) {
        defmt::info!("Enabling backlight fully");
        pac::PWM.ch(2).cc().modify(|w| w.set_a(u16::MAX));
    }

    pub fn no_backlight(&mut self) {
        defmt::info!("Disabling backlight");
        pac::PWM.ch(2).cc().modify(|w| w.set_a(0));
    }

    #[inline(always)]
    fn wait_for_vsync(&mut self) {
        while pac::SIO.gpio_in(0).read() & (1 << PIN_VSYNC) != 0 {}
        self.last_vsync_time = pac::TIMER.timerawl().read();
    }

    async fn wait_for_vsync_async(&mut self) {
        use core::future::poll_fn;
        use core::task::Poll;

        poll_fn(|cx| {
            if pac::SIO.gpio_in(0).read() & (1 << PIN_VSYNC) == 0 {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
        self.last_vsync_time = pac::TIMER.timerawl().read();
    }

    /// Async version of [`set_viewport`](Self::set_viewport). Yields to the
    /// executor while waiting for any in-flight frame to complete.
    pub async fn set_viewport_async(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        use core::future::poll_fn;
        use core::task::Poll;

        if self.frame_in_flight {
            poll_fn(|cx| {
                if self.is_frame_done() {
                    Poll::Ready(())
                } else {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            })
            .await;
            self.frame_in_flight = false;
        }

        self.set_viewport_inner(x0, y0, x1, y1);
    }

    /// Set the ST7789 address window for subsequent pixel writes.
    pub fn set_viewport(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        if self.frame_in_flight {
            while !self.is_frame_done() {}
            self.frame_in_flight = false;
        }

        self.set_viewport_inner(x0, y0, x1, y1);
    }

    fn set_viewport_inner(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let ax0 = self.col_offset + x0;
        let ax1 = self.col_offset + x1;
        let ay0 = self.row_offset + y0;
        let ay1 = self.row_offset + y1;

        acquire_spi_pins();

        bb_write_command(Command::CASET);
        bb_write_data_byte((ax0 >> 8) as u8);
        bb_write_data_byte(ax0 as u8);
        bb_write_data_byte((ax1 >> 8) as u8);
        bb_write_data_byte(ax1 as u8);

        bb_write_command(Command::RASET);
        bb_write_data_byte((ay0 >> 8) as u8);
        bb_write_data_byte(ay0 as u8);
        bb_write_data_byte((ay1 >> 8) as u8);
        bb_write_data_byte(ay1 as u8);

        bb_write_command(Command::RAMWR);
        release_spi_pins();
    }

    /// Reset the viewport to the full display area.
    pub fn reset_viewport(&mut self) {
        self.set_viewport(0, 0, self.width - 1, self.height - 1);
    }

    /// Blit pixel data into the current viewport via PIO/DMA.
    pub fn blit(&mut self, data: &[u32], pixel_count: u32) {
        self.start_blit(data, pixel_count);
        while !self.is_frame_done() {}
    }

    /// Async version of [`blit`](Self::blit). Yields to the executor
    /// while waiting for the transfer to complete.
    pub async fn blit_async(&mut self, data: &[u32], pixel_count: u32) {
        use core::future::poll_fn;
        use core::task::Poll;

        self.start_blit(data, pixel_count);
        poll_fn(|cx| {
            if self.is_frame_done() {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
    }

    fn start_blit(&mut self, data: &[u32], pixel_count: u32) {
        self.clear_frame_done_irq();

        self.sm1_tx_fifo.write_value(pixel_count - 1);

        if TColor::MODE.is_direct() {
            let word_count = pixel_count.div_ceil(2);
            self.color_expand_sm_dma.set_transfer_count(word_count);
            self.color_expand_sm_dma.set_read_addr_trigger(data.as_ptr() as u32);
        } else {
            let bpp = match TColor::MODE {
                DisplayMode::Mono(c) => c.bits_per_pixel() as u32,
                _ => unreachable!(),
            };
            let word_count = (pixel_count * bpp).div_ceil(32);
            self.display_sm_dma.restart(pixel_count);
            self.color_expand_sm_dma.set_transfer_count(word_count);
            self.color_expand_sm_dma.set_read_addr_trigger(data.as_ptr() as u32);
        }
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
    ///
    /// NOTE: `embassy_rp::init()` must be called before this.
    pub fn from_slice(fb_data: &'static mut [u32]) -> Self {
        Self(Display::<TDispAttr, TColor>::new(fb_data))
    }

    pub fn flush(&mut self) {
        self.0.flush();
    }

    pub async fn flush_async(&mut self) {
        self.0.flush_async().await;
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

    pub async fn set_viewport_async(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        self.0.set_viewport_async(x0, y0, x1, y1).await;
    }

    pub fn reset_viewport(&mut self) {
        self.0.reset_viewport();
    }

    pub fn blit(&mut self, data: &[u32], pixel_count: u32) {
        self.0.blit(data, pixel_count);
    }

    pub async fn blit_async(&mut self, data: &[u32], pixel_count: u32) {
        self.0.blit_async(data, pixel_count).await;
    }
}

/// Create a [`PicoDisplay`] with an internally-allocated, DMA-aligned
/// framebuffer. The buffer is placed in a `static` so no `unsafe` is
/// needed at the call site.
///
/// NOTE: `embassy_rp::init()` must be called before invoking this macro.
///
/// # Example
/// ```ignore
/// let _p = embassy_rp::init(Default::default());
/// type MyDisplay = PicoDisplay<Display2_8, Rgb565>;
/// let mut display = pico_display::pico_display_new!(MyDisplay);
/// ```
#[macro_export]
macro_rules! pico_display_new {
    ($ty:ty) => {{
        #[repr(C, align(512))]
        struct Buf([u32; <$ty>::BUF_WORDS]);
        static mut FB: Buf = Buf([0; <$ty>::BUF_WORDS]);
        // SAFETY: Display::new() panics on second call (AtomicBool guard),
        // so this static is only accessed once.
        let fb: &'static mut [u32] = unsafe { &mut *core::ptr::addr_of_mut!(FB.0) };
        <$ty>::from_slice(fb)
    }};
}
