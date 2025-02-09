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
use rp2040_hal::gpio::PinId;
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
#[repr(u8)]
pub enum MonochromeColor {
    Bpp1 = 1,
    Bpp2 = 2,
    Bpp4 = 4,
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

#[repr(align(512))]
struct MonochromeDisplayBuffer<const N: usize> {
    buffer: [u32; N],
    width: u16,
    height: u16,
    color: MonochromeColor,
}

type DisplayPins<MOSI, CLK, CS, DC> = (
    Pin<MOSI, FunctionSioOutput, PullDown>,
    Pin<CLK, FunctionSioOutput, PullDown>,
    Pin<CS, FunctionSioOutput, PullDown>,
    Pin<DC, FunctionSioOutput, PullDown>,
);

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
    pub fn new(pins: DisplayPins<MOSI, CLK, CS, DC>) -> Self {
        let (mosi, clk, cs, dc) = pins;
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

    /// Leaves the pins in the state that is suitable for sending data
    fn release(mut self) -> DisplayPins<MOSI, CLK, CS, DC> {
        self.dc.set_high().unwrap(); // Data/Command high for data
        self.cs.set_low().unwrap(); // Chip select active

        (self.mosi, self.clk, self.cs, self.dc)
    }
}

struct Display<TDispAttr, const N: usize>
where
    TDispAttr: DisplayAttributes,
{
    display_attr: PhantomData<TDispAttr>,
    display_buffer: MonochromeDisplayBuffer<N>,

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
    // Using different PIO blocks to not share the bandwidth
    color_expand_sm: StateMachine<(PIO0, SM0), Running>,
    display_spi_sm: StateMachine<(PIO0, SM1), Running>,

    width: u16,
    height: u16,
    pixel_count: u32,
    last_vsync_time: u32,
}

/// Generates a PIO program to produce greyscale color encoded as RGB444
/// physically. Each pixel may have 2, 4, or 16 greyscale levels (1, 2, or 4 bpp).
fn gen_monochrome_pio_program(
    color: MonochromeColor,
) -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new();

    const RGB_BPP: u8 = 12;
    let bpp = color as u8;

    let mut repeat = a.label();
    let mut more = a.label();

    // Pull `bpp` bits (1, 2, or 4) from the TX FIFO into OSR and to X
    a.out(pio::OutDestination::X, bpp);
    a.bind(&mut more);

    // Loop counter in `Y` to repeat `bpp` as many times as need
    // to fill RGB444 for the greyscale color.
    a.set(pio::SetDestination::Y, RGB_BPP / bpp - 1);
    a.bind(&mut repeat);
    // Push the bits into ISR which goes into RX FIFO.
    a.r#in(pio::InSource::X, bpp);
    // Repeat the bits
    a.jmp(pio::JmpCondition::YDecNonZero, &mut repeat);

    //a.wait(1, pio::WaitSource::IRQ, 4, false);

    // If there are more pixels to process, go back to the beginning
    a.jmp(pio::JmpCondition::OutputShiftRegisterNotEmpty, &mut more);

    a.assemble_program()
}

fn rgb444_pio_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    // Pin assignments:
    // - CSn is side-set bit 0
    // - SCK is side-set bit 1
    // - MOSI is OUT bit 0 (host-to-device)

    pio_proc::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        "   set     y, 921600 side 2",
        "   set     x, 0 side 0",
        "bitloop:",
        "   out     pins, 1 side 2",
        "   jmp     y-- bitloop side 0",
        ".wrap",
    )
    .program
}

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

impl<TDispAttr, const N: usize> Display<TDispAttr, N>
where
    TDispAttr: DisplayAttributes,
{
    pub fn new(color: MonochromeColor) -> Self {
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
            crate::XOSC_CRYSTAL_FREQ,
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

        let mut display = ManualDisplaySpi::new((mosi_pin, sck_pin, cs_pin, dc_pin));

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

        // Prepare the display hardware for sending data.
        // The pins will be released after the display is done with and
        // the display hardware is ready to receive data.
        let (mosi_pin, sck_pin, cs_pin, dc_pin) = {
            display.write_command(Command::RAMWR);
            display.release()
        };

        let display_buffer = MonochromeDisplayBuffer {
            buffer: [0; N],
            width: TDispAttr::width(),
            height: TDispAttr::height(),
            color,
        };

        let (color_expand_sm, color_expand_rx, color_expand_tx) =
            PIOBuilder::from_installed_program(
                pio.install(&gen_monochrome_pio_program(color)).unwrap(),
            )
            .autopull(true)
            .autopush(true)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);

        defmt::info!("MOSI pin: {:?}", mosi_pin.id().num);
        defmt::info!("SCK pin: {:?}", sck_pin.id().num);
        defmt::info!("CS pin: {:?}", cs_pin.id().num);
        defmt::info!("DC pin: {:?}", dc_pin.id().num);

        // Single-cycle I/O output pin
        let base_pin = mosi_pin.id().num;
        let pin_count = 1;
        let mosi_pin = mosi_pin.into_function::<FunctionPio0>(); // Pin 19

        // Side-set pins for the PIO state machine
        let sck_pin = sck_pin.into_function::<FunctionPio0>(); // Pin 18
        let cs_pin = cs_pin.into_function::<FunctionPio0>(); // Pin 17

        let (mut display_spi_sm, _display_spi_rx, display_spi_tx) =
            PIOBuilder::from_installed_program(pio.install(&rgb444_pio_program()).unwrap())
                .autopull(true)
                .autopush(true)
                .clock_divisor_fixed_point(1, 0)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .set_pins(base_pin, pin_count)
                .out_pins(base_pin, pin_count)
                .side_set_pin_base(cs_pin.id().num)
                .build(sm1);
        display_spi_sm.set_pindirs([
            (mosi_pin.id().num, PinDir::Output),
            (sck_pin.id().num, PinDir::Output),
            (cs_pin.id().num, PinDir::Output),
        ]);

        let color_expand_sm = color_expand_sm.start();
        let display_spi_sm = display_spi_sm.start();

        // This DMA channel transfers data from the PIO state machine's
        // RX FIFO to the display. It will be stalled until the
        // next DMA channel is started and feeds the PIO TX FIFO.
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
            tx_count: 12 / color.bits_per_pixel() as u32 * N as u32, // RGB444, monochrome color
            tx_req: TxReq::Pio0Tx1,
            byte_swap: false,
            start: true,
        });

        // This DMA channel transfers data from the input buffer to the PIO state machine's TX FIFO.
        // If this one is chained to dma0 (that writes to this channel's read trigger address),
        // the two will be res-starting together, running in the ping-pong mode.
        let color_expand_sm_dma = LaxDmaWrite::new::<dma::CH1>(Config {
            high_priority: true,
            word_size: TxSize::_32bit,
            source: Source {
                address: core::ptr::null(),
                increment: true,
            },
            destination: Destination {
                address: color_expand_tx.fifo_address().cast_mut().cast(),
                increment: false,
            },
            tx_count: N as u32,
            tx_req: TxReq::Pio0Tx0,
            byte_swap: false,
            start: false,
        });
        let dma_trig_addr: *mut u32 = color_expand_sm_dma.read_trig_addr().cast_mut().cast();

        let mut display = Display {
            display_buffer,
            display_attr: PhantomData,
            width: TDispAttr::width(),
            height: TDispAttr::height(),
            pixel_count: TDispAttr::width() as u32 * TDispAttr::height() as u32,
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
            color_expand_sm,
            display_spi_sm,
        };

        display.set_backlight(40);
        delay.delay_ms(10);

        display
    }

    pub fn flush(&mut self) {
        self.wait_for_vsync();

        // Start the DMA channel that writes to the PIO state machine's TX FIFO
        unsafe { *self.dma_trig_addr = self.display_buffer.buffer.as_ptr() as u32 };

        while !self.display_sm_dma.is_done() {
            defmt::info!(
                "N 0x{:x}, color DMA 0x{:x} remaining, display DMA 0x{:x} remaining",
                N,
                self.color_expand_sm_dma.tx_count_remaining(),
                self.display_sm_dma.tx_count_remaining()
            );
        }
        defmt::info!(
            "N 0x{:x}, color DMA 0x{:x} remaining, display DMA 0x{:x} remaining",
            N,
            self.color_expand_sm_dma.tx_count_remaining(),
            self.display_sm_dma.tx_count_remaining()
        );

        defmt::info!("Dispaly DMA done");
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
        self.last_vsync_time = crate::time::time_us();
    }
}

pub struct PicoDisplay2_8(Display<Display2_8, 2400>);

impl PicoDisplay2_8 {
    pub fn new() -> Self {
        Self(Display::<Display2_8, 2400>::new(MonochromeColor::Bpp1))
    }

    pub fn flush(&mut self) {
        self.0.flush();
    }
}

//pub type PicoDisplay2_0 = Display<Display2_0>;
//pub type PicoDisplay1_14 = Display<Display1_14>;
//pub type PicoDisplaySquare = Display<DisplaySquare>;
