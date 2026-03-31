//! PIO programs for the ST7789 display driver and DMA test utilities.
//!
//! Contains the color expansion (SM0) and SPI output (SM1) programs
//! shared between the firmware, DMA tests, and host-side test simulator.

#![no_std]

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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

/// Display color mode: monochrome (greyscale via PIO color expansion),
/// native RGB444 (12-bit pixels), or native RGB565 (16-bit pixels).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DisplayMode {
    Mono(MonochromeColor),
    Rgb444,
    Rgb565,
}

impl DisplayMode {
    pub const fn is_rgb565(&self) -> bool {
        matches!(self, DisplayMode::Rgb565)
    }

    pub const fn is_rgb444(&self) -> bool {
        matches!(self, DisplayMode::Rgb444)
    }

    pub const fn is_mono(&self) -> bool {
        matches!(self, DisplayMode::Mono(_))
    }

    /// Whether this mode feeds SM1 directly from DMA (no SM0 color expansion).
    pub const fn is_direct(&self) -> bool {
        matches!(self, DisplayMode::Rgb565 | DisplayMode::Rgb444)
    }

    /// Number of u32 words per frame buffer for the given display dimensions.
    pub const fn frame_words(&self, width: u16, height: u16) -> usize {
        let pixels = width as usize * height as usize;
        match self {
            DisplayMode::Mono(c) => pixels * c.bits_per_pixel() as usize / 32,
            // RGB444: 2 pixels per u32 (12 bits each, 8 bits unused per word)
            DisplayMode::Rgb444 => pixels / 2,
            // RGB565: 2 pixels per u32 (16 bits each)
            DisplayMode::Rgb565 => pixels / 2,
        }
    }

    /// Whether the mode supports double buffering (fits in 264 KB RAM).
    /// Conservative: double-buffer only if two frame buffers fit in ~200 KB
    /// (leaving headroom for stack, statics, and glyph data).
    pub const fn double_buffered_for(&self, width: u16, height: u16) -> bool {
        let single = self.frame_words(width, height) * 4; // bytes
        single * 2 <= 200 * 1024
    }

    /// ST7789 COLMOD value for this mode.
    pub const fn colmod(&self) -> u8 {
        match self {
            DisplayMode::Mono(_) | DisplayMode::Rgb444 => 0x03, // 12-bit RGB444
            DisplayMode::Rgb565 => 0x05,                        // 16-bit RGB565
        }
    }

    /// PIO cycles per pixel (SM1 bottleneck).
    pub const fn cycles_per_pixel(&self) -> u64 {
        match self {
            // 12-bit SPI: 3 cycles/bit × 12 + 3 overhead = 39
            DisplayMode::Mono(_) | DisplayMode::Rgb444 => 39,
            // 16-bit SPI: 3 cycles/bit × 16 + 2 overhead = 50
            DisplayMode::Rgb565 => 50,
        }
    }
}

// ── Monochrome color expansion ──────────────────────────────────────────────

/// Core monochrome expansion: reads `bpp` bits from TX FIFO, replicates to
/// 12-bit RGB444, pushes to RX FIFO. If `irq_handshake` is true, appends
/// `wait 1 irq 4` for SM0↔SM1 flow control.
fn build_monochrome_program(
    color: MonochromeColor,
    irq_handshake: bool,
) -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new();

    const RGB_BPP: u8 = 12;
    let bpp = color as u8;

    let mut repeat = a.label();

    // Pull `bpp` bits (1, 2, or 4) from the TX FIFO into OSR and to X
    a.out(pio::OutDestination::X, bpp);

    // Loop counter in `Y` to repeat `bpp` as many times as needed
    // to fill RGB444 for the greyscale color.
    a.set(pio::SetDestination::Y, RGB_BPP / bpp - 1);
    a.bind(&mut repeat);
    // Push the bits into ISR which goes into RX FIFO.
    a.r#in(pio::InSource::X, bpp);
    // Repeat the bits
    a.jmp(pio::JmpCondition::YDecNonZero, &mut repeat);

    if irq_handshake {
        // Wait for SM1 to signal it has consumed the previous pixel.
        // `wait 1 irq 4` waits for flag 4 to be set, then clears it.
        a.wait(1, pio::WaitSource::IRQ, 4, false);
    }

    // Default wrap returns to instruction 0 (out x, bpp) to read the next pixel.
    a.assemble_program()
}

/// Monochrome color expansion with IRQ 4 handshake (SM0 in the display pipeline).
///
/// After expanding each pixel, waits for SM1 to signal readiness via IRQ 4.
/// This ensures correct flow control at any clock ratio between the two SMs.
///
/// Configuration requirements:
///   - autopush threshold: 12
///   - in_shift_direction: Left (data enters at LSB)
///   - buffers: OnlyRx (double RX FIFO depth for DMA headroom)
pub fn gen_monochrome_pio_program(
    color: MonochromeColor,
) -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    build_monochrome_program(color, true)
}

/// Monochrome color expansion without IRQ handshake (standalone DMA tests).
///
/// Same pixel expansion logic as [`gen_monochrome_pio_program`] but without
/// the `wait 1 irq 4` at the end, allowing it to run on a single SM without
/// a partner.
pub fn gen_monochrome_expand_program(
    color: MonochromeColor,
) -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    build_monochrome_program(color, false)
}

// ── SPI output ──────────────────────────────────────────────────────────────

/// SPI transmit PIO program for the ST7789 display (SM1).
///
/// Shifts 12 bits (one RGB444 pixel) per iteration via SPI mode 0
/// (CPOL=0, CPHA=0). After each 12-bit group, signals SM0 via IRQ 4
/// that it is ready for the next pixel.
///
/// At the start of each frame, loads the pixel count from its TX FIFO.
/// After shifting the last pixel, raises host-visible IRQ 0 (frame done)
/// and stalls on `pull` waiting for the next frame's pixel count.
///
/// Pin assignments (side-set 2 bits):
///   - CSn: side-set bit 0 (GPIO17, directly after side-set base)
///   - SCK: side-set bit 1 (GPIO18)
///   - MOSI: out bit 0     (GPIO19)
///
/// ```text
///   Instruction flow (9 instructions):
///
///   .wrap_target
///   0: pull  block        side 0  Load pixel count from TX FIFO (stalls between frames)
///   1: out   x, 32        side 0  X = pixel count - 1
///   pixel:
///   2: irq   set 4        side 0  Signal SM0: ready for next pixel
///   3: set   y, 11        side 0  12-bit loop counter; CSn=0 SCK=0
///   bitloop:
///   4: out   pins, 1      side 0  MOSI=data bit; SCK=0 (data setup)
///   5: nop                side 2  SCK=1 (rising edge, display samples)
///   6: jmp   y-- bitloop  side 0  SCK=0 (falling edge); repeat 12x
///   7: jmp   x-- pixel    side 0  More pixels? decrement counter, next pixel
///   8: irq   set 0        side 0  Frame done: raise host-visible IRQ 0
///   .wrap                         → back to pull for next frame's count
///
///   Wave diagram for 2 pixels (24 bits: b0..b11, b0..b11):
///
///            2   3   4   5   6   4   5   6       4   5   6   7
///          +---+---+---+---+---+---+---+---+   +---+---+---+---+
///   SCK  __| L | L | L | H | L | L | H | L |...| L | H | L | L |...
///          +---+---+---+---+---+---+---+---+   +---+---+---+---+
///          |irq|set|out|nop|jmp|out|nop|jmp|   |out|nop|jmp|jmp|
///          +---+---+---+---+---+---+---+---+   +---+---+---+---+
///   MOSI __|___|___| b0        | b1       |...| b11       |___|...
///                    ^-sample    ^-sample       ^-sample
///   IRQ4  _/ \_________________________/ \_______ ...  (SM0 handshake)
///
///   Timing per bit: 3 PIO cycles (out + nop + jmp)
///   Timing per pixel: 3 * 12 + 3 (irq + set + jmp) = 39 cycles
///   Effective SPI clock: PIO_clk / 3 (e.g., 125 MHz / 3 ~ 41.7 MHz)
///
///   Frame-done IRQ:
///     When X reaches 0, `jmp x--` falls through to `irq set 0`,
///     raising host-visible PIO IRQ flag 0. The host can poll
///     `is_frame_done()` or enable PIO0_IRQ_0 in the NVIC for an
///     interrupt-driven callback. After the IRQ, SM1 wraps back to
///     `pull block` and stalls until the host writes the next
///     frame's pixel count, naturally gating the pipeline.
///
///   Configuration requirements:
///     - out_shift_direction: Right (LSB first, matches SM0's in_shift Left)
///     - autopull threshold: 12 (one pull per pixel, pixel-aligned FIFO words)
///     - buffers: OnlyTx (double TX FIFO depth for DMA headroom)
/// ```
pub fn rgb444_pio_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        // Frame start: load pixel count from TX FIFO.
        // The host writes (pixel_count - 1) before each frame.
        // `pull block` stalls here between frames until the host is ready.
        "   pull   block        side 0",
        "   out    x, 32        side 0",  // X = pixel count - 1
        "pixel:",
        "   irq    set 4        side 0",  // Signal SM0: consumed pixel, ready for next
        "   set    y, 11        side 0",  // 12 bits per RGB444 pixel; SCK=0, CS=0
        "bitloop:",
        "   out    pins, 1      side 0",  // Set MOSI with SCK low (data setup)
        "   nop                 side 2",  // SCK high — display samples MOSI
        "   jmp    y-- bitloop  side 0",  // SCK low (falling edge)
        "   jmp    x-- pixel    side 0",  // More pixels? decrement counter, next pixel
        "   irq    set 0        side 0",  // Frame done: raise host-visible IRQ 0
        ".wrap",                           // → back to pull block for next frame
    )
    .program
}

// ── RGB565 SPI output ──────────────────────────────────────────────────────

/// RGB565 SPI transmit PIO program for the ST7789 display (SM1).
///
/// Shifts 16 bits (one RGB565 pixel) per iteration via SPI mode 0
/// (CPOL=0, CPHA=0). No IRQ handshake — DMA feeds pixels directly from
/// the frame buffer to SM1's TX FIFO.
///
/// At the start of each frame, loads the pixel count from its TX FIFO.
/// After shifting the last pixel, raises host-visible IRQ 0 (frame done)
/// and stalls on `pull` waiting for the next frame's pixel count.
///
/// The frame buffer stores two pixels per u32 word. Even pixels occupy
/// bits [31:16] and odd pixels bits [15:0], matching the shift-left OSR
/// direction so that pixel order is preserved on the wire.
///
/// ```text
///   Timing per bit: 3 PIO cycles (out + nop + jmp)
///   Timing per pixel: 3 × 16 + 2 (set + jmp) = 50 cycles
///   Effective SPI clock: PIO_clk / 3 (e.g., 125 MHz / 3 ~ 41.7 MHz)
/// ```
///
/// Configuration requirements:
///   - out_shift_direction: Left (MSB first for SPI)
///   - autopull threshold: 32 (one pull per two pixels)
///   - buffers: OnlyTx
pub fn rgb565_spi_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        "   pull   block        side 0", // Frame start: load pixel count
        "   out    x, 32        side 0", // X = pixel count - 1
        "pixel:",
        "   set    y, 15        side 0", // 16 bits per RGB565 pixel
        "bitloop:",
        "   out    pins, 1      side 0", // MOSI = data bit; SCK low
        "   nop                 side 2", // SCK high — display samples
        "   jmp    y-- bitloop  side 0", // SCK low; repeat 16×
        "   jmp    x-- pixel    side 0", // next pixel
        "   irq    set 0        side 0", // frame done
        ".wrap",
    )
    .program
}

// ── Frame timing estimation ─────────────────────────────────────────────────

/// Estimated PIO cycle counts and frame rate for the SM0+SM1 display pipeline.
///
/// SM1 is the bottleneck: it shifts 12 SPI bits per pixel at 3 PIO cycles
/// per bit, plus 3 overhead cycles per pixel (irq + set + jmp), plus 3
/// one-time cycles per frame (pull + out + irq).
///
/// SM0 (color expansion) runs in parallel and is always faster than SM1
/// for any bpp ≤ 12, so it never limits throughput.
pub struct FrameTiming {
    /// Total pixels in the frame.
    pub pixel_count: u32,
    /// PIO cycles SM1 spends shifting pixel data (39 × pixel_count).
    pub pixel_cycles: u64,
    /// PIO cycles for the full frame (pixel_cycles + 3 overhead).
    pub total_cycles: u64,
    /// Frame time in microseconds at the given PIO clock frequency.
    pub frame_us: u64,
    /// Frames per second (integer, rounded down).
    pub fps: u32,
    /// Effective SPI clock in Hz (PIO clock / 3).
    pub spi_clock_hz: u32,
}

impl FrameTiming {
    /// Compute frame timing for a given display size and PIO clock frequency.
    ///
    /// `pio_clock_hz` is the PIO state machine clock (system clock / divisor).
    /// With `clock_divisor_fixed_point(1, 0)` this equals the system clock,
    /// typically 125 MHz on the RP2040.
    pub const fn new(width: u16, height: u16, pio_clock_hz: u32) -> Self {
        let pixel_count = width as u32 * height as u32;
        // SM1 timing: 39 cycles/pixel + 3 frame overhead (pull + out x + irq set 0)
        let pixel_cycles = 39 * pixel_count as u64;
        let total_cycles = pixel_cycles + 3;
        // frame_us = total_cycles * 1_000_000 / pio_clock_hz
        let frame_us = total_cycles * 1_000_000 / pio_clock_hz as u64;
        let fps = if frame_us > 0 {
            1_000_000 / frame_us as u32
        } else {
            0
        };
        let spi_clock_hz = pio_clock_hz / 3;
        FrameTiming {
            pixel_count,
            pixel_cycles,
            total_cycles,
            frame_us,
            fps,
            spi_clock_hz,
        }
    }

    /// Compute frame timing for RGB565 mode (50 cycles/pixel).
    pub const fn new_rgb565(width: u16, height: u16, pio_clock_hz: u32) -> Self {
        let pixel_count = width as u32 * height as u32;
        let pixel_cycles = 50 * pixel_count as u64;
        let total_cycles = pixel_cycles + 3;
        let frame_us = total_cycles * 1_000_000 / pio_clock_hz as u64;
        let fps = if frame_us > 0 {
            1_000_000 / frame_us as u32
        } else {
            0
        };
        let spi_clock_hz = pio_clock_hz / 3;
        FrameTiming {
            pixel_count,
            pixel_cycles,
            total_cycles,
            frame_us,
            fps,
            spi_clock_hz,
        }
    }
}

// ── DMA test utility programs ───────────────────────────────────────────────

/// Bitwise-invert each 32-bit word. Uses explicit pull/push (no autopull).
/// After each word, sets IRQ 4 and waits for partner SM to clear it.
///
/// Configuration: autopull(false), autopush(false).
pub fn invert_pio_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        "more:",
        "       pull",
        "       mov     x, osr",
        "       mov     x, ~x",
        "       mov     isr, x",
        "       push",
        "       irq     wait 4",
        "       jmp     !osre, more",
    )
    .program
}

/// Bitwise-invert each 32-bit word, synchronized via IRQ 4 from partner SM.
/// Waits for IRQ 4 before pulling the next word.
///
/// Configuration: autopull(false), autopush(false).
pub fn invert_pio_again_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        "more:",
        "       wait    1 irq 4",
        "       pull",
        "       mov     x, osr",
        "       mov     x, ~x",
        "       mov     isr, x",
        "       push",
        "       jmp     !osre, more",
    )
    .program
}
