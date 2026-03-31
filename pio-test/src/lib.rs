//! Host-side tests for PIO state machine programs.
//!
//! These tests simulate PIO instruction execution to verify correctness
//! of the color expansion (SM0) and SPI output (SM1) programs without
//! requiring RP2040 hardware.
//!
//! Run with: cargo test -p pio-test

use pio::{
    InSource, Instruction, InstructionOperands, JmpCondition, MovDestination, MovOperation,
    MovSource, OutDestination, Program, RP2040_MAX_PROGRAM_SIZE, SetDestination, SideSet,
    WaitSource,
};
pub use pio_programs::{
    FrameTiming, MonochromeColor, gen_monochrome_expand_program, gen_monochrome_pio_program,
    invert_pio_again_program, invert_pio_program, rgb444_pio_program,
};
use std::cell::Cell;
use std::collections::VecDeque;
use std::rc::Rc;

// ── Minimal PIO simulator ───────────────────────────────────────────────────

/// Result of a single PIO step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepResult {
    /// Instruction executed successfully.
    Ok,
    /// Stalled: TX FIFO empty on autopull or explicit PULL block.
    TxStall,
    /// Stalled: RX FIFO full on autopush or explicit PUSH block.
    RxStall,
    /// Stalled: WAIT condition not met.
    WaitStall,
}

/// Simulated PIO state machine state. Supports the instruction subset used
/// by the two display programs: OUT, IN, SET, JMP, MOV/NOP, PULL, WAIT, IRQ.
pub struct PioSim {
    pub x: u32,
    pub y: u32,
    pub osr: u32,
    pub osr_shift_count: u8,
    pub isr: u32,
    pub isr_shift_count: u8,
    pub pc: u8,
    program: Vec<Instruction>,
    wrap_source: u8,
    wrap_target: u8,
    #[allow(dead_code)]
    side_set: SideSet,
    pub autopull_threshold: u8,
    pub autopush_threshold: u8,
    /// When true, IN shifts left (new data enters at LSB, old data moves up).
    /// When false (default), IN shifts right (new data enters at MSB).
    pub isr_shift_left: bool,
    pub tx_fifo: VecDeque<u32>,
    pub rx_fifo: VecDeque<u32>,
    /// Maximum RX FIFO depth (default 4, RP2040 standard; 8 with OnlyRx buffers).
    pub rx_fifo_depth: usize,
    /// Overflow buffer: words drained from rx_fifo when it was full during `run()`.
    pub rx_overflow: VecDeque<u32>,
    /// Shared PIO IRQ flags (bits 0–7). Use `Rc<Cell<u8>>` to share between SMs.
    pub irq_flags: Rc<Cell<u8>>,
    /// Tracks whether an `irq wait N` instruction has already set its flag
    /// and is waiting for the partner SM to clear it.
    irq_wait_pending: bool,
    /// Recorded side-set values per step (for SPI clock verification).
    pub side_set_log: Vec<Option<u8>>,
    /// Recorded OUT pin values per step.
    pub out_pin_log: Vec<Option<u32>>,
    pub steps: u64,
}

impl PioSim {
    pub fn new(program: &Program<{ RP2040_MAX_PROGRAM_SIZE }>) -> Self {
        let instructions: Vec<Instruction> = program
            .code
            .iter()
            .map(|&word| Instruction::decode(word, program.side_set).unwrap())
            .collect();

        PioSim {
            x: 0,
            y: 0,
            osr: 0,
            osr_shift_count: 32, // empty
            isr: 0,
            isr_shift_count: 0,
            pc: program.wrap.target,
            program: instructions,
            wrap_source: program.wrap.source,
            wrap_target: program.wrap.target,
            side_set: program.side_set,
            autopull_threshold: 0, // 0 means 32
            autopush_threshold: 0,
            isr_shift_left: false,
            tx_fifo: VecDeque::new(),
            rx_fifo: VecDeque::new(),
            rx_fifo_depth: 4,
            rx_overflow: VecDeque::new(),
            irq_flags: Rc::new(Cell::new(0u8)),
            irq_wait_pending: false,
            side_set_log: Vec::new(),
            out_pin_log: Vec::new(),
            steps: 0,
        }
    }

    fn effective_pull_threshold(&self) -> u8 {
        if self.autopull_threshold == 0 { 32 } else { self.autopull_threshold }
    }

    fn effective_push_threshold(&self) -> u8 {
        if self.autopush_threshold == 0 { 32 } else { self.autopush_threshold }
    }

    pub fn push_tx(&mut self, word: u32) {
        self.tx_fifo.push_back(word);
    }

    pub fn pop_rx(&mut self) -> Option<u32> {
        self.rx_overflow
            .pop_front()
            .or_else(|| self.rx_fifo.pop_front())
    }

    /// Try autopull: refill OSR from TX FIFO if shift count reached threshold.
    fn try_autopull(&mut self) -> bool {
        if self.osr_shift_count >= self.effective_pull_threshold() {
            if let Some(word) = self.tx_fifo.pop_front() {
                self.osr = word;
                self.osr_shift_count = 0;
                return true;
            }
            return false; // TX FIFO empty — would stall in real hardware
        }
        true
    }

    /// Try autopush: push ISR to RX FIFO if shift count reached threshold.
    /// Returns false if the RX FIFO is full (would stall in real hardware).
    fn try_autopush(&mut self) -> bool {
        if self.isr_shift_count >= self.effective_push_threshold() {
            if self.rx_fifo.len() >= self.rx_fifo_depth {
                return false; // RX FIFO full — stall
            }
            self.rx_fifo.push_back(self.isr);
            self.isr = 0;
            self.isr_shift_count = 0;
        }
        true
    }

    /// Execute one instruction. Returns the step result indicating success
    /// or the type of stall.
    pub fn step(&mut self) -> StepResult {
        let instr = self.program[self.pc as usize];

        // Pre-check instructions that can stall before committing side-set.
        match instr.operands {
            InstructionOperands::WAIT {
                polarity,
                source: WaitSource::IRQ,
                index,
                ..
            } => {
                let flags = self.irq_flags.get();
                let flag_val = (flags >> index) & 1;
                if flag_val != polarity {
                    return StepResult::WaitStall;
                }
                // Condition met — clear the flag (hardware behavior for IRQ waits).
                self.irq_flags.set(flags & !(1 << index));
            }
            InstructionOperands::PULL { block, .. } => {
                if self.tx_fifo.is_empty() {
                    if block {
                        return StepResult::TxStall;
                    }
                    // Non-blocking: copy X to OSR.
                    self.osr = self.x;
                    self.osr_shift_count = 0;
                }
            }
            InstructionOperands::PUSH { block, .. } => {
                // Non-blocking with full FIFO: silently drop (don't push).
                if self.rx_fifo.len() >= self.rx_fifo_depth && block {
                    return StepResult::RxStall;
                }
            }
            InstructionOperands::IN { bit_count, .. } => {
                // Autopush stall: if this IN would trigger autopush but the
                // RX FIFO is full, the real hardware stalls the IN entirely.
                let bits = if bit_count == 0 { 32 } else { bit_count };
                let new_count = self.isr_shift_count + bits;
                if new_count >= self.effective_push_threshold()
                    && self.rx_fifo.len() >= self.rx_fifo_depth
                {
                    return StepResult::RxStall;
                }
            }
            InstructionOperands::IRQ {
                clear: false,
                wait: true,
                index,
                ..
            } => {
                // `irq wait N`: set flag, then stall until partner clears it.
                let flags = self.irq_flags.get();
                if self.irq_wait_pending {
                    // Already set the flag — waiting for partner to clear.
                    if (flags >> index) & 1 == 0 {
                        // Partner cleared it — proceed.
                        self.irq_wait_pending = false;
                    } else {
                        return StepResult::WaitStall;
                    }
                } else {
                    // First encounter: set the flag and stall.
                    self.irq_flags.set(flags | (1 << index));
                    self.irq_wait_pending = true;
                    return StepResult::WaitStall;
                }
            }
            _ => {}
        }

        // Instruction will execute — commit side-set.
        self.side_set_log.push(instr.side_set);

        let mut out_pin_val = None;
        let mut next_pc = self.pc + 1;

        match instr.operands {
            InstructionOperands::OUT {
                destination,
                bit_count,
            } => {
                let bits = if bit_count == 0 { 32 } else { bit_count };

                // Autopull before OUT if OSR is exhausted.
                if !self.try_autopull() {
                    self.side_set_log.pop(); // undo log — we stalled
                    return StepResult::TxStall;
                }

                // Shift right: extract LSBs.
                let mask = if bits == 32 { u32::MAX } else { (1u32 << bits) - 1 };
                let data = self.osr & mask;
                self.osr = if bits == 32 { 0 } else { self.osr >> bits };
                self.osr_shift_count += bits;

                match destination {
                    OutDestination::X => self.x = data,
                    OutDestination::Y => self.y = data,
                    OutDestination::PINS => out_pin_val = Some(data),
                    OutDestination::NULL => {}
                    _ => panic!("unsupported OUT destination: {:?}", destination),
                }
            }
            InstructionOperands::IN { source, bit_count } => {
                let bits = if bit_count == 0 { 32 } else { bit_count };

                let data = match source {
                    InSource::X => self.x,
                    InSource::Y => self.y,
                    InSource::NULL => 0,
                    _ => panic!("unsupported IN source: {:?}", source),
                };

                let mask = if bits == 32 { u32::MAX } else { (1u32 << bits) - 1 };
                let data = data & mask;

                if self.isr_shift_left {
                    // Shift left: old data moves up, new data enters at LSB.
                    self.isr = (self.isr << bits) | data;
                } else {
                    // Shift right: new data enters at MSB side.
                    self.isr >>= bits;
                    self.isr |= data << (32 - bits);
                }
                self.isr_shift_count += bits;

                self.try_autopush();
            }
            InstructionOperands::SET { destination, data } => match destination {
                SetDestination::Y => self.y = data as u32,
                SetDestination::X => self.x = data as u32,
                SetDestination::PINS => out_pin_val = Some(data as u32),
                _ => panic!("unsupported SET destination: {:?}", destination),
            },
            InstructionOperands::JMP {
                condition,
                address,
            } => {
                let take = match condition {
                    JmpCondition::Always => true,
                    JmpCondition::YDecNonZero => {
                        let nz = self.y != 0;
                        self.y = self.y.wrapping_sub(1);
                        nz
                    }
                    JmpCondition::XDecNonZero => {
                        let nz = self.x != 0;
                        self.x = self.x.wrapping_sub(1);
                        nz
                    }
                    JmpCondition::XIsZero => self.x == 0,
                    JmpCondition::YIsZero => self.y == 0,
                    JmpCondition::OutputShiftRegisterNotEmpty => {
                        self.osr_shift_count < self.effective_pull_threshold()
                    }
                    _ => panic!("unsupported JMP condition: {:?}", condition),
                };
                if take {
                    next_pc = address;
                }
            }
            InstructionOperands::PULL { .. } => {
                // Stall check was handled in the pre-check above.
                // If we're here, FIFO has data (or non-blocking already handled).
                if let Some(word) = self.tx_fifo.pop_front() {
                    self.osr = word;
                    self.osr_shift_count = 0;
                }
            }
            InstructionOperands::WAIT { .. } => {
                // Condition check and flag clear handled in pre-check above.
            }
            InstructionOperands::IRQ {
                clear,
                index,
                ..
            } => {
                let flags = self.irq_flags.get();
                if clear {
                    self.irq_flags.set(flags & !(1 << index));
                } else {
                    self.irq_flags.set(flags | (1 << index));
                }
            }
            InstructionOperands::PUSH { .. } => {
                // Pre-check handled stall cases above. If we're here, either
                // the FIFO has room (block or non-block) or it's a non-blocking
                // push that will silently drop.
                if self.rx_fifo.len() < self.rx_fifo_depth {
                    self.rx_fifo.push_back(self.isr);
                    self.isr = 0;
                    self.isr_shift_count = 0;
                }
                // Non-blocking with full FIFO: ISR unchanged, data lost.
            }
            InstructionOperands::MOV {
                destination,
                op,
                source,
            } => {
                let val = match source {
                    MovSource::X => self.x,
                    MovSource::Y => self.y,
                    MovSource::NULL => 0,
                    MovSource::ISR => self.isr,
                    MovSource::OSR => self.osr,
                    _ => panic!("unsupported MOV source: {:?}", source),
                };
                let val = match op {
                    MovOperation::None => val,
                    MovOperation::Invert => !val,
                    MovOperation::BitReverse => val.reverse_bits(),
                };
                match destination {
                    MovDestination::X => self.x = val,
                    MovDestination::Y => self.y = val,
                    MovDestination::ISR => {
                        self.isr = val;
                        self.isr_shift_count = 0;
                    }
                    MovDestination::OSR => {
                        self.osr = val;
                        self.osr_shift_count = 0;
                    }
                    MovDestination::PINS => out_pin_val = Some(val),
                    _ => panic!("unsupported MOV destination: {:?}", destination),
                }
            }
            _ => panic!("unsupported instruction: {:?}", instr.operands),
        }

        self.out_pin_log.push(out_pin_val);

        // Wrap: if we just executed the wrap-source instruction and would
        // advance past it, jump to wrap-target instead.
        if self.pc == self.wrap_source && next_pc == self.wrap_source + 1 {
            next_pc = self.wrap_target;
        }
        self.pc = next_pc;
        self.steps += 1;
        StepResult::Ok
    }

    /// Run for up to `max_steps`, stopping early if the SM would stall on
    /// an empty TX FIFO. RX stalls are resolved by draining the RX FIFO
    /// into `rx_overflow`, simulating a DMA consumer. WAIT stalls are
    /// auto-satisfied by setting the awaited IRQ flag, simulating an
    /// always-ready partner SM (for isolated testing).
    pub fn run(&mut self, max_steps: u64) -> u64 {
        let start = self.steps;
        for _ in 0..max_steps {
            match self.step() {
                StepResult::Ok => {}
                StepResult::TxStall => break,
                StepResult::RxStall => {
                    // Drain the RX FIFO into overflow storage to unblock,
                    // then re-execute the stalled instruction.
                    while let Some(word) = self.rx_fifo.pop_front() {
                        self.rx_overflow.push_back(word);
                    }
                    continue;
                }
                StepResult::WaitStall => {
                    // Auto-satisfy the stall for isolated SM testing.
                    let instr = self.program[self.pc as usize];
                    match instr.operands {
                        InstructionOperands::WAIT {
                            source: WaitSource::IRQ,
                            index,
                            ..
                        } => {
                            // WAIT needs the flag set.
                            let flags = self.irq_flags.get();
                            self.irq_flags.set(flags | (1 << index));
                        }
                        InstructionOperands::IRQ {
                            wait: true,
                            index,
                            ..
                        } => {
                            // `irq wait` needs the flag cleared by partner.
                            let flags = self.irq_flags.get();
                            self.irq_flags.set(flags & !(1 << index));
                        }
                        _ => {}
                    }
                }
            }
        }
        self.steps - start
    }
}

// ── Dual-SM simulator for pipeline tests ───────────────────────────────────

/// Simulates two PIO state machines with shared IRQ flags and a DMA-like
/// transfer from SM0 RX FIFO to SM1 TX FIFO on every tick.
pub struct DualSim {
    pub sm0: PioSim,
    pub sm1: PioSim,
}

impl DualSim {
    pub fn new(
        sm0_prog: &Program<{ RP2040_MAX_PROGRAM_SIZE }>,
        sm1_prog: &Program<{ RP2040_MAX_PROGRAM_SIZE }>,
    ) -> Self {
        let flags = Rc::new(Cell::new(0u8));
        let mut sm0 = PioSim::new(sm0_prog);
        let mut sm1 = PioSim::new(sm1_prog);
        sm0.irq_flags = flags.clone();
        sm1.irq_flags = flags;
        DualSim { sm0, sm1 }
    }

    /// Run both SMs, stepping SM1 then SM0 each tick. DMA transfers SM0 RX
    /// words to SM1 TX at the start of each tick. Stops when both SMs stall
    /// and no DMA transfer occurred.
    pub fn run(&mut self, max_ticks: u64) -> u64 {
        let mut ticks = 0u64;
        for _ in 0..max_ticks {
            // DMA: drain SM0 RX into SM1 TX.
            let mut dma_transferred = false;
            while let Some(word) = self.sm0.rx_fifo.pop_front() {
                self.sm1.tx_fifo.push_back(word);
                dma_transferred = true;
            }
            // DMA: drain SM1 RX into overflow (simulates DMA consumer).
            while let Some(word) = self.sm1.rx_fifo.pop_front() {
                self.sm1.rx_overflow.push_back(word);
            }

            // Step SM1 first (it produces IRQ 4), then SM0 (consumes IRQ 4).
            let r1 = self.sm1.step();
            let r0 = self.sm0.step();
            ticks += 1;

            let sm0_stuck = r0 != StepResult::Ok;
            let sm1_stuck = r1 != StepResult::Ok;
            if sm0_stuck && sm1_stuck && !dma_transferred {
                break;
            }
        }
        ticks
    }
}

// ── Test helpers ────────────────────────────────────────────────────────────

#[cfg(test)]
/// Collect the RGB444 output stream from SM0 for a given bpp and input words.
/// Configures ISR shift-left and autopush threshold 12 to match firmware.
fn run_color_expand(color: MonochromeColor, input_words: &[u32], max_steps: u64) -> Vec<u32> {
    let program = gen_monochrome_pio_program(color);
    let mut sim = PioSim::new(&program);
    sim.isr_shift_left = true;
    sim.autopush_threshold = 12;
    for &word in input_words {
        sim.push_tx(word);
    }
    sim.run(max_steps);

    let mut output = Vec::new();
    while let Some(word) = sim.pop_rx() {
        output.push(word);
    }
    output
}

#[cfg(test)]
/// Each RX FIFO word contains one 12-bit RGB444 pixel in bits [11:0]
/// (ISR shift-left with autopush threshold 12).
fn extract_rgb444_pixels(words: &[u32]) -> Vec<u16> {
    words.iter().map(|&w| (w & 0xFFF) as u16).collect()
}

#[cfg(test)]
/// Side-set bit positions.
const SIDE_SCK: u8 = 1; // bit 1

#[cfg(test)]
fn sck_from_side(side: u8) -> u8 {
    (side >> SIDE_SCK) & 1
}

#[cfg(test)]
fn cs_from_side(side: u8) -> u8 {
    side & 1 // bit 0
}

#[cfg(test)]
/// Sample the SPI bit stream from an SM1 simulation: returns MOSI values
/// captured on each rising SCK edge.
fn sample_spi_bits(sim: &PioSim) -> Vec<u32> {
    let mut sampled = Vec::new();
    let mut prev_sck: u8 = 0;
    let mut last_mosi: u32 = 0;

    for i in 0..sim.side_set_log.len() {
        let side = sim.side_set_log[i].unwrap_or(0);
        let sck = sck_from_side(side);
        if let Some(val) = sim.out_pin_log[i] {
            last_mosi = val;
        }
        if prev_sck == 0 && sck == 1 {
            sampled.push(last_mosi);
        }
        prev_sck = sck;
    }
    sampled
}

#[cfg(test)]
/// Reassemble 12-bit pixels from an SPI bit stream (LSB-first order).
fn pixels_from_spi_bits(bits: &[u32], count: usize) -> Vec<u16> {
    (0..count)
        .map(|px| {
            (0..12)
                .map(|i| (bits[px * 12 + i] as u16) << i)
                .sum()
        })
        .collect()
}

// ── SM0 tests: color expansion ──────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod sm0_tests {
    use super::*;

    #[test]
    fn wrap_reads_every_pixel() {
        let program = gen_monochrome_pio_program(MonochromeColor::Bpp1);
        assert_eq!(
            program.wrap.target, 0,
            "wrap target must be instruction 0 (out x, bpp)"
        );
        assert_eq!(
            program.wrap.source as usize,
            program.code.len() - 1,
            "wrap source must be last instruction"
        );
    }

    #[test]
    fn first_instruction_is_out_x() {
        for color in [
            MonochromeColor::Bpp1,
            MonochromeColor::Bpp2,
            MonochromeColor::Bpp4,
        ] {
            let program = gen_monochrome_pio_program(color);
            let instr = Instruction::decode(program.code[0], program.side_set).unwrap();
            assert!(
                matches!(
                    instr.operands,
                    InstructionOperands::OUT {
                        destination: OutDestination::X,
                        ..
                    }
                ),
                "instruction 0 must be `out x, bpp` for {:?}",
                color
            );
        }
    }

    #[test]
    fn last_instruction_is_wait_irq() {
        for color in [
            MonochromeColor::Bpp1,
            MonochromeColor::Bpp2,
            MonochromeColor::Bpp4,
        ] {
            let program = gen_monochrome_pio_program(color);
            let last = program.code.len() - 1;
            let instr = Instruction::decode(program.code[last], program.side_set).unwrap();
            assert!(
                matches!(
                    instr.operands,
                    InstructionOperands::WAIT {
                        polarity: 1,
                        source: WaitSource::IRQ,
                        index: 4,
                        ..
                    }
                ),
                "last instruction must be `wait 1 irq 4` for {:?}",
                color
            );
        }
    }

    #[test]
    fn bpp1_expands_black_and_white() {
        // 32 pixels packed 1-bpp: alternating 1,0,1,0,...
        let input: u32 = 0xAAAA_AAAA;
        let words = run_color_expand(MonochromeColor::Bpp1, &[input], 10_000);
        let pixels = extract_rgb444_pixels(&words);

        assert!(
            pixels.len() >= 32,
            "expected >= 32 pixels, got {}",
            pixels.len()
        );
        for i in 0..32 {
            let src_bit = (input >> i) & 1;
            let expected = if src_bit == 1 { 0xFFF } else { 0x000 };
            assert_eq!(
                pixels[i], expected,
                "pixel {}: src_bit={}, expected 0x{:03X}, got 0x{:03X}",
                i, src_bit, expected, pixels[i]
            );
        }
    }

    #[test]
    fn bpp2_expands_all_levels() {
        // 16 pixels at 2-bpp: values 0,1,2,3 repeated.
        let mut input: u32 = 0;
        for i in 0..16u32 {
            input |= (i % 4) << (i * 2);
        }

        let words = run_color_expand(MonochromeColor::Bpp2, &[input], 10_000);
        let pixels = extract_rgb444_pixels(&words);
        assert!(
            pixels.len() >= 16,
            "expected >= 16 pixels, got {}",
            pixels.len()
        );

        for i in 0..16 {
            let src_val = ((input >> (i * 2)) & 0x3) as u16;
            // 2-bit value repeated 6 times to fill 12 bits.
            let expected: u16 = (0..6).map(|rep| src_val << (rep * 2)).sum();
            assert_eq!(
                pixels[i], expected,
                "pixel {}: src=0b{:02b}, expected 0x{:03X}, got 0x{:03X}",
                i, src_val, expected, pixels[i]
            );
        }
    }

    #[test]
    fn bpp4_expands_all_levels() {
        // 8 pixels at 4-bpp.
        let mut input: u32 = 0;
        for i in 0..8u32 {
            input |= (i * 2) << (i * 4);
        }

        let words = run_color_expand(MonochromeColor::Bpp4, &[input], 10_000);
        let pixels = extract_rgb444_pixels(&words);
        assert!(
            pixels.len() >= 8,
            "expected >= 8 pixels, got {}",
            pixels.len()
        );

        for i in 0..8 {
            let src_val = ((input >> (i * 4)) & 0xF) as u16;
            let expected = src_val | (src_val << 4) | (src_val << 8);
            assert_eq!(
                pixels[i], expected,
                "pixel {}: src=0x{:X}, expected 0x{:03X}, got 0x{:03X}",
                i, src_val, expected, pixels[i]
            );
        }
    }

    #[test]
    fn crosses_word_boundaries() {
        // 32 white pixels then 32 black pixels across two u32 words.
        let words =
            run_color_expand(MonochromeColor::Bpp1, &[0xFFFF_FFFF, 0x0000_0000], 100_000);
        let pixels = extract_rgb444_pixels(&words);
        assert!(
            pixels.len() >= 64,
            "expected >= 64 pixels, got {}",
            pixels.len()
        );

        for i in 0..32 {
            assert_eq!(pixels[i], 0xFFF, "pixel {} should be white", i);
        }
        for i in 32..64 {
            assert_eq!(pixels[i], 0x000, "pixel {} should be black", i);
        }
    }
}

// ── SM1 tests: SPI output ───────────────────────────────────────────────────

#[cfg(test)]
mod sm1_tests {
    use super::*;

    /// Create an SM1 sim pre-loaded with pixel count and data words.
    /// Data words are pixel-aligned: one 12-bit pixel per 32-bit word in bits [11:0].
    fn setup_sm1(pixel_count: u32, data_words: &[u32]) -> PioSim {
        let program = rgb444_pio_program();
        let mut sim = PioSim::new(&program);
        sim.autopull_threshold = 12;
        // First TX word: pixel count - 1 (consumed by `pull block` + `out x, 32`).
        sim.push_tx(pixel_count - 1);
        for &w in data_words {
            sim.push_tx(w);
        }
        sim
    }

    #[test]
    fn no_spurious_clock_edges() {
        // Key property: every SCK rising edge must be paired with a preceding
        // `out pins` instruction. No extra edges from irq/set/jmp instructions.
        let mut sim = setup_sm1(1, &[0x0000_0ABC]);
        sim.run(500);

        let sampled = sample_spi_bits(&sim);

        // With the pixel counter, SM1 outputs exactly 12 bits per pixel.
        assert_eq!(
            sampled.len(),
            12,
            "expected exactly 12 rising edges for 1 pixel, got {}",
            sampled.len()
        );

        // Verify: each rising SCK edge was preceded by exactly one `out pins`.
        let mut prev_sck: u8 = 0;
        let mut outs_since_last_edge = 0u32;

        for i in 0..sim.side_set_log.len() {
            let sck = sck_from_side(sim.side_set_log[i].unwrap_or(0));
            if sim.out_pin_log[i].is_some() {
                outs_since_last_edge += 1;
            }
            if prev_sck == 0 && sck == 1 {
                assert_eq!(
                    outs_since_last_edge, 1,
                    "step {}: expected exactly 1 `out` per rising SCK edge, got {}",
                    i, outs_since_last_edge
                );
                outs_since_last_edge = 0;
            }
            prev_sck = sck;
        }
    }

    #[test]
    fn correct_bit_order() {
        let pixel: u32 = 0b1010_1100_1111; // 0xACF
        let mut sim = setup_sm1(1, &[pixel]);
        sim.run(500);

        let sampled = sample_spi_bits(&sim);
        assert!(
            sampled.len() >= 12,
            "expected >= 12 bits, got {}",
            sampled.len()
        );
        for (i, &bit) in sampled.iter().enumerate().take(12) {
            let expected = (pixel >> i) & 1;
            assert_eq!(bit, expected, "bit {}: expected {}, got {}", i, expected, bit);
        }
    }

    #[test]
    fn cs_stays_active_during_transfer() {
        let mut sim = setup_sm1(1, &[0x0000_0ABC]);
        sim.run(500);

        for (i, side) in sim.side_set_log.iter().enumerate() {
            let cs = cs_from_side(side.unwrap_or(0));
            assert_eq!(
                cs, 0,
                "step {}: CSn must be 0 (active) during transfer, got {}",
                i, cs
            );
        }
    }

    #[test]
    fn data_setup_before_clock_rise() {
        let mut sim = setup_sm1(1, &[0x0000_0FFF]);
        sim.run(500);

        // For every rising SCK edge, verify that MOSI was set on a *prior*
        // step (the `out pins, 1 side 0` precedes the `nop side 2`).
        let mut prev_sck: u8 = 0;
        let mut mosi_set_prev = false;

        for i in 0..sim.side_set_log.len() {
            let side = sim.side_set_log[i].unwrap_or(0);
            let sck = sck_from_side(side);
            let has_out = sim.out_pin_log[i].is_some();

            if prev_sck == 0 && sck == 1 {
                assert!(
                    !has_out,
                    "step {}: data must be set up before rising SCK, not simultaneously",
                    i
                );
                assert!(
                    mosi_set_prev,
                    "step {}: MOSI should have been set on the preceding step",
                    i
                );
            }

            mosi_set_prev = has_out;
            prev_sck = sck;
        }
    }

    #[test]
    fn multi_pixel_spi_output() {
        // 3 pixels, each in its own pixel-aligned FIFO word.
        let mut sim = setup_sm1(3, &[0xFFF, 0x000, 0xA5A]);
        sim.run(2000);

        let sampled = sample_spi_bits(&sim);
        assert!(
            sampled.len() >= 36,
            "expected >= 36 clocked bits for 3 pixels, got {}",
            sampled.len()
        );

        let pixels = pixels_from_spi_bits(&sampled, 3);
        assert_eq!(
            pixels[0], 0xFFF,
            "pixel 0: expected 0xFFF, got 0x{:03X}",
            pixels[0]
        );
        assert_eq!(
            pixels[1], 0x000,
            "pixel 1: expected 0x000, got 0x{:03X}",
            pixels[1]
        );
        assert_eq!(
            pixels[2], 0xA5A,
            "pixel 2: expected 0xA5A, got 0x{:03X}",
            pixels[2]
        );
    }

    #[test]
    fn frame_done_irq_fires() {
        let mut sim = setup_sm1(2, &[0xFFF, 0x000]);
        sim.run(2000);

        // After all pixels, SM1 should have set IRQ 0 (frame done).
        let flags = sim.irq_flags.get();
        assert!(
            flags & 1 != 0,
            "IRQ 0 (frame done) should be set after all pixels, flags=0b{:08b}",
            flags
        );
    }

    #[test]
    fn pixel_counter_controls_length() {
        // Request exactly 4 pixels.
        let mut sim = setup_sm1(4, &[0x111, 0x222, 0x333, 0x444]);
        sim.run(5000);

        let sampled = sample_spi_bits(&sim);
        // Exactly 4 * 12 = 48 rising edges.
        assert_eq!(
            sampled.len(),
            48,
            "expected 48 rising edges for 4 pixels, got {}",
            sampled.len()
        );
    }
}

// ── End-to-end: SM0 → SM1 pipeline (dual-SM with IRQ handshake) ──────────

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod pipeline_tests {
    use super::*;

    /// Set up a DualSim with correct SM0/SM1 configuration for the display pipeline.
    fn setup_dual(
        color: MonochromeColor,
        sm0_data: &[u32],
        pixel_count: u32,
    ) -> DualSim {
        let sm0_program = gen_monochrome_pio_program(color);
        let sm1_program = rgb444_pio_program();
        let mut dual = DualSim::new(&sm0_program, &sm1_program);

        dual.sm0.isr_shift_left = true;
        dual.sm0.autopush_threshold = 12;
        dual.sm1.autopull_threshold = 12;

        for &w in sm0_data {
            dual.sm0.push_tx(w);
        }
        dual.sm1.push_tx(pixel_count - 1);

        dual
    }

    #[test]
    fn sm0_feeds_sm1_correctly() {
        // 8 pixels of bpp=4, values 0x0 through 0x7.
        let mut input: u32 = 0;
        for i in 0..8u32 {
            input |= i << (i * 4);
        }

        let mut dual = setup_dual(MonochromeColor::Bpp4, &[input], 8);
        dual.run(50_000);

        let sampled = sample_spi_bits(&dual.sm1);
        assert!(
            sampled.len() >= 8 * 12,
            "expected >= 96 clocked bits for 8 pixels, got {}",
            sampled.len()
        );

        let pixels = pixels_from_spi_bits(&sampled, 8);
        for px in 0..8 {
            let src_val = (input >> (px * 4)) & 0xF;
            let expected = (src_val | (src_val << 4) | (src_val << 8)) as u16;
            assert_eq!(
                pixels[px], expected,
                "pixel {}: src=0x{:X}, expected 0x{:03X}, got 0x{:03X}",
                px, src_val, expected, pixels[px]
            );
        }
    }

    #[test]
    fn full_frame_bpp1() {
        // Simulate a small "frame": 64 pixels at 1-bpp (2 words).
        // Checkerboard pattern: alternating black/white.
        let input = [0x5555_5555u32, 0xAAAA_AAAAu32];

        let mut dual = setup_dual(MonochromeColor::Bpp1, &input, 64);
        dual.run(500_000);

        let sampled = sample_spi_bits(&dual.sm1);
        assert!(
            sampled.len() >= 64 * 12,
            "expected >= 768 clocked bits for 64 pixels, got {}",
            sampled.len()
        );

        let pixels = pixels_from_spi_bits(&sampled, 64);
        for px in 0..64 {
            let word_idx = px / 32;
            let bit_idx = px % 32;
            let src_bit = (input[word_idx] >> bit_idx) & 1;
            let expected = if src_bit == 1 { 0xFFF } else { 0x000 };
            assert_eq!(
                pixels[px], expected,
                "pixel {}: src_bit={}, expected 0x{:03X}, got 0x{:03X}",
                px, src_bit, expected, pixels[px]
            );
        }
    }

    #[test]
    fn irq_handshake_prevents_overflow() {
        // SM0 should never get far ahead of SM1 thanks to the IRQ handshake.
        // Monitor SM0 RX FIFO depth throughout execution.
        let sm0_program = gen_monochrome_pio_program(MonochromeColor::Bpp1);
        let sm1_program = rgb444_pio_program();

        let flags = Rc::new(Cell::new(0u8));
        let mut sm0 = PioSim::new(&sm0_program);
        let mut sm1 = PioSim::new(&sm1_program);
        sm0.irq_flags = flags.clone();
        sm1.irq_flags = flags;
        sm0.isr_shift_left = true;
        sm0.autopush_threshold = 12;
        sm1.autopull_threshold = 12;

        // 32 pixels.
        sm0.push_tx(0xFFFF_FFFF);
        sm1.push_tx(31); // pixel count - 1

        let mut max_rx_depth = 0usize;

        for _ in 0..100_000u64 {
            // DMA transfer.
            while let Some(word) = sm0.rx_fifo.pop_front() {
                sm1.tx_fifo.push_back(word);
            }

            let r1 = sm1.step();
            let r0 = sm0.step();

            // Track SM0 RX FIFO depth (after SM0 step, before next DMA drain).
            max_rx_depth = max_rx_depth.max(sm0.rx_fifo.len());

            if r0 != StepResult::Ok && r1 != StepResult::Ok {
                break;
            }
        }

        // With IRQ handshake, SM0 can be at most ~1 pixel ahead.
        assert!(
            max_rx_depth <= 2,
            "SM0 RX FIFO depth should stay <= 2 with IRQ handshake, got {}",
            max_rx_depth
        );
    }

    #[test]
    fn frame_done_after_pipeline() {
        let mut dual = setup_dual(MonochromeColor::Bpp4, &[0x1234_5678], 8);
        dual.run(50_000);

        let flags = dual.sm0.irq_flags.get(); // shared with sm1
        assert!(
            flags & 1 != 0,
            "IRQ 0 (frame done) should be set after pipeline completes"
        );
    }
}

// ── lax_dma PIO tests ─────────────────────────────────────────────────────

#[cfg(test)]
mod lax_dma_tests {
    use super::*;

    #[test]
    fn invert_single_word() {
        let program = invert_pio_program();
        let mut sim = PioSim::new(&program);
        sim.push_tx(0x5555_5555);
        sim.run(100);

        let out = sim.pop_rx().expect("expected one output word");
        assert_eq!(out, 0xAAAA_AAAA, "expected bitwise invert of 0x55555555");
    }

    #[test]
    fn invert_multiple_words() {
        let program = invert_pio_program();
        let mut sim = PioSim::new(&program);
        let inputs: [u32; 4] = [0x0000_0000, 0xFFFF_FFFF, 0xDEAD_BEEF, 0x1234_5678];
        for &w in &inputs {
            sim.push_tx(w);
        }
        sim.run(1000);

        for &input in &inputs {
            let out = sim.pop_rx().unwrap();
            assert_eq!(out, !input, "invert of 0x{:08X} should be 0x{:08X}", input, !input);
        }
    }

    #[test]
    fn invert_twice_pipeline() {
        // SM0: invert → SM1: invert = original value.
        // SM0 uses `irq wait 4`, SM1 uses `wait 1 irq 4` to synchronize.
        let sm0_program = invert_pio_program();
        let sm1_program = invert_pio_again_program();
        let mut dual = DualSim::new(&sm0_program, &sm1_program);

        let inputs: [u32; 8] = [0x55555555; 8];
        for &w in &inputs {
            dual.sm0.push_tx(w);
        }
        dual.run(10_000);

        for &input in &inputs {
            let out = dual.sm1.pop_rx().expect("expected output word");
            assert_eq!(
                out, input,
                "double invert should restore original: 0x{:08X} != 0x{:08X}",
                out, input
            );
        }
    }

    #[test]
    fn invert_twice_varied_data() {
        let sm0_program = invert_pio_program();
        let sm1_program = invert_pio_again_program();
        let mut dual = DualSim::new(&sm0_program, &sm1_program);

        let inputs: [u32; 4] = [0x0000_0000, 0xFFFF_FFFF, 0xDEAD_BEEF, 0x1234_5678];
        for &w in &inputs {
            dual.sm0.push_tx(w);
        }
        dual.run(10_000);

        for &input in &inputs {
            let out = dual.sm1.pop_rx().expect("expected output word");
            assert_eq!(out, input, "double invert should restore 0x{:08X}", input);
        }
    }

    #[test]
    fn expand_times12_all_ones() {
        // Input: 0xFFFFFFFF (32 bits, all 1). Each bit → twelve 1s.
        // 32 pixels × 12 bits = 384 bits = 12 words of 32 bits.
        let program = gen_monochrome_expand_program(MonochromeColor::Bpp1);
        let mut sim = PioSim::new(&program);
        sim.push_tx(0xFFFF_FFFF);
        sim.run(10_000);

        let mut output = Vec::new();
        while let Some(w) = sim.pop_rx() {
            output.push(w);
        }
        assert_eq!(output.len(), 12, "expected 12 output words for 32 pixels × 12 bits");
        for (i, &w) in output.iter().enumerate() {
            assert_eq!(w, 0xFFFF_FFFF, "word {}: expected all 1s, got 0x{:08X}", i, w);
        }
    }

    #[test]
    fn expand_times12_all_zeros() {
        let program = gen_monochrome_expand_program(MonochromeColor::Bpp1);
        let mut sim = PioSim::new(&program);
        sim.push_tx(0x0000_0000);
        sim.run(10_000);

        let mut output = Vec::new();
        while let Some(w) = sim.pop_rx() {
            output.push(w);
        }
        assert_eq!(output.len(), 12, "expected 12 output words");
        for (i, &w) in output.iter().enumerate() {
            assert_eq!(w, 0x0000_0000, "word {}: expected all 0s, got 0x{:08X}", i, w);
        }
    }

    #[test]
    fn expand_times12_alternating() {
        // Input: 0x5A5A5A5A. Each bit expanded to 12.
        // Verify by extracting 12-bit groups from the output bit stream.
        let program = gen_monochrome_expand_program(MonochromeColor::Bpp1);
        let mut sim = PioSim::new(&program);
        let input: u32 = 0x5A5A_5A5A;
        sim.push_tx(input);
        sim.run(10_000);

        let mut output = Vec::new();
        while let Some(w) = sim.pop_rx() {
            output.push(w);
        }
        assert_eq!(output.len(), 12, "expected 12 output words");

        // Shift-right ISR packs bits LSB-first: first IN result lands at
        // bit 0 of the final output word. Extract the bit stream accordingly.
        let mut bit_stream: Vec<u8> = Vec::new();
        for &w in &output {
            for bit in 0..32 {
                bit_stream.push(((w >> bit) & 1) as u8);
            }
        }

        for px in 0..32 {
            let src_bit = (input >> px) & 1;
            let start = px * 12;
            for i in 0..12 {
                assert_eq!(
                    bit_stream[start + i] as u32, src_bit,
                    "pixel {} bit {}: expected {}, got {}",
                    px, i, src_bit, bit_stream[start + i]
                );
            }
        }
    }

    #[test]
    fn expand_matches_gen_monochrome_bpp1() {
        // expand_times12 is the precursor to gen_monochrome_pio_program(Bpp1).
        // Both should produce the same output for the same input (ignoring
        // the wait instruction and ISR shift direction difference).
        //
        // expand_times12: shift-right ISR, autopush at 32
        // gen_monochrome: shift-left ISR, autopush at 12
        //
        // We compare at the pixel level: both should produce the same
        // sequence of 12-bit pixel values.
        let input: u32 = 0xA5A5_A5A5;

        // Run expand_times12 (shift-right, autopush 32)
        let expand_prog = gen_monochrome_expand_program(MonochromeColor::Bpp1);
        let mut expand_sim = PioSim::new(&expand_prog);
        expand_sim.push_tx(input);
        expand_sim.run(10_000);

        let mut expand_words = Vec::new();
        while let Some(w) = expand_sim.pop_rx() {
            expand_words.push(w);
        }

        // Extract pixels from shift-right packed 32-bit words (LSB-first).
        let mut expand_pixels = Vec::new();
        let mut bit_stream: Vec<u8> = Vec::new();
        for &w in &expand_words {
            for bit in 0..32 {
                bit_stream.push(((w >> bit) & 1) as u8);
            }
        }
        for px in 0..32 {
            let mut pixel: u16 = 0;
            for i in 0..12 {
                pixel |= (bit_stream[px * 12 + i] as u16) << i;
            }
            expand_pixels.push(pixel);
        }

        // Run gen_monochrome (shift-left, autopush 12)
        let mono_words = run_color_expand(MonochromeColor::Bpp1, &[input], 10_000);
        let mono_pixels = extract_rgb444_pixels(&mono_words);

        assert_eq!(mono_pixels.len(), expand_pixels.len());
        for px in 0..32 {
            assert_eq!(
                mono_pixels[px], expand_pixels[px],
                "pixel {}: mono=0x{:03X} expand=0x{:03X}",
                px, mono_pixels[px], expand_pixels[px]
            );
        }
    }
}

// ── Frame timing tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod timing_tests {
    use super::*;

    const PIO_CLK: u32 = 125_000_000; // 125 MHz default RP2040

    #[test]
    fn timing_320x240_at_125mhz() {
        let t = FrameTiming::new(320, 240, PIO_CLK);
        assert_eq!(t.pixel_count, 76_800);
        assert_eq!(t.pixel_cycles, 39 * 76_800);
        assert_eq!(t.total_cycles, 39 * 76_800 + 3);
        assert_eq!(t.spi_clock_hz, 41_666_666);
        // 2_995_203 cycles / 125 MHz = ~23.96 ms → ~41 FPS
        assert_eq!(t.frame_us, 23_961);
        assert_eq!(t.fps, 41);
    }

    #[test]
    fn timing_240x135_at_125mhz() {
        let t = FrameTiming::new(240, 135, PIO_CLK);
        assert_eq!(t.pixel_count, 32_400);
        // ~10.1 ms → ~99 FPS
        assert!(t.fps >= 98 && t.fps <= 100, "fps={}", t.fps);
    }

    #[test]
    fn timing_240x240_at_125mhz() {
        let t = FrameTiming::new(240, 240, PIO_CLK);
        assert_eq!(t.pixel_count, 57_600);
        // ~17.97 ms → ~55 FPS
        assert!(t.fps >= 54 && t.fps <= 56, "fps={}", t.fps);
    }

    #[test]
    fn simulated_cycles_match_estimate() {
        // Run the actual SM1 program for a small pixel count and verify
        // the cycle count matches the formula.
        let pixel_count: u32 = 16;
        let program = rgb444_pio_program();
        let mut sim = PioSim::new(&program);
        sim.autopull_threshold = 12;

        // Load pixel count - 1 (frame setup word).
        sim.push_tx(pixel_count - 1);
        // Each pixel autopulls one 32-bit word (threshold 12, rest discarded).
        for _ in 0..pixel_count {
            sim.push_tx(0xAAAA_AAAA);
        }

        sim.run(100_000);

        // SM1 should stall on pull-block after the frame-done IRQ,
        // having executed: 2 (setup) + 39*16 (pixels) + 1 (irq) = 627 cycles.
        let expected = 39 * pixel_count as u64 + 3;
        assert_eq!(
            sim.steps, expected,
            "simulated {} cycles, expected {}",
            sim.steps, expected
        );
    }
}
