# pico-github-stars

GitHub stars display for the Raspberry Pi Pico with Pimoroni Pico Display.

Uses Embassy async runtime with PIO-based SPI and DMA for display output.

## Supported hardware

- **RP2040** (Raspberry Pi Pico) -- default
- **RP2350** (Raspberry Pi Pico 2, Cortex-M33 variant)

## Prerequisites

- **Rust** -- install via [rustup](https://rustup.rs/). The `rust-toolchain.toml`
  file will automatically install the correct toolchain and targets
  (`thumbv6m-none-eabi` for RP2040, `thumbv8m.main-none-eabihf` for RP2350).

- **probe-rs** -- used to flash and debug over SWD:

  ```sh
  cargo install probe-rs-tools
  ```

- **Hardware** -- a Raspberry Pi Pico (or Pico 2) with a
  [Pimoroni Pico Display 2.8](https://shop.pimoroni.com/products/pico-display-pack-2-8),
  connected to a debug probe (e.g. a second Pico running
  [Picoprobe](https://github.com/raspberrypi/picoprobe) / debugprobe firmware).

## Building

```sh
cargo build --release
cargo run --release           # flash via probe-rs
```

### Examples

```sh
cargo run --release --example matrix
cargo run --release --example matrix_async
cargo run --release --example color_bars
cargo run --release --example color_bars_async
cargo run --release --example bounce
cargo run --release --example bounce_async
cargo run --release --example dma_tests
```

## Selecting the target chip

The project defaults to RP2040. To build for RP2350, three things need to change:

### 1. Cargo features

Build with the `rp2350` feature instead of the default `rp2040`:

```sh
cargo build --release --no-default-features --features rp2350
```

### 2. `.cargo/config.toml`

Switch the build target and probe-rs chip:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = ["probe-rs", "run", "--log-format", "{L} {s}", "--chip", "RP2350", "--protocol", "swd", "--speed", "16000"]

[build]
target = "thumbv8m.main-none-eabihf"    # Cortex-M33
```

### 3. Linker script

Replace `memory.x` with the RP2350 variant:

```sh
cp memory_rp2350.x memory.x
```

The RP2350 linker script provides 520K RAM (vs 264K) and 4MB flash (vs 2MB).

## Testing

There are no host-side unit tests -- the code targets `no_std` embedded hardware.
All testing is done on-device:

- **`dma_tests` example** -- exercises the `LaxDmaWrite` DMA driver with
  memory-to-memory transfers (various word sizes, byte-swap modes) and
  PIO-driven pipelines (invert, monochrome expansion). Results are printed
  over defmt RTT. Run with:

  ```sh
  cargo run --release --example dma_tests
  ```

- **Visual examples** (`matrix`, `color_bars`, `bounce` and their `_async`
  variants) -- verify the full PIO/DMA display pipeline. A working display
  confirms correct PIO program relocation, DMA triggering, and frame
  synchronisation.

- **`pio-test` crate** -- a host-side PIO instruction simulator for testing
  PIO programs without hardware. Currently a work in progress:

  ```sh
  cargo test -p pio-test
  ```

## Project structure

```
.
+-- src/                  Main binary (GitHub stars display)
+-- examples/             Standalone demo examples
+-- pico-display/         Display driver library (PIO + DMA)
+-- pio-programs/         PIO program definitions
+-- memory.x              RP2040 linker script (264K RAM, 2MB flash)
+-- memory_rp2350.x       RP2350 linker script (520K RAM, 4MB flash)
```
