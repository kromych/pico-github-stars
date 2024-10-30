//! This is the main file for the project.
//!
//! * It initializes the peripherals, the network stack, and then
//! * enters a loop where it calls data processing functions.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::Config;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, Peripheral};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use rand::RngCore;
use static_cell::StaticCell;

mod process_data;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const DEFAULT_SLEEP_TIME_SEC: u64 = 60;

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, cyw43_pio::PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(runner: &'static embassy_net::Stack<cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

// Static cells for holding state, resources, and stack
static STATE: StaticCell<cyw43::State> = StaticCell::new();
static RESOURCES: StaticCell<embassy_net::StackResources<32>> = StaticCell::new();
static STACK: StaticCell<embassy_net::Stack<cyw43::NetDriver>> = StaticCell::new();

fn initialize_peripherals() -> (
    Output<'static>,
    cyw43_pio::PioSpi<'static, PIO0, 0, DMA_CH0>,
    RoscRng,
) {
    let peripherals = embassy_rp::init(Default::default());
    let pwr = Output::new(peripherals.PIN_23, Level::Low);
    let wifi_cs = Output::new(peripherals.PIN_25, Level::High);
    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let wifi_spi = cyw43_pio::PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        wifi_cs,
        peripherals.PIN_24,
        peripherals.PIN_29,
        peripherals.DMA_CH0,
    );

    let miso = unsafe { peripherals.PIN_16.clone_unchecked() };
    let rst = embassy_rp::gpio::Output::new(peripherals.PIN_15, embassy_rp::gpio::Level::Low);

    let dc = embassy_rp::gpio::Output::new(peripherals.PIN_16, embassy_rp::gpio::Level::Low);
    let display_cs =
        embassy_rp::gpio::Output::new(peripherals.PIN_17, embassy_rp::gpio::Level::High);
    let clk = peripherals.PIN_18;
    let mosi = peripherals.PIN_19;

    let mut display_config = embassy_rp::spi::Config::default();
    display_config.frequency = 64_000_000;
    display_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = embassy_rp::spi::Polarity::IdleHigh;

    let spi: embassy_rp::spi::Spi<'_, _, embassy_rp::spi::Blocking> =
        embassy_rp::spi::Spi::new_blocking(
            peripherals.SPI0,
            clk,
            mosi,
            miso,
            display_config.clone(),
        );
    let spi_bus: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        _,
    > = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(spi));

    let display_spi = embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig::new(
        &spi_bus,
        display_cs,
        display_config,
    );

    let di = display_interface_spi::SPIInterface::new(display_spi, dc);
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7789, di)
        .orientation(mipidsi::options::Orientation::new().flip_horizontal())
        .display_size(240, 320)
        .invert_colors(mipidsi::options::ColorInversion::Normal)
        .reset_pin(rst)
        .init(&mut embassy_time::Delay)
        .unwrap();
    // Backlight pin
    let _ = embassy_rp::gpio::Output::new(peripherals.PIN_20, embassy_rp::gpio::Level::High);

    use embedded_graphics::draw_target::DrawTarget;
    display
        .clear(embedded_graphics::pixelcolor::RgbColor::GREEN)
        .unwrap();

    let rng = RoscRng;
    (pwr, wifi_spi, rng)
}

async fn initialize_network_stack(
    spawner: Spawner,
    pwr: Output<'static>,
    spi: cyw43_pio::PioSpi<'static, PIO0, 0, DMA_CH0>,
    rng: &mut RoscRng,
) -> &'static embassy_net::Stack<cyw43::NetDriver<'static>> {
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    info!("Initializing CYW43...");

    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    info!("CYW43 initialized!");
    info!("Initializing CYW43 firmware...");

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!("CYW43 firmware initialized!");
    info!("Joining WiFi network...");

    loop {
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                warn!("join failed with status={}", err.status);
            }
        }
    }

    info!("WiFi network joined!");

    let seed = rng.next_u64();
    let stack = &*STACK.init(embassy_net::Stack::new(
        net_device,
        Config::dhcpv4(Default::default()),
        RESOURCES.init(embassy_net::StackResources::new()),
        seed,
    ));

    spawner.spawn(net_task(stack)).unwrap();
    stack
}

async fn wait_for_network(stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>) {
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("DHCP is now up!");

    info!("waiting for link up...");
    while !stack.is_link_up() {
        Timer::after_millis(500).await;
    }
    info!("Link is up!");

    info!("waiting for stack to be up...");
    stack.wait_config_up().await;
    info!("Stack is up!");
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!(
        "project: {}, version: {}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let (pwr, spi, mut rng) = initialize_peripherals();
    let stack = initialize_network_stack(spawner, pwr, spi, &mut rng).await;
    wait_for_network(stack).await;

    loop {
        process_data::read_data(stack).await;

        let sleep_sec = env!("SLEEP_TIME_SEC")
            .parse()
            .unwrap_or(DEFAULT_SLEEP_TIME_SEC);

        // Randomize sleep time to hopefully make it easier for the server.
        let sleep_sec = sleep_sec + (rng.next_u32() % (DEFAULT_SLEEP_TIME_SEC as u32)) as u64;
        info!("Sleeping for {} seconds", sleep_sec);
        Timer::after(Duration::from_secs(sleep_sec)).await;
    }
}
