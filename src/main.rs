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
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use rand::RngCore;
use static_cell::StaticCell;

mod float;
mod pico_display;
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

    let rng = RoscRng;
    (pwr, wifi_spi, rng)
}

async fn initialize_peripherals_no_net() {
    let peripherals = embassy_rp::init(Default::default());

    let mosi = peripherals.PIN_19;
    let dc = embassy_rp::gpio::Output::new(peripherals.PIN_16, embassy_rp::gpio::Level::Low);
    let clk = peripherals.PIN_18;
    let display_cs =
        embassy_rp::gpio::Output::new(peripherals.PIN_17, embassy_rp::gpio::Level::Low);

    let bl_pwm = embassy_rp::pwm::Pwm::new_output_a(
        peripherals.PWM_SLICE2,
        peripherals.PIN_20,
        embassy_rp::pwm::Config::default(),
    );
    let tx_dma = peripherals.DMA_CH0;
    let mut display = pico_display::PicoDisplay::new(
        pico_display::DisplayKind::PicoDisplay2_0,
        pico_display::DisplayRotation::Rotate0,
        peripherals.SPI0,
        clk,
        mosi,
        tx_dma,
        display_cs,
        dc,
        bl_pwm,
    )
    .await;
    display.clear(pico_display::RGB565::white()).await;
    loop {
        let sleep_sec = 1;
        info!("Sleeping for {} seconds", sleep_sec);
        pico_display::delay_ms(1000);
    }
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

async fn run(spawner: Spawner) {
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!(
        "project: {}, version: {}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    initialize_peripherals_no_net().await;
    loop {
        let sleep_sec = 1;
        info!("Sleeping for {} seconds", sleep_sec);
        Timer::after(Duration::from_secs(sleep_sec)).await;
    }
}
