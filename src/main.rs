#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpAddress, IpEndpoint, Ipv4Address};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use embedded_nal_async::{AddrType, Dns, IpAddr};
use embedded_tls::{Aes128GcmSha256, TlsConfig, TlsConnection, TlsContext, UnsecureProvider};
use panic_probe as _;
use rand::{RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::Deserialize;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const DEFAULT_SLEEP_TIME_SEC: u64 = 60;
const HOST: &str = env!("HOST");
const REQ_HEADERS: &[u8] = b"GET /repos/kromych/pico-github-stars HTTP/1.1\r\n\
Host: api.github.com\r\n\
User-Agent: kromych/pico-github-stars\r\n\
Accept: application/vnd.github+json\r\n\
X-GitHub-Api-Version:2022-11-28\r\n\
Connection: close\r\n\
\r\n";

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(runner: &'static embassy_net::Stack<cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[allow(dead_code)]
fn process_data(body: &str) {
    info!("Response body: {:?}", &body);

    #[derive(Deserialize)]
    struct ApiResponse {
        stargazers_count: u32,
        // other fields as needed
    }

    let bytes = body.as_bytes();
    match serde_json_core::de::from_slice::<ApiResponse>(bytes) {
        Ok((output, _used)) => {
            info!("STARS: {:?}", output.stargazers_count);
        }
        Err(_e) => {
            error!("Failed to parse response body");
        }
    }
}

// Static cells for holding state, resources, and stack
static STATE: StaticCell<cyw43::State> = StaticCell::new();
static RESOURCES: StaticCell<embassy_net::StackResources<32>> = StaticCell::new();
static STACK: StaticCell<embassy_net::Stack<cyw43::NetDriver>> = StaticCell::new();

fn initialize_peripherals() -> (Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>, RoscRng) {
    let p = embassy_rp::init(Default::default());
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );
    let rng = RoscRng;
    (pwr, spi, rng)
}

async fn initialize_network_stack(
    spawner: Spawner,
    pwr: Output<'static>,
    spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    rng: &mut RoscRng,
) -> &'static embassy_net::Stack<cyw43::NetDriver<'static>> {
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    loop {
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

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

async fn read_data_from_rest_api(
    stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>,
    seed: u64,
) {
    let dns_client = DnsSocket::new(stack);

    let mut rx_buffer = [0; 8192];
    let mut tx_buffer = [0; 8192];

    info!("resolving {}", HOST);
    let remote_addr = dns_client
        .get_host_by_name(HOST, AddrType::IPv4)
        .await
        .unwrap();

    let remote_endpoint = match remote_addr {
        IpAddr::V4(addr) => IpEndpoint::new(IpAddress::Ipv4(Ipv4Address(addr.octets())), 443),
        _ => defmt::unreachable!("IPv6 not supported"),
    };
    info!("connecting to {}", remote_endpoint);

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    let r = socket.connect(remote_endpoint).await;
    if let Err(e) = r {
        warn!("connect error: {:?}", e);
        return;
    }
    log::info!("TCP connected!");

    let mut read_record_buffer = [0; 16640];
    let mut write_record_buffer = [0; 16640];
    let config = TlsConfig::new();
    let mut tls = TlsConnection::new(socket, &mut read_record_buffer, &mut write_record_buffer);

    tls.open(TlsContext::new(
        &config,
        UnsecureProvider::new::<Aes128GcmSha256>(ChaCha8Rng::seed_from_u64(seed)),
    ))
    .await
    .expect("error establishing TLS connection");

    tls.write_all(REQ_HEADERS)
        .await
        .expect("error writing data");
    tls.flush().await.expect("error flushing data");

    let mut rx_buf = [0; 16384];
    let sz = tls.read(&mut rx_buf[..]).await.expect("error reading data");

    info!("Read {} bytes", sz);
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
        read_data_from_rest_api(stack, rng.next_u64()).await;
        Timer::after(Duration::from_secs(
            env!("SLEEP_TIME_SEC")
                .parse()
                .unwrap_or(DEFAULT_SLEEP_TIME_SEC),
        ))
        .await;
    }
}
