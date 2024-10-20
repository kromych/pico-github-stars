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
fn process_data(body: &str) {
    info!("Response body: {:?}", &body);

    // Parse the response body

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

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

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());
    // Use static IP configuration instead of DHCP
    //let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = rng.next_u64();

    static RESOURCES: StaticCell<embassy_net::StackResources<32>> = StaticCell::new();
    static STACK: StaticCell<embassy_net::Stack<cyw43::NetDriver>> = StaticCell::new();
    let stack = &*STACK.init(embassy_net::Stack::new(
        net_device,
        config,
        RESOURCES.init(embassy_net::StackResources::new()),
        seed,
    ));
    // Launch network task that runs `stack.run().await`
    spawner.spawn(net_task(stack)).unwrap();

    loop {
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
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

    let dns_client = DnsSocket::new(stack);
    loop {
        let mut rx_buffer = [0; 8192];
        let mut tx_buffer = [0; 8192];

        info!("resolving {}", &HOST);
        let remote_addr = dns_client
            .get_host_by_name(HOST, AddrType::IPv4)
            .await
            .unwrap();
        let remote_endpoint = match remote_addr {
            IpAddr::V4(addr) => IpEndpoint::new(IpAddress::Ipv4(Ipv4Address(addr.octets())), 443),
            _ => defmt::unreachable!("IPv6 not supported"),
        };
        info!("connecting to {}", &HOST);

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_timeout(Some(Duration::from_secs(10)));

        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            warn!("connect error: {:?}", e);
            return;
        }
        log::info!("TCP connected!");

        let mut read_record_buffer = [0; 16384];
        let mut write_record_buffer = [0; 16384];
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

        let mut rx_buf = [0; 128];
        let sz = tls.read(&mut rx_buf[..]).await.expect("error reading data");

        log::info!("Read {} bytes: {:?}", sz, &rx_buf[..sz]);

        Timer::after(Duration::from_secs(5)).await;
    }
}
