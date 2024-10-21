use defmt::*;
use defmt_rtt as _;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address};
use embassy_rp::clocks::RoscRng;
use embassy_time::Duration;
use embedded_io_async::Write;
use embedded_nal_async::{AddrType, Dns, IpAddr};
use embedded_tls::{
    Aes128GcmSha256, TlsConfig, TlsConnection, TlsContext, TlsError, UnsecureProvider,
};
use panic_probe as _;
use rand::{RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::Deserialize;

const HOST: &str = env!("HOST");
const REQ_HEADERS: &[u8] = b"GET /repos/kromych/pico-github-stars HTTP/1.1\r\n\
Host: api.github.com\r\n\
User-Agent: kromych/pico-github-stars\r\n\
Accept: application/vnd.github+json\r\n\
X-GitHub-Api-Version:2022-11-28\r\n\
Connection: close\r\n\
\r\n";

fn process_body(body: &str) -> Option<u32> {
    debug!("Response body: {:?}", &body);

    #[derive(Deserialize)]
    struct ApiResponse {
        stargazers_count: u32,
        // other fields as needed
    }

    let bytes = body.as_bytes();
    match serde_json_core::de::from_slice::<ApiResponse>(bytes) {
        Ok((output, _used)) => {
            info!("STARS: {:?}", output.stargazers_count);
            Some(output.stargazers_count)
        }
        Err(_e) => {
            error!("Failed to parse response body");
            None
        }
    }
}

async fn call_rest_api(stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>, seed: u64) {
    let dns_client = DnsSocket::new(stack);

    let mut rx_buffer = [0; 8192];
    let mut tx_buffer = [0; 8192];

    debug!("resolving {}", HOST);
    let remote_addr = dns_client.get_host_by_name(HOST, AddrType::IPv4).await;
    let remote_addr = match remote_addr {
        Ok(addr) => addr,
        Err(e) => {
            error!("DNS error: {:?}", e);
            return;
        }
    };

    let remote_endpoint = match remote_addr {
        IpAddr::V4(addr) => IpEndpoint::new(IpAddress::Ipv4(Ipv4Address(addr.octets())), 443),
        _ => defmt::unreachable!("IPv6 not supported"),
    };
    debug!("connecting to {}", remote_endpoint);

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    let r = socket.connect(remote_endpoint).await;
    if let Err(e) = r {
        warn!("connect error: {:?}", e);
        return;
    }
    log::debug!("TCP connected!");

    let mut read_record_buffer = [0; 16640];
    let mut write_record_buffer = [0; 16640];
    let config = TlsConfig::new();
    let mut tls = TlsConnection::new(socket, &mut read_record_buffer, &mut write_record_buffer);

    if let Err(e) = tls
        .open(TlsContext::new(
            &config,
            UnsecureProvider::new::<Aes128GcmSha256>(ChaCha8Rng::seed_from_u64(seed)),
        ))
        .await
    {
        error!("TLS error: {:?}", e);
        return;
    }

    if let Err(e) = tls.write_all(REQ_HEADERS).await {
        error!("write error: {:?}", e);
        return;
    }
    if let Err(e) = tls.flush().await {
        error!("flush error: {:?}", e);
        return;
    }

    let mut rx_buf = [0; 16384];
    let sz = {
        let mut pos = 0;
        loop {
            let sz = match tls.read(&mut rx_buf[pos..]).await {
                Ok(sz) => sz,
                Err(TlsError::ConnectionClosed) => {
                    debug!("Connection closed by the server");
                    break;
                }
                Err(e) => {
                    error!("read error: {:?}", e);
                    return;
                }
            };
            if sz == 0 {
                break;
            }
            pos += sz;
        }
        info!("Read {} bytes from the server", pos);
        pos
    };

    let mut pos = 0;
    let mut content_length: Option<usize> = None;
    while pos < sz {
        let tail = &rx_buf[pos..];
        let line_end = match tail.iter().position(|&c| c == b'\n') {
            Some(i) => i,
            None => {
                warn!("no newline found");
                break;
            }
        };

        let line = &rx_buf[pos..pos + line_end];
        let line = match core::str::from_utf8(line) {
            Ok(line) => line,
            Err(_) => {
                warn!("invalid utf8");
                continue;
            }
        };
        if !line.ends_with('\r') {
            warn!("no CR found");
            return;
        }
        let line = &line[..line.len() - 1];

        if pos == 0 {
            if !line.starts_with("HTTP/1.1 2") {
                error!("HTTP error: {:?}", line);
            } else {
                debug!("HTTP status: {:?}", line);
            }
        } else if !line.is_empty() {
            debug!("HTTP header: {:?}", line);
            if let Some(cl) = line.strip_prefix("Content-Length: ") {
                if let Ok(cl) = cl.parse() {
                    content_length = Some(cl);
                    debug!("Content-Length: {}", cl);
                } else {
                    error!("invalid Content-Length header {:?}", line);
                    return;
                }
            } else if line.starts_with("Transfer-Encoding: chunked") {
                error!("Transfer-Encoding: chunked is not supported");
                return;
            } else if line.starts_with("Content-Type: ") && !line.contains("application/json") {
                error!("Content-Type is not application/json");
                return;
            }
        }

        // Skip the CRLF.
        pos += line.len() + 2;

        if line.is_empty() {
            debug!("Size of headers: {}", pos);
            break;
        }
    }

    match content_length {
        Some(cl) => {
            if cl + pos != sz {
                error!("Content-Length is not equal to the actual content size");
                return;
            }
        }
        None => {
            error!("Content-Length not found");
            return;
        }
    };

    let body = if let Ok(body) = core::str::from_utf8(&rx_buf[pos..sz]) {
        body
    } else {
        error!("invalid utf8 in body");
        return;
    };

    process_body(body);
}

pub async fn read_data(stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>) {
    let mut rng = RoscRng;
    let seed = rng.next_u64();
    call_rest_api(stack, seed).await;
}
