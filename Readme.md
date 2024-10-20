# Pico shows Github stars!

About to be tested. To adjust Wi-Fi and others settings, update [the configuration](.cargo/config.toml).

With MicroPython that would be
```python
import network
import socket
import ssl
import json

ssid = "<Wi-Fi network name>"
password = "<Wi-Fi password>"

user_or_org = "kromych"
repo = "pico-github-stars"
hostname = "api.github.com"
port = 443
req_headers = (f"GET /repos/{user_or_org}/{repo} HTTP/1.1\r\n" +
       "Host: api.github.com\r\n" +
       "User-Agent: kromych/pico-github-stars\r\n" +
       "Accept: application/vnd.github+json\r\n" +
       "X-GitHub-Api-Version:2022-11-28\r\n" +
       "Connection: close\r\n" +
       "\r\n")

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)
wlan.ifconfig()

addr = socket.getaddrinfo(hostname, port)[0][4]
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(10)

sock.connect(addr)
ssock = ssl.wrap_socket(sock)

print(ssock.cipher())

ssock.write(req_headers.encode())

response = b""
while True:
    data = ssock.read(2048)
    if len(data) < 1:
        break
    response += data

headers_end = response.find(b"\r\n\r\n")
(headers, body) = (response[:headers_end], response[headers_end+4:])

# print("Headers:")
# print(headers)
# print("Body:")
# print(body)

response_json = json.loads(body)
print("star gazers:", response_json["stargazers_count"])
```

With `wget` could be:
```bash
wget -dvO - --secure-protocol=TLSv1_3 https://api.github.com/repos/kromych/pico-github-stars
```

An option with `curl`:
```bash
curl -v --http1.1 --tlsv1.2 https://api.github.com/repos/kromych/pico-github-stars
```

Employing `openssl`:
```bash
# Removing "-quiet" prints detailed data about the ciphers and handshake
(echo -ne "GET /repos/kromych/pico-github-stars HTTP/1.1\r\n\
Host: api.github.com\r\n\
User-Agent: kromych/pico-github-stars\r\n\
Accept: application/vnd.github+json\r\n\
X-GitHub-Api-Version:2022-11-28\r\n\
Connection: close\r\n\r\n") | openssl s_client -quiet -tls1_3 -connect api.github.com:443
```

Finally, with Python:

```python
import socket
import ssl
import json

user_or_org = "kromych"
repo = "pico-github-stars"
hostname = "api.github.com"
port = 443
req_headers = (f"GET /repos/{user_or_org}/{repo} HTTP/1.1\r\n" +
       "Host: api.github.com\r\n" +
       "User-Agent: kromych/pico-github-stars\r\n" +
       "Accept: application/vnd.github+json\r\n" +
       "X-GitHub-Api-Version:2022-11-28\r\n" +
       "Connection: close\r\n" +
       "\r\n")

response = b""
context = ssl.create_default_context()
with socket.create_connection((hostname, port)) as sock:
    with context.wrap_socket(sock, server_hostname=hostname) as ssock:
        print(ssock.version(), ssock.cipher())
        ssock.send(req_headers.encode())
        while True:
            data = ssock.read(2048)
            if len(data) < 1:
                break
            response += data

headers_end = response.find(b"\r\n\r\n")
(headers, body) = (response[:headers_end], response[headers_end+4:])

# print("Headers:")
# print(headers)
# print("Body:")
# print(body)

response_json = json.loads(body)
print("star gazers:", response_json["stargazers_count"])
```
