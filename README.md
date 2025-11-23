<p align="center">
  <img src="https://raw.githubusercontent.com/mobizt/SerialNetworkBridge/refs/heads/main/assets/logo.svg" width="600" alt="SerialNetworkBridge Logo">
</p>

<h1 align="center">SerialNetworkBridge</h1>

<p align="center">
  <a href="https://www.arduino.cc/">
    <img src="https://img.shields.io/badge/Arduino-Library-blue.svg" alt="Arduino Library">
  </a>
  <a href="LICENSE">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT">
  </a>
</p>

<p align="center">
  <b>The Arduino Serial bridge for TCP, UDP, and WebSocket Clients</b><br>
  Provides a simple way to use advanced network functionality over a serial link, enabling boards without native networking to communicate through a WiFi‑capable device.
</p>

<p align="center">It is designed for Arduino boards such as AVR, STM32, and Teensy that lack built‑in WiFi or Ethernet, offering a straightforward alternative to firmware‑based solutions. By bridging communication through modules like ESP32, ESP8266, Raspberry Pi Pico W, or MKR WiFi 1010, the library makes network access broadly available. With support for <b>SSL/TLS</b>, <b>WebSockets</b>, and <b>UDP</b>, SerialNetworkBridge enables secure communication without requiring firmware‑level certificate management.</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/mobizt/SerialNetworkBridge/refs/heads/main/assets/diagram.svg" alt="SerialNetworkBridge communication flow" width="800"/>
</p>

---

## ✨ Features

- **Multi-Protocol Support:** Bridge **TCP**, **UDP**, and **WebSocket** clients via serial.
- **Hardware Agnostic:** Designed for any Arduino board with a HardwareSerial port.
- **Secure:** Support for **SSL/TLS** (HTTPS/WSS) and **STARTTLS** upgrades handled by the host.
- **Lightweight:** Header-only design for embedded use.
- **Event-Driven:** WebSocket implementation supports async events and callbacks.

---

## 📦 Installation

### Arduino IDE  
1. Open Arduino IDE.  
2. Go to **Sketch → Include Library → Manage Libraries…** 3. Search for **SerialNetworkBridge**.  
4. Click **Install**.  

### PlatformIO  
1. Open your project’s `platformio.ini`.  
2. Add the library under `lib_deps`:  

```ini
lib_deps =
    mobizt/SerialNetworkBridge
```

---

## 🛠 Supported Platforms

**Clients (Non-Networked Boards):**
- Arduino AVR (Uno, Mega2560, Nano)
- STM32 series
- Teensy boards
- Any board with `HardwareSerial`

**Hosts (Network Bridges):**
- ESP32 / ESP8266
- Raspberry Pi Pico W
- MKR WiFi 1010, MKR 1000 WiFi
- Arduino UNO WiFi Rev2

---

## 🔌 Getting Started & Wiring

### Logic Level Warning ⚠️

Most Arduino AVR boards (Uno, Mega) operate at **5V**, while ESP32/ESP8266 modules operate at **3.3V**.

* **Arduino TX (5V) → ESP32 RX (3.3V):** You **MUST** use a logic level converter or a voltage divider (e.g., 10kΩ + 20kΩ resistors).
* **ESP32 TX (3.3V) → Arduino RX (5V):** This is usually safe directly.

### Wiring Diagram (Arduino Uno ↔ ESP32)

| Arduino Uno (Client) | Connection | ESP32 (Host) | Note |
| :--- | :---: | :--- | :--- |
| **Pin 2 (RX)** | ← | **Pin 17 (TX)** | Direct connection usually OK |
| **Pin 3 (TX)** | → | **Pin 16 (RX)** | **Use Level Shifter (5V to 3.3V)** |
| **GND** | ↔ | **GND** | Common Ground is required |

---

## 🚀 Usage

See the [`examples`](/examples/) folder for full sketches:

### 1. TCP Client Example (HTTP GET)

```cpp
#define ENABLE_SERIALTCP_DEBUG 
#include <SerialTCPClient.h>

SerialTCPClient client(Serial2, 0 /* slot */); 

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // Link to Host

  if (client.connect("httpbin.org", 80)) {
    client.println("GET /get HTTP/1.1");
    client.println("Host: httpbin.org");
    client.println("Connection: close");
    client.println();

    while (client.available() == 0) delay(0);

    while (client.available()) {
      Serial.write(client.read());
    }
    client.stop();
  }
}

void loop() {}
```

### 2. UDP Client Example (NTP)

```cpp
#define ENABLE_SERIALTCP_DEBUG
#include <SerialUDPClient.h>

SerialUDPClient udp(Serial2, 1 /* slot */); 

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  udp.begin(2390); // Listen on local port

  // Send NTP Packet
  byte packet[48] = {0};
  packet[0] = 0b11100011; 
  
  udp.beginPacket("pool.ntp.org", 123);
  udp.write(packet, 48);
  udp.endPacket();
}

void loop() {
  int size = udp.parsePacket();
  if (size > 0) {
    Serial.print("Packet received, size: ");
    Serial.println(size);
    udp.read(packet, 48); // Read response
    // Process NTP data...
  }
}
```

### 3. WebSocket Client Example

Note: The corresponding Host setup for this example is located in [examples/Features/Websocket/Host](examples/Features/Websocket/Host).

```cpp
#define ENABLE_SERIALTCP_DEBUG
#include <SerialWebsocketClient.h>

SerialWebsocketClient ws(Serial2, 2 /* slot */);

void onWsEvent(WSMessageType type, const uint8_t* payload, size_t len) {
    switch(type) {
        case WS_EVENT_CONNECTED:
            Serial.println("Connected!");
            ws.sendText("Hello World!");
            break;
        case WS_FRAME_TEXT:
            Serial.printf("Received: %.*s\n", len, payload);
            break;
    }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  ws.onEvent(onWsEvent);
  // Connect SSL (WSS)
  ws.connect("socketsbay.com", 443, "/wss/v2/1/demo/", true);
}

void loop() {
  ws.loop(); // Essential for event processing
}
```

---

## 🧩 Host Setup (ESP32 Example)

The **Host** device acts as the bridge. It requires the `SerialNetworkHost` library and definitions for the native clients (TCP, UDP, WebSocket).

```cpp
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h> // Native WS Library

#define ENABLE_SERIALTCP_DEBUG 
#include <SerialNetworkHost.h>

const char* ssid = "WIFI_SSID";
const char* pass = "WIFI_PASS";

// Native Clients
WiFiClientSecure ssl_client;
WiFiUDP udp_client;
WebSocketsClient ws_client;

// Bridge Manager
SerialNetworkHost host(Serial2);

// Callback to translate Serial commands to Native WS calls
bool onWsCommand(int slot, uint8_t cmd, const uint8_t *payload, size_t len) {
    if (slot != 2) return false; // We assigned Slot 2 to WS
    // ... Implementation details in examples/Features/Websocket/Host ...
    return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  ssl_client.setInsecure();

  // Register Clients to Slots
  host.setTCPClient(&ssl_client, 0); // Slot 0: TCP
  host.setUDPClient(&udp_client, 1); // Slot 1: UDP
  host.setWebSocketClient(&ws_client, 2); // Slot 2: WS
  
  host.setWebSocketCallback(onWsCommand); // Register WS Handler

  host.notifyBoot();
}

void loop() {
  host.loop(); // Handle Serial Traffic
  ws_client.loop(); // Handle WS Traffic
}
```

---

## 📚 API Highlights

### SerialTCPClient
- **`connect(host, port)`**: Connect to a TCP server.
- **`write(buffer, size)`**: Send data.
- **`available()`**: Check for incoming data.
- **`read()`**: Read byte from buffer.
- **`stop()`**: Close connection.
- **`startTLS()`**: Upgrade connection to SSL.

### SerialUDPClient
- **`begin(localPort)`**: Start listening on a port.
- **`beginPacket(host, port)`**: Start a UDP packet.
- **`write(buffer, size)`**: Add data to packet.
- **`endPacket()`**: Send the packet.
- **`parsePacket()`**: Check for incoming UDP datagrams.

### SerialWebsocketClient
- **`connect(host, port, path, ssl)`**: Open WebSocket connection.
- **`onEvent(callback)`**: Register event handler.
- **`sendText(message)`**: Send text frame.
- **`sendBinary(buffer, len)`**: Send binary frame.
- **`pingHost()`**: Check bridge status.

### SerialNetworkHost
- **`setTCPClient(Client*, slot)`**: Bind TCP client to slot.
- **`setUDPClient(UDP*, slot)`**: Bind UDP client to slot.
- **`setWebSocketClient(void*, slot)`**: Bind WebSocket client to slot.
- **`notifyBoot()`**: Reset connected clients on startup.

---

## ⚖️ License

This library is released under the **MIT License**.  
See [LICENSE](LICENSE) for details.

---

## 🙌 Contributing

Pull requests are welcome! Please open an issue first to discuss proposed changes or enhancements.

---

## 📧 Author

Developed and maintained by **mobizt**.