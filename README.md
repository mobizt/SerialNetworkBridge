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
  Enable advanced network functionality on non-networked boards by bridging them to a WiFi‑capable device (ESP32/ESP8266) or a PC via USB.
</p>

<p align="center">It is designed for Arduino boards such as AVR, STM32, and Teensy that lack built‑in WiFi or Ethernet. By bridging communication through modules like ESP32, Raspberry Pi Pico W, or even a PC running a Python script, the library makes network access broadly available. With support for <b>SSL/TLS</b>, <b>WebSockets</b>, and <b>UDP</b>, SerialNetworkBridge enables secure communication without requiring firmware‑level certificate management.</p>

---

## 🏗 Architecture Options

You can deploy this library in two ways depending on your hardware:

### Option A: Microcontroller Bridge (e.g., ESP32)
Use a WiFi-capable microcontroller as a dedicated network co-processor.
<p align="center">
  <img src="https://raw.githubusercontent.com/mobizt/SerialNetworkBridge/refs/heads/main/assets/diagram.svg" alt="Microcontroller Bridge Architecture" width="800"/>
</p>

### Option B: PC / USB Bridge (Python)
Use your computer or Raspberry Pi as the gateway via the USB cable.
<p align="center">
  <img src="https://raw.githubusercontent.com/mobizt/SerialNetworkBridge/refs/heads/main/assets/diagram_pc.svg" alt="PC USB Bridge Architecture" width="800"/>
</p>

---

## ✨ Features

- **Multi-Protocol Support:** Bridge **TCP**, **UDP**, and **WebSocket** clients via serial.
- **Universal Compatibility:** Works with any interface implementing the `Stream` class (`HardwareSerial`, `SoftwareSerial`, `USBSerial`, etc.).
- **PC Host Mode:** Connect your Arduino directly to a **PC or Raspberry Pi** via USB to access the internet using the provided Python script.
- **Secure:** Support for **SSL/TLS** (HTTPS/WSS) and **STARTTLS** upgrades handled by the host.
- **Performance:** Supports `NeoHWSerial` for high-performance interrupt-driven communication on AVR boards (see `examples/Features/NeoHWSerial_Client`).
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
- Any board with a `Stream` interface (Hardware or Software Serial)

**Hosts (Network Bridges):**
- **Microcontrollers:** ESP32, ESP8266, Raspberry Pi Pico W, MKR WiFi 1010, Arduino UNO WiFi Rev2.
- **PC / Linux:** Windows, Linux, macOS, or Raspberry Pi (via USB & Python).

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

## 🚀 Usage: The Client (Your Arduino)

These sketches run on your non-networked board (e.g., Arduino Mega). They communicate with a "Host" (see the next section) to access the internet.

### 1. TCP Client Example (HTTP GET)

```cpp
// NOTE: Comment out this line if connecting to PC Host via USB!
#define ENABLE_SERIALTCP_DEBUG 
#include <SerialNetworkBridge.h>

SerialTCPClient client(Serial2, 0 /* slot */); 

void setup() {
  Serial.begin(115200); // Debug
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
#include <SerialNetworkBridge.h>

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
    // Read response...
  }
}
```

### 3. WebSocket Client Example

```cpp
#define ENABLE_SERIALTCP_DEBUG
#include <SerialNetworkBridge.h>

SerialWebsocketClient ws(Serial2, 2 /* slot */);

void onWsEvent(WSMessageType type, const uint8_t* payload, size_t len) {
    switch(type) {
        case WS_EVENT_CONNECTED:
            ws.sendText("Hello World!");
            break;
        case WS_FRAME_TEXT:
            Serial.printf("RX: %.*s\n", len, payload);
            break;
    }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  ws.onEvent(onWsEvent);
  ws.connect("socketsbay.com", 443, "/wss/v2/1/demo/", true);
}

void loop() {
  ws.loop(); // Essential
}
```

---

## 🧩 Host Setup (Choose One)

You need **one** of the following acting as the bridge.

### Option A: Microcontroller Host (ESP32/ESP8266)

Upload the **Host** sketch to an ESP32 or ESP8266. This device connects to WiFi and forwards traffic from the Serial port.

```cpp
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h> // Native WS Library
#define ENABLE_SERIALTCP_DEBUG 
#include <SerialNetworkBridge.h>

// Native Clients
WiFiClientSecure ssl_client;
WiFiUDP udp_client;
WebSocketsClient ws_client;

SerialNetworkHost host(Serial2); // Serial Port connected to Client

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.begin("SSID", "PASS");
  
  ssl_client.setInsecure();

  // Register Clients to Slots
  host.setTCPClient(&ssl_client, 0); 
  host.setUDPClient(&udp_client, 1); 
  host.setWebSocketClient(&ws_client, 2); 
  
  host.notifyBoot();
}

void loop() {
  host.loop();      // Handle Serial Traffic
  ws_client.loop(); // Handle WS Traffic
}
```

### Option B: PC / Raspberry Pi Host (Python)

You can use your computer as the bridge via the USB cable!

1.  **Arduino Setup:** * Use `Serial` (USB) for the client connection.
    * **Disable Debugging:** Comment out `#define ENABLE_SERIALTCP_DEBUG`.
2.  **PC Setup:**
    * Install Python 3.7+.
    * Navigate to `examples/Features/PC_USB_Host`.
    * Run the installer script (`install_libs.bat` or `install_libs.sh`).
    * Run the bridge script (`run.bat` or `run.sh`).

*See [`examples/Features/PC_USB_Host`](examples/Features/PC_USB_Host) for detailed instructions.*

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

## 📧 Author

Developed and maintained by **mobizt**.