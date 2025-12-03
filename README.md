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
  <img src="https://img.shields.io/badge/Version-1.0.4-orange.svg" alt="Version 1.0.4">
</p>

<p align="center">
  <b>The Arduino Serial bridge for TCP, UDP, WebSocket, and AsyncTCP Clients</b><br>
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
Use your computer (Windows, Linux, macOS) or Raspberry Pi as the gateway via the USB cable.
<p align="center">
  <img src="https://raw.githubusercontent.com/mobizt/SerialNetworkBridge/refs/heads/main/assets/diagram_pc.svg" alt="PC USB Bridge Architecture" width="800"/>
</p>

---

## ✨ Features

- **Multi-Protocol Support:** Bridge **TCP**, **UDP**, **WebSocket**, and **AsyncTCP** clients via serial.
- **Universal Compatibility:** Works with any interface implementing the `Stream` class.
- **PC Host Mode:** Connect directly to a PC/Raspberry Pi via USB for internet access.
- **Secure:** Support for **SSL/TLS** (HTTPS/WSS) and **STARTTLS**.
- **Performance:** Supports `NeoHWSerial` for AVR.
- **AsyncTCP Support (New):** Non-blocking, event-driven TCP for high-throughput applications (Requires Dual-Core ESP32 Host).

---

## 📦 Installation

### Arduino IDE  
1. Open Arduino IDE.  
2. Go to **Sketch → Include Library → Manage Libraries…** 
3. Search for **SerialNetworkBridge**.  
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
- Any board with a `Stream` interface.

**Hosts (Network Bridges):**
- **Microcontrollers:** ESP32, ESP8266, Raspberry Pi Pico W, MKR WiFi 1010.
- **PC / Linux (USB):** - **Windows** (requires Admin for WiFi control).
    - **Linux / Raspberry Pi** (requires Sudo/NetworkManager).
    - **macOS** (requires Sudo).

---

## 🚀 Usage: The Client (Your Arduino)

These sketches run on your non-networked board. They connect to the "Host" (Arduino device or PC) to access the internet.

### ⚠️ CRITICAL: PC Host Setup Rules
If you are using the **PC Host Bridge**, you **must** follow these rules in your client sketch:
1. **Using Relay Debug instead of System Debugging:** Comment out `#define ENABLE_LOCAL_DEBUG`. Use the predefined helper functions (`debug::prnt`, `debug::printRaw`, `debug::printNewLinr`) with `HOST_RELAY_DEBUG` to relay the debug message to PC terminal.
2. **Use Main Serial:** Pass `Serial` (USB) to the client constructor.
3. **Flush Bootloader:** Add `Serial.write(0x00); delay(500);` in `setup()`.
4. **Explicit SSL:** Pass `true` as the 3rd argument to `connect()` for HTTPS (e.g., `client.connect("site.com", 443, true)`).

### 1. TCP Client Example (HTTP GET)

```cpp
// [PC Host] Comment out debug to prevent protocol corruption
// #define ENABLE_LOCAL_DEBUG 
#include <SerialNetworkBridge.h>

// Use Serial for PC Host, Serial2 for ESP32 Host
SerialTCPClient client(Serial, 0); 

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // [PC Host] Flush bootloader garbage
  Serial.write(0x00); 
  delay(500);

  if (client.pingHost(500)) {
    // 3rd param 'true' enables SSL/TLS (Required for PC Host HTTPS)
    if (client.connect("httpbin.org", 443, true)) {
      client.println("GET /get HTTP/1.1");
      client.println("Host: httpbin.org");
      client.println("Connection: close");
      client.println();
      // Read response...
    }
  }
}

void loop() {}
```

### 2. WebSocket Client Example

```cpp
#include <SerialNetworkBridge.h>

SerialWebsocketClient ws(Serial, 0);

void onWsEvent(WSMessageType type, const uint8_t* payload, size_t len) {
    if(type == WS_EVENT_CONNECTED) {
        // Connection Success
    }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.write(0x00); delay(500); // Flush

  ws.onEvent(onWsEvent);
  ws.connect("echo.websocket.org", 443, "/", true);
}

void loop() {
  ws.loop(); // Essential for events
}
```

### 3. AsyncTCP Client Example
Use this for non-blocking HTTP requests or high-performance streaming.

```cpp
#define USE_ASYNC_CLIENT // Required for Async support
#define ENABLE_LOCAL_DEBUG 
#include <SerialNetworkBridge.h>

SerialAsyncTCPClient asyncClient(Serial1, 0); // Use Bridge Serial

void onData(void* arg, SerialAsyncTCPClient* c, void* data, size_t len) {
  Serial.print("Received: ");
  Serial.write((uint8_t*)data, len);
}

void setup() {
  Serial.begin(115200);

  Serial1.begin(115200); // Bridge
  
  asyncClient.onData(onData);
  asyncClient.onConnect([](void* arg, SerialAsyncTCPClient* c) {
    c->write("GET / HTTP/1.1\r\nHost: example.com\r\n\r\n");
  });
  
  asyncClient.connect("example.com", 80);
}

void loop() {
  asyncClient.loop(); // Critical for event processing
}
```

---

## 🧩 Host Setup (Choose One)

### Option A: Microcontroller Host (ESP32/ESP8266)
Upload the **Host** sketch to an ESP32/ESP8266. It bridges Serial traffic to WiFi.
* **Example:** `examples/Device_Host/Generic_Client/Host/Host.ino`

### Option B: PC / Raspberry Pi Host (Python)
Use your computer as the bridge via USB! Supports real system network management.

1.  **Install Dependencies:**
    Navigate to `examples/PC_Host` and run:
    * **Windows:** `install_libs.bat`
    * **Linux/Mac:** `./install_libs.sh`
2.  **Configure:**
    Edit `serial_bridge.py`: Set `SERIAL_PORT` (e.g., `COM3`, `/dev/ttyUSB0`) and `BAUD_RATE`.
3.  **Run:**
    * **Windows:** `run.bat` (Run as Administrator for WiFi/Reboot features).
    * **Linux/Mac:** `./run.sh` (Run with `sudo` for WiFi/Reboot features).
    

### ⚠️ Important Note for AsyncTCP (ESP32)
If you use the `SerialAsyncTCPClient` feature, a **Dual-Core ESP32** (e.g., ESP32-WROOM, ESP32-S3) is strongly recommended. 
* **Single-Core Devices (ESP32-C3, ESP32-S2):** May experience data packet loss or connection stalls (receiving only ~1400 bytes) due to CPU contention between the Network Stack and Serial processing.
* **Recommendation:** Use the standard synchronous `SerialTCPClient` on single-core devices.

---

## 📚 API Highlights

### SerialTCPClient
- **`connect(host, port, [ssl])`**: Connect to a TCP server. **Pass `true` for SSL on PC Host.**
- **`setCACert(filename)`**: Load a custom Root CA file (relative path on PC).
- **`startTLS()`**: Upgrade connection to SSL (e.g., for SMTP).

### SerialHostManager (Global Commands)
- **`pingHost()`**: Check if bridge is running.
- **`setWiFi(ssid, pass)`**: Set Wi-Fi credentials on Host (PC or ESP32).
- **`connectNetwork()`**: Trigger Wi-Fi connection on Host (OS-level on PC).
- **`disconnectNetwork()`**: Disconnect Host Wi-Fi.
- **`rebootHost()`**: Reboot the Host (Restarts script on PC, reboots ESP32).

---

## ⚖️ License

This library is released under the **MIT License**.  
See [LICENSE](LICENSE) for details.

---

## 📧 Author

Developed and maintained by **mobizt**.