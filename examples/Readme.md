# SerialNetworkBridge Examples

This folder contains example sketches demonstrating how to use the `SerialNetworkBridge` library. The examples are categorized by their connection architecture: **Basics (Device Host)** and **Features (Device Host & PC Host)**.

## 📂 Directory Structure

```
examples/
├── Basics/                  # Core examples using an ESP32/ESP8266 as the Host Bridge
│   ├── Client/              # Sketches for the Client Device (e.g., Arduino Mega)
│   │   ├── HTTP_GET/
│   │   ├── HTTP_POST/
│   │   ├── HTTP_Streaming/  # Server-Sent Events (SSE)
│   │   └── MQTT/
│   └── Host/                # Sketch for the Host Device (ESP32/ESP8266)
│
├── Features/                # Advanced features and PC Host examples
│   ├── HostManagement/      # Control Host WiFi/Reboot from Client
│   ├── NeoHWSerial_Client/  # High-performance serial for AVR
│   ├── SecureConnection/    # SSL/TLS with custom Certificates
│   ├── STARTTLS/            # Upgrading plain connections to SSL
│   ├── UDP_NTP/             # UDP Datagram examples
│   ├── WebSocket/           # WebSocket Client examples
│   │
│   └── PC_USB_Host/         # [NEW] Connect directly via PC USB (No ESP32/ESP8266 needed!)
│       ├── serial_bridge.py # The Python Bridge Script
│       ├── HTTP_GET/
│       ├── HTTP_POST/
│       ├── WebSocket/
│       ├── MQTT/
│       └── ...
```

---

## 🚀 Getting Started with PC Host Mode (USB)

PC Host mode allows your Arduino to access the internet directly through your computer's USB connection using a Python script. This eliminates the need for a separate ESP32 bridge.

### 1. Prerequisites
* **Python 3.7+** installed on your computer.
* **Pip** installed.

### 2. Installation
Navigate to `examples/Features/PC_USB_Host/` and run the installer script to download required Python libraries (`pyserial`, `cobs`, `websocket-client`, `flask`).

* **Windows:** Double-click `install_libs.bat`.
* **Linux/macOS:** Run `./install_libs.sh` in a terminal.

### 3. Configure the Bridge
Open `examples/Features/PC_USB_Host/serial_bridge.py` in a text editor and update the configuration at the top:

```python
SERIAL_PORT = 'COM3'   # <-- Update to your Arduino's Port (e.g., COM3 or /dev/ttyUSB0)
BAUD_RATE = 115200     # Must match your Arduino sketch
```

### 4. Run an Example
1.  **Upload** one of the PC Host sketches (e.g., `HTTP_GET.ino`) to your Arduino.
    * *Note:* Ensure `#define ENABLE_SERIALTCP_DEBUG` is commented out in the sketch!
2.  **Close** the Arduino Serial Monitor.
3.  **Run the Bridge:**
    * **Windows:** Double-click `run.bat`.
    * **Linux/macOS:** Run `./run.sh`.
4.  **Watch the LED:** The built-in LED on your Arduino will flash to indicate status (Fast Flash = Success, Slow Blink = Error).

---

## 📚 Example Descriptions

### 1. Basics (Device Host)
Standard setup using an Arduino Client (e.g., Mega) connected via Serial to an ESP32 Host.
* **HTTP_GET:** Simple data retrieval.
* **HTTP_POST:** Sending JSON data to a server.
* **MQTT:** Connecting to a broker and publishing messages.
* **HTTP_Streaming:** Reading chunked data (SSE) from a PHP server.

### 2. PC USB Host (Direct USB)
Equivalent examples adapted for the Python Bridge.
* **HTTP_GET:** Basic connectivity test.
* **HTTP_POST:** Demonstrates HTTPS (SSL) by passing `true` as the 3rd argument to `connect()`.
* **WebSocket:** Connects to a public echo server (`echo.websocket.org`) with SSL verification disabled for compatibility.
* **MQTT:** Connects to `broker.hivemq.com` on port 8883 (SSL).
* **UDP_NTP:** Fetches time from `pool.ntp.org` using UDP packets.
* **HostManagement:** **[Advanced]** Allows the Arduino to reboot the PC script or change the PC's Wi-Fi network.
* **SecureConnection:** Demonstrates loading a custom Root CA (`cert/Amazon_Root_CA1.pem`) to verify a secure connection.
* **STARTTLS:** Upgrades a plain text email connection to SSL.

### 3. HTTP Streaming (SSE)
Located in `examples/Features/PC_USB_Host/HTTP_Streaming`.
* **Server:** Includes a local Python server (`Server/sse_server.py`) that generates dummy events.
* **Client:** The Arduino sketch connects to `localhost:5000` via the bridge to read the stream.

---

## ⚠️ Critical Notes for PC Host
1.  **No Debug Prints:** You cannot use `Serial.print()` for debugging in your Arduino sketch because the USB port is used for the network protocol. Use the Built-in LED for status indication.
2.  **Bootloader Noise:** All PC Host sketches include `Serial.write(0x00); delay(500);` in `setup()` to clear garbage data caused by the Arduino rebooting. Do not remove this line.
3.  **Admin Privileges:** The `HostManagement` example (controlling Wi-Fi/Reboot) requires running the Python script as Administrator (Windows) or with `sudo` (Linux/macOS).