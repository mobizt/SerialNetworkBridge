# SerialNetworkBridge Examples

This folder contains example sketches demonstrating how to use the `SerialNetworkBridge` library. The examples are categorized by their connection architecture: **Device Host** and **PC Host**.


## 🚀 Getting Started with PC Host Mode (USB)

PC Host mode allows your Arduino to access the internet directly through your computer's USB connection using a Python script. This eliminates the need for a separate Arduino device bridge.

### 1. Prerequisites
* **Python 3.7+** installed on your computer.
* **Pip** installed.

### 2. Installation
Navigate to `examples/PC_Host/` and run the installer script to download required Python libraries (`pyserial`, `cobs`, `websocket-client`, `colorama`, `pywifi`, `flask`, and `comtypes`).

* **Windows:** Double-click `install_libs.bat`.
* **Linux/macOS:** Run `./install_libs.sh` in a terminal.

### 3. Configure the Bridge
Open `examples/PC_Host/serial_bridge.py` in a text editor and update the configuration at the top:

```python
SERIAL_PORT = 'COM3'   # <-- Update to your Arduino's Port (e.g., COM3 or /dev/ttyUSB0)
BAUD_RATE = 115200     # Must match your Arduino sketch
```

### 4. Run an Example
1.  **Upload** one of the PC Host sketches (e.g., `examples/Device_Host/Generic_Client/HTTP_GET/HTTP_GET.ino`) to your Arduino.
    * *Note:* Ensure `#define ENABLE_LOCAL_DEBUG` is commented out and `HOST_RELAY_DEBUG` is defined in the sketch!
2.  **Close** the Arduino Serial Monitor.
3.  **Run the Bridge:**
    * **Windows:** Double-click `run.bat`.
    * **Linux/macOS:** Run `./run.sh`.

---

## 📚 Example Descriptions

### 1. Device Host (using other Arduino WiFi/Ethernet capable devices as a bridge host device)
Standard setup using an Arduino Client (e.g., Mega) connected via Serial to an ESP32 Host.
* **HTTP_GET:** Simple data retrieval.
* **HTTP_POST:** Sending JSON data to a server.
* **WebSocket:** Connects to a public echo server (`echo.websocket.org`) with SSL verification disabled for compatibility.
* **MQTT:** Connecting to a broker and publishing messages.
* **HTTP_Streaming:** Reading chunked data (SSE) from a local sse server running on the same PC (`localhost:5000`) using python script (`Server/sse_server.py`).
* **UDP_NTP:** Fetches time from `pool.ntp.org` using UDP packets.
* **Host_Management:** **[Advanced]** Allows the Arduino to reboot the PC script or change the PC's Wi-Fi network.
* **Secure_Connection:** Demonstrates loading a custom Root CA (`cert/Amazon_Root_CA1.pem`) to verify a secure connection.
* **STARTTLS:** Upgrades a plain text email connection to SSL.

### 2. PC Host (Direct USB)
Equivalent examples adapted for the Python Bridge.
* **HTTP_GET:** Simple data retrieval.
* **HTTP_POST:** Sending JSON data to a server.
* **WebSocket:** Connects to a public echo server (`echo.websocket.org`) with SSL verification disabled for compatibility.
* **MQTT:** Connecting to a broker and publishing messages.
* **HTTP_Streaming:** Reading chunked data (SSE) from a local sse server running on the same PC (`localhost:5000`) using python script (`Server/sse_server.py`).
* **UDP_NTP:** Fetches time from `pool.ntp.org` using UDP packets.
* **Host_Management:** **[Advanced]** Allows the Arduino to reboot the PC script or change the PC's Wi-Fi network.
* **Secure_Connection:** Demonstrates loading a custom Root CA (`cert/Amazon_Root_CA1.pem`) to verify a secure connection.
* **STARTTLS:** Upgrades a plain text email connection to SSL.

---

## ⚠️ Critical Notes for PC Host
1.  **Using Relay Debug:** You cannot use `Serial.print()` for debugging in your Arduino sketch because the USB port is used for the network protocol. Use the predefined helper functions (`debug::prnt`, `debug::printRaw`, `debug::printNewLinr`) with `HOST_RELAY_DEBUG` to relay the debug message to PC terminal.
2.  **Bootloader Noise:** All PC Host sketches include `Serial.write(0x00); delay(500);` in `setup()` to clear garbage data caused by the Arduino rebooting. Do not remove this line.
3.  **Admin Privileges:** The `Host_Management` example (controlling Wi-Fi/Reboot) requires running the Python script as Administrator (Windows) or with `sudo` (Linux/macOS).