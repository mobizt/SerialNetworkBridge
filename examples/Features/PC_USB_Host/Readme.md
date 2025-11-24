# Serial Network Bridge (Python Host)

This Python script acts as a network bridge for your Arduino device. It listens to the serial port and proxies TCP, UDP, and WebSocket connections from the Arduino to the PC's internet connection.

## 📋 Prerequisites

* **Python 3.7 or newer** installed on your system.
* An Arduino board running the `SerialTCPClient` (or `SerialNetworkBridge`) library.

---

## 📦 Installation

You can install the required dependencies automatically using the provided scripts, or manually via the terminal.

### Option A: Automatic Installation (Recommended)

**🪟 Windows:**
Double-click **`install_libs.bat`**.
* [cite_start]This script will automatically request **Administrator privileges**[cite: 1].
* [cite_start]It upgrades `pip` and installs `pyserial`, `cobs`, and `websocket-client`[cite: 1].

**🐧 Linux / Raspberry Pi:**
Open a terminal in this folder and run:
```bash
chmod +x install_libs.sh
./install_libs.sh
```
* This script will prompt for your `sudo` password to install dependencies globally.

### Option B: Manual Installation

It is recommended to use a virtual environment to keep dependencies isolated, though you can install them globally if preferred.

**1. Create a Virtual Environment (Optional)**
```powershell
# Windows
python -m venv venv
.\venv\Scripts\activate

# Linux / Mac
python3 -m venv venv
source venv/bin/activate
```

**2. Install Dependencies**
```bash
pip install pyserial cobs websocket-client
```

---

## ⚙️ Configuration

### 1. Python Script Setup

Open `serial_bridge.py` in a text editor and update the constants at the top:

```python
SERIAL_PORT = 'COM3'  # Windows Example
# SERIAL_PORT = '/dev/ttyUSB0'  # Linux/RPi Example

BAUD_RATE = 115200    # Must match your Arduino sketch!
```

### 2. Arduino Sketch Setup (CRITICAL)

When connecting directly to a PC via USB, you **must** follow these two rules in your Arduino sketch:

1.  **Use `Serial`**: Pass the main `Serial` object to the client constructor.
2.  **Disable Debug**: You **MUST** remove or comment out `#define ENABLE_SERIALTCP_DEBUG`. If debug is enabled, text logs will mix with the binary data, causing CRC errors and connection failure.

**Example Sketch:**

```cpp
// 1. DISABLE DEBUG (Comment this line out!)
// #define ENABLE_SERIALTCP_DEBUG 

#include <SerialNetworkBridge.h>

// 2. Use 'Serial' (The USB connection)
SerialTCPClient client(Serial, 0); 

void setup() {
  // 3. Start Serial at the SAME baud rate as your Python script
  Serial.begin(115200);

  // Wait for Serial to be ready (important for native USB boards like Leonardo/Micro)
  while (!Serial);

  // Note: Do NOT use Serial.println() for logs! 
  // It will break the bridge connection.
}

void loop() {
  // Your network logic here...
}
```

---

## 🚀 How to Run

### Option A: Quick Start (Scripts)

**🪟 Windows:**
Double-click **`run.bat`**.
* [cite_start]This will launch the Python bridge with Administrator privileges (required for some serial port access)[cite: 2].

**🐧 Linux / Raspberry Pi:**
Open a terminal in this folder and run:
```bash
chmod +x run.sh
./run.sh
```
* This launches the script using `python3`.

### Option B: Manual Run

**🪟 Windows**
1.  Connect your Arduino via USB.
2.  Find your COM port in **Device Manager**.
3.  Update `SERIAL_PORT` in `serial_bridge.py`.
4.  Run the script:
    ```powershell
    python serial_bridge.py
    ```

**🐧 Linux / Raspberry Pi**
1.  Connect your Arduino.
2.  Find the port name: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`.
3.  Update `SERIAL_PORT` in `serial_bridge.py`.
4.  **Permission Setup (One-time only):**
    If you get a "Permission denied" error, add your user to the dialout group:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    *Log out and back in for this to take effect.*
5.  Run the script:
    ```bash
    python3 serial_bridge.py
    ```

---

## ❓ Troubleshooting

**`CRC Mismatch` / `COBS Decode Error`**
* **Cause 1:** You forgot to comment out `#define ENABLE_SERIALTCP_DEBUG` in the Arduino sketch. The PC is receiving text logs instead of data packets.
* **Cause 2:** The Baud Rate in Python does not match `Serial.begin(...)` in the sketch.
* **Fix:** Disable debug and match the baud rates.

**`Permission denied: '/dev/ttyUSB0'`**
* This happens on Linux/Pi if your user isn't in the `dialout` group.
* **Fix:** Use the `install_libs.sh` script or run `sudo chmod 666 /dev/ttyUSB0` (temporary fix).

**`SerialException: could not open port`**
* The port name is wrong, or the port is already open in another program (like the Arduino Serial Monitor).
* **Fix:** Close the Arduino Serial Monitor and double-check the port name.