# Python SSE Server for HTTP Streaming Example

This folder contains a simple Python Flask server that acts as the target for the **HTTP Streaming (SSE)** example. It simulates a live data stream by sending JSON events every 2 seconds.

## 📋 Prerequisites

* **Python 3.7+** installed on your system.
* **Pip** (Python package manager).

## 📦 Installation

You need to install the `Flask` library. Automatic scripts are provided for convenience.

### Windows
Double-click **`install_libs.bat`**.
* This script runs as Administrator to upgrade pip and install `flask`.

### Linux / macOS
Open a terminal in this directory and run:
```bash
chmod +x install_libs.sh
./install_libs.sh
```
* This script uses `sudo` to install dependencies.

## 🚀 How to Run

### Windows
Double-click **`run.bat`**.
* This launches `sse_server.py` in a new window.

### Linux / macOS
Open a terminal in this directory and run:
```bash
chmod +x run.sh
./run.sh
```
* This executes `python3 sse_server.py`.

## 🌐 Server Details
* **URL:** `http://localhost:5000/stream`
* **Method:** `GET`
* **Format:** Server-Sent Events (SSE)
* **Output:** JSON payloads (`{"news": [...]}`) sent every 2 seconds.

---
**Note:** Keep this server running **AND** the `serial_bridge.py` (from the parent directory) running simultaneously to allow the Arduino to connect.