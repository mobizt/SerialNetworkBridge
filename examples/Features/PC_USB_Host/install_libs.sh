#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Requesting sudo privileges to install dependencies..."
  sudo "$0" "$@"
  exit
fi

cd "$(dirname "$0")"

echo "Installing Python Dependencies..."
# Ensure pip is installed (common issue on some Linux distros)
if ! command -v pip3 &> /dev/null; then
    echo "pip3 not found. Attempting to install python3-pip..."
    if [ -x "$(command -v apt-get)" ]; then
        apt-get update && apt-get install -y python3-pip
    fi
fi

python3 -m pip install --upgrade pip
# 'flask' is optional but included for the HTTP Streaming server example
python3 -m pip install pyserial cobs websocket-client flask

echo ""
echo "Dependencies installed."

# OS-Specific Checks
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Checking for network managers..."
    if command -v nmcli &> /dev/null; then
        echo "  [OK] NetworkManager (nmcli) found."
    elif command -v wpa_cli &> /dev/null; then
        echo "  [OK] wpa_supplicant (wpa_cli) found."
    else
        echo "  [WARNING] Neither nmcli nor wpa_cli found. WiFi management may not work."
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Checking for network setup tools..."
    if command -v networksetup &> /dev/null; then
        echo "  [OK] networksetup found."
    else
        echo "  [WARNING] networksetup command not found. WiFi management may not work."
    fi
fi

echo ""
echo "Setup complete. You can now run './run.sh'."