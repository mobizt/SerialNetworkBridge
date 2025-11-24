#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Re-running with sudo..."
  exec sudo "$0" "$@"
fi

cd "$(dirname "$0")"
python3 -m pip install --upgrade pip
python3 -m pip install pyserial cobs websocket-client
