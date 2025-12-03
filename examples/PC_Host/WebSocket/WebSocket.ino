/**
 * ===================================================
 * WebSocket Example (for client to work with PC Host)
 * ===================================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to configure your device to connect to
 * the 'serial_bridge.py' script running on your computer via the USB cable.
 * It connects to a WebSocket echo server and blinks the LED on events.
 * * * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor (The USB port is used for the bridge).
 * 4. Run 'python serial_bridge.py' on your computer.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSlot_0       The slot (session) 0 which data will be transferred
 *                    This corresponds to the ssl wrapped socket assigned on slot 0 of the python script running on the PC device.
 *                    The implementation on python script is different and flexible. The transport layer does not fix to the slot
 *                    as it is assigned to the host device. 
 *
 * ENABLE_LOCAL_DEBUG The macro to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *                    It shiukd not defined in this example.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be defined in this example.
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * CLIENT             The client object e.g. SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define HOST_RELAY_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeSlot_0 0
#define BridgeRate 115200

SerialWebsocketClient ws(Serial, BridgeSlot_0);

#if defined(HOST_RELAY_DEBUG)
#define STREAM ws
#define CLIENT ws
#else
#define STREAM Serial
#define CLIENT ws
#endif

unsigned long ms = 0;
bool hostReady = false;

// Use the main USB Serial port (Slot 0)
SerialWebsocketClient ws(Serial, BridgeSlot_0);

const unsigned long interval = 5000; // Send message every 5 seconds

// WebSocket Event Callback
void onWsEvent(WSMessageType type, const uint8_t *payload, size_t len)
{
    switch (type)
    {
    case WS_EVENT_CONNECTED:
        debug::print(STREAM, "WS Connected!\r\n\r\n");
        break;
    case WS_EVENT_DISCONNECTED:
        debug::print(STREAM, "Warning: WS Disconnected!\r\n\r\n");
        break;
    case WS_FRAME_TEXT:
        if (type == WS_FRAME_TEXT)
        {
            char *msg = (char *)malloc(len + 1);
            memcpy(msg, payload, len);
            msg[len] = '\0';
            debug::print(STREAM, "Text message received: ");
            debug::printRaw(STREAM, msg);
            debug::printRaw(STREAM, "\r\n\r\n");
        }
        else if (type == WS_FRAME_BINARY)
        {
            debug::print(STREAM, "Binary message received, Size: ");
            debug::printRaw(STREAM, String(len).c_str());
            debug::printRaw(STREAM, "\r\n\r\n");
        }

        break;
    }
}

void setup()
{
    Serial.begin(BridgeRate);
    while (!Serial)
        ;

    // [CRITICAL FIX] Flush bootloader noise so the PC Python script
    // doesn't get confused by random bytes on startup.
    Serial.write(0x00);
    delay(500);

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = ws.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Please make sure serial_bridge.py is running...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

    ws.stop();

    // Skip SSL certificate verification
    ws.setInsecure();
    // Set SSL Client RX/TX buffer sizest
    ws.setBufferSizes(2048, 1024);

    // Register the event callback
    ws.onEvent(onWsEvent);

    debug::print(STREAM, "Connecting to echo server...\r\n\r\n");

    // Connect to the public echo server (SSL Enabled)
    ws.connect("echo.websocket.org", 443, "/", true);
}

void loop()
{
    // WebSocket is event-driven, so we simply call loop() repeatedly
    ws.loop();

    // Send Periodic Requests
    if (ms == 0 || millis() - ms > interval)
    {
        ms = millis();
        debug::print(STREAM, "Sending message...\r\n\r\n");
        ws.sendText("Hello from Arduino!");
    }
}