/**
 * ======================================================
 * WebSocket Example (for client to work with Device Host)
 * ======================================================
 * Runs on: Arduino device (e.g., Uno, Nano).
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Purpose: Demonstrates the Websocket client usage.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should be matched with host device serial baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSerial       The serial port that is connected to host device serial port.
 *
 * BridgeSlot_2       The slot (channel or session) 2 which data will be transferred
 *                    This corresponds to the Websocket client assigned on slot 2 of the host device.
 *                    See Host.ino example.
 *
 * ENABLE_LOCAL_DEBUG The macro to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be commented out or undefined in this example (debug info is printed locally).
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * CLIENT             The client object e.g. SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define ENABLE_LOCAL_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeSlot_2 2
#define BridgeRate 115200    // Change this to match with the host serial baud rate.
#define BridgeSerial Serial2 // Change this to match your hardware.

SerialWebsocketClient ws(BridgeSerial, BridgeSlot_2);

#if defined(HOST_RELAY_DEBUG)
#define STREAM ws
#define CLIENT ws
#else
#define STREAM Serial
#define CLIENT ws
#endif

unsigned long ms = 0;
bool hostReady = false;

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
    Serial.begin(115200);
    delay(1000);

    BridgeSerial.begin(BridgeRate);

    ws.setLocalDebugLevel(1); // Enable debug prints

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = ws.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Check serial port, baud rate and host device...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

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