/*
 * =============================================================
 * AsyncTCP Client Example (for client to work with Device Host)
 * =============================================================
 * Runs on: Any Arduino device (Client).
 * Host: Requires "Device_Host/Async_Client/Host/Host.ino" running on ESP32
 * Purpose: Demonstrates a AsyncTCP client usage.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should be matched with host device serial baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSerial       The serial port that is connected to host device serial port.
 *
 * BridgeSlot_0       The slot (channel or session) 0 which data will be transferred
 *                    This corresponds to the SSL client assigned on slot 0 of the host device.
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

#define BridgeSlot_0 0
#define BridgeRate 115200    // Change this to match with the host serial baud rate.
#define BridgeSerial Serial2 // Change this to match your hardware.

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

SerialAsyncTCPClient asyncClient(BridgeSerial, BridgeSlot_0);

#if defined(HOST_RELAY_DEBUG)
#define STREAM client
#define CLIENT client
#else
#define STREAM Serial
#define CLIENT client
#endif

// State tracking
bool hostReady = false;
bool requestSent = false;

void onConnect(void *arg, SerialAsyncTCPClient *c)
{
    debug::print(STREAM, "Event: Connected to Server!\r\n");

    debug::print(STREAM, "Sending GET request...\r\n");

    // [FIX] Combine payload into one string to prevent packet dropping
    String req = "GET /get HTTP/1.1\r\n";
    req += "Host: httpbin.org\r\n";
    req += "User-Agent: SerialNetworkBridge/2.0\r\n";
    req += "Connection: close\r\n\r\n";

    c->write(req.c_str());

    requestSent = true;
}

void onData(void *arg, SerialAsyncTCPClient *c, void *data, size_t len)
{
    debug::print(STREAM, "Event: Data Received (");
    debug::printRaw(STREAM, String(len).c_str());
    debug::printRaw(STREAM, " bytes)\r\n");

    char *str = (char *)malloc(len + 1);
    if (str)
    {
        memcpy(str, data, len);
        str[len] = '\0';
        debug::printRaw(STREAM, str);
        debug::printNewLine(STREAM);
        free(str);
    }
}

void onDisconnect(void *arg, SerialAsyncTCPClient *c)
{
    debug::print(STREAM, "Event: Disconnected.\r\n");
    requestSent = false;
}

void onError(void *arg, SerialAsyncTCPClient *c, int8_t error)
{
    debug::print(STREAM, "Event: Error Code: ");
    debug::printRaw(STREAM, String(error).c_str());
    debug::printNewLine(STREAM);
}

void onTimeout(void *arg, SerialAsyncTCPClient *c, uint32_t time)
{
    debug::print(STREAM, "Event: Ack Timeout!\r\n");
    c->close();
}

void setup()
{

    Serial.begin(115200);

    delay(2000);

    BridgeSerial.begin(BridgeRate);

    asyncClient.setLocalDebugLevel(1);

    debug::print(STREAM, "Pinging Host...\r\n");

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = asyncClient.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Check serial port, baud rate and host device3...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

    debug::print(STREAM, "Host is ready\r\n");

    asyncClient.onConnect(onConnect);
    asyncClient.onData(onData);
    asyncClient.onDisconnect(onDisconnect);
    asyncClient.onError(onError);
    asyncClient.onTimeout(onTimeout);

    debug::print(STREAM, "Connecting to httpbin.org...\r\n");

    if (!asyncClient.connect("httpbin.org", 80))
    {
        debug::print(STREAM, "Failed to send connect command.\r\n");
    }
}

void loop()
{

    asyncClient.loop();

    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 10000)
    {
        lastCheck = millis();

        // Only warn if we expected to be connected
        if (requestSent && !asyncClient.connected())
        {
            debug::print(STREAM, "Server is not connected.\r\n");
        }
    }
}