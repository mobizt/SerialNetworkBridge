/*
 * ================================================================
 * NeoHWSerial Client Example (for client to work with Device Host)
 * ================================================================
 * Runs on: Any Arduino AVR device e.g. mega.
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Purpose: Demonstrates how to use NeoHWSerial and InterruptStream wrapper.
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

#include <Arduino.h>
#include <NeoHWSerial.h>
#include <SerialNetworkBridge.h>
#include "debug.h"
#include "InterruptStream.h" // The wrapper class we created earlier

#define BridgeSlot_0 0
#define BridgeRate 115200 // Change this to match with the host serial baud rate.
// Mega Pin 18 (TX1) -> Host device RX (may use Voltage Divider!)
// Mega Pin 19 (RX1) <- Host device TX (may use Voltage Divider!)
#define BridgeSerial NeoSerial1

// Initialize the Wrapper
InterruptStream bridgeStream(BridgeSerial);

// Initialize the Client using the Wrapper
// We pass 'bridgeStream' instead of NeoSerial1
SerialTCPClient client(bridgeStream, 0);

#if defined(HOST_RELAY_DEBUG)
#define STREAM client
#define CLIENT client
#else
#define STREAM NeoSerial
#define CLIENT client
#endif

bool hostReady = false;
unsigned long ms = 0;

// Interrupt service routime for receiving data from Host
static void handleRxChar(uint8_t c)
{
    // Push data from the MKR immediately into our buffer
    bridgeStream.push(c);
}

void setup()
{
    // Debug Serial (USB to PC)
    NeoSerial.begin(115200);
    while (!NeoSerial)
        ;

    BridgeSerial.begin(BridgeRate);

    // Attach the interrupt for high-performance receiving
    BridgeSerial.attachInterrupt(handleRxChar);

    client.setLocalDebugLevel(1); // Enable debug prints

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = client.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Check serial port, baud rate and host device...");
            debug::blink(10, 500);
            delay(2000);
        }
    }
}

void loop()
{
    if (hostReady && (ms == 0 || millis() - ms > 10000))
    {
        ms = millis();

        client.setBufferSizes(2048, 1024);
        client.setInsecure();

        debug::print(STREAM, "Connecting to server...\r\n");

        if (client.connect("httpbin.org", 443))
        {
            debug::print(STREAM, "Server connected, sending request...\r\n");

            client.println("GET /get HTTP/1.1");
            client.println("Host: httpbin.org");
            client.println("Connection: close");
            client.println();

            debug::print(STREAM, "Waiting for response...\r\n");

            char buffer[128];
            int ret = 0, totalRead = 0;
            char endToken = '\n'; // start by reading lines
            do
            {
                ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken);

                if (endToken == '\n' && strcmp(buffer, "\r\n") == 0)
                {
                    // Headers done
                    // read all remaining data without line endings
                    endToken = '\0';
                }
            } while (ret > 0);

            debug::printNewLine(STREAM);
            debug::print(STREAM, ret >= 0 ? "Response complete.\r\n" : (ret < -1 ? "Error: Response timeout.\r\n" : "Error: Socket closed.\r\n"));
            client.stop();
        }
        else
        {
            debug::print(STREAM, "Unable to connect to server\r\n");
        }
    }
}