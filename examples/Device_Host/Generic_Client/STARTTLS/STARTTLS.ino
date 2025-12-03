/**
 * =======================================================
 * STARTTLS Example (for client to work with Device Host)
 * =======================================================
 * Runs on: Any Arduino device.
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Purpose: Demonstrates how to upgrade a plain
 * connection to a secure one using startTLS().
 * This is common for protocols like SMTP, IMAP, and FTP.
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
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeSlot_0 0
#define BridgeRate 115200    // Change this to match with the host serial baud rate.
#define BridgeSerial Serial2 // Change this to match your hardware.

SerialTCPClient client(BridgeSerial, BridgeSlot_0);

#if defined(HOST_RELAY_DEBUG)
#define STREAM client
#define CLIENT client
#else
#define STREAM Serial
#define CLIENT client
#endif

unsigned long ms = 0;
bool hostReady = false;

const char *server = "smtp.gmail.com";
const int port = 587; // Standard STARTTLS port

void setup()
{
    Serial.begin(115200);
    delay(1000);

    BridgeSerial.begin(BridgeRate);
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
    if (hostReady && (ms == 0 || millis() - ms > 20000))
    {
        ms = millis();

        // This will set SSL I/O buffer (if SSL client on host device is supported)
        client.setBufferSizes(2048, 1024);

        // Set this to skip SSL certificate verification on SSL client assigned to slot 0 in host device.
        client.setInsecure();

        int ret = 0, totalRead = 0;
        char endToken = '\n'; // Read until newline
        char buffer[128];

        debug::print(STREAM, "Connecting to server...\r\n");
        //  Connect in PLAIN mode
        // Note: 3rd param is 'false' (No SSL yet)
        if (client.connect(server, port, false))
        {
            debug::printNewLine(STREAM);
            debug::print(STREAM, "Reading Greeting...\r\n");

            ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken);

            debug::printNewLine(STREAM);
            debug::print(STREAM, "Sending EHLO ...\r\n");

            client.println("EHLO my-arduino-client.com");

            debug::print(STREAM, "Reading multi-line response...\r\n\r\n");

            bool found_starttls = false;
            while (true)
            {
                ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken);

                if (strstr(buffer, "250-STARTTLS") != NULL)
                {
                    found_starttls = true;
                }

                if (strstr(buffer, "250 ") != NULL)
                { // Last line of EHLO
                    break;
                }
            }

            if (found_starttls)
            {
                debug::printNewLine(STREAM);
                debug::print(STREAM, "Sending STARTTLS and reading response...\r\n\r\n");

                client.println("STARTTLS");

                ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken); // Read "220 2.0.0 Ready to start TLS"

                // Upgrade our *proxy* connection
                // This tells the Python script to wrap the existing socket in SSL
                if (client.startTLS())
                {
                    debug::printNewLine(STREAM);
                    debug::print(STREAM, "TLS Upgrade Successful!\r\n");

                    debug::print(STREAM, "Sending EHLO over secure channel...\r\n");

                    client.println("EHLO my-arduino-client.com");

                    debug::print(STREAM, "Reading secure EHLO response...\r\n\r\n");

                    // Read the new, secure EHLO response
                    while (ret > 0)
                    {
                        ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken);
                        if (strstr(buffer, "250 ") != NULL)
                        {
                            break;
                        }
                    }
                    debug::printNewLine(STREAM);
                    debug::print(STREAM, "Secure EHLO complete.\r\n");
                }
                else
                {
                    // Error: Upgrade Failed
                    debug::printNewLine(STREAM);
                    debug::print(STREAM, "Error: TLS Upgrade Failed!\r\n");
                }
            }

            client.stop();
        }
        else
        {
            // Error: Connection Failed (Slow Blink)
            debug::printNewLine(STREAM);
            debug::print(STREAM, "Error: Connection Failed!\r\n");
        }
    }
}