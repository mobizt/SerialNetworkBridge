/**
 * ==================================================
 * STARTTLS Example (for client to work with PC Host)
 * ==================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Connects to broker.hivemq.com on Port 8883 (SSL) via PC Bridge.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch.
 * 3. Close Serial Monitor.
 * 4. Run 'python serial_bridge.py'.
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

SerialTCPClient client(Serial, BridgeSlot_0);

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
    Serial.begin(BridgeRate);
    while (!Serial)
        ;

    Serial.write(0x00);
    delay(500);

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = client.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Please make sure serial_bridge.py is running...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

    client.stop();
}

void loop()
{
    if (hostReady && (ms == 0 || millis() - ms > 20000))
    {
        ms = millis();

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