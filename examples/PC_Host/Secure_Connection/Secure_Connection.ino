/**
 * ===========================================================
 * Secure Connection Example (for client to work with PC Host)
 * ===========================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates a secure HTTPS request using a specific CA Certificate.
 * It verifies httpbin.org against a local certificate file on the PC.
 *
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Certificate Setup:
 * - This example assumes you are running 'serial_bridge.py' from the folder:
 * 'examples/Features/PC_USB_Host/'
 * - The certificate file MUST be located at:
 * 'examples/Features/PC_USB_Host/cert/Amazon_Root_CA1.pem'
 * 3. Upload this sketch to your Arduino.
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
#define BridgeRate 115200    // Change this to match with the host serial baud rate.

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

const char *server = "httpbin.org";
const int port = 443;

// [IMPORTANT] Certificate Path Configuration:
// The path provided below is RELATIVE to the working directory where you run 'serial_bridge.py'.
// If you run the script from 'examples/Features/PC_USB_Host/', then:
// "cert/Amazon_Root_CA1.pem" maps to -> "examples/Features/PC_USB_Host/cert/Amazon_Root_CA1.pem"
const char *ca_cert = "cert/Amazon_Root_CA1.pem";

#if defined(HOST_RELAY_DEBUG)
#define STREAM client
#define CLIENT client
#else
#define STREAM Serial
#define CLIENT client
#endif

void setup()
{
    Serial.begin(115200);
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
    if (hostReady && (ms == 0 || millis() - ms > 10000))
    {
        ms = millis();

        debug::print(STREAM, "Set the CA Certificate...\r\n");
        // Set the CA Certificate
        // This tells the Python script to load the specific file defined in 'ca_cert'
        // for verification on the next connection attempt.
        if (client.setCACert(ca_cert))
        {
            debug::print(STREAM, "Connecting to server...\r\n");

            // Connect to the server (Secure)
            // 3rd param 'true' enables SSL/TLS (HTTPS)
            if (client.connect(server, port, true))
            {
                debug::print(STREAM, "Server connected, sending request...\r\n");
                // Send HTTP GET request
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
                // This likely means the certificate file was not found at the specified path
                // on the PC, or the server certificate did not match.
                debug::print(STREAM, "Unable to connect to server\r\n");
            }
        }
        else
        {
            debug::print(STREAM, "Failed to send setCACert command\r\n");
        }
    }
}