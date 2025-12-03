/**
 * ===================================================
 * HTTP POST Example (for client to work with PC Host)
 * ===================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to configure your device to connect to
 * the 'serial_bridge.py' script running on your computer via the USB cable.
 * It pings the bridge and sends an HTTP POST request to reqres.in.
 * INSTRUCTIONS:
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
#define BridgeRate 115200 // Change this to match with the host serial baud rate.

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
  if (hostReady && (ms == 0 || millis() - ms > 10000))
  {
    ms = millis();

    debug::print(STREAM, "Connecting to server...\r\n");

    // [CRITICAL] The 3rd parameter 'true' enables SSL/TLS (HTTPS).
    // Unlike the Arduino device Host, the PC Host script REQUIRES this flag to know
    // it must wrap the socket in SSL. If omitted, it attempts a plain
    // TCP connection to port 443, which the server will reject.
    if (client.connect("jsonplaceholder.typicode.com", 443))
    {
      debug::print(STREAM, "Server connected, sending request...\r\n");

      const char *request =
          "POST /posts HTTP/1.1\r\n"
          "Host: jsonplaceholder.typicode.com\r\n"
          "Content-Type: application/json\r\n"
          "Content-Length: 34\r\n"
          "Connection: close\r\n"
          "\r\n"
          "{\"name\":\"morpheus\",\"job\":\"leader\"}";

      // Send the request
      client.write((const uint8_t *)request, strlen(request));

      // Wait for response
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