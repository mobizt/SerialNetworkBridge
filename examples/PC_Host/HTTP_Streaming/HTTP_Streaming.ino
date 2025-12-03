/**
 * ==============================================================
 * HTTP Streaming (SSE) Example (for client to work with PC Host)
 * ==============================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Server: Requires the 'sse_server.py' running on your PC (port 5000).
 * Purpose: Demonstrates reading a "Chunked" Server-Sent Event stream via the PC USB bridge.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor.
 * 4. Start the SSE Server: Run 'examples/Features/PC_USB_Host/HTTP_Streaming/Server/run.bat' (or .sh).
 * 5. Start the Bridge: Run 'python serial_bridge.py' in a separate terminal.
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
 * 
 * ENABLE_SERIALTCP_CHUNKED_DECODING Allows the chunk decoding in SerialTCPClient.
 */

#define ENABLE_SERIALTCP_CHUNKED_DECODING
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

// SERVER CONFIGURATION
// Configuration for the local Python SSE server (sse_server.py)
const char *host = "localhost";
const char *uri = "/stream";
const int port = 5000;

// State machine for chunked decoding
enum ChunkState
{
  READ_HEADERS,
  READ_CHUNK_SIZE,
  READ_CHUNK_DATA,
  READ_CHUNK_CRLF
};

ChunkState state = READ_HEADERS;
long chunkSize = 0;
long bytesReadInChunk = 0;
bool headerStarted = false;

void setup()
{
  Serial.begin(115200);
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

bool connectServer()
{
  debug::print(STREAM, "Connecting to server...\r\n");
  // Connect to the server
  // Note: We use the default connect() which assumes plain TCP.
  // The Python SSE server runs on HTTP (port 5000), not HTTPS.
  // So we do NOT pass 'true' as the 3rd argument.
  if (client.connect(host, port))
  {
    debug::print(STREAM, "Server connected, sending request...\r\n");

    String header = "GET ";
    header += uri;
    header += " HTTP/1.1\r\n";
    header += "Host: ";
    header += host;
    header += "\r\n";
    header += "Connection: keep-alive\r\n";
    header += "\r\n";

    int ret = client.print(header);

    // Reset state for new connection
    state = READ_HEADERS;
    chunkSize = 0;
    bytesReadInChunk = 0;

    return ret == header.length();
  }
  else
  {
    debug::print(STREAM, "Unable to connect to server\r\n");
  }

  return false;
}

void loop()
{
  // Auto-reconnect if disconnected
  if (!client.connected())
  {
    debug::print(STREAM, "Server disconnected. Reconnecting...\r\n");
    if (!connectServer())
      return;
  }

  if (client.available())
  {
    switch (state)
    {
    case READ_HEADERS:
    {
      // Read headers until empty line
      char buffer[128];
      int ret = 0, totalRead = 0;
      char endToken = '\n'; // start by reading lines
      ret = debug::readResonse(CLIENT, STREAM, buffer, 128, totalRead, endToken);

      // Check for end of headers (Empty line is just \r)
      if (strcmp(buffer, "\r\n") == 0)
      {
        debug::print(STREAM, "--- Headers End ---\r\n");
        debug::print(STREAM, "--- Stream Starting ---\r\n\r\n");
        state = READ_CHUNK_SIZE;
      }
      else
      {
        if (!headerStarted)
        {
          debug::print(STREAM, "--- Headers Start ---\r\n");
          headerStarted = true;
        }
        debug::print(STREAM, buffer);
        debug::printNewLine(STREAM);
      }
      break;
    }

    case READ_CHUNK_SIZE:
    {
      // Parse the hex chunk size sent by Flask/Server
      long size = client.readChunkSize();

      if (size >= 0)
      {
        chunkSize = size;
        bytesReadInChunk = 0;
        debug::print(STREAM, "Reading Chunk [");
        debug::printRaw(STREAM, String(chunkSize).c_str());
        debug::printRaw(STREAM, "]... ");

        if (chunkSize == 0)
        {
          debug::print(STREAM, "End of stream (0 chunk)\r\n");
          client.stop();
        }
        else
        {
          state = READ_CHUNK_DATA;
        }
      }
      break;
    }

    case READ_CHUNK_DATA:
    {
      if (chunkSize > 0)
      {
        long available = client.available();
        long remaining = chunkSize - bytesReadInChunk;
        int bytesToRead = (int)min(available, remaining);

        while (bytesToRead > 0)
        {
          int b = client.read();
          if (b != -1)
          {
            debug::printRaw(STREAM, String((char)b).c_str()); // Print stream data to Monitor
            bytesReadInChunk++;
            bytesToRead--;
          }
          else
          {
            break;
          }
        }

        if (bytesReadInChunk >= chunkSize)
        {
          state = READ_CHUNK_CRLF;
        }
      }
      else
      {
        debug::print(STREAM, "End of stream (0 chunk)\r\n");
      }
      break;
    }

    case READ_CHUNK_CRLF:
    {
      // Consume the CRLF that follows every chunk
      if (client.available() >= 2)
      {
        client.read();           // \r
        client.read();           // \n
        state = READ_CHUNK_SIZE; // Ready for next chunk
      }
      break;
    }
    }
  }
}