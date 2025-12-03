/**
 * ===================================================================
 * HTTP Streaming (SSE) Example (for client to work with Device Host)
 * ===================================================================
 * Runs on: Any Arduino device (Client) with a second Serial port.
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Server: Simulate your PC as server which requires 'sse_server.py' running on your PC.
 * Your PC and the Device Host must be on the SAME network.
 *
 * Purpose: Demonstrates reading a chunked HTTP stream from the Python server.
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
 *
 * ENABLE_SERIALTCP_CHUNKED_DECODING Allows the chunk decoding in SerialTCPClient.
 */

#define ENABLE_SERIALTCP_CHUNKED_DECODING
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

bool connectServer()
{
  // This will set SSL I/O buffer (if SSL client on host device is supported)
  client.setBufferSizes(2048, 1024);

  // Set this to skip SSL certificate verification on SSL client assigned to slot 0 in host device.
  client.setInsecure();

  debug::print(STREAM, "Connecting to server...\r\n");

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