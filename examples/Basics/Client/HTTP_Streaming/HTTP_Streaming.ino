/**
 * ===============================================
 * HTTP Streaming (SSE) Client Example
 * ===============================================
 * Runs on: Any Arduino device (Client) with a second Serial port.
 * Host: Requires a Arduino device host (ESP32/ESP8266) connected via Serial port.
 * Server: Simulate your PC as server which requires 'sse_server.py' running on your PC.
 * Your PC and the Device Host must be on the SAME network.
 *
 * Purpose: Demonstrates reading a chunked HTTP stream from the Python server.
 */

// Define this BEFORE including SerialTCPClient.h to enable chunk decoding methods
#define ENABLE_SERIALTCP_CHUNKED_DECODING
#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient

#include <SerialNetworkBridge.h>

// Serial TCP Client Config
const int CLIENT_SLOT = 0;       // Slot 0 on the Device Host
const long SERIAL_BAUD = 115200; // Baud rate for Serial2 (Bridge)

// Use Serial2 to talk to the ESP32 Host
SerialTCPClient client(Serial2, CLIENT_SLOT);

// SERVER CONFIGURATION
// Your PC's Local IP Address (e.g., "192.168.1.50").
// On Windows, run 'ipconfig' in cmd to find this.
// On Mac/Linux, run 'ifconfig' or 'ip a'.
String host = "192.168.1.XXX"; // <--- UPDATE THIS WITH YOUR PC's IP
String uri = "/stream";        // Endpoint defined in sse_server.py
uint16_t port = 5000;          // Default Flask port

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

bool connectServer()
{
  Serial.println();
  Serial.print("Connecting to server (");
  Serial.print(host);
  Serial.print(":");
  Serial.print(port);
  Serial.println(")...");

  // Connect to the server (Plain HTTP)
  // The Device Host (ESP32) handles the actual WiFi connection.
  if (client.connect(host.c_str(), port))
  {
    Serial.println("Connected!");

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
    client.stop();
    Serial.println("Connection failed.");
    delay(2000);
  }

  return false;
}

void setup()
{
  // Debug Serial (USB to PC)
  Serial.begin(115200);
  delay(1000);

  // Bridge Serial (To ESP32 Host)
  Serial2.begin(SERIAL_BAUD);

  client.setLocalDebugLevel(1); // Enable library debug prints

  Serial.print("Pinging host... ");
  if (!client.pingHost())
  {
    Serial.println("failed");
    Serial.println("Check wiring on Serial2 and ensure ESP32 Host is running.");
    while (1)
      delay(100);
  }
  Serial.println("success");

  Serial.println("Client is ready. Attempting connection...");
  connectServer();
}

void loop()
{
  // Auto-reconnect if disconnected
  if (!client.connected())
  {
    Serial.println("Disconnected. Reconnecting...");
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
      String line = client.readStringUntil('\n');

      // Check for end of headers (Empty line is just \r)
      if (line == "\r")
      {
        Serial.println("--- Headers End. Stream Starting");
        state = READ_CHUNK_SIZE;
      }
      else
      {
        Serial.print("Header: ");
        Serial.println(line);
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
        Serial.print("[Chunk Size: ");
        Serial.print(chunkSize);
        Serial.println("]");

        if (chunkSize == 0)
        {
          Serial.println("End of stream (0 chunk)");
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
            Serial.write((char)b); // Print stream data to Monitor
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