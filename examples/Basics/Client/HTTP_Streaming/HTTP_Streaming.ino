/**
 * ===============================================
 * HTTP Streaming (Server-Sent Events) Client Example
 * With Chunked Transfer Encoding Support
 * ===============================================
 * Runs on: Any Arduino device.
 * Host: Requires the "Basics/Host" sketch running on the host.
 * Server: Runs examples/Basics/HTTP_Streaming/Server/sse.php
 *
 * Purpose: Demonstrates reading a chunked HTTP stream.
 */

// Define this BEFORE including SerialTCPClient.h to enable chunk decoding methods
#define ENABLE_SERIALTCP_CHUNKED_DECODING
#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient

#include <SerialNetworkBridge.h>

// Serial TCP Client Config
const int CLIENT_SLOT = 0;       // Corresponding to Network client or SSL client slot 0 on the host
const long SERIAL_BAUD = 115200; // Corresponding to the baud rate used in the host Serial

SerialTCPClient client(Serial2, CLIENT_SLOT);

String host = "your_server_ip_or_host";
String uri = "/sse.php"; // if sse.php is located in the root directory.
uint16_t port = 443;

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
  Serial.print("Connecting to server...");

  if (client.connect(host.c_str(), port))
  {
    Serial.println(" ok");

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
    Serial.println(" failed\n");
    delay(2000);
  }

  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial2.begin(SERIAL_BAUD);
  client.setLocalDebugLevel(1); // Enable debug prints

  Serial.print("Pinging host... ");
  if (!client.pingHost())
  {
    Serial.println("failed");
    Serial.println("Please check wiring/Serial configuration and ensure the host is running.");
    while (1)
      delay(100);
  }
  Serial.println("success");

  Serial.println("Client is ready. Will attempt connection in loop().");

  connectServer();
}

void loop()
{
  if (!client.connected())
  {
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
      // Note: readStringUntil is blocking but efficient enough here
      String line = client.readStringUntil('\n');

      // Simple check for end of headers (CRLF)
      // readStringUntil strips the delimiter (\n), so we check for \r
      if (line == "\r")
      {
        Serial.println("--- Headers End ---");
        state = READ_CHUNK_SIZE;
      }
      else
      {
        // Optional: You could check for "Transfer-Encoding: chunked" here
        Serial.print("Header: ");
        Serial.println(line);
      }
      break;
    }

    case READ_CHUNK_SIZE:
    {
      // Use the library's built-in helper!
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
          // Or handle trailers if needed
        }
        else
        {
          state = READ_CHUNK_DATA;
        }
      }
      // If -1 (error/timeout), loop will retry
      break;
    }

    case READ_CHUNK_DATA:
    {
      if (chunkSize > 0)
      {
        // Read up to available bytes, but not more than remaining chunk
        long available = client.available();
        long remaining = chunkSize - bytesReadInChunk;
        int bytesToRead = (int)min(available, remaining);

        while (bytesToRead > 0)
        {
          int b = client.read();
          if (b != -1)
          {
            Serial.write((char)b);
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
      // After chunk data, there is a CRLF. We need to consume it.
      // We might need to wait for 2 bytes.
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