/**
 * ===============================================
 * PC USB Host HTTP Streaming (SSE) Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Server: Requires the 'sse_server.py' running on your PC (port 5000).
 * Purpose: Demonstrates reading a "Chunked" Server-Sent Event stream via the PC USB bridge.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor.
 * 4. Start the SSE Server: Run 'examples/Features/PC_USB_Host/HTTP_Streaming/Server/run.bat' (or .sh).
 * 5. Start the Bridge: Run 'python serial_bridge.py' in a separate terminal.
 * 6. Watch the Built-in LED:
 * - Fast Flashing (20x): Connected to SSE Server.
 * - Short Blip (2x):     Data Chunk (Event) received.
 * - Slow Blink (5x):     Error.
 */

// [CRITICAL] Define this BEFORE including SerialNetworkBridge to enable chunk decoding
#define ENABLE_SERIALTCP_CHUNKED_DECODING

// CRITICAL: Debugging MUST be disabled
// #define ENABLE_SERIALTCP_DEBUG

#include <Arduino.h>

// Define LED_BUILTIN if not defined by the board package
#ifndef LED_BUILTIN
#if defined(ESP8266)
// Most ESP8266 modules (NodeMCU, Wemos) use GPIO 2
#define LED_BUILTIN 2
#elif defined(ESP32)
// Many ESP32 dev boards use GPIO 2
#define LED_BUILTIN 2
#elif defined(STM32F1xx)
// "Blue Pill" STM32 usually uses PC13
#define LED_BUILTIN PC13
#else
// Default for Arduino Uno, Mega, Nano, Leonardo, etc.
#define LED_BUILTIN 13
#endif
#endif

#include <SerialNetworkBridge.h>

// Use the main USB Serial port (Slot 0)
SerialTCPClient client(Serial, 0);

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

void flashLED(int times, int delayMs)
{
#if defined(LED_BUILTIN)
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayMs);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayMs);
    }
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // [CRITICAL FIX] Flush bootloader noise
    Serial.write(0x00);
    delay(500);

#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Start OFF
#endif
}

void connectServer()
{
    // 1. Ping the PC Host first
    if (client.pingHost(500))
    {
        // 2. Connect to the server
        // Note: We use the default connect() which assumes plain TCP.
        // The Python SSE server runs on HTTP (port 5000), not HTTPS.
        // So we do NOT pass 'true' as the 3rd argument.
        if (client.connect(host, port))
        {
            // Step 1: Connection Established (Flash LED Fast)
            flashLED(20, 50);

            String header = "GET ";
            header += uri;
            header += " HTTP/1.1\r\n";
            header += "Host: ";
            header += host;
            header += "\r\n";
            header += "Connection: keep-alive\r\n";
            header += "\r\n";

            client.print(header);

            // Reset state for new connection
            state = READ_HEADERS;
            chunkSize = 0;
            bytesReadInChunk = 0;
        }
        else
        {
            // Error: Connection Failed (Slow Blink)
            flashLED(5, 500);
            delay(2000);
        }
    }
    else
    {
        // Error: Ping Failed (Slow Blink)
        flashLED(5, 500);
        delay(2000);
    }
}

void loop()
{
    // Auto-reconnect if disconnected
    if (!client.connected())
    {
        connectServer();
        return;
    }

    if (client.available())
    {
        switch (state)
        {
        case READ_HEADERS:
        {
            // Read headers until empty line
            // readStringUntil consumes the \n, so we check for \r
            String line = client.readStringUntil('\n');
            if (line == "\r")
            {
                state = READ_CHUNK_SIZE;
            }
            break;
        }

        case READ_CHUNK_SIZE:
        {
            // Use the library's built-in helper to parse hex size
            long size = client.readChunkSize();

            if (size >= 0)
            {
                chunkSize = size;
                bytesReadInChunk = 0;

                if (chunkSize == 0)
                {
                    // End of stream (0 chunk)
                    client.stop();
                }
                else
                {
                    state = READ_CHUNK_DATA;
                    // Step 2: New Chunk Started (Short Double Blip)
                    // This indicates we are receiving live data events from sse_server.py
                    flashLED(2, 50);
                }
            }
            break;
        }

        case READ_CHUNK_DATA:
        {
            if (chunkSize > 0)
            {
                // Read available bytes, but don't exceed current chunk
                long available = client.available();
                long remaining = chunkSize - bytesReadInChunk;
                int bytesToRead = (int)min(available, remaining);

                while (bytesToRead > 0)
                {
                    int b = client.read();
                    if (b != -1)
                    {
                        // Process byte (Note: We cannot print to Serial!)
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
            // Consume CRLF that follows every chunk
            if (client.available() >= 2)
            {
                client.read(); // \r
                client.read(); // \n
                state = READ_CHUNK_SIZE;
            }
            break;
        }
        }
    }
}