/**
 * ===============================================
 * PC USB Host HTTP POST Example (Fixed SSL)
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to send a POST request via the PC USB bridge.
 */

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

const char *server = "reqres.in";
const int port = 443;

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

void loop()
{
    // 1. Ping the PC Host first
    if (client.pingHost(500))
    {
        // 2. Connect to the server
        // [CRITICAL] The 3rd parameter 'true' enables SSL/TLS (HTTPS).
        // Unlike the Arduino device Host, the PC Host script REQUIRES this flag to know
        // it must wrap the socket in SSL. If omitted, it attempts a plain
        // TCP connection to port 443, which the server will reject.
        if (client.connect(server, port, true))
        {
            // Step 1: Connection Established (Flash LED Fast)
            flashLED(20, 50);

            const char *request_part1 =
                "POST /api/users HTTP/1.1\r\n"
                "Host: reqres.in\r\n"
                "x-api-key: reqres-free-v1\r\n"
                "Content-Type: application/json\r\n";

            const char *request_part2 =
                "Content-Length: 34\r\n"
                "Connection: close\r\n"
                "\r\n"
                "{\"name\":\"morpheus\",\"job\":\"leader\"}";

            // Send the request
            client.write((const uint8_t *)request_part1, strlen(request_part1));
            client.write((const uint8_t *)request_part2, strlen(request_part2));

            // Wait for response
            while (client.connected() || client.available())
            {
                if (client.available())
                {
                    // Step 2: Data Received (Short Double Blip)
                    flashLED(2, 50);

                    // Consume data (Bridge mode doesn't print to Serial)
                    while (client.available())
                    {
                        client.read();
                    }
                }
                delay(1);
            }
            client.stop();
        }
        else
        {
            // Error: Connection Failed (Slow Blink)
            flashLED(5, 500);
        }
    }
    else
    {
        // Error: Ping Failed (Slow Blink)
        flashLED(5, 500);
    }

    delay(5000);
}