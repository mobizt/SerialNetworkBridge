/**
 * ===============================================
 * PC USB Host HTTP Client Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to configure your device to connect to
 * the 'serial_bridge.py' script running on your computer via the USB cable.
 * It pings the bridge and sends an HTTP GET request to httpbin.org.
 * * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor (The USB port is used for the bridge).
 * 4. Run 'python serial_bridge.py' on your computer.
 * 5. Watch the Built-in LED:
 * - 1 Short Flash:  Pong received (Bridge is active).
 * - 3 Fast Flashes: HTTP Request successful (Internet access working).
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

// Use the main USB Serial port
SerialTCPClient client(Serial, 0);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}

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

void loop()
{
  // 1. Ping the PC Host first to ensure bridge is running
  if (client.pingHost(500))
  {

    // Step 1: Pong Received (Flash LED Once)
    flashLED(20, 50);

    // Step 2: Send HTTP Request
    if (client.connect("httpbin.org", 443))
    {

      client.println("GET /get HTTP/1.1");
      client.println("Host: httpbin.org");
      client.println("Connection: close");
      client.println();

      // Read and consume the response
      while (client.connected() || client.available())
      {
        if (client.available())
        {
          client.read();
        }
      }
      client.stop();

      // Step 3: HTTP Success (Flash LED Twice quickly)
      // Total visual sequence: Flash (Pong)... Flash-Flash (HTTP)
      flashLED(60, 50);
    }
  }

  // Wait 5 seconds before repeating
  delay(5000);
}