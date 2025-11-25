/**
 * ===============================================
 * PC USB Host HTTP Client Example (Revised)
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to configure your device to connect to
 * the 'serial_bridge.py' script running on your computer via the USB cable.
 * It pings the bridge and sends an HTTP GET request to httpbin.org.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor (The USB port is used for the bridge).
 * 4. Run 'python serial_bridge.py' on your computer.
 * 5. Watch the Built-in LED:
 * - Fast Flashing (20x): Connected to Server (httpbin.org).
 * - Short Blip (2x):     Data received from Server.
 * - Slow Blink (5x):     Error (Ping failed or Connection failed).
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

  // [CRITICAL FIX] Flush bootloader noise so the PC Python script 
  // doesn't get confused by random bytes on startup.
  Serial.write(0x00); 
  delay(500);

#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  // Start OFF (Assuming Active LOW for most builtin LEDs)
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}

void loop()
{
  // 1. Ping the PC Host first to ensure bridge is running
  if (client.pingHost(500))
  {
    // 2. Connect to the server
    // Matches WS_EVENT_CONNECTED style
    if (client.connect("httpbin.org", 443))
    {
      // Step 1: Connection Established (Flash LED Fast)
      flashLED(20, 50);

      // Send HTTP GET request
      client.println("GET /get HTTP/1.1");
      client.println("Host: httpbin.org");
      client.println("Connection: close");
      client.println();

      // Wait for response
      while (client.connected() || client.available())
      {
        if (client.available())
        {
          // Step 2: Data Received (Short Double Blip)
          // We blink once when we start receiving data to indicate "Message Received"
          flashLED(2, 50);
          
          // Consume all available data (we can't print to Serial because it's the bridge)
          while(client.available()) {
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

  // Wait 5 seconds before repeating
  delay(5000);
}