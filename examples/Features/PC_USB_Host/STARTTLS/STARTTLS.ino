/**
 * ===============================================
 * PC USB Host STARTTLS Client Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to upgrade a plain connection to a secure one (STARTTLS).
 * It connects to smtp.gmail.com:587, sends STARTTLS, and upgrades the link.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor.
 * 4. Run 'python serial_bridge.py' on your computer.
 * 5. Watch the Built-in LED:
 * - Fast Flashing (20x): Connected (Plain) to Server.
 * - Short Blip (2x):     STARTTLS Upgrade Successful.
 * - Slow Blink (5x):     Error (Ping failed, Connect failed, or Upgrade failed).
 */

// [CRITICAL] Debugging MUST be disabled
// #define ENABLE_SERIALTCP_DEBUG

#include <Arduino.h>
#include <SerialNetworkBridge.h>

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

// Use the main USB Serial port (Slot 0)
SerialTCPClient client(Serial, 0);

const char *server = "smtp.gmail.com";
const int port = 587; // Standard STARTTLS port

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

// Helper to read a line from the client
String readLine()
{
    String line = "";
    while (client.available() == 0)
    {
        if (!client.connected())
            return "";
        delay(1);
    }
    while (client.available() > 0)
    {
        char c = client.read();
        if (c == '\n')
            break;
        if (c != '\r')
            line += c;
    }
    return line;
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
        // 2. Connect in PLAIN mode
        // Note: 3rd param is 'false' (No SSL yet)
        if (client.connect(server, port, false))
        {
            // Step 1: Connected Plain (Fast Flash)
            flashLED(20, 50);

            // Read Greeting (e.g., "220 smtp.gmail.com ...")
            readLine();

            // 3. Send EHLO
            client.println("EHLO my-arduino-client.com");
            client.flush();

            // Read multi-line response
            bool found_starttls = false;
            while (true)
            {
                String line = readLine();
                if (line.startsWith("250-STARTTLS"))
                {
                    found_starttls = true;
                }
                if (line.startsWith("250 "))
                { // Last line of EHLO
                    break;
                }
            }

            if (found_starttls)
            {
                // 4. Send STARTTLS command
                client.println("STARTTLS");
                client.flush();
                readLine(); // Read "220 2.0.0 Ready to start TLS"

                // 5. Upgrade our *proxy* connection
                // This tells the Python script to wrap the existing socket in SSL
                if (client.startTLS())
                {
                    // Step 2: Upgrade Successful (Short Double Blip)
                    flashLED(2, 50);

                    // 6. Send EHLO *again* (over new secure channel)
                    client.println("EHLO my-arduino-client.com");
                    client.flush();

                    // Read the new, secure EHLO response
                    while (true)
                    {
                        String line = readLine();
                        if (line.startsWith("250 "))
                        {
                            break;
                        }
                    }

                    // Step 3: Protocol Complete (Short Double Blip again)
                    flashLED(2, 50);
                }
                else
                {
                    // Error: Upgrade Failed
                    flashLED(5, 500);
                }
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

    // Wait 10 seconds before repeating
    delay(10000);
}