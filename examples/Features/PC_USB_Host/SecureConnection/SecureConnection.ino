/**
 * ===============================================
 * PC USB Host Secure HTTP Client Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates a secure HTTPS request using a specific CA Certificate.
 * It verifies httpbin.org against a local certificate file on the PC.
 *
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Certificate Setup:
 * - This example assumes you are running 'serial_bridge.py' from the folder:
 * 'examples/Features/PC_USB_Host/'
 * - The certificate file MUST be located at:
 * 'examples/Features/PC_USB_Host/cert/Amazon_Root_CA1.pem'
 * 3. Upload this sketch to your Arduino.
 * 4. Run 'python serial_bridge.py' on your computer.
 * 5. Watch the Built-in LED:
 * - Fast Flashing (20x): Connected Securely to Server.
 * - Short Blip (2x):     Response received.
 * - Slow Blink (5x):     Error (Cert missing, Connect failed, etc).
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

const char *server = "httpbin.org";
const int port = 443;

// [IMPORTANT] Certificate Path Configuration:
// The path provided below is RELATIVE to the working directory where you run 'serial_bridge.py'.
// If you run the script from 'examples/Features/PC_USB_Host/', then:
// "cert/Amazon_Root_CA1.pem" maps to -> "examples/Features/PC_USB_Host/cert/Amazon_Root_CA1.pem"
const char *ca_cert = "cert/Amazon_Root_CA1.pem";

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
    digitalWrite(LED_BUILTIN, HIGH); // Start OFF
#endif
}

void loop()
{
    // 1. Ping the PC Host first
    if (client.pingHost(500))
    {
        // 2. Set the CA Certificate
        // This tells the Python script to load the specific file defined in 'ca_cert'
        // for verification on the next connection attempt.
        if (client.setCACert(ca_cert))
        {
            // 3. Connect to the server (Secure)
            // 3rd param 'true' enables SSL/TLS (HTTPS)
            if (client.connect(server, port, true))
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
                // This likely means the certificate file was not found at the specified path
                // on the PC, or the server certificate did not match.
                flashLED(5, 500);
            }
        }
        else
        {
            // Error: Failed to send setCACert command (Slow Blink)
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