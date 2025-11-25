/**
 * ===============================================
 * PC USB Host Host Management Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the UPDATED 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates using "Global Commands" to control the Host PC.
 * WARNING:
 * The 'rebootHost' command will REBOOT YOUR COMPUTER if running as Administrator/Root.
 * The WiFi commands will disconnect/connect your PC's actual Wi-Fi interface.
 * Use with caution!
 *
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor.
 * 4. Run 'python serial_bridge.py' on your computer (Admin/Sudo required for WiFi/Reboot).
 * 5. Watch the Built-in LED for status.
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

// Use SerialHostManager on the main USB Serial port
SerialHostManager manager(Serial);

// WiFi Credentials for testing (Update these!)
const char *new_ssid = "MyWiFiSSID";
const char *new_pass = "MyPassword123";

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

    // [CRITICAL] Flush bootloader noise
    Serial.write(0x00);
    delay(500);

#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Start OFF
#endif

    // 1. Ping Host (Check Bridge)
    if (manager.pingHost(500))
    {
        flashLED(20, 50); // Ping Success
        delay(1000);

        // NETWORK STATUS CHECK
        // Checks if PC has internet access (e.g. can reach 8.8.8.8)
        if (manager.isNetworkConnected())
        {
            // PC is Online
            flashLED(20, 50);
        }
        else
        {
            // PC is Offline
            flashLED(5, 500);
        }
        delay(2000);

        // WIFI MANAGEMENT (Comment out to skip)

        // 2. Disconnect Current WiFi
        if (manager.disconnectNetwork())
        {
            flashLED(2, 100); // Disconnect command sent
        }
        delay(5000); // Wait for OS to disconnect

        // 3. Set New WiFi Credentials
        // This just stores the SSID/Pass in the Python script for the next connect command
        if (manager.setWiFi(new_ssid, new_pass))
        {
            flashLED(2, 100); // Credentials sent
        }
        delay(500);

        // 4. Connect to New Network
        // This triggers the OS command (netsh/nmcli) on the PC
        if (manager.connectNetwork())
        {
            flashLED(20, 50); // Connection command accepted
        }
        else
        {
            flashLED(5, 500); // Command failed
        }

        // Give the PC some time to associate and get an IP
        delay(10000);

        // Check connection again
        if (manager.isNetworkConnected())
        {
            flashLED(50, 20); // Successfully reconnected!
        }
        else
        {
            flashLED(5, 500); // Failed to get internet
        }

        // SYSTEM REBOOT (WARNING: REBOOTS PC)
        // Uncomment below to test PC Reboot
        /*
        delay(5000);
        if (manager.rebootHost())
        {
           // Fast Flash indicates command sent successfully
           // PC will reboot in 10 seconds
           flashLED(50, 20);
        }
        */
    }
    else
    {
        // Ping Failed (Bridge Down)
        flashLED(5, 500);
        while (1)
            delay(100);
    }
}

void loop()
{
    // Test Complete.
    delay(1000);
}