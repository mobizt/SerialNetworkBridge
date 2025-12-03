/**
 * =========================================================
 * Host Management Example (for client to work with PC Host)
 * =========================================================
 * Runs on: Any Arduino device (as a client).
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
 * 
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * ENABLE_LOCAL_DEBUG The macro to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *                    It shiukd not defined in this example.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be defined in this example.
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define HOST_RELAY_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeRate 115200

SerialHostManager manager(Serial);

#if defined(HOST_RELAY_DEBUG)
#define STREAM manager
#else
#define STREAM Serial
#endif

// WiFi Credentials for testing (Update these!)
const char *new_ssid = "SSID";
const char *new_pass = "Password";

void setup()
{
    Serial.begin(BridgeRate);
    while (!Serial)
        ;

    Serial.write(0x00);
    delay(500);

    // Ping host
    debug::print(STREAM, "Pinging host...\r\n");
    if (!manager.pingHost())
    {
        // This debug message will not be unable to send to host PC.
        Serial.println("No response from host. Please make sure serial_bridge.py is running...");
        while (1)
            delay(100);
    }
    debug::print(STREAM, "Success\r\n");

    delay(2000);

    debug::print(STREAM, "Disconnecting Current WiFi...\r\n");

    // Disconnect from WiFi AP
    if (manager.disconnectNetwork())
        debug::print(STREAM, "Success\r\n");
    else
        debug::print(STREAM, "Failed\r\n");

    delay(5000);

    // Checks if host has internet access
    debug::print(STREAM, "Checking the network connection...\r\n");

    if (manager.isNetworkConnected())
        debug::print(STREAM, "WiFi is connected.\r\n");
    else
        debug::print(STREAM, "WiFi is disconnected.\r\n");

    delay(500);

    // Set WiFi credentials
    debug::print(STREAM, "Setting the WiFi AP..,\r\n");

    if (manager.setWiFi(new_ssid, new_pass))
        debug::print(STREAM, "Success\r\n");
    else
        debug::print(STREAM, "Failed\r\n");

    delay(500);

    // Tell host to connect to the new network
    debug::print(STREAM, "Connecting to the new WiFi AP...\r\n");

    if (manager.connectNetwork())
        debug::print(STREAM, "Success\r\n");
    else
        debug::print(STREAM, "Failed\r\n");

    delay(10000);

    // Checks if host has internet access
    debug::print(STREAM, "Checking the network connection...\r\n");

    if (manager.isNetworkConnected())
        debug::print(STREAM, "WiFi is connected.\r\n");
    else
        debug::print(STREAM, "WiFi is disconnected.\r\n");

    delay(5000);

    // Reboot host
    debug::print(STREAM, "Rebooting the host...\r\n");
    if (manager.rebootHost())
        debug::print(STREAM, "Success\r\n");
    else
        debug::print(STREAM, "Failed\r\n");

    debug::print(STREAM, "Test complete.\r\n");
}

void loop()
{
}