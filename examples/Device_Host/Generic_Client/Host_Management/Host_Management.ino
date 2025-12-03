/**
 * =============================================================
 * HOST Management Example (for client to work with Device Host)
 * =============================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Purpose: Demonstrates how to use the "global"
 * commands to remotely control the host.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should be matched with host device serial baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSerial       The serial port that is connected to host device serial port.
 *
 * ENABLE_LOCAL_DEBUG The macro to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be commented out or undefined in this example (debug info is printed locally).
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define ENABLE_LOCAL_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeSlot_0 0
#define BridgeRate 115200    // Change this to match with the host serial baud rate.
#define BridgeSerial Serial2 // Change this to match your hardware.

SerialHostManager manager(BridgeSerial);

#if defined(HOST_RELAY_DEBUG)
#define STREAM manager
#else
#define STREAM Serial
#endif

unsigned long ms = 0;
bool hostReady = false;

// WiFi Credentials for testing (Update these!)
const char *new_ssid = "SSID";
const char *new_pass = "Password";

void setup()
{
  Serial.begin(115200);
  delay(1000);

  BridgeSerial.begin(BridgeRate);

  manager.setLocalDebugLevel(1); // Enable debug prints

  // Ping host
  debug::print(STREAM, "Pinging host...\r\n");
  if (!manager.pingHost())
  {
    Serial.println("No response from host. Check serial port, baud rate and host device...");
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

  // Reboot host device
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