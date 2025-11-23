/**
 * ===============================================
 * Global_Commands (Client)
 * ===============================================
 * Runs on: The client device.
 * Host: Requires the corresponding "Host.ino" for this feature.
 * Purpose: Demonstrates how to use the "global"
 * commands to remotely control the host.
 */
#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialHostManager
#include <SerialNetworkBridge.h>

// Seial Host Manager Config
// We don't need a slot for global commands
SerialHostManager manager(Serial2);

const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the host Serial

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial2.begin(SERIAL_BAUD);
  manager.setLocalDebugLevel(1);

  Serial.print("Pinging host... ");
  if (!manager.pingHost())
  {
    Serial.println("failed");
    while (1)
      delay(100);
  }
  Serial.println("success");

  // Set new WiFi credentials
  Serial.print("Setting the WiFi AP... ");

  if (manager.setWiFi("SSID", "Password"))
    Serial.println("success");
   else
    Serial.println("failed");

  delay(1000);

  // Tell host to connect to the new network
  Serial.print("Connecting to the new WiFi AP... ");

  if (manager.connectNetwork())
    Serial.println("success");
  else
    Serial.println("failed");

  delay(1000);

  // Check if host is connected
  Serial.print("Checking the network connection... ");

  if (manager.isNetworkConnected())
    Serial.println("connected");
  else
    Serial.println("disconnected");

  delay(1000);

  // Tell host to reboot
  Serial.print("Rebooting the host... ");
  if (manager.rebootHost())
    Serial.println("success");
  else
    Serial.println("failed");

  Serial.println("Test complete.");
}

void loop()
{
}