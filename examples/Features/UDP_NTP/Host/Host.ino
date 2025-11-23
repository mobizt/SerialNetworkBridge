/**
 * ===============================================
 * Serial Network Host (UDP Support)
 * ===============================================
 * Runs on: ESP32, ESP8266, or any board with WiFi/UDP.
 * Purpose: Acts as the serial-to-WiFi bridge for UDP datagrams.
 */

#include <WiFi.h>
#include <WiFiUdp.h> // Native WiFi UDP client

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialNetworkHost
#include <SerialNetworkHost.h> // Use the new Host name

// Network Config
const char* ssid     = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

// Serial Configuration
const long SERIAL_BAUD = 115200;

// Host Instantiation
// We use a native WiFiUDP client and assign it to a slot in the host bridge
WiFiUDP udpClient;
SerialNetworkHost host(Serial2); // Use Serial2 for communication

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[Host] Serial Network Host Starting (UDP)...");

  // Initialize serial link to the client
  Serial2.begin(SERIAL_BAUD);

  // Connect to WiFi
  Serial.print("[Host] Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Host] WiFi Connected!");

  // Set up the UDP slot
  // Assign the native WiFiUDP object to slot 0.
  host.setUDPClient(&udpClient, 0 /* slot */);
  host.setLocalDebugLevel(1);

  // Notify the client that the host is ready
  host.notifyBoot();
  Serial.println("[Host] Ready. Listening for client UDP commands...");
}

void loop() 
{
  // Reqouirements for Host operation
  host.loop();
}