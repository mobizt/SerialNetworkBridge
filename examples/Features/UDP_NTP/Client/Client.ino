/**
 * ===============================================
 * UDP NTP Client Example
 * ===============================================
 * Runs on: Arduino device (e.g., Uno, Nano).
 * Host: Requires the "UDP Host" sketch running on the bridge device.
 * Purpose: Demonstrates initiating a connectionless UDP exchange 
 * (Network Time Protocol request and response).
 */

#include <WiFi.h>
#define ENABLE_SERIALTCP_DEBUG // Enable debug prints
#include <SerialNetworkBridge.h> 

// Serial Configuration
const int CLIENT_SLOT = 0;
const long SERIAL_BAUD = 115200;

// Instantiate the clients
SerialUDPClient udpClient(Serial2, CLIENT_SLOT); 

// NTP Server Details
const char* ntpServerName = "pool.ntp.org";
const int ntpPort = 123;
const int localPort = 2390;

// NTP packet buffer (48 bytes is the standard NTP packet size)
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

void sendNTPpacket(const char* host) 
{
  // Begin the packet transmission
  if (!udpClient.beginPacket(host, ntpPort)) 
  {
    Serial.println("Error: Failed to begin packet!");
    return;
  }

  // Initialize the packet buffer
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // NTP header setup (Leap Indicator, Version Number, Mode)
  packetBuffer[0] = 0b11100011; 

  // Write the buffer to the outgoing stream
  udpClient.write(packetBuffer, NTP_PACKET_SIZE);

  // Finalize and send the packet over the serial bridge
  if (udpClient.endPacket())
  {
    Serial.println("Success: NTP packet sent.");
  }
  else
  {
    Serial.println("Error: Failed to finalize/send packet!");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Serial2.begin(SERIAL_BAUD);
  
  udpClient.setLocalDebugLevel(1);

  // To set up new WiFi AP, connecting to new WiFi AP,
  // please see examples/Features/HostManagement/Client/Client.ino

  Serial.print("Pinging host...");
  if (!udpClient.pingHost())
  {
    Serial.println("failed");
    while (1)
      delay(100);
  }
  Serial.println("success!");
  
  // Start UDP listening on a local port (on the host side)
  Serial.print("Starting UDP listener on port ");
  Serial.print(localPort);
  Serial.print("...");
  if (udpClient.begin(localPort))
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("Failed to start UDP listener.");
    while (1) delay(1000);
  }
  
  // Send the NTP request
  Serial.println("Sending NTP request...");
  sendNTPpacket(ntpServerName);
}

void loop()
{
  // Check for response from the host bridge
  int packetSize = udpClient.parsePacket();
  
  if (packetSize) 
  {
    Serial.print("Received packet of size ");
    Serial.print(packetSize);
    Serial.print(" from ");
    Serial.print(udpClient.remoteIP());
    Serial.print(":");
    Serial.println(udpClient.remotePort());

    // Read the packet data
    udpClient.read(packetBuffer, NTP_PACKET_SIZE);
    
    // ... (Time conversion logic remains the same) ...
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;

    Serial.print("Current Epoch Time: ");
    Serial.println(epoch);
    
    // Test complete, stop the listener
    udpClient.stop();
    Serial.println("UDP Test Complete.");
    while (1) delay(1000);
  }
}