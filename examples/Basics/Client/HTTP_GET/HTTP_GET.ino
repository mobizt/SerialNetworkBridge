/**
 * ===============================================
 * HTTP GET
 * ===============================================
 * Runs on: The any Arduino device.
 * Host: Requires the "Basics/Host" sketch running on the host.
 * Purpose: Demonstrates a HTTP GET request.
 */

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient
#include <SerialNetworkBridge.h>

// Serial TCP Client Config
const int CLIENT_SLOT = 0;       // Coresponding to Network client or SSL client slot 0 on the host
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the host Serial

SerialTCPClient client(Serial2, CLIENT_SLOT);

const char *server = "httpbin.org";
const int port = 443; // Please ensure that the client on slot 0 of the host side
                     // can handle HTTPS (port 443) requests.

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial2.begin(SERIAL_BAUD);
  client.setLocalDebugLevel(1); // Enable debug prints

  // To set up new WiFi AP, connecting to new WiFi AP,
  // please see examples/Features/HostManagement/Client/Client.ino

  Serial.print("Pinging host... ");
  if (!client.pingHost())
  {
    Serial.println("failed");
    Serial.println("Please check wiring/Serial configuration and ensure the host is running.");
    while (1)
      delay(100);
  }
  Serial.println("success");

  // Connect to the server
  Serial.print("Connecting to ");
  Serial.print(server);
  Serial.print("... ");

  if (!client.connect(server, port))
  {
    Serial.println("failed");
    return;
  }

  Serial.println("success!");

  Serial.println("Sending simple HTTP GET request...");

  // Send the HTTP GET request
  client.print("GET / HTTP/1.1\r\n");
  client.print("Host: ");
  client.print(server);
  client.print("\r\n");
  client.print("Connection: close\r\n\r\n");

  while (client.available() == 0)
  {
    if (!client.connected())
    {
      Serial.println("Disconnected while waiting.");
      break;
    }
    delay(0);
  }

  Serial.println("Response Received");
  while (client.available() > 0)
  {
    int c = client.read();
    if (c >= 0)
    { // Check for -1 (which read() returns)
      Serial.print((char)c);
    }
  }
  Serial.println("\nEnd of Response");

  client.stop();
  Serial.println("Connection stopped.");
}

void loop()
{
}