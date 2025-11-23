/**
 * ===============================================
 * HTTP GET
 * ===============================================
 * Runs on: Arduino UNO.
 * Host: Requires the "Basics/Host" sketch running on the host.
 * Purpose: Demonstrates a HTTP GET request.
 */

// For debugging
// Remove ENABLE_SERIALTCP_DEBUG for AVR 
// to save ram and flash usage on production
#define ENABLE_SERIALTCP_DEBUG

#include <SoftwareSerial.h>
#include <SerialTCPClient.h>

// Configuration
// RX Pin: 2 (Connect to TX of host device)
// TX Pin: 3 (Connect to RX of host device)
SoftwareSerial softSerial(2, 3);

// Instantiate the SerialTCPClient
// Slot 0: Matches the slot configured on the host
SerialTCPClient client(softSerial, 0);

// Target Server (HTTPS Echo Service)
const char server[] = "httpbin.org";
const int port = 443; // HTTPS Port

void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect
  }

  // Initialize Link Serial (to Host)
  // IMPORTANT: Set your Host to 9600 baud as well!
  softSerial.begin(9600);

  // Connect to Server
  Serial.print(F("Connecting to "));
  Serial.print(server);
  Serial.print(F(":"));
  Serial.println(port);

  // connect(host, port, use_ssl) -> true enables SSL/TLS
  if (client.connect(server, port, true))
  {
    Serial.println(F("Connected securely!"));

    // Send HTTPS GET Request
    client.println(F("GET /get HTTP/1.1"));
    client.println(F("Host: httpbin.org"));
    client.println(F("Connection: close"));
    client.println(); // Empty line to end headers

    Serial.println(F("Request sent. Waiting for response..."));

    // Read Response (Inside Setup)
    // Loop until connection closes or data is processed
    while (client.available())
    {
      if (client.available())
      {
        char c = client.read();
        Serial.write(c);
      }
    }

    Serial.println();
    Serial.println(F("Server disconnected."));
    client.stop();
  }
  else
  {
    Serial.println(F("Connection failed!"));
  }

  Serial.println(F("Test Complete."));
}

void loop()
{
}