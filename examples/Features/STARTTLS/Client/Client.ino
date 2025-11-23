/**
 * ===============================================
 * STARTTLS (Client)
 * ===============================================
 * Runs on: The any Arduino device.
 * Host: Requires the corresponding "Host.ino" for this feature.
 * Purpose: Demonstrates how to upgrade a plain
 * connection to a secure one using startTLS().
 * This is common for protocols like SMTP, IMAP, and FTP.
 */

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient
#include <SerialNetworkBridge.h>

// Serial TCP Client Config
const int CLIENT_SLOT = 0; // Coresponding to slot 0 on host device which supports STARTTLS
const long SERIAL_BAUD = 115200;

SerialTCPClient client(Serial2, CLIENT_SLOT);

// We'll use a public SMTP server for this test
const char *server = "smtp.gmail.com";
const int port = 587; // Standard STARTTLS port

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
  delay(1000);

  Serial2.begin(SERIAL_BAUD);
  client.setLocalDebugLevel(1);

  // To set up new WiFi AP, connecting to new WiFi AP,
  // please see examples/Features/HostManagement/Client/Client.ino

  Serial.print("Pinging host...");
  if (!client.pingHost())
  {
    Serial.println("failed");
    while (1)
      delay(100);
  }
  Serial.println("success!");

  // 1. Connect in PLAIN mode
  Serial.print("Connecting to ");
  Serial.print(server);
  Serial.println(" (Plain)...");

  // Note: 'use_ssl' is false
  if (!client.connect(server, port, false))
  {
    Serial.println("Connection failed.");
    return;
  }

  Serial.println("Connected (Plain). Reading greeting...");
  Serial.println(readLine()); // Read "220 smtp.gmail.com ..."

  // 2. Send EHLO
  Serial.println("Sending EHLO...");
  client.println("EHLO my-arduino-client.com");
  client.flush();

  // Read multi-line response
  bool found_starttls = false;
  while (true)
  {
    String line = readLine();
    Serial.println(line);
    if (line.startsWith("250-STARTTLS"))
    {
      found_starttls = true;
    }
    if (line.startsWith("250 "))
    { // Last line of EHLO
      break;
    }
  }

  if (!found_starttls)
  {
    Serial.println("Server does not support STARTTLS. Halting.");
    client.stop();
    return;
  }

  // 3. Send STARTTLS command
  Serial.println("Server supports STARTTLS. Sending command...");
  client.println("STARTTLS");
  client.flush();
  Serial.println(readLine()); // Read "220 2.0.0 Ready to start TLS"

  // 4. Upgrade our *proxy* connection
  Serial.println("Telling host to upgrade connection to SSL...");
  if (!client.startTLS())
  {
    Serial.println("Host failed to upgrade to TLS! Halting.");
    client.stop();
    return;
  }
  Serial.println("Host upgraded to TLS successfully!");

  // 5. Send EHLO *again* (over new secure channel)
  Serial.println("Sending EHLO again (over SSL)...");
  client.println("EHLO my-arduino-client.com");
  client.flush();

  // Read the new, secure EHLO response
  while (true)
  {
    String line = readLine();
    Serial.println(line);
    if (line.startsWith("250 "))
    { // Last line of EHLO
      break;
    }
  }

  Serial.println("Successfully negotiated STARTTLS!");
  client.stop();
  Serial.println("Connection stopped.");
}

void loop()
{
}