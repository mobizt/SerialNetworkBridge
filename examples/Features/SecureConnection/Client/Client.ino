/**
 * ===============================================
 * Secure_Connection (Client)
 * ===============================================
 * Runs on: The any Arduino device.
 * Host: Requires the corresponding "Host.ino" for this feature.
 * Purpose: Demonstrates how to use setCACert() to
 * tell the host to load a specific certificate file
 * for a secure HTTPS connection.
 */

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient
#include <SerialTCPClient.h>

// Serial TCP Client Config
const int CLIENT_SLOT = 1; // Coresponding to slot 1 on client device which supports SSL/TLS
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the host Serial

SerialTCPClient client(Serial2, CLIENT_SLOT);

const char *server = "www.google.com";
const int port = 443;
// This file path is ON THE HOST'S filesystem
const char *ca_file = "/certs/google_ca.pem";

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial2.begin(SERIAL_BAUD);
    client.setLocalDebugLevel(1);

    // To set up new WiFi AP, connecting to new WiFi AP,
    // please see examples/Features/HostManagement/Client/Client.ino

    Serial.println("Pinging host...");
    if (!client.pingHost())
    {
        Serial.println("Host ping failed! Halting.");
        while (1)
            delay(100);
    }
    Serial.println("Host ping success!");

    // 1. Set the CA Cert on the host
    Serial.println("Telling host to use CA file: " + String(ca_file));
    if (!client.setCACert(ca_file))
    {
        Serial.println("Failed to send setCACert command. Halting.");
        while (1)
            delay(100);
    }
    Serial.println("Host acknowledged setCACert command.");

    // 2. Now, connect (as SSL)
    Serial.print("Connecting to ");
    Serial.print(server);
    Serial.println(" (SSL)...");

    if (!client.connect(server, port, true))
    {
        Serial.println("Connection failed.");
        return;
    }

    Serial.println("Connected!");
    Serial.println("Sending simple GET request...");

    client.print("GET / HTTP/1.1\r\n");
    client.print("Host: ");
    client.print(server);
    client.print("\r\n");
    client.print("Connection: close\r\n\r\n");

    Serial.println("Waiting for response...");
    while (client.available() == 0)
    {
        if (!client.connected())
            break;
        delay(0);
    }

    Serial.println("Response Received");
    while (client.available() > 0)
    {
        int c = client.read();
        if (c >= 0)
        {
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