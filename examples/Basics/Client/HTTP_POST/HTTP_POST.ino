/**
 * ===============================================
 * HTTP POST
 * ===============================================
 * Runs on: The any Arduino device.
 * Host: Requires the "Basics/Host" sketch running on the host.
 * Purpose: Demonstrates a HTTP POST request.
 */
#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient
#include <SerialNetworkBridge.h>

// Seial TCP Client Config
const int CLIENT_SLOT = 0;       // Coresponding to Network client or SSL client slot 0 on the host
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the host Serial

SerialTCPClient client(Serial2, CLIENT_SLOT);

const char *server = "reqres.in";
const int port = 443;

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
        while (1)
            delay(100);
    }
    Serial.println("success!");

    // Connect to the server
    Serial.print("Connecting to ");
    Serial.print(server);
    Serial.println("...");

    if (!client.connect(server, port))
    {
        Serial.println("Connection failed.");
        return;
    }

    Serial.println("Connected!");
    Serial.println("Sending simple HTTP POST request...");

    const char *request_part1 =
        "POST /api/users HTTP/1.1\r\n"
        "Host: reqres.in\r\n"
        "x-api-key: reqres-free-v1\r\n"
        "Content-Type: application/json\r\n";

    const char *request_part2 =
        "Content-Length: 34\r\n"
        "Connection: close\r\n"
        "\r\n"
        "{\"name\":\"morpheus\",\"job\":\"leader\"}";

    // Write the request to the TX buffer
    client.write((const uint8_t *)request_part1, strlen(request_part1));
    client.write((const uint8_t *)request_part2, strlen(request_part2));

    Serial.println("Waiting for response...");

    // This is the "perfected" blocking read loop
    while (client.available() == 0)
    {
        if (!client.connected())
        {
            Serial.println("Disconnected while waiting.");
            break;
        }
        delay(0); // This calls the "smart" available()
    }

    Serial.println("Response Received");
    while (client.available() > 0)
    {
        int c = client.read();
        if (c >= 0)
        { // Check for -1
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