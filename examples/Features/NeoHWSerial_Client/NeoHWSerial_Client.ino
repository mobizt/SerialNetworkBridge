/*
 * ===============================================
 * NeoHWSerial Client Example (The Mega)
 * ===============================================
 * Runs on: The any Arduino AVR device e.g. mega.
 * Host: Requires the corresponding "Host.ino" for this feature.
 * Purpose: Demonstrates how to use NeoHWSerial and InterruptStream wrapper.
 */

#include <NeoHWSerial.h>
#include "InterruptStream.h" // The wrapper class we created earlier
#include <SerialTCPClient.h>

// Mega Pin 18 (TX1) -> Host device RX (may use Voltage Divider!)
// Mega Pin 19 (RX1) <- Host device TX (may use Voltage Divider!)
#define BridgeSerial NeoSerial1

// Initialize the Wrapper
InterruptStream bridgeStream(BridgeSerial);

// nitialize the Client using the Wrapper
// We pass 'bridgeStream' instead of 'Serial1'
SerialTCPClient client(bridgeStream, 0);

// Interrupt service routime for receiving data from Host
static void handleRxChar(uint8_t c)
{
    // Push data from the MKR immediately into our buffer
    bridgeStream.push(c);
}

void setup()
{
    // Debug Serial (USB to PC)
    NeoSerial.begin(115200);
    while (!NeoSerial)
        ;

    // Bridge Serial (To Host)
    BridgeSerial.begin(115200);

    // Attach the interrupt for high-performance receiving
    BridgeSerial.attachInterrupt(handleRxChar);

    NeoSerial.println(F("Mega Client (NeoHWSerial) Started"));
    NeoSerial.println(F("Connecting to httpbin.org via Bridge..."));

    // Connect to a server
    // The request goes: Mega -> Serial -> Host -> WiFi -> httpbin.org
    if (client.connect("httpbin.org", 443))
    {
        NeoSerial.println(F("Connected!"));

        // Send HTTP Request
        client.println("GET / HTTP/1.1");
        client.println("Host: httpbin.org");
        client.println("Connection: close");
        client.println();
    }
    else
    {
        NeoSerial.println(F("Connection Failed. Check Host power/wiring."));
    }
}

void loop()
{
    // Read response
    while (client.available())
    {
        char c = client.read();
        NeoSerial.write(c);
    }

    if (!client.connected())
    {
        NeoSerial.println();
        NeoSerial.println(F("Disconnecting."));
        client.stop();
        while (true)
            ; // Stop
    }
}