/**
 * ==================================================
 * UDP NTP Exmample (for client to work with PC Host)
 * ==================================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to send and receive UDP packets via the PC USB bridge.
 * It sends an NTP time request to 'pool.ntp.org' and flashes the LED on response.
 * * * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor.
 * 4. Run 'python serial_bridge.py' on your computer.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSlot_0       The slot (session) 0 which data will be transferred
 *                    This corresponds to the ssl wrapped socket assigned on slot 0 of the python script running on the PC device.
 *                    The implementation on python script is different and flexible. The transport layer does not fix to the slot
 *                    as it is assigned to the host device.
 *
 * ENABLE_LOCAL_DEBUG The macro to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *                    It shiukd not defined in this example.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be defined in this example.
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * CLIENT             The client object e.g. SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define HOST_RELAY_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

#define BridgeSlot_0 0
#define BridgeRate 115200

SerialUDPClient udp(Serial, BridgeSlot_0);

#if defined(HOST_RELAY_DEBUG)
#define STREAM udp
#define CLIENT udp
#else
#define STREAM Serial
#define CLIENT udp
#endif

unsigned long ms = 0;
bool hostReady = false;

// NTP Server Settings
const char *ntpServer = "pool.ntp.org";
const int ntpPort = 123;
const int localPort = 2390; // Local port to listen on

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

const unsigned long interval = 20000; // Send request every 20 seconds

// Helper to send an NTP request
void sendNTPpacket()
{
    debug::printNewLine(STREAM);
    debug::print(STREAM, "Begin the UDP packet...\r\n");

    // Begin the packet transmission
    // Note: For PC Host, we don't explicitly need to check connection before every UDP send
    // but beginPacket might fail if the bridge is down.
    if (!udp.beginPacket(ntpServer, ntpPort))
    {
        debug::print(STREAM, "Error: Failed to begin packet!\r\n");
        return;
    }

    debug::print(STREAM, "Preparing NTP request data...\r\n");
    // Prepare Data
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    debug::print(STREAM, "Writing the packet...\r\n");
    // Write & Send
    udp.write(packetBuffer, NTP_PACKET_SIZE);

    // Finalize and send the packet over the serial bridge
    debug::print(STREAM, udp.endPacket() ? "Success: NTP packet sent.\r\n" : "Error: Failed to finalize/send packet!\r\n");
}

void setup()
{
    Serial.begin(BridgeRate);
    while (!Serial)
        ;

    Serial.write(0x00);
    delay(500);

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = udp.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Please make sure serial_bridge.py is running...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

    udp.stop();

    // Skip SSL certificate verification
    udp.setInsecure();
    // Set SSL Client RX/TX buffer sizest
    udp.setBufferSizes(2048, 1024);

    debug::print(STREAM, "Starting UDP listener on port ");
    debug::printRaw(STREAM, String(localPort).c_str());
    debug::printRaw(STREAM, "...\r\n");

    // Start UDP Listener on the PC
    if (udp.begin(localPort))
    {
        debug::print(STREAM, "UDP listener is started on port ");
        debug::printRaw(STREAM, String(localPort).c_str());
        debug::printRaw(STREAM, "\r\n");

        // Send first packet immediately
        sendNTPpacket();
    }
    else
    {
        // Error: Could not bind port (Slow Blink)
        debug::print(STREAM, "Error: Failed to start UDP listener.\r\n");
    }
}

void loop()
{
    // Check for Incoming UDP Packets
    int packetSize = udp.parsePacket();

    if (packetSize)
    {
        debug::print(STREAM, "Received packet of size ");
        debug::printRaw(STREAM, String(packetSize).c_str());
        debug::printRaw(STREAM, " from ");
        debug::printRaw(STREAM, String(udp.remoteIP()).c_str());
        debug::printRaw(STREAM, ":");
        debug::printRaw(STREAM, String(udp.remotePort()).c_str());
        debug::printRaw(STREAM, "\r\n");

        debug::print(STREAM, "Reading the packet...\r\n");

        // Read the packet
        udp.read(packetBuffer, NTP_PACKET_SIZE);

        // (Time conversion logic remains the same)
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;

        const unsigned long seventyYears = 2208988800UL;
        unsigned long epoch = secsSince1900 - seventyYears;

        debug::print(STREAM, "Current Epoch Time: ");
        debug::printRaw(STREAM, String(epoch).c_str());
        debug::printRaw(STREAM, "\r\n");
    }

    // Send Periodic Requests
    if (ms == 0 || millis() - ms > interval)
    {
        ms = millis();
        sendNTPpacket();
    }

    // For UDP, we don't have a 'connected()' state to check,
    // so we just loop.
}