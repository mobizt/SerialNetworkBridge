/**
 * ===============================================
 * STARTTLS (Host)
 * ===============================================
 * Runs on: The host device (e.g., ESP8266, ESP32, Raspberry Pi Pico W).
 * Purpose: Demonstrates how to register the
 * StartTLSCallback to allow a client to upgrade
 * a plain connection to a secure one.
 */

#include <WiFi.h>
#include <WiFiClient.h>
// https://github.com/mobizt/ESP_SSLClient
#include <ESP_SSLClient.h>

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialNetworkHost
#include <SerialNetworkHost.h>

// Network Config
const char *ssid = "DEFAULT_WIFI_SSID";
const char *password = "DEFAULT_WIFI_PASSWORD";

// Bridge Config
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the client Serial
SerialNetworkHost host(Serial2);

// We will create two clients:
WiFiClient basic_client;
ESP_SSLClient ssl_client; // (STARTTLS capable SSL client)

bool handle_start_tls(int slot)
{
    Serial.println("[Host] STARTTLS Callback Triggered for slot " + String(slot));
    Serial.println("  > Calling connectSSL()...");
    if (ssl_client.connectSSL())
    {
        Serial.println("  > SUCCESS: connectSSL() completed.");
        return true;
    }
    else
    {
        Serial.println("  > ERROR: connectSSL() failed.");
        return false;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial2.begin(SERIAL_BAUD);
    host.setLocalDebugLevel(1);

    // Connect to WiFi
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    ssl_client.setClient(&basic_client, false /* starts in plain mode */);

    ssl_client.setBufferSizes(2048 /* rx buffer */, 1024 /* tx buffer */);

    host.setTCPClient(&ssl_client, 0 /* slot */); // Coresponding to slot 0 on host device

    // Register the STARTTLS Callback
    // We only register it for the secure-capable slot
    host.setStartTLSCallback(0, handle_start_tls);

    /// Notify the client that host is rebooted
    // Now the server connection was closed
    host.notifyBoot();

    Serial.println("Host is ready.");
    Serial.println("  > Slot 0: Plain/STARTTLS Capable");
}

void loop()
{
    // Reqouirements for Host operation
    host.loop();
}