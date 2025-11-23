/**
 * ===============================================
 * Global_Commands (Host)
 * ===============================================
 * Runs on: The host device (e.g., ESP8266, ESP32, Raspberry Pi Pico W).
 * Purpose: Demonstrates how to register all the
 * global callbacks to allow a client to remotely
 * configure and reboot the host.
 */

#include <WiFi.h>

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialNetworkHost
#include <SerialNetworkBridge.h>
#include <WiFiClientSecure.h>

// Network Config
String current_ssid = "DEFAULT_WiFI_SSID";
String current_pass = "DEFAULT_WiFI_PASSWORD";

// Serial TCP Host Config
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the client Serial
SerialNetworkHost host(Serial2);

WiFiClientSecure client; // Netwotk client or SSL client

bool handle_set_wifi(const char *new_ssid, const char *new_pass)
{
    Serial.println("[Host] Received setWiFi command!");
    Serial.print("  New SSID: ");
    Serial.println(new_ssid);
    Serial.print("  New Pass: [REDACTED]");

    // Here, you would save this to NVS/EEPROM
    current_ssid = new_ssid;
    current_pass = new_pass;

    return true; // Return true to send ACK
}

bool handle_connect_network()
{
    Serial.println("[Host] Received connectNetwork command!");
    Serial.print("Connecting to: ");
    Serial.println(current_ssid);

    WiFi.begin(current_ssid.c_str(), current_pass.c_str());

    int retries = 20; // Try for 10 seconds
    while (WiFi.status() != WL_CONNECTED && retries > 0)
    {
        delay(500);
        Serial.print(".");
        retries--;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true; // Success, send ACK
    }
    else
    {
        Serial.println("\nFailed to connect to WiFi.");
        return false; // Failure, send NAK
    }
}

void handle_reboot()
{
    Serial.println("[Host] Received reboot command! Rebooting in 3 seconds...");
    delay(3000);
#if defined(ESP32) || defined(ESP8266)
    ESP.restart();
#elif defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_NANO_RP2040_CONNECT)
    rp2040.restart();
#endif
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial2.begin(SERIAL_BAUD);
    host.setLocalDebugLevel(1);

    // Register our global callbacks
    host.setSetWiFiCallback(handle_set_wifi);
    host.setConnectNetworkCallback(handle_connect_network);
    host.setRebootCallback(handle_reboot);

    Serial.print("Connecting to initial WiFi...");
    WiFi.begin(current_ssid.c_str(), current_pass.c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    host.setTCPClient(&client, 0 /* slot */); // Coresponding to slot 0 on client device

    // Notify the client that host is rebooted
    // Now the server connection was closed
    host.notifyBoot();

    Serial.println("Host is ready. Listening for client commands...");
}

void loop()
{
    // Reqouirements for Host operation
    host.loop();
}