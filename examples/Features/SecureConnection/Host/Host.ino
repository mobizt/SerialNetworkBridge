/**
 * ===============================================
 * Secure_Connection (Host)
 * ===============================================
 * Runs on: The host device (e.g., ESP8266, ESP32, Raspberry Pi Pico W).
 * Purpose: Demonstrates how to use a WiFiClientSecure object
 * and register the new PreConnectCallback to load
 * CA certificates dynamically from a filesystem.
 */

// Use WiFiClientSecure for SSL/TLS connections
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialNetworkHost
#include <SerialNetworkBridge.h>

// Filesystem (e.g., LittleFS) for storing certs
#include "FS.h"
#include "LittleFS.h"

// Network Config
const char *ssid = "DEFAULT_WIFI_SSID";
const char *password = "DEFAULT_WIFI_PASSWORD";

// Serial TCP Client Config
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the client Serial
SerialNetworkHost host(Serial2);

WiFiClient basic_client;
WiFiClientSecure ssl_client;

void handle_pre_connect(int slot, const char *ca_cert_filename)
{
    Serial.println("[Host] Pre-Connect Callback for slot " + String(slot));

    if (slot == 1 /* slot for Secure SSL/TLS */)
    {
        Serial.println("  > Client wants to use CA file: " + String(ca_cert_filename));

        Serial.println("  > Slot is a Secure Client. Attempting to load cert...");

        if (LittleFS.exists(ca_cert_filename))
        {
            File certFile = LittleFS.open(ca_cert_filename, "r");
            if (certFile)
            {
                // Use setCACertStream for efficiency
                if (ssl_client.loadCACert(certFile, certFile.size()))
                {
                    Serial.println("  > SUCCESS: Certificate loaded from file.");
                }
                else
                {
                    Serial.println("  > ERROR: loadCACert() failed.");
                }
                certFile.close();
            }
        }
        else
        {
            Serial.println("  > ERROR: File not found on host: " + String(ca_cert_filename));
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("SerialNetworkHost Secure Connection Example...");

    // Initialize Filesystem
    if (!LittleFS.begin(true))
    { // true = format if mount failed
        Serial.println("LittleFS Mount Failed. Halting.");
        while (1)
            delay(100);
    }
    Serial.println("LittleFS Mounted.");

    // Example: Write a dummy cert file
    if (!LittleFS.exists("/certs/google_ca.pem"))
    {
        Serial.println("Writing dummy google_ca.pem file...");
        LittleFS.mkdir("/certs");
        File f = LittleFS.open("/certs/google_ca.pem", "w");
        if (f)
        {
            f.println("-----BEGIN CERTIFICATE-----");
            f.println("MIIEvjCCA6agAwIBAgIQIHYEnaQ/sNud1s1/pDltgzANBgkqhkiG9w0BAQsF");
            f.println("ADBMMSIwIAYDVQQLExlHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMTETMBEGA1UE");
            f.println("CgwKR2xvYmFsU2lnbjETMBEGA1UEAwwKR2xvYmFsU2lnbjAeFw0xNzA2MTUw");
            // (This is just a truncated example cert)
            f.println("qDTMB7cE6ISv6ii18W8f5n2s5s1bWGeYk0S8Ds/geqi6fLh4d3aDtmA3OqRj");
            f.println("9N2vSSyM5j/u/f0DofhS2w==");
            f.println("-----END CERTIFICATE-----");
            f.close();
            Serial.println("Dummy file written.");
        }
        else
        {
            Serial.println("Failed to create dummy file.");
        }
    }

    Serial2.begin(SERIAL_BAUD);
    host.setLocalDebugLevel(1);

    // Register the new Pre-Connect Callback
    host.setPreConnectCallback(handle_pre_connect);

    // Connect to WiFi
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    // Set up slots
    host.setTCPClient(&basic_client, 0 /* slot */); // Coresponding to slot 0 on client device
    host.setTCPClient(&ssl_client, 1 /* slot */);   // Coresponding to slot 1 on client device

    // Notify the client that host is rebooted
    // Now the server connection was closed
    host.notifyBoot();

    Serial.println("Host is ready.");
    Serial.println("  > Slot 0: Plain TCP");
    Serial.println("  > Slot 1: Secure SSL/TLS");
}

void loop()
{
    // Reqouirements for Host operation
    host.loop();
}