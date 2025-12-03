/**
 * ===============================================
 * Device Host Serial Bridge for AsyncTCP
 * ===============================================
 * Runs on: ESP32 using AsyncTCP client.
 * Purpose: Provide the example to use ESP32's AsyncTCP as a client in ESP32 host device.
 * Requirement: Define USE_ASYNC_CLIENT macro to use. 
 * 
 * For more Host features, please see examples/Device_Host/Generic_Client/Host
 *
 * Macros explanation
 *
 * ENABLE_LOCAL_DEBUG   The macro to allow local debug to show on Serial port.
 *
 * WIFI_SSID            The default WiFi AP SSID
 *
 * WIFI_PASSWORD        The default WiFi AP Password
 *
 * BridgeRate           The bridge serial port baud rate.
 *                      Should be matched with client device serial baud rate.
 *                      Should not exceed 115200 for AVR
 *
 * BridgeSerial         The serial port that is connected to the client device serial port.
 * 
 */

#define ENABLE_LOCAL_DEBUG
#define USE_ASYNC_CLIENT

#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "password"
#define BridgeRate 115200
#define BridgeSerial Serial2

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <SerialNetworkBridge.h>

SerialNetworkHost host(BridgeSerial);
AsyncClient asyncTcpClient; // The AsyncTCP object

void setup()
{
    Serial.begin(115200);
    delay(2000);

    BridgeSerial.begin(BridgeRate);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
        delay(500);

    // Assign the AsyncClient to Slot 0
    host.setAsyncClient(&asyncTcpClient, 0);

    // Connect to WiFi

    Serial.println("[Host] Connecting to WiFi AP: ");
    Serial.println(WIFI_SSID);
    Serial.println("...");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
        delay(500);

    Serial.println("[Host] WiFi AP connected!");

    // Notify the client that host is rebooted/ready.
    // This resets the client's internal state for a fresh session.
    host.notifyBoot();

    Serial.println("Device Host Serial Bridge started.");
}

void loop()
{
    // Reqouirements for Host operation
    host.loop();
}