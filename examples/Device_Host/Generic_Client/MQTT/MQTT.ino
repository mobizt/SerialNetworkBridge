/**
 * ==================================================
 * MQTT Example (for client to work with Device Host)
 * ==================================================
 * Runs on: Any Arduino device.
 * Host: Requires "Device_Host/Generic_Client/Host/Host.ino" running on
 * other Arduino devices that connected to this device via Serial port.
 * Purpose: Demonstrates how to use with Arduino Mqtt Client library.
 *
 * Macros explanation
 *
 * BridgeRate         The bridge serial port baud rate.
 *                    Should be matched with host device serial baud rate.
 *                    Should not exceed 115200 for AVR
 *
 * BridgeSerial       The serial port that is connected to host device serial port.
 *
 * BridgeSlot_0       The slot (channel or session) 0 which data will be transferred
 *                    This corresponds to the SSL client assigned on slot 0 of the host device.
 *                    See Host.ino example.
 *
 * ENABLE_LOCAL_DEBUG The default to allow local debug to show on Serial port.
 *                    This macro can be defined only when the Bridge serial port is not a USB Serial port.
 *
 * HOST_RELAY_DEBUG   The macro that should be defined when working with PC host to relay the debug info.
 *                    This macro should be commented out or undefined in this example (debug info is printed locally).
 *
 * STREAM             The sink object to print debug info which is one of the following clients.
 *                    Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 *
 * CLIENT             The client object e.g. SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
 * 
 * BLINK_LED_PIN      The GPIO that is connected to LED which is used for pinging error display.
 */

#define ENABLE_LOCAL_DEBUG
#define BLINK_LED_PIN -1

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include "debug.h"

// This ibrary is required for this example
#include <ArduinoMqttClient.h>

#define BridgeSlot_0 0
#define BridgeRate 115200    // Change this to match with the host serial baud rate.
#define BridgeSerial Serial2 // Change this to match your hardware.

SerialTCPClient client(BridgeSerial, BridgeSlot_0);
MqttClient mqtt(client);

#if defined(HOST_RELAY_DEBUG)
#define STREAM client
#define CLIENT client
#else
#define STREAM Serial
#define CLIENT client
#endif

unsigned long ms = 0;
bool hostReady = false;

const char broker[] = "broker.hivemq.com";
const int port = 8883;
const char topic[] = "arduino/serialtcp-test";

int count = 0;
const long interval = 3000;

void connectMqt()
{
    debug::print(STREAM, "Attempting to connect to the MQTT broker over ssl: ");
    debug::print(STREAM, broker, false, false);
    debug::printNewLine(STREAM);
    
    // This will set SSL I/O buffer (if SSL client on host device is supported)
    client.setBufferSizes(2048, 1024);

    // Set this to skip SSL certificate verification on SSL client assigned to slot 0 in host device.
    client.setInsecure();

    if (!mqtt.connect(broker, port))
    {
        debug::print(STREAM, "MQTT connection failed! Error code = ");
        debug::printRaw(STREAM, String(mqtt.connectError()).c_str());
        debug::printNewLine(STREAM);
        return;
    }

    debug::print(STREAM, "You're connected to the MQTT broker!\r\n");
    debug::print(STREAM, "Subscribing to topic: ");
    debug::printRaw(STREAM, topic);
    debug::printNewLine(STREAM);

    // subscribe to a topic
    mqtt.subscribe(topic);

    // topics can be unsubscribed using:
    // mqtt.unsubscribe(topic);

    debug::print(STREAM, "Waiting for messages on topic: ");
    debug::printRaw(STREAM, topic);
    debug::printNewLine(STREAM);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    BridgeSerial.begin(BridgeRate);
    client.setLocalDebugLevel(1); // Enable debug prints

    // Sending Ping request to host;
    debug::initBlink();
    while (!hostReady)
    {
        hostReady = client.pingHost(500);
        if (!hostReady)
        {
            Serial.println("No response from host. Check serial port, baud rate and host device...");
            debug::blink(10, 500);
            delay(2000);
        }
    }

    connectMqt();
}

void loop()
{
    if (!hostReady)
        return;

    if (!mqtt.connected())
    {
        connectMqt();
        delay(2000);
        return;
    }

    mqtt.poll();

    int messageSize = mqtt.parseMessage();
    if (messageSize)
    {
        debug::print(STREAM, "Message Received: ");
        while (mqtt.available())
        {
            debug::printRaw(STREAM, String(mqtt.read()).c_str());
        }

        debug::printNewLine(STREAM);
    }

    if (millis() - ms > interval)
    {
        ms = millis();
        mqtt.beginMessage(topic);
        mqtt.print(count);
        mqtt.endMessage();
        count++;
    }
}