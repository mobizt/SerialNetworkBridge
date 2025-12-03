/**
 * ===============================================
 * MQTT Example (for client to work with PC Host)
 * ===============================================
 * Runs on: Any Arduino device (as a client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Connects to broker.hivemq.com on Port 8883 (SSL) via PC Bridge.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch.
 * 3. Close Serial Monitor.
 * 4. Run 'python serial_bridge.py'.
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
#include "SerialSSLClient.h"
#include "debug.h"

// This ibrary is required for this example
#include <ArduinoMqttClient.h>

#define BridgeSlot_0 0
#define BridgeRate 115200 // Change this to match with the host serial baud rate.

SerialSSLClient client(Serial, BridgeSlot_0);
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

// MQTT broker config
const char broker[] = "broker.hivemq.com";
const int port = 8883; // SSL Port
const char topic[] = "arduino/serialtcp-test";

int count = 0;
const long interval = 3000;

void connectMqt()
{
  debug::print(STREAM, "Attempting to connect to the MQTT broker over ssl: ");
  debug::print(STREAM, broker, false, false);
  debug::printNewLine(STREAM);

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
  while (!Serial)
    ;

  // Flush bootloader noise
  Serial.write(0x00);
  delay(500);

  // Sending Ping request to host;
  debug::initBlink();
  while (!hostReady)
  {
    hostReady = client.pingHost(500);
    if (!hostReady)
    {
      Serial.println("No response from host. Please make sure serial_bridge.py is running...");
      debug::blink(10, 500);
      delay(2000);
    }
  }

  client.stop();

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

  if (ms == 0 || millis() - ms > interval)
  {
    ms = millis();
    mqtt.beginMessage(topic);
    mqtt.print(count);
    mqtt.endMessage();
    count++;
  }
}
