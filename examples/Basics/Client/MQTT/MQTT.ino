/**
 * ===============================================
 * MQTT
 * ===============================================
 * Runs on: The any Arduino device.
 * Host: Requires the "Basics/Host" sketch running on the host.
 * Purpose: Demonstrates how to use with Arduino Mqtt Client library.
 */

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialTCPClient
#include <SerialTCPClient.h>
#include <ArduinoMqttClient.h> // Include the standard MQTT library

// Serial TCP Client Config
const int CLIENT_SLOT = 0;       // Coresponding to Network client or SSL client slot 0 on the host
const long SERIAL_BAUD = 115200; // Coresponding to the baud rate used in the host Serial

// MQTT Config
const char broker[] = "test.mosquitto.org";
const int port = 1883; // Use 1883 for non-SSL and make sure the host slot 0 client can handle it
const char topic[] = "arduino/serialtcp-test";

// Create the clients
SerialTCPClient client(Serial2, CLIENT_SLOT);
MqttClient mqttClient(client);

unsigned long lastMillis = 0;
int count = 0;
const long interval = 3000;

void connectMqt()
{
    Serial.print("Attempting to connect to the MQTT broker over ssl: ");
    Serial.println(broker);

    if (!mqttClient.connect(broker, port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        return;
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

    Serial.print("Subscribing to topic: ");
    Serial.println(topic);
    Serial.println();

    // subscribe to a topic
    mqttClient.subscribe(topic);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(topic);

    Serial.print("Waiting for messages on topic: ");
    Serial.println(topic);
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial2.begin(SERIAL_BAUD);
    client.setLocalDebugLevel(1);

    // To set up new WiFi AP, connecting to new WiFi AP,
    // please see examples/Features/HostManagement/Client/Client.ino

    connectMqt();
}

void loop()
{

    if (!mqttClient.connected())
    {
        connectMqt();
        return;
    }

    mqttClient.poll();

    int messageSize = mqttClient.parseMessage();
    if (messageSize)
    {
        // we received a message, print out the topic and contents
        Serial.print("Received a message with topic '");
        Serial.print(mqttClient.messageTopic());
        Serial.print("', length ");
        Serial.print(messageSize);
        Serial.println(" bytes:");

        // use the Stream interface to print the contents
        while (mqttClient.available())
        {
            Serial.print((char)mqttClient.read());
        }
        Serial.println();
        Serial.println();
    }

    if (millis() - lastMillis > interval)
    {
        lastMillis = millis();

        Serial.print("Sending message to topic: ");

        Serial.println(topic);

        // send message, the Print interface can be used to set the message contents
        mqttClient.beginMessage(topic);

        mqttClient.print(count);

        mqttClient.endMessage();
        count++;
    }
}