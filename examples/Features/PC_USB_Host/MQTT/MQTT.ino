/**
 * ===============================================
 * PC USB Host MQTT Client Example (HiveMQ SSL)
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Connects to broker.hivemq.com on Port 8883 (SSL) via PC Bridge.
 * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch.
 * 3. Close Serial Monitor.
 * 4. Run 'python serial_bridge.py'.
 * 5. Watch LED:
 * - Fast Flash (20x): Connected to Broker.
 * - Short Blip (2x):  Message Received.
 * - Slow Blink (5x):  Error.
 */

// [CRITICAL] Debugging MUST be disabled
// #define ENABLE_SERIALTCP_DEBUG

#include <Arduino.h>
#include <SerialNetworkBridge.h>
#include <ArduinoMqttClient.h>

// Define LED_BUILTIN if not defined by the board package
#ifndef LED_BUILTIN
#if defined(ESP8266)
// Most ESP8266 modules (NodeMCU, Wemos) use GPIO 2
#define LED_BUILTIN 2
#elif defined(ESP32)
// Many ESP32 dev boards use GPIO 2
#define LED_BUILTIN 2
#elif defined(STM32F1xx)
// "Blue Pill" STM32 usually uses PC13
#define LED_BUILTIN PC13
#else
// Default for Arduino Uno, Mega, Nano, Leonardo, etc.
#define LED_BUILTIN 13
#endif
#endif

// HELPER CLASS TO FORCE SSL
// The ArduinoMqttClient calls connect(host, port) without the 3rd 'ssl' arg.
// This class overrides that behavior to ALWAYS send 'true' (SSL Enabled) to the bridge.
class SerialSSLClient : public SerialTCPClient
{
public:
    using SerialTCPClient::SerialTCPClient; // Inherit constructors

    // Override the standard connect method
    int connect(const char *host, uint16_t port) override
    {
        // Force use_ssl = true
        return SerialTCPClient::connect(host, port, true);
    }

    // Pull in IPAddress overload to avoid hiding it
    using SerialTCPClient::connect;
};

// Use our custom SSL Client on the main Serial port (Slot 0)
SerialSSLClient client(Serial, 0);
MqttClient mqttClient(client);

// MQTT Config for HiveMQ (Public)
const char broker[] = "broker.hivemq.com";
const int port = 8883; // SSL Port
const char topic[] = "arduino/serialtcp-test";

unsigned long lastMillis = 0;
int count = 0;
const long interval = 3000;

void flashLED(int times, int delayMs)
{
#if defined(LED_BUILTIN)
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayMs);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayMs);
    }
#endif
}

void connectMqt()
{
    // 1. Ping the PC Host
    if (!client.pingHost(500))
    {
        flashLED(5, 500); // Ping Error
        return;
    }

    // 2. Connect to MQTT Broker
    // This uses SerialSSLClient, so the Python script will receive the SSL flag.
    if (mqttClient.connect(broker, port))
    {
        flashLED(20, 50); // Success!
        mqttClient.subscribe(topic);
    }
    else
    {
        flashLED(5, 500); // Connect Error
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // [CRITICAL] Flush bootloader noise
    Serial.write(0x00);
    delay(500);

#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Start OFF
#endif

    connectMqt();
}

void loop()
{
    if (!mqttClient.connected())
    {
        connectMqt();
        delay(2000);
        return;
    }

    mqttClient.poll();

    int messageSize = mqttClient.parseMessage();
    if (messageSize)
    {
        flashLED(2, 50); // Message Received
        while (mqttClient.available())
        {
            mqttClient.read(); // Consume data
        }
    }

    if (millis() - lastMillis > interval)
    {
        lastMillis = millis();
        mqttClient.beginMessage(topic);
        mqttClient.print(count);
        mqttClient.endMessage();
        count++;
    }
}