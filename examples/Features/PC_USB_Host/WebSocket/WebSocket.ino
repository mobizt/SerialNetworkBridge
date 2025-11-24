/**
 * ===============================================
 * PC USB Host WebSocket Client Example
 * ===============================================
 * Runs on: Any Arduino device (as Client).
 * Host: Requires the 'serial_bridge.py' script running on your PC.
 * Purpose: Demonstrates how to configure your device to connect to
 * the 'serial_bridge.py' script running on your computer via the USB cable.
 * It connects to a WebSocket echo server and blinks the LED on events.
 * * * INSTRUCTIONS:
 * 1. CRITICAL: Ensure '#define ENABLE_SERIALTCP_DEBUG' is commented out/disabled.
 * 2. Upload this sketch to your Arduino.
 * 3. Close the Serial Monitor (The USB port is used for the bridge).
 * 4. Run 'python serial_bridge.py' on your computer.
 * 5. Watch the Built-in LED:
 * - Fast Flashing (20x): Connected to WebSocket Server.
 * - Short Blip (2x):     Message (Echo) received from Server.
 * - Slow Blink (5x):     Error.
 */

// CRITICAL: Debugging MUST be disabled
// #define ENABLE_SERIALTCP_DEBUG

#include <Arduino.h>

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

#include <SerialNetworkBridge.h>

// Use the main USB Serial port (Slot 0)
SerialWebsocketClient ws(Serial, 0);

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

// WebSocket Event Callback
void onWsEvent(WSMessageType type, const uint8_t *payload, size_t len)
{
    switch (type)
    {
    case WS_EVENT_CONNECTED:
        // Step 1: Connection Established (Flash LED Fast)
        flashLED(20, 50);

        // Send a hello message to the server
        ws.sendText("Hello from Arduino PC Host!");
        break;

    case WS_EVENT_DISCONNECTED:
        // Turn LED OFF (High is OFF for active-low LEDs)
        digitalWrite(LED_BUILTIN, HIGH);
        break;

    case WS_FRAME_TEXT:
    case WS_FRAME_BINARY:
        // Step 2: Message Received (Short Double Blip)
        flashLED(2, 50);
        break;

    case WS_EVENT_ERROR:
        // Error (Slow Blink)
        flashLED(5, 500);
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // [CRITICAL FIX] Flush bootloader noise so the PC Python script
    // doesn't get confused by random bytes on startup.
    Serial.write(0x00);
    delay(500);

#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    // Start OFF (Assuming Active LOW for most builtin LEDs)
    digitalWrite(LED_BUILTIN, HIGH);
#endif

    // Register the event callback
    ws.onEvent(onWsEvent);

    // Connect to the public echo server (SSL Enabled)
    // Host: echo.websocket.org
    // Port: 443
    // Path: /
    ws.connect("echo.websocket.org", 443, "/", true);
}

void loop()
{
    // WebSocket is event-driven, so we simply call loop() repeatedly
    ws.loop();
}