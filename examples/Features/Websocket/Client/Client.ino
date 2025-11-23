#define ENABLE_SERIALTCP_DEBUG
#include <SerialWebsocketClient.h>
#include <SerialHostManager.h>

SerialWebsocketClient ws(Serial2, 0); // Slot 0
SerialHostManager manager(Serial2);

void onWsEvent(WSMessageType type, const uint8_t* payload, size_t len) {
    switch(type) {
        case WS_EVENT_CONNECTED:
            Serial.println("WS Connected!");
            ws.sendText("Hello from Arduino!");
            break;
        case WS_EVENT_DISCONNECTED:
            Serial.println("WS Disconnected!");
            break;
        case WS_FRAME_TEXT:
            Serial.print("Text Msg: ");
            Serial.write(payload, len);
            Serial.println();
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    
    delay(2000);

    ws.onEvent(onWsEvent);
    
    Serial.println("Connecting to echo server...");
    // Connect to standard echo server
    ws.connect("echo.websocket.org", 443, "/", true); 
}

void loop() {
    ws.loop(); // Essential for event processing
}