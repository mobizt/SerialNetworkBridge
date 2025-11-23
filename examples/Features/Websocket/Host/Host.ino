#include <WiFi.h>
// https://github.com/Links2004/arduinoWebSockets
#include <WebSocketsClient.h>

#define ENABLE_SERIALTCP_DEBUG
#include <SerialNetworkBridge.h>

const char *ssid = "WIFI_SSID";
const char *password = "WIFI_PASSWORD";

SerialNetworkHost host(Serial2);
WebSocketsClient ws; // Native WebSocket Client

// Callback to handle incoming WS events from the Internet
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
    WSMessageType serialType = WS_EVENT_ERROR;

    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.println("[WS] Disconnected!");
        serialType = WS_EVENT_DISCONNECTED;
        break;
    case WStype_CONNECTED:
        Serial.printf("[WS] Connected to url: %s\n", payload);
        serialType = WS_EVENT_CONNECTED;
        break;
    case WStype_TEXT:
        serialType = WS_FRAME_TEXT;
        break;
    case WStype_BIN:
        serialType = WS_FRAME_BINARY;
        break;
    default:
        return;
    }

    // Push the event to the Serial Client (Slot 0)
    host.pushWsEvent(0, serialType, payload, length);
}

// Callback to handle Serial Commands for WebSocket
bool onWsCommand(int slot, uint8_t cmd, const uint8_t *payload, size_t len)
{
    if (slot != 0)
        return false; // We only put WS on slot 0 for this demo

    switch (cmd)
    {
    case CMD_C_WS_CONNECT:
    {
        // Payload: [ssl(1)] [port(2)] [host_len(1)] [host] [path_len(1)] [path]
        bool ssl = payload[0];
        uint16_t port = (payload[1] << 8) | payload[2];
        uint8_t hl = payload[3];
        char h[hl + 1];
        memcpy(h, &payload[4], hl);
        h[hl] = 0;
        uint8_t pl = payload[4 + hl];
        char p[pl + 1];
        memcpy(p, &payload[5 + hl], pl);
        p[pl] = 0;

        if (ssl)
            ws.beginSSL(h, port, p, NULL, ""); // "wss" protocol sometimes helps
        else
            ws.begin(h, port, p, "");

        ws.onEvent(webSocketEvent);
        ws.setReconnectInterval(5000);
        return true;
    }
    case CMD_C_WS_SEND_FRAME:
    {
        // Payload: [type(1)] [data]
        WSMessageType type = (WSMessageType)payload[0];
        const uint8_t *data = &payload[1];
        size_t dlen = len - 1;

        if (type == WS_FRAME_TEXT)
            ws.sendTXT((uint8_t *)data, dlen);
        else
            ws.sendBIN((const uint8_t *)data, dlen);
        return true;
    }
    case CMD_C_WS_DISCONNECT:
        ws.disconnect();
        return true;
    case CMD_C_WS_LOOP:
        ws.loop();
        return true;
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        delay(500);
    Serial.println("WiFi Connected");

    host.setWebSocketClient(&ws, 0); // Slot 0

    // Register the callback that handles translation
    host.setWebSocketCallback(onWsCommand);

    host.notifyBoot();
}

void loop()
{
    host.loop();
    ws.loop();
}