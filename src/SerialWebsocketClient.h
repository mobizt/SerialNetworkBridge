/**
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_WEBSOCKET_CLIENT_H
#define SERIAL_WEBSOCKET_CLIENT_H

#include <Arduino.h>
#include <Stream.h>
#include "SerialNetworkProtocol.h"

using namespace SerialNetworkProtocol;

// Buffers
// WebSocket traffic can be bursty and carry large frames.
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
#define SERIAL_WS_TX_BUFFER_SIZE 128
#define SERIAL_WS_RX_BUFFER_SIZE 512
#else
#define SERIAL_WS_TX_BUFFER_SIZE 256
#define SERIAL_WS_RX_BUFFER_SIZE 1024
#endif

// Forward declaration for event handler
typedef void (*WebSocketEventCallback)(WSMessageType type, const uint8_t *payload, size_t length);

class SerialWebsocketClient
{
private:
    Stream *sink = nullptr;
    int slot = 0;
    volatile bool _connected_status = false;

    // Buffers
    uint8_t _rx_buffer[SERIAL_WS_RX_BUFFER_SIZE];
    volatile size_t _rx_head = 0;
    volatile size_t _rx_tail = 0;

    // Callback
    WebSocketEventCallback _event_callback = nullptr;

    // Protocol
    PacketReceiver _receiver;
    uint8_t _decoded_buffer[MAX_PACKET_BUFFER_SIZE];
    volatile bool _ack_received = false;
    volatile bool _nak_received = false;
    volatile bool _ping_response_received = false;

    // Flag Handling
    volatile bool _flag_response_received = false;
    volatile uint8_t _received_flag = 0;

    uint8_t _debug_level = 1;

    void maintenance()
    {
        if (!sink)
            return;

        // Process incoming serial data
        while (sink->available())
        {
            serial_tcp_yield();
            uint8_t b = sink->read();

            // Capture length from read_byte
            size_t cobsLen = _receiver.read_byte(b);

            if (cobsLen > 0)
            {
                // Use captured length instead of pos
                size_t len = cobs_decode(_receiver.buffer, cobsLen, _decoded_buffer);
                if (len > 2)
                {
                    uint16_t crc = calculate_crc16(_decoded_buffer, len - 2);
                    uint16_t r_crc = (uint16_t)(_decoded_buffer[len - 1] << 8 | _decoded_buffer[len - 2]);

                    if (crc == r_crc)
                    {
                        processPacket(_decoded_buffer, len);
                    }
                    else
                    {
#if defined(ENABLE_LOCAL_DEBUG)
                        DEBUG_PRINT(_debug_level, "[Client]", "CRC Error");
#endif
                    }
                }
            }
        }
        // Prompt host loop
        sendCommand(CMD_C_WS_LOOP, this->slot, nullptr, 0, false);
    }

    void processPacket(const uint8_t *pkt, size_t len)
    {
        uint8_t cmd = pkt[0];
        uint8_t slot = pkt[1];

        if (slot == GLOBAL_SLOT_ID)
        {
            if (cmd == CMD_H_PING_RESPONSE)
            {
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "Global PING_RESPONSE received");
#endif
                _ping_response_received = true;
                return;
            }
        }

        if (slot != this->slot)
            return;

        switch (cmd)
        {
        case CMD_H_ACK:
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "ACK received");
#endif
            _ack_received = true;
            break;
        case CMD_H_NAK:
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "NAK received");
#endif
            _nak_received = true;
            break;
        case CMD_H_WS_EVENT:
            if (len >= 4)
            {
                WSMessageType type = (WSMessageType)pkt[2];
                const uint8_t *payload = &pkt[3];
                size_t p_len = len - 5;

#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "WS Event received");
#endif

                switch (type)
                {
                case WS_EVENT_CONNECTED:
                    _connected_status = true;
                    if (_event_callback)
                        _event_callback(type, nullptr, 0);
                    break;
                case WS_EVENT_DISCONNECTED:
                    _connected_status = false;
                    if (_event_callback)
                        _event_callback(type, nullptr, 0);
                    break;
                case WS_FRAME_TEXT:
                case WS_FRAME_BINARY:
                    // Direct callback with payload
                    if (_event_callback)
                        _event_callback(type, payload, p_len);
                    break;
                default:
                    if (_event_callback)
                        _event_callback(type, payload, p_len);
                    break;
                }
            }
            break;
        case CMD_H_FLAG_RESPONSE:
            if (len >= 3)
            {
                _received_flag = pkt[2];
                _flag_response_received = true;
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "Got Flag Response");
#endif
            }
            break;
        }
    }

    bool awaitAckNak(uint32_t timeout)
    {
        uint32_t start = millis();
        _ack_received = false;
        _nak_received = false;

#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Waiting for ACK...");
#endif

        while (millis() - start < timeout)
        {
            maintenance();
            if (_ack_received)
                return true;
            if (_nak_received)
                return false;
            delay(1);
        }

#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "ACK Timeout!");
#endif

        return false;
    }

    bool awaitPingResponse(uint32_t timeout)
    {
        uint32_t start = millis();
        _ping_response_received = false;

#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Waiting for PING_RESPONSE...");
#endif

        while (millis() - start < timeout)
        {
            maintenance();
            if (_ping_response_received)
                return true;
            delay(1);
        }

#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "PING Timeout!");
#endif

        return false;
    }

    bool awaitFlagResponse(uint32_t timeout)
    {
        uint32_t start = millis();
        _flag_response_received = false;
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Waiting for FLAG_RESPONSE...");
#endif
        while (millis() - start < timeout)
        {
            maintenance();
            if (_flag_response_received)
                return true;
            delay(1);
        }
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "ERROR: Flag Timeout!");
#endif
        return false;
    }

    bool sendCommand(uint8_t cmd, uint8_t slot, const uint8_t *payload, size_t len, bool wait, uint32_t timeout = DEFAULT_CMD_TIMEOUT)
    {
        if (!sink)
            return false;
        _ack_received = false;
        _nak_received = false;

#if defined(ENABLE_LOCAL_DEBUG)
        if (cmd != CMD_C_WS_LOOP)
        { // Avoid spamming logs with loop commands
            char msg[50];
            snprintf(msg, sizeof(msg), "Sending WS command: %02X for slot %d", cmd, slot);
            DEBUG_PRINT(_debug_level, "[Client]", msg);
        }
#endif

        if (sendPacket(*sink, cmd, slot, payload, len) == 0)
        {
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "Send packet failed");
#endif
            return false;
        }

        if (wait)
        {
            if (cmd == CMD_C_PING_HOST)
                return awaitPingResponse(timeout);
            return awaitAckNak(timeout);
        }
        return true;
    }

    void _push_data(const uint8_t *data, size_t len, WSMessageType type)
    {
        if (_event_callback)
        {
            _event_callback(type, data, len);
        }
    }

    size_t _get_buffered_count()
    {
        if (_rx_head == _rx_tail)
            return 0;
        if (_rx_head > _rx_tail)
            return _rx_head - _rx_tail;
        return SERIAL_WS_RX_BUFFER_SIZE - (_rx_tail - _rx_head);
    }

public:
    /**
     * @brief Constructor for SerialWebsocketClient.
     * @param sink The Stream interface (e.g., Serial1, SoftwareSerial) used for communication.
     * @param slot The specific client slot ID to use on the serial bridge (0 to MAX_SLOTS-1).
     */
    SerialWebsocketClient(Stream &sink, int slot = 0) : sink(&sink), slot(slot) {}

    /**
     * @brief Registers a callback function to handle incoming frames and connection events.
     * This is the primary way the client application receives data.
     * @param callback Function pointer matching the WebSocketEventCallback signature.
     */
    void onEvent(WebSocketEventCallback callback) { _event_callback = callback; }

    /**
     * @brief Initiates a WebSocket connection on the host bridge.
     * Host will handle the HTTP/1.1 upgrade handshake.
     * @param host The hostname or IP address of the server.
     * @param port The port number (e.g., 80 or 443).
     * @param path The WebSocket path (e.g., "/ws").
     * @param use_ssl Set to true for WSS connection.
     * @return true if connection command was acknowledged by host, false otherwise.
     */
    bool connect(const char *host, uint16_t port, const char *path, bool use_ssl = false)
    {
        size_t hl = strlen(host);
        size_t pl = strlen(path);
        if (hl > 100 || pl > 100)
            return false;

        size_t total = 5 + hl + pl;
        uint8_t p[total];
        p[0] = (uint8_t)use_ssl;
        p[1] = (uint8_t)(port >> 8);
        p[2] = (uint8_t)(port & 0xFF);
        p[3] = (uint8_t)hl;
        memcpy(&p[4], host, hl);
        p[4 + hl] = (uint8_t)pl;
        memcpy(&p[5 + hl], path, pl);

        return sendCommand(CMD_C_WS_CONNECT, this->slot, p, total, true, SERIAL_TCP_CONNECT_TIMEOUT);
    }

    /**
     * @brief Sends a text frame over the established WebSocket connection.
     * @param payload The text data to send (null-terminated).
     * @return true if command was acknowledged, false otherwise.
     */
    bool sendText(const char *payload)
    {
        return sendBinary((const uint8_t *)payload, strlen(payload), WS_FRAME_TEXT);
    }

    /**
     * @brief Sends a binary frame over the established WebSocket connection.
     * @param payload The binary data buffer.
     * @param len The length of the binary data.
     * @return true if command was acknowledged, false otherwise.
     */
    bool sendBinary(const uint8_t *payload, size_t len, WSMessageType type = WS_FRAME_BINARY)
    {
        if (!_connected_status || len == 0)
            return false;
        size_t total = 1 + len;
        uint8_t p[total];
        p[0] = (uint8_t)type;
        memcpy(&p[1], payload, len);
        return sendCommand(CMD_C_WS_SEND_FRAME, this->slot, p, total, true);
    }

    /**
     * @brief Sends a WebSocket close frame and initiates the closure of the underlying TCP connection.
     * @return true if command was acknowledged, false otherwise.
     */
    bool disconnect()
    {
        bool s = sendCommand(CMD_C_WS_DISCONNECT, this->slot, nullptr, 0, true);
        if (s)
            _connected_status = false;
        return s;
    }

    /**
     * @brief Alias for disconnect(), stops the WebSocket connection.
     */
    void stop()
    {
        disconnect();
    }

    /**
     * @brief Checks the connection status.
     * @return true if the WebSocket is connected, false otherwise.
     */
    bool connected() { return _connected_status; }

    /**
     * @brief Polls the host for new WebSocket events (frames or status updates).
     * This function MUST be called frequently in the main loop() to ensure timely data processing.
     */
    void loop() { maintenance(); }

    /**
     * @brief Pings the host to check if the serial bridge is alive.
     * @param timeout The maximum time (in milliseconds) to wait for a response. Defaults to 1000ms.
     * @return true if the host responded with a PING_RESPONSE, false otherwise.
     */
    bool pingHost(uint32_t timeout = 1000)
    {
        return sendCommand(CMD_C_PING_HOST, GLOBAL_SLOT_ID, nullptr, 0, true, timeout);
    }

    /**
     * @brief Sets the debug level only on the client device.
     * This controls the verbosity of client-side debug prints.
     * @param level The debug level (0 = none, higher = more verbose).
     */
    void setLocalDebugLevel(int level) { _debug_level = (uint8_t)level; }

    /**
     * @brief Printing message to Host Stream (Serial).
     * @param info Information to print.
     */
    void hostPrint(const char *info)
    {
        size_t len = strlen(info);
        size_t offset = 0;
        // Use small chunk size to avoid flooding serial buffer all at once
        const size_t CHUNK_SIZE = 64;

        while (offset < len)
        {
            size_t toSend = len - offset;
            if (toSend > CHUNK_SIZE)
                toSend = CHUNK_SIZE;

            // wait_for_ack = false: Fire and forget.
            // This prevents blocking the Arduino waiting for a response while it might be
            // receiving data on another slot, avoiding deadlock.
            sendCommand(CMD_C_DEBUG_INFO, GLOBAL_SLOT_ID, (const uint8_t *)(info + offset), toSend, false);
            offset += toSend;
        }
    }

    /**
     * @brief Printing message to Host Stream (Serial).
     * @param info Information to print.
     */
    void hostPrint(String info) // Overload for String
    {
        hostPrint(info.c_str());
    }

    /**
     * @brief Sets the custom 8-bit flag on the Host bridge for this slot.
     * @param flag The 8-bit value to set.
     * @return true if successful (ACK received), false otherwise.
     */
    bool setFlag(uint8_t flag)
    {
        return sendCommand(CMD_C_SET_FLAG, this->slot, &flag, 1, true);
    }

    /**
     * @brief Gets the custom 8-bit flag from the Host bridge for this slot.
     * @return The 8-bit flag value (defaults to 0 if request fails).
     */
    uint8_t getFlag()
    {
        if (sendCommand(CMD_C_GET_FLAG, this->slot, nullptr, 0, false))
        {
            if (awaitFlagResponse(DEFAULT_CMD_TIMEOUT))
            {
                return _received_flag;
            }
        }
        return 0;
    }

    /**
     * @brief Sets the insecure flag (bit 2) to skip SSL certificate verification.
     * @param insecure True to disable verification, false to enable it.
     */
    void setInsecure(bool insecure = true)
    {
        uint8_t flag = getFlag();
        if (insecure)
            flag |= SSL_INSECURE_BIT;
        else
            flag &= ~SSL_INSECURE_BIT;
        setFlag(flag);
    }

    /**
     * @brief Sets the plain start flag (bit 1) to force SSL client to start in plain mode.
     * @param plain True to start in plain text, false to start in SSL (if configured).
     */
    void setPlainStart(bool plain = true)
    {
        uint8_t flag = getFlag();
        if (plain)
            flag |= SSL_PLAIN_START_BIT;
        else
            flag &= ~SSL_PLAIN_START_BIT;
        setFlag(flag);
    }

    /**
     * @brief Checks if the remote host reports the connection is secure.
     * @return True if secure (SSL/TLS), false if plain text.
     */
    bool isSecure()
    {
        uint8_t flag = getFlag();
        return (flag & SSL_STATUS_BIT) != 0;
    }

    /**
     * @brief Sets the Receive and Transmit buffer sizes for the socket on the host.
     * @param recv The receive buffer size in bytes.
     * @param xmit The transmit buffer size in bytes.
     */
    void setBufferSizes(int recv, int xmit)
    {
        uint8_t payload[4];
        payload[0] = (uint8_t)((recv >> 8) & 0xFF);
        payload[1] = (uint8_t)(recv & 0xFF);
        payload[2] = (uint8_t)((xmit >> 8) & 0xFF);
        payload[3] = (uint8_t)(xmit & 0xFF);
        sendCommand(CMD_C_SET_BUF_SIZE, this->slot, payload, 4, true);
    }
};

#endif // SERIAL_WEBSOCKET_CLIENT_H