/**
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_NETWORK_PROTOCOL_H
#define SERIAL_NETWORK_PROTOCOL_H

#include <Arduino.h>
#include <Stream.h>

// DEBUG
#if defined(ENABLE_LOCAL_DEBUG)
#define DEBUG_PRINT(level, tag, message) \
    do                                   \
    {                                    \
        if (level > 0)                   \
        {                                \
            Serial.print(tag);           \
            Serial.print(" ");           \
            Serial.println(message);     \
        }                                \
    } while (0)
#else
#define DEBUG_PRINT(level, tag, message)
#endif
// END DEBUG

#if !defined(MAX_CLIENT_SLOTS)
#define MAX_CLIENT_SLOTS 4
#endif

// Configuration
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
#define SERIAL_TCP_RX_BUFFER_SIZE 256
#define SERIAL_TCP_HOST_TX_BUFFER_SIZE 256
#define SERIAL_TCP_DATA_PAYLOAD_SIZE 64
#define SERIAL_UDP_RX_BUFFER_SIZE 256
const size_t MAX_PACKET_BUFFER_SIZE = 128;
#else
// Defaults for ESP32, ESP8266, etc.
#define SERIAL_TCP_RX_BUFFER_SIZE 1024
#define SERIAL_TCP_HOST_TX_BUFFER_SIZE 1024
#define SERIAL_TCP_DATA_PAYLOAD_SIZE 200
#define SERIAL_UDP_RX_BUFFER_SIZE 1024
const size_t MAX_PACKET_BUFFER_SIZE = 256;
#endif

#define SERIAL_TCP_DATA_PACKET_TIMEOUT 500

/**
 * @brief Contains the shared logic for the Serial Network bridge protocol.
 */
namespace SerialNetworkProtocol
{
    const uint8_t FRAME_DELIMITER = 0x00;
    const uint8_t GLOBAL_SLOT_ID = 0xFF;
    const uint32_t DEFAULT_CMD_TIMEOUT = 5000;
    const uint32_t SERIAL_TCP_CONNECT_TIMEOUT = 30000;
    const uint8_t MAX_SLOTS = MAX_CLIENT_SLOTS; // Max number of independent network clients (TCP, UDP, WS)
    const uint32_t AUTO_FLUSH_TIMEOUT_MS = 20;

    // SSL Flag Bitmasks
    // Bit 0: READ-ONLY status (set by Host). 1 = Secure Connection Active.
    const uint8_t SSL_STATUS_BIT = 0x01;
    // Bit 1: CONFIG (set by Client). 1 = Start in Plain Text mode (ignore SSL defaults).
    const uint8_t SSL_PLAIN_START_BIT = 0x02;
    // Bit 2: CONFIG (set by Client). 1 = Insecure Mode (Skip Verify).
    const uint8_t SSL_INSECURE_BIT = 0x04;

    // WebSocket Message Types (for CMD_H_WS_EVENT)
    enum WSMessageType : uint8_t
    {
        WS_EVENT_DISCONNECTED = 0x00,
        WS_EVENT_CONNECTED = 0x01,
        WS_FRAME_TEXT = 0x02,
        WS_FRAME_BINARY = 0x03,
        WS_FRAME_PONG = 0x04,
        WS_EVENT_ERROR = 0xFF
    };

    enum Command : uint8_t
    {
        // Client -> Host Global Commands (0x01 - 0x0F)
        CMD_C_SET_WIFI = 0x01,
        CMD_C_CONNECT_NET = 0x02,
        CMD_C_DISCONNECT_NET = 0x03,
        CMD_C_IS_NET_CONNECTED = 0x04,
        CMD_C_SET_DEBUG = 0x05,
        CMD_C_PING_HOST = 0x06,
        CMD_C_REBOOT_HOST = 0x07,
        CMD_C_DEBUG_INFO = 0x08,

        // Client -> Host TCP Commands (0x10 - 0x1F)
        CMD_C_CONNECT_HOST = 0x10,
        CMD_C_WRITE = 0x11,
        CMD_C_STOP = 0x13, // Used to stop TCP connection or UDP/WS listener
        CMD_C_IS_CONNECTED = 0x14,
        CMD_C_DATA_ACK = 0x15,
        CMD_C_START_TLS = 0x16,
        CMD_C_SET_CA_CERT = 0x17,
        CMD_C_POLL_DATA = 0x18,

        // Client -> Host UDP Commands (0x20 - 0x2F)
        CMD_C_UDP_BEGIN = 0x20,
        CMD_C_UDP_END = 0x21,
        CMD_C_UDP_BEGIN_PACKET = 0x22,
        CMD_C_UDP_WRITE_DATA = 0x23,
        CMD_C_UDP_END_PACKET = 0x24,
        CMD_C_UDP_PARSE_PACKET = 0x25,

        // Client -> Host WebSocket Commands (0x30 - 0x3F)
        CMD_C_WS_CONNECT = 0x30,
        CMD_C_WS_SEND_FRAME = 0x31,
        CMD_C_WS_DISCONNECT = 0x32,
        CMD_C_WS_LOOP = 0x33, // Signal host to process WS events/data

        // Client -> Host AsyncTCP Commands (0x40 - 0x5F)
        CMD_C_ATC_CONNECT = 0x40,
        CMD_C_ATC_CLOSE = 0x41,
        CMD_C_ATC_ABORT = 0x42,
        CMD_C_ATC_ADD = 0x43,
        CMD_C_ATC_SEND = 0x44,
        CMD_C_ATC_ACK = 0x45,
        CMD_C_ATC_SET_RX_TIMEOUT = 0x46,
        CMD_C_ATC_SET_ACK_TIMEOUT = 0x47,
        CMD_C_ATC_SET_NO_DELAY = 0x48,
        CMD_C_ATC_SET_KEEP_ALIVE = 0x49,

        // Host -> Client AsyncTCP Events (0x50 - 0x6F)
        CMD_H_ATC_CONNECTED = 0x50,
        CMD_H_ATC_DISCONNECTED = 0x51,
        CMD_H_ATC_DATA = 0x52,
        CMD_H_ATC_ACKED = 0x53,
        CMD_H_ATC_ERROR = 0x54,
        CMD_H_ATC_TIMEOUT = 0x55,
        CMD_H_ATC_POLL = 0x56,

        // Host -> Client Commands (0x80 - 0x9F)
        CMD_H_ACK = 0x80,
        CMD_H_NAK = 0x81,
        CMD_H_NET_STATUS = 0x84,
        CMD_H_PING_RESPONSE = 0x86,
        CMD_H_HOST_RESET = 0x87,

        // Host -> Client TCP Responses (0x90 - 0x9F)
        CMD_H_CONNECTED_STATUS = 0x94,
        CMD_H_DATA_PAYLOAD = 0x95,
        CMD_H_POLL_RESPONSE = 0x96,

        // Host -> Client UDP Responses (0xA0 - 0xAF)
        CMD_H_UDP_PACKET_INFO = 0xA0,
        CMD_H_UDP_DATA_PAYLOAD = 0xA1,

        // Host -> Client WebSocket Responses (0xB0 - 0xBF)
        CMD_H_WS_EVENT = 0xB0,

        // Client <-> Host get/set value (0xC0 - 0xCF)
        CMD_C_GET_FLAG = 0xC0,
        CMD_C_SET_FLAG = 0xC1,      // Client sets flag on Host
        CMD_H_FLAG_RESPONSE = 0xC2, // Host responds with flag value
        CMD_C_SET_BUF_SIZE = 0xC3,  // Set Buffer Sizes
    };

    // CRC16-MODBUS Implementation
    inline uint16_t crc16_update(uint16_t crc, uint8_t a)
    {
        crc ^= a;
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = (crc >> 1);
            }
        }
        return crc;
    }

    inline uint16_t calculate_crc16(const uint8_t *data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++)
        {
            crc = crc16_update(crc, data[i]);
        }
        return crc;
    }

    // COBS (Consistent Overhead Byte Stuffing)
    inline size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
    {
        const uint8_t *start = dst;
        uint8_t *code_ptr = dst++;
        uint8_t code = 1;

        for (size_t i = 0; i < len; i++)
        {
            if (src[i] == 0)
            {
                *code_ptr = code;
                code_ptr = dst++;
                code = 1;
            }
            else
            {
                *dst++ = src[i];
                code++;
                if (code == 0xFF)
                {
                    *code_ptr = code;
                    code_ptr = dst++;
                    code = 1;
                }
            }
        }
        *code_ptr = code;
        return (size_t)(dst - start);
    }

    inline size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst)
    {
        const uint8_t *start = dst;
        const uint8_t *end = src + len;

        while (src < end)
        {
            uint8_t code = *src++;
            if (code == 0)
                return 0;

            for (uint8_t i = 1; i < code; i++)
            {
                if (src >= end)
                    return 0;
                *dst++ = *src++;
            }
            if (code < 0xFF && src < end)
            {
                *dst++ = 0;
            }
        }
        return (size_t)(dst - start);
    }

    // Packet Sending
    inline size_t sendPacket(Stream &sink, uint8_t cmd, uint8_t slot, const uint8_t *payload, size_t len)
    {
        if (len > (MAX_PACKET_BUFFER_SIZE - 4))
            return 0;

        uint8_t rawPkt[MAX_PACKET_BUFFER_SIZE];
        rawPkt[0] = cmd;
        rawPkt[1] = slot;
        if (len > 0 && payload != nullptr)
        {
            memcpy(&rawPkt[2], payload, len);
        }

        size_t rawLen = 2 + len;
        uint16_t crc = calculate_crc16(rawPkt, rawLen);
        rawPkt[rawLen] = (uint8_t)(crc & 0xFF);
        rawPkt[rawLen + 1] = (uint8_t)(crc >> 8);
        rawLen += 2;

        uint8_t cobsPkt[MAX_PACKET_BUFFER_SIZE + 2];
        size_t cobsLen = cobs_encode(rawPkt, rawLen, cobsPkt);

        size_t written = sink.write(cobsPkt, cobsLen);
        written += sink.write(FRAME_DELIMITER);

        return (written == cobsLen + 1) ? written : 0;
    }

    static inline void serial_tcp_yield()
    {
#if defined(ARDUINO_ESP8266_MAJOR) && defined(ARDUINO_ESP8266_MINOR) && defined(ARDUINO_ESP8266_REVISION) && ((ARDUINO_ESP8266_MAJOR == 3 && ARDUINO_ESP8266_MINOR >= 1) || ARDUINO_ESP8266_MAJOR > 3)
        esp_yield();
#else
        delay(0);
#endif
    }

    // Packet Receiver
    class PacketReceiver
    {
    public:
        uint8_t buffer[MAX_PACKET_BUFFER_SIZE];
        size_t pos = 0;

        size_t read_byte(uint8_t b)
        {
            if (b == FRAME_DELIMITER)
            {
                if (pos == 0)
                    return 0;
                size_t len = pos;
                pos = 0;
                return len;
            }
            else
            {
                if (pos < MAX_PACKET_BUFFER_SIZE)
                {
                    buffer[pos++] = b;
                }
                else
                {
                    pos = 0;
                }
            }
            return 0;
        }
    };
} // namespace SerialNetworkProtocol

#endif