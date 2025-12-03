/**
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_UDP_CLIENT_H
#define SERIAL_UDP_CLIENT_H

#include <Arduino.h>
#include <Stream.h>
#include <Udp.h>                   // Standard Arduino UDP base class
#include "SerialNetworkProtocol.h" // Use the new protocol

using namespace SerialNetworkProtocol;

// Buffers
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
#define SERIAL_UDP_TX_BUFFER_SIZE 128
#define SERIAL_UDP_RX_BUFFER_SIZE 256
#else
#define SERIAL_UDP_TX_BUFFER_SIZE 256
#define SERIAL_UDP_RX_BUFFER_SIZE 1024
#endif

class SerialUDPClient : public UDP
{
private:
    Stream *sink = nullptr;
    int slot = 0;

    // TX Buffer (for outgoing packet data)
    uint8_t _tx_buffer[SERIAL_UDP_TX_BUFFER_SIZE];
    size_t _tx_buffer_len = 0;

    // RX Buffer (for incoming packet data)
    uint8_t _rx_buffer[SERIAL_UDP_RX_BUFFER_SIZE];
    volatile size_t _rx_head = 0;
    volatile size_t _rx_tail = 0;

    // Incoming packet metadata
    IPAddress _remoteIP;
    uint16_t _remotePort = 0;
    volatile size_t _packet_size = 0; // Remaining bytes in the entire packet

    // Packet Handling (from SerialNetworkProtocol.h)
    PacketReceiver _receiver;
    uint8_t _decoded_buffer[MAX_PACKET_BUFFER_SIZE];
    volatile bool _ack_received = false;
    volatile bool _nak_received = false;
    volatile bool _packet_info_received = false;

    // For Ping/Debug functionality
    volatile bool _ping_response_received = false;

    // Flag Handling
    volatile bool _flag_response_received = false;
    volatile uint8_t _received_flag = 0;

    uint8_t _debug_level = 1;

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
        DEBUG_PRINT(_debug_level, "[Client]", "ERROR: ACK Timeout!");
#endif
        return false;
    }

    bool awaitPacketInfo(uint32_t timeout)
    {
        uint32_t start = millis();
        _packet_info_received = false;
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Waiting for PACKET_INFO...");
#endif
        while (millis() - start < timeout)
        {
            maintenance();
            if (_packet_info_received)
                return true;
            delay(1);
        }
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "ERROR: PACKET_INFO Timeout!");
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
        DEBUG_PRINT(_debug_level, "[Client]", "ERROR: PING Timeout!");
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

    bool sendCommand(uint8_t cmd, uint8_t slot, const uint8_t *payload, size_t len, bool wait_for_ack, uint32_t timeout = DEFAULT_CMD_TIMEOUT)
    {
        if (!sink)
            return false;

        _ack_received = false;
        _nak_received = false;

#if defined(ENABLE_LOCAL_DEBUG)
        char msg[50];
        snprintf(msg, sizeof(msg), "Sending UDP command: %02X for slot %d", cmd, slot);
        DEBUG_PRINT(_debug_level, "[Client]", msg);
#endif

        if (sendPacket(*sink, cmd, slot, payload, len) == 0)
        {
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "ERROR: sendPacket failed!");
#endif
            return false;
        }

        if (wait_for_ack)
        {
            if (cmd == CMD_C_PING_HOST)
            {
                return awaitPingResponse(timeout);
            }
            else
            {
                return awaitAckNak(timeout);
            }
        }
        return true;
    }

    void processPacket(const uint8_t *pkt, size_t len)
    {
        uint8_t cmd = pkt[0];
        uint8_t slot = pkt[1];

        // Global commands are generally ignored here, focusing on slot-specific.
        // Check for Global responses
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
            DEBUG_PRINT(_debug_level, "[Client]", "Control ACK received");
#endif
            _ack_received = true;
            break;
        case CMD_H_NAK:
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "Control NAK received");
#endif
            _nak_received = true;
            break;
        case CMD_H_UDP_PACKET_INFO:
        {
            if (len < 10)
                break; // cmd(1)+slot(1)+payload(8)+crc(2)

            // Payload: IP (4 bytes) + Port (2 bytes) + Size (2 bytes)
            _remoteIP = IPAddress(pkt[2], pkt[3], pkt[4], pkt[5]);
            _remotePort = (uint16_t)(pkt[6] << 8) | pkt[7];
            size_t total_size = (size_t)((pkt[8] << 8) | pkt[9]);

            _packet_size = total_size;
            _rx_head = 0; // Clear buffer pointers (data will follow immediately)
            _rx_tail = 0;
            _packet_info_received = true;
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "UDP PACKET_INFO received");
#endif
            break;
        }
        case CMD_H_UDP_DATA_PAYLOAD:
        {
            size_t data_len = len - 4; // cmd, slot, crc_lo, crc_hi
            const uint8_t *payload = &pkt[2];

            // Write data to circular buffer
            for (size_t i = 0; i < data_len; i++)
            {
                size_t next_head = (_rx_head + 1) % SERIAL_UDP_RX_BUFFER_SIZE;
                if (next_head != _rx_tail)
                {
                    _rx_buffer[_rx_head] = payload[i];
                    _rx_head = next_head;
                }
            }
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "UDP DATA_PAYLOAD received");
#endif
            break;
        }
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

    void maintenance()
    {
        if (!sink)
            return;
        while (sink->available())
        {
            serial_tcp_yield();
            uint8_t b = sink->read();
            size_t cobsLen = _receiver.read_byte(b);
            if (cobsLen > 0)
            {
                size_t decodedLen = cobs_decode(_receiver.buffer, cobsLen, _decoded_buffer);

                if (decodedLen > 2)
                {
                    uint16_t rcvd_crc = (uint16_t)(_decoded_buffer[decodedLen - 1] << 8) | _decoded_buffer[decodedLen - 2];
                    uint16_t calc_crc = calculate_crc16(_decoded_buffer, decodedLen - 2);

                    if (rcvd_crc == calc_crc)
                    {
                        processPacket(_decoded_buffer, decodedLen);
                    }
#if defined(ENABLE_LOCAL_DEBUG)
                    else
                    {
                        DEBUG_PRINT(_debug_level, "[Client]", "ERROR: Bad CRC on incoming packet!");
                    }
#endif
                }
            }
        }
    }

    size_t _get_buffered_count()
    {
        if (_rx_head == _rx_tail)
            return 0;
        if (_rx_head > _rx_tail)
            return _rx_head - _rx_tail;
        return SERIAL_UDP_RX_BUFFER_SIZE - (_rx_tail - _rx_head);
    }

public:
    /**
     * @brief Constructor for SerialUDPClient.
     * @param sink The Stream interface (e.g., Serial1, SoftwareSerial) used for communication.
     * @param slot The specific client slot ID to use on the serial bridge (0 to MAX_SLOTS-1).
     */
    SerialUDPClient(Stream &sink, int slot = 0)
        : sink(&sink), slot(slot) {}

    // Global/Utility Methods

    /**
     * @brief Pings the host to check if it's alive.
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
    void setLocalDebugLevel(int level)
    {
        _debug_level = (uint8_t)level;
    }

    /**
     * @brief Starts UDP listening on the host device at the specified local port.
     * @param localPort The UDP port number to listen on.
     * @return 1 if the host successfully starts listening, 0 otherwise.
     */
    uint8_t begin(uint16_t localPort) override
    {
        uint8_t payload[2];
        payload[0] = (uint8_t)(localPort >> 8);
        payload[1] = (uint8_t)(localPort & 0xFF);
        return sendCommand(CMD_C_UDP_BEGIN, this->slot, payload, 2, true) ? 1 : 0;
    }

    /**
     * @brief Stops the UDP listener on the host device and clears local state.
     */
    void stop() override
    {
        sendCommand(CMD_C_UDP_END, this->slot, nullptr, 0, false);
        _packet_size = 0;
        _rx_head = 0;
        _rx_tail = 0;
    }

    /**
     * @brief Checks for the presence of a pending incoming UDP datagram from the host.
     * If a packet is found, it updates internal metadata (remote IP/port) and buffers the data.
     * @return The size of the incoming packet in bytes, or 0 if no packet is available.
     */
    int parsePacket() override
    {
        // If data is already buffered, return the remaining size.
        if (_packet_size > 0)
            return _packet_size;

        // Command host to check for a new packet
        if (sendCommand(CMD_C_UDP_PARSE_PACKET, this->slot, nullptr, 0, false))
        {
            // Await the host's response (CMD_H_UDP_PACKET_INFO)
            if (awaitPacketInfo(DEFAULT_CMD_TIMEOUT))
            {
                return _packet_size;
            }
        }
        return 0;
    }

    /**
     * @brief Prepares a new packet to be sent to a remote IP address and port.
     * @param ip The IPAddress object of the destination.
     * @param port The destination port number.
     * @return 1 on success, 0 on failure.
     */
    int beginPacket(IPAddress ip, uint16_t port) override
    {
        _tx_buffer_len = 0;

        // Payload: [Port (2 bytes)] [Type (1 byte)] [IP (4 bytes)]
        uint8_t payload[7];
        payload[0] = (uint8_t)(port >> 8);
        payload[1] = (uint8_t)(port & 0xFF);
        payload[2] = 0x00; // Type 0: IP Address
        payload[3] = ip[0];
        payload[4] = ip[1];
        payload[5] = ip[2];
        payload[6] = ip[3];

        return sendCommand(CMD_C_UDP_BEGIN_PACKET, this->slot, payload, 7, true) ? 1 : 0;
    }

    /**
     * @brief Prepares a new packet to be sent to a remote hostname and port.
     * Host will perform DNS resolution upon sending the packet.
     * @param host The hostname (e.g., "google.com").
     * @param port The destination port number.
     * @return 1 on success, 0 on failure.
     */
    int beginPacket(const char *host, uint16_t port) override
    {
        _tx_buffer_len = 0;
        size_t host_len = strlen(host);

        // Payload: [Port (2 bytes)] [Type (1 byte)] [Host Len (1 byte)] [Host (N bytes)]
        size_t payload_len = 4 + host_len;
        uint8_t payload[payload_len];

        payload[0] = (uint8_t)(port >> 8);
        payload[1] = (uint8_t)(port & 0xFF);
        payload[2] = 0x01; // Type 1: Hostname
        payload[3] = (uint8_t)host_len;
        memcpy(&payload[4], host, host_len);

        return sendCommand(CMD_C_UDP_BEGIN_PACKET, this->slot, payload, payload_len, true) ? 1 : 0;
    }

    /**
     * @brief Sends the currently buffered packet data to the destination defined by beginPacket.
     * The local buffer is cleared after transmission.
     * @return 1 on success (packet successfully sent), 0 on failure.
     */
    int endPacket() override
    {
        size_t sent = 0;
        size_t remaining = _tx_buffer_len;

        // Send the data using CMD_C_UDP_WRITE_DATA in chunks
        while (remaining > 0)
        {
            size_t len_to_send = min(remaining, (size_t)SERIAL_TCP_DATA_PAYLOAD_SIZE);

            if (sendCommand(CMD_C_UDP_WRITE_DATA, this->slot, &_tx_buffer[sent], len_to_send, true))
            {
                sent += len_to_send;
                remaining -= len_to_send;
            }
            else
            {
                _tx_buffer_len = 0;
                return 0;
            }
        }

        _tx_buffer_len = 0;

        // Send the final END_PACKET command
        return sendCommand(CMD_C_UDP_END_PACKET, this->slot, nullptr, 0, true) ? 1 : 0;
    }

    /**
     * @brief Writes a single byte to the outgoing packet buffer.
     * The byte is not sent over the network until endPacket() is called.
     * @param b The byte to write.
     * @return 1 if the byte was buffered, 0 if the buffer is full.
     */
    size_t write(uint8_t b) override { return write(&b, 1); }

    /**
     * @brief Writes a buffer of data to the outgoing packet buffer.
     * Data is not sent over the network until endPacket() is called.
     * @param buffer Pointer to the data buffer.
     * @param size The number of bytes to write.
     * @return The number of bytes successfully buffered.
     */
    size_t write(const uint8_t *buffer, size_t size) override
    {
        size_t written = 0;
        for (size_t i = 0; i < size; i++)
        {
            if (_tx_buffer_len >= SERIAL_UDP_TX_BUFFER_SIZE)
                return written;
            _tx_buffer[_tx_buffer_len++] = buffer[i];
            written++;
        }
        return written;
    }

    /**
     * @brief Gets the number of bytes available to read in the current incoming packet buffer.
     * Calls maintenance to process available serial data.
     * @return The number of available bytes in the local buffer.
     */
    int available() override
    {
        maintenance();
        return _get_buffered_count();
    }

    /**
     * @brief Reads a single byte from the incoming packet buffer.
     * Decrements the remaining packet size counter.
     * @return The next available byte, or -1 if the buffer is empty.
     */
    int read() override
    {
        maintenance();

        if (_get_buffered_count() == 0)
            return -1;

        uint8_t b = _rx_buffer[_rx_tail];
        _rx_tail = (_rx_tail + 1) % SERIAL_UDP_RX_BUFFER_SIZE;
        _packet_size = _packet_size - 1;

        return b;
    }

    /**
     * @brief Reads a buffer of data from the incoming packet.
     * @param buffer Pointer to the destination buffer.
     * @param len The maximum number of bytes to read.
     * @return The number of bytes actually read.
     */
    int read(unsigned char *buffer, size_t len) override { return read((char *)buffer, len); }
    int read(char *buffer, size_t len) override
    {
        size_t count = 0;
        while (count < len && available() > 0)
        {
            int b = read();
            if (b >= 0)
                buffer[count++] = (char)b;
            else
                break;
        }
        return count;
    }

    /**
     * @brief Peeks at the next byte in the incoming packet buffer without consuming it.
     * @return The next available byte, or -1 if the buffer is empty.
     */
    int peek() override
    {
        maintenance();
        if (_get_buffered_count() == 0)
            return -1;
        return _rx_buffer[_rx_tail];
    }

    /**
     * @brief Required by the base Stream/Print class.
     * Since UDP packets are sent with endPacket(), this method is a no-op for datagrams.
     */
    void flush() override
    {
        // No operation needed for UDP.
    }

    /**
     * @brief Gets the IPAddress of the sender of the currently parsed packet.
     * @return The remote IPAddress.
     */
    IPAddress remoteIP() override { return _remoteIP; }

    /**
     * @brief Gets the port number of the sender of the currently parsed packet.
     * @return The remote port number.
     */
    uint16_t remotePort() override { return _remotePort; }

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
     * Note: For UDP, this affects the datagram queue size.
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

    using Print::write;
};

#endif