/*
 * SerialAsyncTCPClient.h
 * Complete AsyncTCP client proxy for SerialNetworkBridge
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_ASYNC_TCP_CLIENT_H
#define SERIAL_ASYNC_TCP_CLIENT_H

#include <Arduino.h>
#include <Stream.h>
#include "SerialNetworkProtocol.h"

using namespace SerialNetworkProtocol;

// Forward declaration
class SerialAsyncTCPClient;

/**
 * @brief Mock pbuf structure to match AsyncTCP signature.
 * Renamed from pbuf to SerialPbuf to avoid conflict with LwIP.
 */
struct SerialPbuf {
    struct SerialPbuf *next;
    void *payload;
    uint16_t tot_len;
    uint16_t len;
};

typedef void (*SatcConnectHandler)(void *arg, SerialAsyncTCPClient *client);
typedef void (*SatcAckHandler)(void *arg, SerialAsyncTCPClient *client, size_t len, uint32_t time);
typedef void (*SatcErrorHandler)(void *arg, SerialAsyncTCPClient *client, int8_t error);
typedef void (*SatcDataHandler)(void *arg, SerialAsyncTCPClient *client, void *data, size_t len);
typedef void (*SatcPacketHandler)(void *arg, SerialAsyncTCPClient *client, struct SerialPbuf *pb);
typedef void (*SatcTimeoutHandler)(void *arg, SerialAsyncTCPClient *client, uint32_t time);

// Async Write Flags
#define ASYNC_WRITE_FLAG_COPY 0x01
#define ASYNC_WRITE_FLAG_MORE 0x02

// LwIP State Codes (Simplified)
#define SATC_CLOSED      0
#define SATC_LISTEN      1
#define SATC_SYN_SENT    2
#define SATC_SYN_RCVD    3
#define SATC_ESTABLISHED 4
#define SATC_FIN_WAIT_1  5
#define SATC_FIN_WAIT_2  6
#define SATC_CLOSE_WAIT  7
#define SATC_CLOSING     8
#define SATC_LAST_ACK    9
#define SATC_TIME_WAIT   10

class SerialAsyncTCPClient
{
private:
    Stream *sink = nullptr;
    int slot = 0;

    // Internal State
    volatile uint8_t _state = SATC_CLOSED; 
    uint16_t _mss = 1460; // Default MSS
    
    // IP Cache
    uint32_t _remote_ip = 0;
    uint16_t _remote_port = 0;
    uint32_t _local_ip = 0;
    uint16_t _local_port = 0;

    // Config Cache
    uint32_t _rx_timeout = 0;
    uint32_t _ack_timeout = 0;
    bool _no_delay = false;
    uint8_t _debug_level = 1;

    // Flow Control
    size_t _estimated_space = 5760; 
    volatile bool _ping_resp = false;

    // Callbacks
    SatcConnectHandler _connect_cb = nullptr; void *_connect_arg = nullptr;
    SatcConnectHandler _disconnect_cb = nullptr; void *_disconnect_arg = nullptr;
    SatcAckHandler _ack_cb = nullptr; void *_ack_arg = nullptr;
    SatcErrorHandler _error_cb = nullptr; void *_error_arg = nullptr;
    SatcDataHandler _data_cb = nullptr; void *_data_arg = nullptr;
    SatcPacketHandler _packet_cb = nullptr; void *_packet_arg = nullptr;
    SatcTimeoutHandler _timeout_cb = nullptr; void *_timeout_arg = nullptr;
    SatcConnectHandler _poll_cb = nullptr; void *_poll_arg = nullptr;

    // Protocol Handling
    PacketReceiver _receiver;
    uint8_t _decoded_buffer[MAX_PACKET_BUFFER_SIZE];

    void processPacket(const uint8_t *pkt, size_t len)
    {
        uint8_t cmd = pkt[0];
        uint8_t pkt_slot = pkt[1];

        // Handle Global Responses (like Ping)
        if (pkt_slot == GLOBAL_SLOT_ID) {
            if (cmd == CMD_H_PING_RESPONSE) {
                _ping_resp = true;
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "Ping Response Received");
#endif
            }
            return;
        }

        if (pkt_slot != this->slot) return;

        const uint8_t *payload = &pkt[2];
        size_t p_len = len - 2;

        switch (cmd)
        {
        case CMD_H_ATC_CONNECTED:
            if (p_len >= 14)
            {
                _state = SATC_ESTABLISHED;
                _mss = (uint16_t)(payload[0] << 8) | payload[1];
                // Remote IP (Little Endian in payload)
                _remote_ip = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8) | ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
                _remote_port = (uint16_t)(payload[6] << 8) | payload[7];
                // Local IP
                _local_ip = (uint32_t)payload[8] | ((uint32_t)payload[9] << 8) | ((uint32_t)payload[10] << 16) | ((uint32_t)payload[11] << 24);
                _local_port = (uint16_t)(payload[12] << 8) | payload[13];
                
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "Connected!");
#endif
                if (_connect_cb) _connect_cb(_connect_arg, this);
            }
            break;

        case CMD_H_ATC_DISCONNECTED:
            _state = SATC_CLOSED;
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "Disconnected");
#endif
            if (_disconnect_cb) _disconnect_cb(_disconnect_arg, this);
            break;

        case CMD_H_ATC_DATA:
#if defined(ENABLE_LOCAL_DEBUG)
            {
                char msg[32];
                snprintf(msg, sizeof(msg), "Data Rx: %u bytes", (unsigned int)p_len);
                DEBUG_PRINT(_debug_level, "[Client]", msg);
            }
#endif
            // Send Protocol ACK to Host immediately
            // This tells the Host "Packet Received, you can send the next one".
            sendCmd(CMD_C_DATA_ACK, nullptr, 0);

            if (_packet_cb) {
                struct SerialPbuf pb;
                pb.next = nullptr;
                pb.payload = (void*)payload;
                pb.len = (uint16_t)p_len;
                pb.tot_len = (uint16_t)p_len;
                _packet_cb(_packet_arg, this, &pb);
            }
            else if (_data_cb) {
                _data_cb(_data_arg, this, (void *)payload, p_len);
            }
            break;

        case CMD_H_ATC_ACKED:
            if (p_len >= 6) {
                size_t ack_len = (size_t)((payload[0] << 8) | payload[1]);
                uint32_t time = (uint32_t)((payload[2] << 24) | (payload[3] << 16) | (payload[4] << 8) | payload[5]);
                _estimated_space += ack_len; 
#if defined(ENABLE_LOCAL_DEBUG)
                char msg[40];
                snprintf(msg, sizeof(msg), "Ack: %u bytes, Time: %lu ms", (unsigned int)ack_len, (unsigned long)time);
                DEBUG_PRINT(_debug_level, "[Client]", msg);
#endif
                if (_ack_cb) _ack_cb(_ack_arg, this, ack_len, time);
            }
            break;

        case CMD_H_ATC_ERROR:
            if (p_len >= 1) {
                _state = SATC_CLOSED;
                int8_t err = (int8_t)payload[0];
#if defined(ENABLE_LOCAL_DEBUG)
                char msg[32];
                snprintf(msg, sizeof(msg), "Error Code: %d", err);
                DEBUG_PRINT(_debug_level, "[Client]", msg);
#endif
                if (_error_cb) _error_cb(_error_arg, this, err);
            }
            break;

        case CMD_H_ATC_TIMEOUT:
            if (p_len >= 4 && _timeout_cb) {
                uint32_t time = (uint32_t)((payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]);
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Client]", "Timeout Event");
#endif
                _timeout_cb(_timeout_arg, this, time);
            }
            break;

        case CMD_H_ATC_POLL:
            if (_poll_cb) _poll_cb(_poll_arg, this);
            break;
        }
    }

    void sendCmd(uint8_t cmd, const uint8_t *payload, size_t len)
    {
        if (sink) SerialNetworkProtocol::sendPacket(*sink, cmd, slot, payload, len);
    }

public:
    /**
     * @brief Constructor
     * @param serial The Stream object (Serial port) connected to the Host.
     * @param slot_id The unique slot ID for this client (0-MAX_SLOTS).
     */
    SerialAsyncTCPClient(Stream &serial, int slot_id) : sink(&serial), slot(slot_id) {}
    
    /**
     * @brief Checks if the Host Bridge is responsive.
     * Sends a PING command and waits for a response.
     * @param timeout Max time to wait in milliseconds (default 1000).
     * @return true if Host responded, false otherwise.
     */
    bool pingHost(uint32_t timeout = 1000) {
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Pinging Host...");
#endif
        _ping_resp = false;
        if (sink) SerialNetworkProtocol::sendPacket(*sink, CMD_C_PING_HOST, GLOBAL_SLOT_ID, nullptr, 0);
        
        uint32_t start = millis();
        while (millis() - start < timeout) {
            loop(); 
            if (_ping_resp) return true;
            delay(1);
        }
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Ping Timeout");
#endif
        return false;
    }

    /**
     * @brief Sets the debug level for local printouts.
     * @param level 0=Off, 1=Basic, 2=Verbose
     */
    void setLocalDebugLevel(uint8_t level) {
        _debug_level = level;
    }

    bool operator==(const SerialAsyncTCPClient &other) const { return slot == other.slot && sink == other.sink; }
    bool operator!=(const SerialAsyncTCPClient &other) const { return !(*this == other); }

    /**
     * @brief Connects to a TCP Server asynchronously.
     * @param host Hostname or IP address string.
     * @param port TCP Port.
     * @return true if command sent successfully, false if busy or invalid.
     */
    bool connect(const char *host, uint16_t port)
    {
        if (_state != SATC_CLOSED) {
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Client]", "Connect failed: Already active");
#endif
            return false;
        }
        size_t host_len = strlen(host);
        if (host_len > 100) return false;
        
        size_t pl_len = 3 + host_len;
        uint8_t payload[pl_len];
        payload[0] = (uint8_t)(port >> 8);
        payload[1] = (uint8_t)(port & 0xFF);
        payload[2] = (uint8_t)host_len;
        memcpy(&payload[3], host, host_len);
        
        _state = SATC_SYN_SENT; 
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Connecting...");
#endif
        sendCmd(CMD_C_ATC_CONNECT, payload, pl_len);
        return true;
    }

    /**
     * @brief Connects to a TCP Server using IPAddress.
     * @param ip Target IP Address.
     * @param port TCP Port.
     * @return true if command sent successfully.
     */
    bool connect(IPAddress ip, uint16_t port)
    {
        char ip_str[16];
        sprintf(ip_str, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        return connect(ip_str, port);
    }

    /**
     * @brief Gracefully closes the connection.
     * @param now (Unused in this implementation, kept for API compat).
     */
    void close(bool now = false)
    {
        if (_state == SATC_CLOSED) return;
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Closing...");
#endif
        sendCmd(CMD_C_ATC_CLOSE, nullptr, 0);
        _state = SATC_FIN_WAIT_1; 
    }

    /**
     * @brief Stops the client (Alias for close).
     */
    void stop() { close(); }

    /**
     * @brief Aborts the connection immediately.
     * @return 0 (ERR_OK).
     */
    int8_t abort()
    {
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Client]", "Aborting");
#endif
        sendCmd(CMD_C_ATC_ABORT, nullptr, 0);
        _state = SATC_CLOSED;
        return 0; // ERR_OK
    }

    /**
     * @brief Checks if the client is free/closed.
     */
    bool free() 
    {
        return _state == SATC_CLOSED || _state == SATC_TIME_WAIT;
    }

    /**
     * @brief Returns the available space in the write buffer (estimated).
     */
    size_t space() const { return _estimated_space; }

    /**
     * @brief Checks if data can be sent.
     */
    bool canSend() const { return space() > 0; }

    /**
     * @brief Adds data to the send buffer.
     * @param data Pointer to data.
     * @param size Length of data.
     * @param apiflags Copy flags (ignored in bridge).
     * @return Number of bytes added.
     */
    size_t add(const char *data, size_t size, uint8_t apiflags = 0)
    {
        if (_state != SATC_ESTABLISHED) return 0;
        
        const size_t max_payload = SERIAL_TCP_DATA_PAYLOAD_SIZE;
        size_t sent = 0;
        
        while (size > 0)
        {
            size_t chunk = (size > max_payload) ? max_payload : size;
            sendCmd(CMD_C_ATC_ADD, (const uint8_t *)data + sent, chunk);
            
            sent += chunk;
            size -= chunk;
            
            if (_estimated_space >= chunk) _estimated_space -= chunk;
            else _estimated_space = 0;
        }
        return sent;
    }

    /**
     * @brief Flushes the send buffer (Triggers sending on Host).
     * @return true
     */
    bool send()
    {
        sendCmd(CMD_C_ATC_SEND, nullptr, 0);
        return true;
    }

    /**
     * @brief writes data to the connection (Add + Send).
     */
    size_t write(const char *data, size_t size, uint8_t apiflags = 0)
    {
        size_t ret = add(data, size, apiflags);
        send();
        return ret;
    }

    size_t write(const char *data) { return write(data, strlen(data)); }

    /**
     * @brief Acknowledges receipt of data to the server (Window Update).
     * @param len Number of bytes to acknowledge.
     */
    size_t ack(size_t len)
    {
        uint8_t p[2] = {(uint8_t)(len >> 8), (uint8_t)(len & 0xFF)};
        sendCmd(CMD_C_ATC_ACK, p, 2);
        return len;
    }

    void ackPacket(struct SerialPbuf *pb) 
    { 
        if(pb) ack(pb->len); 
    }
    
    void ackLater() { }

    /**
     * @brief Sets the RX timeout.
     * @param timeout Timeout in seconds.
     */
    void setRxTimeout(uint32_t timeout)
    {
        _rx_timeout = timeout;
        uint8_t p[4] = {(uint8_t)(timeout >> 24), (uint8_t)(timeout >> 16), (uint8_t)(timeout >> 8), (uint8_t)timeout};
        sendCmd(CMD_C_ATC_SET_RX_TIMEOUT, p, 4);
    }
    uint32_t getRxTimeout() const { return _rx_timeout; }

    /**
     * @brief Sets the ACK timeout.
     * @param timeout Timeout in milliseconds.
     */
    void setAckTimeout(uint32_t timeout)
    {
        _ack_timeout = timeout;
        uint8_t p[4] = {(uint8_t)(timeout >> 24), (uint8_t)(timeout >> 16), (uint8_t)(timeout >> 8), (uint8_t)timeout};
        sendCmd(CMD_C_ATC_SET_ACK_TIMEOUT, p, 4);
    }
    uint32_t getAckTimeout() const { return _ack_timeout; }

    /**
     * @brief Sets NoDelay (Nagle's Algorithm).
     * @param nodelay true to disable Nagle.
     */
    void setNoDelay(bool nodelay)
    {
        _no_delay = nodelay;
        uint8_t p = (uint8_t)nodelay;
        sendCmd(CMD_C_ATC_SET_NO_DELAY, &p, 1);
    }
    bool getNoDelay() { return _no_delay; }

    /**
     * @brief Configures TCP KeepAlive.
     * @param ms Interval in milliseconds.
     * @param cnt Count.
     */
    void setKeepAlive(uint32_t ms, uint8_t cnt)
    {
        uint8_t p[5];
        p[0] = (uint8_t)(ms >> 24); p[1] = (uint8_t)(ms >> 16); p[2] = (uint8_t)(ms >> 8); p[3] = (uint8_t)(ms & 0xFF);
        p[4] = cnt;
        sendCmd(CMD_C_ATC_SET_KEEP_ALIVE, p, 5);
    }

    uint8_t state() const { return _state; }
    
    bool connected() const { return _state == SATC_ESTABLISHED; }
    bool connecting() const { return _state == SATC_SYN_SENT || _state == SATC_SYN_RCVD; }
    bool disconnecting() const { return _state == SATC_FIN_WAIT_1 || _state == SATC_FIN_WAIT_2 || _state == SATC_CLOSING; }
    bool disconnected() const { return _state == SATC_CLOSED || _state == SATC_TIME_WAIT; }
    bool freeable() const { return _state == SATC_CLOSED || _state == SATC_TIME_WAIT; }
    
    uint16_t getMss() const { return _mss; }

    // Address Getters (IPv4 Only for now)
    uint32_t getRemoteAddress() const { return _remote_ip; }
    uint16_t getRemotePort() const { return _remote_port; }
    uint32_t getLocalAddress() const { return _local_ip; }
    uint16_t getLocalPort() const { return _local_port; }

    IPAddress remoteIP() const { return IPAddress(_remote_ip); }
    uint16_t remotePort() const { return _remote_port; }
    IPAddress localIP() const { return IPAddress(_local_ip); }
    uint16_t localPort() const { return _local_port; }

    void onConnect(SatcConnectHandler cb, void *arg = 0) { _connect_cb = cb; _connect_arg = arg; }
    void onDisconnect(SatcConnectHandler cb, void *arg = 0) { _disconnect_cb = cb; _disconnect_arg = arg; }
    void onAck(SatcAckHandler cb, void *arg = 0) { _ack_cb = cb; _ack_arg = arg; }
    void onError(SatcErrorHandler cb, void *arg = 0) { _error_cb = cb; _error_arg = arg; }
    void onData(SatcDataHandler cb, void *arg = 0) { _data_cb = cb; _data_arg = arg; }
    void onPacket(SatcPacketHandler cb, void *arg = 0) { _packet_cb = cb; _packet_arg = arg; }
    void onTimeout(SatcTimeoutHandler cb, void *arg = 0) { _timeout_cb = cb; _timeout_arg = arg; }
    void onPoll(SatcConnectHandler cb, void *arg = 0) { _poll_cb = cb; _poll_arg = arg; }

    static const char *errorToString(int8_t error)
    {
        switch (error) {
            case 0: return "OK";
            case -1: return "Out of memory";
            case -2: return "Buffer error";
            case -3: return "Timeout";
            case -4: return "Routing problem";
            case -5: return "In progress";
            case -6: return "Illegal value";
            case -7: return "Would block";
            case -8: return "Address in use";
            case -9: return "Already connected";
            case -10: return "Not connected";
            case -11: return "IF error";
            case -12: return "Aborted";
            case -13: return "Reset";
            case -14: return "Closed";
            case -15: return "Illegal argument";
            case -55: return "DNS failed";
            default: return "Unknown";
        }
    }

    const char *stateToString() const {
        switch (_state) {
            case SATC_CLOSED: return "Closed";
            case SATC_LISTEN: return "Listen";
            case SATC_SYN_SENT: return "SYN Sent";
            case SATC_SYN_RCVD: return "SYN Received";
            case SATC_ESTABLISHED: return "Established";
            case SATC_FIN_WAIT_1: return "FIN Wait 1";
            case SATC_FIN_WAIT_2: return "FIN Wait 2";
            case SATC_CLOSE_WAIT: return "Close Wait";
            case SATC_CLOSING: return "Closing";
            case SATC_LAST_ACK: return "Last ACK";
            case SATC_TIME_WAIT: return "Time Wait";
            default: return "Unknown";
        }
    }

    /**
     * @brief Main processing loop.
     * Must be called frequently in the main sketch loop to handle incoming serial data.
     */
    void loop()
    {
        if (!sink) return;
        while (sink->available())
        {
            serial_tcp_yield();
            uint8_t b = sink->read();
            size_t cobsLen = _receiver.read_byte(b);
            if (cobsLen > 0)
            {
                size_t len = cobs_decode(_receiver.buffer, cobsLen, _decoded_buffer);
                if (len > 2)
                {
                    uint16_t calc_crc = calculate_crc16(_decoded_buffer, len - 2);
                    uint16_t recv_crc = (uint16_t)(_decoded_buffer[len - 1] << 8 | _decoded_buffer[len - 2]);
                    if (calc_crc == recv_crc)
                    {
                        processPacket(_decoded_buffer, len - 2);
                    }
                }
            }
        }
    }
};

#endif