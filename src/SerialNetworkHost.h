/*
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_NETWORK_HOST_H
#define SERIAL_NETWORK_HOST_H

#if defined(ENABLE_LOCAL_DEBUG)
#define HOST_DEBUG
#endif

#include <Stream.h>
#include <Client.h>
#include <Udp.h>
#include "SerialNetworkProtocol.h"

// Include AsyncTCP & FreeRTOS if enabled
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
#include <AsyncTCP.h>
#include <freertos/queue.h>
#endif

using namespace SerialNetworkProtocol;

#include "include/HostNetwork.h"
#include "include/HostReboot.h"
#include "include/HostNVS.h"

#define MAX_FILENAME_LEN 64

// State Structures
struct WsSlotState
{
    bool is_connected = false;
};

struct UdpSlotState
{
    IPAddress remoteIP;
    uint16_t remotePort = 0;
    size_t packetSize = 0;
    size_t readPos = 0;
    uint8_t *packetData = nullptr;

    // For TX reassembly
    uint8_t *txBuffer = nullptr;
    size_t txLen = 0;
    uint16_t txTargetPort = 0;

    // To store the address payload from CMD_C_UDP_BEGIN_PACKET
    uint8_t targetAddrPayload[MAX_PACKET_BUFFER_SIZE];
    size_t targetAddrPayloadLen = 0;
};

// Queue Classes for Sync Clients
struct QueueNode
{
    uint8_t data;
    QueueNode *next;
};

class DynamicQueue
{
private:
    QueueNode *head = nullptr;
    QueueNode *tail = nullptr;
    size_t _count = 0;
    size_t _limit = SERIAL_TCP_HOST_TX_BUFFER_SIZE;

public:
    ~DynamicQueue() { clear(); }
    size_t available() { return _count; }
    size_t space() { return (_count >= _limit) ? 0 : _limit - _count; }

    // Helper to increase limit for AsyncTCP burst handling
    void setLimit(size_t new_limit) { _limit = new_limit; }

    bool enqueue(uint8_t b)
    {
        if (space() == 0)
            return false;
        QueueNode *newNode = new QueueNode;
        if (!newNode)
            return false;
        newNode->data = b;
        newNode->next = nullptr;
        if (tail)
            tail->next = newNode;
        else
            head = newNode;
        tail = newNode;
        _count++;
        return true;
    }

    int dequeue()
    {
        if (!head)
            return -1;
        QueueNode *temp = head;
        int data = temp->data;
        head = head->next;
        if (!head)
            tail = nullptr;
        delete temp;
        _count--;
        return data;
    }

    void clear()
    {
        while (dequeue() != -1)
            ;
    }
};

// Event Structure for Queueing
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
enum AsyncEventType
{
    AE_CONNECT,
    AE_DISCONNECT,
    AE_DATA,
    AE_ACK,
    AE_ERROR,
    AE_TIMEOUT
};

struct AsyncEvent
{
    AsyncEventType type;
    int slot;
    uint8_t *data; // Heap allocated buffer
    size_t len;
    uint32_t extra1;
    uint32_t extra2;
};
#endif

// Callbacks
typedef bool (*StartTLSCallback)(int slot);
typedef bool (*SetWiFiCallback)(const char *ssid, const char *pass);
typedef bool (*ConnectNetworkCallback)();
typedef void (*RebootCallback)();
typedef void (*PreConnectCallback)(int slot, const char *ca_cert_filename);
typedef void (*SetSSLCertificateCallback)(int slot, const char *ca_cert_filename);
typedef bool (*WsCommandCallback)(int slot, uint8_t cmd, const uint8_t *payload, size_t len);
typedef void (*SetFlagCallback)(int slot, uint8_t flag);
typedef uint8_t (*GetFlagCallback)(int slot);
typedef void (*SetBufferSizeCallback)(int slot, int rx, int tx);


class SerialNetworkHost
{
private:
    Stream *sink = nullptr;

    // Network Clients
    Client *_tcp_clients[MAX_SLOTS] = {nullptr};
    UDP *_udp_clients[MAX_SLOTS] = {nullptr};
    void *_ws_clients[MAX_SLOTS] = {nullptr};

    // Array for Async Clients
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
    AsyncClient *_async_clients[MAX_SLOTS] = {nullptr};
    QueueHandle_t _async_queue = NULL;
    // Track pending disconnects to synchronize with data stream
    bool _disconnect_pending[MAX_SLOTS] = {false};
#endif

    // Connection States
    bool _tcp_connected_state[MAX_SLOTS] = {false};
    uint8_t _flag[MAX_SLOTS] = {0};
    UdpSlotState _udp_states[MAX_SLOTS];
    WsSlotState _ws_states[MAX_SLOTS];

    PacketReceiver _receiver;
    uint8_t _decoded_buffer[MAX_PACKET_BUFFER_SIZE];

    uint8_t _debug_level = 1;
    uint16_t _session_id = 0;

    StartTLSCallback _tls_callbacks[MAX_SLOTS] = {nullptr};
    SetWiFiCallback _set_wifi_callback = nullptr;
    ConnectNetworkCallback _connect_net_callback = nullptr;
    RebootCallback _reboot_callback = nullptr;
    SetSSLCertificateCallback _set_ssl_certificate_callback = nullptr;
    WsCommandCallback _ws_command_callback = nullptr;
    SetFlagCallback _set_flag_callback = nullptr;
    GetFlagCallback _get_flag_callback = nullptr;
    SetBufferSizeCallback _set_buffer_size_callback = nullptr;

    char _ca_cert_filenames[MAX_SLOTS][MAX_FILENAME_LEN];

    DynamicQueue _tx_queues[MAX_SLOTS];
    enum class TxState
    {
        IDLE,
        WAIT_FOR_ACK
    };
    TxState _tx_states[MAX_SLOTS] = {TxState::IDLE};

    uint32_t _last_data_send_time[MAX_SLOTS] = {0};
    uint8_t _last_data_packet[MAX_SLOTS][MAX_PACKET_BUFFER_SIZE];
    size_t _last_data_packet_len[MAX_SLOTS] = {0};

    void sendPacketInternal(uint8_t cmd, uint8_t slot, const uint8_t *payload, size_t len)
    {
        sendPacket(*sink, cmd, slot, payload, len);
    }

#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
    // Called from AsyncTCP Callback (LwIP Task)
    void queueAsyncEvent(AsyncEvent evt)
    {
        if (_async_queue)
        {
            // Queue depth is 64. If full, we drop to avoid hanging the network stack.
            // portMAX_DELAY should NOT be used in LwIP callbacks.
            if (xQueueSend(_async_queue, &evt, 0) != pdTRUE)
            {
#if defined(ENABLE_LOCAL_DEBUG)
                if (evt.type == AE_DATA)
                    ets_printf("[Host] Queue FULL! Dropping DATA (%u bytes)\n", evt.len);
                else
                    ets_printf("[Host] Queue FULL! Dropping Event %d\n", evt.type);
#endif
                if (evt.data)
                    free(evt.data); // Prevent Memory Leak
            }
        }
        else
        {
            if (evt.data)
                free(evt.data); // Queue not ready
        }
    }

    // Consumer: Called from loop() (Arduino Task)
    void processAsyncEvents()
    {
        if (!_async_queue)
            return;

        AsyncEvent evt;
        // Process multiple events to drain queue efficiently
        int count = 0;
        while (xQueueReceive(_async_queue, &evt, 0) == pdTRUE && count < 50)
        {
            count++;
            switch (evt.type)
            {
            case AE_DATA:
            {
                const uint8_t *d = evt.data;
#if defined(ENABLE_LOCAL_DEBUG)
                ets_printf("[Host] Buffering DATA: %u bytes\n", evt.len);
#endif

                // Push data to the standard TX buffer logic
                // This allows processDataQueues() to handle flow control (Stop-and-Wait)
                for (size_t i = 0; i < evt.len; i++)
                {
                    if (!_tx_queues[evt.slot].enqueue(d[i]))
                    {
#if defined(ENABLE_LOCAL_DEBUG)
                        ets_printf("[Host] TX Buffer Full! Dropped byte\n");
#endif
                        break;
                    }
                }
                if (evt.data)
                    free(evt.data);
                break;
            }
            case AE_CONNECT:
            {
                if (evt.data && evt.len == 8)
                {
                    uint8_t p[14];
                    uint16_t mss = (evt.extra1 >> 16) & 0xFFFF;
                    uint16_t rp = evt.extra1 & 0xFFFF;
                    uint16_t lp = (uint16_t)evt.extra2;

                    p[0] = mss >> 8;
                    p[1] = mss & 0xFF;
                    memcpy(&p[2], evt.data, 4); // RemoteIP
                    p[6] = rp >> 8;
                    p[7] = rp & 0xFF;
                    memcpy(&p[8], evt.data + 4, 4); // LocalIP
                    p[12] = lp >> 8;
                    p[13] = lp & 0xFF;

                    sendPacketInternal(CMD_H_ATC_CONNECTED, evt.slot, p, 14);
                }
                if (evt.data)
                    free(evt.data);
                // Clear any pending disconnect on new connection
                _disconnect_pending[evt.slot] = false;
                break;
            }
            case AE_DISCONNECT:
#if defined(ENABLE_LOCAL_DEBUG)
                ets_printf("[Host] Pending DISCONNECT set for slot %d\n", evt.slot);
#endif
                // [CRITICAL] Don't send yet! Wait until buffer is empty.
                _disconnect_pending[evt.slot] = true;
                break;
            case AE_ACK:
            {
                uint8_t p[6];
                p[0] = (evt.len >> 8) & 0xFF;
                p[1] = evt.len & 0xFF;
                p[2] = (evt.extra1 >> 24) & 0xFF;
                p[3] = (evt.extra1 >> 16) & 0xFF;
                p[4] = (evt.extra1 >> 8) & 0xFF;
                p[5] = evt.extra1 & 0xFF;
                sendPacketInternal(CMD_H_ATC_ACKED, evt.slot, p, 6);
                break;
            }
            case AE_ERROR:
            {
                int8_t err = (int8_t)evt.extra1;
                sendPacketInternal(CMD_H_ATC_ERROR, evt.slot, (uint8_t *)&err, 1);
                break;
            }
            case AE_TIMEOUT:
            {
                uint32_t t = evt.extra1;
                uint8_t p[4] = {(uint8_t)(t >> 24), (uint8_t)(t >> 16), (uint8_t)(t >> 8), (uint8_t)t};
                sendPacketInternal(CMD_H_ATC_TIMEOUT, evt.slot, p, 4);
                break;
            }
            }
        }
    }
#endif

    void processCommand(const uint8_t *pkt, size_t len)
    {
        uint8_t cmd = pkt[0];
        uint8_t slot = pkt[1];
        const uint8_t *payload = &pkt[2];
        size_t payload_len = len - 4;

        // [CRITICAL] Global ACK Handling for Flow Control
        // This MUST be processed before checking specific client slots.
        // This guarantees that ANY client (Sync or Async) can clear the WAIT state.
        if (cmd == CMD_C_DATA_ACK)
        {
            if (slot < MAX_SLOTS && _tx_states[slot] == TxState::WAIT_FOR_ACK)
            {
                _tx_states[slot] = TxState::IDLE;
#if defined(ENABLE_LOCAL_DEBUG)
                // DEBUG_PRINT(_debug_level, "[Host]", "ACK Received");
#endif
            }
            return;
        }

        // Global Command Handling
        if (slot == GLOBAL_SLOT_ID)
        {
#if defined(ENABLE_LOCAL_DEBUG)
            char msg[32];
            snprintf(msg, sizeof(msg), "Got Global Command: %02X", cmd);
            DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif
            bool global_success = false;
            switch (cmd)
            {
            case CMD_C_SET_DEBUG:
                if (payload_len > 0)
                {
                    _debug_level = (uint8_t)payload[0];
                    global_success = true;
                }
                break;
            case CMD_C_PING_HOST:
                sendPacketInternal(CMD_H_PING_RESPONSE, GLOBAL_SLOT_ID, nullptr, 0);
                return;
            case CMD_C_REBOOT_HOST:
                global_success = true;
                sendPacketInternal(CMD_H_ACK, GLOBAL_SLOT_ID, nullptr, 0);
                if (_reboot_callback)
                {
                    delay(100);
                    _reboot_callback();
                }
                return;
            case CMD_C_SET_WIFI:
                if (_set_wifi_callback && payload_len > 2)
                {
                    uint8_t ssid_len = payload[0];
                    if (ssid_len > 0 && payload_len > ssid_len + 1UL)
                    {
                        const char *ssid = (const char *)&payload[1];
                        uint8_t pass_len = payload[1 + ssid_len];
                        if (payload_len >= 2UL + ssid_len + pass_len)
                        {
                            const char *pass = (const char *)&payload[2 + ssid_len];
                            char ssid_buf[ssid_len + 1];
                            memcpy(ssid_buf, ssid, ssid_len);
                            ssid_buf[ssid_len] = '\0';
                            char pass_buf[pass_len + 1];
                            memcpy(pass_buf, pass, pass_len);
                            pass_buf[pass_len] = '\0';
                            global_success = _set_wifi_callback(ssid_buf, pass_buf);
                        }
                    }
                }
                break;
            case CMD_C_CONNECT_NET:
                if (_connect_net_callback)
                    global_success = _connect_net_callback();
                break;
            case CMD_C_IS_NET_CONNECTED:
                global_success = false;

// check WiFi
#if defined(__has_wifi)
                if (WiFi.status() == WL_CONNECTED)
                {
                    global_success = true;
                }
#endif

// check Ethernet
#if defined(__has_ethernet)
// Standard Arduino Ethernet (W5100/W5500)
#if defined(Ethernet_h)
                // Note: linkStatus() requires W5500 or W5100 with modern lib
                if (Ethernet.linkStatus() == LinkON)
                {
                    global_success = true;
                }
#endif

// ESP32 Native Ethernet
#if defined(ETH_H)
                if (ETH.linkUp())
                {
                    global_success = true;
                }
#endif
#endif
                break;
            case CMD_C_DISCONNECT_NET:
                global_success = false; // Default to false

// WiFi Disconnect
#if defined(__has_wifi)
                if (WiFi.status() != WL_NO_SHIELD)
                {
                    WiFi.disconnect();
                    global_success = true;
                }
#endif

// Ethernet Disconnect (No-Op)
#if defined(__has_ethernet)
// Ethernet cannot be "disconnected" via software.
#if defined(Ethernet_h)
                if (Ethernet.hardwareStatus() != EthernetNoHardware)
                {
                    global_success = true;
                }
#endif
#if defined(ETH_H)
                global_success = true;
#endif
#endif
                break;
            case CMD_C_DEBUG_INFO:
                global_success = true;
                Serial.print((const char *)payload);
                return;
            }
            sendPacketInternal(global_success ? CMD_H_ACK : CMD_H_NAK, GLOBAL_SLOT_ID, nullptr, 0);
            return;
        }

        if (slot >= MAX_SLOTS)
        {
            sendPacketInternal(CMD_H_NAK, slot, nullptr, 0);
            return;
        }

        bool success = false;

        // Handle Async Client Commands
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
        if (_async_clients[slot])
        {
            AsyncClient *aClient = _async_clients[slot];
            switch (cmd)
            {
            case CMD_C_ATC_CONNECT:
            {
                if (payload_len < 3)
                    break;
                uint16_t port = (uint16_t)(payload[0] << 8) | payload[1];
                uint8_t host_len = payload[2];
                if (payload_len < 3 + host_len)
                    break;
                char host_name[host_len + 1];
                memcpy(host_name, &payload[3], host_len);
                host_name[host_len] = '\0';

#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Host]", String("Async Connect: ") + host_name + ":" + port);
#endif

                if (!aClient->connect(host_name, port))
                {
                    int8_t err = -1;
                    sendPacketInternal(CMD_H_ATC_ERROR, slot, (uint8_t *)&err, 1);
                }
                return;
            }
            case CMD_C_ATC_CLOSE:
                aClient->close();
                return;
            case CMD_C_ATC_ABORT:
                aClient->abort();
                return;
            case CMD_C_ATC_ADD:
                aClient->add((const char *)payload, payload_len);
                return;
            case CMD_C_ATC_SEND:
                aClient->send();
                return;
            case CMD_C_ATC_ACK:
            {
                // Client manual ACK (e.g. window update), separate from Protocol ACK
                if (payload_len >= 2)
                {
                    size_t ack_len = (size_t)((payload[0] << 8) | payload[1]);
                    aClient->ack(ack_len);
                }
                return;
            }
            case CMD_C_ATC_SET_RX_TIMEOUT:
                if (payload_len >= 4)
                {
                    uint32_t t = (uint32_t)((payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]);
                    aClient->setRxTimeout(t);
                }
                return;
            case CMD_C_ATC_SET_ACK_TIMEOUT:
                if (payload_len >= 4)
                {
                    uint32_t t = (uint32_t)((payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]);
                    aClient->setAckTimeout(t);
                }
                return;
            case CMD_C_ATC_SET_NO_DELAY:
                if (payload_len >= 1)
                    aClient->setNoDelay((bool)payload[0]);
                return;
            case CMD_C_ATC_SET_KEEP_ALIVE:
                if (payload_len >= 5)
                {
                    uint32_t t = (uint32_t)((payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]);
                    aClient->setKeepAlive(t, payload[4]);
                }
                return;
            default:
                return; // Handle other commands or ignore
            }
            return;
        }
#endif

        // WebSocket Command Handling
        if (cmd >= CMD_C_WS_CONNECT && cmd <= CMD_C_WS_LOOP)
        {
#if defined(ENABLE_LOCAL_DEBUG)
            if (cmd != CMD_C_WS_LOOP)
            {
                char msg[64];
                snprintf(msg, sizeof(msg), "Got WS Command: %02X for slot %d", cmd, slot);
                DEBUG_PRINT(_debug_level, "[Host]", msg);
            }
#endif
            if (_ws_command_callback)
            {
                success = _ws_command_callback(slot, cmd, payload, payload_len);
            }

            if (cmd != CMD_C_WS_LOOP)
            {
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Host]", success ? "Sending ACK" : "Sending NAK");
#endif
                sendPacketInternal(success ? CMD_H_ACK : CMD_H_NAK, slot, nullptr, 0);
            }
            return;
        }

        // UDP Command Handling
        if (cmd >= CMD_C_UDP_BEGIN && cmd <= CMD_C_UDP_PARSE_PACKET)
        {
            UDP *udpClient = _udp_clients[slot];
            if (!udpClient)
            {
#if defined(ENABLE_LOCAL_DEBUG)
                DEBUG_PRINT(_debug_level, "[Host]", "Error: No UDP client in slot");
#endif
                sendPacketInternal(CMD_H_NAK, slot, nullptr, 0);
                return;
            }

#if defined(ENABLE_LOCAL_DEBUG)
            char msg[64];
            snprintf(msg, sizeof(msg), "Got UDP Command: %02X for slot %d", cmd, slot);
            DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif

            switch (cmd)
            {
            case CMD_C_UDP_BEGIN:
            {
                if (payload_len < 2)
                    break;
                uint16_t localPort = (uint16_t)(payload[0] << 8) | payload[1];
                if (udpClient->begin(localPort) == 1)
                    success = true;
                break;
            }
            case CMD_C_UDP_END:
            {
                udpClient->stop();
                if (_udp_states[slot].packetData)
                {
                    delete[] _udp_states[slot].packetData;
                    _udp_states[slot].packetData = nullptr;
                }
                if (_udp_states[slot].txBuffer)
                {
                    delete[] _udp_states[slot].txBuffer;
                    _udp_states[slot].txBuffer = nullptr;
                }
                _udp_states[slot].packetSize = 0;

                // [FIX] Also close the TCP client if it's connected on this slot
                if (_tcp_clients[slot] && _tcp_clients[slot]->connected())
                {
                    _tcp_clients[slot]->stop();
                    _tcp_connected_state[slot] = false;
                }
                // Clear queues as well to be safe
                _tx_queues[slot].clear();
                _tx_states[slot] = TxState::IDLE;

                success = true;
                break;
            }
            case CMD_C_UDP_BEGIN_PACKET:
            {
                if (payload_len < 3)
                    break;
                uint16_t port = (uint16_t)(payload[0] << 8) | payload[1];

                if (_udp_states[slot].txBuffer)
                {
                    delete[] _udp_states[slot].txBuffer;
                }
                _udp_states[slot].txBuffer = new uint8_t[SERIAL_UDP_RX_BUFFER_SIZE];
                _udp_states[slot].txLen = 0;
                _udp_states[slot].txTargetPort = port;

                if (_udp_states[slot].txBuffer && payload_len <= MAX_PACKET_BUFFER_SIZE)
                {
                    memcpy(_udp_states[slot].targetAddrPayload, payload, payload_len);
                    _udp_states[slot].targetAddrPayloadLen = payload_len;
                    success = true;
                }
                else
                {
                    if (_udp_states[slot].txBuffer)
                    {
                        delete[] _udp_states[slot].txBuffer;
                    }
                    _udp_states[slot].txBuffer = nullptr;
                    success = false;
                }
                break;
            }
            case CMD_C_UDP_WRITE_DATA:
            {
                if (!_udp_states[slot].txBuffer || _udp_states[slot].txLen + payload_len > SERIAL_UDP_RX_BUFFER_SIZE)
                    break;
                memcpy(&_udp_states[slot].txBuffer[_udp_states[slot].txLen], payload, payload_len);
                _udp_states[slot].txLen += payload_len;
                success = true;
                break;
            }
            case CMD_C_UDP_END_PACKET:
            {
                uint8_t *addr_payload = _udp_states[slot].targetAddrPayload;
                size_t addr_len = _udp_states[slot].targetAddrPayloadLen;
                uint16_t port = _udp_states[slot].txTargetPort;
                uint8_t type = (addr_len > 2) ? addr_payload[2] : 0xFF;

                bool packet_started = false;
                if (type == 0 && addr_len == 7)
                {
                    IPAddress targetIP(addr_payload[3], addr_payload[4], addr_payload[5], addr_payload[6]);
                    packet_started = udpClient->beginPacket(targetIP, port);
                }
                else if (type == 1 && addr_len > 4)
                {
                    uint8_t host_len = addr_payload[3];
                    char host_name[host_len + 1];
                    memcpy(host_name, &addr_payload[4], host_len);
                    host_name[host_len] = '\0';
                    packet_started = udpClient->beginPacket(host_name, port);
                }

                if (packet_started && _udp_states[slot].txBuffer)
                {
                    size_t written = udpClient->write(_udp_states[slot].txBuffer, _udp_states[slot].txLen);
                    if (written == _udp_states[slot].txLen)
                        success = udpClient->endPacket();
                }

                if (_udp_states[slot].txBuffer)
                    delete[] _udp_states[slot].txBuffer;
                _udp_states[slot].txBuffer = nullptr;
                _udp_states[slot].txLen = 0;
                break;
            }
            case CMD_C_UDP_PARSE_PACKET:
            {
                int packetSize = udpClient->parsePacket();
                if (packetSize > 0)
                {
                    if (_udp_states[slot].packetData)
                    {
                        delete[] _udp_states[slot].packetData;
                    }
                    size_t buf_size = min((size_t)packetSize, (size_t)SERIAL_UDP_RX_BUFFER_SIZE);
                    _udp_states[slot].packetData = new uint8_t[buf_size];

                    if (_udp_states[slot].packetData)
                    {
                        size_t readLen = udpClient->read(_udp_states[slot].packetData, buf_size);
                        if (readLen > 0)
                        {
                            _udp_states[slot].packetSize = readLen;
                            _udp_states[slot].readPos = 0;
                            _udp_states[slot].remoteIP = udpClient->remoteIP();
                            _udp_states[slot].remotePort = udpClient->remotePort();

                            IPAddress rip = udpClient->remoteIP();
                            uint16_t rp = udpClient->remotePort();
                            uint8_t rpld[8] = {
                                (uint8_t)rip[0], (uint8_t)rip[1], (uint8_t)rip[2], (uint8_t)rip[3],
                                (uint8_t)(rp >> 8), (uint8_t)(rp & 0xFF),
                                (uint8_t)(readLen >> 8), (uint8_t)(readLen & 0xFF)};

                            sendPacketInternal(CMD_H_UDP_PACKET_INFO, slot, rpld, 8);
                            sendPacketInternal(CMD_H_UDP_DATA_PAYLOAD, slot, _udp_states[slot].packetData, readLen);

                            delete[] _udp_states[slot].packetData;
                            _udp_states[slot].packetData = nullptr;
                        }
                        else
                        {
                            delete[] _udp_states[slot].packetData;
                            _udp_states[slot].packetData = nullptr;
                            _udp_states[slot].packetSize = 0;
                        }
                    }
                }
                success = true;
            }
            }
#if defined(ENABLE_LOCAL_DEBUG)
            DEBUG_PRINT(_debug_level, "[Host]", success ? "Sending ACK" : "Sending NAK");
#endif
            sendPacketInternal(success ? CMD_H_ACK : CMD_H_NAK, slot, nullptr, 0);
            return;
        }

        // TCP Command Handling
        Client *client = _tcp_clients[slot];

        if (!client && cmd != CMD_C_DATA_ACK && cmd != CMD_C_START_TLS)
        {
#if defined(ENABLE_LOCAL_DEBUG)
            char msg[32];
            snprintf(msg, sizeof(msg), "Error: No client in slot %d", slot);
            DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif
            sendPacketInternal(CMD_H_NAK, slot, nullptr, 0);
            return;
        }

#if defined(ENABLE_LOCAL_DEBUG)
        char msg[64];
        snprintf(msg, sizeof(msg), "Got Command: %02X for slot %d", cmd, slot);
        DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif

        switch (cmd)
        {
        case CMD_C_CONNECT_HOST:
        {
            if (payload_len < 5)
                break;
            uint16_t port = (uint16_t)(payload[1] << 8) | payload[2];
            uint8_t host_len = payload[3];
            if (host_len > payload_len - 4)
                break;
            char host_name[host_len + 1];
            memcpy(host_name, &payload[4], host_len);
            host_name[host_len] = '\0';

            // Stop existing connection first
            if (client && client->connected())
            {
                client->stop();
            }

            if (_set_ssl_certificate_callback && _ca_cert_filenames[slot][0] != '\0')
            {
                _set_ssl_certificate_callback(slot, _ca_cert_filenames[slot]);
            }
            if (client->connect(host_name, port))
            {
                success = true;
                _tcp_connected_state[slot] = true;
                _tx_queues[slot].clear();
                _tx_states[slot] = TxState::IDLE;
            }
            _ca_cert_filenames[slot][0] = '\0';
            break;
        }
        case CMD_C_WRITE:
            if (client && client->connected())
            {
                if (client->write(payload, payload_len) == payload_len)
                {
                    client->flush();
                    success = true;
                }
            }
            break;
        case CMD_C_STOP:
            if (client)
                client->stop();
            _tcp_connected_state[slot] = false;
            _tx_queues[slot].clear();
            _tx_states[slot] = TxState::IDLE;
            _ca_cert_filenames[slot][0] = '\0';
            success = true;
            break;
        case CMD_C_POLL_DATA:
        {
            if (payload_len >= 2)
            {
                uint16_t client_sid = (uint16_t)(payload[0] << 8) | payload[1];
                if (client_sid != 0 && client_sid != _session_id)
                {
                    uint8_t reset_pld[2];
                    reset_pld[0] = (uint8_t)(_session_id >> 8);
                    reset_pld[1] = (uint8_t)(_session_id & 0xFF);
                    sendPacketInternal(CMD_H_HOST_RESET, GLOBAL_SLOT_ID, reset_pld, 2);
                    return;
                }
            }
            bool is_connected = (client ? (uint8_t)client->connected() : 0);
            size_t pld_len = 3;
            uint8_t response_pld[pld_len];
            response_pld[0] = (uint8_t)is_connected;
            response_pld[1] = 0;
            response_pld[2] = 0;
            sendPacketInternal(CMD_H_POLL_RESPONSE, slot, response_pld, pld_len);
            return;
        }
        case CMD_C_IS_CONNECTED:
        {
            uint8_t status = (client ? (uint8_t)client->connected() : 0);
            _tcp_connected_state[slot] = status;
            sendPacketInternal(CMD_H_CONNECTED_STATUS, slot, &status, 1);
            return;
        }
        case CMD_C_DATA_ACK:
            if (_tx_states[slot] == TxState::WAIT_FOR_ACK)
                _tx_states[slot] = TxState::IDLE;
            return;
        case CMD_C_START_TLS:
            if (_tls_callbacks[slot] != nullptr)
                success = _tls_callbacks[slot](slot);
            break;
        case CMD_C_SET_CA_CERT:
            if (payload_len > 0 && payload_len < MAX_FILENAME_LEN)
            {
                memcpy(_ca_cert_filenames[slot], payload, payload_len);
                _ca_cert_filenames[slot][payload_len] = '\0';
                success = true;
            }
            break;
        case CMD_C_GET_FLAG:
            if (_get_flag_callback)
            {
                uint8_t current_flag = _get_flag_callback(slot);
                _flag[slot] = current_flag;
            }
            sendPacketInternal(CMD_H_FLAG_RESPONSE, slot, &_flag[slot], 1);
            return;

        case CMD_C_SET_FLAG:
            if (payload_len >= 1)
            {
                _flag[slot] = payload[0];
                if (_set_flag_callback)
                    _set_flag_callback(slot, _flag[slot]);
                success = true;
            }
            break;
        case CMD_C_SET_BUF_SIZE:
            if (payload_len >= 4)
            {
                int rx = (payload[0] << 8) | payload[1];
                int tx = (payload[2] << 8) | payload[3];
                if (_set_buffer_size_callback)
                    _set_buffer_size_callback(slot, rx, tx);
                success = true;
            }
            break;
        }
#if defined(ENABLE_LOCAL_DEBUG)
        DEBUG_PRINT(_debug_level, "[Host]", success ? "Sending ACK" : "Sending NAK");
#endif
        sendPacketInternal(success ? CMD_H_ACK : CMD_H_NAK, slot, nullptr, 0);
    }

    void processSerial()
    {
        while (sink->available())
        {
            uint8_t b = sink->read();
            size_t cobsLen = _receiver.read_byte(b);
            if (cobsLen > 0)
            {
                size_t len = cobs_decode(_receiver.buffer, cobsLen, _decoded_buffer);
                if (len > 2)
                {
                    uint16_t crc = calculate_crc16(_decoded_buffer, len - 2);
                    uint16_t r_crc = (uint16_t)(_decoded_buffer[len - 1] << 8 | _decoded_buffer[len - 2]);
                    if (crc == r_crc)
                    {
                        processCommand(_decoded_buffer, len);
                    }
                    else
                    {
#if defined(ENABLE_LOCAL_DEBUG)
                        DEBUG_PRINT(_debug_level, "[Host]", "Error: Bad CRC!");
#endif
                    }
                }
                _receiver.pos = 0;
            }
        }
    }

    void queueNetworkData()
    {
        for (int i = 0; i < MAX_SLOTS; i++)
        {
            // [ASYNC-TCP] Skip slots managed by AsyncTCP (callbacks handle data)
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
            if (_async_clients[i])
                continue;
#endif

            if (_tcp_clients[i])
            {
                while (_tcp_clients[i]->available() > 0 && _tx_queues[i].space() > 0)
                {
                    _tx_queues[i].enqueue(_tcp_clients[i]->read());
                }

                bool is_connected = _tcp_clients[i]->connected();

                if (is_connected != _tcp_connected_state[i])
                {
                    if (is_connected)
                    {
#if defined(ENABLE_LOCAL_DEBUG)
                        char msg[50];
                        snprintf(msg, sizeof(msg), "Pushing connect status (1) to slot %d", i);
                        DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif
                        _tcp_connected_state[i] = is_connected;
                        uint8_t status = (uint8_t)is_connected;
                        sendPacketInternal(CMD_H_CONNECTED_STATUS, i, &status, 1);
                    }
                    else
                    {
                        while (_tcp_clients[i]->available() > 0 && _tx_queues[i].space() > 0)
                        {
                            _tx_queues[i].enqueue(_tcp_clients[i]->read());
                        }

                        if (_tx_queues[i].available() == 0 && _tx_states[i] == TxState::IDLE)
                        {
#if defined(ENABLE_LOCAL_DEBUG)
                            char msg[50];
                            snprintf(msg, sizeof(msg), "Queue empty. Pushing disconnect to slot %d", i);
                            DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif
                            _tcp_connected_state[i] = is_connected;
                            _ca_cert_filenames[i][0] = '\0';
                            uint8_t status = (uint8_t)is_connected;
                            sendPacketInternal(CMD_H_CONNECTED_STATUS, i, &status, 1);
                        }
                    }
                }
            }
        }
    }

    void processDataQueues()
    {
        for (int i = 0; i < MAX_SLOTS; i++)
        {
            // Check both Sync and Async clients
            bool has_client = (_tcp_clients[i] != nullptr);
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
            if (_async_clients[i])
                has_client = true;
#endif
            if (!has_client)
                continue;

            if (_tx_states[i] == TxState::WAIT_FOR_ACK)
            {
                if (millis() - _last_data_send_time[i] > SERIAL_TCP_DATA_PACKET_TIMEOUT)
                {
#if defined(ENABLE_LOCAL_DEBUG)
                    char msg[50];
                    snprintf(msg, sizeof(msg), "Timeout, resending data to slot %d", i);
                    DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif

                    size_t rawLen = 2 + _last_data_packet_len[i];
                    uint16_t crc = calculate_crc16(_last_data_packet[i], rawLen);
                    _last_data_packet[i][rawLen] = (uint8_t)(crc & 0xFF);
                    _last_data_packet[i][rawLen + 1] = (uint8_t)(crc >> 8);
                    rawLen += 2;

                    uint8_t cobsPkt[MAX_PACKET_BUFFER_SIZE + 2];
                    size_t cobsLen = cobs_encode(_last_data_packet[i], rawLen, cobsPkt);

                    sink->write(cobsPkt, cobsLen);
                    sink->write(FRAME_DELIMITER);

                    _last_data_send_time[i] = millis();
                }
                continue;
            }

            // Check pending Async Disconnect
            // Only send DISCONNECT if the data queue is completely empty AND we are IDLE.
            bool disconnect_now = false;
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
            if (_async_clients[i] && _tx_queues[i].available() == 0 && _disconnect_pending[i])
            {
                disconnect_now = true;
                _disconnect_pending[i] = false;
                sendPacketInternal(CMD_H_ATC_DISCONNECTED, i, nullptr, 0);
                continue;
            }
#endif

            if (_tx_states[i] == TxState::IDLE && _tx_queues[i].available() > 0)
            {
                // Limit chunk size to prevent overflow on client RX buffer.
                size_t len_to_send = min(_tx_queues[i].available(), (size_t)SERIAL_TCP_DATA_PAYLOAD_SIZE);

                _last_data_packet_len[i] = len_to_send;
                _last_data_packet[i][0] = CMD_H_DATA_PAYLOAD;

#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
                if (_async_clients[i])
                    _last_data_packet[i][0] = CMD_H_ATC_DATA;
#endif
                _last_data_packet[i][1] = i;

                for (size_t j = 0; j < len_to_send; j++)
                {
                    int data = _tx_queues[i].dequeue();
                    if (data == -1)
                    {
                        len_to_send = j;
                        break;
                    }
                    _last_data_packet[i][j + 2] = (uint8_t)data;
                }

                if (len_to_send > 0)
                {
                    sendPacketInternal(_last_data_packet[i][0], i, &_last_data_packet[i][2], len_to_send);
                    _tx_states[i] = TxState::WAIT_FOR_ACK;
                    _last_data_send_time[i] = millis();
                }
            }
        }
    }

public:
    /**
     * @brief Constructor for SerialNetworkHost.
     * @param sink The Stream interface (e.g., Serial, Serial1) used for
     * communicating with the device running SerialTCPClient.
     */
    SerialNetworkHost(Stream &sink) : sink(&sink)
    {
        _session_id = (uint16_t)micros();
        if (_session_id == 0)
            _session_id = 1;

#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
        // Create Queue for Async Events (Depth 64)
        _async_queue = xQueueCreate(64, sizeof(AsyncEvent));
#endif

        for (int i = 0; i < MAX_SLOTS; i++)
        {
            _ca_cert_filenames[i][0] = '\0';
            _udp_states[i].packetData = nullptr;
            _udp_states[i].txBuffer = nullptr;

            // [CRITICAL] Increase Buffer to 4KB for ESP32 to absorb TCP bursts
#if defined(ESP32)
            _tx_queues[i].setLimit(4096);
#endif
        }
    }

    /**
     * @brief Destructor to clean up dynamic memory used by UDP state.
     */
    ~SerialNetworkHost()
    {
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
        if (_async_queue)
        {
            AsyncEvent evt;
            while (xQueueReceive(_async_queue, &evt, 0) == pdTRUE)
            {
                if (evt.data)
                    free(evt.data);
            }
            vQueueDelete(_async_queue);
        }
#endif
        for (int i = 0; i < MAX_SLOTS; i++)
        {
            if (_udp_states[i].packetData)
                delete[] _udp_states[i].packetData;
            if (_udp_states[i].txBuffer)
                delete[] _udp_states[i].txBuffer;
        }
    }

    /**
     * @brief Broadcasts a HOST_RESET command to all clients.
     * This function should be called in the Host's setup() function
     * after Serial initialization. It sends a packet containing the
     * Host's current Session ID. Any connected clients receiving this
     * will know the host has rebooted and will reset their connection state.
     */
    void notifyBoot()
    {
#if defined(ENABLE_LOCAL_DEBUG)
        char msg[50];
        snprintf(msg, sizeof(msg), "Broadcasting HOST_RESET. Session: %04X", _session_id);
        DEBUG_PRINT(_debug_level, "[Host]", msg);
#endif
        uint8_t payload[2];
        payload[0] = (uint8_t)(_session_id >> 8);
        payload[1] = (uint8_t)(_session_id & 0xFF);
        sendPacketInternal(CMD_H_HOST_RESET, GLOBAL_SLOT_ID, payload, 2);
        delay(50);
    }

    /**
     * @brief Sets the TCP Client object for a specific slot.
     * @param client Pointer to the Client object (e.g., WiFiClient).
     * @param slot The client slot (0 to MAX_SLOTS-1).
     */
    void setTCPClient(Client *client, int slot)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
        {
            _tcp_clients[slot] = client;
            _tcp_connected_state[slot] = false;
        }
    }

    // [ASYNC-TCP] Register Async Client
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
    /**
     * @brief Sets the AsyncTCP Client object for a specific slot.
     * @param client Pointer to the AsyncClient object.
     * @param slot The client slot (0 to MAX_SLOTS-1).
     */
    void setAsyncClient(AsyncClient *client, int slot)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
        {
            _async_clients[slot] = client;

            // Setup Callbacks - PUSH TO QUEUE
            client->onData([this, slot](void *, AsyncClient *c, void *data, size_t len)
                           {
                               if (len == 0)
                                   return;

#if defined(ENABLE_LOCAL_DEBUG)
                               ets_printf("[Host] Received data for slot %d (%u bytes)!\n", slot, len);
#endif

                               AsyncEvent evt;
                               evt.type = AE_DATA;
                               evt.slot = slot;
                               evt.len = len;
                               // Deep Copy Data to Heap for thread safety
                               evt.data = (uint8_t *)malloc(len);

                               if (evt.data)
                               {
                                   memcpy(evt.data, data, len);
                                   queueAsyncEvent(evt);
                               }
                               else
                               {
#if defined(ENABLE_LOCAL_DEBUG)
                                   ets_printf("[Host] Malloc Failed (%u bytes)!\n", len);
#endif
                               }
                               // AsyncTCP auto-acks. Flow control is handled by _tx_queues blocking send.
                               // To prevent stalling the TCP window, we rely on AsyncTCP's default behavior
                               // combined with our quick consumption into the queue.
                               c->ack(len); // Explicitly ack to keep window open for high-throughput
                           });

            client->onConnect([this, slot](void *, AsyncClient *c)
                              {
                c->setNoDelay(true); 

                AsyncEvent evt; 
                evt.type = AE_CONNECT; 
                evt.slot = slot;
                evt.extra1 = ((uint32_t)c->getMss() << 16) | c->getRemotePort();
                evt.extra2 = c->getLocalPort();
                
                evt.len = 8;
                evt.data = (uint8_t*)malloc(8);
                if (evt.data) {
                    uint32_t rip = c->getRemoteAddress();
                    uint32_t lip = c->getLocalAddress();
                    memcpy(evt.data, &rip, 4);
                    memcpy(evt.data+4, &lip, 4);
                    queueAsyncEvent(evt);
                } });

            client->onDisconnect([this, slot](void *, AsyncClient *c)
                                 {
                AsyncEvent evt; 
                evt.type = AE_DISCONNECT; 
                evt.slot = slot; 
                evt.data = nullptr; 
                queueAsyncEvent(evt); });

            client->onError([this, slot](void *, AsyncClient *c, int8_t error)
                            {
                AsyncEvent evt; 
                evt.type = AE_ERROR; 
                evt.slot = slot; 
                evt.extra1 = (uint32_t)error; 
                evt.data = nullptr;
                queueAsyncEvent(evt); });

            client->onAck([this, slot](void *, AsyncClient *c, size_t len, uint32_t time)
                          {
                AsyncEvent evt; 
                evt.type = AE_ACK; 
                evt.slot = slot; 
                evt.len = len; 
                evt.extra1 = time; 
                evt.data = nullptr;
                queueAsyncEvent(evt); });

            client->onTimeout([this, slot](void *, AsyncClient *c, uint32_t time)
                              {
                AsyncEvent evt; 
                evt.type = AE_TIMEOUT; 
                evt.slot = slot; 
                evt.extra1 = time; 
                evt.data = nullptr;
                queueAsyncEvent(evt); });
        }
    }
#endif

    /**
     * @brief Sets the UDP Client object for a specific slot.
     * @param udpClient Pointer to the UDP object (e.g., WiFiUDP).
     * @param slot The client slot (0 to MAX_SLOTS-1).
     */
    void setUDPClient(UDP *udpClient, int slot)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
        {
            _udp_clients[slot] = udpClient;
        }
    }

    /**
     * @brief Sets the WebSocket Client object for a specific slot.
     * @param wsClient Generic pointer to the native WebSocket Client object.
     * @param slot The client slot (0 to MAX_SLOTS-1).
     */
    void setWebSocketClient(void *wsClient, int slot)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
        {
            _ws_clients[slot] = wsClient;
            _ws_states[slot].is_connected = false;
        }
    }

    /**
     * @brief Registers the callback that handles actual WebSocket logic in the host sketch.
     * @param callback The WsCommandCallback function pointer.
     */
    void setWebSocketCallback(WsCommandCallback callback) { _ws_command_callback = callback; }

    /**
     * @brief Helper to push WS events from Host Sketch -> Serial.
     * Must be called by the Host Sketch when it receives a WS event.
     * @param slot The client slot (0 to MAX_SLOTS-1).
     * @param type The WSMessageType (e.g., WS_FRAME_TEXT).
     * @param payload Pointer to the data payload.
     * @param len Length of the payload.
     */
    void pushWsEvent(int slot, uint8_t type, const uint8_t *payload, size_t len)
    {
        size_t pl = 1 + len;
        uint8_t p[pl];
        p[0] = type;
        if (len > 0)
            memcpy(&p[1], payload, len);
        sendPacketInternal(CMD_H_WS_EVENT, slot, p, pl);
    }

    /**
     * @brief Sets the StartTLS callback for a specific slot.
     * @param slot The client slot (0 to MAX_SLOTS-1).
     * @param callback The StartTLSCallback function pointer.
     */
    void setStartTLSCallback(int slot, StartTLSCallback callback)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
        {
            _tls_callbacks[slot] = callback;
        }
    }

    /**
     * @brief Sets the SetWiFi callback.
     * @param callback The SetWiFiCallback function pointer.
     */
    void setSetWiFiCallback(SetWiFiCallback callback) { _set_wifi_callback = callback; }

    /**
     * @brief Sets the ConnectNetwork callback.
     * @param callback The ConnectNetworkCallback function pointer.
     */
    void setConnectNetworkCallback(ConnectNetworkCallback callback) { _connect_net_callback = callback; }

    /**
     * @brief Sets the Reboot callback.
     * @param callback The RebootCallback function pointer.
     */
    void setRebootCallback(RebootCallback callback) { _reboot_callback = callback; }

    /**
     * @brief Sets the SSL certificate setup callback.
     * @param callback The PreConnectCallback function pointer.
     */
    void setSetSSLCertificateCallback(SetSSLCertificateCallback callback) { _set_ssl_certificate_callback = callback; }

    /**
     * @brief Sets the callback for processing SET_FLAG requests from the client.
     * @param callback The SetFlagCallback function pointer.
     */
    void setSetFlagCallback(SetFlagCallback callback) { _set_flag_callback = callback; }

    /**
     * @brief Sets the callback for processing GET_FLAG requests from the client.
     * @param callback The GetFlagCallback function pointer.
     */
    void setGetFlagCallback(GetFlagCallback callback) { _get_flag_callback = callback; }

    /**
     * @brief Sets the callback for processing SET_BUF_SIZE requests from the client.
     * @param callback The SetBufferSizeCallback function pointer.
     */
    void setSetBufferSizeCallback(SetBufferSizeCallback callback) { _set_buffer_size_callback = callback; }

    /**
     * @brief Sets the local debug level for the host.
     * @param level The debug level (0 = none, higher = more verbose).
     */
    void setLocalDebugLevel(int level) { _debug_level = (uint8_t)level; }

    /**
     * @brief Gets the flag value for a specific slot.
     * @param slot The client slot (0 to MAX_SLOTS-1).
     * @return The 8-bit flag value.
     */
    uint8_t getFlag(int slot)
    {
        if (slot >= 0 && slot < MAX_SLOTS)
            return _flag[slot];
        return 0;
    }

    /**
     * @brief Main loop function to be called regularly.
     * Handles serial processing, network data queueing, and state management.
     */
    void loop()
    {
#if defined(ESP32) && defined(USE_ASYNC_CLIENT)
        processAsyncEvents();
#endif
        processSerial();
        queueNetworkData();
        processDataQueues();
    }
};

#endif