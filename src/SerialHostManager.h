/*
 * SPDX-FileCopyrightText: 2025 Suwatchai K. <suwatchai@outlook.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_HOST_MANAGER_H
#define SERIAL_HOST_MANAGER_H

#include "SerialTCPClient.h"

class SerialHostManager
{
    friend class SerialTCPClient;

public:
    SerialHostManager(Stream &sink)
        : sink(&sink) { _serial_tcp_client.setSerial(sink); }

    /**
     * @brief Sets the WiFi credentials on the host.
     */
    bool setWiFi(const char *ssid, const char *password)
    {
        return _serial_tcp_client.setWiFiImpl(ssid, password);
    }

    /**
     * @brief Commands the host to connect to the configured WiFi.
     */
    bool connectNetwork()
    {
        return _serial_tcp_client.sendCommand(CMD_C_CONNECT_NET, GLOBAL_SLOT_ID, nullptr, 0, true, SERIAL_TCP_CONNECT_TIMEOUT);
    }

    /**
     * @brief Commands the host to disconnect from WiFi.
     */
    bool disconnectNetwork()
    {
        return _serial_tcp_client.sendCommand(CMD_C_DISCONNECT_NET, GLOBAL_SLOT_ID, nullptr, 0, true);
    }

    /**
     * @brief Checks if the host is connected to WiFi.
     */
    bool isNetworkConnected()
    {
        return _serial_tcp_client.sendCommand(CMD_C_IS_NET_CONNECTED, GLOBAL_SLOT_ID, nullptr, 0, true);
    }

    /**
     * @brief Pings the host to check if it's alive.
     */
    bool pingHost(uint32_t timeout = 1000)
    {
        return _serial_tcp_client.sendCommand(CMD_C_PING_HOST, GLOBAL_SLOT_ID, nullptr, 0, true, timeout);
    }

    /**
     * @brief Commands the host to reboot.
     */
    bool rebootHost()
    {
        return _serial_tcp_client.sendCommand(CMD_C_REBOOT_HOST, GLOBAL_SLOT_ID, nullptr, 0, true);
    }

    /**
     * @brief Sets the debug level on the *host*.
     */
    bool setDebugLevel(int level)
    {
        return _serial_tcp_client.setDebugLevelImpl(level);
    }

    /**
     * @brief Sets the debug level only on the client.
     */
    void setLocalDebugLevel(int level)
    {
        _serial_tcp_client.setLocalDebugLevelImpl(level);
    }

private:
    Stream *sink = nullptr;
    SerialTCPClient _serial_tcp_client;
};
#endif