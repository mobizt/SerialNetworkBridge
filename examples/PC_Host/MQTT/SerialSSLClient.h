#ifndef __SERIAL_SSL_CLIENT_H__
#define __SERIAL_SSL_CLIENT_H__

#include <SerialNetworkBridge.h>

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

#endif