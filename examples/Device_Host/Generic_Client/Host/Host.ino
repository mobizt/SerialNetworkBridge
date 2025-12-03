/**
 * ===============================================
 * Device Host Serial Bridge (with NVS Storage)
 * ===============================================
 * Runs on: The host device (e.g., ESP8266, ESP32, Raspberry Pi Pico W).
 * Purpose: Operating the neccessary tasks on the host.
 *
 * Macros explanation
 *
 * MAX_CLIENT_SLOTS     Maximum slots (channel or session)
 *
 * ENABLE_LOCAL_DEBUG   The macro to allow local debug to show on Serial port.
 *
 * USE_UDP              The UDP client will be assigned to the host
 *
 * USE_WEBSOCKET        The Websocket client will be assigned to the host
 *
 * USE_ESP_SSLCLIENT    Use ESP_SSLClient for SSL client
 *
 * USE_LITTLEFS         Use LittleFS for local file storage filesystem
 *
 * WIFI_SSID            The default WiFi AP SSID
 *
 * WIFI_PASSWORD        The default WiFi AP Password
 *
 * BridgeRate           The bridge serial port baud rate.
 *                      Should be matched with client device serial baud rate.
 *                      Should not exceed 115200 for AVR
 *
 * BridgeSerial         The serial port that is connected to the client device serial port.
 * 
 * Predefined macros for network class (Optional)
 * 
 * __ssl_client         SSL client class that is defined in HostNetwork.h
 *                      WiFiClientSecure or WiFiSSLClient in WiFi capable device.
 *                      ESP_SSLClient for generic or non-WiFi capable device.
 * 
 * __udp_client         UDP client class that is defined in HostNetwork.h
 *                      WiFiUDP in WiFi capable device.
 *                      EthernetUDP for non-WiFi capable device.
 * 
 * __ssl_set_network_class  Defined in HostNetwork.h where ESP_SSLClient is used.
 *                          If this macro is defined, it means the network client (WiFi/Ethernet) 
 *                          is required to use with SSL client object.
 * 
 * __network_class      Defined in HostNetwork.h for network client class.
 *                      WiFiClient for WiFi capable device.
 *                      EthernetClient for non-WiFi capable device.
 *
 */

#define MAX_CLIENT_SLOTS 4
#define ENABLE_LOCAL_DEBUG
#define USE_UDP
#define USE_WEBSOCKET
#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "password"
#define BridgeRate 115200
#define BridgeSerial Serial2

// Optional for external WiFi and SSL client libraries
// #include <WiFiNINA.h>
// #include <ESP_SSLClient.h>

#include "Filesystem.h"

#include <SerialNetworkBridge.h>

// https://github.com/Links2004/arduinoWebSockets
// This websocket library can't be used with Teensy and Arduino NANO RP2040
// due to compilation error
#include <WebSocketsClient.h>

SerialNetworkHost host(BridgeSerial);


#if defined(__ssl_client)
__ssl_client ssl_client;
#endif


#if defined(__udp_client)
__udp_client udp_client;
#endif


#if defined(__ssl_set_network_class) && defined(__network_class)
__network_class network_client;
#endif

#if defined(USE_WEBSOCKET)
WebSocketsClient ws_client;
#endif

#include "Callback.h"

void setup()
{
    // Start local Serial for debugging
    Serial.begin(115200);
    delay(1000);

    BridgeSerial.begin(BridgeRate);

    Serial.println("\n[Host] SerialTCP Host Starting...");

    // Load Credentials from Storage
    loadWiFi(WIFI_SSID, WIFI_PASSWORD);

    // Handles WiFi credentials setup
    host.setSetWiFiCallback(handle_set_wifi);

    // Handles WiFi connection
    host.setConnectNetworkCallback(handle_connect_network);

    // Handles host reboot
    host.setRebootCallback(handle_reboot);

    // Callbacks pecific to SSL client in some platforms

    // Handles client side function setCACert() in ESP32's WiFiClientSecure and ESP_SSLClient,
    // setTrustAnchors in ESP8266/RPi Pico W's WiFiClientSecure.
    host.setSetSSLCertificateCallback(handle_certificate_setup);

    // Handles client side function setBufferSizes() in ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient.
    host.setSetBufferSizeCallback(handle_set_buffer_size);

    // Handles the client side function isSecure() in ESP32/ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient.
    host.setGetFlagCallback(handle_get_flag);

    // Handles the client side functions setInsecure() in ESP32/ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient
    // setPlainStart() in ESP32/ESP8266/RPi Pico W WiFiClientSecure and ESP_SSLClient,
    // enableSSL() in ESP_SSLClient.
    host.setSetFlagCallback(handle_set_flag);

    // Handles client side function StartTLS() in ESP32' WiFiClientSecure and connectSSL() in ESP_SSLClient.
    host.setStartTLSCallback(0 /* slot */, handle_start_tls);

#if defined(__ssl_set_network_class) && defined(__network_class)
    ssl_client.setClient(&network_client);
#endif

    // Assign clients for each slot
    // In this example, slot 0 is for TCP client, slot 1 is for UDP and slot 2 is for websocket.

    host.setTCPClient(&ssl_client, 0 /* slot */);

#if defined(__udp_client)
    host.setUDPClient(&udp_client, 1 /* slot */);
#endif

#if defined(USE_WEBSOCKET)
    host.setWebSocketClient(&ws_client, 2 /* slot */);
#endif

    host.setLocalDebugLevel(1);

    // Connect to WiFi
    handle_connect_network();

    // Notify the client that host is rebooted/ready.
    // This resets the client's internal state for a fresh session.
    host.notifyBoot();

    Serial.println("Device Host Serial Bridge started.");
}

void loop()
{
    // Reqouirements for Host operation
    host.loop();
}