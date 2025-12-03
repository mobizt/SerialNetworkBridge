#ifndef __CALLBACK_H__
#define __CALLBACK_H__

#include <Arduino.h>
#include "Filesystem.h"
#include <SerialNetworkBridge.h>

// Handles host device set WiFi
static bool handle_set_wifi(const char *ssid, const char *password)
{
    Serial.print("[Host] Setting WiFi credentials for: ");
    Serial.println(ssid);
#if defined(__has_wifi)

    saveWiFi(ssid, password);
    WiFi.disconnect();
    return true;

#elif defined(__has_ethernet)
    Serial.println("[Host] Board does not support WiFi. The ethernet connection is used instead.");
    return true;
#endif
    Serial.println("[Host] Board does not support WiFi and ethernet connections.");
    return false;
}

// Handles host device connect WiFi
static bool handle_connect_network()
{
#if defined(__has_wifi)
    Serial.println("[Host] Connecting to WiFi AP: ");
    Serial.println(_ssid);
    Serial.println("...");

    if (WiFi.status() == WL_CONNECTED)
        return true;

    WiFi.begin(_ssid.c_str(), _password.c_str());
    while (WiFi.status() != WL_CONNECTED)
        delay(500);
    Serial.println("[Host] WiFi AP connected!");
    return true;
#elif defined(__has_ethernet)
    Serial.println("[Host] Board does not support WiFi. The ethernet connection is used instead.");
    return true;
#endif
    Serial.println("[Host] Board does not support WiFi and ethernet connections.");
    return false;
}

/**
 * Handles client side function StartTLS() in ESP32' WiFiClientSecure and connectSSL() in ESP_SSLClient.
 */
static bool handle_start_tls(int slot)
{
    Serial.println("[Host] Performing STARTTLS for slot " + String(slot));

#if defined(__ssl_upgrade_supports)
    if (slot == 0)
    {
#if defined(__ssl_esp_sslclient) // for ESP_SSLClient
        if (ssl_client.connectSSL())
#elif defined(ESP32) // for ESP32 without ESP_SSLClient
        if (ssl_client.startTLS())
#endif
        {
            Serial.println("[Host] Success: STARTTLS completed.");
            return true;
        }
    }
#endif

    Serial.println("[Host] Error: STARTTLS failed.");
    return false;
}

/**
 * Handles client side function setCACert() in ESP32's WiFiClientSecure and ESP_SSLClient,
 * setTrustAnchors in ESP8266/RPi Pico W's WiFiClientSecure.
 */
static void handle_certificate_setup(int slot, const char *ca_cert_filename)
{
    String caFile = ca_cert_filename;
    if (!caFile.startsWith("/"))
        caFile = "/" + caFile;

    Serial.println("[Host] Setting the CA certificate for slot " + String(slot));
#if defined(__filesystem)
    if (slot == 0)
    {
        Serial.println("[Host] Use CA file: " + caFile);

        Serial.println("[Host] Slot is a Secure Client. Attempting to load cert...");

        if (__filesystem.exists(caFile.c_str()))
        {
            File certFile = __filesystem.open(caFile.c_str(), "r");
            if (certFile)
            {
#if defined(__ssl_client)

// Support for ESP32 and Generic ESP_SSLClient (which have loadCACert)
#if defined(__ssl_esp_sslclient) || defined(ESP32)
                if (ssl_client.loadCACert(certFile, certFile.size()))
                {
                    Serial.println("[Host] Success: Certificate loaded from file.");
                }
                else
                {
                    Serial.println("[Host] Error: loadCACert() failed.");
                }

// Support for ESP8266 and Pico W (BearSSL - Requires manual X509List)
#elif defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
                size_t len = certFile.size();
                if (len > 0)
                {
                    uint8_t *buf = new uint8_t[len];
                    if (buf)
                    {
                        certFile.read(buf, len);

                        // Create X509List. IMPORTANT: This object must persist!
                        // We use a static pointer to keep it alive between connections.
                        // Ideally, we should free the old one if it exists.
                        static BearSSL::X509List *trustedCA = nullptr;
                        if (trustedCA)
                            delete trustedCA;

                        trustedCA = new BearSSL::X509List(buf, len);
                        ssl_client.setTrustAnchors(trustedCA);

                        delete[] buf; // X509List copies the data, so we can free the buffer
                        Serial.println("[Host] Success: Certificate loaded via setTrustAnchors.");
                    }
                    else
                    {
                        Serial.println("[Host] Error: Failed to allocate memory for cert.");
                    }
                }
                else
                {
                    Serial.println("[Host] Error: Empty certificate file.");
                }
#else
                Serial.println("[Host] Warning: Certificate loading not supported on this platform/library.");
#endif

#endif
                certFile.close();
            }
        }
        else
        {
            Serial.println("[Host] Error: File not found on host: " + caFile);
        }
        return;
    }
    Serial.println("[Host] Error: Invalid slot or loadCACert() failed.");
#else
    Serial.println("[Host] Error: Filesystem not available.");
#endif
}

/**
 * Handles host device reboot
 */
static void handle_reboot()
{
    Serial.println("[Host] Rebooting device in 3 seconds...");
    delay(3000);
    // Utility function to reboot device.
    HostUtil::reboot();
}

/**
 * Handles client side function setBufferSizes() in ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient.
 */
static void handle_set_buffer_size(int slot, int rx, int tx)
{
#if defined(__ssl_iobuf_supports)
    if (slot == 0)
        ssl_client.setBufferSizes(rx, tx);
#endif
}

/**
 * Handles the client side function isSecure() in ESP32/ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient.
 */
static uint8_t handle_get_flag(int slot)
{
    uint8_t flag = host.getFlag(slot); // Get current stored flag (persisted from setFlag)
#if defined(__ssl_upgrade_supports)    // for ESP32 WiFiClientSecure and ESP_SSLClient
    if (slot == 0)
    {

#if defined(__ssl_esp_sslclient) // for ESP_SSLClient
        if (ssl_client.isSecure())
            flag |= SSL_STATUS_BIT; // Set bit 0 (SSL Mode)
        else
            flag &= ~SSL_STATUS_BIT; // Clear bit 0 (Plain Mode)

#elif defined(ESP32) // for ESP32 WiFiClientSecure
        if (ssl_client.stillInPlainStart())
            flag &= ~SSL_STATUS_BIT; // Clear bit 0 (Plain Mode)
        else
            flag |= SSL_STATUS_BIT; // Set bit 0 (SSL Mode)

#endif
    }
#else // generic SSL client e.g. ESP8266 WiFiClientSecure, Raspberry Pi Pico W WiFiClientSecure, WiFiSSLClient
    flag |= SSL_STATUS_BIT;
    // Set bit 0(SSL Mode)
#endif
    return flag;
}

/**
 * Handles the client side functions setInsecure() in ESP32/ESP8266/RPi Pico W's WiFiClientSecure and ESP_SSLClient
 * setPlainStart() in ESP32/ESP8266/RPi Pico W WiFiClientSecure and ESP_SSLClient,
 * enableSSL() in ESP_SSLClient.
 */
static void handle_set_flag(int slot, uint8_t flag)
{
    if (slot == 0)
    {
#if defined(__ssl_client)

#if defined(__ssl_esp_sslclient) // for ESP_SSLClient
        if ((flag & SSL_PLAIN_START_BIT) != 0)
            ssl_client.enableSSL(false);
        else
            ssl_client.enableSSL(true);

#elif defined(ESP32) // for ESP32 WiFiClientSecure
        if ((flag & SSL_PLAIN_START_BIT) != 0)
            ssl_client.setPlainStart();
#endif

#if defined(__ssl_insecure_supports) // for WiFiClientSecure and ESP_SSLClient
        if ((flag & SSL_INSECURE_BIT) != 0)
        {
            ssl_client.setInsecure();
        }
#endif

#endif
    }
}

#endif