#ifndef __HOST_NETWORK_H__
#define __HOST_NETWORK_H__

#include <Arduino.h>

/**
 * __has_wifi               Has core WiFIi or external WiFi library installed
 *
 * __has_ethernet           Has Ethernet libary installed
 *
 * __udp_client             WiFiUDP or EthernetUDP class
 *
 * __ssl_client             The SSL client class
 *
 * __ssl_insecure_supports  setInsecure() function supports in SSL client
 *
 * __ssl_upgrade_supports   connectSSL() and startTLS() functions in SSL client
 *
 * __ssl_iobuf_supports     setBufferSizes supports in SSL client
 *
 * __network_class          WiFiClient or EthernetClient class
 *
 * __ssl_set_network_class  Network client is required for SSL client
 */

// Ethernet only platform
#if defined(ARDUINO_ARCH_STM32) || defined(Ethernet_h) || defined(ethernet_h_)

#include <SPI.h>
#include <Ethernet.h>

#if defined(USE_UDP)
#include <EthernetUdp.h>
#define __udp_client EthernetUDP
#endif
#define __network_class EthernetClient
#define __has_ethernet

#endif

// Check if WiFi library already installation and has include
#if defined(WiFiNINA_h) || defined(WIFI_H)

#include <WiFiClient.h>
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#include <WiFiSSLClient.h>

#define __ssl_client WiFiSSLClient
#define __network_class WiFiClient

#define __has_wifi

#else // Check core WiFi library

// ESP8266 native WiFi library
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#define __ssl_client WiFiClientSecure
#define __network_class WiFiClient // prefered WiFi

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __ssl_insecure_supports
#define __has_wifi

// ESP32  native WiFi library
#elif defined(ESP32)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#define __ssl_client WiFiClientSecure

#define __network_class NetworkClient
#define __has_ethernet

#if defined(USE_UDP)
#include <NetworkUdp.h>
#define __udp_client NetworkUDP
#endif

#define __ssl_insecure_supports
#define __ssl_upgrade_supports
#define __has_wifi

// Raspberry Pi Pico W (arduino-pico sdk) native WiFi library
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#define __ssl_client WiFiClientSecure
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __ssl_insecure_supports
#define __has_wifi

// Arduino Uno R4 WiFi (Renesas RA4M1) native WiFi library
#elif defined(ARDUINO_UNOR4_WIFI)

#include <WiFiS3.h>
#include <WiFiClient.h>
#include <WiFiSSLClient.h>

#define __ssl_client WiFiSSLClient
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __has_wifi

// Arduino boards require WiFiNINA/WiFi101 installation and include
#elif defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_SAMD_MKR1000)

#if !defined(__has_wifi)

#if defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#error "WiFi101 library is required. Please install it first and include it on top of your code."
#else
#error "WiFiNINA library is required. Please install it first and include it on top of your code."
#endif

#else

#if defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFi101.h>
#else
#include <WiFiNINA.h>
#endif
#include <WiFiClient.h>
#include <WiFiSSLClient.h>

#define __ssl_client WiFiSSLClient
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __has_wifi
#endif

#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#include <WiFiSSLClient.h>

#define __ssl_client WiFiSSLClient
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __has_wifi

#elif defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)

#include <WiFi.h>
#include <WiFiSSLClient.h>

#define __ssl_client WiFiSSLClient
#define __network_class WiFiClient

#if defined(USE_UDP)
#include <WiFiUdp.h>
#define __udp_client WiFiUDP
#endif

#define __has_wifi

#else

#if !defined(ESP_SSLCLIENT_H)
#error "ESP_SSLClient library is required. Please install it first and include it on top of your code."
#else

// Generic SSL wrapper (Install "ESP_SSLClient" library)
// https://github.com/mobizt/ESP_SSLClient
#include <ESP_SSLClient.h>
#define __ssl_client ESP_SSLClient
#define __ssl_insecure_supports
#define __ssl_upgrade_supports
#define __ssl_iobuf_supports
#define __ssl_set_network_class
#define __ssl_esp_sslclient

#endif

#endif

#endif

// No WiFi and Ethernet devices
#if !defined(__has_wifi) && !defined(__has_ethernet)
#warning "You cannot use this device as a host device due to lag of WiFi and Ethernet onnection capabilities."
#endif

// If ESP_SSLClient is used
#if defined(USE_ESP_SSLCLIENT)

#if !defined(ESP_SSLCLIENT_H)
#error "ESP_SSLClient library is required. Please install it first and include it on top of your code."
#else

// Generic SSL wrapper (Install "ESP_SSLClient" library)
// https://github.com/mobizt/ESP_SSLClient
#include <ESP_SSLClient.h
#define __ssl_client ESP_SSLClient
#define __ssl_insecure_supports
#define __ssl_upgrade_supports
#define __ssl_iobuf_supports
#define __ssl_set_network_class
#define __ssl_esp_sslclient

#endif

#endif

#endif // __HOST_NETWORK_H__
