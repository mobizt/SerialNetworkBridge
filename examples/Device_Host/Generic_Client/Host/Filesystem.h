#ifndef __FILESYSTEM_H__
#define __FILESYSTEM_H__

// ESP8266
#if defined(ESP8266)
#include <LittleFS.h>
#define __filesystem LittleFS

// ESP32
#elif defined(ESP32)
#ifdef USE_LITTLEFS
#include <LittleFS.h>
#define __filesystem LittleFS
#else
#include <SPIFFS.h>
#define __filesystem SPIFFS
#endif

// Raspberry Pi Pico W (Earle Philhower Core)
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <LittleFS.h>
#define __filesystem LittleFS

// SAMD (Arduino Zero, MKR)
#elif defined(ARDUINO_ARCH_SAMD)


// STM32
#elif defined(ARDUINO_ARCH_STM32)


// Teensy
#elif defined(TEENSYDUINO)
#include <LittleFS.h> // requires external flash chip
#define __filesystem LittleFS

// Renesas RA/RX
#elif defined(__RENESAS_RA__) || defined(__RENESAS_RX__)
#include <FatFS.h> // vendor-provided
#define __filesystem FatFS

// AVR (Uno, Mega)
#elif defined(ARDUINO_ARCH_AVR)


#else
// #warning "No flash filesystem available for this platform"
#endif

#endif