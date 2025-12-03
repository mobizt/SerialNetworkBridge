#ifndef __HOST_REBOOT_H__
#define __HOST_REBOOT_H__

#include <Arduino.h>

namespace HostUtil
{

#if defined(ARDUINO_ARCH_AVR)
#include <avr/wdt.h>
    inline void reboot()
    {
        wdt_enable(WDTO_15MS);
        while (true)
        {
        }
    }

#elif defined(ESP32)
#include <esp_system.h>
    inline void reboot() { esp_restart(); }
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
    inline void reboot() { ESP.restart(); }

#elif defined(TEENSYDUINO)
    inline void reboot() { SCB_AIRCR = 0x05FA0004; }

#elif defined(ARDUINO_ARCH_SAMD)
    inline void reboot() { NVIC_SystemReset(); }

#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F4)

#if defined(ARDUINO_ARCH_STM32)
    // Official STM32 Core (STM32duino)
    inline void reboot() { NVIC_SystemReset(); }
#elif defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F4)
    // Legacy Maple Core (Roger Clark)
    inline void reboot() { nvic_sys_reset(); }
#endif

#elif defined(ARDUINO_ARCH_RP2040)
// Differentiate between Official Mbed Core and Earle Philhower Core
#if defined(ARDUINO_ARCH_MBED)
    inline void reboot() { NVIC_SystemReset(); }
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
    // Earle Philhower Core
    inline void reboot() { rp2040.reboot(); }
#else

    inline void reboot()
    {
        watchdog_enable(1000, 1);
        while (true)
        {
        }
    }

#endif

#elif defined(ARDUINO_ARCH_NRF52)
#include "Arduino.h"
    inline void reboot() { NVIC_SystemReset(); }

#elif defined(__RENESAS_RA__)
#include "cmsis_gcc.h"
    inline void reboot() { NVIC_SystemReset(); }

#elif defined(__RENESAS_RX__)
#include "iodefine.h"
    inline void reboot()
    {
        SYSTEM.PRCR.WORD = 0xA50B;
        SYSTEM.SWRSTCR.BIT.SWRST = 1;
        while (true)
        {
        }
    }

#elif defined(__RENESAS_RL78__)
#include <iodefine.h>
    inline void reboot()
    {
        WDTE = 0x00; // Trigger Watchdog
        while (true)
        {
        }
    }

// Fallback for generic ARM Cortex-M devices not explicitly caught above
#elif defined(__arm__)
#if defined(ARDUINO_ARCH_MBED)
#include "mbed.h"
    inline void reboot() { NVIC_SystemReset(); }
#else
    // Try standard CMSIS reset
    extern "C" void NVIC_SystemReset();
    inline void reboot()
    {
        NVIC_SystemReset();
    }
#endif

#else
    // Final Fallback: Hang (Safe Fail)
    inline void reboot()
    {
        while (true)
        {
        }
    }

#endif
}
#endif