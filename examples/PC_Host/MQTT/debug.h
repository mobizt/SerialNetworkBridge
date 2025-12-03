#include "HardwareSerial.h"
#ifndef __DEBUG_H__
#define __DEBUG_H__
#include <Arduino.h>
#include <SerialNetworkBridge.h>

// #define BLINK_LED_PIN -1 // GPIO pin used here should not be used by serial port for bridge
#define PULLED_UP_LED 0 // 1 = pull-up, 0 = pull-down

struct debug
{
    static void initBlink()
    {
#if defined(BLINK_LED_PIN) && BLINK_LED_PIN > -1
        pinMode(BLINK_LED_PIN, OUTPUT);
        digitalWrite(BLINK_LED_PIN, PULLED_UP_LED);
#endif
    }

    static void blink(int times, int delayMs)
    {
#if defined(BLINK_LED_PIN) && BLINK_LED_PIN > -1
        for (int i = 0; i < times; i++)
        {
            digitalWrite(BLINK_LED_PIN, !PULLED_UP_LED);
            delay(delayMs);
            digitalWrite(BLINK_LED_PIN, PULLED_UP_LED);
            delay(delayMs);
        }
#endif
    }
    /**
     * Print debug information with tag and spaced ident options to the sink object
     * @param sink The object to print to e.g. Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
     * @param msg The message to print.
     * @param tag Option to prepend with tag i.e. [Client].
     * @param tag Option to prepend with space.
     */
    template <typename T>
    static void print(T &sink, const char *msg = "", bool tag = true, bool indent = true)
    {
#if defined(HOST_RELAY_DEBUG)
        if (indent)
            sink.hostPrint(" ");
        if (tag)
            sink.hostPrint("[Client] ");
        sink.hostPrint(msg);
#else
        if (tag)
            sink.print("[Client] ");
        sink.print(msg);
#endif
    }

    /**
     * Print new line to the sink object
     * @param sink The object to print to e.g. Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
     */
    template <typename T>
    static void printNewLine(T &sink)
    {
        print(sink, "\r\n", false, false);
    }

    /**
     * Print debug information to the sink object
     * @param sink The object to print to e.g. Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
     * @param msg The message to print.
     */
    template <typename T>
    static void printRaw(T &sink, const char *msg)
    {
        print(sink, msg, false, false);
    }

    /**
     * Read data stream from sink object to beffer
     * Data will read from stream until the endToken was found or the buffer is full.
     * @param source The object to read data e.g. Serial, SerialTCPClient, SerialUDPClient, SerialWebsocketClient, SerialHostManager
     * @param buffer The buffer to store the data.
     * @param bufferLen The size of buffer.
     * @param totalRead The total data that has been read.
     * @param endToken The token to terminate the reading.
     */
    template <typename TSRC, typename TSNK>
    static int readResonse(TSRC &source, TSNK &sink, char *buffer, size_t bufferLen, int &totalRead, const char endToken)
    {
        size_t pos = 0;
        int c;

        unsigned long ms_start = millis();
        unsigned long timeout = 5000; // 5 second timeout

        if (!source.connected() && source.available() == 0 && totalRead == 0)
            return -1; // socket closed

        while ((source.connected() || source.available()) && (millis() - ms_start < timeout))
        {
            if (!source.available())
            {
                delay(1); // avoid busy loop
                continue;
            }

            c = source.read();
            if (c == -1)
                continue;

            buffer[pos++] = (char)c;
            totalRead++;

            if (c == endToken || (endToken == 0 && source.available() == 0) || pos >= bufferLen - 1)
            {
                buffer[pos] = '\0';
                print(sink, buffer, false); // debug
                return (int)pos;
            }

            ms_start = millis(); // reset idle timeout only after valid read
        }

        return millis() - ms_start < timeout ? 0 /* no data available */ : -2 /* timeout */;
    }
};

#endif