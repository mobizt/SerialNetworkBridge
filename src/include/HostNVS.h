#ifndef __HOST_NVS_H__
#define __HOST_NVS_H__

#include <Arduino.h>

#if defined(ESP32)
#include <Preferences.h>
#define USE_PREFERENCES
#else
#include <EEPROM.h>
#define USE_EEPROM
// Opta and ESP8266 require EEPROM size definition or begin()
#define EEPROM_SIZE 512
#define EEPROM_ADDR 0 // Start address
#endif

static String _ssid;
static String _password;

#if defined(USE_PREFERENCES)
static Preferences prefs;
#elif defined(USE_EEPROM)
// Structure required for EEPROM because we cannot save String objects directly
struct WiFiCreds
{
    uint8_t magic; // Simple check to verify data validity (0xA5)
    char ssid[33]; // Max SSID length is 32 + null terminator
    char pass[65]; // Max WPA2 pass length is 64 + null terminator
};
#endif

static void saveWiFi(const char *ssid, const char *password)
{
    _ssid = ssid;
    _password = password;

#if defined(USE_PREFERENCES)
    if (prefs.begin("serialtcp", false))
    {
        prefs.putString("ssid", ssid);
        prefs.putString("pass", password);
        prefs.end();
        Serial.println("[Host] Credentials saved to Preferences.");
    }
    else
    {
        Serial.println("[Host] Failed to open Preferences.");
    }

#elif defined(USE_EEPROM)
#if defined(ESP8266) || defined(ARDUINO_OPTA)
    EEPROM.begin(EEPROM_SIZE);
#endif

    WiFiCreds creds;
    creds.magic = 0xA5; // distinct pattern to indicate valid data

    // Safely copy strings to fixed char arrays
    strncpy(creds.ssid, ssid, sizeof(creds.ssid) - 1);
    creds.ssid[sizeof(creds.ssid) - 1] = '\0'; // Ensure null termination

    strncpy(creds.pass, password, sizeof(creds.pass) - 1);
    creds.pass[sizeof(creds.pass) - 1] = '\0';

    EEPROM.put(EEPROM_ADDR, creds);

#if defined(ESP8266) || defined(ARDUINO_OPTA)
    // Commit is required for flash-based EEPROM emulation
    if (EEPROM.commit())
    {
        Serial.println("[Host] Credentials committed to Flash/EEPROM.");
    }
    else
    {
        Serial.println("[Host] EEPROM Commit failed.");
    }
#else
    // AVR writes instantly
    Serial.println("[Host] Credentials saved to EEPROM.");
#endif
#endif
}

static void loadWiFi(const char *default_ssid, const char *default_password)
{
    _ssid = default_ssid;
    _password = default_password;

#if defined(USE_PREFERENCES)
    if (prefs.begin("serialtcp", true))
    {
        if (prefs.isKey("ssid"))
        {
            _ssid = prefs.getString("ssid");
            _password = prefs.getString("pass");
            Serial.println("[Host] Loaded from Preferences.");
        }
        else
        {
            Serial.println("[Host] No saved keys. Using defaults.");
        }
        prefs.end();
    }

#elif defined(USE_EEPROM)
#if defined(ESP8266) || defined(ARDUINO_OPTA)
    EEPROM.begin(EEPROM_SIZE);
#endif

    WiFiCreds creds;
    EEPROM.get(EEPROM_ADDR, creds);

    // Check magic byte to see if this is valid data or empty flash (0xFF)
    if (creds.magic == 0xA5)
    {
        _ssid = String(creds.ssid);
        _password = String(creds.pass);
        Serial.println("[Host] Loaded from EEPROM.");
    }
    else
    {
        Serial.println("[Host] EEPROM empty or invalid. Using defaults.");
    }
#endif
}

#endif