/**
 * ===============================================
 * Basic Host (with NVS Storage)
 * ===============================================
 * Runs on: A device with network access (e.g., ESP32, Raspberry Pi Pico W).
 * Purpose: Acts as the network bridge. Stores WiFi credentials persistently.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>

#define ENABLE_SERIALTCP_DEBUG // Enable debug prints for SerialNetworkHost
#include <SerialNetworkHost.h>

// --- Persistence Configuration ---
// For ESP32, we use the Preferences library (NVS) to store WiFi credentials.
#if defined(ESP32)
#include <Preferences.h>
Preferences prefs;
#define USE_NVS_STORAGE
#endif

// Network Config (Defaults)
// These are used if no credentials are found in storage.
String ssid = "DEFAULT_WIFI_SSID";
String password = "DEFAULT_WIFI_PASSWORD";

// Serial TCP Host Config
const int CLIENT_SLOT = 0;       // Corresponding to Network client or SSL client slot 0 on the host
const long SERIAL_BAUD = 115200; // Corresponding to the baud rate used in the client Serial

WiFiClientSecure ssl_client; // Or WiFiClient for plain text
SerialNetworkHost host(Serial2); // Use Serial2 for communication


/**
 * @brief Callback to handle "Set WiFi" command from the Client.
 * Saves the new credentials to NVS so they persist after reboot.
 */
bool onSetWiFi(const char *new_ssid, const char *new_pass)
{
  Serial.print("[Host] Received new WiFi credentials for: ");
  Serial.println(new_ssid);

  ssid = String(new_ssid);
  password = String(new_pass);

#if defined(USE_NVS_STORAGE)
  if (prefs.begin("serialtcp", false))
  { // Open in Read/Write mode
    prefs.putString("ssid", ssid);
    prefs.putString("pass", password);
    prefs.end();
    Serial.println("[Host] Credentials saved to NVS.");
  }
  else
  {
    Serial.println("[Host] Failed to open NVS for writing.");
  }
#endif

  // Optional: Disconnect to allow the 'Connect' command to re-establish
  // with new credentials immediately if requested.
  WiFi.disconnect();
  return true;
}

/**
 * @brief Callback to handle "Connect Network" command from the Client.
 */
bool onConnectNetwork()
{
  Serial.println("[Host] Client requested WiFi connection...");
  if (WiFi.status() == WL_CONNECTED)
  {
    return true;
  }

  WiFi.begin(ssid.c_str(), password.c_str());
  // We don't block here; the main loop or status check handles the rest.
  return true;
}


void setup()
{
  // Start local Serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n[Host] SerialTCP Host Starting...");

// Load Credentials from Storage
#if defined(USE_NVS_STORAGE)
  if (prefs.begin("serialtcp", true))
  { // Open in Read-Only mode
    if (prefs.isKey("ssid"))
    {
      ssid = prefs.getString("ssid");
      password = prefs.getString("pass");
      Serial.println("[Host] Loaded credentials from NVS.");
    }
    else
    {
      Serial.println("[Host] No saved credentials found. Using defaults.");
    }
    prefs.end();
  }
#endif

  // Initialize Communication
  // Start Serial2 for communication with the client device
  // ESP32 Default Pins: RX=16, TX=17
  Serial2.begin(SERIAL_BAUD);

  // Register Callbacks
  host.setSetWiFiCallback(onSetWiFi);
  host.setConnectNetworkCallback(onConnectNetwork);

  // Bind the network client
  host.setTCPClient(&ssl_client, CLIENT_SLOT);
  host.setLocalDebugLevel(1);

  Serial.print("[Host] Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid.c_str(), password.c_str());

  // Wait for connection (Optional: Could be non-blocking if preferred)
  // For a bridge, it's often good to ensure connectivity before notifying client.
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20)
  {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n[Host] WiFi Connected!");
    Serial.print("[Host] IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\n[Host] WiFi Connection Failed (Check credentials).");
  }

  // Configure SSL Client (Optional)
  ssl_client.setInsecure(); // Skip certificate validation for simplicity
  // ssl_client.setBufferSizes(2048, 1024); // Adjust buffers if needed


  // Notify the client that host is rebooted/ready.
  // This resets the client's internal state for a fresh session.
  host.notifyBoot();

  Serial.println("[Host] Ready. Listening for client commands...");
}

void loop()
{
  // Requirements for Host operation
  host.loop();

  // Optional: Reconnect WiFi if lost
  if (WiFi.status() != WL_CONNECTED && ssid.length() > 0)
  {
    static unsigned long last_reconnect = 0;
    if (millis() - last_reconnect > 10000)
    {
      last_reconnect = millis();
      Serial.println("[Host] WiFi lost. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid.c_str(), password.c_str());
    }
  }
}