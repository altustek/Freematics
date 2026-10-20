// Local overrides for config.h: WiFi credentials and server settings.
// Template with fake values: cp sample_secrets.h secrets.h (git-ignored),
// then fill in real ones. Anything left undefined uses the config.h default.
#define WIFI_NETWORKS { \
  {"HomeWifi",  "home-password",  10}, \
  {"PhoneWifi", "phone-password", 20}, \
}

// HTTPD soft AP password (min 8 chars)
//#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "change-me-123"

// Traccar server (Freematics protocol, UDP)
#define SERVER_HOST "traccar.example.com"
#define SERVER_PORT 5170
//#define SERVER_PROTOCOL PROTOCOL_UDP

// Cellular APN
//#define CELL_APN "hologram"
