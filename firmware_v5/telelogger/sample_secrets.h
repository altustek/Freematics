// Sample local overrides for config.h. All values below are fake.
// Copy to secrets.h (git-ignored) and fill in real values:
//   cp sample_secrets.h secrets.h
// Anything left undefined falls back to the defaults in config.h.

// Known WiFi networks {ssid, password, metric}: the visible network with the
// lowest metric wins, ties broken by signal strength.
#define WIFI_NETWORKS { \
  {"HomeWifi",  "home-password",  10}, \
  {"PhoneWifi", "phone-password", 20}, \
}

// HTTPD soft AP (password min 8 chars)
//#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "change-me-123"

// Server (Freematics protocol, UDP)
#define SERVER_HOST "traccar.example.com"
#define SERVER_PORT 5170
//#define SERVER_PROTOCOL PROTOCOL_UDP

// Cellular APN
//#define CELL_APN "hologram"
