#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#ifdef CONFIG_ENABLE_OBD
#define ENABLE_OBD CONFIG_ENABLE_OBD
#endif
#ifdef CONFIG_ENABLE_MEMS
#define ENABLE_MEMS CONFIG_ENABLE_MEMS
#endif
#ifdef CONFIG_GNSS
#define GNSS CONFIG_GNSS
#endif
#ifdef CONFIG_STORAGE
#define STORAGE CONFIG_STORAGE
#endif
#ifdef CONFIG_BOARD_HAS_PSRAM
#define BOARD_HAS_PSRAM 1
#endif
#ifdef CONFIG_ENABLE_WIFI
#define ENABLE_WIFI CONFIG_ENABLE_WIFI
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#endif
#ifdef CONFIG_ENABLE_BLE
#define ENABLE_BLE CONFIG_ENABLE_BLE
#endif
#ifdef CONFIG_ENABLE_HTTPD
#define ENABLE_HTTPD CONFIG_ENABLE_HTTPD
#endif
#ifdef CONFIG_SERVER_HOST
#define SERVER_HOST CONFIG_SERVER_HOST
#define SERVER_PORT CONFIG_SERVER_PORT
#define SERVER_PROTOCOL CONFIG_SERVER_PROTOCOL
#endif
#ifdef CONFIG_CELL_APN
#define CELL_APN CONFIG_CELL_APN
#endif

/**************************************
* Circular Buffer Configuration
**************************************/
#if BOARD_HAS_PSRAM
#define BUFFER_SLOTS 1024 /* max number of buffer slots */
#define BUFFER_LENGTH 384 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 4096 /* bytes */
#else
#define BUFFER_SLOTS 32 /* max number of buffer slots */
#define BUFFER_LENGTH 256 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 1024 /* bytes */
#endif

/**************************************
* Configuration Definitions
**************************************/
#define STORAGE_NONE 0
#define STORAGE_SPIFFS 1
#define STORAGE_SD 2

#define GNSS_NONE 0
#define GNSS_STANDALONE 1
#define GNSS_CELLULAR 2

#define PROTOCOL_UDP 1
#define PROTOCOL_HTTPS_GET 2
#define PROTOCOL_HTTPS_POST 3

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif

// maximum consecutive OBD access errors before entering standby
#define MAX_OBD_ERRORS 3

// AT command sets to try when the ECU doesn't answer with the default (auto
// protocol) settings, each a ';'-separated list. The set that worked is tried
// first next time. Undefined: default settings only. Commands:
//   ATSP n  protocol: 6 = CAN 11-bit 500k, 7 = CAN 29-bit 500k,
//           8 = CAN 11-bit 250k, 9 = CAN 29-bit 250k
//   ATCF h  CAN receive filter, ATCM h  CAN receive mask (0: accept all)
// Toyota (from the ankostis fork):
//#define OBD_ALT_INIT_CMDS {"ATSP7;ATCF18DAF101;ATCM1FFFFF00", "ATSP6;ATCF7E8;ATCM7F8"}
// generic, no CAN mask:
//#define OBD_ALT_INIT_CMDS {"ATSP7;ATCM0", "ATSP6;ATCM0"}

// report ignition state with every sample:
//  - PID_IGNITION (0x92, Traccar attribute io146): 1 when the ECU answers or
//    the alternator is charging (>= CHARGING_VOLTAGE), else 0
//  - RPM (0x10C): a responding ECU at RPM 0 (engine auto-stop) is sent as 1,
//    and RPM 0 is sent while the ignition is off, so rpm > 0 means ignition on
// In Traccar add a computed attribute "ignition" = io146 == 1 (or rpm > 0).
#ifndef IGNITION_REPORT
#define IGNITION_REPORT 1
#endif
#define PID_IGNITION 0x92

// don't enter standby while the engine runs (RPM > 0), even when stationary
#ifndef STAY_AWAKE_ENGINE_RUNNING
#define STAY_AWAKE_ENGINE_RUNNING 1
#endif

// send the serving cell (when on cellular, and only when it changes) as
// PIDs 0x41-0x44: MCC, MNC, LAC/TAC, cell ID (Traccar io65-io68)
#ifndef CELL_INFO_REPORT
#define CELL_INFO_REPORT 1
#endif
#define PID_CELL_MCC 0x41
#define PID_CELL_MNC 0x42
#define PID_CELL_LAC 0x43
#define PID_CELL_CID 0x44

// keep reporting this long after the ECU stops answering before entering
// standby (0: standby at once). GNSS and network stay up, so the parked
// position is accurate and the server sees the ignition go off. Standby may
// still come sooner through the STATIONARY_TIME_TABLE timeout.
#ifndef IGNITION_OFF_LINGER
#define IGNITION_OFF_LINGER 120 /* seconds */
#endif

/**************************************
* Networking configurations
**************************************/
// Local overrides (not committed): server host/port, APN, WiFi credentials.
// Start from sample_secrets.h: cp sample_secrets.h secrets.h
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1
// WiFi settings
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#endif 

// cellular network settings
#ifndef CELL_APN
#define CELL_APN "hologram"
#endif
// server settings (Freematics protocol)
#ifndef SERVER_HOST
#define SERVER_HOST "hub.freematics.com"
#endif
#ifndef SERVER_PROTOCOL
#define SERVER_PROTOCOL PROTOCOL_UDP
#endif

// SIM card setting
#define SIM_CARD_PIN ""
#define APN_USERNAME NULL
#define APN_PASSWORD NULL

// HTTPS settings
#define SERVER_PATH "/hub/api"

#if !SERVER_PORT
#undef SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT 8081
#else
#define SERVER_PORT 443
#endif
#endif

// Known WiFi networks {ssid, password, metric}: the visible network with the
// lowest metric wins, ties broken by signal strength. Falls back to
// WIFI_SSID / NVS settings when none of these is in range.
// Define WIFI_NETWORKS in secrets.h (see sample_secrets.h).
// WiFi transmit power (library default is WIFI_POWER_8_5dBm)
#define WIFI_TX_POWER WIFI_POWER_19_5dBm
// ignore networks weaker than this (dBm)
#define WIFI_MIN_RSSI -85
// minimum time between scans for known networks while not connected
#define WIFI_SCAN_INTERVAL 60 /* seconds */

// WiFi Mesh settings
#define WIFI_MESH_ID "123456"
#define WIFI_MESH_CHANNEL 13

// WiFi AP settings (override in secrets.h)
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "TELELOGGER"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "PASSWORD"
#define WIFI_AP_PASSWORD_DEFAULT 1
#endif

// maximum consecutive communication errors before resetting network
#define MAX_CONN_ERRORS_RECONNECT 5
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// data interval settings
#define STATIONARY_TIME_TABLE {10, 60, 180} /* seconds */
#define DATA_INTERVAL_TABLE {1000, 2000, 5000} /* ms */
#define PING_BACK_INTERVAL 900 /* seconds */
// each standby ping is followed by a full report (position, battery,
// temperature); GNSS is powered up for at most this long to get a fix
// (0: no GNSS, the report goes without a position)
#define STANDBY_GPS_TIMEOUT 90 /* seconds */
#define SIGNAL_CHECK_INTERVAL 10 /* seconds */

/**************************************
* Data storage configurations
**************************************/
#ifndef STORAGE
// change the following line to change storage type
#define STORAGE STORAGE_SD
#endif

/**************************************
* MEMS sensors
**************************************/
#ifndef ENABLE_MEMS
#define ENABLE_MEMS 1
#endif

/**************************************
* GPS
**************************************/
#ifndef GNSS
// change the following line to change GNSS setting
#define GNSS GNSS_STANDALONE
#endif
// keeping GNSS power on during standby 
#define GNSS_ALWAYS_ON 0
// GNSS reset timeout while no signal
#define GNSS_RESET_TIMEOUT 300 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
// motion threshold for waking up
#define MOTION_THRESHOLD 0.4f /* vehicle motion threshold in G */
// samples above MOTION_THRESHOLD needed within MOTION_CONFIRM_WINDOW to wake
// up, so a door slam doesn't (1: wake on the first one). Samples are about
// 100-200 ms apart. Standby also wakes when the battery starts charging
// (engine running), checked every BATTERY_CHECK_INTERVAL.
#define MOTION_CONFIRM_SAMPLES 2
#define MOTION_CONFIRM_WINDOW 2000 /* ms */
// engine jumpstart voltage for waking up (when MEMS unavailable) 
#define JUMPSTART_VOLTAGE 14 /* V */
// reset device after waking up
#define RESET_AFTER_WAKEUP 1

// battery protection while parked (standby)
// below this the standby pings stop and motion alone no longer wakes the device
#define LOW_BATTERY_VOLTAGE 12.0f /* V */
// above this the alternator is charging (engine running): clears low battery
#define CHARGING_VOLTAGE 13.2f /* V */
// readings below this mean bench/USB power, not a vehicle battery: ignored
#define MIN_VEHICLE_VOLTAGE 9.0f /* V */
// how often to check the battery in standby
#define BATTERY_CHECK_INTERVAL 60 /* seconds */
// keep soft AP and BLE up this long after entering standby (low battery: off at once)
#define STANDBY_RADIO_OFF_DELAY 600 /* seconds */

/**************************************
* Additional features
**************************************/
// seconds to wait at boot for a key on USB serial that opens a pipe to the
// OBD co-processor or the modem, for typing AT commands (0: off). Adds the
// wait to every boot, so only for bench debugging.
#ifndef BOOT_AT_PIPE_TIMEOUT
#define BOOT_AT_PIPE_TIMEOUT 0
#endif

#define PIN_SENSOR1 34
#define PIN_SENSOR2 26

#define COOLING_DOWN_TEMP 75 /* celsius degrees */
// send device temperature (PID 0x82) in 0.1 degC: Traccar's Freematics decoder
// divides it by 10. Set to 0 for whole degrees (Freematics Hub).
#ifndef TEMP_TENTHS
#define TEMP_TENTHS 1
#endif

// set to 1 to silence the buzzer (no connection beeps)
#ifndef SILENT
#define SILENT 0
#endif

// enable(1)/disable(0) http server
#ifndef ENABLE_HTTPD
#define ENABLE_HTTPD 1
#endif
#if ENABLE_HTTPD && WIFI_AP_PASSWORD_DEFAULT
#warning "HTTPD soft AP uses the default password; set WIFI_AP_PASSWORD in secrets.h"
#endif

// enable(1)/disable(0) BLE SPP server (for Freematics Controller App).
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif


#endif // CONFIG_H_INCLUDED
