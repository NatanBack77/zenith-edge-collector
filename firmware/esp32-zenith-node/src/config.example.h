// Copy this file to config.h and fill in your own values.
// config.h is gitignored so credentials never leave your machine.

#pragma once

// ---- WiFi ----
// Tries WIFI_SSID first; if it can't connect within WIFI_TRY_TIMEOUT_MS,
// falls back to WIFI_SSID2. Leave WIFI_SSID2 empty to disable the fallback.
#define WIFI_SSID "brisa-162165"
#define WIFI_PASSWORD "iskkkhjde"
#define WIFI_SSID2 ""
#define WIFI_PASSWORD2 ""
#define WIFI_TRY_TIMEOUT_MS 15000

// ---- MQTT ----
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
// Leave empty for an anonymous broker.
#define MQTT_USER ""
#define MQTT_PASSWORD ""

// Topic readings are published to. The node appends "/<sensor-mac>".
#define MQTT_TOPIC_BASE "zenith/readings"
// Retained online/offline status, with the offline message set as the
// MQTT last will so a crashed node is visible.
#define MQTT_TOPIC_STATUS "zenith/status"

// ---- Sensor ----
// Leave empty to connect to the first WTVB01 found. Set a MAC to pin
// the node to one specific sensor, e.g. "e6:6b:9a:cc:88:25".
#define SENSOR_ADDRESS ""

// How often to publish, in milliseconds. The sensor broadcasts far
// faster than this; readings between publishes are coalesced.
#define PUBLISH_INTERVAL_MS 1000
