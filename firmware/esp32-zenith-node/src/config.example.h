// Copy this file to config.h and fill in your own values.
// config.h is gitignored so credentials never leave your machine.

#pragma once

#include "wifi_credential.h"

// ---- WiFi ----
// Add as many networks as you want here. The node tries each one in
// order, giving up on one after WIFI_TRY_TIMEOUT_MS and moving to the
// next, looping forever until one connects.
static const WifiCredential WIFI_NETWORKS[] = {
    {"your-ssid", "your-password"},
    // {"second-ssid", "second-password"},
};
#define WIFI_NETWORKS_COUNT (sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]))
// Per-SSID timeout. The node also remembers which network last worked
// and tries that one first on reconnect, so this mostly bounds how long
// a single transient blip blocks the loop before moving on.
#define WIFI_TRY_TIMEOUT_MS 8000

// ---- MQTT ----
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
// Leave empty for an anonymous broker.
#define MQTT_USER ""
#define MQTT_PASSWORD ""

// PubSubClient's default keepalive is 15s, which is shorter than some
// blocking sections of the main loop (BLE scan alone can take 10s). If
// no PINGREQ reaches the broker within ~1.5x this window, it drops the
// connection even though the node is otherwise healthy. Keep this well
// above the worst-case blocking stretch.
#define MQTT_KEEPALIVE_SEC 45

// Bounds how long a single stalled publish() can block loop() for.
// Applied two ways in main.cpp: PubSubClient::setSocketTimeout() (its
// own MQTT-level ack waits) and WiFiClient::setTimeout() (the actual
// TCP socket's SO_SNDTIMEO/SO_RCVTIMEO -- this is the one that matters;
// PubSubClient's own timeout does NOT reach the raw socket write, so
// setting only that one still left writes blocking for 10-20s in
// testing). A failed write leaves the reading queued and retried (see
// DrainReadingBuffer), so shortening this is a pure
// latency/responsiveness win, not a new source of loss.
#define MQTT_SOCKET_TIMEOUT_SEC 5

// Topic readings are published to. The node appends "/<sensor-mac>".
#define MQTT_TOPIC_BASE "zenith/readings"
// Retained online/offline status, with the offline message set as the
// MQTT last will so a crashed node is visible.
#define MQTT_TOPIC_STATUS "zenith/status"

// ---- Sensor(s) ----
// List every WTVB01-BT50 MAC this node should connect to and stream
// from at the same time. Leave the array empty to auto-connect to up
// to MAX_AUTO_SENSORS units found during a scan (whichever ones are in
// range first -- non-deterministic if more than that are nearby).
static const char *const SENSOR_ADDRESSES[] = {
    // "e6:6b:9a:cc:88:25",
    // "c1:02:5f:aa:11:09",
};
#define SENSOR_ADDRESSES_COUNT \
  (sizeof(SENSOR_ADDRESSES) / sizeof(SENSOR_ADDRESSES[0]))
// Only used when SENSOR_ADDRESSES is empty (auto-discovery mode).
#define MAX_AUTO_SENSORS 2

// How often to sample, in milliseconds. The sensor broadcasts far
// faster than this; readings between samples are coalesced. Sampling
// keeps running at this cadence even while MQTT is disconnected.
#define PUBLISH_INTERVAL_MS 1000

// How many samples to hold in the ring buffer while MQTT is
// disconnected, so a reconnect drains and republishes what was missed
// instead of just resuming live. At one sample per PUBLISH_INTERVAL_MS,
// the default holds 60s of outage before the oldest samples start
// getting overwritten.
#define READING_BUFFER_CAPACITY 60
