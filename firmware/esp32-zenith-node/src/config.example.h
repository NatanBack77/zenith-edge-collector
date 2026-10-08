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
// 1 = MQTT sobre TLS (broker na AWS, porta 8883, certificado Let's Encrypt validado com src/root_ca.h, hora via NTP).
// 0 = MQTT em claro na LAN (Mosquitto local, porta 1883). Com TLS o host precisa ser o nome do certificado
// (<ip-com-hifens>.sslip.io), nunca o IP cru.
#define MQTT_USE_TLS 0
#if MQTT_USE_TLS
#define MQTT_HOST "x-x-x-x.sslip.io"
#define MQTT_PORT 8883
#else
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
#endif
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
// PIN YOUR SENSOR HERE for anything that must keep running: with one slot there is
// no periodic BLE re-scan, the first scan ends as soon as the sensor is seen, and a
// reboot recovers in ~13 s instead of ~23 s. Auto mode (empty list) is for the bench.
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
// faster than this (README: "sensor com intervalo de notificação de
// ~200 ms e tranquilo" -- i.e. the BLE notify itself already arrives
// close to this cadence); readings between samples are coalesced.
// Sampling keeps running at this cadence even while MQTT is disconnected.
//
// This was 1000ms, which threw away ~4/5 of the readings the sensor was
// already delivering before ever reaching MQTT -- a pure software
// bottleneck, not a sensor limit. 200ms keeps close to native notify
// cadence without pushing so much MQTT traffic that a multi-sensor node
// saturates WiFi (README also warns about that). This does NOT raise the
// sensor's own output rate (see SENSOR_CONFIGURE_RATE below for that,
// separate and experimental) -- it only stops discarding what already
// arrives.
#define PUBLISH_INTERVAL_MS 200

// How many samples to hold in the ring buffer while MQTT is
// disconnected, so a reconnect drains and republishes what was missed
// instead of just resuming live. At one sample per PUBLISH_INTERVAL_MS,
// 300 holds the same ~60s of outage as before (60 samples was sized for
// the old 1000ms cadence; keeping it at 60 now would only cover ~12s).
#define READING_BUFFER_CAPACITY 300

// ---- Sensor output rate (optional, experimental) ----
// MEASURED on the unit E6:6B:9A:CC:88:25 (6 Oct 2026, frames over MQTT):
//   0x09 = 100.00 Hz (what this unit runs at; reference, 100% of frames)
//   0x0A = 125.00 Hz (works, 100% of frames, same memory) -- NOT 200 Hz as the
//          V260410 manual says: this unit follows the old app's code table
//   0x0B = no usable stream with this firmware (the sensor went back to the
//          Default data mode and no waveform window ever closed)
//   0x08 would be 50 Hz.
// 8 Oct 2026: the project now trains and analyses at 125 Hz (SENSOR_CONFIGURE_RATE 1, SENSOR_RATE_CODE 0x0A). At 100 Hz the motor's
// 2x line-frequency vibration (120 Hz) folds to ~20 Hz, inside the analysed band, and inflates the velocity; at 125 Hz it folds to ~4 Hz,
// outside it. Every session of one training must use the same rate.
// The write is saved in the sensor's memory and persists, so enable it only to
// flash once and turn it off again (it also rewrites on every reconnect).
// The WTVB01-BT50's RATE register (0x03) defaults to 0x06 (10Hz). The table
// below was documented for the general WitMotion WT/BWT protocol family. This firmware normally never writes to the
// sensor at all (see README's "por que é simples") -- this is the one
// optional exception, off by default. The write-command format itself
// (unlock/write/save via 0xFF 0xAA framing) IS confirmed from the
// official SDK -- see docs/protocol.md §7 -- but nobody has bench-tested
// an actual register write against physical hardware in this repo yet.
// Enable and verify on a bench unit (watch Serial output, confirm the
// notify interval actually changes) before relying on this in the field.
#define SENSOR_CONFIGURE_RATE 0
// 0x06=10Hz (factory default) 0x07=20Hz 0x08=50Hz 0x09=100Hz 0x0A=200Hz.
// Recommended starting point on the bench: 0x08 (50Hz) -- comfortably
// above what 1x-3x order analysis needs for typical industrial RPMs,
// without generating BLE/WiFi traffic a multi-sensor node can't keep up
// with. Only relevant when SENSOR_CONFIGURE_RATE is 1.
#define SENSOR_RATE_CODE 0x08

// ---- Sensor data mode: "Now data" (optional, writes to the sensor) ----
// The sensor has two data modes. Default sends amplitudes (what the Zenith
// Monitoramento page shows) at ~100 packets/s. "Now data" sends raw
// acceleration with a chip timestamp at the same rate, which is what an FFT
// (unbalance, looseness) needs. Register 0x96: 1 = Now data, 0 = Default --
// not in the manual, found by capturing the official app (see
// docs/protocol.md). The write is the same unlock -> write -> save the
// official app sends, and it PERSISTS in the sensor, so the node only
// writes when the sensor is not already in the wanted mode.
//
// With Now data on, zenith/readings keeps publishing: velocity /
// displacement / angle become the node's smoothed RMS of the signed
// samples, frequency comes from a 256-point FFT of each window, and
// temperature / battery are read from the sensor every
// SENSOR_STATUS_POLL_MS. Each message carries "mode":"instant".
// The raw acceleration goes to MQTT_TOPIC_WAVEFORM_BASE/<mac> as a binary
// frame (see the README).
#define SENSOR_CONFIGURE_DATA_MODE 0
// 1 = Now data (waveform + derived readings), 0 = Default. Only used when
// SENSOR_CONFIGURE_DATA_MODE is 1.
#define SENSOR_DATA_MODE_INSTANT 1
// Rare read of temperature and battery while in Now data mode. More often
// than ~10 s makes waveform windows restart (measured), so keep it high.
#define SENSOR_STATUS_POLL_MS 30000
// Publish gap-free acceleration windows (needs Now data mode to produce
// anything; costs nothing in Default mode).
#define WAVEFORM_ENABLE 1
#define MQTT_TOPIC_WAVEFORM_BASE "zenith/waveform"
