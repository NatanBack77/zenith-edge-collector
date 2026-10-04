// Zenith Edge node: reads a WitMotion WTVB01-BT50 over BLE and
// publishes normalized readings to MQTT.
//
// The sensor broadcasts every measurement register in its 0x61 packet
// without being asked, so by default this node never writes to the
// sensor: it scans, connects, subscribes to the notify characteristic,
// and parses. Two opt-in writes exist (SENSOR_CONFIGURE_RATE and
// SENSOR_CONFIGURE_DATA_MODE in config.h), both off by default.
//
// The sensor has two data modes (see docs/protocol.md and the Zenith
// repo's docs/sensor-wtvb01-bt50.md):
//   - Default: ~100 packets/s of amplitudes. The node keeps the latest
//     one and publishes it every PUBLISH_INTERVAL_MS on zenith/readings.
//   - "Now data": ~100 packets/s of raw acceleration with a chip
//     timestamp. Nothing is dropped: samples are gathered into gap-free
//     windows and published on zenith/waveform for FFT downstream.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "config.h"
#include "wtvb01.h"

// Settings added after the first config.h templates went out: defaulted
// here so an older src/config.h keeps compiling. See config.example.h.
#ifndef WAVEFORM_ENABLE
#define WAVEFORM_ENABLE 1
#endif
#ifndef MQTT_TOPIC_WAVEFORM_BASE
#define MQTT_TOPIC_WAVEFORM_BASE "zenith/waveform"
#endif
#ifndef SENSOR_CONFIGURE_RATE
#define SENSOR_CONFIGURE_RATE 0
#endif
#ifndef SENSOR_RATE_CODE
#define SENSOR_RATE_CODE 0x09
#endif
#ifndef SENSOR_CONFIGURE_DATA_MODE
#define SENSOR_CONFIGURE_DATA_MODE 0
#endif
#ifndef SENSOR_DATA_MODE_INSTANT
#define SENSOR_DATA_MODE_INSTANT 1
#endif
// In "Now data" mode the sensor stops broadcasting the Default packet
// (amplitudes, frequency, temperature, battery). zenith/readings keeps its
// fields anyway:
//   - velocity / displacement / angle: smoothed RMS of the signed samples,
//     computed on the node (wtvb01::InstantStats);
//   - frequency: dominant frequency of each waveform window (FFT);
//   - temperature / battery: read from the sensor every
//     SENSOR_STATUS_POLL_MS (they are not in the Now data packet).
// Reading registers more often than that disturbs the 100 samples/s
// stream and makes waveform windows restart (measured on hardware), so the
// poll is deliberately rare. 0 disables it (temperature and battery then
// stay 0 in Now data mode).
#ifndef SENSOR_STATUS_POLL_MS
#define SENSOR_STATUS_POLL_MS 30000
#endif

namespace {

constexpr uint32_t kScanDurationSec = 10;
// A BLE scan blocks loop() for its whole duration, which stalls sampling
// and publishing for every connected sensor. Once at least one slot is
// streaming, a still-empty slot (auto mode has MAX_AUTO_SENSORS slots, so
// one sensor in range leaves one empty) is only retried this rarely, and
// with a shorter scan. Pin the sensor in SENSOR_ADDRESSES (and keep the
// list as long as the sensors you own) to avoid scanning altogether.
constexpr uint32_t kRescanWhileStreamingMs = 300000;
constexpr uint32_t kScanWhileStreamingSec = 4;
constexpr uint32_t kReconnectDelayMs = 3000;
// Minimum gap between WiFi / MQTT reconnect rounds. Keeps a missing
// network from stalling BLE sampling on every loop() pass.
constexpr uint32_t kWifiRetryIntervalMs = 30000;
constexpr uint32_t kMqttRetryIntervalMs = 5000;

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);

// One sensor per slot: SENSOR_ADDRESSES_COUNT slots pinned to those
// specific MACs, or MAX_AUTO_SENSORS slots filled by whichever WTVB01
// units are found first, if that list is left empty. Each slot owns an
// independent BLE client, decoder and reading buffer so the sensors
// stream concurrently instead of one displacing the other.
constexpr size_t kMaxSensors =
    SENSOR_ADDRESSES_COUNT > 0 ? SENSOR_ADDRESSES_COUNT : MAX_AUTO_SENSORS;

// ------------------------------------------------------ reading buffer
//
// Sampling (SampleIfDue, at PUBLISH_INTERVAL_MS cadence) and publishing
// (DrainReadingBuffer) are decoupled: sampling keeps running even while
// MQTT is down, stashing snapshots in a ring buffer, so a reconnect
// drains and republishes what was missed instead of silently resuming
// live and losing the gap. Only loop() touches this buffer, so no
// locking is needed beyond what already guards latest_reading.

struct BufferedReading {
  wtvb01::SensorReading reading;
  bool instant_mode = false;  // sensor was in Now data mode (values polled)
  uint32_t seq = 0;
  uint32_t captured_ms = 0;
};

struct SensorSlot {
  NimBLEClient *client = nullptr;

  // Set by ScanCallbacks during a scan pass, consumed by EnsureSensors
  // right after; only valid until scan->clearResults().
  NimBLEAdvertisedDevice *pending_target = nullptr;
  String pending_address;

  wtvb01::Decoder decoder;
  String address;  // peer address once connected; MQTT topic suffix

  // Written from the BLE notification callback, read from loop().
  volatile bool has_reading = false;
  portMUX_TYPE reading_mux = portMUX_INITIALIZER_UNLOCKED;
  wtvb01::SensorReading latest_reading;

  // "Now data" waveform path. The BLE task fills `waveform` through the
  // decoder's sink; when a window completes it is copied into wf_ready
  // under wf_mux and published from loop(). A single pending window is
  // kept: if loop() has not published the previous one yet, the new one
  // is dropped and counted, never queued (windows are too big to buffer
  // through an MQTT outage).
  NimBLERemoteCharacteristic *write_char = nullptr;  // ffe9, may be null
  // Now data mode: derived readings and the rare status poll.
  wtvb01::InstantStats stats;    // BLE task adds, loop() reads (wf_mux)
  float derived_freq[3] = {0, 0, 0};  // loop() only: FFT of the last window
  uint32_t last_status_poll_ms = 0;
  uint8_t status_poll_stage = 0;  // 0 idle, 1 = battery read still to send

  wtvb01::WaveformAccumulator waveform;
  portMUX_TYPE wf_mux = portMUX_INITIALIZER_UNLOCKED;
  volatile bool wf_pending = false;
  wtvb01::WaveformWindow wf_ready;
  uint32_t wf_seq = 0;
  uint32_t wf_dropped = 0;  // loop() was still busy with the previous window
  uint32_t wf_unsent = 0;   // MQTT was down or the publish failed

  BufferedReading reading_buffer[READING_BUFFER_CAPACITY];
  size_t buffer_head = 0;   // index of the oldest buffered sample
  size_t buffer_count = 0;  // how many entries are in use
  uint32_t next_seq = 0;
  uint32_t dropped_samples = 0;
  uint32_t last_sample_ms = 0;
};

SensorSlot sensors[kMaxSensors];

// ---------------------------------------------------------------- BLE

// Runs on the BLE task, once per "Now data" packet, in stream order.
void OnInstantSample(const wtvb01::InstantSample &sample, void *ctx) {
  SensorSlot *slot = static_cast<SensorSlot *>(ctx);

  portENTER_CRITICAL(&slot->wf_mux);
  slot->stats.Add(sample);
  portEXIT_CRITICAL(&slot->wf_mux);

#if WAVEFORM_ENABLE
  if (!slot->waveform.Push(sample)) {
    return;
  }
  portENTER_CRITICAL(&slot->wf_mux);
  if (slot->wf_pending) {
    slot->wf_dropped++;
  } else {
    slot->wf_ready = slot->waveform.window();
    slot->wf_pending = true;
  }
  portEXIT_CRITICAL(&slot->wf_mux);
#endif
}

#if SENSOR_CONFIGURE_RATE || SENSOR_CONFIGURE_DATA_MODE
// Sends unlock -> write -> save the way the official app does, with
// kCommandSpacingMs between commands (the manual does not mention the
// delay; it was observed on the wire). The writes persist in the sensor.
bool WriteSensorRegister(NimBLERemoteCharacteristic *write_char, uint8_t reg,
                         uint16_t value) {
  uint8_t cmd[wtvb01::kCommandLen];
  bool ok = true;

  wtvb01::BuildWriteRegisterCommand(wtvb01::kRegUnlock, wtvb01::kUnlockValue,
                                    cmd);
  ok &= write_char->writeValue(cmd, sizeof(cmd), false);
  delay(wtvb01::kCommandSpacingMs);

  wtvb01::BuildWriteRegisterCommand(reg, value, cmd);
  ok &= write_char->writeValue(cmd, sizeof(cmd), false);
  delay(wtvb01::kCommandSpacingMs);

  wtvb01::BuildWriteRegisterCommand(wtvb01::kRegSave, 0x0000, cmd);
  ok &= write_char->writeValue(cmd, sizeof(cmd), false);
  return ok;
}
#endif  // SENSOR_CONFIGURE_RATE || SENSOR_CONFIGURE_DATA_MODE

void OnNotify(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len,
              bool) {
  NimBLEClient *client = chr->getClient();
  for (size_t i = 0; i < kMaxSensors; i++) {
    if (sensors[i].client != client) {
      continue;
    }
    if (!sensors[i].decoder.Feed(data, len)) {
      return;
    }
    portENTER_CRITICAL(&sensors[i].reading_mux);
    sensors[i].latest_reading = sensors[i].decoder.reading();
    sensors[i].has_reading = true;
    portEXIT_CRITICAL(&sensors[i].reading_mux);
    return;
  }
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *client, int reason) override {
    for (size_t i = 0; i < kMaxSensors; i++) {
      if (sensors[i].client == client) {
        Serial.printf("[ble] %s disconnected (reason %d)\n",
                      sensors[i].address.c_str(), reason);
        return;
      }
    }
    Serial.printf("[ble] disconnected (reason %d)\n", reason);
  }
};

ClientCallbacks client_callbacks;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *device) override {
    // Match on the service UUID rather than the name: it is the
    // reliable identifier, and some units advertise no local name.
    if (!device->isAdvertisingService(NimBLEUUID(wtvb01::kServiceUUID))) {
      return;
    }

    const String addr = device->getAddress().toString().c_str();

    for (size_t i = 0; i < kMaxSensors; i++) {
      if (sensors[i].client != nullptr && sensors[i].client->isConnected()) {
        continue;  // already streaming
      }
      if (sensors[i].pending_target != nullptr) {
        continue;  // already matched earlier in this scan pass
      }

      if (SENSOR_ADDRESSES_COUNT > 0) {
        // Pinned mode: slot i is reserved for SENSOR_ADDRESSES[i].
        if (addr != SENSOR_ADDRESSES[i]) {
          continue;
        }
      } else {
        // Auto mode: first-come, first-served, skipping an address
        // another slot already grabbed this pass.
        bool already_targeted = false;
        for (size_t j = 0; j < kMaxSensors; j++) {
          if (sensors[j].pending_target != nullptr &&
              sensors[j].pending_address == addr) {
            already_targeted = true;
            break;
          }
        }
        if (already_targeted) {
          continue;
        }
      }

      Serial.printf("[ble] found %s (%s) RSSI %d, slot %u\n", addr.c_str(),
                    device->getName().c_str(), device->getRSSI(),
                    (unsigned)i);
      sensors[i].pending_target = const_cast<NimBLEAdvertisedDevice *>(device);
      sensors[i].pending_address = addr;
      break;
    }

    for (size_t i = 0; i < kMaxSensors; i++) {
      const bool connected =
          sensors[i].client != nullptr && sensors[i].client->isConnected();
      if (!connected && sensors[i].pending_target == nullptr) {
        return;  // still missing at least one slot; keep scanning
      }
    }
    NimBLEDevice::getScan()->stop();  // every slot is filled
  }
};

ScanCallbacks scan_callbacks;

// Connects one slot's pending_target (set during the scan pass just
// run) and subscribes to notifications. Leaves pending_target consumed
// either way.
bool ConnectSlot(SensorSlot &slot, size_t index) {
  NimBLEAdvertisedDevice *dev = slot.pending_target;
  slot.pending_target = nullptr;
  if (dev == nullptr) {
    return false;
  }

  if (slot.client == nullptr) {
    slot.client = NimBLEDevice::createClient();
    slot.client->setClientCallbacks(&client_callbacks, false);
  }

  if (!slot.client->connect(dev)) {
    Serial.printf("[ble] slot %u connect failed\n", (unsigned)index);
    return false;
  }

  // Fresh stream state for this (re)connection: drop any half packet
  // left over from the previous link.
  slot.decoder = wtvb01::Decoder();
  slot.decoder.SetInstantSink(OnInstantSample, &slot);
  slot.stats.Reset();
  slot.waveform.Reset();
  slot.wf_pending = false;

  NimBLERemoteService *service =
      slot.client->getService(NimBLEUUID(wtvb01::kServiceUUID));
  if (service == nullptr) {
    Serial.printf("[ble] slot %u service ffe5 not found\n", (unsigned)index);
    slot.client->disconnect();
    return false;
  }

  NimBLERemoteCharacteristic *notify_char =
      service->getCharacteristic(NimBLEUUID(wtvb01::kNotifyCharUUID));
  if (notify_char == nullptr || !notify_char->canNotify()) {
    Serial.printf("[ble] slot %u notify characteristic ffe4 unavailable\n",
                  (unsigned)index);
    slot.client->disconnect();
    return false;
  }

  if (!notify_char->subscribe(true, OnNotify)) {
    Serial.printf("[ble] slot %u subscribe failed\n", (unsigned)index);
    slot.client->disconnect();
    return false;
  }

  slot.write_char =
      service->getCharacteristic(NimBLEUUID(wtvb01::kWriteCharUUID));
  if (slot.write_char != nullptr && !slot.write_char->canWrite()) {
    slot.write_char = nullptr;
  }

#if SENSOR_CONFIGURE_RATE || SENSOR_CONFIGURE_DATA_MODE
  // Optional writes, off by default -- see config.example.h. They run
  // once per (re)connect; a failure does not disconnect the slot, since
  // the sensor keeps streaming with whatever it already has.
  if (slot.write_char == nullptr) {
    Serial.printf(
        "[ble] slot %u write characteristic ffe9 unavailable -- sensor "
        "configuration left unchanged\n",
        (unsigned)index);
  } else {
#if SENSOR_CONFIGURE_RATE
    const bool rate_ok = WriteSensorRegister(
        slot.write_char, wtvb01::kRegRate, static_cast<uint16_t>(SENSOR_RATE_CODE));
    Serial.printf(
        "[ble] slot %u requested output rate code 0x%02X (%s) -- verify the "
        "packet cadence actually changed before trusting this\n",
        (unsigned)index, SENSOR_RATE_CODE, rate_ok ? "sent" : "write failed");
#endif
#if SENSOR_CONFIGURE_DATA_MODE
    // Only write when the sensor is not already in the wanted mode: every
    // write ends with a "save", and there is no reason to wear the
    // sensor's flash on each reconnect. The mode shows up in the packet
    // size within a notification or two.
    const wtvb01::DataMode wanted = SENSOR_DATA_MODE_INSTANT
                                        ? wtvb01::DataMode::kInstant
                                        : wtvb01::DataMode::kDefault;
    for (int i = 0; i < 30 && slot.decoder.mode() == wtvb01::DataMode::kUnknown; i++) {
      delay(50);
    }
    if (slot.decoder.mode() == wanted) {
      Serial.printf("[ble] slot %u already in the wanted data mode; no write\n",
                    (unsigned)index);
    } else {
      const uint16_t mode = SENSOR_DATA_MODE_INSTANT ? wtvb01::kDataModeInstant
                                                     : wtvb01::kDataModeDefault;
      const bool mode_ok =
          WriteSensorRegister(slot.write_char, wtvb01::kRegDataMode, mode);
      // The sensor starts sending the new packet size right away; tell the
      // decoder so it does not wait to detect it. A wrong hint corrects
      // itself.
      slot.decoder.SetExpectedOutputLength(SENSOR_DATA_MODE_INSTANT
                                               ? wtvb01::kInstantPacketLen
                                               : wtvb01::kOutputPacketLen);
      Serial.printf(
          "[ble] slot %u data mode register 0x%02X <- %u (%s, persists in the "
          "sensor)\n",
          (unsigned)index, wtvb01::kRegDataMode, (unsigned)mode,
          mode_ok ? "sent" : "write failed");
    }
#endif
  }
#endif

  slot.address = slot.client->getPeerAddress().toString().c_str();
  Serial.printf("[ble] slot %u streaming from %s (ATT MTU %u)\n",
                (unsigned)index, slot.address.c_str(),
                (unsigned)slot.client->getMTU());
  return true;
}

// Scans once (unless every slot is already connected) and connects
// whatever slots the scan pass matched.
void EnsureSensors() {
  bool need_scan = false;
  for (size_t i = 0; i < kMaxSensors; i++) {
    if (sensors[i].client == nullptr || !sensors[i].client->isConnected()) {
      need_scan = true;
      break;
    }
  }
  if (!need_scan) {
    return;
  }

  // Throttle rescans while something is already streaming (see
  // kRescanWhileStreamingMs): a scan would stall the live sensor's feed.
  static uint32_t last_scan_ms = 0;
  static bool scanned_once = false;
  bool any_connected = false;
  for (size_t i = 0; i < kMaxSensors; i++) {
    if (sensors[i].client != nullptr && sensors[i].client->isConnected()) {
      any_connected = true;
      break;
    }
  }
  if (any_connected && scanned_once &&
      millis() - last_scan_ms < kRescanWhileStreamingMs) {
    return;
  }

  Serial.println("[ble] scanning...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scan_callbacks, false);
  scan->setActiveScan(true);
  scan->getResults(
      (any_connected ? kScanWhileStreamingSec : kScanDurationSec) * 1000,
      false);
  scanned_once = true;
  last_scan_ms = millis();

  bool any_found = false;
  for (size_t i = 0; i < kMaxSensors; i++) {
    if (sensors[i].pending_target != nullptr) {
      any_found = true;
      ConnectSlot(sensors[i], i);
    }
  }
  if (!any_found) {
    Serial.println("[ble] no sensors found");
    Serial.println("      are they powered on, and not held by the phone app?");
  }
  scan->clearResults();
}

// --------------------------------------------------------- WiFi / MQTT

// Diagnostic: scans and logs every network the radio can actually see,
// flagging which ones match an entry in WIFI_NETWORKS. Helps tell apart
// "AP out of range" from "AP visible but rejects the connection".
// Which entries of WIFI_NETWORKS the last scan actually saw. EnsureWiFi uses
// it to skip networks that are out of range instead of waiting
// WIFI_TRY_TIMEOUT_MS on each of them (a configured-but-absent hotspot cost
// ~8 s of boot time per entry).
bool network_visible[WIFI_NETWORKS_COUNT];

void LogVisibleNetworks() {
  Serial.println("[wifi] scanning...");
  for (size_t j = 0; j < WIFI_NETWORKS_COUNT; j++) {
    network_visible[j] = false;
  }
  const int count = WiFi.scanNetworks();
  if (count <= 0) {
    Serial.println("[wifi] scan found nothing");
    return;
  }
  for (int i = 0; i < count; i++) {
    bool known = false;
    for (size_t j = 0; j < WIFI_NETWORKS_COUNT; j++) {
      if (WiFi.SSID(i) == WIFI_NETWORKS[j].ssid) {
        known = true;
        network_visible[j] = true;
        break;
      }
    }
    Serial.printf("[wifi]   %-24s ch=%2d rssi=%4d auth=%d%s\n",
                  WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i),
                  WiFi.encryptionType(i), known ? "  <- configured" : "");
  }
  WiFi.scanDelete();
}

// Tries one SSID, giving up after WIFI_TRY_TIMEOUT_MS.
bool TryWiFiNetwork(const char *ssid, const char *password) {
  Serial.printf("[wifi] connecting to %s\n", ssid);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WIFI_TRY_TIMEOUT_MS) {
      Serial.printf("\n[wifi] %s timed out, status=%d\n", ssid,
                    WiFi.status());
      return false;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[wifi] connected to %s, ip %s\n", ssid,
                WiFi.localIP().toString().c_str());
  return true;
}

// Index of the network that last connected successfully. A reconnect
// after a transient blip is usually the same AP coming back, so trying
// it first (instead of always restarting from WIFI_NETWORKS[0]) avoids
// wasting up to (index * WIFI_TRY_TIMEOUT_MS) on networks that are not
// even in range right now.
size_t last_good_network = 0;

// Tries every network in WIFI_NETWORKS (src/config.h) once, starting
// from last_good_network and wrapping around. Returns true if connected.
// Never blocks forever: after a failed round it backs off for
// kWifiRetryIntervalMs, so BLE sampling keeps running with no network.
bool EnsureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  static bool attempted = false;
  static uint32_t last_round_ms = 0;
  if (attempted && millis() - last_round_ms < kWifiRetryIntervalMs) {
    return false;
  }
  attempted = true;

  WiFi.mode(WIFI_STA);
  LogVisibleNetworks();
  // Only try networks the scan saw. If it saw none of them (a hidden SSID
  // does not show up in scans, or the scan failed), fall back to trying
  // every entry in order.
  bool any_visible = false;
  for (size_t j = 0; j < WIFI_NETWORKS_COUNT; j++) {
    any_visible = any_visible || network_visible[j];
  }
  for (size_t k = 0; k < WIFI_NETWORKS_COUNT; k++) {
    const size_t i = (last_good_network + k) % WIFI_NETWORKS_COUNT;
    if (any_visible && !network_visible[i]) {
      continue;
    }
    if (TryWiFiNetwork(WIFI_NETWORKS[i].ssid, WIFI_NETWORKS[i].password)) {
      last_good_network = i;
      return true;
    }
  }
  last_round_ms = millis();
  Serial.println("[wifi] no network available; sampling continues offline");
  return false;
}

// One connect attempt per kMqttRetryIntervalMs. Returns true if connected.
bool EnsureMQTT() {
  if (mqtt.connected()) {
    return true;
  }

  static uint32_t last_attempt_ms = 0;
  static bool attempted = false;
  if (attempted && millis() - last_attempt_ms < kMqttRetryIntervalMs) {
    return false;
  }
  attempted = true;
  last_attempt_ms = millis();

  const String client_id = "zenith-" + WiFi.macAddress();
  Serial.printf("[mqtt] connecting to %s:%d\n", MQTT_HOST, MQTT_PORT);

  // The last will publishes "offline" if this node drops off without
  // saying goodbye, so a dead node is visible on the broker.
  const bool ok =
      mqtt.connect(client_id.c_str(),
                   strlen(MQTT_USER) > 0 ? MQTT_USER : nullptr,
                   strlen(MQTT_PASSWORD) > 0 ? MQTT_PASSWORD : nullptr,
                   MQTT_TOPIC_STATUS, 0, true, "offline");
  if (ok) {
    Serial.println("[mqtt] connected");
    mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
    return true;
  }

  Serial.printf("[mqtt] failed, rc=%d; will retry\n", mqtt.state());
  return false;
}

// Returns false if the broker write failed (transient broker-side
// hiccups on the public test broker are common; see docs/broker-migration.md),
// in which case the caller must keep the sample queued and retry rather
// than drop it.
bool PublishReading(SensorSlot &slot, const BufferedReading &buffered) {
  const wtvb01::SensorReading &r = buffered.reading;

  // Schema matches wtvb01.SensorReading in the Go collector, plus seq
  // and published_at_ms which only the ESP32 node adds. seq lets a
  // consumer detect gaps exactly (a skip in the sequence is a lost or
  // not-yet-drained sample) instead of inferring loss from uptime_ms
  // deltas. published_at_ms is stamped right before the MQTT write, so
  // published_at_ms - uptime_ms is queueing/buffering delay: near zero
  // in steady state, large right after a reconnect drains a backlog.
  JsonDocument doc;
  doc["sensor"] = slot.address;
  doc["seq"] = buffered.seq;
  doc["uptime_ms"] = buffered.captured_ms;
  doc["published_at_ms"] = millis();
  // Default-mode amplitudes. "Now data" samples are not published here:
  // they go to MQTT_TOPIC_WAVEFORM_BASE (see PublishWaveformIfReady).
  doc["mode"] = buffered.instant_mode ? "instant" : "default";

  JsonObject velocity = doc["velocity"].to<JsonObject>();
  velocity["x"] = r.velocity.x;
  velocity["y"] = r.velocity.y;
  velocity["z"] = r.velocity.z;

  JsonObject displacement = doc["displacement"].to<JsonObject>();
  displacement["x"] = r.displacement.x;
  displacement["y"] = r.displacement.y;
  displacement["z"] = r.displacement.z;

  JsonObject angle = doc["angle"].to<JsonObject>();
  angle["x"] = r.angle.x;
  angle["y"] = r.angle.y;
  angle["z"] = r.angle.z;

  JsonObject frequency = doc["frequency"].to<JsonObject>();
  frequency["x"] = r.frequency.x;
  frequency["y"] = r.frequency.y;
  frequency["z"] = r.frequency.z;

  // Module temperature, not the machine's. See docs/indicators.md.
  JsonObject device = doc["device"].to<JsonObject>();
  device["temperature"] = r.device.temperature;
  // power_raw is the battery register 0x64 (centivolts), kept under its
  // old name for existing consumers; battery_v / battery_pct are derived
  // with the official app's table.
  device["power_raw"] = r.device.power_raw;
  device["battery_v"] = r.device.battery_volts;
  device["battery_pct"] = r.device.battery_percent;
  device["alarm"] = r.device.alarm_status;
  device["rssi"] = slot.client != nullptr ? slot.client->getRssi() : 0;

  char payload[640];
  const size_t n = serializeJson(doc, payload, sizeof(payload));

  const String topic = String(MQTT_TOPIC_BASE) + "/" + slot.address;
  if (!mqtt.publish(topic.c_str(), payload, n)) {
    Serial.printf("[mqtt] publish failed (seq=%lu)\n",
                  (unsigned long)buffered.seq);
    return false;
  }

  Serial.printf(
      "seq=%lu vel(%.1f,%.1f,%.1f)mm/s disp(%.0f,%.0f,%.0f)um "
      "freq(%.0f,%.0f,%.0f)Hz temp=%.1fC bat=%.2fV(%.0f%%)\n",
      (unsigned long)buffered.seq, r.velocity.x, r.velocity.y, r.velocity.z,
      r.displacement.x, r.displacement.y, r.displacement.z, r.frequency.x,
      r.frequency.y, r.frequency.z, r.device.temperature,
      r.device.battery_volts, r.device.battery_percent);
  return true;
}

#if WAVEFORM_ENABLE
// Binary waveform frame, little-endian (the ESP32 and the consumers we use
// are all little-endian, so fields are copied as they are). 48-byte header
// followed by three int16 blocks of `n` samples each: ax[n], ay[n], az[n].
// Raw sensor counts, so nothing is lost to rounding; multiply by
// g_per_count for g. The sensor MAC is the last topic level. Versus JSON
// this is ~3.4x smaller (1584 B vs ~5.4 KB for 256 samples) and costs only
// memcpy on the ESP32.
#pragma pack(push, 1)
struct WaveformFrameHeader {
  char magic[4];  // "ZWF1" (format version in the last byte)
  uint16_t n;     // samples per axis
  uint8_t axes;   // 3
  uint8_t reserved;
  uint32_t seq;            // per-sensor frame counter
  uint32_t chip_start_ms;  // sensor chip clock of the first sample
  uint32_t chip_end_ms;    // ... of the last sample
  uint32_t gaps;           // gaps that forced this window to restart
  uint32_t dropped;        // windows lost: loop() was still busy (total)
  uint32_t unsent;         // windows lost: MQTT down / publish failed (total)
  float fs_hz;             // sample rate implied by the chip clock
  float g_per_count;       // acceleration scale: g = count * g_per_count
  uint32_t uptime_ms;      // ESP32 millis() when the frame was built
  uint32_t published_at_ms;
};
#pragma pack(pop)
static_assert(sizeof(WaveformFrameHeader) == 48, "waveform header layout");

constexpr size_t kWaveformFrameBytes =
    sizeof(WaveformFrameHeader) + 3 * sizeof(int16_t) * WAVEFORM_WINDOW_SAMPLES;
// The MQTT client buffer must also hold the topic and the MQTT header.
constexpr size_t kMqttBufferBytes = kWaveformFrameBytes + 160;

uint8_t waveform_frame[kWaveformFrameBytes];
// Only loop() touches this copy, so every slot can share it.
wtvb01::WaveformWindow waveform_local;

// Publishes the window the BLE task completed, if any. A window is only
// worth its FFT if it is contiguous, so it is never buffered through an
// outage: if MQTT is down the window is counted in wf_unsent and dropped.
void PublishWaveformIfReady(SensorSlot &slot) {
  if (!slot.wf_pending) {
    return;
  }
  portENTER_CRITICAL(&slot.wf_mux);
  waveform_local = slot.wf_ready;
  slot.wf_pending = false;
  portEXIT_CRITICAL(&slot.wf_mux);

  // Dominant frequency per axis for zenith/readings' `frequency`, from the
  // same window that is about to be published. A 256-point FFT x3 is well
  // under a millisecond on the ESP32.
  wtvb01::DominantFrequencies(waveform_local, waveform_local.SampleRateHz(),
                              slot.derived_freq);

  if (!mqtt.connected()) {
    slot.wf_unsent++;
    return;
  }

  const wtvb01::WaveformWindow &w = waveform_local;
  const size_t n = w.count;
  const uint32_t now = millis();

  WaveformFrameHeader h = {};
  memcpy(h.magic, "ZWF1", 4);
  h.n = static_cast<uint16_t>(n);
  h.axes = 3;
  h.seq = slot.wf_seq;
  h.chip_start_ms = w.start_chip_ms;
  h.chip_end_ms = w.end_chip_ms;
  h.gaps = w.gaps;
  h.dropped = slot.wf_dropped;
  h.unsent = slot.wf_unsent;
  h.fs_hz = w.SampleRateHz();
  h.g_per_count = wtvb01::kAccelScaleG;
  h.uptime_ms = now;
  h.published_at_ms = now;

  uint8_t *out = waveform_frame;
  memcpy(out, &h, sizeof(h));
  out += sizeof(h);
  memcpy(out, w.ax, n * sizeof(int16_t));
  out += n * sizeof(int16_t);
  memcpy(out, w.ay, n * sizeof(int16_t));
  out += n * sizeof(int16_t);
  memcpy(out, w.az, n * sizeof(int16_t));
  out += n * sizeof(int16_t);
  const size_t bytes = static_cast<size_t>(out - waveform_frame);

  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_WAVEFORM_BASE,
           slot.address.c_str());
  if (!mqtt.publish(topic, waveform_frame, bytes, false)) {
    Serial.printf("[wave] publish failed (seq=%lu, %u bytes)\n",
                  (unsigned long)slot.wf_seq, (unsigned)bytes);
    slot.wf_unsent++;
    return;
  }
  Serial.printf(
      "[wave] seq=%lu n=%u fs=%.2fHz gaps=%lu dropped=%lu unsent=%lu "
      "bytes=%u\n",
      (unsigned long)slot.wf_seq, (unsigned)n, h.fs_hz, (unsigned long)w.gaps,
      (unsigned long)slot.wf_dropped, (unsigned long)slot.wf_unsent,
      (unsigned)bytes);
  slot.wf_seq++;
}
#else
void PublishWaveformIfReady(SensorSlot &) {}
#endif

// Now data mode has no temperature or battery in its packets, so ask the
// sensor for them now and then: register block 0x3A..0x41 (carries the
// temperature at 0x40) and then 0x64 (battery). The two reads are sent one
// loop pass apart, the way the official app spaces its commands. Read
// commands change nothing in the sensor. Each pair makes the waveform
// window being built restart once, which is why this is rare.
#if SENSOR_STATUS_POLL_MS > 0
void StatusPollIfDue(SensorSlot &slot) {
  if (slot.write_char == nullptr ||
      slot.decoder.mode() != wtvb01::DataMode::kInstant) {
    return;
  }
  const uint32_t now = millis();
  uint8_t cmd[wtvb01::kCommandLen];
  if (slot.status_poll_stage == 0) {
    if (slot.last_status_poll_ms != 0 &&
        now - slot.last_status_poll_ms < SENSOR_STATUS_POLL_MS) {
      return;
    }
    wtvb01::BuildReadRegisterCommand(wtvb01::kRegVelocityX, cmd);
    slot.write_char->writeValue(cmd, sizeof(cmd), false);
    slot.last_status_poll_ms = now;
    slot.status_poll_stage = 1;
  } else if (now - slot.last_status_poll_ms >= wtvb01::kCommandSpacingMs) {
    wtvb01::BuildReadRegisterCommand(wtvb01::kRegBattery, cmd);
    slot.write_char->writeValue(cmd, sizeof(cmd), false);
    slot.status_poll_stage = 0;
  }
}
#else
void StatusPollIfDue(SensorSlot &) {}
#endif

// Snapshots the latest decoded BLE reading into the ring buffer at
// PUBLISH_INTERVAL_MS cadence, independent of MQTT connectivity. If the
// buffer is already full, the oldest unpublished sample is overwritten
// and counted in dropped_samples — that only happens once an outage has
// outlasted READING_BUFFER_CAPACITY seconds of backlog.
void SampleIfDue(SensorSlot &slot) {
  const uint32_t now = millis();
  if (now - slot.last_sample_ms < PUBLISH_INTERVAL_MS) {
    return;
  }

  wtvb01::SensorReading reading;
  const bool instant = slot.decoder.mode() == wtvb01::DataMode::kInstant;
  if (instant) {
    // Now data mode: the sensor sends no amplitudes, so velocity /
    // displacement / angle are the smoothed RMS of the signed samples, the
    // frequency comes from the last waveform window's FFT, and temperature
    // and battery from the rare status poll (kept in latest_reading).
    wtvb01::SensorReading derived;
    portENTER_CRITICAL(&slot.wf_mux);
    const bool ok = slot.stats.Rms(&derived);
    portEXIT_CRITICAL(&slot.wf_mux);
    portENTER_CRITICAL(&slot.reading_mux);
    reading = slot.latest_reading;
    portEXIT_CRITICAL(&slot.reading_mux);
    if (!ok) {
      return;
    }
    reading.velocity = derived.velocity;
    reading.displacement = derived.displacement;
    reading.angle = derived.angle;
    reading.frequency = {slot.derived_freq[0], slot.derived_freq[1],
                         slot.derived_freq[2]};
  } else {
    if (!slot.has_reading) {
      return;
    }
    portENTER_CRITICAL(&slot.reading_mux);
    reading = slot.latest_reading;
    slot.has_reading = false;
    portEXIT_CRITICAL(&slot.reading_mux);
  }

  if (slot.buffer_count == READING_BUFFER_CAPACITY) {
    slot.dropped_samples++;
    Serial.printf(
        "[buffer] %s full, dropping oldest sample (seq=%lu), "
        "dropped_total=%lu\n",
        slot.address.c_str(),
        (unsigned long)slot.reading_buffer[slot.buffer_head].seq,
        (unsigned long)slot.dropped_samples);
    slot.buffer_head = (slot.buffer_head + 1) % READING_BUFFER_CAPACITY;
    slot.buffer_count--;
  }

  const size_t idx =
      (slot.buffer_head + slot.buffer_count) % READING_BUFFER_CAPACITY;
  slot.reading_buffer[idx] = {reading, slot.decoder.mode() == wtvb01::DataMode::kInstant,
                              slot.next_seq++, now};
  slot.buffer_count++;
  slot.last_sample_ms = now;
}

// Drains the ring buffer into MQTT as fast as the broker accepts it. In
// steady state this publishes the single sample SampleIfDue just
// pushed; after an outage it burns through the whole backlog in one go
// instead of resuming live and leaving the gap unpublished.
//
// A failed publish() leaves the sample queued instead of discarding it:
// on test.mosquitto.org, mqtt.connected() can still read true while a
// single write transiently fails (measured: ~10-20s stalls with no
// disconnect logged, see docs/broker-migration.md). Without this retry,
// every one of those hiccups silently dropped exactly the sample that
// was in flight -- confirmed on the bench by correlating serial logs
// against the subscriber feed. Stop after one failure per call so a
// broker that is genuinely down doesn't spin this loop forever; the
// next loop() iteration (and EnsureWiFi/EnsureMQTT ahead of it) gets a
// chance to run before retrying.
void DrainReadingBuffer(SensorSlot &slot) {
  while (slot.buffer_count > 0 && mqtt.connected()) {
    if (!PublishReading(slot, slot.reading_buffer[slot.buffer_head])) {
      break;
    }
    slot.buffer_head = (slot.buffer_head + 1) % READING_BUFFER_CAPACITY;
    slot.buffer_count--;
    mqtt.loop();
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nZenith Edge node starting");

  NimBLEDevice::init("zenith-edge-node");
  // The sensor is a low-power peripheral; boosting TX power helps at
  // range in a noisy plant.
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
#if WAVEFORM_ENABLE
  // Big enough for one waveform window plus topic and MQTT header.
  if (!mqtt.setBufferSize(kMqttBufferBytes)) {
    Serial.printf("[mqtt] could not allocate a %u-byte buffer\n",
                  (unsigned)kMqttBufferBytes);
  }
#else
  mqtt.setBufferSize(768);
#endif
  // Default keepalive (15s) is shorter than the BLE scan alone (10s),
  // so a scan plus any other blocking work could starve mqtt.loop()
  // long enough for the broker to drop the connection on its own. See
  // config.h for the full reasoning.
  mqtt.setKeepAlive(MQTT_KEEPALIVE_SEC);
  // PubSubClient::setSocketTimeout() only bounds its own MQTT-level ack
  // waits (e.g. CONNACK); it does NOT touch the underlying TCP socket,
  // so a stalled write() still blocked for 10-20s even with it set.
  // WiFiClient::setTimeout() is what actually applies SO_SNDTIMEO /
  // SO_RCVTIMEO to the socket -- that's the one that bounds a stuck
  // publish(). Measured: see config.h.
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);
  wifi_client.setTimeout(MQTT_SOCKET_TIMEOUT_SEC);
}

void loop() {
  if (EnsureWiFi() && EnsureMQTT()) {
    mqtt.loop();
  }

  EnsureSensors();

  // Sampling runs regardless of MQTT state, so an outage accumulates a
  // backlog in the ring buffer instead of losing the readings generated
  // while offline. Draining only happens while connected, and catches
  // up on any backlog as soon as a reconnect lands. Every connected
  // slot is serviced each loop() pass, independent of the others.
  bool any_connected = false;
  for (size_t i = 0; i < kMaxSensors; i++) {
    if (sensors[i].client == nullptr || !sensors[i].client->isConnected()) {
      continue;
    }
    any_connected = true;
    StatusPollIfDue(sensors[i]);
    SampleIfDue(sensors[i]);
    DrainReadingBuffer(sensors[i]);
    PublishWaveformIfReady(sensors[i]);
  }

  if (!any_connected) {
    delay(kReconnectDelayMs);
    return;
  }
  delay(10);
}
