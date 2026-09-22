// Zenith Edge node: reads a WitMotion WTVB01-BT50 over BLE and
// publishes normalized readings to MQTT.
//
// The sensor broadcasts every measurement register in its 0x61 packet
// without being asked, so this node never writes to the sensor: it
// scans, connects, subscribes to the notify characteristic, and parses.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "config.h"
#include "wtvb01.h"

namespace {

constexpr uint32_t kScanDurationSec = 10;
constexpr uint32_t kReconnectDelayMs = 3000;

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

  BufferedReading reading_buffer[READING_BUFFER_CAPACITY];
  size_t buffer_head = 0;   // index of the oldest buffered sample
  size_t buffer_count = 0;  // how many entries are in use
  uint32_t next_seq = 0;
  uint32_t dropped_samples = 0;
  uint32_t last_sample_ms = 0;
};

SensorSlot sensors[kMaxSensors];

// ---------------------------------------------------------------- BLE

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

  slot.address = slot.client->getPeerAddress().toString().c_str();
  Serial.printf("[ble] slot %u streaming from %s\n", (unsigned)index,
                slot.address.c_str());
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

  Serial.println("[ble] scanning...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scan_callbacks, false);
  scan->setActiveScan(true);
  scan->getResults(kScanDurationSec * 1000, false);

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
void LogVisibleNetworks() {
  Serial.println("[wifi] scanning...");
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

// Tries every network in WIFI_NETWORKS (src/config.h), starting from
// last_good_network and wrapping around, looping forever until one
// connects.
void EnsureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.mode(WIFI_STA);
  LogVisibleNetworks();
  do {
    for (size_t k = 0; k < WIFI_NETWORKS_COUNT; k++) {
      const size_t i = (last_good_network + k) % WIFI_NETWORKS_COUNT;
      if (TryWiFiNetwork(WIFI_NETWORKS[i].ssid, WIFI_NETWORKS[i].password)) {
        last_good_network = i;
        return;
      }
    }
  } while (true);
}

void EnsureMQTT() {
  if (mqtt.connected()) {
    return;
  }

  const String client_id = "zenith-" + WiFi.macAddress();

  while (!mqtt.connected()) {
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
      return;
    }

    Serial.printf("[mqtt] failed, rc=%d; retrying\n", mqtt.state());
    delay(kReconnectDelayMs);
  }
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
  device["power_raw"] = r.device.power_raw;
  device["rssi"] = slot.client != nullptr ? slot.client->getRssi() : 0;

  char payload[512];
  const size_t n = serializeJson(doc, payload, sizeof(payload));

  const String topic = String(MQTT_TOPIC_BASE) + "/" + slot.address;
  if (!mqtt.publish(topic.c_str(), payload, n)) {
    Serial.printf("[mqtt] publish failed (seq=%lu)\n",
                  (unsigned long)buffered.seq);
    return false;
  }

  Serial.printf(
      "seq=%lu vel(%.1f,%.1f,%.1f)mm/s disp(%.0f,%.0f,%.0f)um "
      "freq(%.0f,%.0f,%.0f)Hz temp=%.1fC power_raw=%.0f\n",
      (unsigned long)buffered.seq, r.velocity.x, r.velocity.y, r.velocity.z,
      r.displacement.x, r.displacement.y, r.displacement.z, r.frequency.x,
      r.frequency.y, r.frequency.z, r.device.temperature, r.device.power_raw);
  return true;
}

// Snapshots the latest decoded BLE reading into the ring buffer at
// PUBLISH_INTERVAL_MS cadence, independent of MQTT connectivity. If the
// buffer is already full, the oldest unpublished sample is overwritten
// and counted in dropped_samples — that only happens once an outage has
// outlasted READING_BUFFER_CAPACITY seconds of backlog.
void SampleIfDue(SensorSlot &slot) {
  const uint32_t now = millis();
  if (now - slot.last_sample_ms < PUBLISH_INTERVAL_MS || !slot.has_reading) {
    return;
  }

  wtvb01::SensorReading reading;
  portENTER_CRITICAL(&slot.reading_mux);
  reading = slot.latest_reading;
  slot.has_reading = false;
  portEXIT_CRITICAL(&slot.reading_mux);

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
  slot.reading_buffer[idx] = {reading, slot.next_seq++, now};
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

  EnsureWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(768);
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
  EnsureMQTT();

  NimBLEDevice::init("zenith-edge-node");
  // The sensor is a low-power peripheral; boosting TX power helps at
  // range in a noisy plant.
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
}

void loop() {
  EnsureWiFi();
  EnsureMQTT();
  mqtt.loop();

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
    SampleIfDue(sensors[i]);
    DrainReadingBuffer(sensors[i]);
  }

  if (!any_connected) {
    delay(kReconnectDelayMs);
    return;
  }
  delay(10);
}
