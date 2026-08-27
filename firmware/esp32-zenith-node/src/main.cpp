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

wtvb01::Decoder decoder;

NimBLEClient *ble_client = nullptr;
NimBLEAdvertisedDevice *target = nullptr;

// Written from the BLE notification callback, read from loop().
volatile bool has_reading = false;
portMUX_TYPE reading_mux = portMUX_INITIALIZER_UNLOCKED;
wtvb01::SensorReading latest_reading;

String sensor_address;
uint32_t last_publish_ms = 0;

// ---------------------------------------------------------------- BLE

void OnNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len,
              bool) {
  if (!decoder.Feed(data, len)) {
    return;
  }
  portENTER_CRITICAL(&reading_mux);
  latest_reading = decoder.reading();
  has_reading = true;
  portEXIT_CRITICAL(&reading_mux);
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *, int reason) override {
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
    if (strlen(SENSOR_ADDRESS) > 0 &&
        !device->getAddress().toString().equals(SENSOR_ADDRESS)) {
      return;
    }

    Serial.printf("[ble] found %s (%s) RSSI %d\n",
                  device->getAddress().toString().c_str(),
                  device->getName().c_str(), device->getRSSI());

    target = const_cast<NimBLEAdvertisedDevice *>(device);
    NimBLEDevice::getScan()->stop();
  }
};

ScanCallbacks scan_callbacks;

bool ConnectToSensor() {
  target = nullptr;

  Serial.println("[ble] scanning...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scan_callbacks, false);
  scan->setActiveScan(true);
  scan->getResults(kScanDurationSec * 1000, false);

  if (target == nullptr) {
    Serial.println("[ble] sensor not found");
    Serial.println("      is it powered on, and not held by the phone app?");
    scan->clearResults();
    return false;
  }

  if (ble_client == nullptr) {
    ble_client = NimBLEDevice::createClient();
    ble_client->setClientCallbacks(&client_callbacks, false);
  }

  if (!ble_client->connect(target)) {
    Serial.println("[ble] connect failed");
    scan->clearResults();
    return false;
  }

  NimBLERemoteService *service =
      ble_client->getService(NimBLEUUID(wtvb01::kServiceUUID));
  if (service == nullptr) {
    Serial.println("[ble] service ffe5 not found");
    ble_client->disconnect();
    return false;
  }

  NimBLERemoteCharacteristic *notify_char =
      service->getCharacteristic(NimBLEUUID(wtvb01::kNotifyCharUUID));
  if (notify_char == nullptr || !notify_char->canNotify()) {
    Serial.println("[ble] notify characteristic ffe4 unavailable");
    ble_client->disconnect();
    return false;
  }

  if (!notify_char->subscribe(true, OnNotify)) {
    Serial.println("[ble] subscribe failed");
    ble_client->disconnect();
    return false;
  }

  sensor_address = ble_client->getPeerAddress().toString().c_str();
  Serial.printf("[ble] streaming from %s\n", sensor_address.c_str());
  scan->clearResults();
  return true;
}

// --------------------------------------------------------- WiFi / MQTT

void EnsureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.printf("[wifi] connecting to %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[wifi] connected, ip %s\n", WiFi.localIP().toString().c_str());
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

void PublishReading(const wtvb01::SensorReading &r) {
  // Schema matches wtvb01.SensorReading in the Go collector, so both
  // are interchangeable downstream.
  JsonDocument doc;
  doc["sensor"] = sensor_address;
  doc["uptime_ms"] = millis();

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
  device["rssi"] = ble_client != nullptr ? ble_client->getRssi() : 0;

  char payload[512];
  const size_t n = serializeJson(doc, payload, sizeof(payload));

  const String topic = String(MQTT_TOPIC_BASE) + "/" + sensor_address;
  if (!mqtt.publish(topic.c_str(), payload, n)) {
    Serial.println("[mqtt] publish failed");
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

  if (ble_client == nullptr || !ble_client->isConnected()) {
    if (!ConnectToSensor()) {
      delay(kReconnectDelayMs);
      return;
    }
  }

  const uint32_t now = millis();
  if (now - last_publish_ms < PUBLISH_INTERVAL_MS) {
    delay(10);
    return;
  }

  if (!has_reading) {
    delay(10);
    return;
  }

  wtvb01::SensorReading reading;
  portENTER_CRITICAL(&reading_mux);
  reading = latest_reading;
  has_reading = false;
  portEXIT_CRITICAL(&reading_mux);

  PublishReading(reading);
  last_publish_ms = now;

  Serial.printf(
      "vel(%.1f,%.1f,%.1f)mm/s disp(%.0f,%.0f,%.0f)um "
      "freq(%.0f,%.0f,%.0f)Hz temp=%.1fC\n",
      reading.velocity.x, reading.velocity.y, reading.velocity.z,
      reading.displacement.x, reading.displacement.y, reading.displacement.z,
      reading.frequency.x, reading.frequency.y, reading.frequency.z,
      reading.device.temperature);
}
