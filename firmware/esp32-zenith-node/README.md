# ESP32 Zenith Edge node

Reads a WitMotion WTVB01-BT50 over BLE and publishes normalized readings
to MQTT over WiFi. No PC required.

## Why it is simple

The sensor puts every measurement register in its `0x61` broadcast and
sends it unprompted, so this firmware **never writes to the sensor**. No
register polling, no unlock/save, no command encoding. It scans,
connects, subscribes to the notify characteristic, and parses.

The measurements are **not** in the BLE advertisement — only the service
UUIDs are — so a connection is required. Passive sniffing will not work.

## Hardware

| Board | Works | Note |
|---|---|---|
| ESP32 (classic) | yes | |
| ESP32-C3 | yes | cheapest option that works |
| ESP32-S3 | yes | |
| **ESP32-S2** | **no** | has no Bluetooth radio at all |

BLE and WiFi share one antenna on the ESP32. One sensor at a ~200 ms
notification interval is comfortable; avoid saturating WiFi with
continuous bulk transfers.

NimBLE allows about three concurrent connections by default, so one node
can serve several sensors after extending the connection handling here.

## Setup

```bash
cp src/config.example.h src/config.h
# edit src/config.h with your WiFi and MQTT details
pio run -e esp32-c3 -t upload -t monitor
```

`src/config.h` is gitignored, so credentials stay local. Pick the env
matching your board: `esp32dev`, `esp32-c3`, or `esp32-s3`.

## Published data

Topic: `zenith/readings/<sensor-mac>`

```json
{
  "sensor": "e6:6b:9a:cc:88:25",
  "uptime_ms": 42000,
  "velocity":     { "x": 1.0,   "y": 0.0,   "z": 0.0 },
  "displacement": { "x": 21.0,  "y": 9.0,   "z": 6.0 },
  "angle":        { "x": 0.088, "y": 0.005, "z": 0.033 },
  "frequency":    { "x": 11.0,  "y": 12.0,  "z": 16.0 },
  "device":       { "temperature": 24.9, "rssi": -49 }
}
```

Units: velocity mm/s, displacement µm, angle degrees, frequency Hz,
temperature °C. The schema matches `wtvb01.SensorReading` in the Go
collector, so the two are interchangeable downstream.

`device.temperature` is the **sensor module's** temperature, not the
machine's. See [`docs/indicators.md`](../../docs/indicators.md).

The node also publishes a retained `online`/`offline` value to
`zenith/status`, with `offline` set as the MQTT last will so a crashed
node is visible on the broker.

## Tests

`src/wtvb01.{h,cpp}` is dependency-free and builds on a host, so the
decoder is tested against the same real captured sensor bytes as the Go
implementation:

```bash
c++ -std=c++17 -I src -o /tmp/wtvb01_test test/decoder_test.cpp src/wtvb01.cpp
/tmp/wtvb01_test ../../internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex
```

The strongest check exploits the fact that the `0x61` broadcast and the
`0x71` register read-backs independently encode the same registers, so
decoding either must agree.

## Troubleshooting

**Sensor not found.** Most often the phone app still holds the
connection — BLE peripherals accept one central at a time and stop
advertising while connected. Disconnect it in the phone's Bluetooth
settings, not just by closing the app. Otherwise check the sensor is
powered on.

**Compiles but no data.** Confirm the board actually has BLE (not an
ESP32-S2) and that `SENSOR_ADDRESS` in `config.h` is either empty or
lowercase-matching the sensor MAC.
