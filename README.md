# Zenith Edge Collector

A Go service that discovers, connects to, and reads the WitMotion
WTVB01-BT50 BLE vibration sensor, producing normalized readings.

Runs as a BLE central on Linux via BlueZ/D-Bus, using
[tinygo.org/x/bluetooth](https://tinygo.org/x/bluetooth).

## Status

MVP. Scanning, connecting, and decoding work against physical hardware.
Transport (MQTT/HTTP), local buffering, metrics, and multi-sensor
handling are deliberately not implemented yet.

## Install

```bash
go install github.com/NatanBack77/zenith-edge-collector/cmd/zenith-edge@latest
```

Or from a clone:

```bash
go install ./cmd/zenith-edge
```

Requires a working BlueZ stack (`systemctl status bluetooth`).

## Usage

Find the sensor:

```console
$ zenith-edge scan
Scanning for 10s (WitMotion devices: "WT" name or service 0000ffe5-...)...
E6:6B:9A:CC:88:25     WTVB01-BT50               RSSI  -49  <- WitMotion service ffe5
```

`--all` lists every BLE device in range, `--verbose` adds advertised
service UUIDs and manufacturer IDs — useful when a sensor advertises no
local name.

Stream decoded readings:

```console
$ zenith-edge test --sensor E6:6B:9A:CC:88:25
Connecting to E6:6B:9A:CC:88:25...
Streaming decoded readings (Ctrl+C to stop)...
[10:38:37.832] angle(0.00,0.01,0.00) vel(1.000,0.000,0.000)mm/s disp(21.0,9.0,6.0)um freq(11.0,12.0,16.0)Hz temp=24.9C
```

`--raw` dumps hex notification payloads instead, for capturing fixtures.

## What it measures

Five indicators, three of them axial:

| Field | Unit | What it tells you |
|---|---|---|
| `velocity` | mm/s | Overall machine health — what ISO 10816/20816 limits target |
| `displacement` | µm | Low-frequency faults: unbalance, misalignment, looseness |
| `frequency` | Hz | *Which* fault, via multiples of shaft speed |
| `angle` | degrees | Angular vibration amplitude — rocking, not mounting tilt |
| `device.temperature` | °C | The **sensor module's** temperature, not the machine's |

`device.temperature` is named that way deliberately. The manual calls
register `0x40` "Product temperature": it is the module's own
temperature, not a bearing or motor probe. See
[docs/indicators.md](docs/indicators.md) for what each indicator catches
and how they relate.

## Protocol

[docs/protocol.md](docs/protocol.md) documents the BLE protocol, derived
from the official Python SDK and verified against bytes captured from a
physical sensor.

One correction worth highlighting: WitMotion's generic BWT901 SDK
hardcodes a 20-byte packet for both packet types. The WTVB01-BT50's
`0x61` broadcast is **32 bytes**. Decoding it as 20 silently misreads
every field while still producing plausible-looking numbers.

## Layout

```
cmd/zenith-edge/            CLI: scan, test
internal/ble/               BLE scanning and connection (BlueZ)
internal/protocol/wtvb01/   Packet framing and register decoding
  testdata/                 Bytes captured from a physical sensor
docs/                       Protocol and indicator reference
```

## Tests

```bash
go test ./...
```

Tests replay real captured sensor bytes. The strongest check,
`TestOutputAndRegisterPacketsAgree`, exploits the fact that the `0x61`
broadcast and `0x71` register read-backs independently encode the same
registers, so they must decode identically.

## Not yet verified

Scale factors for velocity, displacement and frequency come from the
manual and produce physically plausible values at rest, but have not
been compared side by side against the official WitMotion app under real
vibration. Any correction is a one-line change in
`internal/protocol/wtvb01/registers.go`.
