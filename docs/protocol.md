# WTVB01-BT50 BLE Protocol

Two sources back this document, and every claim below says which one it
comes from:

1. **The official Python SDK.** `Python/BWT901BLE5.0_python_sdk` from
   [WITMOTION/WitBluetooth_BWT901BLE5_0](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0)
   (commit `9efaab0`), files `device_model.py` and `test.py`. This is the
   only SDK code in that repository. There is no `Wtvb01Processor`, no
   `Wtvb01Resolver`, and no `Wit.Example_WTVB01BT50` project.
2. **Bytes captured from a physical WTVB01-BT50** (MAC
   `E6:6B:9A:CC:88:25`), stored in
   `internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex`.

The SDK is generic across the WT sensor family and does **not** decode
the WTVB01's vibration fields, so the capture is what settles the packet
layout. Where the two disagree, the capture wins and this is called out.

## 1. Device discovery and connection lifecycle

From `test.py` and `device_model.py`; `internal/ble` mirrors it:

1. **Scan** — `BleakScanner.discover(timeout=20.0)`, keeping devices whose
   name contains `"WT"` (`test.py:22`). Our scanner also matches on the
   advertised service UUID, because a BLE device may advertise no local
   name at all.
2. **Connect** — `BleakClient(BLEDevice, timeout=15)` (`device_model.py:62`).
3. **Resolve characteristics** — iterate `client.services`, match the
   fixed UUIDs below (`device_model.py:71-82`).
4. **Subscribe** — `client.start_notify(notify_uuid, onDataReceived)`
   (`device_model.py:93`).
5. **Poll registers** — after a 3 s settle, a background task issues
   register reads 100 ms apart (`device_model.py:88, 112-117`). For the
   WTVB01 this is optional: its `0x61` broadcast already carries every
   measurement register.
6. **Accumulate** — `deviceData` is a dict updated per packet, with a
   callback fired on each update (`device_model.py:39-56, 156`). Our
   `Decoder` keeps the same accumulate-then-emit behaviour with a typed
   struct.
7. **Close** — clear `isOpen`, then `stop_notify` (`device_model.py:96-103`).

## 2. BLE UUIDs

From `device_model.py:66-68`, verbatim:

```python
target_service_uuid              = "0000ffe5-0000-1000-8000-00805f9a34fb"
target_characteristic_uuid_read  = "0000ffe4-0000-1000-8000-00805f9a34fb"  # notify
target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"  # write
```

Confirmed against hardware: the physical sensor advertises service
`ffe5` and both characteristics resolve.

## 3. Framing

From `device_model.py:121-133`:

```python
for var in tempdata:
    self.TempBytes.append(var)
    if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
        del self.TempBytes[0]; continue
    if len(self.TempBytes) == 2 and (self.TempBytes[1] != 0x61 and self.TempBytes[1] != 0x71):
        del self.TempBytes[0]; continue
    if len(self.TempBytes) == 20:
        self.processData(self.TempBytes); self.TempBytes.clear()
```

Rules: byte 0 is `0x55`; byte 1 is the type, `0x61` or `0x71`; anything
else resyncs.

### ⚠️ Packet length: the SDK is wrong for this sensor

The SDK hardcodes **20 bytes for both types**. Measured against 3772
captured bytes:

| Type | SDK assumes | Actual (WTVB01-BT50) | Packets measured |
|---|---|---|---|
| `0x61` broadcast | 20 | **32** | 110 |
| `0x71` register read-back | 20 | **20** | 11 |

Decoding a `0x61` as 20 bytes silently misreads every field — it
happens to yield plausible-looking numbers, which is exactly why this
was caught only by comparing against register read-backs. The decoder
therefore selects length by type (`packetLenFor`).

The `0x71` layout matches both the SDK and the official manual:
`0x55 0x71 <start reg, 2 bytes LE> <16 bytes = 8 registers, LE>`.

## 4. Register map

Addresses confirmed by the official manual and by capture; see §6 for
the cross-check.

| Register | Field | Scale | Unit |
|---|---|---|---|
| `0x3A` `0x3B` `0x3C` | Vibration velocity X/Y/Z | raw | mm/s |
| `0x3D` `0x3E` `0x3F` | Angular vibration amplitude X/Y/Z | `raw / 32768 * 180` | degrees |
| `0x40` | Temperature | `raw / 100` | °C |
| `0x41` `0x42` `0x43` | Vibration displacement X/Y/Z | raw | µm |
| `0x44` `0x45` `0x46` | Vibration frequency X/Y/Z | raw | Hz |

All values are signed int16, little-endian. `getSignInt16`
(`device_model.py:180-184`) subtracts 2^16 when the raw value is ≥ 2^15,
which is plain two's-complement int16.

Documented ranges (manual): velocity 0–100 mm/s, displacement
0–30000 µm, frequency 1–100 Hz, temperature −40 to +85 °C.

### Temperature is the module's own temperature

The manual's register table names `0x40` **"Product temperature"** — the
temperature of the sensor module itself, not of the machine it is
mounted on and not a calibrated ambient probe. It is modelled as
`device.temperature` and must never be called `bearing_temperature` or
`motor_temperature`.

Observed on hardware: the sensor resting on a desk reported 24.4–25.1 °C
against a room at roughly the same temperature, which is expected — with
no heat source, the module equilibrates with the air around it. Mounted
on a hot machine it reads somewhere between the machine and the ambient
air, dominated by conduction through the mount, and it lags. Useful as a
sanity signal and for drift detection; not a substitute for a probe on
the bearing.

## 5. The `0x61` broadcast packet

32 bytes: 2 header bytes followed by 15 signed int16 values.

| Value index | Bytes | Meaning |
|---|---|---|
| 0-2 | 2-7 | Velocity X, Y, Z (registers `0x3A`-`0x3C`) |
| 3-5 | 8-13 | Angular amplitude X, Y, Z (`0x3D`-`0x3F`) |
| 6 | 14-15 | Temperature (`0x40`) |
| 7-9 | 16-21 | Displacement X, Y, Z (`0x41`-`0x43`) |
| 10-12 | 22-27 | Frequency X, Y, Z (`0x44`-`0x46`) |
| 13 | 28-29 | Constant `0x0000` in all captures; not decoded |
| 14 | 30-31 | Slowly drifting counter; not a documented measurement register. The official app shows a "Power Percent(%)" field, which this may back — unconfirmed, so not decoded |

This matches the manual's stated order: *"vibration velocity XYZ,
vibration angle XYZ, temperature, vibration displacement XYZ, vibration
frequency XYZ, with the low byte first and the high byte last."* The
manual describes this packet as 28 bytes (header plus the 13 measurement
values); the sensor actually sends 32, with the two trailing values
above.

## 6. How the layout was verified

The `0x61` broadcast and the `0x71` register read-backs are independent
encodings of the same registers, so they can be checked against each
other with no external reference. From the capture:

```
0x61 values:            [17, 7, 18, 233, 243, 57, 2455, 300, 19, 239, 9, 13, 10, 0, 418]
0x71 block 0x3A regs:   [17, 7, 18, 233, 243, 57, 2436, 300]
0x71 block 0x42 regs:   [19, 239, 9, 13, 10, -21, 27, -3]
```

Values 0-7 of the broadcast are byte-identical to registers
`0x3A`-`0x41`, and values 8-12 to registers `0x42`-`0x46`. Temperature
differs only because it is recomputed continuously (2455 vs 2436, i.e.
24.55 vs 24.36 °C).

Because of this, the decoder routes both packet types through one
register-address dispatch (`applyRegister`) rather than having two
parallel field layouts. `TestOutputAndRegisterPacketsAgree` enforces the
equivalence.

Temperature is additionally confirmed absolutely: `0x40 / 100` gives
24.36 °C, matching the room the capture was taken in.

## 7. Command format

From `device_model.py:214-246`.

**Read a register** (triggers a `0x71` response):
```
[0xFF, 0xAA, 0x27, regAddr, 0x00]
```
Byte 2 is the fixed read-trigger register `0x27`; byte 3 is the register
to read back.

**Write a register:**
```
[0xFF, 0xAA, regAddr, valueLow, valueHigh]
```

**Unlock** before writing config: `writeReg(0x69, 0xB588)`.
**Save** after writing: `writeReg(0x00, 0x0000)`.

All are sent to the write characteristic `ffe9`.

## 8. Still unconfirmed

- **Scales for velocity, displacement and frequency.** Register
  addresses and units are documented, and the values are physically
  plausible at rest (≈1 mm/s, 6-21 µm, 9-16 Hz), but no side-by-side
  comparison against the official WitMotion app under real vibration has
  been done. If a scale is off it is a one-line change in
  `internal/protocol/wtvb01/registers.go`.
- **Value 14 of the `0x61` packet** (the drifting counter). Likely the
  battery percentage shown in the app, but unverified, so not decoded.

Temperature, packet lengths, register addresses, framing, UUIDs and the
command encoding are all confirmed.
