# Architecture and data flow

Two collectors read the same sensor and emit the same JSON schema. Which
one you deploy depends on whether a Linux box is available at the
machine.

- **Go collector** (`cmd/zenith-edge`) — runs on Linux via BlueZ. Used
  for development, protocol work, and capturing fixtures.
- **ESP32 node** (`firmware/esp32-zenith-node`) — standalone hardware,
  publishes to MQTT over WiFi. Used in the field.

The decoder is the same logic in both, ported line for line, and both
are tested against the same captured sensor bytes.

## The whole path, end to end

```mermaid
flowchart TD
    subgraph SENSOR["WTVB01-BT50 sensor"]
        ACCEL["MEMS accelerometer<br/>samples the vibrating surface"]
        DSP["On-sensor processing<br/>computes velocity, displacement,<br/>frequency, angular amplitude"]
        REGS["Registers 0x3A - 0x46<br/>13 signed int16 values"]
        ACCEL --> DSP --> REGS
    end

    REGS -->|"BLE notify on ffe4<br/>0x61 packet, 32 bytes,<br/>sent unprompted"| COLLECT

    subgraph COLLECT["Collector: Go on Linux, or ESP32"]
        FRAME["Framing<br/>find 0x55, read type byte,<br/>collect the whole packet"]
        DISPATCH["Register dispatch<br/>map each int16 to its register<br/>address, then apply its scale"]
        READING["SensorReading<br/>velocity, displacement, angle,<br/>frequency, device.temperature"]
        FRAME --> DISPATCH --> READING
    end

    READING -->|JSON| OUT["Terminal (Go)<br/>or MQTT (ESP32)"]

    style SENSOR fill:#e8f0fe,stroke:#4285f4
    style COLLECT fill:#e6f4ea,stroke:#34a853
    style OUT fill:#fef7e0,stroke:#fbbc04
```

The sensor does the signal processing. By the time anything reaches the
collector it is already velocity, displacement and frequency — the
collector never sees raw acceleration samples and does no FFT.

## Connection sequence

The measurements are not in the BLE advertisement, only the service
UUIDs are, so a connection is mandatory. Passive sniffing does not work.

```mermaid
sequenceDiagram
    participant C as Collector
    participant S as WTVB01-BT50

    C->>S: BLE scan
    S-->>C: advertisement<br/>name "WTVB01-BT50", service ffe5
    Note over C: match on service UUID, not name:<br/>some units advertise no name

    C->>S: connect
    C->>S: discover service ffe5
    S-->>C: notify char ffe4, write char ffe9

    C->>S: subscribe to ffe4

    loop continuously, unprompted
        S-->>C: 0x61 packet, 32 bytes<br/>all 13 measurement registers
    end

    Note over C,S: the write characteristic is never needed:<br/>the broadcast already carries everything

    opt optional cross-check
        C->>S: FF AA 27 3A 00 (read block 0x3A)
        S-->>C: 0x71 packet, 20 bytes, 8 registers
    end
```

The optional read-back path exists only to verify the decoder. Because
`0x61` and `0x71` are independent encodings of the same registers,
decoding both and comparing is a correctness check that needs no
external reference — this is how the packet layout was proven, and it
runs as a test in both implementations.

## Inside the decoder

Both packet types feed one register-address dispatch rather than two
parallel field layouts, because capture showed the broadcast's first 13
values are byte-identical to registers `0x3A`-`0x46`.

```mermaid
flowchart TD
    IN["raw BLE bytes<br/>may split or concatenate packets"] --> B0

    B0{"byte 0 == 0x55?"}
    B0 -->|no| DROP["drop byte, resync"] --> B0
    B0 -->|yes| B1{"byte 1 type?"}

    B1 -->|"0x61"| L32["expect 32 bytes"]
    B1 -->|"0x71"| L20["expect 20 bytes"]
    B1 -->|other| DROP

    L32 --> FULL{"have a whole packet?"}
    L20 --> FULL
    FULL -->|no| WAIT["keep buffering"] --> IN

    FULL -->|"yes, 0x61"| MAP61["values 0..12<br/>map to registers 0x3A..0x46"]
    FULL -->|"yes, 0x71"| MAP71["read start register from bytes 2-3,<br/>map 8 registers"]

    MAP61 --> APPLY
    MAP71 --> APPLY

    APPLY["applyRegister(addr, raw)"]
    APPLY --> SCALE["0x3A-0x3C velocity, raw mm/s<br/>0x3D-0x3F angle, raw/32768*180 deg<br/>0x40 temperature, raw/100 C<br/>0x41-0x43 displacement, raw um<br/>0x44-0x46 frequency, raw Hz"]
    SCALE --> EMIT["update SensorReading, emit"]

    style DROP fill:#fce8e6,stroke:#ea4335
    style EMIT fill:#e6f4ea,stroke:#34a853
```

The decoder accumulates: a `0x61` packet refreshes every field at once,
while a `0x71` read-back refreshes only the eight registers in its
block, leaving the rest at their last known value.

### The 20-byte trap

WitMotion's generic BWT901 SDK hardcodes a 20-byte packet for **both**
types. That is correct for `0x71` and wrong for the WTVB01's `0x61`,
which is 32 bytes.

This matters more than a normal off-by-one because decoding a 32-byte
packet as 20 bytes does not crash or produce obvious garbage — it reads
the wrong fields and yields *plausible-looking* numbers. In our first
run it reported an "angle" of 13.35°, which was actually the temperature
register read at the wrong offset. It was caught only by comparing
against the register read-backs.

## Deployment shapes

```mermaid
flowchart LR
    subgraph DEV["Development"]
        S1["sensor"] -->|BLE| GO["zenith-edge<br/>on Linux/BlueZ"] --> TERM["terminal"]
    end

    subgraph FIELD["Field"]
        S2["sensor"] -->|BLE| ESP["ESP32 node"]
        ESP -->|"WiFi + MQTT<br/>zenith/readings/mac"| BROKER["MQTT broker"]
        BROKER --> APPS["dashboards,<br/>alerting, storage"]
    end

    style DEV fill:#e8f0fe,stroke:#4285f4
    style FIELD fill:#e6f4ea,stroke:#34a853
```

Both emit the same JSON, so anything downstream can consume either
without knowing which produced it.

## Why the sensor is the bottleneck, not the link

BLE notifications arrive roughly every 20-50 ms, but the vibration
registers update far more slowly — in captures the vibration values held
steady across dozens of packets while only temperature and the trailing
counter moved. The sensor computes vibration metrics over an internal
window.

Consequences:

- Publishing faster than about 1 Hz sends duplicates. The ESP32 node
  coalesces to `PUBLISH_INTERVAL_MS` (1 s by default).
- Timestamps mark when the collector decoded a packet, not when the
  sensor sampled the surface.
- A single missed packet costs nothing; the next one carries the full
  state again. There is no delta encoding to resynchronize.
