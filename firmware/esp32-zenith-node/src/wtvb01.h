// WTVB01-BT50 packet decoder.
//
// Direct port of internal/protocol/wtvb01 from the Go collector. See
// docs/protocol.md for how the layout was derived and verified.
//
// The decoder is dependency-free and does no allocation, so it can be
// unit-tested on a host as well as run on the ESP32.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wtvb01 {

// Framing. NOTE: WitMotion's generic BWT901 SDK hardcodes 20 bytes for
// both packet types. That is wrong for the WTVB01-BT50, whose 0x61
// broadcast is 32 bytes. Verified over 110 captured packets.
constexpr uint8_t kSyncByte = 0x55;
constexpr uint8_t kPacketTypeOutput = 0x61;
constexpr uint8_t kPacketTypeRegister = 0x71;
constexpr size_t kOutputPacketLen = 32;
constexpr size_t kRegisterPacketLen = 20;
constexpr size_t kRegistersPerBlock = 8;

// Measurement registers.
constexpr uint8_t kRegVelocityX = 0x3A;
constexpr uint8_t kRegVelocityY = 0x3B;
constexpr uint8_t kRegVelocityZ = 0x3C;
constexpr uint8_t kRegAngleX = 0x3D;
constexpr uint8_t kRegAngleY = 0x3E;
constexpr uint8_t kRegAngleZ = 0x3F;
constexpr uint8_t kRegTemperature = 0x40;
constexpr uint8_t kRegDisplacementX = 0x41;
constexpr uint8_t kRegDisplacementY = 0x42;
constexpr uint8_t kRegDisplacementZ = 0x43;
constexpr uint8_t kRegFrequencyX = 0x44;
constexpr uint8_t kRegFrequencyY = 0x45;
constexpr uint8_t kRegFrequencyZ = 0x46;

// BLE UUIDs, from the official Python SDK (device_model.py:66-68).
constexpr const char *kServiceUUID = "0000ffe5-0000-1000-8000-00805f9a34fb";
constexpr const char *kNotifyCharUUID = "0000ffe4-0000-1000-8000-00805f9a34fb";
constexpr const char *kWriteCharUUID = "0000ffe9-0000-1000-8000-00805f9a34fb";

// Configuration registers and command format, from the official SDK
// (device_model.py:214-246) -- see docs/protocol.md §7. Unlike the
// measurement registers above (confirmed against a physical capture),
// these are universal across WitMotion's whole WT/BWT product line and
// have NOT been bench-verified against a real WTVB01-BT50 register write
// in this repo. kRegRate itself is well established across the product
// line; treat it as documented-but-unverified-on-this-unit, not as solid
// as the measurement registers.
constexpr uint8_t kRegRate = 0x03;
constexpr uint8_t kRegUnlock = 0x69;
constexpr uint8_t kRegSave = 0x00;
constexpr uint16_t kUnlockValue = 0xB588;
constexpr size_t kCommandLen = 5;

// Builds a `[0xFF, 0xAA, reg, valueLow, valueHigh]` write-register
// command into `out`, which must hold at least kCommandLen bytes. Send
// to the write characteristic (kWriteCharUUID). Command sequence to
// change a config register: BuildWriteRegisterCommand(kRegUnlock,
// kUnlockValue, ...), then the target register, then
// BuildWriteRegisterCommand(kRegSave, 0x0000, ...).
void BuildWriteRegisterCommand(uint8_t reg, uint16_t value, uint8_t *out);

struct Vector3 {
  float x = 0;
  float y = 0;
  float z = 0;
};

struct DeviceInfo {
  // Temperature is the sensor module's own temperature in Celsius
  // (register 0x40, "Product temperature"). Not the machine's
  // temperature, and not a calibrated ambient probe.
  float temperature = 0;

  // power_raw is value 14 of the 0x61 broadcast: a slowly-drifting
  // counter with no documented register address. The WitMotion app
  // shows a "Power Percent(%)" field this may feed, but that is
  // UNCONFIRMED — observed values (e.g. 418) are well outside 0-100,
  // so it is exposed raw rather than as a percentage. See
  // docs/protocol.md.
  float power_raw = 0;
};

struct SensorReading {
  Vector3 velocity;      // mm/s
  Vector3 displacement;  // micrometres
  Vector3 angle;         // degrees, angular vibration amplitude
  Vector3 frequency;     // Hz
  DeviceInfo device;
};

// Returns the total packet length for a type byte, or 0 if unknown.
size_t PacketLenFor(uint8_t packet_type);

// Decoder accumulates raw BLE notification bytes into packets.
//
// It keeps the latest value of every field across packets: a 0x61
// broadcast carries all measurement registers at once, while a 0x71
// read-back refreshes only the eight registers in its block.
class Decoder {
 public:
  // Feeds a raw notification payload, which may hold several packets or
  // split one across calls. Returns true if at least one packet was
  // decoded, in which case reading() is updated.
  bool Feed(const uint8_t *data, size_t len);

  const SensorReading &reading() const { return current_; }

 private:
  bool ProcessPacket(const uint8_t *pkt);
  bool DecodeOutput(const uint8_t *pkt);
  bool DecodeRegisterBlock(const uint8_t *pkt);
  bool ApplyRegister(uint8_t reg, float raw);

  uint8_t buf_[kOutputPacketLen] = {};
  size_t len_ = 0;
  SensorReading current_;
};

}  // namespace wtvb01
