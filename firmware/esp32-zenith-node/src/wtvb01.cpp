#include "wtvb01.h"

namespace wtvb01 {
namespace {

// Scale factors. Only temperature is hardware-confirmed; the rest
// follow the documented WTVB01 units. Keep these in sync with
// internal/protocol/wtvb01/registers.go.
constexpr float kTemperatureScale = 100.0f;  // raw/100 -> Celsius
constexpr float kAngleScale = 32768.0f;
constexpr float kAngleRange = 180.0f;  // raw/32768*180 -> degrees
constexpr float kVelocityScale = 1.0f;      // raw -> mm/s
constexpr float kDisplacementScale = 1.0f;  // raw -> micrometres
constexpr float kFrequencyScale = 1.0f;     // raw -> Hz

// Reads a little-endian signed int16, matching the SDK's
// getSignInt16(high<<8 | low).
inline float SignInt16LE(uint8_t low, uint8_t high) {
  return static_cast<float>(
      static_cast<int16_t>(static_cast<uint16_t>(low) |
                           (static_cast<uint16_t>(high) << 8)));
}

}  // namespace

size_t PacketLenFor(uint8_t packet_type) {
  switch (packet_type) {
    case kPacketTypeOutput:
      return kOutputPacketLen;
    case kPacketTypeRegister:
      return kRegisterPacketLen;
    default:
      return 0;
  }
}

bool Decoder::Feed(const uint8_t *data, size_t len) {
  bool updated = false;

  for (size_t i = 0; i < len; i++) {
    const uint8_t b = data[i];
    buf_[len_++] = b;

    // Resync: byte 0 must be the sync byte.
    if (len_ == 1 && buf_[0] != kSyncByte) {
      len_ = 0;
      continue;
    }
    // Resync: byte 1 must be a known type. The byte just read may
    // itself start a real packet, so keep it if it is a sync byte.
    if (len_ == 2 && PacketLenFor(buf_[1]) == 0) {
      len_ = 0;
      if (b == kSyncByte) {
        buf_[len_++] = b;
      }
      continue;
    }
    if (len_ < 2) {
      continue;
    }

    if (len_ == PacketLenFor(buf_[1])) {
      if (ProcessPacket(buf_)) {
        updated = true;
      }
      len_ = 0;
    }
  }

  return updated;
}

bool Decoder::ProcessPacket(const uint8_t *pkt) {
  switch (pkt[1]) {
    case kPacketTypeOutput:
      return DecodeOutput(pkt);
    case kPacketTypeRegister:
      return DecodeRegisterBlock(pkt);
    default:
      return false;
  }
}

// The 0x61 broadcast's first 13 int16 values are byte-identical to
// registers 0x3A..0x46 read back via 0x71, so it goes through the same
// register dispatch. Value 13 (a constant zero) is ignored. Value 14
// has no documented register address, but is a candidate for the app's
// "Power Percent(%)" field, so it is decoded into device.power_raw (see
// its doc comment for why it is raw, not a percentage).
bool Decoder::DecodeOutput(const uint8_t *pkt) {
  const uint8_t *values = pkt + 2;
  const size_t count = (kOutputPacketLen - 2) / 2;

  bool updated = false;
  for (size_t i = 0; i < count; i++) {
    const uint8_t reg = static_cast<uint8_t>(kRegVelocityX + i);
    if (reg > kRegFrequencyZ) {
      break;
    }
    if (ApplyRegister(reg, SignInt16LE(values[i * 2], values[i * 2 + 1]))) {
      updated = true;
    }
  }
  if (kOutputPacketLen - 2 >= 30) {
    current_.device.power_raw = SignInt16LE(values[28], values[29]);
    updated = true;
  }
  return updated;
}

// A 0x71 read-back: pkt[2:4] is the little-endian start register
// address, pkt[4:20] is eight registers.
bool Decoder::DecodeRegisterBlock(const uint8_t *pkt) {
  const uint8_t start_reg = pkt[2];
  const uint8_t *data = pkt + 4;

  bool updated = false;
  for (size_t i = 0; i < kRegistersPerBlock; i++) {
    const uint8_t reg = static_cast<uint8_t>(start_reg + i);
    if (ApplyRegister(reg, SignInt16LE(data[i * 2], data[i * 2 + 1]))) {
      updated = true;
    }
  }
  return updated;
}

bool Decoder::ApplyRegister(uint8_t reg, float raw) {
  switch (reg) {
    case kRegVelocityX:
      current_.velocity.x = raw * kVelocityScale;
      break;
    case kRegVelocityY:
      current_.velocity.y = raw * kVelocityScale;
      break;
    case kRegVelocityZ:
      current_.velocity.z = raw * kVelocityScale;
      break;

    case kRegAngleX:
      current_.angle.x = raw / kAngleScale * kAngleRange;
      break;
    case kRegAngleY:
      current_.angle.y = raw / kAngleScale * kAngleRange;
      break;
    case kRegAngleZ:
      current_.angle.z = raw / kAngleScale * kAngleRange;
      break;

    case kRegTemperature:
      current_.device.temperature = raw / kTemperatureScale;
      break;

    case kRegDisplacementX:
      current_.displacement.x = raw * kDisplacementScale;
      break;
    case kRegDisplacementY:
      current_.displacement.y = raw * kDisplacementScale;
      break;
    case kRegDisplacementZ:
      current_.displacement.z = raw * kDisplacementScale;
      break;

    case kRegFrequencyX:
      current_.frequency.x = raw * kFrequencyScale;
      break;
    case kRegFrequencyY:
      current_.frequency.y = raw * kFrequencyScale;
      break;
    case kRegFrequencyZ:
      current_.frequency.z = raw * kFrequencyScale;
      break;

    default:
      return false;
  }
  return true;
}

}  // namespace wtvb01
