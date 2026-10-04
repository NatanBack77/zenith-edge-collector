#include "wtvb01.h"

#include <math.h>
#include <string.h>

namespace wtvb01 {
namespace {

// Scale factors. Only temperature is hardware-confirmed; the rest
// follow the documented WTVB01 units. Keep these in sync with
// internal/protocol/wtvb01/registers.go.
constexpr float kTemperatureScale = 100.0f;  // raw/100 -> Celsius
constexpr float kAngleScale = 32768.0f;
constexpr float kAngleRange = 180.0f;  // raw/32768*180 -> degrees
constexpr float kGyroRange = 2000.0f;  // raw/32768*2000 -> deg/s
constexpr float kVelocityScale = 1.0f;      // raw -> mm/s
constexpr float kDisplacementScale = 1.0f;  // raw -> micrometres
constexpr float kFrequencyScale = 1.0f;     // raw -> Hz
constexpr float kBatteryCentivolts = 100.0f;  // raw/100 -> volts

// Reads a little-endian signed int16, matching the SDK's
// getSignInt16(high<<8 | low).
inline int16_t SignInt16(const uint8_t *p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

inline float SignInt16LE(uint8_t low, uint8_t high) {
  const uint8_t b[2] = {low, high};
  return static_cast<float>(SignInt16(b));
}

inline uint16_t UnsignedInt16(const uint8_t *p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

// Piecewise-linear interpolation clamped at both ends, same as the
// official app's Interp().
float Interp(float v, const float *xs, const float *ys, size_t n) {
  if (v < xs[0]) {
    return ys[0];
  }
  if (v > xs[n - 1]) {
    return ys[n - 1];
  }
  for (size_t i = 0; i + 1 < n; i++) {
    if (v <= xs[i + 1]) {
      return ys[i] + (v - xs[i]) / (xs[i + 1] - xs[i]) * (ys[i + 1] - ys[i]);
    }
  }
  return ys[n - 1];
}

}  // namespace

float BatteryPercentFromVolts(float volts) {
  // Tables from the official Android app 4.0.9 (WTVB01DataProcessor.
  // getEqPercent). The second table is the app's >5.5 V branch.
  static const float kCellV[] = {3.4f,  3.5f,  3.68f, 3.7f,  3.73f, 3.77f,
                                 3.79f, 3.82f, 3.87f, 3.93f, 3.96f, 3.99f};
  static const float kCellPct[] = {0.0f,  5.0f,  10.0f, 15.0f, 20.0f, 30.0f,
                                   40.0f, 50.0f, 60.0f, 75.0f, 90.0f, 100.0f};
  static const float kPackV[] = {6.5f, 6.8f, 7.35f, 7.75f, 8.5f, 8.8f};
  static const float kPackPct[] = {0.0f, 10.0f, 30.0f, 60.0f, 90.0f, 100.0f};

  if (volts > 5.5f) {
    return Interp(volts, kPackV, kPackPct, sizeof(kPackV) / sizeof(kPackV[0]));
  }
  return Interp(volts, kCellV, kCellPct, sizeof(kCellV) / sizeof(kCellV[0]));
}

void BuildWriteRegisterCommand(uint8_t reg, uint16_t value, uint8_t *out) {
  out[0] = 0xFF;
  out[1] = 0xAA;
  out[2] = reg;
  out[3] = static_cast<uint8_t>(value & 0xFF);
  out[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void BuildReadRegisterCommand(uint8_t reg, uint8_t *out) {
  out[0] = 0xFF;
  out[1] = 0xAA;
  out[2] = 0x27;
  out[3] = reg;
  out[4] = 0x00;
}

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

// ----------------------------------------------------------- Decoder

bool Decoder::IsHeader(const uint8_t *p) {
  return p[0] == kSyncByte &&
         (p[1] == kPacketTypeOutput || p[1] == kPacketTypeRegister);
}

void Decoder::Consume(size_t n) {
  if (n >= len_) {
    len_ = 0;
    return;
  }
  memmove(buf_, buf_ + n, len_ - n);
  len_ -= n;
}

bool Decoder::Feed(const uint8_t *data, size_t len) {
  bool updated = false;
  size_t off = 0;
  while (off < len) {
    // Drain() always leaves fewer than kDetectBytes bytes buffered, so
    // there is room; the guard below only protects against a logic bug.
    size_t space = kBufCap - len_;
    if (space == 0) {
      Consume(1);
      resyncs_++;
      space = 1;
    }
    const size_t n = (len - off < space) ? (len - off) : space;
    memcpy(buf_ + len_, data + off, n);
    len_ += n;
    off += n;
    if (Drain()) {
      updated = true;
    }
  }
  return updated;
}

// Extracts every complete packet currently buffered.
//
// The 0x61 length (32 vs 40) is the only ambiguous part, so it is decided
// from the stream: at least kDetectBytes are needed to see where the NEXT
// header sits for both hypotheses. After that the length is locked and
// each packet is double-checked against the header that follows it (when
// those two bytes are already buffered), which re-opens detection if the
// sensor switched mode mid-stream.
bool Decoder::Drain() {
  bool updated = false;
  for (;;) {
    size_t skip = 0;
    while (skip < len_ && buf_[skip] != kSyncByte) {
      skip++;
    }
    if (skip > 0) {
      Consume(skip);
      resyncs_++;
    }
    if (len_ < 2) {
      return updated;
    }

    const uint8_t type = buf_[1];
    size_t want = 0;
    if (type == kPacketTypeRegister) {
      want = kRegisterPacketLen;
    } else if (type == kPacketTypeOutput) {
      if (output_len_ == 0) {
        if (len_ < kDetectBytes) {
          return updated;  // not enough bytes to decide yet
        }
        const bool is_default = IsHeader(buf_ + kOutputPacketLen);
        const bool is_instant = IsHeader(buf_ + kInstantPacketLen);
        if (is_default == is_instant) {
          // Neither or both: not a trustworthy packet start. Slide one
          // byte and look for the next real header.
          Consume(1);
          resyncs_++;
          continue;
        }
        output_len_ = is_default ? kOutputPacketLen : kInstantPacketLen;
      }
      want = output_len_;
    } else {
      Consume(1);
      resyncs_++;
      continue;
    }

    if (len_ < want) {
      return updated;  // wait for the rest of the packet
    }

    // Verify against the following header when it is already buffered.
    if (len_ >= want + 2 && !IsHeader(buf_ + want)) {
      if (type == kPacketTypeOutput) {
        output_len_ = 0;  // wrong length (mode switched?): re-detect here
        resyncs_++;
        continue;
      }
      Consume(1);
      resyncs_++;
      continue;
    }

    if (ProcessPacket(buf_, want)) {
      updated = true;
    }
    Consume(want);
  }
}

bool Decoder::ProcessPacket(const uint8_t *pkt, size_t len) {
  switch (pkt[1]) {
    case kPacketTypeOutput:
      return len == kInstantPacketLen ? DecodeInstant(pkt) : DecodeOutput(pkt);
    case kPacketTypeRegister:
      return DecodeRegisterBlock(pkt);
    default:
      return false;
  }
}

// The Default 0x61 broadcast's first 13 int16 values are byte-identical to
// registers 0x3A..0x46 read back via 0x71, so they go through the same
// register dispatch. Value 13 is the alarm status (manual §5.2.8) and
// value 14 is the battery register 0x64 (confirmed against a read-back on
// hardware).
bool Decoder::DecodeOutput(const uint8_t *pkt) {
  const uint8_t *values = pkt + 2;
  const size_t count = (kOutputPacketLen - 2) / 2;

  for (size_t i = 0; i < count; i++) {
    const uint8_t reg = static_cast<uint8_t>(kRegVelocityX + i);
    if (reg > kRegFrequencyZ) {
      break;
    }
    ApplyRegister(reg, SignInt16LE(values[i * 2], values[i * 2 + 1]));
  }
  current_.device.alarm_status = SignInt16(values + 26);
  ApplyRegister(kRegBattery, SignInt16LE(values[28], values[29]));
  mode_ = DataMode::kDefault;
  return true;
}

// "Now data" packet, 40 bytes (manual §5.2.2, byte layout confirmed
// against the official app's display on hardware):
//   55 61 | year month day hour minute second | ms(2) |
//   AX AY AZ | GX GY GZ | NVX NVY NVZ | NADX NADY NADZ | NDX NDY NDZ
// Everything after the header is little-endian int16 except the
// single-byte time fields.
bool Decoder::DecodeInstant(const uint8_t *pkt) {
  InstantSample s;
  s.year = pkt[2];
  s.month = pkt[3];
  s.day = pkt[4];
  s.hour = pkt[5];
  s.minute = pkt[6];
  s.second = pkt[7];
  s.millisecond = UnsignedInt16(pkt + 8);

  // A false match on `55 61` inside payload bytes would decode to an
  // impossible calendar time almost every time; reject those.
  if (s.month < 1 || s.month > 12 || s.day < 1 || s.day > 31 || s.hour > 23 ||
      s.minute > 59 || s.second > 59 || s.millisecond > 999) {
    rejected_++;
    return false;
  }

  const uint64_t ms =
      ((static_cast<uint64_t>(s.day) * 24 + s.hour) * 60 + s.minute) * 60000ULL +
      static_cast<uint64_t>(s.second) * 1000ULL + s.millisecond;
  s.chip_ms = static_cast<uint32_t>(ms);

  const uint8_t *v = pkt + 10;
  for (int k = 0; k < 3; k++) {
    s.acc_raw[k] = SignInt16(v + 2 * k);
  }
  s.accel = {s.acc_raw[0] * kAccelScaleG, s.acc_raw[1] * kAccelScaleG,
             s.acc_raw[2] * kAccelScaleG};
  s.gyro = {SignInt16(v + 6) / kAngleScale * kGyroRange,
            SignInt16(v + 8) / kAngleScale * kGyroRange,
            SignInt16(v + 10) / kAngleScale * kGyroRange};
  s.velocity = {static_cast<float>(SignInt16(v + 12)) * kVelocityScale,
                static_cast<float>(SignInt16(v + 14)) * kVelocityScale,
                static_cast<float>(SignInt16(v + 16)) * kVelocityScale};
  s.angle = {SignInt16(v + 18) / kAngleScale * kAngleRange,
             SignInt16(v + 20) / kAngleScale * kAngleRange,
             SignInt16(v + 22) / kAngleScale * kAngleRange};
  s.displacement = {static_cast<float>(SignInt16(v + 24)) * kDisplacementScale,
                    static_cast<float>(SignInt16(v + 26)) * kDisplacementScale,
                    static_cast<float>(SignInt16(v + 28)) * kDisplacementScale};

  last_instant_ = s;
  mode_ = DataMode::kInstant;
  if (sink_ != nullptr) {
    sink_(s, sink_ctx_);
  }
  return false;  // reading() is unchanged; instant data flows via the sink
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

    case kRegBattery:
      current_.device.power_raw = raw;
      current_.device.battery_volts = raw / kBatteryCentivolts;
      current_.device.battery_percent =
          BatteryPercentFromVolts(current_.device.battery_volts);
      break;

    case kRegDataMode:
      current_.device.data_mode = static_cast<int>(raw);
      break;

    default:
      return false;
  }
  return true;
}

// ---------------------------------------------------------- Waveform

float WaveformWindow::SampleRateHz() const {
  if (count != WAVEFORM_WINDOW_SAMPLES || end_chip_ms <= start_chip_ms) {
    return 0.0f;
  }
  return 1000.0f * static_cast<float>(count - 1) /
         static_cast<float>(end_chip_ms - start_chip_ms);
}

bool WaveformAccumulator::Push(const InstantSample &s) {
  if (complete_) {
    // The previous window was consumed (or overwritten); the next sample
    // continues the same stream, so keep last_ms_/step_ms_ for the gap
    // check and just start filling a fresh window.
    window_.count = 0;
    window_.gaps = 0;
    complete_ = false;
  }

  bool gap = false;
  if (have_last_) {
    if (s.chip_ms <= last_ms_) {
      gap = true;  // clock stood still or went backwards
    } else {
      const uint32_t dt = s.chip_ms - last_ms_;
      if (step_ms_ == 0) {
        if (dt > 1000) {
          gap = true;
        } else {
          step_ms_ = dt;
        }
      } else if (dt * 2 < step_ms_ || dt * 2 > step_ms_ * 3) {
        gap = true;  // outside [0.5x, 1.5x] of the learned interval
      }
    }
  }
  if (gap) {
    gaps_total_++;
    window_.gaps++;
    window_.count = 0;
    step_ms_ = 0;
  }
  last_ms_ = s.chip_ms;
  have_last_ = true;

  if (window_.count == 0) {
    window_.start_chip_ms = s.chip_ms;
  }
  window_.ax[window_.count] = s.acc_raw[0];
  window_.ay[window_.count] = s.acc_raw[1];
  window_.az[window_.count] = s.acc_raw[2];
  window_.count++;
  window_.end_chip_ms = s.chip_ms;

  if (window_.count == WAVEFORM_WINDOW_SAMPLES) {
    complete_ = true;
    windows_completed_++;
    return true;
  }
  return false;
}

// ----------------------------------------------------- Instant stats

void InstantStats::Add(const InstantSample &s) {
  const float x[9] = {s.velocity.x,     s.velocity.y,     s.velocity.z,
                      s.displacement.x, s.displacement.y, s.displacement.z,
                      s.angle.x,        s.angle.y,        s.angle.z};
  // alpha = dt / (dt + tau): a first-order low-pass on x^2. The first
  // sample seeds the average so it does not start from zero.
  float alpha = 1.0f;
  if (count_ > 0) {
    uint32_t dt = s.chip_ms - last_ms_;
    if (s.chip_ms <= last_ms_ || dt > 1000) {
      dt = 10;  // clock hiccup: assume the nominal step rather than jump
    }
    alpha = static_cast<float>(dt) / (static_cast<float>(dt) + kTimeConstantMs);
  }
  for (int i = 0; i < 9; i++) {
    const float sq = x[i] * x[i];
    mean_sq_[i] = count_ == 0 ? sq : mean_sq_[i] + alpha * (sq - mean_sq_[i]);
  }
  last_ms_ = s.chip_ms;
  count_++;
}

bool InstantStats::Rms(SensorReading *out) const {
  if (count_ < kMinSamples) {
    return false;
  }
  out->velocity = {sqrtf(mean_sq_[0]), sqrtf(mean_sq_[1]), sqrtf(mean_sq_[2])};
  out->displacement = {sqrtf(mean_sq_[3]), sqrtf(mean_sq_[4]), sqrtf(mean_sq_[5])};
  out->angle = {sqrtf(mean_sq_[6]), sqrtf(mean_sq_[7]), sqrtf(mean_sq_[8])};
  return true;
}

// ------------------------------------------------------------- FFT

namespace {

constexpr size_t kFftN = WAVEFORM_WINDOW_SAMPLES;
static_assert(kFftN >= 8 && (kFftN & (kFftN - 1)) == 0,
              "WAVEFORM_WINDOW_SAMPLES must be a power of two for the FFT");

// In-place iterative radix-2 FFT. Twiddles come from a table built on the
// first call.
void Fft(float *re, float *im) {
  static float cos_t[kFftN / 2];
  static float sin_t[kFftN / 2];
  static bool ready = false;
  if (!ready) {
    for (size_t i = 0; i < kFftN / 2; i++) {
      const float a = -2.0f * static_cast<float>(M_PI) * static_cast<float>(i) /
                      static_cast<float>(kFftN);
      cos_t[i] = cosf(a);
      sin_t[i] = sinf(a);
    }
    ready = true;
  }

  // Bit-reversal permutation.
  for (size_t i = 1, j = 0; i < kFftN; i++) {
    size_t bit = kFftN >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      const float tr = re[i], ti = im[i];
      re[i] = re[j];
      im[i] = im[j];
      re[j] = tr;
      im[j] = ti;
    }
  }

  for (size_t len = 2; len <= kFftN; len <<= 1) {
    const size_t half = len >> 1;
    const size_t step = kFftN / len;
    for (size_t i = 0; i < kFftN; i += len) {
      for (size_t k = 0; k < half; k++) {
        const float wr = cos_t[k * step];
        const float wi = sin_t[k * step];
        const size_t a = i + k, b = a + half;
        const float xr = re[b] * wr - im[b] * wi;
        const float xi = re[b] * wi + im[b] * wr;
        re[b] = re[a] - xr;
        im[b] = im[a] - xi;
        re[a] += xr;
        im[a] += xi;
      }
    }
  }
}

}  // namespace

void DominantFrequencies(const WaveformWindow &w, float fs_hz,
                         float out_xyz[3]) {
  static float re[kFftN];
  static float im[kFftN];
  static float mag[kFftN / 2];

  const int16_t *axes[3] = {w.ax, w.ay, w.az};
  for (int a = 0; a < 3; a++) {
    out_xyz[a] = 0.0f;
    if (w.count != kFftN || fs_hz <= 0.0f) {
      continue;
    }
    float mean = 0.0f;
    for (size_t i = 0; i < kFftN; i++) {
      mean += axes[a][i];
    }
    mean /= static_cast<float>(kFftN);
    for (size_t i = 0; i < kFftN; i++) {
      const float hann = 0.5f * (1.0f - cosf(2.0f * static_cast<float>(M_PI) *
                                             static_cast<float>(i) /
                                             static_cast<float>(kFftN - 1)));
      re[i] = (axes[a][i] - mean) * hann;
      im[i] = 0.0f;
    }
    Fft(re, im);

    for (size_t k = 0; k < kFftN / 2; k++) {
      mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
    }
    const float bin_hz = fs_hz / static_cast<float>(kFftN);
    size_t k_min = static_cast<size_t>(kMinDominantHz / bin_hz + 0.999f);
    if (k_min < 1) {
      k_min = 1;
    }
    size_t best = 0;
    float best_mag = 0.0f;
    for (size_t k = k_min; k + 1 < kFftN / 2; k++) {
      if (mag[k] > best_mag) {
        best_mag = mag[k];
        best = k;
      }
    }
    if (best == 0 || best_mag <= 0.0f) {
      continue;
    }
    // Parabolic interpolation around the peak bin.
    float delta = 0.0f;
    const float denom = mag[best - 1] - 2.0f * mag[best] + mag[best + 1];
    if (denom != 0.0f) {
      delta = 0.5f * (mag[best - 1] - mag[best + 1]) / denom;
      if (delta > 0.5f) delta = 0.5f;
      if (delta < -0.5f) delta = -0.5f;
    }
    out_xyz[a] = (static_cast<float>(best) + delta) * bin_hz;
  }
}

}  // namespace wtvb01
