// WTVB01-BT50 packet decoder.
//
// Direct port of internal/protocol/wtvb01 from the Go collector, extended
// with the "Now data" packet found by reverse-engineering the official
// WitMotion app. See docs/protocol.md for how the layout was derived and
// verified, and the Zenith repo's docs/sensor-wtvb01-bt50.md for the
// manual cross-check and the hardware captures behind the 40-byte packet.
//
// The decoder is dependency-free and does no allocation, so it can be
// unit-tested on a host as well as run on the ESP32.

#pragma once

#include <stddef.h>
#include <stdint.h>

// Samples per waveform window handed to the MQTT publisher. 256 samples
// at the sensor's observed 100 packets/s is 2.56 s and gives 0.39 Hz
// resolution in a later FFT. Must stay a power of two for that FFT.
#ifndef WAVEFORM_WINDOW_SAMPLES
#define WAVEFORM_WINDOW_SAMPLES 256
#endif

namespace wtvb01 {

// Framing. NOTE: WitMotion's generic BWT901 SDK hardcodes 20 bytes for
// both packet types. That is wrong for the WTVB01-BT50, whose Default
// 0x61 broadcast is 32 bytes. Verified over 110 captured packets.
//
// The sensor has TWO 0x61 formats and both use the same type byte, so the
// type byte alone cannot give the length (manual §5.2, confirmed on
// hardware):
//   - Default data:  32 bytes, amplitudes (always positive) + frequency +
//                    battery.
//   - "Now data":    40 bytes, chip time + acceleration + gyro + signed
//                    instantaneous velocity/angle/displacement.
// The Decoder tells them apart from the stream itself (see Decoder).
constexpr uint8_t kSyncByte = 0x55;
constexpr uint8_t kPacketTypeOutput = 0x61;
constexpr uint8_t kPacketTypeRegister = 0x71;
constexpr size_t kOutputPacketLen = 32;   // Default data
constexpr size_t kInstantPacketLen = 40;  // "Now data"
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

// Battery. Manual: "BatPer" at 0x64, also the last int16 of the Default
// 0x61 packet. Confirmed on hardware: the packet value matched the 0x64
// read-back (437..441 vs 439). The official app (4.0.9) reads it as
// centivolts (raw / 100 = volts) and maps volts to a percentage.
constexpr uint8_t kRegBattery = 0x64;

// Data mode register. NOT in the V260410 manual: found by capturing the
// official app's "Data mode" selector. 1 = "Now data", 0 = Default.
constexpr uint8_t kRegDataMode = 0x96;
constexpr uint16_t kDataModeDefault = 0;
constexpr uint16_t kDataModeInstant = 1;

// BLE UUIDs, from the official Python SDK (device_model.py:66-68).
constexpr const char *kServiceUUID = "0000ffe5-0000-1000-8000-00805f9a34fb";
constexpr const char *kNotifyCharUUID = "0000ffe4-0000-1000-8000-00805f9a34fb";
constexpr const char *kWriteCharUUID = "0000ffe9-0000-1000-8000-00805f9a34fb";

// Configuration registers and command format, from the official SDK
// (device_model.py:214-246) -- see docs/protocol.md §7. The unlock ->
// write -> save sequence with ~100 ms between commands was observed on
// the wire from the official app (the manual does not mention the delay).
// A register write was captured from the app for kRegDataMode only; kRegRate
// writes follow the manual and the official SDK samples but have not been
// seen on the wire from this unit.
constexpr uint8_t kRegRate = 0x03;
constexpr uint8_t kRegUnlock = 0x69;
constexpr uint8_t kRegSave = 0x00;
constexpr uint16_t kUnlockValue = 0xB588;
constexpr size_t kCommandLen = 5;
constexpr uint32_t kCommandSpacingMs = 100;

// Accelerometer full scale: +-16 g over int16 (manual §5.2.10).
constexpr float kAccelScaleG = 16.0f / 32768.0f;

// Builds a `[0xFF, 0xAA, reg, valueLow, valueHigh]` write-register
// command into `out`, which must hold at least kCommandLen bytes. Send
// to the write characteristic (kWriteCharUUID), without response (the
// official app uses ATT Write Command). Command sequence to change a
// config register: BuildWriteRegisterCommand(kRegUnlock, kUnlockValue,
// ...), then the target register, then
// BuildWriteRegisterCommand(kRegSave, 0x0000, ...), kCommandSpacingMs apart.
void BuildWriteRegisterCommand(uint8_t reg, uint16_t value, uint8_t *out);

// Builds the `[0xFF, 0xAA, 0x27, reg, 0x00]` command that asks the sensor to
// send back a 0x71 packet with eight registers starting at `reg`. It does
// not change anything in the sensor (the official app issues these
// constantly). Needs no unlock.
void BuildReadRegisterCommand(uint8_t reg, uint8_t *out);

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

  // Raw battery register value (last int16 of the Default 0x61 packet,
  // or the 0x64 read-back). Kept under its old name because the MQTT
  // schema already publishes it as `power_raw`.
  float power_raw = 0;

  // power_raw / 100, in volts, and the percentage from the official
  // app's piecewise-linear table (BatteryPercentFromVolts).
  float battery_volts = 0;
  float battery_percent = 0;

  // Alarm flags (manual §5.2.8): bit0..2 displacement alarm X/Y/Z,
  // bit3..5 frequency alarm X/Y/Z. 0 = no alarm. Observed 0 on every one
  // of 4868 packets when no alarm thresholds were configured.
  int alarm_status = 0;

  // Data mode read back from register 0x96, or -1 if never read.
  int data_mode = -1;
};

// Default-mode reading. Instantaneous ("Now data") samples never touch
// this struct: they go through the Decoder's InstantSink instead, so the
// amplitude fields below always mean amplitude.
struct SensorReading {
  Vector3 velocity;      // mm/s, amplitude (always positive)
  Vector3 displacement;  // micrometres, amplitude
  Vector3 angle;         // degrees, angular vibration amplitude
  Vector3 frequency;     // Hz
  DeviceInfo device;
};

// One 40-byte "Now data" packet. Values are signed instantaneous values
// (manual §4.6: "maximum and minimum vibration values, which may be
// negative"), plus raw acceleration and gyro.
struct InstantSample {
  // Chip clock. Without time calibration it starts at 2015-01-01 00:00:00
  // when the sensor powers on, so only differences between samples are
  // meaningful. chip_ms folds day/hour/minute/second/ms into a single
  // monotonic millisecond counter (month/year rollover is not handled).
  uint8_t year = 0;  // two digits, 15 == 2015
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;
  uint32_t chip_ms = 0;

  int16_t acc_raw[3] = {0, 0, 0};  // counts; * kAccelScaleG gives g
  Vector3 accel;                   // g
  Vector3 gyro;                    // deg/s
  Vector3 velocity;                // mm/s, signed
  Vector3 angle;                   // degrees, signed
  Vector3 displacement;            // micrometres, signed
};

enum class DataMode : uint8_t {
  kUnknown = 0,
  kDefault,  // 32-byte packets
  kInstant,  // 40-byte packets
};

// Converts a battery voltage (volts) to a percentage using the official
// app's table (single-cell range up to 5.5 V; a separate 2-cell table
// above that), linearly interpolated and clamped to 0..100.
float BatteryPercentFromVolts(float volts);

// Total packet length for a type byte when it is unambiguous, or 0 if
// unknown. For 0x61 this returns the Default length (32); the Decoder
// decides between 32 and 40 from the stream, never from this function.
size_t PacketLenFor(uint8_t packet_type);

// Callback invoked once per decoded "Now data" packet, in stream order,
// from inside Decoder::Feed (so from the BLE task on the ESP32). A single
// BLE notification carries several packets (4 observed), so reading()
// alone would lose all but the last: use this sink to see every sample.
using InstantSink = void (*)(const InstantSample &sample, void *ctx);

// Decoder accumulates raw BLE notification bytes into packets.
//
// It keeps the latest value of every Default-mode field across packets: a
// 0x61 broadcast carries all measurement registers at once, while a 0x71
// read-back refreshes only the eight registers in its block.
//
// Length detection for 0x61: both formats start `55 61`. The first time
// it sees `55 61` with 42 bytes buffered it checks which of offset 32 or
// offset 40 holds the next packet header (`55 61` or `55 71`). Exactly
// one match locks the length. While locked, a packet whose following
// header is present but invalid unlocks it again, so a mode switch in the
// middle of a stream (Default <-> Now data) re-detects by itself.
class Decoder {
 public:
  // Feeds a raw notification payload, which may hold several packets or
  // split one across calls. Returns true if the Default-mode reading()
  // changed (a Default 0x61 or a 0x71 read-back). "Now data" packets
  // return false here and are delivered through the InstantSink instead.
  bool Feed(const uint8_t *data, size_t len);

  const SensorReading &reading() const { return current_; }

  // Most recently decoded "Now data" packet, valid once mode() == kInstant
  // has been seen. Prefer the sink; this is only a convenience for tests.
  const InstantSample &last_instant() const { return last_instant_; }

  void SetInstantSink(InstantSink sink, void *ctx) {
    sink_ = sink;
    sink_ctx_ = ctx;
  }

  // Skips 0x61 length detection when the mode is known by other means,
  // for example right after this firmware wrote register 0x96, or when a
  // lone packet has to be decoded without waiting for the next header.
  // Pass kOutputPacketLen (Default), kInstantPacketLen (Now data) or 0 to
  // go back to detection. A wrong hint corrects itself: a packet whose
  // following header is invalid unlocks the length again.
  void SetExpectedOutputLength(size_t len) {
    output_len_ = (len == kOutputPacketLen || len == kInstantPacketLen) ? len : 0;
  }

  // Mode of the last 0x61 packet decoded.
  DataMode mode() const { return mode_; }

  // Diagnostics.
  uint32_t resyncs() const { return resyncs_; }
  uint32_t rejected_packets() const { return rejected_; }

 private:
  static constexpr size_t kBufCap = 128;
  // Bytes needed to evaluate both length hypotheses: the longer packet
  // plus the two header bytes that follow it.
  static constexpr size_t kDetectBytes = kInstantPacketLen + 2;

  bool Drain();
  void Consume(size_t n);
  static bool IsHeader(const uint8_t *p);

  bool ProcessPacket(const uint8_t *pkt, size_t len);
  bool DecodeOutput(const uint8_t *pkt);
  bool DecodeInstant(const uint8_t *pkt);
  bool DecodeRegisterBlock(const uint8_t *pkt);
  bool ApplyRegister(uint8_t reg, float raw);

  uint8_t buf_[kBufCap] = {};
  size_t len_ = 0;
  size_t output_len_ = 0;  // 0 = not locked yet, else 32 or 40
  DataMode mode_ = DataMode::kUnknown;
  SensorReading current_;
  InstantSample last_instant_;
  InstantSink sink_ = nullptr;
  void *sink_ctx_ = nullptr;
  uint32_t resyncs_ = 0;
  uint32_t rejected_ = 0;
};

// One contiguous block of acceleration samples, ready to publish. The
// samples are raw int16 counts (scale with kAccelScaleG) so the window is
// small and loses nothing. Samples are guaranteed evenly spaced: the
// accumulator restarts the window whenever the chip clock shows a gap.
struct WaveformWindow {
  uint32_t start_chip_ms = 0;  // chip clock of the first sample
  uint32_t end_chip_ms = 0;    // chip clock of the last sample
  uint16_t count = 0;          // samples filled (== N when complete)
  uint32_t gaps = 0;           // gaps seen since the previous window
  int16_t ax[WAVEFORM_WINDOW_SAMPLES] = {};
  int16_t ay[WAVEFORM_WINDOW_SAMPLES] = {};
  int16_t az[WAVEFORM_WINDOW_SAMPLES] = {};

  // Sampling rate implied by the chip clock over the whole window, or 0
  // if the window is not complete. Computed over (count - 1) intervals.
  float SampleRateHz() const;
};

// Builds fixed-size, gap-free windows out of InstantSamples.
class WaveformAccumulator {
 public:
  // Adds a sample. Returns true when the window just became complete; its
  // contents stay valid in window() until the next Push, which starts a
  // new window. A gap (non-increasing timestamp, or an interval far from
  // the one this window started with) discards the partial window and
  // starts again at the current sample, because an FFT over unevenly
  // spaced samples would be wrong.
  bool Push(const InstantSample &sample);

  const WaveformWindow &window() const { return window_; }

  // Forgets the stream position (call on reconnect). Does not clear the
  // sample arrays; count == 0 marks the window empty.
  void Reset() {
    window_.count = 0;
    window_.gaps = 0;
    last_ms_ = 0;
    step_ms_ = 0;
    complete_ = false;
    have_last_ = false;
  }

  uint32_t gaps_total() const { return gaps_total_; }
  uint32_t windows_completed() const { return windows_completed_; }

 private:
  WaveformWindow window_;
  uint32_t last_ms_ = 0;
  uint32_t step_ms_ = 0;  // interval learned from the first two samples
  bool complete_ = false;
  bool have_last_ = false;
  uint32_t gaps_total_ = 0;
  uint32_t windows_completed_ = 0;
};

// Smoothed RMS of the signed "Now data" channels, so zenith/readings can keep
// publishing velocity / displacement / angle while the sensor is in Now
// data mode (it stops sending its own amplitudes in that mode). It is an
// exponentially weighted mean of squares with a ~1 s time constant, updated
// per sample from the chip clock, so it needs no window bookkeeping and no
// reads from the sensor (reads disturb the 100 samples/s stream: see the
// repo docs).
//
// These are the node's own RMS values, not the sensor's amplitude registers
// (the manual does not define how those are computed), so consumers should
// treat them as "RMS of the instantaneous signal".
class InstantStats {
 public:
  void Add(const InstantSample &sample);

  // Number of samples folded in since Reset().
  uint32_t count() const { return count_; }

  // Fills velocity (mm/s), displacement (um) and angle (deg) with the
  // current RMS per axis. Returns false until enough samples have been seen
  // for the average to mean something (kMinSamples).
  bool Rms(SensorReading *out) const;

  void Reset() { *this = InstantStats(); }

 private:
  static constexpr uint32_t kMinSamples = 20;
  static constexpr float kTimeConstantMs = 1000.0f;

  float mean_sq_[9] = {};  // vel xyz, disp xyz, angle xyz
  uint32_t last_ms_ = 0;
  uint32_t count_ = 0;
};

// Dominant frequency (Hz) per axis (X, Y, Z) of an acceleration window, by a
// Hann-windowed FFT with parabolic peak interpolation. The DC bin and
// everything below kMinDominantHz are ignored (the sensor's own frequency
// register is documented as 5-100 Hz). `fs_hz` is the window's sample rate
// (WaveformWindow::SampleRateHz()). Writes 0 for an axis with no signal.
// Uses static scratch memory: call it from one task only.
constexpr float kMinDominantHz = 5.0f;
void DominantFrequencies(const WaveformWindow &window, float fs_hz,
                         float out_xyz[3]);

}  // namespace wtvb01
