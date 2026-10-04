// Host-side test for the WTVB01 decoder.
//
// Replays the same real captured sensor bytes the Go tests use, so both
// implementations are checked against identical input. Build and run:
//
//   c++ -std=c++17 -I src -o /tmp/wtvb01_test test/decoder_test.cpp src/wtvb01.cpp
//   /tmp/wtvb01_test ../../internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "wtvb01.h"

namespace {

int failures = 0;

void Check(bool ok, const char *what) {
  if (!ok) {
    std::printf("  FAIL: %s\n", what);
    failures++;
  }
}

std::vector<uint8_t> FromHex(const std::string &hex) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(
        static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::vector<std::vector<uint8_t>> LoadCapture(const char *path) {
  std::vector<std::vector<uint8_t>> payloads;
  std::ifstream f(path);
  if (!f) {
    std::printf("  FAIL: cannot open %s\n", path);
    failures++;
    return payloads;
  }
  std::string line;
  while (std::getline(f, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    payloads.push_back(FromHex(line));
  }
  return payloads;
}

void TestPacketLengths() {
  std::printf("TestPacketLengths\n");
  Check(wtvb01::PacketLenFor(wtvb01::kPacketTypeOutput) == 32,
        "0x61 packet is 32 bytes");
  Check(wtvb01::PacketLenFor(wtvb01::kPacketTypeRegister) == 20,
        "0x71 packet is 20 bytes");
  Check(wtvb01::PacketLenFor(0x99) == 0, "unknown type has no length");
}

void TestTemperatureScale() {
  std::printf("TestTemperatureScale\n");
  // Real 0x71 read-back of block 0x3A. Register 0x40 reads 0x0984 =
  // 2436 -> 24.36 C.
  const auto pkt = FromHex("55713a00110007001200e900f300390084092c01");

  wtvb01::Decoder d;
  Check(d.Feed(pkt.data(), pkt.size()), "register block decodes");
  Check(std::fabs(d.reading().device.temperature - 24.36f) < 0.01f,
        "temperature is 24.36 C");
}

void TestRealCapture(const char *path) {
  std::printf("TestRealCapture\n");
  const auto payloads = LoadCapture(path);
  if (payloads.empty()) {
    return;
  }

  wtvb01::Decoder d;
  int decoded = 0;
  for (const auto &p : payloads) {
    if (d.Feed(p.data(), p.size())) {
      decoded++;
    }
  }
  Check(decoded > 0, "packets decoded from the real capture");

  const auto &r = d.reading();
  std::printf("  last: vel(%.1f,%.1f,%.1f) disp(%.0f,%.0f,%.0f) "
              "ang(%.3f,%.3f,%.3f) freq(%.0f,%.0f,%.0f) temp=%.2f\n",
              r.velocity.x, r.velocity.y, r.velocity.z, r.displacement.x,
              r.displacement.y, r.displacement.z, r.angle.x, r.angle.y,
              r.angle.z, r.frequency.x, r.frequency.y, r.frequency.z,
              r.device.temperature);

  Check(r.device.temperature > 20.0f && r.device.temperature < 30.0f,
        "temperature is a plausible ambient value");
  Check(std::fabs(r.angle.x) <= 180.0f && std::fabs(r.angle.y) <= 180.0f &&
            std::fabs(r.angle.z) <= 180.0f,
        "angles within [-180, 180]");
}

// The 0x61 broadcast and the 0x71 read-backs independently encode the
// same registers, so decoding either must agree.
void TestOutputAndRegisterAgree(const char *path) {
  std::printf("TestOutputAndRegisterAgree\n");
  const auto payloads = LoadCapture(path);
  if (payloads.empty()) {
    return;
  }

  std::vector<uint8_t> stream;
  for (const auto &p : payloads) {
    stream.insert(stream.end(), p.begin(), p.end());
  }

  wtvb01::SensorReading from_output, from_register;
  bool got_output = false, got_register = false;

  for (size_t i = 0; i + 2 <= stream.size();) {
    if (stream[i] != wtvb01::kSyncByte) {
      i++;
      continue;
    }
    const size_t want = wtvb01::PacketLenFor(stream[i + 1]);
    if (want == 0 || i + want > stream.size()) {
      i++;
      continue;
    }

    if (stream[i + 1] == wtvb01::kPacketTypeOutput && !got_output) {
      wtvb01::Decoder d;
      // A lone packet has no following header to detect the length from.
      d.SetExpectedOutputLength(wtvb01::kOutputPacketLen);
      if (d.Feed(&stream[i], want)) {
        from_output = d.reading();
        got_output = true;
      }
    } else if (stream[i + 1] == wtvb01::kPacketTypeRegister &&
               stream[i + 2] == wtvb01::kRegVelocityX && !got_register) {
      wtvb01::Decoder d;
      if (d.Feed(&stream[i], want)) {
        from_register = d.reading();
        got_register = true;
      }
    }
    i += want;
  }

  if (!got_output || !got_register) {
    std::printf("  SKIP: capture lacked both packet types\n");
    return;
  }

  Check(from_output.velocity.x == from_register.velocity.x &&
            from_output.velocity.y == from_register.velocity.y &&
            from_output.velocity.z == from_register.velocity.z,
        "velocity agrees between 0x61 and 0x71");
  Check(from_output.angle.x == from_register.angle.x &&
            from_output.angle.y == from_register.angle.y &&
            from_output.angle.z == from_register.angle.z,
        "angle agrees between 0x61 and 0x71");
  // Temperature is recomputed continuously, so it drifts slightly.
  Check(std::fabs(from_output.device.temperature -
                  from_register.device.temperature) < 1.0f,
        "temperature agrees between 0x61 and 0x71");
}

void TestResyncOnGarbage() {
  std::printf("TestResyncOnGarbage\n");
  const auto real = FromHex("55713a00110007001200e900f300390084092c01");
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x12, wtvb01::kSyncByte, 0x99};
  stream.insert(stream.end(), real.begin(), real.end());

  wtvb01::Decoder d;
  Check(d.Feed(stream.data(), stream.size()), "decoder resyncs after garbage");
  Check(std::fabs(d.reading().device.temperature - 24.36f) < 0.01f,
        "temperature correct after resync");
}

void TestSplitWrites() {
  std::printf("TestSplitWrites\n");
  const auto pkt = FromHex("55713a00110007001200e900f300390084092c01");

  wtvb01::Decoder d;
  bool decoded = false;
  for (size_t i = 0; i < pkt.size(); i++) {
    if (d.Feed(&pkt[i], 1)) {
      decoded = true;
    }
  }
  Check(decoded, "packet decoded across single-byte writes");
  Check(std::fabs(d.reading().device.temperature - 24.36f) < 0.01f,
        "temperature correct after split writes");
}

// ------------------------------------------------ Now data / batteries
//
// Fixtures in test/testdata were captured from the physical sensor
// (firmware 10057.2.7, return rate 100 Hz) through the official app's HCI
// snoop log. Each line is one BLE notification carrying 4 packets.

const char *kDefaultCapture = "test/testdata/capture-default-100hz.hex";
const char *kNowDataCapture = "test/testdata/capture-now-data-100hz.hex";
const char *kModeSwitchCapture = "test/testdata/capture-mode-switch.hex";

void CollectInstant(const wtvb01::InstantSample &s, void *ctx) {
  static_cast<std::vector<wtvb01::InstantSample> *>(ctx)->push_back(s);
}

// Feeds `payloads` to `d`, optionally re-chunked to `chunk` bytes per call
// (0 = as captured) to mimic a low ATT MTU.
void FeedAll(wtvb01::Decoder &d,
             const std::vector<std::vector<uint8_t>> &payloads, size_t chunk) {
  for (const auto &p : payloads) {
    if (chunk == 0) {
      d.Feed(p.data(), p.size());
      continue;
    }
    for (size_t i = 0; i < p.size(); i += chunk) {
      d.Feed(&p[i], std::min(chunk, p.size() - i));
    }
  }
}

void TestBatteryPercent() {
  std::printf("TestBatteryPercent\n");
  // Official app table (single-cell branch).
  Check(wtvb01::BatteryPercentFromVolts(4.39f) == 100.0f, "4.39 V is 100%");
  Check(std::fabs(wtvb01::BatteryPercentFromVolts(3.70f) - 15.0f) < 0.01f,
        "3.70 V is 15%");
  Check(wtvb01::BatteryPercentFromVolts(3.0f) == 0.0f, "below table is 0%");
  Check(std::fabs(wtvb01::BatteryPercentFromVolts(3.59f) - 7.5f) < 0.01f,
        "3.59 V interpolates to 7.5%");
  // > 5.5 V uses the second table.
  Check(std::fabs(wtvb01::BatteryPercentFromVolts(8.0f) - 70.0f) < 0.01f,
        "8.0 V interpolates to 70% on the pack table");
}

void TestDefaultCaptureFromNewFirmware() {
  std::printf("TestDefaultCaptureFromNewFirmware\n");
  const auto payloads = LoadCapture(kDefaultCapture);
  if (payloads.empty()) {
    return;
  }
  wtvb01::Decoder d;
  std::vector<wtvb01::InstantSample> instants;
  d.SetInstantSink(CollectInstant, &instants);
  FeedAll(d, payloads, 0);

  const auto &r = d.reading();
  Check(d.mode() == wtvb01::DataMode::kDefault, "mode detected as Default");
  Check(instants.empty(), "no instant samples in Default mode");
  Check(d.resyncs() == 0 && d.rejected_packets() == 0,
        "aligned stream needs no resync");
  // Hardware: value 14 matched the 0x64 read-back (437..441 -> ~4.39 V).
  Check(r.device.power_raw >= 430.0f && r.device.power_raw <= 445.0f,
        "battery raw is in the observed 430..445 range");
  Check(std::fabs(r.device.battery_volts - r.device.power_raw / 100.0f) < 1e-4f,
        "battery volts is raw/100");
  Check(r.device.battery_percent == 100.0f, "battery reads 100%");
  Check(r.device.alarm_status == 0, "no alarm flags");
  Check(r.device.temperature > 25.0f && r.device.temperature < 45.0f,
        "module temperature is plausible");
}

void TestNowDataCapture() {
  std::printf("TestNowDataCapture\n");
  const auto payloads = LoadCapture(kNowDataCapture);
  if (payloads.empty()) {
    return;
  }
  wtvb01::Decoder d;
  std::vector<wtvb01::InstantSample> samples;
  d.SetInstantSink(CollectInstant, &samples);
  FeedAll(d, payloads, 0);

  Check(d.mode() == wtvb01::DataMode::kInstant, "mode detected as Now data");
  Check(samples.size() == payloads.size() * 4,
        "4 samples per notification, none lost");
  Check(d.resyncs() == 0 && d.rejected_packets() == 0,
        "aligned stream needs no resync and rejects nothing");

  bool all_10ms = samples.size() > 1;
  bool in_range = true;
  for (size_t i = 0; i < samples.size(); i++) {
    if (i > 0 && samples[i].chip_ms - samples[i - 1].chip_ms != 10) {
      all_10ms = false;
    }
    if (std::fabs(samples[i].accel.x) > 16.0f ||
        std::fabs(samples[i].accel.y) > 16.0f ||
        std::fabs(samples[i].accel.z) > 16.0f) {
      in_range = false;
    }
  }
  Check(all_10ms, "chip clock advances exactly 10 ms per sample (100 Hz)");
  Check(in_range, "acceleration stays within +-16 g");
  Check(samples.front().year == 15 && samples.front().month == 1,
        "chip clock starts at the 2015-01-01 default");
  Check(std::fabs(samples[0].accel.x - samples[0].acc_raw[0] * (16.0f / 32768.0f)) <
            1e-6f,
        "acceleration scale is 16 g / 32768");
}

void TestNowDataSplitWrites() {
  std::printf("TestNowDataSplitWrites\n");
  const auto payloads = LoadCapture(kNowDataCapture);
  if (payloads.empty()) {
    return;
  }
  std::vector<wtvb01::InstantSample> whole, split;
  {
    wtvb01::Decoder d;
    d.SetInstantSink(CollectInstant, &whole);
    FeedAll(d, payloads, 0);
  }
  for (size_t chunk : {20u, 7u, 1u}) {
    split.clear();
    wtvb01::Decoder d;
    d.SetInstantSink(CollectInstant, &split);
    FeedAll(d, payloads, chunk);
    char label[96];
    std::snprintf(label, sizeof(label),
                  "same sample count with %zu-byte chunks (low MTU)", chunk);
    Check(split.size() == whole.size(), label);
    bool same = split.size() == whole.size();
    for (size_t i = 0; same && i < whole.size(); i++) {
      same = split[i].chip_ms == whole[i].chip_ms &&
             split[i].acc_raw[0] == whole[i].acc_raw[0] &&
             split[i].acc_raw[1] == whole[i].acc_raw[1] &&
             split[i].acc_raw[2] == whole[i].acc_raw[2];
    }
    std::snprintf(label, sizeof(label),
                  "identical samples with %zu-byte chunks", chunk);
    Check(same, label);
  }
}

void TestModeSwitch() {
  std::printf("TestModeSwitch\n");
  const auto payloads = LoadCapture(kModeSwitchCapture);
  if (payloads.empty()) {
    return;
  }
  wtvb01::Decoder d;
  std::vector<wtvb01::InstantSample> samples;
  d.SetInstantSink(CollectInstant, &samples);
  bool default_seen = false;
  for (const auto &p : payloads) {
    if (d.Feed(p.data(), p.size())) {
      default_seen = true;
    }
  }
  Check(default_seen, "Default packets before the switch were decoded");
  Check(!samples.empty(), "Now data packets after the switch were decoded");
  Check(d.mode() == wtvb01::DataMode::kInstant, "ends in Now data mode");
  Check(d.rejected_packets() == 0, "no packet rejected across the switch");

  // After the switch, samples must be contiguous: the mixed 32+40 byte
  // notification must not poison the samples that follow it.
  bool contiguous = samples.size() > 1;
  for (size_t i = 1; i < samples.size(); i++) {
    if (samples[i].chip_ms - samples[i - 1].chip_ms != 10) {
      contiguous = false;
    }
  }
  Check(contiguous, "Now data samples after the switch are contiguous (10 ms)");
}

void TestExpectedLengthHint() {
  std::printf("TestExpectedLengthHint\n");
  const auto payloads = LoadCapture(kDefaultCapture);
  if (payloads.empty()) {
    return;
  }
  // One lone Default packet decodes immediately with the hint...
  wtvb01::Decoder hinted;
  hinted.SetExpectedOutputLength(wtvb01::kOutputPacketLen);
  Check(hinted.Feed(payloads[0].data(), wtvb01::kOutputPacketLen),
        "lone 32-byte packet decodes with the length hint");
  // ...and a wrong hint corrects itself on a Now data stream.
  const auto instant = LoadCapture(kNowDataCapture);
  if (instant.empty()) {
    return;
  }
  wtvb01::Decoder wrong;
  std::vector<wtvb01::InstantSample> samples;
  wrong.SetInstantSink(CollectInstant, &samples);
  wrong.SetExpectedOutputLength(wtvb01::kOutputPacketLen);
  FeedAll(wrong, instant, 0);
  Check(wrong.mode() == wtvb01::DataMode::kInstant &&
            samples.size() >= instant.size() * 4 - 4,
        "a wrong length hint self-corrects (at most one notification lost)");
}

wtvb01::InstantSample MakeSample(uint32_t chip_ms, int16_t ax = 0);

void TestInstantStats() {
  std::printf("TestInstantStats\n");
  wtvb01::InstantStats st;
  wtvb01::SensorReading r;
  Check(!st.Rms(&r), "no RMS before enough samples");
  // 25 Hz sine, amplitude 100 mm/s on X, 100 Hz sampling: RMS = 100/sqrt(2).
  for (int i = 0; i < 400; i++) {
    wtvb01::InstantSample s = MakeSample(1000 + i * 10, 0);
    s.velocity.x = 100.0f * std::sin(2.0f * 3.14159265f * 25.0f * i / 100.0f);
    s.velocity.y = 5.0f;  // constant 5 -> RMS 5
    st.Add(s);
  }
  Check(st.Rms(&r), "RMS available after enough samples");
  Check(std::fabs(r.velocity.x - 70.71f) < 3.0f, "X RMS ~ 100/sqrt(2)");
  Check(std::fabs(r.velocity.y - 5.0f) < 0.1f, "constant 5 has RMS 5");
  Check(r.velocity.z == 0.0f, "silent axis has RMS 0");

  // After the signal stops, the RMS decays to exactly 0 (no 1e-19 leftovers).
  for (int i = 400; i < 3000; i++) {
    st.Add(MakeSample(1000 + i * 10, 0));
  }
  Check(st.Rms(&r) && r.velocity.x == 0.0f && r.velocity.y == 0.0f,
        "RMS reaches exactly 0 after the signal stops");
}

void TestDominantFrequencies() {
  std::printf("TestDominantFrequencies\n");
  struct Case { float f; float fs; } cases[] = {{25.0f, 100.0f}, {12.3f, 100.0f},
                                                {37.77f, 100.0f}, {60.0f, 200.0f}};
  for (const auto &c : cases) {
    wtvb01::WaveformWindow w;
    for (int i = 0; i < WAVEFORM_WINDOW_SAMPLES; i++) {
      const float t = i / c.fs;
      w.ax[i] = static_cast<int16_t>(2000.0f * std::sin(2.0f * 3.14159265f * c.f * t));
      w.ay[i] = static_cast<int16_t>(1500.0f * std::sin(2.0f * 3.14159265f * 2.0f * c.f * t) + 800.0f);
      w.az[i] = 700;  // constant: no signal
    }
    w.count = WAVEFORM_WINDOW_SAMPLES;
    float out[3];
    wtvb01::DominantFrequencies(w, c.fs, out);
    char label[96];
    std::snprintf(label, sizeof(label), "X peak %.2f Hz found as %.2f (fs %.0f)",
                  c.f, out[0], c.fs);
    Check(std::fabs(out[0] - c.f) < 0.25f, label);
    if (2.0f * c.f < c.fs / 2.0f) {
      std::snprintf(label, sizeof(label), "Y 2nd harmonic %.2f Hz found as %.2f",
                    2.0f * c.f, out[1]);
      Check(std::fabs(out[1] - 2.0f * c.f) < 0.3f, label);
    }
    Check(out[2] == 0.0f, "constant axis reports 0 Hz");
  }
  // Resting noise (a few counts) is not a signal: 0 Hz, not a noise peak.
  {
    wtvb01::WaveformWindow quiet;
    unsigned seed = 12345;
    for (int i = 0; i < WAVEFORM_WINDOW_SAMPLES; i++) {
      seed = seed * 1103515245u + 12345u;
      quiet.ax[i] = static_cast<int16_t>((seed >> 16) % 5) - 2;  // -2..+2 counts
      quiet.ay[i] = static_cast<int16_t>(2048 + ((seed >> 20) % 5) - 2);
      quiet.az[i] = static_cast<int16_t>(-64 + ((seed >> 24) % 3) - 1);
    }
    quiet.count = WAVEFORM_WINDOW_SAMPLES;
    float q[3] = {9, 9, 9};
    wtvb01::DominantFrequencies(quiet, 100.0f, q);
    Check(q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f,
          "noise below kMinSignalRmsCounts reports 0 Hz");
  }
  // An incomplete window yields nothing.
  wtvb01::WaveformWindow partial;
  partial.count = 10;
  float out[3] = {1, 1, 1};
  wtvb01::DominantFrequencies(partial, 100.0f, out);
  Check(out[0] == 0.0f && out[1] == 0.0f && out[2] == 0.0f,
        "partial window reports 0 Hz");
}

void TestFrequencyFromRealCapture() {
  std::printf("TestFrequencyFromRealCapture\n");
  const auto payloads = LoadCapture(kNowDataCapture);
  if (payloads.empty()) {
    return;
  }
  struct Ctx {
    wtvb01::WaveformAccumulator acc;
    bool done = false;
    float freq[3] = {0, 0, 0};
    float rate = 0;
  } ctx;
  wtvb01::Decoder d;
  d.SetInstantSink(
      [](const wtvb01::InstantSample &s, void *p) {
        auto *c = static_cast<Ctx *>(p);
        if (c->acc.Push(s) && !c->done) {
          c->rate = c->acc.window().SampleRateHz();
          wtvb01::DominantFrequencies(c->acc.window(), c->rate, c->freq);
          c->done = true;
        }
      },
      &ctx);
  FeedAll(d, payloads, 0);
  Check(ctx.done, "a window was analysed");
  std::printf("  dominant freq of the real capture: %.2f %.2f %.2f Hz\n",
              ctx.freq[0], ctx.freq[1], ctx.freq[2]);
  bool in_band = true;
  for (float f : ctx.freq) {
    if (f < wtvb01::kMinDominantHz || f > ctx.rate / 2.0f) in_band = false;
  }
  Check(in_band, "dominant frequencies lie within [5 Hz, Nyquist]");
}

void TestImplausibleTimeRejected() {
  std::printf("TestImplausibleTimeRejected\n");
  // Three well-formed 40-byte frames whose "time" is impossible (month 0).
  std::vector<uint8_t> stream;
  for (int i = 0; i < 3; i++) {
    std::vector<uint8_t> pkt(40, 0);
    pkt[0] = wtvb01::kSyncByte;
    pkt[1] = wtvb01::kPacketTypeOutput;
    pkt[2] = 15;  // year
    pkt[3] = 0;   // month 0 -> invalid
    pkt[4] = 1;
    stream.insert(stream.end(), pkt.begin(), pkt.end());
  }
  wtvb01::Decoder d;
  std::vector<wtvb01::InstantSample> samples;
  d.SetInstantSink(CollectInstant, &samples);
  d.Feed(stream.data(), stream.size());
  Check(samples.empty(), "implausible calendar time yields no sample");
  Check(d.rejected_packets() > 0, "implausible packets are counted");
}

wtvb01::InstantSample MakeSample(uint32_t chip_ms, int16_t ax) {
  wtvb01::InstantSample s;
  s.chip_ms = chip_ms;
  s.acc_raw[0] = ax;
  return s;
}

void TestWaveformAccumulator() {
  std::printf("TestWaveformAccumulator\n");
  constexpr uint32_t N = WAVEFORM_WINDOW_SAMPLES;

  {
    wtvb01::WaveformAccumulator acc;
    bool done = false;
    uint32_t completed_at = 0;
    for (uint32_t i = 0; i < N; i++) {
      if (acc.Push(MakeSample(1000 + i * 10, static_cast<int16_t>(i)))) {
        done = true;
        completed_at = i + 1;
      }
    }
    Check(done && completed_at == N, "window completes on the Nth sample");
    Check(std::fabs(acc.window().SampleRateHz() - 100.0f) < 0.01f,
          "window reports 100 Hz from the chip clock");
    Check(acc.window().ax[0] == 0 && acc.window().ax[N - 1] == N - 1,
          "samples stored in order");
    Check(acc.window().gaps == 0 && acc.gaps_total() == 0, "no gaps");

    // A second window continues the same stream without a gap.
    bool second = false;
    for (uint32_t i = N; i < 2 * N; i++) {
      second = acc.Push(MakeSample(1000 + i * 10));
    }
    Check(second && acc.windows_completed() == 2,
          "contiguous second window completes");
    Check(acc.gaps_total() == 0, "no gap counted between windows");
  }

  {
    // 200 Hz stream (5 ms): step is learned, not assumed.
    wtvb01::WaveformAccumulator acc;
    for (uint32_t i = 0; i < N; i++) {
      acc.Push(MakeSample(5000 + i * 5));
    }
    Check(std::fabs(acc.window().SampleRateHz() - 200.0f) < 0.01f,
          "step learned from the stream: 200 Hz");
  }

  {
    // A dropped chunk mid-window restarts the window.
    wtvb01::WaveformAccumulator acc;
    for (uint32_t i = 0; i < 100; i++) {
      acc.Push(MakeSample(i * 10));
    }
    acc.Push(MakeSample(100 * 10 + 40));  // 4 samples missing
    Check(acc.gaps_total() == 1, "missing samples counted as a gap");
    Check(acc.window().count == 1, "partial window discarded, restarted");
    bool done = false;
    for (uint32_t i = 1; i < N; i++) {
      done = acc.Push(MakeSample(1040 + i * 10));
    }
    Check(done, "window completes after the restart");
    Check(acc.window().gaps == 1, "window records the gap that preceded it");
  }

  {
    // Clock going backwards (sensor restarted) is a gap too.
    wtvb01::WaveformAccumulator acc;
    acc.Push(MakeSample(5000));
    acc.Push(MakeSample(5010));
    acc.Push(MakeSample(20));
    Check(acc.gaps_total() == 1 && acc.window().count == 1,
          "backwards clock restarts the window");
  }
}

void TestWaveformFromRealCapture() {
  std::printf("TestWaveformFromRealCapture\n");
  const auto payloads = LoadCapture(kNowDataCapture);
  if (payloads.empty()) {
    return;
  }
  struct Ctx {
    wtvb01::WaveformAccumulator acc;
    float rate_when_complete = 0.0f;
  } ctx;
  wtvb01::Decoder d;
  d.SetInstantSink(
      [](const wtvb01::InstantSample &s, void *p) {
        auto *c = static_cast<Ctx *>(p);
        if (c->acc.Push(s)) {
          c->rate_when_complete = c->acc.window().SampleRateHz();
        }
      },
      &ctx);
  FeedAll(d, payloads, 0);
  Check(ctx.acc.windows_completed() >= 1,
        "real capture fills at least one window");
  Check(ctx.acc.gaps_total() == 0, "real capture has no gaps");
  Check(std::fabs(ctx.rate_when_complete - 100.0f) < 0.5f,
        "real capture window measures ~100 Hz");
}

}  // namespace

int main(int argc, char **argv) {
  const char *capture =
      argc > 1 ? argv[1]
               : "../../internal/protocol/wtvb01/testdata/"
                 "capture-wtvb01-bt50.hex";

  TestPacketLengths();
  TestTemperatureScale();
  TestRealCapture(capture);
  TestOutputAndRegisterAgree(capture);
  TestResyncOnGarbage();
  TestSplitWrites();
  TestBatteryPercent();
  TestDefaultCaptureFromNewFirmware();
  TestNowDataCapture();
  TestNowDataSplitWrites();
  TestModeSwitch();
  TestExpectedLengthHint();
  TestImplausibleTimeRejected();
  TestInstantStats();
  TestDominantFrequencies();
  TestFrequencyFromRealCapture();
  TestWaveformAccumulator();
  TestWaveformFromRealCapture();

  if (failures > 0) {
    std::printf("\n%d check(s) FAILED\n", failures);
    return 1;
  }
  std::printf("\nall checks passed\n");
  return 0;
}
