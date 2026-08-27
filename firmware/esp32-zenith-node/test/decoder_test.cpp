// Host-side test for the WTVB01 decoder.
//
// Replays the same real captured sensor bytes the Go tests use, so both
// implementations are checked against identical input. Build and run:
//
//   c++ -std=c++17 -I src -o /tmp/wtvb01_test test/decoder_test.cpp src/wtvb01.cpp
//   /tmp/wtvb01_test ../../internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex

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

  if (failures > 0) {
    std::printf("\n%d check(s) FAILED\n", failures);
    return 1;
  }
  std::printf("\nall checks passed\n");
  return 0;
}
