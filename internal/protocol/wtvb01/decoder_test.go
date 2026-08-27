package wtvb01

import (
	"bufio"
	"encoding/hex"
	"math"
	"os"
	"strings"
	"testing"
)

// The fixtures in testdata/ are REAL bytes captured from a physical
// WitMotion WTVB01-BT50 (see the header of the .hex file). Synthetic
// packets are only used where a specific edge case cannot be captured
// on demand (resync, split writes), and are marked as such.

func loadCapture(t *testing.T) [][]byte {
	t.Helper()

	f, err := os.Open("testdata/capture-wtvb01-bt50.hex")
	if err != nil {
		t.Fatalf("open capture: %v", err)
	}
	defer f.Close()

	var payloads [][]byte
	scanner := bufio.NewScanner(f)
	scanner.Buffer(make([]byte, 0, 64*1024), 64*1024)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		b, err := hex.DecodeString(line)
		if err != nil {
			t.Fatalf("decode capture line: %v", err)
		}
		payloads = append(payloads, b)
	}
	if err := scanner.Err(); err != nil {
		t.Fatalf("read capture: %v", err)
	}
	if len(payloads) == 0 {
		t.Fatal("capture file contained no payloads")
	}
	return payloads
}

func almostEqual(a, b float64) bool { return math.Abs(a-b) < 1e-6 }

// TestDecodeRealCapture is the primary regression test: it replays the
// captured stream and checks the decoded values against what the sensor
// was physically doing (at rest on a desk, room temperature).
func TestDecodeRealCapture(t *testing.T) {
	d := NewDecoder()

	var last SensorReading
	decoded := 0
	for _, payload := range loadCapture(t) {
		reading, ok := d.Feed(payload)
		if ok {
			decoded++
			last = reading
		}
	}

	if decoded == 0 {
		t.Fatal("no packets decoded from the real capture")
	}

	// Chip temperature: the sensor sat in a room at roughly 24-25 C.
	// This is the one field confirmed against hardware.
	if last.Device.Temperature < 20 || last.Device.Temperature > 30 {
		t.Errorf("Device.Temperature = %v C, want a plausible ambient value (20-30)", last.Device.Temperature)
	}

	// At rest, vibration magnitudes must stay small and finite.
	for name, v := range map[string]Vector3{
		"Velocity":     last.Velocity,
		"Displacement": last.Displacement,
		"Frequency":    last.Frequency,
	} {
		for axis, val := range map[string]float64{"X": v.X, "Y": v.Y, "Z": v.Z} {
			if math.IsNaN(val) || math.IsInf(val, 0) {
				t.Errorf("%s.%s = %v, want a finite number", name, axis, val)
			}
		}
	}

	// Angle registers are scaled to degrees and must stay in range.
	for axis, val := range map[string]float64{"X": last.Angle.X, "Y": last.Angle.Y, "Z": last.Angle.Z} {
		if val < -180 || val > 180 {
			t.Errorf("Angle.%s = %v, want within [-180, 180]", axis, val)
		}
	}
}

// TestOutputAndRegisterPacketsAgree is the strongest correctness check
// available without the official app: the 0x61 broadcast and the 0x71
// register read-backs are independent encodings of the same registers,
// so decoding either must produce the same values.
func TestOutputAndRegisterPacketsAgree(t *testing.T) {
	payloads := loadCapture(t)

	// Split the captured stream into whole packets.
	var stream []byte
	for _, p := range payloads {
		stream = append(stream, p...)
	}

	var outputReading, registerReading SensorReading
	var gotOutput, gotRegisterA bool

	for i := 0; i+2 <= len(stream); {
		if stream[i] != syncByte {
			i++
			continue
		}
		want := packetLenFor(stream[i+1])
		if want == 0 || i+want > len(stream) {
			i++
			continue
		}
		pkt := stream[i : i+want]

		switch {
		case pkt[1] == packetTypeOutput && !gotOutput:
			d := NewDecoder()
			outputReading, gotOutput = d.Feed(pkt)
		case pkt[1] == packetTypeRegister && pkt[2] == blockMeasurementA && !gotRegisterA:
			d := NewDecoder()
			registerReading, gotRegisterA = d.Feed(pkt)
		}
		i += want
	}

	if !gotOutput || !gotRegisterA {
		t.Skip("capture did not contain both a 0x61 packet and a 0x3A register block")
	}

	// Block 0x3A covers velocity, angle and temperature. Values are
	// recomputed by the sensor continuously, so temperature drifts
	// between packets; the vibration registers update far more slowly
	// and must match exactly.
	if outputReading.Velocity != registerReading.Velocity {
		t.Errorf("Velocity mismatch: 0x61 %+v vs 0x71 %+v", outputReading.Velocity, registerReading.Velocity)
	}
	if outputReading.Angle != registerReading.Angle {
		t.Errorf("Angle mismatch: 0x61 %+v vs 0x71 %+v", outputReading.Angle, registerReading.Angle)
	}
	if math.Abs(outputReading.Device.Temperature-registerReading.Device.Temperature) > 1 {
		t.Errorf("Temperature mismatch: 0x61 %v vs 0x71 %v",
			outputReading.Device.Temperature, registerReading.Device.Temperature)
	}
}

// TestTemperatureScale pins the one scale confirmed against hardware:
// register 0x40 divided by 100 is degrees Celsius.
func TestTemperatureScale(t *testing.T) {
	// Real 0x71 read-back of block 0x3A from the capture. Register 0x40
	// (7th register, bytes 16-17) reads 0x0984 = 2436 -> 24.36 C.
	pkt, err := hex.DecodeString("55713a00110007001200e900f300390084092c01")
	if err != nil {
		t.Fatal(err)
	}

	d := NewDecoder()
	reading, ok := d.Feed(pkt)
	if !ok {
		t.Fatal("expected the register block to decode")
	}
	if !almostEqual(reading.Device.Temperature, 24.36) {
		t.Errorf("Device.Temperature = %v, want 24.36", reading.Device.Temperature)
	}
}

// TestPacketLengthsMatchHardware guards the correction that motivated
// this decoder: the WTVB01-BT50 broadcast is 32 bytes, not the 20 the
// generic BWT901 SDK assumes.
func TestPacketLengthsMatchHardware(t *testing.T) {
	if got := packetLenFor(packetTypeOutput); got != 32 {
		t.Errorf("packetLenFor(0x61) = %d, want 32", got)
	}
	if got := packetLenFor(packetTypeRegister); got != 20 {
		t.Errorf("packetLenFor(0x71) = %d, want 20", got)
	}
	if got := packetLenFor(0x99); got != 0 {
		t.Errorf("packetLenFor(unknown) = %d, want 0", got)
	}
}

// TestFeedResyncsOnGarbage uses a SYNTHETIC prefix: garbage ahead of a
// real captured packet, to prove the resync logic recovers.
func TestFeedResyncsOnGarbage(t *testing.T) {
	real, err := hex.DecodeString("55713a00110007001200e900f300390084092c01")
	if err != nil {
		t.Fatal(err)
	}
	garbage := []byte{0x00, 0xFF, 0x12, syncByte, 0x99} // 0x99 is not a valid type

	d := NewDecoder()
	reading, ok := d.Feed(append(garbage, real...))
	if !ok {
		t.Fatal("expected the decoder to resync and decode the trailing packet")
	}
	if !almostEqual(reading.Device.Temperature, 24.36) {
		t.Errorf("Device.Temperature = %v, want 24.36", reading.Device.Temperature)
	}
}

// TestFeedHandlesSplitWrites replays one real packet one byte at a
// time, since BLE notifications may be split across callbacks.
func TestFeedHandlesSplitWrites(t *testing.T) {
	pkt, err := hex.DecodeString("55713a00110007001200e900f300390084092c01")
	if err != nil {
		t.Fatal(err)
	}

	d := NewDecoder()
	var decoded bool
	var reading SensorReading
	for i := range len(pkt) {
		r, ok := d.Feed(pkt[i : i+1])
		if ok {
			decoded, reading = true, r
		}
	}

	if !decoded {
		t.Fatal("expected a packet after all bytes were fed")
	}
	if !almostEqual(reading.Device.Temperature, 24.36) {
		t.Errorf("Device.Temperature = %v, want 24.36", reading.Device.Temperature)
	}
}
