package wtvb01

import (
	"bufio"
	"encoding/hex"
	"math"
	"os"
	"strings"
	"testing"
)

// These tests replay REAL bytes captured from the physical sensor
// (firmware 10057.2.7, return rate 100 Hz) through the official app's HCI
// snoop log on 2026-10-04: the headers of the .hex files say so. They are
// the same files the ESP32 firmware's host tests use, so the two decoders
// are checked against identical input. The expectations mirror
// firmware/esp32-zenith-node/test/decoder_test.cpp.

func loadFile(t *testing.T, name string) [][]byte {
	t.Helper()
	f, err := os.Open("testdata/" + name)
	if err != nil {
		t.Fatalf("open %s: %v", name, err)
	}
	defer f.Close()
	var out [][]byte
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 64*1024), 64*1024)
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		b, err := hex.DecodeString(line)
		if err != nil {
			t.Fatalf("decode line: %v", err)
		}
		out = append(out, b)
	}
	if len(out) == 0 {
		t.Fatalf("%s has no payloads", name)
	}
	return out
}

// feedAll feeds payloads to d, re-chunked to `chunk` bytes per call when
// chunk > 0 (a low ATT MTU).
func feedAll(d *Decoder, payloads [][]byte, chunk int) {
	for _, p := range payloads {
		if chunk <= 0 {
			d.Feed(p)
			continue
		}
		for i := 0; i < len(p); i += chunk {
			end := min(i+chunk, len(p))
			d.Feed(p[i:end])
		}
	}
}

func collect(d *Decoder) *[]InstantSample {
	var got []InstantSample
	d.OnInstant = func(s InstantSample) { got = append(got, s) }
	return &got
}

func TestBatteryPercent(t *testing.T) {
	cases := []struct{ v, want float64 }{
		{4.39, 100}, {3.70, 15}, {3.0, 0}, {3.59, 7.5}, {8.0, 70},
	}
	for _, c := range cases {
		if got := BatteryPercentFromVolts(c.v); math.Abs(got-c.want) > 0.01 {
			t.Errorf("BatteryPercentFromVolts(%v) = %v, want %v", c.v, got, c.want)
		}
	}
}

func TestDefaultCaptureFromNewFirmware(t *testing.T) {
	d := NewDecoder()
	got := collect(d)
	feedAll(d, loadFile(t, "capture-default-100hz.hex"), 0)

	if d.Mode() != DataModeDefault {
		t.Fatalf("mode = %v, want Default", d.Mode())
	}
	if len(*got) != 0 {
		t.Errorf("Default mode produced %d instant samples", len(*got))
	}
	if d.Resyncs() != 0 || d.Rejected() != 0 {
		t.Errorf("aligned stream needed resyncs=%d rejected=%d", d.Resyncs(), d.Rejected())
	}
	dev := d.current.Device
	// Hardware: value 14 matched the 0x64 read-back (437..441 -> ~4.39 V).
	if dev.PowerRaw < 430 || dev.PowerRaw > 445 {
		t.Errorf("battery raw %v outside the observed 430..445", dev.PowerRaw)
	}
	if math.Abs(dev.BatteryVolts-dev.PowerRaw/100) > 1e-9 {
		t.Errorf("BatteryVolts %v != raw/100", dev.BatteryVolts)
	}
	if dev.BatteryPercent != 100 {
		t.Errorf("battery percent = %v, want 100", dev.BatteryPercent)
	}
	if dev.AlarmStatus != 0 {
		t.Errorf("alarm = %d, want 0", dev.AlarmStatus)
	}
	if dev.Temperature < 25 || dev.Temperature > 45 {
		t.Errorf("temperature %v implausible", dev.Temperature)
	}
}

func TestNowDataCapture(t *testing.T) {
	payloads := loadFile(t, "capture-now-data-100hz.hex")
	d := NewDecoder()
	got := collect(d)
	feedAll(d, payloads, 0)

	if d.Mode() != DataModeInstant {
		t.Fatalf("mode = %v, want Instant", d.Mode())
	}
	if want := len(payloads) * 4; len(*got) != want {
		t.Fatalf("got %d samples, want %d (4 per notification, none lost)", len(*got), want)
	}
	if d.Resyncs() != 0 || d.Rejected() != 0 {
		t.Errorf("aligned stream needed resyncs=%d rejected=%d", d.Resyncs(), d.Rejected())
	}
	for i, s := range *got {
		if i > 0 && s.ChipMs-(*got)[i-1].ChipMs != 10 {
			t.Fatalf("chip clock step at sample %d is %d ms, want exactly 10 (100 Hz)", i, s.ChipMs-(*got)[i-1].ChipMs)
		}
		for _, a := range []float64{s.Accel.X, s.Accel.Y, s.Accel.Z} {
			if math.Abs(a) > 16 {
				t.Fatalf("acceleration %v g outside +-16 g", a)
			}
		}
	}
	first := (*got)[0]
	if first.Year != 15 || first.Month != 1 {
		t.Errorf("chip clock starts at %d-%d, want the 2015-01 default", first.Year, first.Month)
	}
	if want := float64(first.AccelRaw[0]) * (16.0 / 32768.0); math.Abs(first.Accel.X-want) > 1e-12 {
		t.Errorf("accel scale: %v != %v", first.Accel.X, want)
	}
	// Instant packets never touch the Default-mode reading.
	if d.current.Velocity != (Vector3{}) {
		t.Errorf("instant packets leaked into the reading: %+v", d.current.Velocity)
	}
}

func TestNowDataSplitWrites(t *testing.T) {
	payloads := loadFile(t, "capture-now-data-100hz.hex")
	whole := NewDecoder()
	wholeGot := collect(whole)
	feedAll(whole, payloads, 0)

	for _, chunk := range []int{20, 7, 1} {
		d := NewDecoder()
		got := collect(d)
		feedAll(d, payloads, chunk)
		if len(*got) != len(*wholeGot) {
			t.Fatalf("chunk %d: %d samples, want %d", chunk, len(*got), len(*wholeGot))
		}
		for i := range *got {
			if (*got)[i].ChipMs != (*wholeGot)[i].ChipMs || (*got)[i].AccelRaw != (*wholeGot)[i].AccelRaw {
				t.Fatalf("chunk %d: sample %d differs from the unsplit decode", chunk, i)
			}
		}
	}
}

func TestModeSwitch(t *testing.T) {
	d := NewDecoder()
	got := collect(d)
	defaultSeen := false
	for _, p := range loadFile(t, "capture-mode-switch.hex") {
		if _, ok := d.Feed(p); ok {
			defaultSeen = true
		}
	}
	if !defaultSeen {
		t.Error("Default packets before the switch were not decoded")
	}
	if len(*got) == 0 {
		t.Fatal("Now data packets after the switch were not decoded")
	}
	if d.Mode() != DataModeInstant {
		t.Errorf("ends in %v, want Instant", d.Mode())
	}
	if d.Rejected() != 0 {
		t.Errorf("%d packets rejected across the switch", d.Rejected())
	}
	// The mixed 32+40 byte notification must not poison what follows it.
	for i := 1; i < len(*got); i++ {
		if (*got)[i].ChipMs-(*got)[i-1].ChipMs != 10 {
			t.Fatalf("samples after the switch are not contiguous at %d", i)
		}
	}
}

func TestImplausibleTimeRejected(t *testing.T) {
	// SYNTHETIC: three well-formed 40-byte frames whose time is impossible
	// (month 0); a real capture cannot produce this on demand.
	var stream []byte
	for i := 0; i < 3; i++ {
		pkt := make([]byte, instantPacketLen)
		pkt[0], pkt[1], pkt[2], pkt[3], pkt[4] = syncByte, packetTypeOutput, 15, 0, 1
		stream = append(stream, pkt...)
	}
	d := NewDecoder()
	got := collect(d)
	d.Feed(stream)
	if len(*got) != 0 {
		t.Errorf("impossible calendar time yielded %d samples", len(*got))
	}
	if d.Rejected() == 0 {
		t.Error("implausible packets were not counted")
	}
}

func TestExpectedOutputLenHint(t *testing.T) {
	def := loadFile(t, "capture-default-100hz.hex")
	lone := NewDecoder()
	lone.SetExpectedOutputLen(outputPacketLen)
	if _, ok := lone.Feed(def[0][:outputPacketLen]); !ok {
		t.Error("a lone 32-byte packet should decode with the length hint")
	}

	// A wrong hint corrects itself on a Now data stream.
	instant := loadFile(t, "capture-now-data-100hz.hex")
	wrong := NewDecoder()
	got := collect(wrong)
	wrong.SetExpectedOutputLen(outputPacketLen)
	feedAll(wrong, instant, 0)
	if wrong.Mode() != DataModeInstant || len(*got) < len(instant)*4-4 {
		t.Errorf("wrong hint did not self-correct: mode=%v samples=%d", wrong.Mode(), len(*got))
	}
}
