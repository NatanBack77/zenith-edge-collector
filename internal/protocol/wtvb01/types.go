package wtvb01

import "time"

// Vector3 is a generic X/Y/Z triple.
type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// DeviceInfo holds fields about the sensor module itself, not the
// machine it is mounted on.
type DeviceInfo struct {
	// Temperature is the sensor module's own temperature in degrees
	// Celsius, from register 0x40 ("Product temperature" in the
	// manual).
	//
	// This is NOT the machine's temperature and NOT a calibrated
	// ambient probe, which is why it is never named
	// bearing_temperature or motor_temperature. Mounted on a machine
	// it reads between the machine surface and the surrounding air,
	// dominated by conduction through the mount, and it lags.
	Temperature float64 `json:"temperature"`

	// PowerRaw is the battery register 0x64 ("BatPer" in the manual), also
	// the last int16 (bytes 30-31) of the Default 0x61 packet, in
	// centivolts. CONFIRMED on hardware: the packet value matched the 0x64
	// read-back (437..441 vs 439). The name is kept for the existing JSON
	// schema, which the ESP32 firmware also publishes.
	PowerRaw float64 `json:"power_raw"`

	// BatteryVolts is PowerRaw / 100, and BatteryPercent comes from the
	// official app's piecewise-linear table (BatteryPercentFromVolts).
	BatteryVolts   float64 `json:"battery_v"`
	BatteryPercent float64 `json:"battery_pct"`

	// AlarmStatus is the sensor's embedded alarm flags (manual §5.2.8):
	// bits 0-2 displacement alarm X/Y/Z, bits 3-5 frequency alarm X/Y/Z.
	// 0 means no alarm. It was 0 in all 4868 Default packets captured.
	AlarmStatus int `json:"alarm"`
}

// DataMode is which 0x61 format the sensor is currently sending.
type DataMode uint8

const (
	// DataModeUnknown means no 0x61 packet has been decoded yet.
	DataModeUnknown DataMode = iota
	// DataModeDefault is the 32-byte packet: amplitudes (always positive),
	// frequency, temperature, battery.
	DataModeDefault
	// DataModeInstant ("Now data") is the 40-byte packet: chip time,
	// acceleration, gyro and signed instantaneous velocity/angle/
	// displacement. Selected with register 0x96 (1 = Now data, 0 = Default);
	// that register is NOT in the V260410 manual, it was found by capturing
	// the official app. See docs/protocol.md §9.
	DataModeInstant
)

// InstantSample is one decoded 40-byte "Now data" packet. The values are
// signed instantaneous values, not amplitudes.
type InstantSample struct {
	// Year is two digits (15 == 2015) and ChipMs folds day/hour/minute/
	// second/millisecond into one monotonic counter. Without time
	// calibration the chip clock starts at 2015-01-01 00:00:00 on power-up,
	// so only differences between samples are meaningful.
	Year, Month, Day, Hour, Minute, Second uint8
	Millisecond                            uint16
	ChipMs                                 uint32

	AccelRaw [3]int16 // counts; multiply by AccelScaleG for g
	Accel    Vector3  // g
	Gyro     Vector3  // deg/s
	Velocity Vector3  // mm/s, signed
	Angle    Vector3  // degrees, signed

	Displacement Vector3 // micrometres, signed
}

// SensorReading is the normalized, decoded output of a WTVB01-BT50
// sensor at a point in time.
//
// The ESP32 firmware in firmware/esp32-zenith-node publishes the same
// schema, so both collectors are interchangeable downstream.
type SensorReading struct {
	Timestamp time.Time `json:"timestamp"`

	// Velocity is vibration velocity in mm/s. The general-purpose
	// machine-health indicator, and what ISO 10816/20816 limits target.
	Velocity Vector3 `json:"velocity"`

	// Displacement is vibration displacement in micrometres. Most
	// sensitive to low-frequency faults.
	Displacement Vector3 `json:"displacement"`

	// Angle is angular vibration amplitude in degrees: how much the
	// surface rocks or twists through a vibration cycle. It is not the
	// sensor's mounting orientation.
	Angle Vector3 `json:"angle"`

	// Frequency is vibration frequency in Hz per axis. The diagnostic
	// indicator: amplitude says something is wrong, frequency says what.
	Frequency Vector3 `json:"frequency"`

	Device DeviceInfo `json:"device"`
}
