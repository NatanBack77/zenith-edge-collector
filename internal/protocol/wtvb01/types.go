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
