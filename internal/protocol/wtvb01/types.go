package wtvb01

import "time"

// Vector3 is a generic X/Y/Z triple.
type Vector3 struct {
	X float64
	Y float64
	Z float64
}

// DeviceInfo holds fields about the sensor chip itself, not the
// machine it is mounted on.
type DeviceInfo struct {
	// Temperature is the sensor chip's internal temperature in °C.
	// NEVER bearing/motor temperature — this is not a machine reading.
	Temperature float64
}

// SensorReading is the normalized, decoded output of a WTVB01-BT50
// sensor at a point in time.
type SensorReading struct {
	Timestamp time.Time

	// Angle is roll/pitch/yaw in degrees, from the 0x61 output packet.
	Angle Vector3

	// Velocity is vibration velocity in mm/s, from register read-back.
	Velocity Vector3
	// Displacement is vibration displacement in um, from register read-back.
	Displacement Vector3
	// Frequency is dominant vibration frequency in Hz, from register read-back.
	Frequency Vector3

	Device DeviceInfo
}
