package wtvb01

import (
	"encoding/binary"
	"time"
)

// Decoder accumulates raw BLE notification bytes into packets and
// decodes them into SensorReading updates.
//
// It follows the byte-resync state machine from device_model.py
// (onDataReceived): drop bytes until 0x55 lands at index 0 and a valid
// type byte at index 1, then collect a whole packet. Unlike that SDK,
// packet length is not fixed: the register read-back is 20 bytes, and the
// 0x61 broadcast is either 32 bytes (Default) or 40 bytes ("Now data").
// Both 0x61 formats use the same type byte, so the length is detected from
// the stream: with 42 bytes buffered, whichever of offset 32 or offset 40
// holds the next packet header (55 61 or 55 71) is the packet size, and
// each later packet is checked against the header that follows it, so a
// Default <-> Now data switch in the middle of a stream re-detects itself.
// This mirrors the ESP32 firmware's decoder (firmware/esp32-zenith-node/
// src/wtvb01.cpp) and is tested against the same captured bytes.
//
// A Decoder keeps the latest value of every Default-mode field across
// packets, the same way device_model.py's deviceData dict accumulates: a
// 0x61 broadcast carries all measurement registers at once, while a 0x71
// read-back refreshes only the eight registers in its block. "Now data"
// packets never touch the SensorReading (they are signed instantaneous
// values, not amplitudes): they go to the OnInstant handler instead.
type Decoder struct {
	buf       []byte
	outputLen int // 0 until the 0x61 length is detected, then 32 or 40
	mode      DataMode
	current   SensorReading
	last      InstantSample

	// OnInstant, if set, is called once per decoded Now data packet, in
	// stream order. One BLE notification carries several packets (4 seen
	// on hardware), so the returned reading alone would lose most of them.
	OnInstant func(InstantSample)

	resyncs  uint64
	rejected uint64
}

// NewDecoder returns a ready-to-use Decoder.
func NewDecoder() *Decoder {
	return &Decoder{buf: make([]byte, 0, 4*instantPacketLen)}
}

// packetLenFor returns the total packet length for a type byte when it is
// unambiguous, or 0 if the type is not recognised. For 0x61 it returns the
// Default length (32); the Decoder decides between 32 and 40 from the
// stream, never from this function.
func packetLenFor(packetType byte) int {
	switch packetType {
	case packetTypeOutput:
		return outputPacketLen
	case packetTypeRegister:
		return registerPacketLen
	default:
		return 0
	}
}

// Mode reports the format of the last 0x61 packet decoded.
func (d *Decoder) Mode() DataMode { return d.mode }

// Resyncs and Rejected are diagnostics: bytes skipped to find a header, and
// Now data packets dropped for an impossible calendar time.
func (d *Decoder) Resyncs() uint64  { return d.resyncs }
func (d *Decoder) Rejected() uint64 { return d.rejected }

// LastInstant returns the most recent Now data packet (valid once Mode()
// has been DataModeInstant). Prefer OnInstant.
func (d *Decoder) LastInstant() InstantSample { return d.last }

// SetExpectedOutputLen skips 0x61 length detection when the mode is known
// by other means (for example right after writing register 0x96, or to
// decode a lone packet). Pass 32 (Default), 40 (Now data) or 0 to go back to
// detection. A wrong hint corrects itself: a packet whose following header
// is invalid unlocks the length again.
func (d *Decoder) SetExpectedOutputLen(n int) {
	if n == outputPacketLen || n == instantPacketLen {
		d.outputLen = n
	} else {
		d.outputLen = 0
	}
}

// isHeader reports whether p starts a known packet header (55 61 / 55 71).
func isHeader(p []byte) bool {
	return len(p) >= 2 && p[0] == syncByte && (p[1] == packetTypeOutput || p[1] == packetTypeRegister)
}

// Feed processes a raw BLE notification payload, which may contain any
// number of packets and may split a packet across calls. It returns the
// latest reading and true if the Default-mode reading changed (a Default
// 0x61 or a 0x71 read-back). Now data packets return false here and are
// delivered through OnInstant.
func (d *Decoder) Feed(data []byte) (SensorReading, bool) {
	d.buf = append(d.buf, data...)
	updated := d.drain()
	return d.current, updated
}

// drain extracts every complete packet currently buffered.
func (d *Decoder) drain() bool {
	updated := false
	const detectBytes = instantPacketLen + 2

	for {
		if i := indexByte(d.buf, syncByte); i < 0 {
			d.buf = d.buf[:0]
			return updated
		} else if i > 0 {
			d.consume(i)
			d.resyncs++
		}
		if len(d.buf) < 2 {
			return updated
		}

		typ := d.buf[1]
		want := 0
		switch typ {
		case packetTypeRegister:
			want = registerPacketLen
		case packetTypeOutput:
			if d.outputLen == 0 {
				if len(d.buf) < detectBytes {
					return updated // not enough bytes to decide yet
				}
				isDefault := isHeader(d.buf[outputPacketLen:])
				isInstant := isHeader(d.buf[instantPacketLen:])
				if isDefault == isInstant {
					// Neither or both: not a trustworthy packet start.
					d.consume(1)
					d.resyncs++
					continue
				}
				if isDefault {
					d.outputLen = outputPacketLen
				} else {
					d.outputLen = instantPacketLen
				}
			}
			want = d.outputLen
		default:
			d.consume(1)
			d.resyncs++
			continue
		}

		if len(d.buf) < want {
			return updated // wait for the rest of the packet
		}

		// Verify against the following header when it is already buffered.
		if len(d.buf) >= want+2 && !isHeader(d.buf[want:]) {
			if typ == packetTypeOutput {
				d.outputLen = 0 // wrong length (mode switched?): re-detect here
				d.resyncs++
				continue
			}
			d.consume(1)
			d.resyncs++
			continue
		}

		if d.processPacket(d.buf[:want]) {
			updated = true
		}
		d.consume(want)
	}
}

func (d *Decoder) consume(n int) {
	k := copy(d.buf, d.buf[n:])
	d.buf = d.buf[:k]
}

func indexByte(b []byte, c byte) int {
	for i, v := range b {
		if v == c {
			return i
		}
	}
	return -1
}

// processPacket decodes one complete packet into d.current, returning
// true if the Default-mode reading was updated.
func (d *Decoder) processPacket(pkt []byte) bool {
	var ok bool
	switch pkt[1] {
	case packetTypeOutput:
		if len(pkt) == instantPacketLen {
			d.decodeInstant(pkt)
			return false
		}
		ok = d.decodeOutput(pkt)
	case packetTypeRegister:
		ok = d.decodeRegisterBlock(pkt)
	}
	if ok {
		d.current.Timestamp = time.Now()
	}
	return ok
}

// decodeOutput decodes the 32-byte Default 0x61 broadcast.
//
// Capture shows its first 13 int16 values are byte-identical to registers
// 0x3A..0x46 read back via 0x71, so it is decoded through the same register
// dispatch. Value 13 is the alarm status (manual §5.2.8) and value 14 is the
// battery register 0x64 (confirmed against a read-back on hardware).
func (d *Decoder) decodeOutput(pkt []byte) bool {
	values := pkt[2:]
	for i := range len(values) / 2 {
		reg := regVelocityX + byte(i)
		if reg > regFrequencyZ {
			break
		}
		d.applyRegister(reg, signInt16LE(values[i*2], values[i*2+1]))
	}
	d.current.Device.AlarmStatus = int(signInt16LE(values[26], values[27]))
	d.applyRegister(regBattery, signInt16LE(values[28], values[29]))
	d.mode = DataModeDefault
	return true
}

// decodeInstant decodes the 40-byte "Now data" packet (manual §5.2.2, byte
// layout confirmed against the official app's display on hardware):
//
//	55 61 | year month day hour minute second | ms(2) |
//	AX AY AZ | GX GY GZ | NVX NVY NVZ | NADX NADY NADZ | NDX NDY NDZ
//
// Everything after the time bytes is little-endian int16.
func (d *Decoder) decodeInstant(pkt []byte) {
	s := InstantSample{
		Year: pkt[2], Month: pkt[3], Day: pkt[4], Hour: pkt[5], Minute: pkt[6], Second: pkt[7],
		Millisecond: binary.LittleEndian.Uint16(pkt[8:10]),
	}
	// A false match on 55 61 inside payload bytes would decode to an
	// impossible calendar time almost every time; reject those.
	if s.Month < 1 || s.Month > 12 || s.Day < 1 || s.Day > 31 || s.Hour > 23 ||
		s.Minute > 59 || s.Second > 59 || s.Millisecond > 999 {
		d.rejected++
		return
	}
	ms := ((uint64(s.Day)*24+uint64(s.Hour))*60+uint64(s.Minute))*60000 + uint64(s.Second)*1000 + uint64(s.Millisecond)
	s.ChipMs = uint32(ms)

	v := pkt[10:]
	i16 := func(off int) float64 { return signInt16LE(v[off], v[off+1]) }
	for k := 0; k < 3; k++ {
		s.AccelRaw[k] = int16(binary.LittleEndian.Uint16(v[2*k:]))
	}
	s.Accel = Vector3{float64(s.AccelRaw[0]) * AccelScaleG, float64(s.AccelRaw[1]) * AccelScaleG, float64(s.AccelRaw[2]) * AccelScaleG}
	s.Gyro = Vector3{i16(6) / angleScale * gyroRange, i16(8) / angleScale * gyroRange, i16(10) / angleScale * gyroRange}
	s.Velocity = Vector3{i16(12) * velocityScale, i16(14) * velocityScale, i16(16) * velocityScale}
	s.Angle = Vector3{i16(18) / angleScale * angleRange, i16(20) / angleScale * angleRange, i16(22) / angleScale * angleRange}
	s.Displacement = Vector3{i16(24) * displacementScale, i16(26) * displacementScale, i16(28) * displacementScale}

	d.last = s
	d.mode = DataModeInstant
	if d.OnInstant != nil {
		d.OnInstant(s)
	}
}

// decodeRegisterBlock decodes a 0x71 read-back: pkt[2:4] is the
// little-endian start register address, pkt[4:20] is eight registers.
func (d *Decoder) decodeRegisterBlock(pkt []byte) bool {
	startReg := pkt[2]
	data := pkt[4:]
	updated := false
	for i := range registersPerBlock {
		if d.applyRegister(startReg+byte(i), signInt16LE(data[i*2], data[i*2+1])) {
			updated = true
		}
	}
	return updated
}

// BatteryPercentFromVolts converts a battery voltage to a percentage with
// the official Android app 4.0.9's table (single-cell range up to 5.5 V; a
// separate table above that), linearly interpolated and clamped to 0..100.
func BatteryPercentFromVolts(volts float64) float64 {
	cellV := []float64{3.4, 3.5, 3.68, 3.7, 3.73, 3.77, 3.79, 3.82, 3.87, 3.93, 3.96, 3.99}
	cellPct := []float64{0, 5, 10, 15, 20, 30, 40, 50, 60, 75, 90, 100}
	packV := []float64{6.5, 6.8, 7.35, 7.75, 8.5, 8.8}
	packPct := []float64{0, 10, 30, 60, 90, 100}
	if volts > 5.5 {
		return interp(volts, packV, packPct)
	}
	return interp(volts, cellV, cellPct)
}

func interp(v float64, xs, ys []float64) float64 {
	if v < xs[0] {
		return ys[0]
	}
	if v > xs[len(xs)-1] {
		return ys[len(ys)-1]
	}
	for i := 0; i+1 < len(xs); i++ {
		if v <= xs[i+1] {
			return ys[i] + (v-xs[i])/(xs[i+1]-xs[i])*(ys[i+1]-ys[i])
		}
	}
	return ys[len(ys)-1]
}

// applyRegister writes one register's raw value into the reading,
// applying its scale. It returns false for registers we do not model.
func (d *Decoder) applyRegister(reg byte, raw float64) bool {
	switch reg {
	case regVelocityX:
		d.current.Velocity.X = raw * velocityScale
	case regVelocityY:
		d.current.Velocity.Y = raw * velocityScale
	case regVelocityZ:
		d.current.Velocity.Z = raw * velocityScale

	case regAngleX:
		d.current.Angle.X = raw / angleScale * angleRange
	case regAngleY:
		d.current.Angle.Y = raw / angleScale * angleRange
	case regAngleZ:
		d.current.Angle.Z = raw / angleScale * angleRange

	case regTemperature:
		d.current.Device.Temperature = raw / temperatureScale

	case regBattery:
		d.current.Device.PowerRaw = raw
		d.current.Device.BatteryVolts = raw / battCentiV
		d.current.Device.BatteryPercent = BatteryPercentFromVolts(raw / battCentiV)

	case regDisplacementX:
		d.current.Displacement.X = raw * displacementScale
	case regDisplacementY:
		d.current.Displacement.Y = raw * displacementScale
	case regDisplacementZ:
		d.current.Displacement.Z = raw * displacementScale

	case regFrequencyX:
		d.current.Frequency.X = raw * frequencyScale
	case regFrequencyY:
		d.current.Frequency.Y = raw * frequencyScale
	case regFrequencyZ:
		d.current.Frequency.Z = raw * frequencyScale

	default:
		return false
	}
	return true
}

// signInt16LE reads a little-endian signed int16 from two bytes
// (low, high), matching device_model.py's getSignInt16(high<<8|low).
func signInt16LE(low, high byte) float64 {
	return float64(int16(binary.LittleEndian.Uint16([]byte{low, high})))
}
