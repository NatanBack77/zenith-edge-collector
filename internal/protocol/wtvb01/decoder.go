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
// packet length depends on the type, because the WTVB01-BT50's 0x61
// broadcast is 32 bytes, not the 20 the generic SDK assumes.
//
// A Decoder keeps the latest value of every field across packets, the
// same way device_model.py's deviceData dict accumulates: a 0x61
// broadcast carries all measurement registers at once, while a 0x71
// read-back refreshes only the eight registers in its block.
type Decoder struct {
	buf     []byte
	current SensorReading
}

// NewDecoder returns a ready-to-use Decoder.
func NewDecoder() *Decoder {
	return &Decoder{buf: make([]byte, 0, outputPacketLen)}
}

// packetLenFor returns the total packet length for a type byte, or 0 if
// the type is not recognised.
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

// Feed processes a raw BLE notification payload, which may contain any
// number of packets and may split a packet across calls. It returns the
// latest reading and true if at least one packet was decoded.
func (d *Decoder) Feed(data []byte) (SensorReading, bool) {
	updated := false

	for _, b := range data {
		d.buf = append(d.buf, b)

		if len(d.buf) == 1 && d.buf[0] != syncByte {
			d.buf = d.buf[:0]
			continue
		}
		if len(d.buf) == 2 && packetLenFor(d.buf[1]) == 0 {
			// Drop the stale sync byte and re-test the remainder: the
			// byte we just read may itself start a real packet.
			d.buf = d.buf[:0]
			if b == syncByte {
				d.buf = append(d.buf, b)
			}
			continue
		}

		if len(d.buf) < 2 {
			continue
		}
		if want := packetLenFor(d.buf[1]); len(d.buf) == want {
			if d.processPacket(d.buf) {
				updated = true
			}
			d.buf = d.buf[:0]
		}
	}

	return d.current, updated
}

// processPacket decodes one complete packet into d.current, returning
// true if any field was updated.
func (d *Decoder) processPacket(pkt []byte) bool {
	var ok bool
	switch pkt[1] {
	case packetTypeOutput:
		ok = d.decodeOutput(pkt)
	case packetTypeRegister:
		ok = d.decodeRegisterBlock(pkt)
	}
	if ok {
		d.current.Timestamp = time.Now()
	}
	return ok
}

// decodeOutput decodes the 0x61 broadcast packet.
//
// Capture shows its first 13 int16 values are byte-identical to
// registers 0x3A..0x46 read back via 0x71, so it is decoded through the
// same register dispatch. Value 13 (a constant zero) is ignored. Value
// 14 has no documented register address, but is a candidate for the
// app's "Power Percent(%)" field, so it is decoded into Device.PowerRaw
// (see the field's doc comment for why it is raw, not a percentage).
func (d *Decoder) decodeOutput(pkt []byte) bool {
	values := pkt[2:]
	updated := false
	for i := range (len(values) / 2) {
		reg := regVelocityX + byte(i)
		if reg > regFrequencyZ {
			break
		}
		if d.applyRegister(reg, signInt16LE(values[i*2], values[i*2+1])) {
			updated = true
		}
	}
	if len(values) >= 30 {
		d.current.Device.PowerRaw = signInt16LE(values[28], values[29])
		updated = true
	}
	return updated
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
