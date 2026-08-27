package wtvb01

// Framing and command constants confirmed from
// WitBluetooth_BWT901BLE5_0/Python/BWT901BLE5.0_python_sdk/device_model.py,
// with packet lengths corrected against bytes captured from a physical
// WTVB01-BT50 (see docs/protocol.md and testdata/).
const (
	// syncByte is the fixed first byte of every notification packet.
	syncByte byte = 0x55

	// packetTypeOutput is Bytes[1] for the sensor's free-running
	// broadcast packet.
	packetTypeOutput byte = 0x61
	// packetTypeRegister is Bytes[1] for a register read-back response.
	packetTypeRegister byte = 0x71

	// outputPacketLen is the WTVB01-BT50 broadcast packet size: 2 header
	// bytes + 15 int16 values. NOTE: the generic BWT901 SDK hardcodes 20
	// bytes for this packet type; that is wrong for the WTVB01-BT50,
	// which sends 32. Verified over 110 captured packets.
	outputPacketLen = 32

	// registerPacketLen is the register read-back size: 2 header bytes +
	// 2 address bytes + 8 registers * 2 bytes. Matches the SDK and the
	// official manual. Verified over 11 captured packets.
	registerPacketLen = 20

	// registersPerBlock is how many registers one read-back returns.
	registersPerBlock = 8

	// readTriggerReg is the fixed "read register" command register
	// (device_model.py get_readBytes: [0xFF, 0xAA, 0x27, regAddr, 0x00]).
	readTriggerReg byte = 0x27

	// regUnlock / regSave are the config unlock and save registers
	// (device_model.py unlock()/save()).
	regUnlock byte = 0x69
	regSave   byte = 0x00

	unlockValue uint16 = 0xB588
	saveValue   uint16 = 0x0000
)

// WTVB01-BT50 measurement registers.
//
// The register *addresses* below are confirmed by capture: the 0x61
// broadcast packet's first 13 values are byte-identical to the
// registers returned by read-backs of blocks 0x3A and 0x42 (see
// docs/protocol.md §6). The *semantics and scales* follow WitMotion's
// documented WTVB01 register order and still need side-by-side
// confirmation against the official app — only temperature is proven
// so far (reg 0x40 / 100 = 24.4 °C, matching ambient).
const (
	regVelocityX byte = 0x3A
	regVelocityY byte = 0x3B
	regVelocityZ byte = 0x3C

	regAngleX byte = 0x3D
	regAngleY byte = 0x3E
	regAngleZ byte = 0x3F

	// regTemperature is the sensor chip temperature. CONFIRMED against
	// hardware. This is chip temperature, never bearing or motor
	// temperature.
	regTemperature byte = 0x40

	regDisplacementX byte = 0x41
	regDisplacementY byte = 0x42
	regDisplacementZ byte = 0x43

	regFrequencyX byte = 0x44
	regFrequencyY byte = 0x45
	regFrequencyZ byte = 0x46
)

// Register block start addresses used to poll every measurement
// register. Two reads cover 0x3A-0x46.
const (
	blockMeasurementA byte = 0x3A // covers 0x3A-0x41
	blockMeasurementB byte = 0x42 // covers 0x42-0x49
)

// Scale factors. Only temperature is hardware-confirmed; the rest
// follow the documented WTVB01 units and are pending app comparison.
const (
	temperatureScale = 100.0 // raw/100 -> degrees Celsius (CONFIRMED)
	angleScale       = 32768.0
	angleRange       = 180.0 // raw/32768*180 -> degrees
	velocityScale     = 1.0 // raw -> mm/s
	displacementScale = 1.0 // raw -> micrometres
	frequencyScale    = 1.0 // raw -> Hz
)
