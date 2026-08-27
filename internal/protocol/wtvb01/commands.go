package wtvb01

// ReadRegisterCommand builds the 5-byte command that triggers a
// register read-back (device_model.py get_readBytes):
// [0xFF, 0xAA, 0x27, regAddr, 0x00].
func ReadRegisterCommand(regAddr byte) []byte {
	return []byte{0xFF, 0xAA, readTriggerReg, regAddr, 0x00}
}

// WriteRegisterCommand builds the 5-byte command that writes a value
// to a register (device_model.py get_writeBytes):
// [0xFF, 0xAA, regAddr, valueLow, valueHigh].
func WriteRegisterCommand(regAddr byte, value uint16) []byte {
	return []byte{0xFF, 0xAA, regAddr, byte(value & 0xFF), byte(value >> 8)}
}

// UnlockCommand builds the command that unlocks config registers for
// writing (device_model.py unlock()).
func UnlockCommand() []byte {
	return WriteRegisterCommand(regUnlock, unlockValue)
}

// SaveCommand builds the command that persists written config
// (device_model.py save()).
func SaveCommand() []byte {
	return WriteRegisterCommand(regSave, saveValue)
}

// MeasurementBlockCommands returns the register-read commands that
// together cover every measurement register (0x3A-0x46).
//
// Polling these is optional: the sensor's 0x61 broadcast already
// carries all of them. They exist to read values on demand and to
// verify the decoder against register read-backs.
func MeasurementBlockCommands() [][]byte {
	return [][]byte{
		ReadRegisterCommand(blockMeasurementA),
		ReadRegisterCommand(blockMeasurementB),
	}
}
