package ble

import "tinygo.org/x/bluetooth"

var (
	// ServiceUUID / NotifyCharacteristic / WriteCharacteristic are
	// confirmed from device_model.py (see docs/protocol.md §2).
	ServiceUUID          bluetooth.UUID
	NotifyCharacteristic bluetooth.UUID
	WriteCharacteristic  bluetooth.UUID
)

func init() {
	var err error
	ServiceUUID, err = bluetooth.ParseUUID("0000ffe5-0000-1000-8000-00805f9a34fb")
	if err != nil {
		panic(err)
	}
	NotifyCharacteristic, err = bluetooth.ParseUUID("0000ffe4-0000-1000-8000-00805f9a34fb")
	if err != nil {
		panic(err)
	}
	WriteCharacteristic, err = bluetooth.ParseUUID("0000ffe9-0000-1000-8000-00805f9a34fb")
	if err != nil {
		panic(err)
	}
}
