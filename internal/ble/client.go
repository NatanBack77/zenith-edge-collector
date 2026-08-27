package ble

import (
	"fmt"

	"tinygo.org/x/bluetooth"
)

// Client wraps a connected WTVB01-BT50 device, mirroring the
// service/characteristic discovery and notify subscription flow from
// device_model.py's openDevice() (see docs/protocol.md §1).
type Client struct {
	adapter *bluetooth.Adapter
	device  bluetooth.Device
	notify  bluetooth.DeviceCharacteristic
	write   bluetooth.DeviceCharacteristic
}

// Connect discovers the device by MAC address, connects, and resolves
// the notify/write characteristics used by the WTVB01-BT50 protocol.
func Connect(adapter *bluetooth.Adapter, address string) (*Client, error) {
	if err := adapter.Enable(); err != nil {
		return nil, fmt.Errorf("enable adapter: %w", err)
	}

	mac, err := bluetooth.ParseMAC(address)
	if err != nil {
		return nil, fmt.Errorf("parse address %q: %w", address, err)
	}

	device, err := adapter.Connect(bluetooth.Address{MACAddress: bluetooth.MACAddress{MAC: mac}}, bluetooth.ConnectionParams{})
	if err != nil {
		return nil, fmt.Errorf("connect to %s: %w", address, err)
	}

	services, err := device.DiscoverServices([]bluetooth.UUID{ServiceUUID})
	if err != nil {
		return nil, fmt.Errorf("discover services: %w", err)
	}
	if len(services) == 0 {
		return nil, fmt.Errorf("service %s not found on %s", ServiceUUID.String(), address)
	}

	chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{NotifyCharacteristic, WriteCharacteristic})
	if err != nil {
		return nil, fmt.Errorf("discover characteristics: %w", err)
	}

	c := &Client{adapter: adapter, device: device}
	for _, ch := range chars {
		switch ch.UUID() {
		case NotifyCharacteristic:
			c.notify = ch
		case WriteCharacteristic:
			c.write = ch
		}
	}
	if c.notify.UUID() != NotifyCharacteristic {
		return nil, fmt.Errorf("notify characteristic %s not found on %s", NotifyCharacteristic.String(), address)
	}
	if c.write.UUID() != WriteCharacteristic {
		return nil, fmt.Errorf("write characteristic %s not found on %s", WriteCharacteristic.String(), address)
	}

	return c, nil
}

// Subscribe enables notifications on the notify characteristic,
// invoking callback for every raw BLE payload received.
func (c *Client) Subscribe(callback func(data []byte)) error {
	return c.notify.EnableNotifications(callback)
}

// Send writes a raw command to the write characteristic (used for
// register read/write commands, see wtvb01.ReadRegisterCommand etc.).
func (c *Client) Send(data []byte) error {
	_, err := c.write.WriteWithoutResponse(data)
	return err
}

// Disconnect tears down the BLE connection.
func (c *Client) Disconnect() error {
	return c.device.Disconnect()
}
