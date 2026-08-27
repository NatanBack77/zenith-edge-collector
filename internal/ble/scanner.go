package ble

import (
	"strings"
	"sync"
	"time"

	"tinygo.org/x/bluetooth"
)

// FoundDevice describes a discovered advertisement.
type FoundDevice struct {
	Address string
	Name    string
	RSSI    int16
	// NameMatch reports whether the local name contains "WT".
	NameMatch bool
	// ServiceMatch reports whether the advertisement includes the
	// WitMotion service UUID ffe5. This is the reliable identifier:
	// unlike the name, it does not depend on the device broadcasting a
	// local name.
	ServiceMatch bool

	// ServiceUUIDs and ManufacturerIDs are captured for diagnostics,
	// to help identify a sensor that advertises no local name.
	ServiceUUIDs    []string
	ManufacturerIDs []uint16
}

// Matched reports whether the device looks like a WitMotion sensor by
// either signal.
func (d FoundDevice) Matched() bool { return d.NameMatch || d.ServiceMatch }

// ScanOptions controls scan behaviour.
type ScanOptions struct {
	// Duration is how long to scan for.
	Duration time.Duration
	// All returns every BLE device seen instead of only WitMotion ones.
	// Useful when the sensor advertises neither a local name nor its
	// service UUID in the advertisement packet.
	All bool
}

// IsWitMotionName reports whether a local name matches the WitMotion
// WT-series filter used by the official SDK (test.py: `"WT" in d.name`).
func IsWitMotionName(name string) bool {
	return strings.Contains(strings.ToUpper(name), "WT")
}

// Scan runs a BLE scan and returns every unique device discovered,
// filtered per opts.
func Scan(adapter *bluetooth.Adapter, opts ScanOptions) ([]FoundDevice, error) {
	if err := adapter.Enable(); err != nil {
		return nil, err
	}

	var mu sync.Mutex
	seen := make(map[string]FoundDevice)
	done := make(chan struct{})

	go func() {
		time.Sleep(opts.Duration)
		_ = adapter.StopScan()
		close(done)
	}()

	err := adapter.Scan(func(a *bluetooth.Adapter, result bluetooth.ScanResult) {
		found := FoundDevice{
			Address:      result.Address.String(),
			Name:         result.LocalName(),
			RSSI:         result.RSSI,
			ServiceMatch: result.HasServiceUUID(ServiceUUID),
		}
		found.NameMatch = IsWitMotionName(found.Name)

		if !found.Matched() && !opts.All {
			return
		}

		for _, u := range result.ServiceUUIDs() {
			found.ServiceUUIDs = append(found.ServiceUUIDs, u.String())
		}
		for _, m := range result.ManufacturerData() {
			found.ManufacturerIDs = append(found.ManufacturerIDs, m.CompanyID)
		}

		mu.Lock()
		defer mu.Unlock()
		// Merge with any earlier advertisement from the same device: a
		// later packet may carry a local name or service UUID that an
		// earlier one omitted.
		if prev, ok := seen[found.Address]; ok {
			if found.Name == "" {
				found.Name = prev.Name
				found.NameMatch = prev.NameMatch
			}
			found.ServiceMatch = found.ServiceMatch || prev.ServiceMatch
			if len(found.ServiceUUIDs) == 0 {
				found.ServiceUUIDs = prev.ServiceUUIDs
			}
			if len(found.ManufacturerIDs) == 0 {
				found.ManufacturerIDs = prev.ManufacturerIDs
			}
		}
		seen[found.Address] = found
	})
	if err != nil {
		return nil, err
	}

	<-done

	mu.Lock()
	defer mu.Unlock()
	devices := make([]FoundDevice, 0, len(seen))
	for _, d := range seen {
		devices = append(devices, d)
	}
	return devices, nil
}
