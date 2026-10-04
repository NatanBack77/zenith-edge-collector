// Command zenith-edge is the MVP CLI for the Zenith Edge Collector.
// Only two subcommands exist at this stage: `scan` and `test`.
package main

import (
	"encoding/hex"
	"flag"
	"fmt"
	"os"
	"sort"
	"strings"
	"sync/atomic"
	"time"

	"tinygo.org/x/bluetooth"

	"github.com/NatanBack77/zenith-edge-collector/internal/ble"
	"github.com/NatanBack77/zenith-edge-collector/internal/protocol/wtvb01"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(1)
	}

	switch os.Args[1] {
	case "scan":
		runScan(os.Args[2:])
	case "test":
		runTest(os.Args[2:])
	default:
		usage()
		os.Exit(1)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage:")
	fmt.Fprintln(os.Stderr, "  zenith-edge scan [--all] [--verbose] [--duration 10s]")
	fmt.Fprintln(os.Stderr, "  zenith-edge test --sensor <mac-address> [--raw]")
}

func runScan(args []string) {
	fs := flag.NewFlagSet("scan", flag.ExitOnError)
	duration := fs.Duration("duration", 10*time.Second, "how long to scan for")
	all := fs.Bool("all", false, "list every BLE device, not just WitMotion ones")
	verbose := fs.Bool("verbose", false, "show advertised service UUIDs and manufacturer IDs")
	fs.Parse(args)

	if *all {
		fmt.Printf("Scanning for %s (all BLE devices)...\n", *duration)
	} else {
		fmt.Printf("Scanning for %s (WitMotion devices: \"WT\" name or service %s)...\n",
			*duration, ble.ServiceUUID.String())
	}

	devices, err := ble.Scan(bluetooth.DefaultAdapter, ble.ScanOptions{
		Duration: *duration,
		All:      *all,
	})
	if err != nil {
		fmt.Fprintln(os.Stderr, "scan failed:", err)
		os.Exit(1)
	}

	if len(devices) == 0 {
		fmt.Println("No devices found.")
		if !*all {
			fmt.Println("Tip: the sensor may advertise neither a name nor its service UUID.")
			fmt.Println("     Retry with: zenith-edge scan --all")
		}
		return
	}

	// Strongest signal first — the sensor on your bench is usually the
	// closest device.
	sort.Slice(devices, func(i, j int) bool { return devices[i].RSSI > devices[j].RSSI })

	for _, d := range devices {
		name := d.Name
		if name == "" {
			name = "(no name)"
		}

		var why string
		switch {
		case d.ServiceMatch:
			why = "  <- WitMotion service ffe5"
		case d.NameMatch:
			why = "  <- WitMotion name"
		}

		fmt.Printf("%-20s  %-24s  RSSI %4d%s\n", d.Address, name, d.RSSI, why)

		if *verbose {
			if len(d.ServiceUUIDs) > 0 {
				fmt.Printf("%22sservices: %s\n", "", strings.Join(d.ServiceUUIDs, ", "))
			}
			if len(d.ManufacturerIDs) > 0 {
				ids := make([]string, 0, len(d.ManufacturerIDs))
				for _, id := range d.ManufacturerIDs {
					ids = append(ids, fmt.Sprintf("0x%04X", id))
				}
				fmt.Printf("%22smanufacturer: %s\n", "", strings.Join(ids, ", "))
			}
		}
	}
}

func runTest(args []string) {
	fs := flag.NewFlagSet("test", flag.ExitOnError)
	sensor := fs.String("sensor", "", "MAC address of the sensor to connect to")
	raw := fs.Bool("raw", false, "print raw notification bytes as hex instead of decoded values")
	noPoll := fs.Bool("no-poll", false, "never send register read commands (they are skipped automatically in Now data mode)")
	fs.Parse(args)

	if *sensor == "" {
		fmt.Fprintln(os.Stderr, "error: --sensor <mac-address> is required")
		os.Exit(1)
	}

	fmt.Printf("Connecting to %s...\n", *sensor)
	client, err := ble.Connect(bluetooth.DefaultAdapter, *sensor)
	if err != nil {
		fmt.Fprintln(os.Stderr, "connect failed:", err)
		os.Exit(1)
	}
	defer client.Disconnect()

	decoder := wtvb01.NewDecoder()

	// In Now data mode the sensor sends ~100 samples/s; printing them all
	// would flood the terminal, so show every 10th (10 lines/s).
	samples := 0
	decoder.OnInstant = func(s wtvb01.InstantSample) {
		samples++
		if samples%10 == 1 {
			printInstant(s)
		}
	}

	// Reading registers while the sensor streams Now data disturbs the
	// sample stream (measured: 1 contiguous 256-sample window in 30 s at
	// 5 reads/s, versus unbroken windows without reads), so the polling
	// goroutine below stands down when the mode is Now data.
	var instant atomic.Bool

	err = client.Subscribe(func(data []byte) {
		if *raw {
			fmt.Printf("%s  %s\n", time.Now().Format("15:04:05.000"), hex.EncodeToString(data))
			return
		}
		reading, ok := decoder.Feed(data)
		instant.Store(decoder.Mode() == wtvb01.DataModeInstant)
		if !ok {
			return
		}
		printReading(reading)
	})
	if err != nil {
		fmt.Fprintln(os.Stderr, "subscribe failed:", err)
		os.Exit(1)
	}

	// Poll the measurement registers (mirrors device_model.py's
	// sendDataTh). The 0x61 broadcast already carries every field, so
	// this mainly serves to cross-check the decoder against register
	// read-backs.
	go func() {
		for {
			if *noPoll || instant.Load() {
				time.Sleep(500 * time.Millisecond)
				continue
			}
			for _, cmd := range wtvb01.MeasurementBlockCommands() {
				client.Send(cmd)
				time.Sleep(100 * time.Millisecond)
			}
		}
	}()

	fmt.Println("Streaming decoded readings (Ctrl+C to stop)...")
	select {}
}

func printReading(r wtvb01.SensorReading) {
	fmt.Printf(
		"[%s] angle(%.2f,%.2f,%.2f) vel(%.3f,%.3f,%.3f)mm/s disp(%.1f,%.1f,%.1f)um freq(%.1f,%.1f,%.1f)Hz temp=%.1fC bat=%.2fV(%.0f%%) alarm=%d\n",
		r.Timestamp.Format("15:04:05.000"),
		r.Angle.X, r.Angle.Y, r.Angle.Z,
		r.Velocity.X, r.Velocity.Y, r.Velocity.Z,
		r.Displacement.X, r.Displacement.Y, r.Displacement.Z,
		r.Frequency.X, r.Frequency.Y, r.Frequency.Z,
		r.Device.Temperature,
		r.Device.BatteryVolts, r.Device.BatteryPercent, r.Device.AlarmStatus,
	)
}

// printInstant shows one Now data sample: chip clock, acceleration in g and
// signed instantaneous velocity.
func printInstant(s wtvb01.InstantSample) {
	fmt.Printf(
		"[%s] NOW DATA chip=%dms acc(%.3f,%.3f,%.3f)g vel(%.0f,%.0f,%.0f)mm/s\n",
		time.Now().Format("15:04:05.000"), s.ChipMs,
		s.Accel.X, s.Accel.Y, s.Accel.Z,
		s.Velocity.X, s.Velocity.Y, s.Velocity.Z,
	)
}
