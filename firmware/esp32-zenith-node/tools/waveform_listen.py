#!/usr/bin/env python3
"""Listens to the ESP32 node's binary waveform frames and prints a summary.

Frame layout (little-endian), see PublishWaveformIfReady in src/main.cpp:
  48-byte header "ZWF1" | n:u16 | axes:u8 | reserved:u8 | seq:u32 | chip_start_ms:u32 |
  chip_end_ms:u32 | gaps:u32 | dropped:u32 | unsent:u32 | fs_hz:f32 | g_per_count:f32 |
  uptime_ms:u32 | published_at_ms:u32
  then int16 ax[n], ay[n], az[n]  (raw counts; g = count * g_per_count)

Usage:
  python3 waveform_listen.py --host 127.0.0.1 --user zenith-reader --password ... [--count 3]
Needs paho-mqtt; numpy is optional (adds RMS and the FFT peak per axis).
"""
import argparse, struct, sys, time

try:
    import numpy as np
except ImportError:
    np = None
import paho.mqtt.client as mqtt

HEADER = struct.Struct("<4sHBBIIIIIIffII")
assert HEADER.size == 48


def decode(payload: bytes):
    if len(payload) < HEADER.size or payload[:4] != b"ZWF1":
        raise ValueError("not a ZWF1 frame")
    (magic, n, axes, _res, seq, t0, t1, gaps, dropped, unsent, fs, scale, up, pub) = HEADER.unpack_from(payload)
    need = HEADER.size + axes * n * 2
    if len(payload) != need:
        raise ValueError(f"size {len(payload)} != expected {need}")
    body = payload[HEADER.size:]
    if np is not None:
        data = np.frombuffer(body, dtype="<i2").reshape(axes, n).astype(np.float64) * scale
    else:
        vals = struct.unpack(f"<{axes * n}h", body)
        data = [[v * scale for v in vals[a * n:(a + 1) * n]] for a in range(axes)]
    return dict(n=n, axes=axes, seq=seq, chip_start_ms=t0, chip_end_ms=t1, gaps=gaps,
                dropped=dropped, unsent=unsent, fs_hz=fs, g_per_count=scale, uptime_ms=up, data=data)


def describe(f, topic):
    print(f"{topic} seq={f['seq']} n={f['n']} fs={f['fs_hz']:.2f}Hz span={f['chip_end_ms']-f['chip_start_ms']}ms "
          f"gaps={f['gaps']} dropped={f['dropped']} unsent={f['unsent']}")
    if np is None:
        return
    d, fs, n = f["data"], f["fs_hz"], f["n"]
    for name, axis in zip("XYZ", d):
        ac = axis - axis.mean()
        spec = np.abs(np.fft.rfft(ac * np.hanning(n))) / n
        k = int(np.argmax(spec[1:]) + 1)
        print(f"   {name}: mean={axis.mean():+.3f}g rms(ac)={np.sqrt((ac**2).mean()):.4f}g "
              f"peak={fs*k/n:.2f}Hz (bin {k}, resolution {fs/n:.2f}Hz)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--user"); ap.add_argument("--password")
    ap.add_argument("--topic", default="zenith/waveform/#")
    ap.add_argument("--count", type=int, default=0, help="stop after N frames (0 = forever)")
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args()
    got = []
    state = {"connected": False}
    def on_connect(c, *args):
        state["connected"] = True
        c.subscribe(a.topic)
    def on_disconnect(c, *args):
        state["connected"] = False
    def on_message(c, _u, m):
        try:
            f = decode(m.payload)
        except ValueError as e:
            print("bad frame:", e); return
        describe(f, m.topic); got.append(f)
        if a.count and len(got) >= a.count:
            c.disconnect()
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        c = mqtt.Client()
    if a.user:
        c.username_pw_set(a.user, a.password)
    c.on_connect, c.on_message, c.on_disconnect = on_connect, on_message, on_disconnect
    c.connect(a.host, a.port, 30); c.loop_start()
    end = time.time() + a.timeout
    t_start = time.time()
    while time.time() < end:
        # is_connected() is False until the CONNACK arrives, so only treat a
        # disconnect as the end once we have actually been connected.
        if not state["connected"] and time.time() - t_start > 5 and not got:
            break
        if a.count and len(got) >= a.count:
            break
        time.sleep(0.2)
    c.loop_stop()
    print(f"{len(got)} frame(s) received")
    sys.exit(0 if got else 1)


if __name__ == "__main__":
    main()
