# What the sensor actually measures

The WTVB01-BT50 reports five things. Four of them describe vibration and
are easy to confuse, because all four move together when a machine
shakes. They answer different questions, and each one catches a
different kind of fault.

## The four vibration indicators

A vibrating machine surface moves back and forth. Describe that motion
three ways and you get three of the indicators:

| Indicator | Answers | Unit | Axes |
|---|---|---|---|
| **Displacement** | How far does it move? | µm | X, Y, Z |
| **Velocity** | How fast does it move? | mm/s | X, Y, Z |
| **Frequency** | How often does it move? | Hz | X, Y, Z |
| **Angular amplitude** | How much does it *tilt* as it moves? | degrees | X, Y, Z |

### Displacement — amplitude of travel

The peak distance the surface travels during a vibration cycle. A shaft
wobbling 200 µm off-centre reports 200 µm regardless of how fast it
spins.

Displacement dominates at **low frequencies**. At 5 Hz a large physical
movement produces low velocity, so a displacement reading catches it
while a velocity reading barely registers. This is the indicator for
unbalance, misalignment, looseness, bent shafts, and structural sway —
slow, large-travel problems.

### Velocity — the general-purpose health number

How fast the surface moves. This is the number most condition-monitoring
standards are written against (ISO 10816 / ISO 20816 set machine
vibration limits in mm/s RMS), because velocity is roughly flat across
the mid frequency band where most mechanical faults live.

If you only ever track one number for overall machine health, track
velocity. Rising velocity on a previously stable machine is the classic
"something is going wrong" signal.

### Frequency — where the energy is, which tells you the cause

How many cycles per second. This is the diagnostic indicator: amplitude
tells you *that* something is wrong, frequency tells you *what*.

Faults show up at characteristic multiples of shaft speed. If a motor
turns at 1800 rpm (30 Hz):

- Energy at **30 Hz** (1× shaft speed) → unbalance
- Energy at **60 Hz** (2×) → misalignment
- Energy at **high, non-integer multiples** → bearing defects
- Energy at **blade or tooth count × speed** → blade/gear problems

Two machines can show the same 5 mm/s velocity and need completely
different repairs. The frequency is what separates them.

### Angular amplitude — rocking, not travelling

How much the sensor *tilts* through the vibration cycle, in degrees. The
manual calls this "angular vibration amplitude"; our code calls it
`angle`.

This is **not** the sensor's mounting orientation. A sensor lying flat
and perfectly still reports ≈0°, and so does a sensor bolted to a
vertical wall. What it measures is rotational oscillation: the surface
rocking or twisting rather than moving in a straight line. Useful for
detecting looseness, a soft foot, or torsional vibration — cases where
part of the machine is pivoting about a point instead of translating.

### How they relate

For a single sinusoidal vibration, the three linear indicators are not
independent — they are linked through frequency:

```
velocity     ≈ 2π × frequency × displacement
```

So a reading of 100 µm at 50 Hz gives roughly 31 mm/s, while the same
100 µm at 2 Hz gives only 1.3 mm/s. This is why displacement and
velocity disagree about which machine is "worse", and why both are
reported:

- **Low frequency** → displacement is the sensitive indicator
- **Mid frequency** → velocity is the sensitive indicator
- **High frequency** → acceleration would be, which this sensor does not
  expose in the vibration block

Three axes matter because vibration has direction. Radial (X/Y) energy
points at unbalance and bearing wear; axial (Z, along the shaft) energy
points at misalignment and thrust-bearing trouble.

## Temperature — read the label carefully

The manual's register table names register `0x40` **"Product
temperature"**: the temperature of the sensor module itself.

It is **not** the machine's temperature, and not a calibrated ambient
probe. Mounted on a hot machine, the module reads somewhere between the
machine surface and the surrounding air — dominated by conduction
through the mount, and lagging behind real changes.

This is why the data model calls it `device.temperature` and never
`bearing_temperature` or `motor_temperature`. Naming it after the
machine would invite someone downstream to alarm on it as if it were a
bearing probe, which it is not.

Observed on hardware: the sensor resting on a desk reported 24.4–25.1 °C
in a room at about the same temperature — expected, since with no heat
source the module equilibrates with the air.

What it is genuinely good for: detecting that the sensor itself is
cooking, correcting for temperature drift in the vibration readings, and
catching gross thermal events. For real bearing temperature, use a probe
mounted on the bearing.

## In practice

A healthy baseline followed by change is what matters — absolute numbers
mean little without knowing what the machine normally does.

1. Baseline every indicator on a known-good machine.
2. Watch **velocity** for overall health; it is what the ISO limits
   target.
3. When velocity rises, read **frequency** to identify the cause.
4. Cross-check **displacement** for slow, large-movement faults that
   velocity underweights.
5. Treat **temperature** as a signal about the sensor, not the machine.

## Sources

- [WTVB01-BT50 instruction manual (RobotShop mirror)](https://cdn.robotshop.com/rbm/f83835f4-5e29-4ee0-9cc2-e49300031503/b/bc40f091-5d65-4712-969d-707ac88c1ca4/8d1ba329_wtvb01-bt50-manual.pdf)
- [WIT WTVB01-BT50 instruction manual (ManualsLib)](https://www.manualslib.com/manual/3151193/Wit-Wtvb01-Bt50.html)
- [WitMotion vibration sensor product page](https://www.wit-motion.com/Vibration.html)
- [WTVB01-BT50 product page](https://witmotion-sensor.com/products/wtvb01-bt50-bluetooth-50m-wireless-multi-connected-vibration-sensor)
