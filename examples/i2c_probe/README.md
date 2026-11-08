# Camera I2C probe

The fastest possible "is anything alive?" test — talks to the camera over **I2C
only**. No MIPI, no ISP, no `esp_video`, no sensor driver. Runs with **any** Pi
camera module you have plugged in (IMX219, IMX477, or IMX708).

Use this the moment a board + camera are connected, before any streaming work.

## Build & flash

```bash
cd examples/i2c_probe
idf.py set-target esp32p4
idf.py build flash monitor
```

## What it does

1. Scans the whole I2C bus (0x08–0x77) and lists every device that ACKs.
2. Tries a chip-ID read at each known camera address:
   - `0x10` reg `0x0000` → `0x0219` = IMX219 (Pi Cam v2)
   - `0x1a` reg `0x0016` → `0x0708` = IMX708 (Pi Cam v3)
   - `0x1a` reg `0x0000` → `0x0477` = IMX477 (Pi HQ Cam)

## Reading the result

- **`==== FOUND … ✓ ====`** then **`CAMERA I2C LINK OK`** — cable, power, pins,
  address, and bus all work. This is the big early win.
- **Scan lists other devices (codec/touch) but no camera** — the *bus* is fine;
  the problem is specific to the camera: cable pin mapping/orientation (verify against the schematic with power disconnected),
  seating, or the camera's power rail.
- **Scan finds nothing at all** — I2C pins wrong, or the connector's 3V3 isn't
  reaching the module. Recheck the cable and pin defines.

## Note

I2C success validates the control path but **not** the MIPI-CSI data lanes —
those are separate wires and are only exercised once you actually stream (the
`imx219_capture` example). But an I2C pass retires the most common failures and
means the sensor is present and powered.

## Guition JC-ESP32P4-M3-DEV configuration

This workspace targets measured P4 revision v1.3 with 16 MB flash. The tracked
`sdkconfig.defaults` selects pre-v3 silicon and minimum revision v1.0. The
Guition schematic connects J3 pin 13 (SCL) through R30 to GPIO8 and pin 14
(SDA) through R32 to GPIO7, sharing the codec bus. See workspace
`PROJECT_STATUS.md` for schematic and board-specific source evidence.

Build a fresh isolated configuration from these defaults:

```bash
idf.py -B build-guition -D SDKCONFIG=build-guition/sdkconfig reconfigure build
```

The probe is prepared but has not been run on hardware. When flashing is
explicitly approved, use the same build/configuration and `/dev/ttyUSB0`.
Do not bypass revision checks. A pass requires IMX219 ID `0x0219` at `0x10`;
other bus devices alone do not establish camera connectivity. Power off before
reseating the camera, and verify cable pin mapping rather than reversing it
under power.
