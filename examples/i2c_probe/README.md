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
  the problem is specific to the camera: cable orientation (flip the FFC),
  seating, or the camera's power rail.
- **Scan finds nothing at all** — I2C pins wrong, or the connector's 3V3 isn't
  reaching the module. Recheck the cable and pin defines.

## Note

I2C success validates the control path but **not** the MIPI-CSI data lanes —
those are separate wires and are only exercised once you actually stream (the
`imx219_capture` example). But an I2C pass retires the most common failures and
means the sensor is present and powered.
