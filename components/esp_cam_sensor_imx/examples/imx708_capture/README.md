# IMX708 capture bring-up test

Streams the IMX708 (Pi Camera Module 3 / NoIR 3) in 2×2 binned 2304×1296 RAW10
and logs frame size + average brightness per frame. Pins are preset for the
Waveshare ESP32-P4-WIFI6.

## Build & flash

```bash
$env:IDF_TARGET = "esp32p4"
```

```bash
idf.py build flash monitor
```

(The `$env:` line is PowerShell — set it once per terminal so `idf.py` targets
the P4 instead of a stale `esp32` default.)

## Success milestones, in order

| Log line | Means |
| -------- | ----- |
| `esp_video_init OK` | CSI + ISP + I2C came up |
| `detected IMX708, PID=0x0708` | the driver bound the sensor over I2C (addr 0x1a) |
| `driver=… card=…` | capture video device exists |
| `frame NN: seq=… bytes=… avg=…` with changing seq | **the IMX708 is streaming** |
| `==== IMX708 bring-up PASSED ====` | full path works |

## If it fails

- **`esp_video_init failed`** — SCCB/power. (The `i2c_probe` example already
  confirmed the sensor answers at 0x1a, so this would point at esp_video's CSI
  or LDO init rather than wiring.)
- **binds but no frames / DQBUF times out** — MIPI lane rate / HS-settle, or a
  mode-register issue. The link is 900 Mbps/lane × 2; cross-check `mipi_clk` in
  the driver against the CSI receiver. Watch for the `detected IMX708` line to
  confirm the driver ran.
- **frames arrive but `avg` is 0 or 255** — streaming works; exposure/gain or
  ISP config is off. That's ISP-tuning territory, not a wiring fault — and with
  the NoIR module, off-looking colour is expected until tuning exists.

## Scope

Autofocus (VCM at I2C 0x0c) and PDAF are not driven yet — the lens parks at its
rest position, so the image may be soft. This test is about getting pixels to
flow, not focus or colour.
