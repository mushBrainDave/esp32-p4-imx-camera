# IMX219 capture bring-up test

Minimal end-to-end smoke test: initialise `esp_video`, bind the IMX219 via
auto-detect, open `/dev/video0`, stream 30 frames, and log size + average
brightness per frame.

## Before you build

Open [`main/imx219_capture_main.c`](main/imx219_capture_main.c) and set the
`EXAMPLE_SCCB_*`, `EXAMPLE_CAM_RESET_PIN`, and `EXAMPLE_CAM_PWDN_PIN` values for
your board. The defaults target the ESP32-P4-Function-EV-Board camera header —
**verify them against your schematic.**

## Build & flash

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

## Reading the output — success milestones in order

| Log line | Means |
| -------- | ----- |
| `esp_video_init OK` | CSI + ISP + I2C came up; SCCB pins/power are right |
| `driver=... card=...` | the capture video device exists |
| `frame NN: seq=… bytes=… avg=…` with non-zero `bytes` arriving at the mode's frame interval | **the IMX219 is actually streaming** |
| `==== IMX219 bring-up PASSED ====` | full path works |

**Ignore `seq`.** esp_video never fills `v4l2_buffer.sequence` at `VIDIOC_DQBUF`
- it keeps its own sequence internally and does not copy it out - so `seq` reads
0 on every frame no matter how well the sensor is streaming. Judge liveness from
`bytes` and from the interval between frames instead: at 1640x1232 RGB565 expect
`bytes=4040960` about every 33 ms.

`W esp_video_init: failed to get configuration to initialize ISP controller` is
expected here and is not a failure: this example registers no esp_ipa JSON, so
there is no AE or AWB and exposure/gain stay at the mode defaults.

## If it fails

- **`esp_video_init failed`** — almost always SCCB (I2C) pins/address, or the
  sensor reset/power lines. The IMX219 is at I2C `0x10`. Confirm the sensor gets
  power and its XCLK.
- **init OK but no frames / DQBUF times out** — MIPI lane rate / HS-settle
  mismatch, or a bad mode register write. Cross-check `mipi_clk` in the driver
  against the P4 CSI receiver, and watch the boot log for the
  `detected IMX219, PID=0x0219` line from the driver.
- **Frames arrive but `avg` is 0 or 255** — sensor streams but exposure/gain or
  ISP config is off. That's ISP-tuning territory (roadmap item 2), not a wiring
  problem — the hard part already works.
