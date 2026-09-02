# IMX219 snapshot

Takes one photograph with a Raspberry Pi Camera Module v2 / NoIR v2 (IMX219) on
an ESP32-P4 and sends it down the console UART as a JPEG. No SD card needed.

This is the [`imx708_snapshot`](../imx708_snapshot) example with the autofocus
taken out — the v2 module is fixed-focus, and an I2C scan with one plugged in
answers at `0x10` (sensor), `0x18` (the board's audio codec) and `0x64` (PMIC)
only, with nothing at `0x0c` where the Camera Module 3's VCM sits. In its place
is an auto-exposure trace, which is this sensor's equivalent diagnostic.

## Build and run

```bash
idf.py set-target esp32p4
```

Then, from the repository root, flash and receive the picture in one step:

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx219_snapshot --out shot
```

The JPEG lands in `shot/imx219.jpg`. `tools/capture.py` owns the serial port for
the whole run — flashing, resetting, and reading the framed payload back out —
so close any other monitor first or esptool fails with `Access is denied`.

Board defaults target the Waveshare ESP32-P4-WIFI6: camera SCCB on I2C port 0,
SCL `GPIO8` / SDA `GPIO7`, and no reset or power-down routed (the Pi 15-pin CSI
connector carries neither, and the sensor free-runs on its own oscillator, so
there is no host XCLK either). Re-check `CAM_*` in
[`main/imx219_snapshot_main.c`](main/imx219_snapshot_main.c) for a different
board.

## Reading the output

```
imx219: detected IMX219, PID=0x0219
imx219: set format: MIPI_2lane_24Minput_RAW10_1640x1232_30fps
imx219_snapshot: AE headroom: exposure 4..1759 lines
imx219_snapshot: AE headroom: gain menu 0..41, max 10667 milli
imx219_snapshot:    t (ms) | luma | exp | gain_idx
imx219_snapshot:        50 |   34 | 1322 | 12
imx219_snapshot:        90 |   34 | 1322 | 41
imx219_snapshot:       120 |   42 | 1322 | 41
imx219_snapshot: AE settled: luma 34 -> 42, exposure 1322 lines, gain index 41
```

| What to look for | Means |
| ---------------- | ----- |
| `detected IMX219, PID=0x0219` | SCCB reaches the sensor and it answered |
| **No** `failed to get configuration to initialize ISP controller` | the IMX219 IPA config loaded; AE and AWB are running |
| `luma`/`exp`/`gain_idx` moving and then flattening | AE converged |
| `luma` never moving at all | AE is not running — the IPA config did not load |
| `[OK ] imx219 jpeg 1640x1232 … crc <a>/<a>` from `capture.py` | the frame arrived intact |

There is deliberately **no** `seq` column: esp_video never fills
`v4l2_buffer.sequence` at `VIDIOC_DQBUF`, so it reads 0 however well the sensor
is streaming. Judge liveness from `bytesused` and the frame interval instead.

## Why the first picture is dark, and what actually limits it

The `AE headroom` lines exist so a dark frame can be diagnosed instead of
argued about. On the bench run above, AE drove **gain to index 41 — the top of
the menu, 10.667x** — and then stopped with `luma` at 42 against the config's
target of 100. That is not a bug, and the trace shows why: three separate
numbers box the exposure in.

- **Gain ceiling.** The IMX219's analog gain is `256/(256-code)` with the code
  capped at 232, so the sensor tops out at **10.667x**. The IMX708 reaches 16x.
  That is most of a stop less light in the same scene, and AE reaches the end of
  it much sooner.
- **Exposure ceiling.** Integration time cannot exceed the frame length, so on
  the 1640x1232 mode (VTS 1763) the ceiling is **1759 lines**, about 33 ms.
- **Anti-flicker quantisation.** With `ac_freq: 60` the AE only picks exposures
  that are whole multiples of 1/120 s. At this mode's 18.904 us line time that
  step is **440.8 lines**, so the legal exposures are 441, 882, 1322 and 1763
  lines. The fourth of those is 1763 — four lines *above* the 1759 ceiling, so
  it can never be taken. AE is pinned at 1322 lines (25.0 ms) with a third of a
  stop of exposure it is not allowed to use.

So in dim light this sensor runs out of road, and the run above is what that
looks like. If you need the last third of a stop, the options are to set
`anti_flicker.mode` to off in
[`imx219_default.json`](../../sensors/imx219/cfg/imx219_default.json) — at the
cost of banding under mains lighting — or to give the mode a longer frame by
raising VTS, which lowers the frame rate. Neither is the right default.

## Colour is not calibrated

The CCM in the IMX219 IPA config is the **identity matrix**. The IMX708's tuned
matrix is deliberately not copied across: a colour matrix is a per-sensor,
per-CFA measurement, and a borrowed one would look like tuning while being an
unmeasured guess. Expect flat, slightly green-cast colour until it is measured
against a colour chart. The NoIR variant makes that worse and harder — with no
IR-cut filter, infrared contaminates all three channels, so any calibration has
to be done under the light the camera will actually work in.

The AWB `rg`/`bg` bounds are also deliberately wider than the IMX708's, for the
same reason: those were narrowed around a white point measured on that module,
and reusing them here would fail in the worse direction — if no pixel falls
inside the box, `min_counted` is never reached and AWB simply never updates,
which reads as a colour cast rather than as a broken config.

## Switches in the source

| Macro | Default | What it does |
| ----- | ------- | ------------ |
| `AIM_SECONDS` | 4 | settling time before the keeper frame. Raise it if the trace is still moving when the window closes |
| `IMAGE_OUT_SERIAL` | 1 | send the frame down the console as JPEG |
| `IMAGE_OUT_SD` | 0 | also write a 24-bit BMP to `/sdcard/imx219.bmp`; costs ~8 s |
| `TEST_PATTERN` | 0 | sensor colour bars. Clean bars mean the MIPI/CSI/ISP transport is healthy and a bad picture is optics or tuning |
| `POISON_BUFFERS` | 0 | fill capture buffers with `0xA5` first and count survivors. Proves a frame is live DMA rather than untouched PSRAM — worth having because untouched PSRAM averages mid-scale and is easy to mistake for a real exposure |
