# IMX219 video

Records ~8 s of H.264 from a Raspberry Pi Camera Module v2 / NoIR v2 (IMX219)
on an ESP32-P4, buffers the clip in PSRAM, then sends it down the console UART.
`tools/capture.py` muxes it into an `.mp4` you can double-click. No SD card.

Measured on the bench: **1632x1232 at 28.1 fps**, 225 frames, zero encoder
failures.

## Build and run

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx219_video --out clip
```

The clip lands in `clip/imx219.mp4`, timed from the capture timestamps rather
than a nominal frame rate. Budget about 40 s: 4 s settling, 8 s recording, and
~14 s to shift 2.6 MB down a 2 Mbaud link.

## What limits the frame rate

The sensor mode is 30 fps and the encoder is what you actually get:

```
recorded 225 frames (8 IDR, 0 failed) in 8006 ms - 28.1 fps, stopped on: time
encode 35302 us mean, 35792 us worst (33333 us per frame available)
clip 2570750 bytes = 2568 kbit/s actual (asked for 4000)
```

35.3 ms of encode against a 33.3 ms frame interval — the H.264 core is 6% short
of real time at this resolution, so one frame in sixteen has nowhere to go.
1000/35.3 = 28.3 predicted against 28.1 measured. That is the same ceiling
`imx708_video` hits at 1920x1072 (2.06 Mpx here vs 2.01 Mpx there), which is
what you would expect from a per-pixel bound of roughly 57 Mpx/s.

The bitrate came in at 2.6 Mbit/s against the 4 Mbit/s asked for. Rate control
spends what the scene needs, and this one is dim and low-detail.

## The macroblock alignment problem — and why the obvious fix is wrong

H.264 codes in 16x16 macroblocks. The IMX219's default mode is **1640x1232**:
the height is fine (1232 = 77 macroblock rows) but **1640 is 102.5 macroblocks**.

`imx708_video` has the mirror-image problem — 1920 divides, 1080 does not — and
solves it by simply telling the encoder a smaller height, 1072. **That trick
does not transfer to width.** A shorter frame is a *prefix* of the buffer the
ISP filled, so the bytes the encoder reads are exactly the bytes it wants. A
narrower frame is not a prefix of anything: the encoder derives its line stride
from the width it is given, so encoding a 1640-wide buffer as 1632 starts each
line 8 pixels further left than the one before and the picture shears
diagonally.

So the crop has to happen where the stride is decided. Two candidates, one of
which does not exist on this board:

| Where | Verdict |
| ----- | ------- |
| **ISP crop**, `VIDIOC_S_SELECTION` / `V4L2_SEL_TGT_CROP` | **Unavailable here.** esp_video gates `ESP_VIDEO_ISP_DEVICE_CROP` on `CONFIG_ESP32P4_REV_MIN_FULL >= 300` — ESP32-P4 revision v3.0. This board's chip is **v1.3**, so the video device has no `set_selection` and the ioctl returns `ESP_ERR_NOT_SUPPORTED` (0x106). Tried first, on hardware. |
| **Sensor readout window** | Works, costs nothing at runtime. |

Hence the driver's **1632x1232 mode** (index 2): mode 0 with 8 columns trimmed,
4 from each side, at the sensor's X window. Same VTS, so identical frame rate
and exposure limits; the X start is a multiple of 4 so 2x2 binning keeps the
RGGB phase. `sdkconfig.defaults` selects it with

```
CONFIG_CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT=2
```

The cost is 0.5% of the horizontal field of view. The check in `app_main` is a
guard, not a fix: if a non-macroblock-aligned frame arrives, the mode index is
wrong and it refuses to record rather than producing a sheared clip.

## A measurement trap worth knowing

The aim window prints a mean-luma trace, and the first version of it lied.
Sampling one line in 64 lands only ever on lines of a **single parity**, and the
YUV420 layout alternates U-carrying and V-carrying lines. That produced a
perfectly regular 45/60 alternation every two frames — which looks exactly like
an auto-exposure loop hunting and never settling.

It was not. The decoded clip's luma was steady to within **0.05 counts** across
40 frames (`ffprobe -f lavfi -i "movie=…,signalstats"`). Sampling line *pairs*
so both parities are always covered collapsed the trace from 81 oscillating
lines to three:

```
   t (ms) | luma
       56 | 39
      122 | 56
      221 | 58
```

which is what AE converging actually looks like. If you adapt `mean_luma` for
another layout, keep the pairs.

## Switches in the source

| Macro | Default | What it does |
| ----- | ------- | ------------ |
| `VIDEO_SECONDS` | 8 | recording length; the real cap is `REC_BUF_BYTES` and the run says which one stopped it |
| `VIDEO_FPS` | 30 | what the encoder is told to expect. Does not make frames arrive faster; affects the bit budget and the SPS timing |
| `VIDEO_BITRATE` | 4000000 | rate-control target |
| `VIDEO_GOP` | 30 | one IDR per second — the only places playback can start or recover |
| `REC_BUF_BYTES` | 6 MB | ~12 s at the target bitrate |
| `AIM_SECONDS` | 4 | settling before recording. Shorter than `imx708_video`'s 6 s, which also waited out an autofocus search |

There is no autofocus and no `.cam_motor` config: the v2 module is fixed-focus,
with nothing at I2C `0x0c`.
