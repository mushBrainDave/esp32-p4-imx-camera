# IMX708 video over USB

Records a few seconds of 1080p H.264 from the IMX708 and sends it down the same
USB cable that carries the log. No SD card. `tools/capture.py` picks the clip up
and writes an `.mp4` you can double-click. Pins preset for the Waveshare
ESP32-P4-WIFI6.

This is the moving-picture counterpart to
[`imx708_snapshot`](../imx708_snapshot/), and shares its serial transport
(`components/imx_serial_img`, shipped inside each of the two examples that
use it), its IPA tuning config and its autofocus setup.
Read that example's README first if you have not — the ISP tuning, autofocus and
"two things that will bite you" sections apply here unchanged.

## Run it

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_video --out clip
```

That flashes, resets the board, and listens. The run is:

| Phase | Time | What is happening |
| ----- | ---- | ----------------- |
| Aim | 6 s | AE, AWB and autofocus converge. Point the camera now. |
| Record | 8 s | Frames go straight into the H.264 encoder. |
| Transfer | ~20 s | The whole clip goes down the console at 2 Mbaud. |

`capture.py` stops as soon as the board says it is done, so the default
`--seconds` is a ceiling rather than a wait. Output lands in `clip/`:

- `imx708.mp4` — the playable file
- `imx708.h264` — the raw Annex-B stream it was muxed from
- `log.txt` — the full run log, including the per-frame table

To rebuild the `.mp4` from a stream you already have:

```bash
python tools/mp4.py clip/imx708.h264 clip/imx708.mp4 --fps 28
```

## Checking the result with ffmpeg

`tools/mp4.py` deliberately has no dependencies, so the capture pipeline never
needs ffmpeg. But ffmpeg is a much better *judge* of the output than a player
is — it will complain about a container this muxer got wrong instead of quietly
showing you something. On this machine it lives in WSL, so paths need the
`/mnt/c/...` form:

```bash
wsl ffprobe -hide_banner /mnt/c/Users/mushbrain/source/repos/imx708/clip/imx708.mp4
```

Expect `Video: h264 (Constrained Baseline), yuv420p, 1920x1072, 28 fps` and a
duration matching the log. A container fault shows up here as a wrong duration,
a wrong frame count, or `moov atom not found`.

To pull frames back out as stills — which is the only way to measure sharpness,
exposure or colour, since nothing on the Windows side can decode H.264:

```bash
wsl ffmpeg -i /mnt/c/.../clip/imx708.mp4 -vsync 0 /mnt/c/.../clip/frame_%04d.png
```

The measurement techniques that apply to those frames are the ones in the
snapshot example's README — and note the same caution: compare frames within one
run, never across runs, because scene and exposure move the numbers more than
anything you are trying to measure.

## Why H.264, when stills are JPEG

The snapshot example sends JPEG, and that is the right answer *for one frame*:
an H.264 I-frame is roughly a JPEG with extra ceremony — SPS, PPS, NAL framing —
and no size win to show for it.

For a sequence the conclusion flips, because temporal prediction is the entire
point. A 1080p JPEG off this sensor is about 250 KB, so 28 fps of them is
~7 MB/s. The same scene as H.264 at 4 Mbit/s is 500 KB/s: fourteen times
smaller, and nearly all of that is frames that barely changed from the one
before.

The P4 has a hardware H.264 encoder. It is **not** listed in `soc_caps.h` —
support comes from the separate `espressif/esp_h264` managed component.

## Why the clip is recorded first and sent afterwards

The console runs at 2 Mbaud, about 200 KB/s or 1.6 Mbit/s. That is below any
1080p bitrate worth recording, so streaming live would mean choosing the
encoder's bitrate to fit the cable — making picture quality a property of the
UART.

Recording into PSRAM decouples the two completely. The encoder runs at whatever
bitrate suits the picture (4 Mbit/s by default, well over what the link could
carry live) and the link only decides how long you wait afterwards. The cost is
that clip length is bounded by `REC_BUF_BYTES` — 6 MB, about 12 s at the default
bitrate — rather than by patience. The run reports which limit stopped it.

If you want longer clips, raise `REC_BUF_BYTES` (there is 32 MB of PSRAM, minus
~9 MB of capture buffers) or lower `VIDEO_BITRATE`.

## YUV420 straight from the ISP

The one piece of pipeline work this example does that the snapshot does not is
in a single `VIDIOC_S_FMT` call:

```c
fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
```

The P4's H.264 core does not take RGB. It wants YUV420 in a packed layout
Espressif call `O_UYY_E_VYY` — odd lines `u y y u y y…`, even lines
`v y y v y y…` — and the ISP's YUV420 output *is* that layout, because the two
blocks were designed to hand off to each other. So there is no colour conversion
anywhere in this example. Converting 1920×1080 from RGB565 in software 28 times
a second would not come close to fitting in the frame budget on this CPU.

It also shrinks the frame from 4.1 MB to 3.1 MB, which is why three capture
buffers fit comfortably where the snapshot uses two.

## 1072 lines, not 1080

H.264 codes in 16×16 macroblocks. 1920 divides by 16; 1080 does not — covering
it needs 68 macroblock rows, i.e. a 1088-line picture. The SPS can signal the
extra 8 lines as a crop, and the encoder does emit that, but it would still be
reading a 1088-line picture out of a buffer the ISP only filled 1080 lines of,
and what the input DMA does with those last 8 lines is not documented.

`ENCODE_16_ALIGNED` (on by default) sidesteps it: the encoder is told 1072 — 67
macroblock rows exactly — and reads only the first 1072 lines, which is a prefix
of what the ISP wrote. No padding, no over-read, nothing to crop. The cost is
8 lines off the bottom, 0.7% of the height.

Set it to `0` to encode the full 1080 and find out what the hardware really does.

## The frame table

Before sending the clip the board prints one line per frame:

```
VIDTABLE frames=224 bytes=1983204
VIDFRAME i=0 t=0 off=0 len=48120 type=idr
VIDFRAME i=1 t=36 off=48120 len=3204 type=p
...
```

The host does not need this to find frame boundaries — `tools/mp4.py` scans for
Annex-B start codes and groups NALs into access units itself. What the table
carries that the bitstream does not is **when each frame was actually
captured**. The `.mp4` is timed from those timestamps, so if the pipeline
dropped frames the clip still plays at real speed with a visible hitch, rather
than playing fast and hiding the drop.

If the table is missing or its frame count disagrees with the bitstream,
`capture.py` says so and falls back to the nominal frame rate.

## Reading the log

A real run, from `clip/log.txt`:

```
I imx708_video: H.264 1920x1072 @ 28 fps, 4000000 bit/s, GOP 28, QP 20-45
I imx708_video: first frame: mean luma 118
W imx708_video: RECORDING for up to 8 s - hold still
I imx708_video: recorded 217 frames (8 IDR, 0 failed) in 7966 ms - 27.2 fps, stopped on: time
I imx708_video: encode 36290 us mean, 36761 us worst (35714 us per frame available)
I imx708_video: clip 3883868 bytes = 3900 kbit/s actual (asked for 4000)
```

> Earlier revisions of this section carried an invented sample log — `encode
> 9204 us mean` at 27.9 fps — that matches no run in the tree. It cost real time
> later, when `imx708_wifi_video` measured 36 ms and went looking for a
> regression that had never existed. Both preserved runs here, `clip/log.txt`
> and `clip2/log.txt`, say 36.3 ms and 27.2 fps. Paste logs; do not write them.

- **mean luma** is a cheap "is there a picture in here at all" check that needs
  no decoder on either end. Near 0 or near 255 means black or blown out. A
  plausible value that is *identical on every run* means nothing is being DMA'd
  in and the clip is stale PSRAM.
- **encode time is ~36 ms and does not move.** That is the whole 35.7 ms frame
  budget for 1920×1072, which is why runs land at 27.2 fps rather than 28 — the
  encoder, not the camera, paces the loop. It is a fixed-throughput pipeline:
  `imx708_wifi_video` swept rate control from 1 to 8 Mbit/s and moved the output
  from 4.8 KB to 34.7 KB per frame with the time flat throughout. Lowering
  `VIDEO_BITRATE` will *not* make it faster; only fewer or smaller frames will.
- **`failed` above zero** is the encoder rejecting frames. `ESP_H264_ERR_OVERFLOW`
  means `ENC_OUT_BYTES` is too small for an IDR of this scene; raise it.
- **actual bitrate far under the target** is normal for a still scene — rate
  control spends what the picture needs, not what it is allowed.

## What you can change

All at the top of `main/imx708_video_main.c`:

| Define | Default | Notes |
| ------ | ------- | ----- |
| `VIDEO_SECONDS` | 8 | Recording length, once settled |
| `VIDEO_BITRATE` | 4000000 | Bits per second the rate control aims at |
| `VIDEO_FPS` | 28 | The sensor's only mode. Sets the bit budget per frame and the SPS timing; it does not make frames arrive faster |
| `VIDEO_GOP` | 28 | One IDR per second. Shorter spends bitrate re-sending the scene; longer makes seeking coarser |
| `VIDEO_QP_MIN/MAX` | 20 / 45 | Quality bounds for rate control. 51 is the codec maximum |
| `REC_BUF_BYTES` | 6 MB | Clip buffer, and the real limit on length |
| `ENC_OUT_BYTES` | 512 KB | Per-frame encoder output. Must fit the largest IDR |
| `AIM_SECONDS` | 6 | Settling before recording. Too short and you record the autofocus hunting |
| `ENCODE_16_ALIGNED` | 1 | See "1072 lines, not 1080" above |

## If the clip is wrong

- **`the ISP would not give us YUV420`** — the CSI/ISP device refused the
  format. Check `CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE` is on; the H.264
  core takes nothing else.
- **`esp_h264_enc_hw_new failed`** — a configuration problem, not a missing
  chip. The core tops out at 1920×2032 and accepts only the packed YUV420
  layout above.
- **CRC mismatch on the payload** — the transport, not the encoder. Everything
  in the snapshot README's "two things that will bite you here" applies, and
  applies harder to a multi-megabyte payload than to one frame.
- **The `.mp4` will not play but the `.h264` is there** — muxing failed and
  `capture.py` printed why. The stream itself is fine; try
  `python tools/mp4.py` on it directly for the full error.
- **Video plays but the colours are wrong** — that is ISP tuning, shared with
  the snapshot example. The CCM has not been calibrated against a colour chart,
  and the module is NoIR, so infrared contaminates all three channels.
