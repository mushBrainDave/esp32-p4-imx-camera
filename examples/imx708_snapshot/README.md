# IMX708 snapshot to SD

Grabs one IMX708 frame and saves it to the microSD card as a 24-bit BMP
(`/sdcard/imx708.bmp`) you can open on any PC. Pins preset for the Waveshare
ESP32-P4-WIFI6.

## Before you run

- Insert a **FAT32-formatted microSD card** into the board's TF slot.
- Have the camera aimed at something with detail and light.

## Build & flash

```bash
$env:IDF_TARGET = "esp32p4"
```

```bash
idf.py build flash monitor
```

After boot it mounts the SD card, starts streaming, prints
`aim the camera — settling for 6 s...`, then grabs a frame and writes the BMP.
The settle window is what gives auto-exposure, white balance and autofocus time
to converge.
When you see `==== done ... ====`, power off, pop the card into your PC, and
open `imx708.bmp`.

## What you're validating

- **The BMP is a real, changing photo of the scene** → the whole pipeline is
  genuinely live end-to-end. This is the visual proof.
- Board SD pins used: CLK 43, CMD 44, D0 39, D1 40, D2 41, D3 42 (4-bit), with
  SD power via on-chip LDO channel 4.

## ISP tuning (IPA)

The sensor's ISP tuning lives in
`components/esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json`, keyed
`"IMX708"` to match `IMX708_SENSOR_NAME`. It turns on six IPA units: `agc`
(auto-exposure), `awb` (white balance), `acc` (saturation + colour-correction
matrix), `aen` (gamma, sharpen, contrast), `adn` (denoise), `ian` (metering
weights) and `af` (autofocus).

Registration happens in this example's **project** `CMakeLists.txt`, between
`include(project.cmake)` and `project()`. It cannot go in the component's
CMakeLists: esp_ipa reads `ESP_IPA_JSON_CONFIG_FILE_PATH` while its own
CMakeLists is processed, which is already done by the time a component's runs.
Get this wrong and there is no error — just
`failed to get configuration to initialize ISP controller` in the log, and no
AE or AWB.

### Things you may need to change

- **Gain is a menu control, not a number.** esp_video's AE reads gain via
  `VIDIOC_QUERYMENU` and sets an *index* into `imx708_total_gain_val_map`. If a
  future sensor here declares `ESP_CAM_SENSOR_GAIN` as
  `ESP_CAM_SENSOR_PARAM_TYPE_NUMBER`, the query fails and the AE silently drives
  exposure only — which looks exactly like a too-dark, too-grainy picture.
- **`agc.anti_flicker.ac_freq` is set to 60.** If your mains is 60 Hz, change it
  or you will see banding under artificial light. This is the single most likely
  thing to need adjusting.
- **The CCM is a seed, not a calibration.** Real values come from shooting a
  colour chart under known illuminants and solving for the matrix. The rows sum
  to 1.0 so neutrals stay neutral, and it is deliberately gentler than
  Espressif's OV5647 matrix because a **NoIR** module has no IR-cut filter —
  infrared contaminates all three channels and aggressive saturation amplifies
  that error.
- **The `af` scan band needs calibrating.** `af.min_pos` / `af.max_pos` (380 and
  940) come from libcamera's Camera Module 3 tuning data, not from your module.
  See *Autofocus* below.

### A harmless build warning

The build prints several `excess elements in array initializer` warnings from
the generated `esp_video_ipa_config.c`. esp_ipa 2.3.0's generator always emits a
5x5 `awb.subwin_weight`, but on ESP-IDF v5.4 `ISP_AWB_WINDOW_X_NUM` is undefined
so the struct member is `float[0][0]`. The initialisers are discarded at compile
time and `enable_sub_win` is false, so nothing reads it. It is a generator/IDF
version mismatch, not a problem with this config.

## Autofocus

The lens is moved by a **DW9807 voice-coil motor at I2C 0x0c** — a separate chip
from the IMX708, on the same bus. Driver:
`components/esp_cam_sensor_imx/motors/dw9807/`.

Focus is closed-loop contrast detection, and it takes four things that each fail
differently if missing:

| Piece | Where | If it's missing |
| --- | --- | --- |
| `CONFIG_CAM_MOTOR_DW9807` | `sdkconfig.defaults` | No driver; nothing can move the lens |
| `cam_motor` entry in `esp_video_init_config_t` | `imx708_snapshot_main.c` | Motor auto-detect array never walked |
| `CONFIG_ESP_IPA_AF_ALGORITHM` | `sdkconfig.defaults` | Nothing decides *where* to focus |
| `CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR` | `sdkconfig.defaults` | AF result computed, then discarded |

The ISP scores "definition" (edge energy) inside the three `af.windows`, and the
algorithm hill-climbs the lens position that maximises it — a coarse pass of
`l1_scan_points_num` stops, then a fine pass of `l2_scan_points_num` around the
winner. That takes time, which is why `AIM_SECONDS` is 6.

The log tells you whether it worked:

```
dw9807: detected DW9807 VCM at 0x0c, lens parked at code 450 (range 0..1023)
imx708_snapshot: lens parked at code 450
imx708_snapshot: focus: 450 -> 683, centre sharpness 2841
```

A `focus: 450 -> 450` with a warning means the lens never moved.

### Calibrating the scan band

The DAC is 10-bit, but only the middle of that travel maps to real focus
distances — roughly code 420 at infinity to 920 at closest macro on a Camera
Module 3. Outside that the lens is against a mechanical stop. Those figures come
from libcamera's IMX708 tuning file, and VCM assemblies vary unit to unit, so
verify them once on your own module:

1. Set `CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR=n` in `sdkconfig` —
   otherwise the AF loop fights the sweep and keeps dragging the lens back. The
   build `#error`s if you forget.
2. Set `FOCUS_SWEEP 1` in `main/imx708_snapshot_main.c`.
3. Aim at something with fine detail (text, brickwork) a couple of metres away,
   keep the camera still, and flash.

It steps the DAC across the full range, prints a sharpness score per position
and writes `focus_<code>.bmp` for each. Read the **table**, not the pictures:
sharpness rises from the infinity end, peaks where your subject is, and falls
towards macro. The codes where the curve goes flat at each end are the
mechanical stops — those bound the useful range, and belong in `af.min_pos` /
`af.max_pos`. A completely flat column means the lens never moved at all.

The sharpness metric is the mean absolute horizontal gradient of the green
channel over the frame's centre band. It is computed independently of the ISP's
own AF statistic on purpose: if the two disagree about where best focus is, the
problem is the `af.windows` or `edge_thresh`, not the actuator. It cannot tell
defocus from motion blur, so hold the camera still.

### Other things you may want to change

- **`CONFIG_CAM_MOTOR_DW9807_INIT_POS`** (450) — where the lens parks before AF
  converges, so it governs the first few frames. 450 sits just inside the
  infinity end. Raise it towards 700–900 if the camera mostly looks at things
  within arm's reach.
- **`CONFIG_CAM_MOTOR_DW9807_PERIOD_US`** (1000) — settling time allowed per DAC
  code. Raise it if autofocus lands somewhere different each run on the same
  static scene; that is the signature of definition being sampled while the lens
  is still moving.


## Getting frames off the board

Captured frames travel down the same USB cable as the log, so a capture no
longer means powering off and carrying the microSD to a PC. The console runs at
**2 Mbaud** for this (`CONFIG_ESP_CONSOLE_UART_BAUDRATE`); a whole run is about
8 seconds.

Two formats, chosen deliberately:

| | format | size | why |
| --- | --- | --- | --- |
| normal capture | JPEG q90 | ~250-300 KB, ~1.5 s | small, and anything can open it |
| `FOCUS_SWEEP` | raw RGB565 | 4.1 MB, ~21 s | lossless, because the sweep is *measurement* |

**Do not measure sharpness on a JPEG.** Measured here with `JPEG_VALIDATE`: the
same frame scores 1411 as JPEG q90 against 1712 raw — 17.6% low. The focus sweep
peak is only ~1.25x its floor, so that shift is comparable to the entire signal,
and it is content-dependent (a sharper frame has more high-frequency energy, is
quantised harder, and is dragged further down). JPEG is for looking at.

`IMAGE_OUT_SD` (default off) restores the old BMP-to-card path. With it off the
example never touches the SD card, so no card need be inserted.

### Receiving

```bash
python tools/capture.py --flash
```

That flashes and then captures in **one process**, which matters: the port has a
single owner. A serial monitor left running in another terminal makes
`idf.py flash` fail with `Access is denied`, and a monitor cannot recover the
image anyway - it mangles binary as it prints it. Close any monitor before
running this. `--flash` is optional; without it the script just listens.

JPEG frames land as `.jpg`. Raw frames are converted to `.bmp` so they open
directly. Use `--seconds` to cover a longer run (a `FOCUS_SWEEP` needs roughly
25 s per position) and `--baud` if the console rate has been changed.

Frames are framed in the stream as:

```
IMGSTART name=<n> fmt=<jpeg|rgb565> w=<w> h=<h> len=<N> crc32=<hex>
<exactly N raw bytes>
IMGEND
```

A length and a CRC, so the receiver knows it got the frame rather than inferring
it from delimiters that could occur inside the data. Verify the CRC — it is what
caught the bug below.

### Two things that will bite you here

**The task watchdog writes into your binary.** A 4 MB frame takes ~21 s, and if
the sending loop never yields, the idle task starves and the TWDT prints a
warning *and a backtrace* straight into the middle of the payload — four
917-byte injections in a 21 s transfer, silently corrupting it. It bypasses
`esp_log_level_set` because it writes directly rather than through the tag
system. The send loop yields every 32 chunks to prevent this.

**The console rewrites `
` as `
`.** Binary payloads must set
`uart_vfs_dev_port_set_tx_line_endings(..., ESP_LINE_ENDINGS_LF)` first, or every
0x0A byte in the image gains a 0x0D.

Also note the baud is only settable under `ESP_CONSOLE_UART_CUSTOM`. Under the
default choice the symbol has no prompt and kconfgen silently keeps 115200 no
matter what `sdkconfig.defaults` says.


## Bring-up switches (all OFF by default)

`main/imx708_snapshot_main.c` has two switches at the top, kept for
diagnosing a new mode or board:

- `TEST_PATTERN 1` — the sensor emits its internal colour bars instead of the
  scene. The bars are generated after the pixel array, so they cross the whole
  MIPI → CSI → ISP path. **Clean bars = the transport works** and any bad photo
  is optics/exposure/ISP tuning. **Noisy bars = the fault is upstream of the
  ISP** (link rate, lane count, data type, sensor mode).
- `POISON_BUFFERS 1` — each capture buffer is filled with `0xA5` before
  streaming, and the log reports how much of it survived. `poison check: ~100%`
  means nothing was DMA'd in and the BMP is stale PSRAM, not a photo. The fill
  is flushed out of L2 first; without that, its dirty cache lines overwrite what
  the DMA later writes and eat the bottom of the frame.


## If the BMP looks wrong

- **Colours swapped / weird** — the ISP's output byte order may differ from the
  assumed RGB565; tell me what you see and we'll adjust `rgb565_to_bgr()`.
- **All one colour / black** — exposure or ISP config; check the monitor log for
  the captured `bytes=` value and the ISP warning.
