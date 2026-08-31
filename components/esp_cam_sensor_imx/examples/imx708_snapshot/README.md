# IMX708 snapshot

Grabs one IMX708 frame, hardware-JPEG encodes it, and sends it down the USB
serial link. Pins preset for the Waveshare ESP32-P4-WIFI6.

**No microSD card is needed.** The card path still exists behind
`IMAGE_OUT_SD` (default off) and writes a 24-bit BMP to `/sdcard/imx708.bmp`;
with it off the example never touches the SD hardware. See
[Getting frames off the board](#getting-frames-off-the-board).

Frames are 1920x1080. The sensor's binned mode is 2304 wide and is **digitally
cropped to 1920** — there is a datapath width ceiling between 1920 and 2048 px
on the P4, above which the scene runs out and the edge columns duplicate each
other exactly 2048 px apart. Do not "restore" the full width.

For the same thing served over WiFi instead of the cable, see
[`imx708_wifi_snapshot`](https://github.com/mushBrainDave/esp32-p4-imx-camera/tree/main/examples/imx708_wifi_snapshot),
in the project repository.

## Before you run

Aim the camera at something with detail and light. That is all.

## Build, flash and receive — one command

```bash
python ../../../../tools/capture.py --flash
```

That flashes, resets the board, captures the stream and writes the image out.
**Do not use `idf.py monitor` here**: the port has a single owner, so a monitor
left running makes `idf.py flash` fail with `Access is denied`, and a monitor
mangles binary as it prints it, so it cannot recover the frame anyway.

After boot the example starts streaming, prints
`aim the camera — settling for 6 s...`, then grabs a frame and sends it. The
settle window is what gives auto-exposure, white balance and autofocus time to
converge. The run ends at `==== done ... ====` and the JPEG lands in the output
directory.

## What you're validating

- **The image is a real, changing photo of the scene** → the whole pipeline is
  genuinely live end-to-end. This is the visual proof.
- With `IMAGE_OUT_SD` on, the board SD pins used are CLK 43, CMD 44, D0 39,
  D1 40, D2 41, D3 42 (4-bit), with SD power via **on-chip LDO channel 4** —
  set `host.pwr_ctrl_handle` or the card never powers up.

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
- **`agc.anti_flicker.ac_freq` is set to 60**, for 60 Hz mains. Change it to 50
  if yours is 50 Hz, or you may see banding under artificial light. Note that
  moving it 50 -> 60 here made **no visible difference**: at 60 Hz the light
  ripples at 120 Hz (8.33 ms), `anti_flicker: full` snaps exposure to integer
  multiples of that, and in a dim room AE is already pinned near the 35 ms
  ceiling, so the constraint is nearly non-binding. Banding shows up at *short*
  exposures, so retest in a bright scene before concluding the setting is
  inert.
- **The CCM is a seed, not a calibration.** Real values come from shooting a
  colour chart under known illuminants and solving for the matrix. The rows sum
  to 1.0 so neutrals stay neutral, and it is deliberately gentler than
  Espressif's OV5647 matrix because a **NoIR** module has no IR-cut filter —
  infrared contaminates all three channels and aggressive saturation amplifies
  that error.
- **The `af` scan band needs calibrating.** `af.min_pos` / `af.max_pos` (380 and
  940) come from libcamera's Camera Module 3 tuning data, not from your module.
  See *Autofocus* below.

### The build warning that was not harmless

This section used to say the `excess elements in array initializer` warnings
from the generated `esp_video_ipa_config.c` were benign generator noise. **That
was wrong, and believing it cost weeks.** They were the visible symptom of an
ABI mismatch that silently broke autofocus.

The prebuilt `esp_ipa` binary picked for us (`lib/esp32p4/v5.4-`) was compiled
with `ISP_AWB_WINDOW_X_NUM` / `_Y_NUM` = 5. Those macros only exist in ESP-IDF
>= v5.4.4, so on v5.4.0 `esp_ipa_stats_t::awb_subwin` collapses to `[0][0]` —
**400 bytes shorter than the library expects**. Everything after that member
then sits at the wrong offset: measured with a compiled `offsetof` probe,
`af_stats` landed at 196 in a 224-byte struct while the library read it from
596, past the end of the struct entirely. AE and AWB survived because their
members sit *before* `awb_subwin`; AF read unrelated heap memory, so every scan
point returned the same `definition` and the lens always parked at `min_pos`.

The fix is in this example's project `CMakeLists.txt`, set globally before
`project()` so esp_video (writes the struct), the generated config (declares it)
and the library (reads it) all agree:

```cmake
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_X_NUM=5" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_Y_NUM=5" APPEND)
```

The 400 bytes come back as inert padding, and `ESP_VIDEO_ISP_DEVICE_AWB_SUBWIN`
stays off because it keys on the IDF version rather than these macros, so
nothing calls sub-window APIs v5.4.0 lacks. **The warnings disappearing is now
the confirmation that the layouts agree** — if they come back, the struct
layouts have diverged again. Moving to IDF >= v5.4.4 defines the macros itself;
remove the block then, or they are defined twice.

General lesson: treat any struct-shaped warning against a prebuilt library as an
ABI report, not noise.

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

From this example's directory:

```bash
python ../../../../tools/capture.py --flash
```

or from the repo root, where it defaults to this example:

```bash
python tools/capture.py --flash
```

`tools/` sits at the repo root while `idf.py flash` has to run inside a project,
so the script resolves the two independently: it flashes the current directory
when that is a project, otherwise this example, or whatever `--project` names.

It flashes and then captures in **one process**, which matters: the port has a
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


## Bring-up switches

`main/imx708_snapshot_main.c` has a block of switches at the top, kept for
diagnosing a new mode or board. Defaults in brackets.

| Switch | Default | What it does |
| --- | --- | --- |
| `IMAGE_OUT_SERIAL` | 1 | Send frames down the console UART |
| `IMAGE_OUT_SD` | 0 | Also write a BMP to the microSD card |
| `AF_DEBUG_LOG` | 1 | Turn on `esp_ipa_af`'s own per-scan-point logging |
| `FOCUS_SWEEP` | 0 | Step the VCM across its range, scoring each position |
| `JPEG_VALIDATE` | 0 | Send the same frame raw *and* as JPEG, to compare |
| `TEST_PATTERN` | 0 | Sensor emits colour bars instead of the scene |
| `POISON_BUFFERS` | 0 | Prefill capture buffers and count survivors |
| `RAW_PATTERN_TEST` | 0 | Send a synthetic counter instead of the image |

The four diagnostics worth understanding before you need them:

- **`TEST_PATTERN`** — the bars are generated after the pixel array, so they
  cross the whole MIPI → CSI → ISP path. **Clean bars = the transport works**
  and any bad photo is optics/exposure/ISP tuning. **Noisy bars = the fault is
  upstream of the ISP** (link rate, lane count, data type, sensor mode). Note
  the sensor clips the bars to its own test-pattern window (regs
  `0x0620`-`0x0627`, left at power-on defaults), so they come out **cropped at
  the left and right edges even when the capture path is perfect** — judge by
  the bars that *are* drawn being clean and correctly placed.
- **`POISON_BUFFERS`** — each capture buffer is filled with `0xA5` before
  streaming and the log reports how much survived. `poison check: ~100%` means
  nothing was DMA'd in and the image is stale PSRAM, not a photo. The fill is
  flushed out of L2 first; without that its dirty cache lines overwrite what the
  DMA later writes and eat the bottom of the frame in cache-line-sized holes.
- **`RAW_PATTERN_TEST`** — replaces the image with a known counter sequence
  before sending. Every byte of image data looks plausible, so an inserted or
  dropped byte is invisible in a photo; against a known sequence a
  resync-and-diff prints the exact offset, length and contents of each
  insertion. This is what identified the watchdog injections described above,
  after several rounds of fruitless inference from image structure.
- **`AF_DEBUG_LOG`** — `esp_ipa` ships as a closed `.a` but its `ESP_LOGD`
  strings are intact, so `esp_log_level_set("esp_ipa_af", ESP_LOG_DEBUG)` from
  the app is enough to make it print one line per scan point. Do **not** raise
  `CONFIG_LOG_MAXIMUM_LEVEL` for this — it does nothing for a prebuilt library
  and turns `ESP_LOGD` on firmware-wide.


## If the image looks wrong

- **Colours swapped / weird** — the ISP's output byte order may differ from the
  assumed RGB565; adjust `rgb565_to_bgr()`.
- **All one colour / black** — exposure or ISP config; check the log for the
  captured `bytes=` value and the ISP warning. Turn on `POISON_BUFFERS` to tell
  "nothing was captured" apart from "captured, but badly exposed".
- **Measure it, do not eyeball it.** Row-to-row correlation near 0.99 means real
  scene content; near 0.0 means there is no image in the buffer at all. A byte
  histogram with heavy 0x55/0xAA bias is the signature of never-written DRAM.
  Luma *percentiles* beat the mean — a p95 of 61 revealed an underexposure that
  a mean of 40 understated.
- **Never compare sharpness across scenes.** The same metric ranged 374-4102
  across pre-autofocus captures purely from scene and exposure. Only compare
  frames of the same scene, and prefer a within-run sweep to two separate runs.
