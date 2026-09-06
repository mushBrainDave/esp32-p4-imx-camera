# esp_cam_sensor_imx

Sony **IMX-series** MIPI-CSI camera drivers for the **ESP32-P4**, plugging into
Espressif's [`esp_cam_sensor`](https://components.espressif.com/components/espressif/esp_cam_sensor)
/ [`esp_video`](https://components.espressif.com/components/espressif/esp_video)
framework — so the Raspberry Pi camera modules work on the P4.

Espressif's stock sensor set covers OV-series and Arducam-branded modules, but
none of the Raspberry Pi IMX sensors have a generic driver. This component fills
that gap.

| Sensor | Module | Status |
| ------ | ------ | ------ |
| IMX708 | Pi Camera Module 3 / NoIR 3 | 🟢 **Working on hardware** — streaming, ISP tuning, autofocus, H.264 |
| IMX219 | Pi Camera Module v2 / NoIR v2 | 🟢 Streaming, ISP tuning, stills and H.264 video — verified on hardware. Fixed-focus, so no AF. Off by default; turn it on and turn the IMX708 off |

| Sensor | Mode | Format | FPS | Notes |
| ------ | ---- | ------ | --- | ----- |
| IMX708 | 1920×1080 | RAW10 | 28 | 2×2 binned, digitally cropped — the mode the examples use |
| IMX708 | 1280×720 | RAW10 | 28 | The same readout cropped harder — 44% of the pixels, 56% of the FOV each way. 16-aligned both axes |
| IMX708 | 1024×768 | RAW10 | 28 | 4:3 — narrower than 720p across, taller down it. Both axes whole macroblocks, so no encoder trim |
| IMX708 | 800×600 | RAW10 | 28 | SVGA, the middle 4:3 step. Width is whole macroblocks; height needs the same encoder trim as 1080 |
| IMX708 | 640×480 | RAW10 | 28 | 15% of the pixels. A tight 4:3 window out of a 16:9 field, so it reframes rather than shrinks |
| IMX219 | 1640×1232 | RAW10 | 30 | 2×2 binned, full FOV — recommended first target |
| IMX219 | 3280×2464 | RAW10 | 15 | Full resolution, higher bandwidth |

### The IMX708 cannot downscale

Worth recording, because it is the obvious thing to reach for and it is not
there. The CCS scaling block at `0x0400..0x0407` is **read-only** on this part:
every byte of it was written and read back unchanged on the bench, while
`0x0408..0x040F`, the digital crop immediately after it, took every write. The
registers exist and report the CCS defaults — `scaling_mode` 0, `scale_m` 16,
`scale_n` 16, meaning "1:1, not scaling" — and are hardwired there. Raspberry
Pi's own driver writes none of them in any of its four modes, which now looks
less like an omission.

So an output size smaller than the digital crop does not resample the crop, it
crops again from the crop's origin — and quietly, because the output-size
registers *do* take: the frame arrives at exactly the size asked for, correctly
formed and sharp, showing a corner of the field. Below the crop size,
resolution has to come from binning, from moving the analog readout window, or
from the P4's ISP downstream.

Two modes that drove the scaler were written and tested against this, then
removed once it was clear the sensor ignores them. If a later revision needs
re-testing, the method is a byte at a time: write, read back, see whether it
moved.

### One readout, three crops

The five IMX708 modes are one readout, not five. Each is a centred digital
crop of the same 2×2-binned 2304×1296 field with the same line and frame
timing, so a smaller mode buys CSI bandwidth, PSRAM and encode time and pays
for it in field of view. It does not make the sensor faster — every mode reads
out at 28 fps — but it does recover frames the pipeline drops at full size:
`imx708_video` measures 27.3 fps at 1920×1080 and 28.0 at every smaller mode.
Exposure range does not change.
Nothing is scaled, so 640×480 is a narrow window on the middle of the scene
rather than the whole scene shrunk. Pick the start-up mode with
`CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT`, or change it at run time by
passing `imx708_format_by_index()` / `imx708_format_by_size()` to esp_video's
`VIDIOC_S_SENSOR_FMT` before you allocate buffers.

## Choosing a resolution

The start-up mode is a build-time setting, `CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT`
(and `CAMERA_IMX219_...` for the other sensor). Indices run largest to smallest:

| Index | IMX708 | Index | IMX219 |
| ----- | ------ | ----- | ------ |
| 0 | 1920×1080 (default) | 0 | 1640×1232 (default) |
| 1 | 1280×720 | 1 | 3280×2464 |
| 2 | 1024×768 | 2 | 1632×1232 |
| 3 | 800×600 | | |
| 4 | 640×480 | | |

### Interactively

```
idf.py menuconfig
```

Then `Component config` → `Camera Sensor (IMX add-on)`. The option sits
indented under **Support IMX708 (Raspberry Pi Camera Module 3 / NoIR 3)**, as
**Default MIPI-CSI mode index**. Set it, exit, save, and rebuild. Each index
has help text describing what that mode costs in field of view.

### From a script or CI

`sdkconfig.defaults` is only read when `sdkconfig` does **not** exist, so
adding a line to it and rebuilding an existing tree changes nothing — the build
succeeds and silently keeps the old mode. Delete `sdkconfig` first:

```bash
echo 'CONFIG_CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT=2' > sdkconfig.mode
rm -f sdkconfig
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.mode" build
```

On PowerShell the equivalents are `Remove-Item sdkconfig -Force` and:

```powershell
Set-Content -Path sdkconfig.mode -Value 'CONFIG_CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT=2' -Encoding ascii
```

`-Encoding ascii` matters: Windows PowerShell 5.1's `utf8` writes a byte-order
mark, and kconfgen then reports `ignoring malformed line '#'`. (PowerShell 7
has `utf8NoBOM`; 5.1 does not.)

Two things that will otherwise cost you an afternoon:

- **`SDKCONFIG_DEFAULTS` sticks in the CMake cache.** Once passed, later plain
  `idf.py build` calls keep using it, and deleting the file it names breaks the
  build with `ninja: error: rebuilding 'build.ninja': subcommand failed`. Pass
  it explicitly every time, or `idf.py fullclean`.
- **An out-of-range index is silently clamped**, not rejected. Ask for 7 and
  kconfgen quietly gives you the default, 0, and builds a working image in the
  wrong mode. If a mode seems not to have taken, check the driver's log line —
  it prints the mode it actually set:

  ```
  I (965) imx708: set format: MIPI_2lane_24Minput_RAW10_1024x768_binned_28fps
  ```

### From application code

Both routes above are build-time: you edit a value, rebuild and flash. This one
is the same for the *examples*, whose knobs are `#define`s — but it is the call
an application makes, and an application that reads its index from somewhere
(a serial command, NVS, a button) can change resolution without being rebuilt.
That is the only sense in which any of this is run-time.

`imx708_format_by_index()` and
`imx708_format_by_size()`, declared in `imx708.h`, return a format to hand to
esp_video's `VIDIOC_S_SENSOR_FMT`:

```c
#include "imx708.h"

int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);

const esp_cam_sensor_format_t *want = imx708_format_by_index(2);   /* 1024x768 */
ioctl(fd, VIDIOC_S_SENSOR_FMT, (void *)want);

struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
f.fmt.pix.width       = want->width;
f.fmt.pix.height      = want->height;
f.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
ioctl(fd, VIDIOC_S_FMT, &f);
/* ... then REQBUFS, QUERYBUF, QBUF, STREAMON as usual */
```

Do it **before the first `VIDIOC_STREAMON`**, and treat that as a hard rule
rather than tidiness. Two reasons:

- Buffers are sized for the format in force when they were requested, so a
  mode change under allocated buffers is wrong by construction.
- **3A does not survive a later switch.** The ISP/IPA pipeline is created once,
  by `esp_video_init()`, and reads the sensor geometry then. Nothing
  re-initialises it afterwards — `esp_video_isp_pipeline_init()` is not in a
  public header — so a mode change after streaming has begun moves the geometry
  and leaves auto-exposure, auto-white-balance and autofocus behind. Measured:
  the frame is the right size and correctly formed, and nearly black, with the
  `esp_ipa_af` log lines simply absent from that point on.

So one switch, before streaming. Changing resolution mid-run means reflashing,
until esp_video exposes a pipeline re-init — and as of **2.4.1**, the newest
release, it does not. Upstream `master` does not either: its public
`esp_video_isp_pipeline.h` declares no init, deinit or restart entry point, and
`esp_video_isp_pipeline_init()` stays in `private_include/`. Restarting the
whole video stack is not a workaround, because `esp_video_init()` re-runs
sensor detect and comes back at the Kconfig default.

Verified on hardware, both examples: firmware built for mode 0, switched at
start-up, with 3A working. `imx708_snapshot` at 800×600 gives a correctly
exposed, in-focus frame (`bytesused=960000`, autofocus 512 → 641);
`imx708_video` at 1024×768 records 224 frames, 0 failed, 28.0 fps.

Both examples exercise this path — `CAPTURE_MODE_INDEX` in
`imx708_snapshot_main.c`, `VIDEO_MODE_INDEX` in `imx708_video_main.c`, `-1` by
default to keep the build-time mode. Being `#define`s they still need a
rebuild, so as a way of *changing* resolution they buy nothing over the Kconfig
option; what they are is a worked example of the call, in code that runs on the
board. Neither example reads the index from anything, so **no example can
change mode without a reflash**.

The index is the unambiguous selector; look-up by size returns the first match.
The IMX219 driver has no equivalent yet, and is build-time only.

Developed and measured on a **Waveshare ESP32-P4-WIFI6** with **ESP-IDF v5.4.0**.
That is the only combination this has run on; the manifest's `idf: ">=5.4"` is
what has been on the bench, not the oldest release that might compile.

## How it plugs in

This is a standalone add-on. It does not fork `esp_cam_sensor`; it registers
itself into that framework's auto-detect array through the
`.esp_cam_sensor_detect_fn` linker section, which `esp_cam_sensor`'s `linker.lf`
gathers from *all* archives. The `-u <name>_detect` flags in `CMakeLists.txt`
force the objects to be linked, since nothing in an application references them
directly. Autofocus motors work the same way, via `.esp_cam_motor_detect_fn`.

At start-up `esp_video` probes the SCCB bus and binds by chip ID: IMX708 at I2C
`0x1a` (`0x0708`), IMX219 at `0x10` (`0x0219`). The first success signal in the
log is `detected IMX708, PID=0x0708`.

## Install

```bash
idf.py add-dependency "mushbraindave/esp_cam_sensor_imx^0.2.0"
```

Or start from a working example, which brings its own `sdkconfig.defaults`:

```bash
idf.py create-project-from-example "mushbraindave/esp_cam_sensor_imx^0.2.0:imx708_capture"
```

Then in `menuconfig`:

- **Camera Sensor (IMX add-on) → Support IMX708**, or **Support IMX219** for a
  Camera Module v2. Both work; they are separate options so a build carries only
  the register tables it uses.
- In the `esp_video` config, enable the MIPI-CSI video device and the ISP video
  device, and configure the CSI controller for **2 data lanes**.
- For IMX708 autofocus, enable **Camera Motor (IMX add-on) → DW9807**, plus
  `ESP_IPA_AF_ALGORITHM`, `ESP_VIDEO_ENABLE_CAMERA_MOTOR_CONTROLLER` and
  `ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR`. Dropping any one of those three
  fails differently and none of them says so out loud: without the IPA algorithm
  nothing decides where to focus, without the motor controller `esp_video` has
  no way to reach the VCM, and without the pipeline control the AF result is
  computed and then discarded.

Each example's `sdkconfig.defaults` is a working reference for all of the above.

## Wiring up the ISP tuning config — required for a usable image

The IMX708's ISP tuning lives in
[`sensors/imx708/cfg/imx708_default.json`](sensors/imx708/cfg/imx708_default.json)
— AE, AWB, denoise, gamma/sharpen, metering weights, saturation and CCM. It
**cannot be registered from this component.** `esp_ipa` reads the
`ESP_IPA_JSON_CONFIG_FILE_PATH` build property while its own `CMakeLists.txt` is
processed, which has already happened by the time a dependency's `CMakeLists.txt`
runs. The application has to register it, between `include(project.cmake)` and
`project()`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH
    "managed_components/mushbraindave__esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json"
    APPEND)

project(my_camera_app)
```

The path is relative to the project directory. It only has to exist by the time
`esp_ipa` is configured, which is after the component manager has populated
`managed_components/`, so it is valid on a clean first build.

**Get this wrong and nothing tells you.** `esp_ipa`'s own existence check is
spelled `message(FETAL_ERROR ...)`, which is not a real CMake mode, so a missing
file prints a line and the build continues. What you get is
`esp_ipa_pipeline_get_config("IMX708")` returning NULL, one
`failed to get configuration to initialize ISP controller` line from `esp_video`,
and then a stream with no auto-exposure and no white balance. The examples here
re-check the resolved path in `main/CMakeLists.txt` and fail the build instead —
worth copying.

## Examples

| Example | What it does |
| ------- | ------------ |
| [`imx708_capture`](examples/imx708_capture/) | Streams frames and logs size and brightness per frame. Start here. |
| [`imx708_snapshot`](examples/imx708_snapshot/) | One still, hardware-JPEG encoded, sent down USB serial. Also carries the focus-sweep and buffer-poison diagnostics. |
| [`imx708_video`](examples/imx708_video/) | ~8 s of 1080p H.264 into PSRAM, then the whole clip down USB serial. Measured **27–28 fps**. |
| [`imx708_wifi_snapshot`](examples/imx708_wifi_snapshot/) | Camera + WiFi + an HTTP server: `GET /snapshot.jpg` from a browser. |
| [`imx708_wifi_video`](examples/imx708_wifi_video/) | **Live 1080p H.264 over WiFi**, played in a browser tab. Fragmented MP4 muxed on the board, plus a raw Annex-B endpoint for `ffplay`. |
| [`imx219_snapshot`](examples/imx219_snapshot/) | One still from a **Camera Module v2 / NoIR v2** (IMX219), hardware-JPEG encoded, sent down USB serial. No autofocus — the v2 is fixed-focus — and an AE convergence trace in its place. |
| [`imx219_video`](examples/imx219_video/) | ~8 s of H.264 from the IMX219 into PSRAM, then the clip down USB serial. Measured **28.1 fps at 1632x1232**; uses the driver's 16-aligned sensor mode because the P4's ISP crop needs chip revision v3.0. |

Each example is self-contained: the glue it needs is vendored into its own
`components/` directory rather than shared, so it still builds after being
copied out of the component. `imx708_snapshot`, `imx708_video`,
`imx219_snapshot` and `imx219_video` carry
`imx_serial_img`, a framed CRC-checked blob format that sends images down the
console UART at 2 Mbaud so no microSD card is needed. The two WiFi examples
carry `imx_wifi`, which routes `esp_wifi` over SDIO to the board's ESP32-C6 —
the P4 has no radio of its own — and `imx708_wifi_video` also carries
`imx_fmp4`, a fragmented-MP4 muxer that compiles on the host.

**The WiFi examples need credentials before they will associate.** Copy
`components/imx_wifi/include/wifi_credentials.h.example` to
`wifi_credentials.h` beside it and fill in your SSID and password. It is pulled
in behind `__has_include`, so creating it for the first time needs an
`idf.py fullclean`; without it the firmware builds and reports that it has no
credentials.

The host-side receiver, the board bring-up tools and the write-ups live in the
[project repository](https://github.com/mushBrainDave/esp32-p4-imx-camera).

Each example carries its own `sdkconfig.defaults` with the target included, so
`idf.py build flash` is enough — no `set-target`, which would discard the
generated config.

## Hardware notes

- **Lanes:** both sensors are 2-lane. IMX219 runs a fixed 456 MHz link
  (912 Mbps/lane) for all modes.
- **XCLK:** 24 MHz. On the Waveshare board the Pi-style 15-pin CSI connector
  routes neither reset nor pwdn and the sensor free-runs on its own oscillator,
  so there is no host XCLK and both pins are `-1`.
- **Bayer order:** RGGB at default orientation. H/V flip changes the effective
  Bayer phase, and the ISP config must track it if you enable flips.
- **Width ceiling between 1920 and 2048 px.** At 2304 wide, scene data ran out
  around x=1918 and the edge columns duplicated each other exactly 2048 px
  apart. The limit is in the datapath, not the sensor — Espressif ship every P4
  camera example at ≤1920 wide. This is why the IMX708 mode is digitally cropped
  to 1920.
- **ESP-IDF below v5.4.4** needs `ISP_AWB_WINDOW_X_NUM`/`_Y_NUM` defined as 5, or
  `esp_ipa_stats_t` is 400 bytes shorter than the prebuilt `esp_ipa` binary
  expects and autofocus silently scores unrelated heap memory. The examples carry
  a version-guarded workaround in their `CMakeLists.txt`.
- The CCM in the tuning config is a deliberately gentle seed rather than a
  calibration, because the NoIR module has no IR-cut filter and infrared
  contaminates all three channels.

## Licensing

Apache-2.0. The Linux kernel `imx219.c` / `imx708.c` used as a reference are
GPL-2.0; only non-copyrightable facts — register addresses and the standard
initialisation values published by Sony and in the Raspberry Pi firmware — were
used, and no source lines were copied. If you believe any content here is
copyrightable and improperly included, please open an issue.
