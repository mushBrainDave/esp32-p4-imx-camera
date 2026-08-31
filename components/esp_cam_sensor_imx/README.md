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
| IMX219 | Pi Camera Module v2 | 🟡 Driver written, **never run on hardware** — off by default |

| Sensor | Mode | Format | FPS | Notes |
| ------ | ---- | ------ | --- | ----- |
| IMX708 | 1920×1080 | RAW10 | 28 | 2×2 binned, digitally cropped — the mode the examples use |
| IMX219 | 1640×1232 | RAW10 | 30 | 2×2 binned, full FOV — recommended first target |
| IMX219 | 3280×2464 | RAW10 | 15 | Full resolution, higher bandwidth |

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
idf.py add-dependency "mushbraindave/esp_cam_sensor_imx^0.1.1"
```

Or start from a working example, which brings its own `sdkconfig.defaults`:

```bash
idf.py create-project-from-example "mushbraindave/esp_cam_sensor_imx^0.1.1:imx708_capture"
```

Then in `menuconfig`:

- **Camera Sensor (IMX add-on) → Support IMX708** (the IMX219 is opt-in; it has
  never been run on hardware).
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

`imx708_snapshot` and `imx708_video` ship a copy of `imx_serial_img` in their own
`components/` directory — a framed, CRC-checked blob format that carries images
down the console UART at 2 Mbaud, so no microSD card is needed. The host-side
receiver, WiFi and live-streaming examples, and the bring-up write-ups live in
the [project repository](https://github.com/mushBrainDave/esp32-p4-imx-camera).

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
