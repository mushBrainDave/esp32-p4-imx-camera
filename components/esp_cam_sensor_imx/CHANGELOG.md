# Changelog

All notable changes to `esp_cam_sensor_imx` are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions
follow [semantic versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-09-01

The IMX219 (Raspberry Pi Camera Module v2 / NoIR v2) goes from written-but-never-run
to verified on hardware: streaming, auto-exposure, stills and H.264 video.

### Upgrading

- **Run `idf.py reconfigure` in any existing build tree.** This release adds the
  Kconfig option `CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT`, and a `sdkconfig`
  that predates it will not have the symbol. The driver carries an `#ifndef`
  fallback so a stale tree keeps its previous behaviour rather than failing to
  build, but reconfiguring is what actually picks the option up.
- **A caret range on `^0.1.x` does not reach this release.** Projects pinned that
  way stay on the 0.1 line; move them to `^0.2.0`.

### Added

- **IMX219 ISP tuning config**, `sensors/imx219/cfg/imx219_default.json` — AE,
  AWB, denoise, gamma, sharpening and metering weights. Without it esp_video
  logs only `failed to get configuration to initialize ISP controller` and runs
  with no auto-exposure and no white balance.
- **`imx219_snapshot` example** — one still, hardware-JPEG encoded, sent down the
  console UART. No autofocus (the v2 module is fixed-focus and has no VCM on the
  bus), with an auto-exposure convergence trace in its place.
- **`imx219_video` example** — ~8 s of H.264 buffered in PSRAM and shipped down
  the console. Measured **1632×1232 at 28.1 fps**, 225 frames, no encoder
  failures. The encoder is the limit, not the sensor: 35.3 ms mean encode
  against a 33.3 ms frame interval.
- **1632×1232 sensor mode** (index 2), the 1640-wide binned mode with 8 columns
  trimmed at the sensor's readout window so the width is a whole number of
  16-pixel H.264 macroblocks. Same VTS, so identical frame rate and exposure
  limits; the X start is a multiple of 4, so 2×2 binning keeps the RGGB phase.
- **`CAMERA_IMX219_MIPI_IF_FORMAT_INDEX_DEFAULT`** to choose the start-up mode.
  Defaults to 0, the existing 1640×1232.

### Fixed

- **IMX219 gain is now an enumeration**, as esp_video requires. It drives AE gain
  as a menu control — `VIDIOC_QUERYMENU`, binary search, set the index — and
  `esp_video_cam_query_menu()` rejects anything that is not
  `ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION`. Declared as a plain number, AE
  silently drove exposure only, which looks like a dark, grainy picture rather
  than like an error. The table spans the sensor's real 1.0×–10.667× at roughly
  1/12 stop. `GROUP_EXP_GAIN` and `get_para_value` are implemented too.
- **IMX219 exposure is clamped to the mode's frame length**, not to the 16-bit
  register width. Integration time cannot exceed VTS, and VTS is per-mode here
  (1763 binned, 3526 full), so a fixed constant cannot express it. The
  descriptor reports the same ceiling, which matters: esp_video range-checks
  `S_EXT_CTRLS` against `qdesc.number.maximum`, so an honest descriptor turns an
  over-range AE request into a clean rejection instead of a write the sensor
  ignores.
- **`isp_info.gain_def` said 0 while `set_format` wrote code 100.** Both now name
  `IMX219_ANA_GAIN_DEFAULT` (104, the table entry nearest the old hardcoded
  value), so the register and the driver's state agree. The picture is unchanged.
- **`examples/imx219_capture`'s README** no longer tells you to look for a
  changing `seq`. esp_video never fills `v4l2_buffer.sequence` at
  `VIDIOC_DQBUF`, so it reads 0 however well the sensor is streaming, and the
  old criterion would have you read a healthy stream as a failure.

### Known limitations

- **The IMX219 colour matrix is the identity matrix — not calibrated.** The
  IMX708's tuned matrix is deliberately not reused: a CCM is a per-sensor,
  per-CFA measurement, and a borrowed one would look like tuning while being an
  unmeasured guess. Colour is flat until it is measured against a chart, and the
  NoIR variant has no IR-cut filter, so infrared contaminates all three channels.
- **The IMX219 runs out of light sooner than the IMX708.** Its gain ceiling is
  10.667× against 16×, and with `ac_freq: 60` the anti-flicker step of 440.8
  lines puts the fourth step (1763) above the 1759-line exposure ceiling, so it
  can never be taken. In a dim room AE pins at 1322 lines with gain maxed.
- The IMX219's 3280×2464 mode is still unusable: it is wider than the ESP32-P4's
  ~1920 px datapath limit and produces duplicated columns.
- **The ESP32-P4's ISP crop needs chip revision v3.0.** esp_video gates
  `ESP_VIDEO_ISP_DEVICE_CROP` on `CONFIG_ESP32P4_REV_MIN_FULL >= 300`, so on
  earlier silicon `VIDIOC_S_SELECTION` returns `ESP_ERR_NOT_SUPPORTED`. This is
  why the 16-alignment above is done at the sensor rather than in the ISP.
- PDAF is not driven.

## [0.1.2] - 2026-08-30

### Added

- The two WiFi examples now ship with the component: **`imx708_wifi_snapshot`**
  (an HTTP server answering `GET /snapshot.jpg`) and **`imx708_wifi_video`**
  (live 1080p H.264 in a browser, fragmented MP4 muxed on the board, plus a raw
  Annex-B endpoint for `ffplay`). Both drive the board's ESP32-C6 over SDIO,
  since the ESP32-P4 has no radio of its own.
- The glue those examples need is vendored into each one's own `components/`
  directory: `imx_wifi` for both, and `imx_fmp4` for the video example. An
  example copied out of the component has nothing around it, so anything shared
  from the repository root would not exist for a consumer.

### Security

- `examples/**/wifi_credentials.h` is excluded from the packed archive.
  The file is gitignored, but `compote component pack` reads the working tree
  rather than git, so a developer who had filled in their SSID and password
  would otherwise have published them. Only the `.example` template ships.

## [0.1.1] - 2026-08-30

First release to the ESP Component Registry.

0.1.0 was tagged but never reached the registry - the upload failed on an
API token scope - so no version was ever created under that number.

### Added

- **IMX708 driver** (Raspberry Pi Camera Module 3 / NoIR 3) for the ESP32-P4
  `esp_cam_sensor` framework. 2×2 binned 1920×1080 RAW10 at 28 fps, digitally
  cropped from 2304 wide to stay under the platform's width ceiling. Verified on
  hardware: streaming, exposure and gain through `esp_video`'s 3A loop, and
  H.264 encode at full frame rate.
- **DW9807 autofocus VCM driver**, the actuator inside the Camera Module 3, on
  I2C `0x0c`. Registers through `.esp_cam_motor_detect_fn` and is driven by
  `esp_ipa`'s AF algorithm via `esp_video`'s pipeline controller. Verified on
  hardware against a measured DAC/position curve.
- **ISP tuning config** for the IMX708 (`sensors/imx708/cfg/imx708_default.json`)
  — AE, AWB, denoise, gamma and sharpening, metering weights, saturation, and a
  seed CCM.
- **IMX219 driver** (Raspberry Pi Camera Module v2), binned 1640×1232 and full
  3280×2464 RAW10. **Written but never run on hardware**, so it is off by
  default in Kconfig; its register timing, MIPI lane rate and ISP tuning all
  still need bench confirmation.
- Three examples shipped with the component: `imx708_capture`,
  `imx708_snapshot` and `imx708_video`.

### Known limitations

- PDAF is not driven.
- The CCM is a seed matrix, not a calibration against a colour chart under known
  illuminants.
- On ESP-IDF below v5.4.4 the examples must define `ISP_AWB_WINDOW_X_NUM` and
  `_Y_NUM` as 5 to match the prebuilt `esp_ipa` binary's `esp_ipa_stats_t`
  layout; without it autofocus silently scores unrelated heap memory. The
  examples carry a version-guarded workaround.
