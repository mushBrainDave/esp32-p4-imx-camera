# Changelog

All notable changes to `esp_cam_sensor_imx` are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions
follow [semantic versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-08-30

First release to the ESP Component Registry.

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
