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
`aim the camera — capturing in 3 s...`, then grabs a frame and writes the BMP.
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
matrix), `aen` (gamma, sharpen, contrast), `adn` (denoise) and `ian` (metering
weights).

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
- **No `af` block.** The VCM at I2C 0x0c isn't driven, so the lens sits at its
  rest position and the image is slightly soft.

### A harmless build warning

The build prints several `excess elements in array initializer` warnings from
the generated `esp_video_ipa_config.c`. esp_ipa 2.3.0's generator always emits a
5x5 `awb.subwin_weight`, but on ESP-IDF v5.4 `ISP_AWB_WINDOW_X_NUM` is undefined
so the struct member is `float[0][0]`. The initialisers are discarded at compile
time and `enable_sub_win` is false, so nothing reads it. It is a generator/IDF
version mismatch, not a problem with this config.

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
