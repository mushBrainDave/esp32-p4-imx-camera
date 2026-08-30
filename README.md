# esp_cam_sensor_imx

Sony **IMX-series** MIPI-CSI camera drivers for the **ESP32-P4**, targeting the
`esp_cam_sensor` / `esp_video` framework — so the Raspberry Pi camera modules
work on the P4.

Espressif's stock sensor set covers OV-series and Arducam-branded modules, but
**none of the Raspberry Pi IMX sensors** (IMX219 / IMX477 / IMX708) have a
generic driver. This project fills that gap.

> **Status: the IMX708 is a working camera.** It streams, the ISP is tuned, and
> autofocus works — see the examples below, which take stills and video off the
> board over USB serial or WiFi. The **IMX219 driver is written but has never
> been flashed**; treat its register timing, MIPI lane rate and ISP tuning as
> needing bench confirmation. See [Roadmap](#roadmap).

## Supported / planned sensors

| Sensor | Module | Status |
| ------ | ------ | ------ |
| IMX708 | Pi Camera v3 / NoIR v3 | 🟢 **Working on hardware** — streaming, ISP tuning, autofocus, H.264 |
| IMX219 | Pi Camera v2 | 🟡 Driver written, **never run on hardware** |
| IMX477 | Pi HQ Camera | ⚪ Planned |

Modes implemented:

| Sensor | Mode | Format | FPS | Notes |
| ------ | ---- | ------ | --- | ----- |
| IMX708 | 1920×1080 | RAW10 | 28 | 2×2 binned, digitally cropped — **the mode everything uses** |
| IMX219 | 1640×1232 | RAW10 | 30 | 2×2 binned, full FOV — recommended first target |
| IMX219 | 3280×2464 | RAW10 | 15 | Full resolution, higher bandwidth |

**Autofocus** is implemented for the IMX708's DW9807 VCM
(`components/esp_cam_sensor_imx/motors/dw9807`, I2C `0x0c`), driven by
`esp_ipa`'s AF algorithm through `esp_video`'s pipeline controller. PDAF is out
of scope.

**ISP tuning** lives in
`components/esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json` — AE,
AWB, denoise, gamma/sharpen, metering weights, saturation and CCM. The CCM is a
deliberately gentle seed rather than a calibration, because the NoIR module has
no IR-cut filter and infrared contaminates all three channels.

Development board is a **Waveshare ESP32-P4-WIFI6**, on **ESP-IDF v5.4.0**.

## How it plugs in

This is a **standalone add-on component**. It does not fork `esp_cam_sensor`; it
registers itself into that framework's auto-detect array through the
`.esp_cam_sensor_detect_fn` linker section (which `esp_cam_sensor`'s `linker.lf`
gathers from *all* archives). The `-u <name>_detect` flag in `CMakeLists.txt`
forces the object to be linked. Motors work the same way, via
`.esp_cam_motor_detect_fn`.

### Install

Copy `components/esp_cam_sensor_imx/` into your project's `components/`
directory (alongside the managed `espressif/esp_cam_sensor` dependency), then in
`menuconfig`:

- Enable **Camera Sensor (IMX add-on) → Support IMX708** (or IMX219).
- For IMX708 autofocus, also enable **CAM_MOTOR_DW9807** plus
  `ESP_IPA_AF_ALGORITHM`, `ESP_VIDEO_ENABLE_CAMERA_MOTOR_CONTROLLER` and
  `ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR`. Dropping any one of the three
  fails differently and none of them says so out loud.
- In the `esp_video` config, make sure the MIPI-CSI controller is enabled and
  configured for **2 data lanes**.

At start-up `esp_video` probes the SCCB bus and binds by chip ID: IMX708 at I2C
`0x1a` (`0x0708`), IMX219 at `0x10` (`0x0219`). The first success signal is the
`detected IMX708, PID=0x0708` line.

Each example's `sdkconfig.defaults` is a working reference for all of the above.

## Examples

| Example | What it does |
| ------- | ------------ |
| [`i2c_probe`](examples/i2c_probe/) | Walks the SCCB bus and reads chip IDs. The first thing to run on new hardware. |
| [`imx708_capture`](examples/imx708_capture/) | Streams frames and logs size and brightness per frame. |
| [`imx708_snapshot`](examples/imx708_snapshot/) | One still, hardware-JPEG encoded, sent down USB serial. Also carries the focus-sweep and buffer-poison diagnostics. |
| [`imx708_video`](examples/imx708_video/) | ~8 s of 1080p H.264 into PSRAM, then the whole clip down USB serial. Measured **27–28 fps**. |
| [`imx708_wifi_snapshot`](examples/imx708_wifi_snapshot/) | Camera + WiFi + an HTTP server: `GET /snapshot.jpg` from a browser. |
| [`imx219_capture`](examples/imx219_capture/) | The IMX219 equivalent of `imx708_capture`. **Untested on hardware.** |
| [`c6_link_check`](examples/c6_link_check/) | Five-second answer to "is the ESP32-C6 radio alive": brings up WiFi and scans. |
| [`c6_wifi_sta`](examples/c6_wifi_sta/) | Associates with an AP, takes a DHCP lease, proves the route out. |

Each carries its own `sdkconfig.defaults`, target included, so `idf.py build flash`
is enough — there is no need to `set-target`, which would discard the generated
config and regenerate it.

## Getting frames off the board

There is no need for a microSD card, and both paths are in-tree.

**USB serial**, via [`components/imx_serial_img/`](components/imx_serial_img/) —
a framed, CRC-checked blob format over the console UART at 2 Mbaud.
[`tools/capture.py`](tools/capture.py) flashes, resets, captures and extracts the
images in one command:

```bash
python tools/capture.py --flash --project examples/imx708_snapshot
```

**WiFi**, via [`components/imx_wifi/`](components/imx_wifi/) — the P4 has no
radio of its own, so this goes over SDIO to the board's ESP32-C6 running
`esp_hosted` slave firmware. `imx708_wifi_snapshot` then serves frames over
HTTP. Measured sustained throughput is **6–7 Mbit/s**; a 260 KB JPEG takes
0.47 s on a warm connection and 1.70 s on a cold one, because TCP slow start
dominates anything small. Full numbers and two non-obvious traps are in that
example's [README](examples/imx708_wifi_snapshot/README.md), and the radio
bring-up is written up in [`docs/esp32c6-bringup.md`](docs/esp32c6-bringup.md).

WiFi credentials go in a **gitignored** `wifi_credentials.h`, written from the
committed `.example` template beside it.

## Docs

- [`docs/cli-cookbook.md`](docs/cli-cookbook.md) — the commands actually used to
  build, flash, capture and diagnose: stills over SD/USB/WiFi, video, the C6
  radio, and decoding a RISC-V panic. Most are not the obvious first guess.
- [`docs/esp32c6-bringup.md`](docs/esp32c6-bringup.md) — the ESP32-C6 radio:
  schematic pin map, version ceiling, the slave OTA, and the measured
  throughput.

## Porting map — how the Linux driver maps onto ESP-IDF

The Linux `drivers/media/i2c/imx219.c` is a V4L2 subdev bolted to the kernel
media framework — not directly portable. What *is* portable is the
sensor-specific data. This table records where each piece came from and where it
lives here, so the same method can be repeated for IMX477.

| Linux `imx219.c` (GPL-2.0) | This project (Apache-2.0) |
| -------------------------- | ------------------------- |
| `IMX219_REG_*` addresses, `CHIP_ID 0x0219` | `imx219_regs.h` |
| `imx219_common_regs[]`, `imx219_2lane_regs[]`, `mode_*_regs[]` | `imx219_settings.h` register arrays |
| `supported_modes[]` (w/h/vts/binning) | `imx219_format_info[]` + `imx219_isp_info[]` in `imx219.c` |
| `imx219_set_ctrl()` exposure/gain/flip → register writes | `imx219_set_para_value()` + `imx219_set_exposure/gain/mirror/vflip()` |
| `V4L2_CID_*` control IDs | `ESP_CAM_SENSOR_*` control IDs (`esp_cam_sensor_types.h`) |
| `regmap`/CCI 16-bit writes | `esp_sccb_*_reg_a16v8()` helpers (16-bit reg, 8-bit val; 16-bit values split big-endian) |
| link freq 456 MHz / pixel rate 182.4 MHz | `IMX219_MIPI_CSI_LINE_RATE` (912 Mbps/lane) / `pclk` in ISP info |
| device-tree probe + PM | `imx219_detect()` + `imx219_power_on/off()` |

Only factual register numbers and standard bring-up values are reproduced. No
GPL code is copied. See [Licensing](#licensing).

## Roadmap

Done:

1. ~~IMX708 streaming on hardware~~
2. ~~ISP tuning (IPA JSON)~~ — AE, AWB, CCM seed, denoise, sharpening
3. ~~Exposure/gain into `esp_video`'s 3A loop~~ — gain had to be exposed as an
   **enumeration**, not a number; see the platform traps below
4. ~~IMX708 autofocus~~ — DW9807 VCM, working
5. ~~Frames off the board without an SD card~~ — JPEG and H.264 over USB serial
6. ~~WiFi~~ — ESP32-C6 radio up, HTTP server serving stills

Next:

7. **Video over WiFi.** The measured 6–7 Mbit/s ceiling rules out MJPEG at full
   quality (~3 fps); H.264 at 4 Mbit/s fits with headroom, on one held-open
   connection.
8. **CCM calibration** against a colour chart under known illuminants, to
   replace the seed matrix.
9. **IMX477.**
10. **First hardware run for the IMX219** — back-port the width-ceiling and
    gain-enumeration lessons before trying it.

## Hardware notes

- **Lanes/bandwidth:** both sensors are 2-lane. IMX219 runs a fixed 456 MHz link
  (912 Mbps/lane) for all modes; the binned mode is the safe first target.
- **XCLK:** 24 MHz. On the Waveshare board the Pi-style 15-pin CSI connector
  routes neither reset nor pwdn, and the sensor free-runs on its own oscillator
  — so there is **no host XCLK** and both pins are `-1`.
- **Bayer order:** RGGB at default orientation. H/V flip changes the effective
  Bayer phase, and the ISP config must track it if you enable flips.
- **Width ceiling between 1920 and 2048 px.** At 2304 wide, scene data ran out
  around x=1918 and the edge columns duplicated each other exactly 2048 px
  apart. The limit is in the datapath, not the sensor — Espressif ship every P4
  camera example at ≤1920 wide. This is why the IMX708 mode is digitally
  cropped to 1920.

Five platform-level traps hit during bring-up — **every one of which fails
silently, with no error and just a bad or absent image** — are worth reading
before adding a sensor. They cover the width ceiling above, the gain-as-
enumeration requirement, where IPA JSON registration must be declared, a PSRAM
speed setting that silently falls back to 20 MHz, and an ABI mismatch with the
prebuilt `esp_ipa` binary that silently breaks autofocus on ESP-IDF v5.4.0.
The `ISP_AWB_WINDOW_X_NUM/_Y_NUM=5` block in each example's `CMakeLists.txt` is
the workaround for the last of these; remove it if you move to IDF ≥ v5.4.4,
which defines those macros itself.

## Licensing

This code is **Apache-2.0**. The Linux kernel `imx219.c` / `imx708.c` used as a
reference are **GPL-2.0**; only non-copyrightable facts (register addresses,
standard initialisation values published by Sony / in the Raspberry Pi firmware)
were used. No source lines were copied. If you believe any content here is
copyrightable and improperly included, open an issue.

## Acknowledgements

- Espressif `esp-video-components` (`esp_cam_sensor`, `esp_video`) — the
  framework and the driver structure this follows.
- The Linux kernel and Raspberry Pi camera driver authors — register reference.
