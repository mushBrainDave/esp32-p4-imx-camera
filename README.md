# esp_cam_sensor_imx

Sony **IMX-series** MIPI-CSI camera drivers for the **ESP32-P4**, targeting the
`esp_cam_sensor` / `esp_video` framework — so the Raspberry Pi camera modules
work on the P4.

Espressif's stock sensor set covers OV-series and Arducam-branded modules, but
**none of the Raspberry Pi IMX sensors** (IMX219 / IMX477 / IMX708) have a
generic driver. This project fills that gap.

> **Status: early bring-up.** IMX219 (Camera Module v2) driver is written and
> structured against the real `esp_cam_sensor` API, but has **not yet been
> validated on hardware**. Treat register timing, MIPI lane rate, and ISP
> tuning as needing bench confirmation. See [Roadmap](#roadmap).

## Supported / planned sensors

| Sensor | Module | Status |
| ------ | ------ | ------ |
| IMX219 | Pi Camera v2 | 🟡 Driver written, untested on HW |
| IMX708 | Pi Camera v3 / NoIR v3 | 🟡 Driver written (2×2 binned RAW10); I2C link verified on HW, streaming untested. AF/PDAF deferred |
| IMX477 | Pi HQ Camera | ⚪ Planned |

**Hardware progress:** on a Waveshare ESP32-P4-WIFI6, the `i2c_probe` example
confirmed a real IMX708 answering at I2C `0x1a` (chip-id reg `0x0016` =
`0x0708`) — cable, power, and SCCB bus all validated. Streaming bring-up of the
IMX708 driver is the current step.

IMX219 modes implemented:

| Mode | Format | FPS | Notes |
| ---- | ------ | --- | ----- |
| 1640×1232 | RAW10 | 30 | 2×2 binned, full FOV — **recommended bring-up mode** |
| 3280×2464 | RAW10 | 15 | Full resolution, higher bandwidth |

## How it plugs in

This is a **standalone add-on component**. It does not fork
`esp_cam_sensor`; it registers itself into that framework's auto-detect array
through the `.esp_cam_sensor_detect_fn` linker section (which `esp_cam_sensor`'s
`linker.lf` gathers from *all* archives). The `-u imx219_detect` flag in
`CMakeLists.txt` forces the object to be linked.

### Install

Copy `components/esp_cam_sensor_imx/` into your project's `components/`
directory (alongside the managed `espressif/esp_cam_sensor` dependency), then in
`menuconfig`:

- Enable **Camera Sensor (IMX add-on) → Support IMX219**.
- In the `esp_video` config, make sure the MIPI-CSI controller is enabled and
  configured for **2 data lanes**, 24 MHz sensor XCLK.

At start-up, `esp_video`'s device init will probe I2C address `0x10`, read the
chip ID (`0x0219`), and bind this driver.

### Try it

[`examples/imx219_capture/`](examples/imx219_capture/) is a self-contained
bring-up test: it inits `esp_video`, streams 30 frames, and logs size +
brightness per frame. Set the board pins at the top of its `main`, then
`idf.py set-target esp32p4 && idf.py build flash monitor`. First success signal
is the `detected IMX219, PID=0x0219` line.

## Porting map — how the Linux driver maps onto ESP-IDF

The Linux `drivers/media/i2c/imx219.c` is a V4L2 subdev bolted to the kernel
media framework — not directly portable. What *is* portable is the
sensor-specific data. This table records where each piece came from and where it
lives here, so the same method can be repeated for IMX477/IMX708.

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

1. **Validate IMX219 binned mode on hardware** — confirm chip-ID read, then a
   clean RAW10 stream into the P4 ISP. Verify `mipi_clk` / HS-settle against the
   CSI controller; adjust if the receiver reports lane errors.
2. **ISP tuning (IPA JSON)** — add an `esp_ipa` tuning file for auto
   exposure/white-balance/CCM so images look right, not just green Bayer.
3. **Exposure/gain via the framework's AE loop** — wire `EXPOSURE_US` + gain
   into `esp_video`'s 3A path (group-hold if needed).
4. **IMX477** — same method, higher bandwidth; no autofocus.
5. **IMX708** — add the VCM autofocus motor driver (see `esp_cam_sensor/motors`,
   `dw9714` as a template) and handle the NoIR color path; PDAF is out of scope.

## Hardware notes

- **Lanes/bandwidth:** IMX219 is 2-lane, fixed 456 MHz link (912 Mbps/lane) for
  all modes. The binned mode is the safe first target; full-res 15fps pushes
  more data through the ISP + PSRAM.
- **XCLK:** 24 MHz. On many P4 boards the sensor clock is provided by the board;
  set `xclk_pin = -1` if so.
- **Bayer order:** RGGB at default orientation. H/V flip changes the effective
  Bayer phase — the ISP config must track it if you enable flips.

## Licensing

This code is **Apache-2.0**. The Linux kernel `imx219.c` used as a reference is
**GPL-2.0**; only non-copyrightable facts (register addresses, standard
initialisation values published by Sony / in the Raspberry Pi firmware) were
used. No source lines were copied. If you believe any content here is
copyrightable and improperly included, open an issue.

## Acknowledgements

- Espressif `esp-video-components` (`esp_cam_sensor`, `esp_video`) — the
  framework and the driver structure this follows.
- The Linux kernel and Raspberry Pi camera driver authors — register reference.
