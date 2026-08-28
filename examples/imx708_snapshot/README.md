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

## Expect rough image quality

There's no ISP tuning for the IMX708 yet, so colour/white-balance/exposure are
uncontrolled — and with the **NoIR** module (no IR-cut filter) colours skew
pink/red in daylight. The image may also be **soft**: autofocus (VCM at I2C
0x0c) isn't driven, so the lens sits at its rest position. Detail + brightness
tracking the scene is the win here; making it *look good* is the ISP-tuning and
autofocus work that comes next.

## If the BMP looks wrong

- **Colours swapped / weird** — the ISP's output byte order may differ from the
  assumed RGB565; tell me what you see and we'll adjust `rgb565_to_bgr()`.
- **All one colour / black** — exposure or ISP config; check the monitor log for
  the captured `bytes=` value and the ISP warning.
