# Using the published component

`esp_cam_sensor_imx` is on the ESP Component Registry as
[`mushbraindave/esp_cam_sensor_imx`](https://components.espressif.com/components/mushbraindave/esp_cam_sensor_imx).
This is the consumer's side of it — pulling the driver down, building it and
getting a stream out of a board, with none of this repository checked out.

Every command below was run end to end against the published 0.1.1 on a
Waveshare ESP32-P4-WIFI6 with ESP-IDF v5.4.0, board on COM3. Activating IDF is
PowerShell-only here; see [Environment](cli-cookbook.md#environment) for the
one-liner, since `export.ps1` does not work on this machine.

- [Start from an example](#start-from-an-example)
- [Reading the log](#reading-the-log)
- [Add it to an existing project](#add-it-to-an-existing-project)
- [When it goes wrong](#when-it-goes-wrong)

---

## Start from an example

This is the path worth using. Everything that fails silently is already correct
in it — the ISP tuning registration, the PSRAM speed, the L2 cache geometry and
the target.

Run it from an empty directory; it creates the project folder itself and does
not need a git repository:

```bash
idf.py create-project-from-example "mushbraindave/esp_cam_sensor_imx:imx708_capture"
```

The version is optional and defaults to the newest published. Pin it with
`mushbraindave/esp_cam_sensor_imx^0.1.1:imx708_capture` if you want a specific
line — a caret range on `0.x` covers that whole minor series, so `^0.1.1` picks
up 0.1.2 and later but stops at 0.2.0.

The other example names are `imx708_snapshot`, `imx708_video`,
`imx708_wifi_snapshot` and `imx708_wifi_video`. The two WiFi ones need a
`wifi_credentials.h` written from the `.example` template inside their vendored
`components/imx_wifi/include/` before they will associate — and creating that
file for the first time needs an `idf.py fullclean`, because it is pulled in
behind `__has_include` and so cannot appear in the depfile while it is
missing.

```bash
cd imx708_capture
idf.py build
```

No `set-target`. The example ships a `sdkconfig.defaults` with
`CONFIG_IDF_TARGET="esp32p4"` in it, and setting the target by hand discards the
generated config and rebuilds it from scratch.

The driver arrives as a managed dependency, so it lands in
`managed_components/mushbraindave__esp_cam_sensor_imx/` — that double underscore
is how the component manager spells `namespace/name` on disk, and the project
`CMakeLists.txt` points `ESP_IPA_JSON_CONFIG_FILE_PATH` at the tuning file
inside it.

```bash
idf.py -p COM3 flash
```

## Reading the log

`idf.py monitor` works but never exits, so for a scripted capture reset the
board over DTR/RTS and read the console directly. This example leaves the
console at the default 115200:

```python
import serial, time
s = serial.Serial('COM3', 115200, timeout=0.2)
s.dtr = False; s.rts = True; time.sleep(0.15); s.rts = False
buf = bytearray()
end = time.time() + 25
while time.time() < end:
    buf += s.read(4096)
s.close()
open('boot.log', 'wb').write(buf)
```

Any Python with pyserial does; the IDF venv has it.

Three lines say it started:

```
I (1524) imx708: detected IMX708, PID=0x0708
I (1584) imx708: set format: MIPI_2lane_24Minput_RAW10_1920x1080_binned_28fps
I (2674) imx708_capture: ==== IMX708 bring-up PASSED ====
```

**But the column that actually matters is the brightness**, because the worst
failure here is one that logs nothing:

```
frame  0: seq=0 bytes=4147200 avg=53
frame  1: seq=0 bytes=4147200 avg=53
frame  2: seq=0 bytes=4147200 avg=53
frame  3: seq=0 bytes=4147200 avg=116
frame  4: seq=0 bytes=4147200 avg=108
...
frame 29: seq=0 bytes=4147200 avg=109
```

Three frames flat, one overshoot, then locked — that is auto-exposure
converging, and it is the only evidence that the ISP tuning config reached
`esp_ipa`. **A brightness column that never moves means the tuning config was
not loaded**, and nothing else in the log will say so. See
[When it goes wrong](#when-it-goes-wrong).

Frame cadence in that run was 930 ms across 26 intervals — 35.8 ms/frame, or
27.9 fps, against the 28 fps the mode advertises. Every frame 4,147,200 bytes:
1920 × 1080 RGB565 out of the ISP.

## Add it to an existing project

Harder, because three things have to be done by hand that the examples already
carry.

```bash
idf.py add-dependency "mushbraindave/esp_cam_sensor_imx"
idf.py add-dependency "espressif/esp_video^2.4"
```

The driver plugs into `esp_cam_sensor` and depends on nothing else, so the
framework that exposes a video device has to be added alongside it.

**1. Kconfig.** `CAMERA_IMX708` is on by default once the component is present.
Enable `esp_video`'s MIPI-CSI and ISP video devices, and set the CSI controller
to **2 data lanes**. For autofocus add the DW9807 motor plus
`ESP_IPA_AF_ALGORITHM`, `ESP_VIDEO_ENABLE_CAMERA_MOTOR_CONTROLLER` and
`ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR` — drop any one of those three and
autofocus fails in a different way, silently, each time.

The IMX219 is off by default. It has never been run on hardware.

**2. The ISP tuning config**, in the project `CMakeLists.txt`, between
`include(project.cmake)` and `project()`. That window is the whole point:
`esp_ipa` reads the property while its own `CMakeLists.txt` is processed, which
is far too late for a dependency to set it.

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH
    "managed_components/mushbraindave__esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json"
    APPEND)

project(my_camera_app)
```

The path resolves against the project directory and only has to exist by the
time `esp_ipa` is configured — which is after the component manager has
populated `managed_components/` — so it is valid on a clean first build.

**3. On ESP-IDF below v5.4.4**, in the same window:

```cmake
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_X_NUM=5" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_Y_NUM=5" APPEND)
```

Without them `esp_ipa_stats_t` is 400 bytes shorter than the prebuilt `esp_ipa`
binary expects, `af_stats` lands at offset 196 where the library reads 596, and
autofocus scores unrelated heap memory — settling on its first scan point every
time. From v5.4.4 ESP-IDF defines these itself, and defining them twice is why
the shipped examples guard this on the version.

Then write plain V4L2 against `/dev/video0`. Application code never includes
`imx708.h` or calls the driver: it registers itself into `esp_cam_sensor`'s
auto-detect array through a linker section, and `esp_video` binds it by chip ID
at start-up.

The fastest way to get all of the above right is to diff your
`sdkconfig.defaults` and project `CMakeLists.txt` against `imx708_capture`'s.

## When it goes wrong

**The brightness column never moves.** The ISP tuning config did not load.
`esp_ipa`'s own existence check is spelled `message(FETAL_ERROR ...)` — not a
real CMake mode — so a wrong path prints one line and the build carries on to
produce a stream with no auto-exposure and no white balance. Check the path in
your project `CMakeLists.txt`, and look for `failed to get configuration to
initialize ISP controller` from `esp_video` in the boot log. The shipped
examples re-check the resolved path in `main/CMakeLists.txt` and fail the build
instead; that check is worth copying.

**`the override_path you're using is pointing to directory "..." that is not a
component`.** An example manifest kept a development-time `override_path`.
Delete that line from `main/idf_component.yml` and the dependency resolves from
the registry normally. The published examples are stripped of it, so this
should not happen — worth reporting if it does.

**`esptool ... Access is denied`** on COM3. The port has exactly one owner and
something else is holding it; a serial monitor left open in VS Code or the
Arduino IDE is the usual culprit. Check it from PowerShell:

```powershell
try { $p = New-Object System.IO.Ports.SerialPort COM3,115200; $p.Open(); $p.Close(); "COM3 is free" } catch { "COM3 busy" }
```

**`CMake Warning ... maximum full path to an object file is 250 characters`.**
Not a component problem — the project path is too long. Every component in the
build emits one, stock IDF components included. Move the project somewhere
shorter.

**No `detected IMX708` line at all.** The sensor is not answering on the SCCB
bus, which is wiring rather than software: the IMX708 sits at I2C `0x1a` and
reports `0x0708`. On the Waveshare board the Pi-style 15-pin CSI connector
routes neither reset nor pwdn, and the sensor free-runs on its own oscillator,
so there is no host XCLK and both pins are `-1`.
