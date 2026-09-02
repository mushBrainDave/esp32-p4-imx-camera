# CLI cookbook

The commands that actually got used bringing this board up — flashing,
standing up a new sensor, capturing stills over three different transports,
recording video, and getting the ESP32-C6 radio onto a router. Kept because most of them are not the
obvious first guess, and several exist to work around something that fails
silently.

Environment is Windows 11, PowerShell 5.1 plus Git Bash, ESP-IDF **v5.4.0** at
`~/esp/v5.4/esp-idf`, board on **COM3**.

**Two shells, and it matters which.** Anything with `$env:`, `Get-Content`,
`Compare-Object`, `Add-Type` or `curl.exe` is **PowerShell**; anything with
`grep`, `tr`, `printf`, `find` or `$(...)` is **Git Bash**. `idf.py`,
`esptool` and `python tools/...` run in either, once IDF is activated — but IDF
activation itself is PowerShell-only here. PowerShell 5.1 has no `&&`, so chain
with `;` or `if ($?) { ... }`.

- [Environment](#environment)
- [Build and flash](#build-and-flash)
- [Sensor bring-up](#sensor-bring-up)
- [Stills: SD card](#stills-sd-card)
- [Stills: USB serial](#stills-usb-serial)
- [Stills: WiFi](#stills-wifi)
- [Video over USB](#video-over-usb)
- [Video over WiFi](#video-over-wifi)
- [ESP32-C6 radio](#esp32-c6-radio)
- [Post-mortem and instrumentation](#post-mortem-and-instrumentation)

---

## Environment

**Activating ESP-IDF.** `export.ps1` fails here with "ESP-IDF Python virtual
environment not found" even though the venv exists. This works, and has to be
one PowerShell invocation because shell state does not persist between calls:

```bash
$env:IDF_PATH="$HOME\esp\v5.4\esp-idf"; $env:IDF_TOOLS_PATH="$HOME\.espressif"; $exp = & "$HOME\.espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe" "$env:IDF_PATH\tools\activate.py" --export 2>$null | Select-Object -Last 1; . $exp; Set-Location "$HOME\source\repos\imx708\components\esp_cam_sensor_imx\examples\imx708_snapshot"; idf.py build
```

`activate.py --export` writes a temp `.ps1` and prints its path as the **last**
line of stdout, which is what gets dot-sourced. Piping stderr away matters
because PowerShell 5.1 wraps native stderr in ErrorRecords and the pipeline then
looks like a failure.

**"running scripts is disabled on this system".** The activation above
dot-sources a temp `.ps1`, which an unsigned-script execution policy refuses.
Fix it for the session only — no admin, no permanent change, gone when the
window closes:

```bash
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

**`capture.py`'s `--project` and `--out` are both relative to your current
directory.** Standing in one example's directory and passing a repo-root-relative
`--project` resolves to a path that does not exist; worse, `--out` quietly
succeeds and drops the capture under whatever example you happened to be
standing in. Either `cd` into the target example and omit `--project`, or pass
absolute paths.

**Is anything holding the serial port?** The port has exactly one owner, and a
monitor left open in another terminal makes `esptool` fail with
`Access is denied`. Check without disturbing anything:

```bash
$p = New-Object System.IO.Ports.SerialPort COM3; try { $p.Open(); $p.Close(); "COM3 free" } catch { "COM3 BUSY: $($_.Exception.Message)" }
```

**Regenerating `sdkconfig` after editing `sdkconfig.defaults`.** The defaults
are only applied when `sdkconfig` does not exist, so an edit is silently ignored
otherwise. Move the old one aside rather than deleting it, then diff to confirm
only what you intended moved:

```bash
mv sdkconfig sdkconfig.before && idf.py build
```

```bash
Compare-Object (Get-Content sdkconfig.before) (Get-Content sdkconfig) | Where-Object { $_.InputObject -match "^CONFIG|^# CONFIG" }
```

Not `sdkconfig.old` — `idf.py` writes that name itself when it rewrites the
config, so a backup parked there gets clobbered by the very build you wanted to
compare against. `sdkconfig.before` is **not** gitignored either, so delete it
once you have read the diff.

**Do not anchor a PowerShell `-replace` with `$` against a `-Raw` file.** This
looks like it reverts a one-line config experiment, and silently does nothing:
without `RegexOptions.Multiline`, `$` matches only the end of the whole string,
not the end of a line.

```bash
(Get-Content $f -Raw) -replace 'CONFIG_FREERTOS_HZ=100$','CONFIG_FREERTOS_HZ=1000' | Set-Content $f
```

Match the line ending instead, and read the value back rather than assuming.
Note `Set-Content -Encoding utf8` writes a BOM, which kconfgen then warns about
as `ignoring malformed line`:

```bash
(Get-Content $f -Raw) -replace 'CONFIG_FREERTOS_HZ=100(\r?\n)','CONFIG_FREERTOS_HZ=1000$1' | Set-Content $f -NoNewline -Encoding utf8; Select-String -Path $f -Pattern "^CONFIG_FREERTOS_HZ"
```

A stale tick rate left behind by exactly this cost most of a day chasing a
"slow network" that was really a `vTaskDelay` rounding to zero — see
[Video over WiFi](#video-over-wifi).

**Reading an effective config value** — the generated `sdkconfig`, not the
defaults file, is the truth:

```bash
grep -n "ESP_HOSTED_SDIO_TX_Q_SIZE\|SPIRAM_SPEED\|FREERTOS_HZ" sdkconfig
```

---

## Build and flash

```bash
idf.py build
```

```bash
idf.py -p COM3 flash
```

`idf.py monitor` is the wrong tool for anything in this repo: it never exits,
and it mangles binary payloads as it prints them. Use `tools/capture.py`, which
owns the port for flashing *and* capture in one process.

**Checking image size against the partition.** The build prints this; it is the
line that catches an app outgrowing a 1 MB factory partition:

```bash
idf.py build 2>&1 | Select-String "binary size|Project build complete"
```

---

## Sensor bring-up

Standing up the IMX219 from scratch, in the order the questions actually arise.
Every step answers one question and nothing else, which is the point: the
platform traps in this repo all fail silently, so a run that "looks wrong" tells
you nothing until you have narrowed down which layer is wrong.

**Does the sensor answer at all?** `examples/i2c_probe` talks I2C only — no
MIPI, no ISP, no esp_video, no driver — so it separates a cable, power or
address problem from everything downstream. It scans the bus, then tries a
chip-ID read at each known Pi camera address:

```bash
python tools/capture.py --flash --project examples/i2c_probe --baud 115200 --seconds 12 --out probe
```

```
i2c_probe:   device ACK at 0x10
i2c_probe:   device ACK at 0x18
i2c_probe:   device ACK at 0x64
i2c_probe: ==== FOUND IMX219 (Pi Cam v2): addr 0x10, reg 0x0000 = 0x0219 ✓ ====
```

The bus is shared with the audio codec (`0x18`) and the PMIC (`0x64`), so
**those two answering while the camera does not** localises the fault to the
camera rather than to the bus. A Camera Module v2 shows exactly these three: no
`0x0c`, because it is fixed-focus and has no VCM, and no `0x50`, because it has
no module EEPROM — unlike the Module 3, which has both. The two
`i2c.master ... unexpected nack` errors after the FOUND line are the probe
trying its `0x1a` IMX708/IMX477 candidates, and are expected.

**Mind the console baud.** `capture.py` defaults to `--baud 2000000`, which is
right for the snapshot and video examples but wrong for `i2c_probe` and
`imx219_capture` — those leave the console at the ESP-IDF default of 115200.
Listening at the wrong rate gives garbage or silence, not an error. The baud is
only settable under `ESP_CONSOLE_UART_CUSTOM`; under the default choice
`CONFIG_ESP_CONSOLE_UART_BAUDRATE` has no prompt and kconfgen silently keeps
115200 whatever `sdkconfig.defaults` says:

```bash
grep -n "ESP_CONSOLE_UART_CUSTOM\|ESP_CONSOLE_UART_BAUDRATE" sdkconfig
```

**Is there really a picture in the buffer?** Do not trust a plausible-looking
mean. Untouched PSRAM is a 0x55/0xAA checkerboard whose byte average is 127.5,
and a real mid-exposure RGB565 frame averages about 130 — indistinguishable by
eye, and mistaking one for the other cost a round here. Fill the buffers with a
poison byte before `VIDIOC_STREAMON` and count what survives (`POISON_BUFFERS`
in the snapshot examples does exactly this):

```c
memset(buffer[i], 0xA5, b.length);
esp_cache_msync(buffer[i], b.length, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
```

The `msync` is not optional: the buffer is in PSRAM, so the memset's tail stays
dirty in L2 and those lines later evict over what the DMA wrote, eating the
bottom of the frame in cache-line-sized holes. Zero surviving poison, no
0x55/0xAA, a full 2..255 range and a per-frame-varying minimum is proof of live
DMA.

**`seq` is not a liveness signal.** esp_video never fills
`v4l2_buffer.sequence` at `VIDIOC_DQBUF` — it tracks a sequence internally and
never copies it out — so it reads 0 however well the sensor is streaming. Judge
liveness from `bytesused` and the DQBUF interval.

**Probing the V4L2 controls the 3A loop drives.** This is how the gain
enumeration and the exposure ceiling were verified on hardware rather than
argued about. Note that **esp_video implements `S_EXT_CTRLS` only** — there is
no `VIDIOC_S_CTRL` case at all — so a probe written with `S_CTRL` fails and
looks like a driver bug when it is a probe bug:

```c
struct v4l2_query_ext_ctrl q = { .id = V4L2_CID_EXPOSURE };
ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q);   /* the min/max the AE will be held to */

struct v4l2_querymenu qm = { .id = V4L2_CID_GAIN };
for (qm.index = 0; ioctl(fd, VIDIOC_QUERYMENU, &qm) == 0; qm.index++) { /* qm.value is milli-gain */ }
```

Walking a menu until it errors is the direct way to prove an
`ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION` really registered. The terminating
`esp_video_cam: ctrl id=.. is out of range(max=lx)` is just the loop ending —
that `max=lx` is esp_video's own broken format string, not a clue. Round-trip a
value through `S_EXT_CTRLS` then `G_EXT_CTRLS` (with
`ctrl_class = V4L2_CTRL_CLASS_USER`) to exercise `get_para_value` as well.

**Why AE stopped short of its target.** Three numbers box the exposure in, and
only the arithmetic separates them. Given the mode's line time and VTS:

```bash
python -c "tl=18904e-9; s=(1/120)/tl; print('flicker step %.1f lines; ceiling VTS-4 = 1759'%s); [print(' %dx -> %.0f'%(k,k*s)) for k in range(1,5)]"
```

For the IMX219's 1640x1232 mode that gives steps of 440.8 lines, so the legal
60 Hz exposures are 441/882/1322/1763 — and the fourth is four lines above the
1759 ceiling, so it can never be taken. AE pins at 1322 with gain maxed. Trace
luma, exposure and gain index *together* in the aim loop: luma alone cannot
distinguish "converged" from "still pushing and out of road".

**Generating a gain table for a new sensor.** The enumeration must hold the
gains the register codes *actually* achieve, not the ideal step targets, so that
what the AE reads back is what the sensor is doing. For the IMX219's
`gain = 256/(256-code)`, capped at code 232:

```bash
python -c "codes=[max(0,min(232,round(256-256000/(1000*2**(i/12))))) for i in range(42)]; print([round(256000/(256-c)) for c in codes]); print(codes)"
```

**Validating an IPA JSON before the build dies inside the generator.** A `//`
comment is only safe *inside* a block dict. `esp_ipa_config.py` treats every
top-level key except `version` as a sensor name and iterates it, so a top-level
`"//"` string gets walked character by character and the build fails with
`TypeError: string indices must be integers` — a traceback pointing at the
generator, not at your file. A sensor-level `"//foo"` sitting beside
`agc`/`awb`/`acc` is the same hazard. Check the shape first:

```bash
python -c "import json,sys; d=json.load(open(sys.argv[1])); [print(k,type(d[k]).__name__) for k in d]; [print(' ',b,type(v).__name__) for k in d if k!='version' for b,v in d[k].items()]" components/esp_cam_sensor_imx/sensors/imx219/cfg/imx219_default.json
```

Every sensor-level member must come out as `dict` or `list`. The runtime tell
that a config never loaded is
`W esp_video_init: failed to get configuration to initialize ISP controller`,
followed by a mean luma that never moves.

**Does esp_video support a feature on this IDF and this silicon?** Capability
macros are chained through several files, and a missing one compiles the whole
op away — the ops-table entry ends up NULL and the ioctl returns
`ESP_ERR_NOT_SUPPORTED` with no hint as to why. Follow the chain in the
*resolved* `managed_components/` copy rather than in the docs:

```bash
grep -rn "ESP_VIDEO_ISP_DEVICE_CROP" managed_components/espressif__esp_video/include/esp_video_caps.h managed_components/espressif__esp_video/src/device/esp_video_csi_device.c
```

That is how the ISP-crop route to a 16-aligned frame was ruled out.
`csi_set_selection` and its ops-table entry both sit behind
`#if CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE && ESP_VIDEO_ISP_DEVICE_CROP`, and
`ESP_VIDEO_ISP_DEVICE_CROP` is defined only under `CONFIG_SOC_ISP_CROP_SUPPORTED`
— **a symbol ESP-IDF v5.4.0 does not define at all**, on any P4 revision. Confirm
against the generated header rather than assuming:

```bash
grep -c "SOC_ISP_CROP_SUPPORTED" build/config/sdkconfig.h
```

A `0` there means the crop is compiled out, and the 16-alignment has to be done
at the sensor's readout window instead — which is what the driver's 1632x1232
mode is for.

---

## Stills: SD card

The original path, still available behind `IMAGE_OUT_SD` in
`components/esp_cam_sensor_imx/examples/imx708_snapshot`. Superseded by serial and then WiFi, but kept
because it needs no host tooling at all.

Set `IMAGE_OUT_SD 1` in `main/imx708_snapshot_main.c`, flash, wait for
`==== done`, power off, and move the card to a PC. Files land at `E:\<n>.bmp`.

**Copy them off with Git Bash first.** Python cannot open `E:/...` paths
reliably here, and after one long focus sweep the first three files came back
`corrupted and unreadable` to Windows:

```bash
cp /e/imx708.bmp ./shot/
```

---

## Stills: USB serial

One command flashes, resets, captures and extracts. Run it from the repo root
(where it defaults to `imx708_snapshot`) or from inside any example directory:

```bash
python tools/capture.py --flash
```

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_snapshot --out shot
```

Options that matter: `--seconds` to cover a longer run (a `FOCUS_SWEEP` needs
roughly 25 s per position), `--out` for the output directory, `--baud` if the
console rate changed, `--port` if not COM3, `--keep-raw` to retain the raw
payload. Without `--flash` it only listens.

**Run it with a Python that has pyserial** — the IDF venv does:

```bash
~/.espressif/python_env/idf5.4_py3.11_env/Scripts/python.exe tools/capture.py --flash
```

**Doing it by hand**, if `capture.py` is not an option: flash, then open the
port with pyserial at the console baud and toggle `dtr=False; rts=True;
sleep(0.15); rts=False` to reset, so the capture starts from boot.

**Decoding a JPEG for measurement.** There is no PIL and no ImageMagick on the
Windows side here (`/c/Windows/system32/convert` is the filesystem tool). Use
Windows' own codecs:

```bash
Add-Type -AssemblyName System.Drawing; $img=[System.Drawing.Image]::FromFile("$env:TEMP\snap.jpg"); "$($img.Width)x$($img.Height) $($img.PixelFormat)"; $img.Dispose()
```

The BMP it can save is **32-bpp** (4 bytes/px, still bottom-up), not 24 — size
your row arithmetic accordingly.

---

## Stills: WiFi

`components/esp_cam_sensor_imx/examples/imx708_wifi_snapshot` serves the camera over HTTP.
Flash it the same
way; the run ends by printing the URL:

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_wifi_snapshot --seconds 45 --out c6log_wifisnap
```

Then everything happens from the host, with no serial port involved at all:

```bash
curl.exe -s --max-time 30 -o shot/wifi_snapshot.jpg -w "size=%{size_download} time=%{time_total}s speed=%{speed_download} B/s\n" http://192.168.0.164/snapshot.jpg
```

**Measuring throughput.** `/bench` sends 8 MB from PSRAM with no camera in the
loop, which separates the network from the capture path:

```bash
curl.exe -s --max-time 90 -o NUL -w "size=%{size_download} time=%{time_total}s speed=%{speed_download} B/s\n" http://192.168.0.164/bench
```

```bash
curl.exe -s --max-time 10 http://192.168.0.164/stats
```

**Cold vs warm connection — the measurement that matters most.** Two URLs in one
`curl` invocation reuse the connection, and `%{num_connects}` proves it. The
second request came back 3.6x faster for the same bytes, which is TCP slow start,
not the radio:

```bash
curl.exe -s --max-time 60 -o a.jpg -o b.jpg -w "#%{num_connects} conn size=%{size_download} time=%{time_total}s ttfb=%{time_starttransfer}s\n" "http://192.168.0.164/snapshot.jpg?a" "http://192.168.0.164/snapshot.jpg?b"
```

**Response headers, when a transfer stalls.** `-D -` shows whether the server
believes it is sending the full payload — a correct `Content-Length` alongside a
few KB received proves the capture and encode were fine and the fault is in
transport:

```bash
curl.exe -s --max-time 12 -D - -o NUL http://192.168.0.164/snapshot.jpg
```

---

## Video over USB

`components/esp_cam_sensor_imx/examples/imx708_video` records ~8 s of 1080p H.264 into PSRAM, then ships the
clip in one framed payload. Allow generous time — a couple of MB has to move
after the recording finishes:

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_video --out clip
```

The IMX219 equivalent is the same command against `imx219_video`, which records
1632x1232 at 28.1 fps — see [Sensor bring-up](#sensor-bring-up) for why the
width is 1632 and not the sensor mode's 1640:

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx219_video --seconds 120 --out clip
```

`capture.py` writes both the raw `.h264` elementary stream and an `.mp4` muxed
around it, because an Annex-B stream is not a container and most players will
not touch one. To re-mux by hand:

```bash
python tools/mp4.py clip/imx708.h264 clip/imx708.mp4 --fps 28
```

**ffmpeg lives in WSL**, not on the Windows side. Use `/mnt/c/...` paths:

```bash
wsl ffprobe -hide_banner /mnt/c/Users/mushbrain/source/repos/imx708/clip/imx708.mp4
```

```bash
wsl ffmpeg -i /mnt/c/Users/mushbrain/source/repos/imx708/clip/imx708.h264 -vsync 0 /mnt/c/Users/mushbrain/source/repos/imx708/clip/frame_%03d.png
```

Windows' own codecs will not decode H.264 here, so this is the only way to turn
a clip into frames that the measurements above can run on. Note the
`D:\Downloads\ffmpeg-9.0.1` tree is **source, not a build** — ignore it.

Expect `Video: h264 (Constrained Baseline), yuv420p, 1920x1072, 28 fps`.

---

## Video over WiFi

`components/esp_cam_sensor_imx/examples/imx708_wifi_video` streams live H.264.
Flashing is the same; the run ends by printing three URLs, and note the
**stream lives on port 81** while the instruments stay on port 80 so they keep
answering during a stream:

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_wifi_video --out boot
```

```
I imx708_wifi_video: watch it at http://192.168.0.164:81/
I imx708_wifi_video:   ffplay http://192.168.0.164:81/stream.h264
I imx708_wifi_video:   instruments: http://192.168.0.164/stats  /bench  /skip?n=
```

### Watching and recording

The page at `:81/` plays it through Media Source Extensions. Without a browser,
ffmpeg takes the raw Annex-B endpoint directly — it is exactly what ffmpeg
expects on the wire, so nothing has to be muxed on either side:

```bash
wsl ffplay -fflags nobuffer -flags low_delay http://192.168.0.164:81/stream.h264
```

```bash
wsl ffmpeg -i http://192.168.0.164:81/stream.h264 -t 30 -c copy /mnt/c/Users/mushbrain/source/repos/imx708/clip/live.mp4
```

### The measurement that matters: frames *delivered*, not frames encoded

`/stats` reports what the encoder produced. What reached the client is a
separate number, and the gap between the two is where every bug in this example
lived. Pull a fixed window, then let `ffprobe` count what is actually in it:

```bash
curl -s --max-time 15 -o /tmp/s.h264 http://192.168.0.164:81/stream.h264; ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of default=nw=1:nk=1 /tmp/s.h264
```

15 s at 28 fps is ~420 frames. Far below that with `resyncs=0` means the sender,
not the link. **`curl` exiting early is itself the signal** — check the wall
time, not just the byte count, or every rate computed over the intended window
is wrong by the ratio of the two.

The same over the browser's endpoint, which also confirms the container:

```bash
curl -s --max-time 15 -o /tmp/s.mp4 http://192.168.0.164:81/stream.mp4; ffprobe -hide_banner -v error -count_frames -select_streams v:0 -show_entries stream=codec_name,profile,width,height,avg_frame_rate,nb_read_frames -of default=nw=1 /tmp/s.mp4
```

Expect `h264 / Constrained Baseline / 1920x1072` and an `avg_frame_rate` near
27. One `Invalid NAL unit size` at the very end is only the fragment `curl` cut
in half; the same error *mid-file* is a real container fault.

**Is there a picture in it, without looking at it?** Percentiles, not just a
mean — a mean of 128 is equally consistent with a good frame and with noise:

```bash
ffmpeg -v error -i /tmp/s.mp4 -vf 'select=gte(n\,15),format=gray' -vframes 1 -f rawvideo - | python3 -c "import sys;d=sys.stdin.buffer.read();n=len(d);r=sorted(d);print('mean %d p5 %d p50 %d p95 %d'%(sum(d)//n,r[n//20],r[n//2],r[n*19//20]))"
```

### Runtime knobs, so a setting can be changed without reflashing

One boot, one scene and one association is the only way two settings are
comparable here — the link moves far more between reboots than any setting does.

```bash
curl.exe -s "http://192.168.0.164/rate?kbit=1500"
```

```bash
curl.exe -s "http://192.168.0.164/skip?n=2"
```

`/rate` is the only one that changes how much of the link the stream asks for.
`/skip` does **not** save bandwidth — rate control's budget is bits per
*second*, so halving the frame rate just doubles the bits per frame. `n=0` stops
the encoder entirely, which is how "is the encoder starving the radio?" gets
answered rather than argued about (it is not).

A sweep of delivered frame rate against the target, all within one boot:

```bash
for KB in 4000 3000 2000 1000; do curl -s "http://192.168.0.164/rate?kbit=$KB" >/dev/null; sleep 3; curl -s --max-time 15 -o /tmp/f.h264 http://192.168.0.164:81/stream.h264; N=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of default=nw=1:nk=1 /tmp/f.h264); echo "$KB kbit/s -> $N frames, $(stat -c %s /tmp/f.h264) bytes"; done
```

### Checking the fMP4 muxer without a board

`components/imx_fmp4` inside the example has no ESP-IDF in it, so it compiles on
the host and can be pointed at a clip `imx708_video` already recorded. Do this
first: a muxing bug and a transport bug are indistinguishable from the far end
of a WiFi link.

```bash
cd components/esp_cam_sensor_imx/examples/imx708_wifi_video && gcc -O2 -Wall -Wextra -Icomponents/imx_fmp4/include components/imx_fmp4/test/fmp4_host_test.c components/imx_fmp4/imx_fmp4.c -o /tmp/fmp4_host_test
```

```bash
/tmp/fmp4_host_test ../../../../clip/imx708.h264 /tmp/frag.mp4 && ffmpeg -v error -i /tmp/frag.mp4 -f null -
```

Silence from ffmpeg means every frame decoded.

### Reading the board's log without resetting it

`tools/capture.py` toggles DTR/RTS to reset, which is right for a capture and
wrong when the point is to watch a *running* server answer a client. Leaving
both lines low attaches without resetting:

```python
s = serial.Serial(); s.port = 'COM3'; s.baudrate = 2000000; s.timeout = 0.2
s.dtr = False; s.rts = False; s.open()
```

This is what cracked the worst bug here. The board states plainly what a
connection did, and no amount of host-side inference was going to get there:

```
I h264 viewer joined at frame 4338 (1 watching)
W no frames for 5000 ms - dropping the h264 viewer
I h264 viewer left after 6 frames, 45 KB in 15 ms -> 24.7 Mbit/s
```

Five thousand milliseconds of waiting, in fifteen — because `pdMS_TO_TICKS(5)`
is `(5 * CONFIG_FREERTOS_HZ) / 1000`, which is **zero** at the 100 Hz default,
and `vTaskDelay(0)` yields without sleeping. **Add a "connection ended, and here
is what it actually did" log to anything long-lived.**

### Is it the link, or is it my code?

Three cheap discriminators, in the order worth trying.

**Reflash the previous known-good firmware.** The fastest way to exonerate new
code, and it worked: `imx708_wifi_snapshot`, unchanged, measured 0.05 Mbit/s on
a day it had done 7.2 the day before.

```bash
python tools/capture.py --flash --project components/esp_cam_sensor_imx/examples/imx708_wifi_snapshot --out snapback --seconds 40
```

**Ping while it is slow.** Loss and jitter mean congestion; **0% loss at 4 ms
while bulk transfer collapses** means bursty interference — small packets fit in
the gaps while 1500-byte frames get corrupted and retried. Neither is fixable in
firmware, and both are worth ruling out before reading any code:

```bash
ping -n 20 192.168.0.164 | Select-String "Lost|Average"
```

**`/bench` against the stream, in the same minute.** `/bench` is on the control
port and keeps working during a stream, so the two are directly comparable:

```bash
curl.exe -s --max-time 12 -o NUL -w "bench %{speed_download} B/s\n" http://192.168.0.164/bench
```

But note what `/bench` cannot see. It always has more data queued, so its writes
are never sub-MSS and **Nagle never engages** — it happily reported 7 Mbit/s
while the stream, which writes a frame then goes quiet for 36 ms, was stalled at
2-5 fps by Nagle interlocking with the receiver's delayed ACK. `TCP_NODELAY` on
the streaming socket is the fix. **A bulk benchmark does not model a paced
sender.**

### Before/after counters around one operation

Cumulative averages hide anything that changed mid-run. Read the counters either
side instead, and prefer `fps_recent`, which covers only the interval since the
previous `/stats`:

```bash
get() { curl -s http://192.168.0.164/stats | grep -E "^$1=" | cut -d= -f2; }
```

```bash
B=$(get frames_encoded); curl -s --max-time 15 -o /tmp/d.h264 http://192.168.0.164:81/stream.h264; echo "encoded $(( $(get frames_encoded) - B )) in 15 s"
```

The fields worth knowing: `dqbuf_us_mean` near zero means the encoder paces the
loop (normal here, and why 27.2 fps rather than 28); `send_us_mean` near the
frame interval means the link is the limit; `resyncs` climbing means viewers are
being jumped forward because the link cannot carry the current bitrate.

### When the board stops answering, check what is running on it

The first assumption is a dropped association. Check the cheap thing first:
**what firmware is actually flashed.** A board that has been reflashed for other
work answers nothing on the network and looks exactly like a WiFi fault. Reset
it and read the banner — no `--flash`, so nothing is overwritten:

```bash
python tools/capture.py --project components/esp_cam_sensor_imx/examples/imx708_wifi_video --seconds 25 --out boot
```

That cost a diagnosis here: an unreachable `192.168.0.164` and
`Destination host unreachable` turned out to be `imx219_video` on the board,
sending a clip down the serial line and never touching the radio.

If it *is* the radio, `components/imx_wifi` does reconnect on
`WIFI_EVENT_STA_DISCONNECTED` — but its `s_retries` counter is never reset on a
successful `GOT_IP`, so `MAX_RETRIES` (5) is a budget for the whole run rather
than per outage. After five cumulative disconnects it gives up for good and says
so:

```
E imx_wifi: giving up after 5 attempts, last reason 200: ...
```

That is fine for a bring-up probe and thin for something meant to stream for
hours; grep the log for `giving up` before concluding the AP is at fault.

---

## ESP32-C6 radio

The P4 has no radio; WiFi goes over SDIO to a C6 running `esp_hosted` slave
firmware. Full write-up in [`esp32c6-bringup.md`](esp32c6-bringup.md).

**Is the radio alive at all?** Five seconds, no credentials, no AP needed:

```bash
python tools/capture.py --flash --project examples/c6_link_check --seconds 40 --out c6log_scan
```

A returned AP list is real evidence — a scan needs a working radio, not merely a
driver that accepts a config. The scan list is also the authoritative spelling of
your SSID, and shows which of your networks are on 2.4 GHz (the C6 cannot see
5 GHz at all).

**Pinning the components.** `esp_hosted` 3.x needs IDF >= 5.5, so on 5.4.0 pin
both host and slave to the same 2.x version:

```bash
idf.py add-dependency "espressif/esp_hosted^2.12.12"
```

**Building the slave firmware** (for OTA staging or a manual flash):

```bash
idf.py create-project-from-example "espressif/esp_hosted^2.12.12:slave"
```

```bash
cd slave && idf.py set-target esp32c6 && idf.py build
```

That produces `network_adapter.bin` (~1.16 MB). For the OTA route, drop it in
the host app's `components/ota_partition/slave_fw_bin/` and set
`CONFIG_OTA_METHOD_PARTITION=y` — it then streams to the C6 over the existing
SDIO link with no wires. The slave image never appears in `flasher_args.json`;
`idf.py flash` writes it through a custom CMake target, and its absence from
`flash_files` is not a problem.

**Manual C6 flash over header H4**, the recovery path. There is no EN pin on
that pad, so hold `C6_IO9` low across a power cycle to enter download mode, and
tell esptool not to attempt its own reset. First park the P4 in its bootloader
so it does not drive SDIO during the flash:

```bash
esptool -p COM3 --before default_reset --after no_reset run
```

```bash
esptool --chip esp32c6 -p <ADAPTER_PORT> -b 460800 --before no_reset --after no_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin 0x10000 network_adapter.bin
```

`<ADAPTER_PORT>` is the UART adapter's port, **not COM3** — COM3 is the CH343 to
the P4 and the C6's UART does not appear on it.

**Connecting to the router.** Credentials go in a gitignored
`wifi_credentials.h` written from the `.example` beside it:

```bash
printf '#pragma once\n#define WIFI_STA_SSID     "your-ssid"\n#define WIFI_STA_PASSWORD "your-psk"\n' > components/imx_wifi/include/wifi_credentials.h
```

```bash
git check-ignore -v components/imx_wifi/include/wifi_credentials.h
```

Run that second command before your first build. It prints the ignore rule that
matches, and silence means the file is **not** ignored and a PSK is one `git add`
away from being permanent.

**Then `idf.py fullclean`.** The header is included behind `__has_include`, so
ninja has no dependency on a file that did not exist at the last build, and will
happily reuse an object compiled without it — the firmware then reports "no
credentials" with the header sitting right there:

```bash
idf.py fullclean && python ../../tools/capture.py --flash
```

**Proving the connection.** `examples/c6_wifi_sta` associates, takes a DHCP
lease and opens a TCP connection to a host on the internet:

```bash
python tools/capture.py --flash --project examples/c6_wifi_sta --seconds 45 --out c6log_sta
```

**Reading the boot log afterwards.** The captured log contains binary, so grep
needs `-a` and a NUL strip:

```bash
tr -d '\000' < c6log_sta/log.txt | grep -a -in "mismatch\|co-proc\|slave\|capabilities\|W ("
```

No `Version mismatch` line means host and slave agree.

---

## Post-mortem and instrumentation

**Decoding a RISC-V panic.** There is no textual backtrace, only a register dump
and a stack dump — but the addresses in it resolve:

```bash
riscv32-esp-elf-addr2line -pfiaC -e build/<project>.elf 0x4ff00d08 0x4ff0a13a 0x4ff0e934
```

Pull every plausible code address out of the `Stack memory:` block and decode the
batch. Frames may be stale, but a coherent chain (for example
`vTaskStartScheduler` → `prvCreateIdleTasks` → the asserting function) is the
real one, and that chain is what identified an out-of-internal-RAM condition
whose only symptom was a FreeRTOS assert before `app_main`.

**Finding which component calls a symbol.** Faster than reading source, and it
settled who was really creating a task with a bad stack:

```bash
NM=~/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-nm
for a in $(find build/esp-idf -name "*.a"); do if "$NM" -u "$a" 2>/dev/null | grep -q "xTaskCreateStaticPinnedToCore"; then echo "== $a"; fi; done
```

```bash
riscv32-esp-elf-nm -u build/esp-idf/espressif__esp_hosted/libespressif__esp_hosted.a | grep xTaskCreate
```

**Getting a closed prebuilt library to talk.** `esp_ipa` ships as a `.a` with its
`ESP_LOGD` strings intact, so a runtime per-tag level is enough — its
`LOG_LOCAL_LEVEL` was fixed when *its* vendor compiled it. Do **not** raise
`CONFIG_LOG_MAXIMUM_LEVEL`; it does nothing for a prebuilt lib and turns
`ESP_LOGD` on firmware-wide.

```c
esp_log_level_set("esp_ipa_af", ESP_LOG_DEBUG);
```

To look inside one — note `strings` is not installed here, so use a Python regex
over the bytes:

```bash
riscv32-esp-elf-ar x libesp_ipa.a
```

```bash
riscv32-esp-elf-objdump -d esp_ipa_af.c.obj | less
```

**Verifying struct layout against a binary** rather than arguing about it.
Compile a probe with `offsetof`/`sizeof` as `const unsigned` globals, using the
project's real flags from `build/compile_commands.json`, then read the values
out of `.srodata`:

```bash
riscv32-esp-elf-objdump -s -j .srodata probe.o
```

This is how the 196-vs-596 `af_stats` offset mismatch was proved.
