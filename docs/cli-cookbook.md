# CLI cookbook

The commands that actually got used bringing this board up — flashing,
capturing stills over three different transports, recording video, and getting
the ESP32-C6 radio onto a router. Kept because most of them are not the
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
- [Stills: SD card](#stills-sd-card)
- [Stills: USB serial](#stills-usb-serial)
- [Stills: WiFi](#stills-wifi)
- [Video over USB](#video-over-usb)
- [ESP32-C6 radio](#esp32-c6-radio)
- [Post-mortem and instrumentation](#post-mortem-and-instrumentation)

---

## Environment

**Activating ESP-IDF.** `export.ps1` fails here with "ESP-IDF Python virtual
environment not found" even though the venv exists. This works, and has to be
one PowerShell invocation because shell state does not persist between calls:

```bash
$env:IDF_PATH="$HOME\esp\v5.4\esp-idf"; $env:IDF_TOOLS_PATH="$HOME\.espressif"; $exp = & "$HOME\.espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe" "$env:IDF_PATH\tools\activate.py" --export 2>$null | Select-Object -Last 1; . $exp; Set-Location "$HOME\source\repos\imx708\examples\imx708_snapshot"; idf.py build
```

`activate.py --export` writes a temp `.ps1` and prints its path as the **last**
line of stdout, which is what gets dot-sourced. Piping stderr away matters
because PowerShell 5.1 wraps native stderr in ErrorRecords and the pipeline then
looks like a failure.

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

## Stills: SD card

The original path, still available behind `IMAGE_OUT_SD` in
`examples/imx708_snapshot`. Superseded by serial and then WiFi, but kept
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
python tools/capture.py --flash --project examples/imx708_snapshot --out shot
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

`examples/imx708_wifi_snapshot` serves the camera over HTTP. Flash it the same
way; the run ends by printing the URL:

```bash
python tools/capture.py --flash --project examples/imx708_wifi_snapshot --seconds 45 --out c6log_wifisnap
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

`examples/imx708_video` records ~8 s of 1080p H.264 into PSRAM, then ships the
clip in one framed payload. Allow generous time — a couple of MB has to move
after the recording finishes:

```bash
python tools/capture.py --flash --project examples/imx708_video --out clip
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
