# Changelog

All notable changes to `esp_cam_sensor_imx` are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions
follow [semantic versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-09-06

### Added

- **Four more IMX708 modes**, so the table now runs largest to smallest:
  1920×1080 (0), 1280×720 (1), 1024×768 (2), 800×600 (3), 640×480 (4). All five
  are centred digital crops of the same 2×2-binned readout the 1920×1080 mode
  already used: identical analog window, PLL, binning, Bayer phase and
  line/frame timing. A smaller mode buys CSI bandwidth, PSRAM and encode time,
  and costs field of view.

  It does not make the *sensor* faster — every mode reads out at 28 fps, with
  the same exposure range — but it does recover frames the pipeline drops at
  full size. Measured with `imx708_video`: 1920×1080 records 218 frames in
  7978 ms (27.3 fps) where 1280×720, 1024×768, 800×600 and 640×480 each record
  224 in 7995 ms (28.0 fps). None dropped an encode in any mode.

  Nothing is scaled, and cannot be (see below), so 640×480 is a
  28%-of-the-width window on the middle of the scene rather than the scene
  shrunk. The three 4:3 modes reframe rather than narrow evenly: 1024×768 sees
  less across than 1280×720 and *more* up and down, 59% of the field against
  56%, on fewer pixels — which is the reason to choose it over mode 1 for a
  tall subject, not a consolation.

  On macroblock alignment the **width** is the half that matters, because it
  cannot be trimmed: the encoder takes its line stride from the width it is
  given, so encoding an N-wide buffer as anything narrower shears the picture
  diagonally. Every width here is a whole 16. Heights divide too, except
  1920×1080 (67.5 rows) and 800×600 (37.5), which take the height trim the
  video example already implements — the encoder is told 1072 and 592 and reads
  a prefix of the buffer. 1024×768 is 64×48 exactly, the only 4:3 size in reach
  needing nothing at all.

  Verified on hardware with `imx708_snapshot`: `bytesused` is exactly 1843200,
  1572864, 960000 and 614400, CRCs match, autofocus converges in all of them.
  Geometry was checked rather than eyeballed — locating each smaller frame in the
  1920×1080 frame by cross-correlation puts 1280×720 at (320, 180) and 640×480
  at (640, 300), the centred positions the crop offsets predict, to within the
  ±1 px the correlation peak can resolve. Correlation falls from 0.98 at the
  peak to 0.70 when displaced 128 px, so the match is specific.

  `imx708_video` records all five too, without a change to that example: it
  takes its geometry from `VIDIOC_G_FMT` and its height trim from the height it
  is handed, so 1920×1080 encodes as 1920×1072 and 800×600 as 800×592 while the
  other three need no trim at all. 0 failed encodes in every mode.
- **`CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT`** to choose the start-up mode,
  matching the IMX219 option. Defaults to 0, the existing 1920×1080.
- **`imx708_format_count()`, `imx708_format_by_index()` and
  `imx708_format_by_size()`** in `imx708.h`, so an application can change mode
  at run time. esp_video can already do it — `VIDIOC_S_SENSOR_FMT` takes an
  `esp_cam_sensor_format_t` and resizes the stream buffers around it — but it
  has no way to enumerate one: `VIDIOC_ENUM_FRAMESIZES` only reports the mode
  in use, and an application that let esp_video auto-detect the camera never
  holds the sensor handle. Call before `REQBUFS`/`STREAMON`, then renegotiate
  the pixel format with `VIDIOC_S_FMT`.

- **Recorded that the IMX708 cannot downscale**, in `imx708_regs.h` and the
  README. Every byte of the CCS scaling block at `0x0400..0x0407` was written
  with its own value XOR 0x02 and read back unchanged, while `0x0408..0x040F` —
  the digital crop immediately after it — took every write. The block reports
  the CCS defaults (`scaling_mode` 0, `scale_m` 16, `scale_n` 16, i.e. "1:1,
  not scaling") and is hardwired to them.

  The failure is quiet, which is the part worth knowing. The output-size
  registers *do* take, so a 2304×1296 crop asked to emit 1152×648 emits
  1152×648 — the top-left corner of the field at full sampling, in a frame that
  is the right size, correctly formed and perfectly sharp, with a field of view
  several times narrower than the mode claims. Cross-correlating such a frame
  against a 1920×1080 one of the same scene puts the match where a top-left
  crop predicts (NCC +0.13, 54× the search median, down to noise within 16 px)
  and finds nothing where a halved full field would be (+0.02).

  So resolution below the crop size has to come from binning, from moving the
  analog readout window, or from the P4's ISP. Two scaled modes and a
  capability probe were written to establish this and then removed rather than
  shipped: modes that silently deliver a corner of the frame are worse than no
  modes. Nothing in the driver drives the scaling block, and its registers are
  deliberately left undefined so the addresses do not invite a retry — the
  write-up carries them instead.

  Worth keeping if a later revision is ever tested: reading the block cannot
  answer the question. CCS fixes `scale_n` at 16, so a part that implements the
  scaler reports 16 and so does this one. Only a write-and-restore separates
  "implemented" from "present but hardwired".

- **Resolution can now be changed on a running board**, in `imx708_snapshot`:
  `MODE_CONSOLE` (default on) prints a `MODESEL> ` prompt and captures whichever
  mode you type — `0`–`4`, `q` to finish, one keystroke per command — and
  `MODE_CYCLE` walks the whole table off a single boot.

  Both work by cycling the video stack around the switch: `close()` →
  `esp_video_deinit()` → `esp_video_init()` → reopen → select → capture. That
  puts the switch back into the legal window before the first `VIDIOC_STREAMON`,
  so each mode gets an ISP/IPA pipeline built for its own geometry instead of
  inheriting the previous one's. `esp_video_isp_pipeline_init()` being private
  does not matter; deinit/init reaches it from further out. No reboot, and the
  cycle itself costs ~120 ms — what takes the time is 3A re-converging.

  Measured: five modes back to back off one boot, every CRC good, free heap and
  free PSRAM byte-identical after all five cycles, the DW9807 re-detected and
  autofocus re-converging to 646/641/641/641/636 across five independent
  searches.

  `tools/capture.py` drives it with `--interactive` (forwards your keystrokes,
  echoes the board, steps over binary payloads so images still extract) or
  `--keys "4,2,0,q"` (scripted, one keystroke per prompt).

- **Both IMX708 examples now select their mode through the driver API**, not
  only through Kconfig: `CAPTURE_MODE_INDEX` in `imx708_snapshot` and
  `VIDEO_MODE_INDEX` in `imx708_video`, `-1` by default (keep the build-time
  mode) and `0..4` to switch before any buffer is allocated, via
  `imx708_format_by_index()` and `VIDIOC_S_SENSOR_FMT`.

  These are `#define`s, so changing one still means a rebuild and a flash —
  they are a worked example of the call. For changing resolution on a running
  board, see `MODE_CONSOLE` below.

  **The switch must happen before the first `VIDIOC_STREAMON`.** Beyond the
  obvious — buffers are sized for the format in force when they were requested
  — the ISP/IPA pipeline is created once by `esp_video_init()` and reads the
  sensor geometry then. Nothing re-initialises it afterwards, so a mode change
  *after* streaming has begun moves the geometry and strands 3A: auto-exposure,
  auto-white-balance and autofocus all stop, and the frame comes back correctly
  sized, correctly formed and nearly black with the `esp_ipa_af` log lines
  absent. That killed the naive form of a "capture every mode in one run"
  sweep, which produced one good frame and four dark ones. Cycling the video
  stack per mode is the form that works — see `MODE_CYCLE` below.

  Verified: `imx708_snapshot` built for mode 0 and switched to 800×600 gives a
  correctly exposed, in-focus frame (`bytesused=960000`, autofocus 512 → 641);
  `imx708_video` switched to 1024×768 records 224 frames, 0 failed, 28.0 fps.

### Fixed

- **`imx708_snapshot` overran its staging buffer when a run raised its
  resolution.** `stage_frame()` allocated the PSRAM staging copy once, at the
  first frame's size, then `memcpy`'d later frames into it. Harmless while a run
  captured a single size or descended through the table; capturing 640×480 and
  then 1024×768 overran it by 958464 bytes and panicked the board with an
  instruction fetch from `0x29282928` — an address made of image bytes. The
  buffer now grows when a larger frame arrives.

- **`imx708.h` could not be included by an application.** It pulled in
  `imx708_regs.h`, which lives in `private_include/` and is therefore invisible
  outside the component — so any consumer got `fatal error: imx708_regs.h: No
  such file or directory`. Present since 0.2.0 and latent: the component builds
  fine, and no example included the public header until the run-time mode API
  gave one a reason to. The include was unused; `imx708.h` defines
  `IMX708_SENSOR_NAME` and `IMX708_SCCB_ADDR` itself. `imx219.h` never had it.

### Changed

- The IMX708 mode tables are spliced from two shared register halves with a
  per-mode crop block between them, in the order the single 1920×1080 table was
  written in when it was verified on hardware. The expanded 1920×1080 sequence
  is byte-for-byte and order-for-order what it was; the point is that a timing
  fix now lands in one place instead of five.
- The IMX708 exposure ceiling is read from the current mode's `isp_info.vts`
  rather than a compile-time constant, as the IMX219 already did. No behaviour
  change today — every mode shares a VTS — but a mode that changed frame rate
  would otherwise leave the AE clamped to a stale limit, and that failure is
  silent: an exposure past `frame_length` does not take effect, so the AE loop
  sees no response to its own request.

### Upgrading

- **Mode indices below are new in this release, except 0.** 0.2.0 shipped a
  single 1920×1080 mode, so nothing outside this repository has ever named
  index 1 or above. New modes were therefore *inserted* rather than appended,
  to keep the table ordered largest to smallest — 1024×768 at 2, then 800×600
  at 3, moving 640×480 twice. That was free only because none of it had
  shipped, and it stops being free the moment this release does: a mode's
  number is part of the driver's interface, `imx708_format_by_index()` hands it
  out, and renumbering later would silently change what an existing build comes
  up in, with nothing to warn whoever wrote that number down. From here modes
  are appended, and the ordering is abandoned before the numbering is.

- **Move dependency pins from `^0.2.0` to `^0.3.0`.** A caret range on `0.x`
  covers one minor line only, so a project left on `^0.2.0` silently keeps
  resolving the old version rather than failing.

- **Run `idf.py reconfigure` in any existing build tree** to pick up
  `CAMERA_IMX708_MIPI_IF_FORMAT_INDEX_DEFAULT`. As with the IMX219 option, the
  driver carries an `#ifndef` fallback so a stale `sdkconfig` keeps mode 0
  rather than failing to build.

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
- **The ISP crop is unavailable on ESP-IDF v5.4.0**, which is why the
  16-alignment above is done at the sensor's readout window rather than in the
  ISP. `csi_set_selection` and its ops-table entry both sit behind
  `ESP_VIDEO_ISP_DEVICE_CROP`, which esp_video defines only under
  `CONFIG_SOC_ISP_CROP_SUPPORTED` — a symbol this IDF version does not define at
  all, on any P4 revision. So `ops->set_selection` is NULL and
  `VIDIOC_S_SELECTION` returns `ESP_ERR_NOT_SUPPORTED`.
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
