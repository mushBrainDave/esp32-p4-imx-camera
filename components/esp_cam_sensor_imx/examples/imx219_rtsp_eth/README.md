# IMX219 Ethernet RTSP prototype

Continuous H.264 camera for Guition JC-ESP32P4-M3-DEV, P4 v1.3,
16 MB flash and 32 MB hex PSRAM. Native RMII Ethernet uses the tested carrier
routing and generic PHY driver (detected PHY address 1).

Seven profiles are installed. `ultra1` and `ultra5` prioritize native full-scene
detail and light compression. Query `/api/v1/profiles` for the active selection.

| ID | Output | Scene coverage | Target bitrate / GOP |
| --- | --- | --- | --- |
| `ultra1` | 1664×1248 at 1 fps, exact 4:3 | Native 1640×1232 full scene, tiny borders | Fixed QP 8 / GOP 1 |
| `ultra5` | 1664×1248 at 5 fps, exact 4:3 | Same native content and borders as ultra1 | Fixed QP 8 / GOP 1 |
| `fullview30` | 832×624 at 30 fps, exact 4:3 | Full sensor scene, resized | 2 Mbit/s / 30 |
| `fullview15` | 1280×960 at 15 fps, exact 4:3 | Full sensor scene, resized | 4 Mbit/s / 15 |
| `detail15` | 1632×1232 at 15 fps, near 4:3 | Almost full sensor; slight side trim | 4 Mbit/s / 15 |
| `720p30` | 1280×720 at 30 fps, 16:9 | Centered 2560×1440 sensor crop | 3 Mbit/s / 30 |
| `balanced30` | 1536×864 at 30 fps, 16:9 | Centered 3072×1728 sensor crop | 4 Mbit/s / 30 |

Full-view profiles capture mode 0 (1640×1232 from the complete 3280×2464 readout)
and sample it down with nearest-neighbour 2×2 luma/chroma groups across two CPU
cores. Endpoint mapping retains every edge of the scene; there are no black
borders. Native aspect ratio differs from exact 4:3 by about 0.16%. The output
has less fine detail because it is sampled down. Fullview30 uses a 30 fps sensor cadence. Fullview15 now uses a real 15 fps
sensor mode, giving longer exposure headroom without discarding alternate frames.

The original three profiles send packed ISP YUV420 directly to the encoder.
All output dimensions are macroblock aligned. The existing five profiles keep
their geometry/compression settings. Detail15 and fullview15 now use real 15 fps
sensor modes; ultra1 and ultra5 use dedicated 1 fps and 5 fps modes. A larger
full-scene output at 30 fps failed its hardware performance test, so fullview15
retains an explicit 15 fps budget.

## Build and use

Use the pinned ESP-IDF v6.0.2 build instructions in the
[repository README](../../../../README.md). The application uses esp_video 2.5.0,
esp_h264 1.4.1 and esp_media_protocols 1.0.3, with the pinned pre-v3 IPA fix.
The serial console runs at 460800 baud. The factory app partition is 4 MiB;
OTA update and automatic rollback slots are not included.

Replace CAMERA_IP with the board's DHCP address:

```bash
ffplay -rtsp_transport tcp rtsp://CAMERA_IP:8554/live
curl http://CAMERA_IP/api/v1/status
curl http://CAMERA_IP/api/v1/profiles
curl -X POST 'http://CAMERA_IP/api/v1/profile?id=fullview30'
```

The camera admits one upstream RTSP reader. Use a LAN relay for multiple viewers.
[Camera operations](../../../../CAMERA_OPERATIONS.md) documents all profiles,
exposure controls, switching behavior and recovery diagnostics.

## Ownership and recovery

Capture/encoding run independently of the network consumer. A capture buffer is
returned immediately after synchronous hardware encoding. Three owned PSRAM
slots hold at most 2 MiB per encoded access unit; the RTSP callback copies into
the protocol library's buffer before releasing a slot. No library-owned pointers
reference camera capture buffers. Queue overflow or a frame older than the
profile age limit requests session restart through a supervisor: 250 ms for
15/30 fps, 450 ms for ultra5, and 1250 ms for ultra1. New sessions wait for SPS/PPS +
IDR. Accepted sockets have a 200 ms send timeout. Link/IP changes restart RTSP
sessions. Queue limits do not establish end-to-end latency; that is measured
separately during qualification.

The socket adapter in `rtsp_socket.c` fixes a pinned-library interoperability
problem: `Content-Length: 192 ` is rejected by MediaMTX. It emits the equivalent
same-length decimal `Content-Length: 0192` instead. RTP bytes are untouched.
The adapter handles partial writes of the corrected control response and closes
the socket on failure. It does not patch managed components or the closed library.
Remove/retest this workaround when upgrading the RTSP component.

The pinned library also omits H.264 parameter sets from its SDP. The bridge now
caches the first encoder SPS/PPS for each profile and waits for that metadata
before exposing RTSP. The control adapter adds `profile-level-id` and
`sprop-parameter-sets` to DESCRIBE and recalculates the actual Content-Length.
This lets an immediate relay viewer initialize its decoder before receiving
in-band parameters. The cache is cleared on every profile start; codec settings,
RTP bytes, frame timestamps and existing IDR/queue handling are unchanged.
`h264_sdp_ready` in status reports whether the current profile's cache is valid.

The pinned server and TCP transport both close the listening descriptor during
shutdown. The SAL listen/close adapter reserves that descriptor until server
stop returns, then releases it once. Accepted clients retain their existing
retirement barrier. This prevents the duplicate listener close from reaching an
HTTP socket allocated during teardown. The listener is excluded from the
pre-stop client-retirement wait because it closes within server stop itself.

SDP scratch buffers run only on the control worker, outside the video timer's
small stack. Transformed replies preserve the pinned library's input-length
return convention while send-byte counters track the expanded wire length.
The library's serial TX log precedes this adapter and can still show its original
SDP; inspect the actual client DESCRIBE response/trace when verifying parameters.

`GET /api/v1/status` also includes a `delivery` object for diagnosing slow
readers and large encoded frames. Counters reset when a profile starts and
survive session recovery, so a disconnect does not erase its cause:

- `queue_age_limit_ms`: current stale-frame ceiling. At 5 fps and below this
  includes one frame period plus 250 ms; faster modes retain 250 ms. It permits
  bounded scheduling variation without adding an intentional send delay.
- `queue_full_resets`, `queue_age_resets`, `callback_capacity_resets`: reset
  conditions; age and capacity can both be reported for one reset.
- `queue_high_water`, `queue_age_max_us`: largest queued frame count and oldest
  observed age when consuming a frame or overflowing the queue.
- `callback_gap_max_us`: spacing between callbacks that find a queued frame,
  excluding gaps between reader sessions; `callback_wait_max_us` measures waiting
  for the bridge mutex. Copy maxima measure producer and consumer copies.
- `send_calls`, `send_bytes`, `send_total_us`, `send_max_us`, `send_max_bytes`,
  `send_short_writes`, `send_failures`, `send_last_errno`: wrapped protocol-library
  sends/writes, including RTSP control traffic. Bytes are accepted by the socket,
  not acknowledged on the wire. Failed sends during normal reader teardown are
  expected; inspect the saved live timeline before diagnosing a transport error.
- `timer_calls`, `timer_total_us`, `timer_max_us`: complete protocol video-timer
  callbacks, including packetization, socket work, empty polls and cleanup.
  Their mean is not necessarily the mean time to deliver one frame.

Times are microseconds, byte counters are cumulative, and these counters do not
measure end-to-end viewer latency. Socket/timer snapshots and bridge state use
separate locks; the full status response is not an atomic cross-task snapshot.
The status handler uses a bounded 4096-byte heap response buffer. Queue capacity,
socket timeout and descriptor/timer retirement rules are unchanged. The ultra5
age ceiling was raised from 250 to 450 ms after reproducible startup age-only
resets at approximately 254 ms with two queued frames; codec quality and FPS
remain unchanged. This allows a higher worst-case queue age in ultra5 and does
not establish a measured viewer-latency bound.

The nested `packetization` object measures SAL allocator work only on the active
RTSP video-timer task. It reports allocation/free counts and microsecond totals,
requested allocation bytes (cumulative, not live heap use), maximum call times,
failures and size buckets (≤128,129–1536,>1536 bytes). Other tasks are excluded.
Callback copy/wait totals let traces distinguish those costs from allocation
and socket work. These wall times include preemption; subtracting totals leaves
an unattributed residual, not a pure packetizer CPU measurement. Sampling during
a callback can observe incomplete totals. Counters reset on profile start and
survive reader/session recovery. Wrappers preserve allocation results and errno.

## Image controls and telemetry

GET `/api/v1/image` reports mode-specific exposure/gain bounds, pending state and
sensor readback. Empty-body POST requests select automatic mode or manual
exposure_us/gain_milli. A queued request returns 202; malformed/out-of-range
requests return 400 and requests during profile switching return 409. Poll GET
until pending is false and last_error is empty. Profile changes and reboot restore
automatic exposure; only profile selection persists.

Automatic exposure respects the selected frame rate. Current approximate ceilings
are 25 ms at 30 fps, 58.3 ms at 15 fps, 192 ms in ultra5 and 992 ms in ultra1.
A faster profile can remain dark at its exposure/gain limits. Fixed QP 8 reduces
compression loss but is not lossless; ultra-mode bitrate varies with scene noise
and detail. The encoder bitrate field is not a cap for those modes.

Status includes cached exposure and output-luma telemetry. These samples are
neither frame-exact metadata nor the ISP's weighted raw exposure meter.

## Host tests

From repository root, run `python3 tools/host-tests.py`. The eight host test groups
cover scaling bounds, image request validation, frame age, H.264 SDP, RTSP framing,
socket retirement, timer shutdown and scoped allocation with ASan/UBSan.
