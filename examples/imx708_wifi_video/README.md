# imx708_wifi_video

Live 1080p H.264 from the IMX708, over WiFi, in a browser tab. No SD card, no
recording step, no player to install.

This is [`imx708_video`](../../components/esp_cam_sensor_imx/examples/imx708_video/) and
[`imx708_wifi_snapshot`](../imx708_wifi_snapshot/) joined up. The first proved
the hardware encoder; the second proved the radio and measured what it can
carry. What is new here is everything between them: an encoded-frame ring that
lets the camera and the network run at their own speeds, and a fragmented-MP4
muxer on the board so a `<video>` element can play a stream that has no end.

| Where | Endpoint | What it is |
| --- | --- | --- |
| `:81` | `GET /` | the player |
| `:81` | `GET /info` | geometry, frame rate, codec string, as JSON |
| `:81` | `GET /stream.mp4` | live fragmented MP4 — what the browser plays |
| `:81` | `GET /stream.h264` | live Annex-B elementary stream — what `ffplay` plays |
| `:80` | `GET /stats` | encoder and link counters, as text |
| `:80` | `GET /bench` | 8 MB of pattern from PSRAM, no camera in the loop |
| `:80` | `GET /rate?kbit=N` | retarget rate control, live |
| `:80` | `GET /skip?n=N` | encode 1 frame in N; `n=0` stops the encoder |
| `:80` | `GET /` | redirects to `:81/` |

Credentials come from `components/imx_wifi/include/wifi_credentials.h`, which is
gitignored. See that component's `.example` template.

## Run it

```bash
python tools/capture.py --flash --project examples/imx708_wifi_video --out boot
```

That flashes, resets, and prints the boot log until the board says it is
serving. Then open the address it gives you:

```
I imx708_wifi_video: watch it at http://192.168.0.164:81/
I imx708_wifi_video:   ffplay http://192.168.0.164:81/stream.h264
I imx708_wifi_video:   instruments: http://192.168.0.164/stats  /bench  /skip?n=
```

The stream starts as soon as the board boots and keeps running with nobody
watching, so AE, AWB and autofocus are already converged when you connect. The
first picture appears within one GOP — one second by default — because a
decoder can only join at a keyframe.

To record instead of watch, point ffmpeg at the raw stream; it needs no help
from this repo, because Annex-B is exactly what it expects on the wire:

```bash
wsl ffmpeg -i http://192.168.0.164:81/stream.h264 -t 30 -c copy /mnt/c/tmp/clip.mp4
```

## Why fragmented MP4, and why the board does the muxing

The encoder emits an Annex-B elementary stream: NAL units separated by start
codes, and nothing else. That is the right thing on the wire — it carries its
own synchronisation, so it can be cut and joined anywhere — but no browser will
play it, and `<video src=...>` will not play *any* endless stream, because `src`
wants something with a length that can be seeked.

The one container a browser accepts as a live byte stream is fragmented MP4
through Media Source Extensions. fMP4 splits an ordinary MP4 in two: an init
segment (`ftyp` + `moov`) declares codec, geometry and timescale but indexes
nothing, and every frame after it is a self-contained `moof` + `mdat` carrying
its own timing. Nothing needs to be known in advance and nothing is patched
afterwards.

That last part is the whole reason [`tools/mp4.py`](../../tools/mp4.py) could not
be reused. It writes a `stco` table of absolute file offsets, which can only be
computed once the recording is complete — fine for a clip, impossible for a
stream. [`components/imx_fmp4/`](../../components/imx_fmp4/) is the same job in
C, on the sending side, and the box layouts are deliberately mirrored from that
file so the two can be read against each other.

**MJPEG was ruled out by measurement, not taste.** The snapshot example found
~260 KB per q90 JPEG against a 6–7 Mbit/s link: about 3 fps. Dropping to q60
lands near 100 KB and ~8 fps, at which point it is worth asking what MJPEG is
buying. H.264 at 3 Mbit/s is 375 KB/s for the whole 28 fps.

### Checking the container without a board

The muxer is ordinary C with no ESP-IDF in it, which means it compiles on the
host and can be pointed at a clip you already have. This is how it was checked
before anything was flashed — 217 frames from `clip/imx708.h264`, muxed and then
decoded by ffmpeg with no errors:

```bash
gcc -O2 -Icomponents/imx_fmp4/include components/imx_fmp4/test/fmp4_host_test.c components/imx_fmp4/imx_fmp4.c -o fmp4_host_test
```

```bash
./fmp4_host_test clip/imx708.h264 frag.mp4 && ffmpeg -v error -i frag.mp4 -f null -
```

Silence from ffmpeg means every frame decoded.

A container fault shows up there as `Invalid NAL unit size`, a wrong frame
count, or a duration that disagrees with the source. Doing this on the host
first is worth the twenty minutes: a muxing bug and a transport bug look
identical from the far end of a WiFi link.

## The frame ring, and why a slow viewer is dropped rather than waited for

The camera-side pipeline is one task and one loop — dequeue, encode, requeue,
publish — and it never touches a socket. Encoded frames go into a 1 MB ring in
PSRAM; the HTTP handlers only ever read from that.

The ring size is a **latency budget, not a buffer**. At 3 Mbit/s it holds about
2.8 s. A viewer that falls further behind than that has its frames overwritten,
and is then jumped forward to the newest keyframe rather than being served stale
frames or allowed to stall the encoder. `resyncs` in `/stats` counts those
jumps.

That is the difference between live video and a file transfer. A stream that
buffers in order to keep up is a stream that is no longer live, so the choice
here is always to skip forward and take the visible jolt. The timestamps carry
the gap honestly, so the picture skips while the clock stays true.

Each fragment is held back until the next frame arrives, so its duration can be
the real interval rather than a nominal one. It costs 36 ms of latency and buys
a timeline where each fragment's decode time is exactly the sum of the durations
before it. MSE is unforgiving of the alternative: a nominal duration against a
true timestamp leaves a few milliseconds of gap or overlap on *every* frame, and
the player either stalls in the gaps or drifts out of them.

## Two servers, and which one gets the page

`esp_http_server` runs one task per instance and serves one request at a time in
it. A live stream is a handler that never returns, so it owns its whole server
for as long as someone is watching — no `/stats` during a stream, which is the
one moment the counters are worth reading.

Splitting into two instances is the obvious fix. **Which side the page goes on
is the part that is not obvious**, and getting it wrong cost a debugging round
here. Serving the page from the control port makes the stream cross-origin; an
`Access-Control-Allow-Origin` header covers that for an ordinary browser, but an
embedded browser refused the fetch outright with `ERR_BLOCKED_BY_CLIENT` and the
page sat on `starting...` with nothing whatever wrong on the board. Serving the
page from the *stream* port makes the fetch same-origin and the question stops
existing.

So port 81 is the viewer — page, `/info` and both streams, one origin, every URL
relative. Port 80 is the instruments, answerable at any moment because nothing
there blocks. The page links to them as plain navigations, which no origin rule
touches, and `http://<ip>/` redirects to `:81/` so the obvious address is not a
dead end.

**Two instances cannot share a control port.** `httpd` uses a UDP socket to wake
its own task, it defaults to 32768 for every instance, and the second
`httpd_start()` fails on the bind with nothing useful to say. Set `ctrl_port`
explicitly on both.

**One viewer at a time.** A second connection to `:81` waits for the first to
finish, which for a live stream is never. One 1080p viewer is more than this
link carries anyway.

## Measured, 2026-08-30

### The encoder

| | |
| --- | --- |
| Encode, camera stopped, one frame fed repeatedly | **32.5 ms** |
| Encode, camera streaming | **36.2–36.5 ms** |
| Per-frame budget at 28 fps | 35.7 ms |
| Delivered pipeline rate | **27.2 fps** |

The encoder needs essentially the whole frame interval for 1920×1072, which is
why the pipeline runs at 27.2 fps rather than 28. The 4 ms difference between
the two rows is the ISP writing the next frame into PSRAM while the encoder
reads the current one.

**Encode time does not depend on how many bits come out.** Sweeping rate control
from 1 to 8 Mbit/s moved the output from 4.8 KB to 34.7 KB per frame and left
the time flat at 36.2–36.5 ms. It is a fixed-throughput pipeline, so the levers
that matter are resolution and frame rate, not quality.

> **A note on `imx708_video`'s README.** It quotes `encode 9204 us mean` in its
> "Reading the log" section, and that number sent this work looking for a
> regression that does not exist. Both preserved runs from that example —
> `clip/log.txt` and `clip2/log.txt` — say `encode 36290 us` and `36287 us` at
> 27.2 fps, which is exactly what this example measures. The 9204 figure matches
> no run in the tree; that README block has been corrected.

### The link, and what actually arrives

`/bench`, no viewer: **6.65, 8.51, 9.02 Mbit/s**.

Delivered stream, 15 s per row, `/stream.h264`, counted by `ffprobe`:

| `/rate` | encoded | delivered | delivered rate | resyncs |
| --- | --- | --- | --- | --- |
| 4000 kbit/s | 408 | 381 | 25.4 fps, 3.63 Mbit/s | 0 |
| 3000 kbit/s | 411 | 394 | **26.2 fps, 2.85 Mbit/s** | 0 |
| 2000 kbit/s | 413 | 416 | 27.7 fps, 1.96 Mbit/s | 0 |
| 1000 kbit/s | 414 | 429 | 28.6 fps, 1.06 Mbit/s | 0 |

Everything the encoder produces reaches the viewer, at every setting, with no
resynchronisation at all. Counts above the encoded figure are the ring's backlog
draining in the first moments of the connection, not frames appearing from
nowhere. `send_us_mean` settles around 17 ms against a 36 ms frame interval, so
there is roughly half the interval spare.

In a browser: 2.99 Mbit/s, **0.6 s behind live**, 1920×1072.

**Throughput on this link is not a constant, and it is worth knowing how badly.**
Over one afternoon `/bench` ranged from 15.4 Mbit/s to 0.01 Mbit/s. The
*unchanged* `imx708_wifi_snapshot` binary — the one that measured 7.2 Mbit/s the
day before — was reflashed during the worst of it and measured **0.05 Mbit/s**,
same board, same room, same −40 dBm. Pings through the same collapse came back
**0% lost, 4 ms average**: healthy for small packets, hopeless for bulk, which is
bursty 2.4 GHz interference rather than congestion. Run `/bench` before
concluding anything from a slow stream.

### Two bugs that both looked exactly like a slow network

Neither was, and both were found only because `/bench` and `/stats` were sitting
on a port that keeps answering while a stream runs.

**1. Nagle's algorithm, against a paced sender.** A frame is written, then the
sender goes quiet for 36 ms. Nagle holds the sub-MSS tail of each frame until
the previous write is acknowledged, and the receiver — having nothing to say —
delays that ACK. The two interlock and the stream advances about once per
delayed-ACK timer. Measured: `send_us_mean` **40 902 µs**, 41 ms blocked in a
socket for 9.5 KB, 2–5 fps arriving while the encoder produced 27.
`setsockopt(TCP_NODELAY)` on the stream socket took it to 10.6 fps.

`/bench` never showed this, and that is the instructive part: it always has more
data queued, so its writes are never sub-MSS and Nagle never engages. **A bulk
benchmark cannot stand in for a paced one.**

**2. `pdMS_TO_TICKS(5)` is zero at 100 Hz.** The frame-wait loop was written as
"1000 iterations of `vTaskDelay(pdMS_TO_TICKS(5))`", intending a five-second
budget. `pdMS_TO_TICKS` is `(ms * CONFIG_FREERTOS_HZ) / 1000`, so at IDF's
default tick rate that is 5 × 100 / 1000 = **0**, and `vTaskDelay(0)` yields
without sleeping. The loop burned its whole budget in **ten milliseconds** and
dropped each viewer about as fast as it accepted one.

From the client this is indistinguishable from a slow link: a handful of frames
arrive, the connection closes, and every rate computed over the intended window
is wrong by the ratio of the two — which is how a stream really running at 27 fps
was measured at 4. The board's own log said it plainly the moment anyone looked:

```
I h264 viewer joined at frame 4338 (1 watching)
W no frames for 5000 ms - dropping the h264 viewer
I h264 viewer left after 6 frames, 45 KB in 15 ms -> 24.7 Mbit/s
```

Five thousand milliseconds of waiting, in fifteen. `stream_next()` now takes an
`esp_timer_get_time()` deadline and sleeps `vTaskDelay(1)` — one tick, whatever
a tick happens to be. **Never express a short delay in milliseconds without
checking what it rounds to.**

(This example sets `CONFIG_FREERTOS_HZ=1000` anyway, as every WiFi example here
does — esp_hosted warns about bus jitter at 100 Hz. The bug was a stale
`sdkconfig` left at 100 by an unrelated experiment, which is exactly how it will
happen to you.)

### What frame-skip does, and what it does not

`/skip?n=` was added on the theory that the encoder, busy ~99% of the time, was
starving the radio. **That theory is wrong**, and the measurement that killed it
is worth keeping:

| `/skip` | encoder | `/bench` |
| --- | --- | --- |
| 0 (encoder stopped) | 0 fps | 8.77 Mbit/s |
| 1 | 27.3 fps | 6.62 Mbit/s |
| 2 | 14.0 fps | 7.83 Mbit/s |
| 4 | 6.9 fps | 7.23 Mbit/s |

No relationship: switching the encoder off entirely buys about as much as the
link's own run-to-run spread, and the row where it runs flat out is not the
slowest. Sampling `frames_encoded` over the same ten-second window while idle,
while streaming and while benching gives 27.5, 27.0 and 27.5 fps — the encoder
does not slow the radio down, and the radio does not slow the encoder down.

The other thing frame-skip does not do is **save bandwidth**. Rate control's
budget is bits per second, so halving the frame rate does not halve the bitrate;
it gives each remaining frame twice as many bits. This is the obvious lever to
reach for when a link is too slow and it is the wrong one. `/rate?kbit=` is the
bandwidth knob. `/skip` trades frame rate for per-frame quality, and buys chip
time.

## What you can change

All at the top of `main/imx708_wifi_video_main.c`.

| Define | Default | Notes |
| ------ | ------- | ----- |
| `VIDEO_BITRATE` | 3000000 | The bandwidth knob. Live, `/rate?kbit=` changes it without breaking the stream |
| `VIDEO_FPS` | 28 | The sensor's only mode |
| `VIDEO_GOP` | 28 | Also the join latency: a viewer waits up to one GOP for its first picture |
| `VIDEO_FRAME_SKIP` | 1 | Encode 1 frame in N. Does not save bandwidth — see above |
| `VIDEO_QP_MIN/MAX` | 20 / 45 | 51 is the codec maximum |
| `RING_BYTES` | 1 MB | ~2.8 s at `VIDEO_BITRATE`; the most lateness a viewer gets before it is dropped forward |
| `ENC_OUT_BYTES` | 512 KB | Must fit the largest IDR. Also sizes three per-viewer buffers, so 1.5 MB of PSRAM per stream |
| `ENCODE_16_ALIGNED` | 1 | See "1072 lines, not 1080" in `imx708_video`'s README |
| `ENCODER_BITRATE_SWEEP` | 0 | Boot-time encoder diagnostics: free-run timing and the bitrate sweep above |
| `CTRL_PORT` / `STREAM_PORT` | 80 / 81 | |

## Reading `/stats`

```
frames_encoded=13373  idr=477  encode_failed=0
fps=26.0  fps_recent=27.4
encode_us_mean=36387  encode_us_worst=51227  encode_us_budget=35714
dqbuf_us_mean=1773  dqbuf_us_worst=44163
send_us_mean=16822  send_us_worst=2413830  frames_sent=8241
encoded_kbit_s=2339  target_kbit_s=3000
viewers=0  resyncs=5  frame_skip=1
```

- **`dqbuf_us_mean`** is who the bottleneck is. Near zero means a frame was
  always already waiting, so the encoder paces the loop and frames are being
  dropped between camera and encoder — which at skip 1 is the normal state here,
  and why 27.2 fps rather than 28. A large value means the camera paces it and
  the encoder has headroom, as it does at any `/skip` above 1.
- **`fps_recent`** covers only the interval since the previous `/stats`. The
  cumulative `fps` cannot show the effect of anything changed while the board is
  running, because it stays dominated by however long the old setting had
  already been averaging.
- **`send_us_mean`** is how long the socket took to accept one frame. Near the
  frame interval means the link is the limit; near zero means the pipeline is.
- **`resyncs`** climbing steadily means the link cannot carry the current
  bitrate and viewers are being jumped forward. Lower `/rate?kbit=`.
- **`encode_failed`** is the encoder rejecting frames. `ESP_H264_ERR_OVERFLOW`
  means `ENC_OUT_BYTES` is too small for an IDR of this scene.

## If there is no picture

- **The page says `waiting for the first keyframe...`** — normal for the first
  second after boot; the init segment cannot be built until the encoder has
  emitted an IDR, because that is the only place the SPS and PPS appear.
- **The page says `this browser will not play ...`** — no MSE, which on iOS
  means Safari. Use `ffplay <the URL it prints>` instead.
- **The page sits on `starting...`** — the `/stream.mp4` fetch was refused.
  Check the browser console; `ERR_BLOCKED_BY_CLIENT` is the client's policy, not
  the board. Confirm the board is fine with `curl -s http://<ip>/stats`.
- **`/stream.h264` works and `/stream.mp4` does not** — that is the muxer, and
  the split is deliberate: the raw endpoint has no container in it at all, so it
  answers whether the problem is the camera, the encoder and the link, or only
  the MP4 boxes.
- **Very low frame rate with `resyncs` climbing** — the link genuinely cannot
  carry the current bitrate. Run `/bench`, then lower `/rate?kbit=`.
- **Very low frame rate with `resyncs` at zero** — not the link. Frames are not
  reaching the sender, or the connection is being dropped and re-made; watch the
  board's log for `viewer left after N frames`, which reports how long the
  connection actually lasted. Both of the bugs above presented this way.
- **Colours wrong** — ISP tuning, shared with every other example here. The CCM
  is an uncalibrated seed and the module is NoIR, so infrared contaminates all
  three channels.

## Traps inherited from the snapshot example

Both still apply and are written up in full in
[its README](../imx708_wifi_snapshot/README.md):

1. **The board asserts in `prvCreateIdleTasks` before `app_main`** if
   esp_hosted's SDIO mempool is left at its default queue sizes. Nothing in the
   log connects it to WiFi. Fixed here by `CONFIG_ESP_HOSTED_SDIO_*_Q_SIZE=10`.
2. **`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y` fixes trap 1 and breaks bulk
   transfers.** Keep the transport's DMA buffers in internal RAM.

And one of this example's own: the `ISP_AWB_WINDOW_X_NUM/_Y_NUM=5` block in
`CMakeLists.txt` is required, between `project.cmake` and `project()`, or
autofocus silently reads unrelated heap memory.
