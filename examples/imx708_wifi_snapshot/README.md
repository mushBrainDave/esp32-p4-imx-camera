# imx708_wifi_snapshot

IMX708 stills over WiFi: camera + STA connect + an HTTP server on the board.

| Endpoint | What it is |
| --- | --- |
| `GET /` | a page with the picture on it |
| `GET /snapshot.jpg` | the newest frame, hardware-JPEG encoded (q90) |
| `GET /snapshot.raw` | the newest frame as raw RGB565, 4 147 200 bytes |
| `GET /bench` | 8 MB of pattern from PSRAM, no camera in the loop |
| `GET /stats` | timings from the last request, plus heap, as text |

Credentials come from `components/imx_wifi/include/wifi_credentials.h`, which is
gitignored. See that component's `.example` template.

## Why a still before video

Nothing had measured *throughput* over this link. `c6_wifi_sta` measured
latency — 21 ms round-trips — which says nothing about sustained MB/s through
SDIO → C6 → air. A single frame is a self-contained blob with a known length,
so it gives a byte-exact pass/fail and a throughput number in one run, and
`/bench` separates the network from the camera.

## Measured, 2026-08-30

Client is a PC on the same 2.4 GHz AP, `curl` timing the whole request.
RSSI −40 to −47 dBm, link negotiated `phy bgn` (the AP is 802.11n, so no `/ax`).

| What | Bytes | Wall time | Rate |
| --- | --- | --- | --- |
| `/bench`, 8 MB, run 1 | 8 388 608 | 9.28 s | **7.2 Mbit/s** |
| `/bench`, 8 MB, run 2 | 8 388 608 | 13.93 s | 4.8 Mbit/s |
| `/snapshot.raw`, 4 MB | 4 147 200 | 5.52 s | **6.1 Mbit/s** |
| `/snapshot.jpg`, cold connection | ~260 KB | 1.70–1.82 s | 1.2–1.4 Mbit/s |
| `/snapshot.jpg`, **warm connection** | 260 381 | **0.47 s** | **4.5 Mbit/s** |

JPEG encode is 33–81 ms for a 1920×1080 frame and is not the bottleneck.

**Connection setup dominates a single snapshot.** Two requests on one keep-alive
connection: the first took 1.70 s, the second 0.47 s — same size, same scene,
3.6× faster. Time-to-first-byte was 85–89 ms in both, so this is TCP slow start,
not the server thinking. A short transfer never reaches steady state.

**Run-to-run variance is large.** The two 8 MB runs differ by 50% with nothing
changed. Treat a single measurement here as an estimate; 2.4 GHz, one shared
channel, and a co-processor link do not give repeatable numbers.

### What this says about video

Sustained ceiling is **6–7 Mbit/s, roughly 800 KB/s**. That settles a question
that was previously open:

- **MJPEG at q90 is not viable** — ~260 KB per frame is about **3 fps** even at
  the full sustained rate. Dropping quality helps proportionally (q60 lands
  nearer 100 KB, so ~8 fps), at which point it is worth asking what MJPEG is
  buying.
- **H.264 fits comfortably.** 1080p at 4 Mbit/s sits inside the measured
  ceiling with headroom, and the P4 has the hardware encoder already used by
  `imx708_video`.
- **Whatever the codec, hold one connection open.** The warm-vs-cold result
  above is a 3.6× difference for free.

## Two traps, both worth not rediscovering

**1. The board asserted before `app_main`, and the message pointed at FreeRTOS.**

```
assert failed: xTaskCreateStaticPinnedToCore freertos_tasks_c_additions.h:299
              (xPortcheckValidStackMem(puxStackBuffer))
```

Decoding the stack gave `vTaskStartScheduler` → `prvCreateIdleTasks` →
assert: the **idle task's** stack allocation returned NULL. The real cause is
that esp_hosted's SDIO mempool is allocated up front from internal DMA-capable
RAM, and at the default queue sizes (20/20) it does not fit alongside the camera
stack. Internal RAM ran out during startup and the idle task was simply the next
allocation in line — nothing in the log connected it to WiFi.

Fix here: `CONFIG_ESP_HOSTED_SDIO_TX_Q_SIZE=10` / `..._RX_Q_SIZE=10`, which
halves the pool. Runtime internal free is then ~300 KB.

**2. `CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y` fixes trap 1 and breaks bulk
transfers.** It is the obvious fix, and esp_hosted's own help text recommends it
for "memory-intensive workloads like sustained 1080p video streaming" on the P4.
It boots. But then **every response larger than the initial TCP congestion
window stalls**: `/bench` delivered 1 432 bytes and hung, `/snapshot.jpg`
delivered 4 293 bytes of a correct `Content-Length: 273904` and hung, while
`/stats` at 109 bytes answered in 116 ms. The stalled sockets then starved the
server's socket pool, so even small requests started failing afterwards.

The camera and encoder were provably fine throughout — the `Content-Length`
proves the frame was captured and encoded. Keep the transport's DMA buffers in
internal RAM.

A hypothesis that was **wrong**, recorded so it is not retried: that
`CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65535` caused the stall by not being a multiple
of `TCP_MSS`. Changing it to 64800 (45 × 1440) changed nothing. The values are
kept as MSS multiples because IDF's own defaults are, but that was hygiene, not
the fix.

## Design notes

**A capture task publishes frames; handlers only read the published copy.**
Serving straight from a dequeued V4L2 buffer would be simpler and wrong — the
camera does not stop while a response is on the wire, and with `BUFFER_COUNT`
buffers, one held by a handler, the driver recycles the buffer being read. That
is exactly how the serial path corrupted its first raw sends. Keeping the stream
running between requests also keeps AE, AWB and autofocus converged.

**No microSD.** SD and esp_hosted share the SDMMC peripheral and hit ESP-IDF
issue 16233. Serial capture replaced the card, and the network now replaces
that.

**The `ISP_AWB_WINDOW_X_NUM/_Y_NUM=5` block in `CMakeLists.txt` is required**,
between `project.cmake` and `project()`. Without it the prebuilt esp_ipa binary
and this build disagree on `esp_ipa_stats_t`, and autofocus silently reads
unrelated heap memory. See trap 5 in the platform-traps notes.

**A 4 MB partition table**, because the image is 995 KB — the stock 1 MB factory
partition is not enough once the camera stack, the WiFi stack and esp_hosted are
in one binary.
