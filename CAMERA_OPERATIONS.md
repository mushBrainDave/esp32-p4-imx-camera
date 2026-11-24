# IMX219 LAN camera operations

Guition JC-ESP32P4-M3-DEV, ESP32-P4 v1.3,
IMX219, 100 Mbps Ethernet, ESP-IDF v6.0.2.

## Connect and inspect

Replace `CAMERA_IP` with the board's DHCP address. A router DHCP reservation
keeps the relay source address stable.

```bash
curl http://CAMERA_IP/api/v1/status
curl http://CAMERA_IP/api/v1/profiles
curl http://CAMERA_IP/api/v1/image
```

At startup the saved profile is restored with automatic exposure. In idle state, `rtsp_listening` and
`h264_sdp_ready` true, `errors` zero. `playing` becomes true when a relay/viewer
reads the stream. Counters apply to the current boot/profile; inspect their
saved context before interpreting a nonzero value after a deliberate cable test.

Direct camera stream: `rtsp://CAMERA_IP:8554/live`.
The camera admits **one upstream RTSP reader**. Use the LAN relay for multiple
viewers; a direct player/recording competes with a continuously pulling relay.

## Modes and switching

| ID | Output / FPS | Scene and compression |
| --- | --- | --- |
| `detail15` | 1632×1232 /15 | Almost full scene; tiny side trim; 4 Mbit/s target |
| `fullview15` | 1280×960 /15 | Full scene resized; 4 Mbit/s target |
| `fullview30` | 832×624 /30 | Full scene resized; 2 Mbit/s target |
| `720p30` | 1280×720 /30 | Wide sensor crop; 3 Mbit/s target |
| `balanced30` | 1536×864 /30 | Wide sensor crop; 4 Mbit/s target |
| `ultra5` | 1664×1248 /5 | Native 1640×1232 scene with small borders; fixed QP8 |
| `ultra1` | 1664×1248 /1 | Same native scene/borders; fixed QP8 |

Use the same command with any ID from the table:

```bash
curl -X POST 'http://CAMERA_IP/api/v1/profile?id=ultra5'
curl http://CAMERA_IP/api/v1/profiles
```

The empty-body POST returns202. Poll until `active` and `requested` match,
`switching` is false and `last_error` is empty. Format changes close the old RTSP
session; the relay reconnects, and a player may need to reopen. Allow up to60s
when selecting `ultra1`, including its longer sensor settling and initial frames.
Selection persists across power loss. Switching profiles and rebooting restore
automatic exposure; manual image settings are not persisted.

Fixed-QP bitrate depends on scene/noise. The nominal bitrate field in ultra modes
is not a bandwidth cap. No public15fps mode uses fixed QP8.

## Automatic and manual brightness

```bash
curl -X POST 'http://CAMERA_IP/api/v1/image?mode=auto'
curl -X POST 'http://CAMERA_IP/api/v1/image?mode=manual&exposure_us=10000&gain_milli=1000'
curl http://CAMERA_IP/api/v1/image
```

Poll until `pending` is false, `active_mode` matches and `last_error` is empty.
GET reports the selected mode's allowed limits and sensor readback. Automatic
exposure cannot exceed the FPS budget: roughly25ms at30fps,58.3ms at15fps,
191.7ms in ultra5 and991.6ms in ultra1 under the current anti-flicker policy.
These differences explain why ultra modes can look brighter in a dim room.

Power-only IR emitters cannot be switched through the camera API.

## MediaMTX relay

The tested example is [server/mediamtx/mediamtx.yml](server/mediamtx/mediamtx.yml),
validated with MediaMTX v1.21.3. Its `cam01` path pulls the camera over RTSP/TCP
without transcoding. The example's global `readTimeout: 3s` improves source-loss
recovery. Review global settings when merging into an existing server.

For a standalone server, from repository root:

```bash
mediamtx server/mediamtx/mediamtx.yml
```

View through the server:

- RTSP: `rtsp://SERVER_IP:8554/cam01`
- HLS player: `http://SERVER_IP:8888/cam01`
- HLS playlist: `http://SERVER_IP:8888/cam01/index.m3u8`

RTSP and HLS decoding/fan-out were tested. A particular browser/player's persistent
playback and visual HLS delay must be checked on that client; FFmpeg decoding does
not establish browser behavior.

## Diagnosis and recovery

If the picture stops, read the three API endpoints and the relay source log.
If `playing` is true, identify the existing upstream reader before opening a
second direct connection. If switching is active, let the format transition finish.

`delivery.queue_age_limit_ms` is250 at15/30fps,450 in ultra5 and1250 in ultra1.
It is a stale-frame reset threshold, not an intentional send delay or a display
latency guarantee. `queue_full_resets`, `queue_age_resets` and callback-capacity
counters explain device resynchronization; socket send failures at normal reader
teardown can be expected. Preserve counters/timelines before changing profiles.

The bridge restarts sessions after link loss/backpressure and retains SPS/PPS for
new viewers. It pins clients and the listener through shutdown to prevent stale
workers or the pinned library's duplicate listener close from reaching HTTP sockets.
Normal profile changes preserve board uptime; a changed boot epoch identifies a reboot.

To build or flash, follow the [repository quick start](README.md#build).
Serial device names vary by host; opening a serial port can reset some boards.
The current partition layout has no OTA update or automatic rollback slots.
