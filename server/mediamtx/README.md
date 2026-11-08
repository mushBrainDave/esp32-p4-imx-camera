# LAN server integration

The board serves H.264 over RTSP/TCP. MediaMTX runs on a LAN server, pulls once,
and redistributes to multiple viewers without transcoding. The example targets
MediaMTX v1.21.3. Replace CAMERA_IP in mediamtx.yml with the board's DHCP address.

From repository root:

```bash
mediamtx server/mediamtx/mediamtx.yml
```

Viewer URLs (replace SERVER_IP with the relay's address):

- RTSP: `rtsp://SERVER_IP:8554/cam01`
- HLS player: `http://SERVER_IP:8888/cam01`
- HLS playlist: `http://SERVER_IP:8888/cam01/index.m3u8`

The camera supports one upstream reader. A continuously pulling relay occupies
that session; use the relay for viewers and stop its source before direct-camera
diagnostics. Retain your deployment's authentication and network policy when
merging this minimal example into an existing server.

The global `readTimeout: 3s` controls source-loss detection and applies beyond
this camera path. The 900 ms HLS minimum segment duration allows margin around
the nominal one-second GOP; actual segment duration follows keyframe timing.
Browser compatibility and playback latency depend on the viewer.

Profile changes close the camera's current session. MediaMTX reconnects; a viewer
may need to reopen when dimensions change. Allow up to 60 seconds for low-rate
profiles to settle and the relay to reconnect. Use the HTTP API documented in
[Camera operations](../../CAMERA_OPERATIONS.md) for profile and exposure controls.
Fixed-QP streams have scene-dependent bitrate; allow network headroom.
