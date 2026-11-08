# ESP32-P4 IMX219 LAN camera

A maintained fork of [mushBrainDave/esp32-p4-imx-camera](https://github.com/mushBrainDave/esp32-p4-imx-camera), extending its IMX219 driver with an Ethernet H.264/RTSP camera application. Original history, licenses and attribution are retained. See the [upstream project guide](docs/UPSTREAM_README.md) for its other sensors and examples (paths in that archived guide are relative to the repository root).

## Tested hardware

Guition JC-ESP32P4-M3-DEV, ESP32-P4 v1.3 (pre-v3), 16 MB flash,
32 MB PSRAM and IMX219. Ethernet is 100 Mbps. Other boards and IMX462 are not
qualified by this milestone.

## Build

Install ESP-IDF v6.0.2 at commit
`7101770dc6db2667b3c477cc31365dd1acd6db4e` with its submodules and ESP32-P4 tools,
then activate that SDK's export.sh. From a clean checkout:

```bash
python3 tools/setup-dependencies.py
bash tools/build-camera.sh
python3 tools/host-tests.py
```

The setup script fetches the exact pre-v3 IPA compatibility dependency.
The build uses the qualified configuration and dependency lock in config/guition/.
Existing different dependency locks are preserved; use a clean checkout if the
wrapper reports a conflict. Generated firmware is in build-camera/.
Builds are not promised byte-identical to the saved hardware-tested firmware.

To flash that new build, after connecting the board:

```bash
idf.py -C components/esp_cam_sensor_imx/examples/imx219_rtsp_eth -B "$PWD/build-camera" -D SDKCONFIG="$PWD/build-camera/sdkconfig" -p /dev/ttyUSB0 flash
```

## Connect and control

Find the board's DHCP lease on your router. With CAMERA_IP replaced:

```bash
curl http://CAMERA_IP/api/v1/status
curl http://CAMERA_IP/api/v1/profiles
curl -X POST 'http://CAMERA_IP/api/v1/profile?id=fullview30'
```

Open `rtsp://CAMERA_IP:8554/live`. Use a single camera reader; a LAN server
running [MediaMTX](server/mediamtx/README.md) can pull once and distribute RTSP/HLS
to multiple viewers. Replace the source address in its example configuration.
MediaMTX runs on the server.

| Profile | Encoded dimensions | FPS | View |
|---|---|---|---|
| detail15 | 1632 × 1232 | 15 | Slight side trim |
| fullview15 | 1280 × 960 | 15 | Full view, resized |
| fullview30 | 832 × 624 | 30 | Full view, resized |
| 720p30 | 1280 × 720 | 30 | Wide crop |
| balanced30 | 1536 × 864 | 30 | Wide crop |
| ultra1 | 1664 × 1248 | 1 | Native full view with padding, QP 8 |
| ultra5 | 1664 × 1248 | 5 | Native full view with padding, QP 8 |

See [camera operations](CAMERA_OPERATIONS.md) for API details, exposure controls
and operational limits. Services are intended for a trusted LAN.

## Validation and development

[Validation](docs/VALIDATION.md) describes tested hardware, checks and limits.
[Upstream review notes](docs/UPSTREAM_REVIEW.md) record compatibility work still
needed before contributing these changes upstream.

Firmware release assets should contain binaries, checksums and flashing
instructions. See the [release guide](release/README.md).
