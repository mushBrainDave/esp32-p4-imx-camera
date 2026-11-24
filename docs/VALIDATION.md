# Validation

The supported baseline is Guition JC-ESP32P4-M3-DEV with ESP32-P4 v1.3,
16 MB flash, 32 MB PSRAM and IMX219 over 100 Mbps Ethernet, using ESP-IDF 6.0.2.

Completed checks include a clean source build with pinned configuration and
dependencies; eight ASan/UBSan host test groups; H.264 decoding and cadence for
all seven profiles; exposure controls and profile persistence; RTSP/HLS relay
playback; and bounded restart, reader-disconnection and physical-link recovery.

These checks do not establish day-long endurance, calibrated colour/IR response,
performance on other boards or sensors, or a universal browser latency bound.
A new build is not automatically hardware-tested. Run the host suite with
`python3 tools/host-tests.py` and follow the root README to build the firmware.
