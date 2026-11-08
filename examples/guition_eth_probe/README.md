# Guition Ethernet probe

Adapted from the local ESP-IDF basic Ethernet example (Apache-2.0).
Uses schematic-backed Guition RMII routing, external clock input GPIO50,
MDC31/MDIO52/reset51, automatic PHY-address detection and generic 802.3 PHY.
Logs MDIO ID registers before link/DHCP testing. Console:115200 baud.

Build/flash with explicit configuration:

```bash
idf.py -B build-guition -D SDKCONFIG=build-guition/sdkconfig reconfigure build
idf.py -B build-guition -D SDKCONFIG=build-guition/sdkconfig -p /dev/ttyUSB0 flash monitor
```

Inspect image headers for P4v1.0–v1.99 and16MB before flashing.
This tests PHY/link/DHCP; it does not establish streaming goodput.

The probe also serves `GET /bench` after DHCP: a fixed 32 MiB HTTP response for
useful TCP goodput measurements. Socket send/receive waits are limited to two
seconds. Three initial measurements were approximately 85–87 Mbit/s; see the
