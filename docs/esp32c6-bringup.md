# ESP32-C6 bring-up on the Waveshare ESP32-P4-WIFI6 — discovery notes

Answers to the three questions that blocked starting this work: how the C6 is
reachable, what it is wired to, and what ESP-IDF version the software wants.
Gathered 2026-08-29 from the board schematic, the `esp_hosted` component itself,
and the Waveshare wiki.

## Result: the link is UP, on ESP-IDF 5.4.0 — no upgrade needed

Verified on hardware 2026-08-29 by `examples/c6_link_check`, which brings up
WiFi and scans. A scan needs a working radio, not merely a driver that accepts
a config, so the AP list is real evidence:

```
sdio_wrapper: SDIO master: Slot 1, Data-Lines: 4-bit Freq(KHz)[40000 KHz]
sdio_wrapper: GPIOs: CLK[18] CMD[19] D0[14] D1[15] D2[16] D3[17] Slave_Reset[54]
transport:    Identified slave [esp32c6]
transport:    capabilities: 0xd  ->  WLAN, HCI over SDIO, BLE only
c6_link_check: C6 station MAC 9c:13:9e:d5:ba:e4
c6_link_check: scan found 7 AP(s)
```

Three things this settles:

1. **The pin map above is correct** — the driver's own log prints it back, and
   the bus enumerates at 4-bit / 40 MHz.
2. **Waveshare does ship the C6 pre-flashed.** This was the one unknown left by
   the paper research, and it means no ESP-Prog and no soldering are needed to
   get started.
3. **esp_hosted 2.12.12 on IDF 5.4.0 is a working combination.** The version
   ceiling is not a blocker for bring-up.

**The slave firmware has since been upgraded — see "Slave OTA" below. The
caveat that follows was the state as shipped.**

**The pre-flashed slave was ancient.**

```
W transport: Version mismatch: Host [2.12.0] > Co-proc [0.0.0]
             ==> Upgrade co-proc to avoid RPC timeouts
```

Scanning works anyway, but Espressif's own warning says to expect RPC timeouts,
so this is a real thing to fix rather than noise to silence — and it is very
likely to bite on anything longer-running than a scan. The good news is that
the link working *is* what unlocks the easy fix: the slave can now be updated
**OTA over SDIO** from the P4 (`esp_hosted`'s `host_performs_slave_ota`
example), with no wires at all. Do that before building anything real on top.

Also needed, and now in `sdkconfig.defaults`: **`CONFIG_FREERTOS_HZ=1000`**.
esp_hosted warns at boot that the 100 Hz default causes "bus level jitters" —
every wait on the SDIO transport rounds up to a 10 ms tick.

## 1. Pin map

**Source: the board schematic** (`ESP32-P4-WIFI6-datasheet.pdf`, Resources ->
Schematic Diagram on the Waveshare wiki). The co-processor is an
**ESP32-C6-MINI-1-N4** (U14).

| Signal | P4 GPIO | Evidence |
| --- | --- | --- |
| SDIO CLK | **18** | in the C6 net cluster, and the only one of the six with **no** pull-up |
| SDIO CMD | **19** | pulled up by R86 51K |
| SDIO D0 | **14** | pulled up by R87 51K |
| SDIO D1 | **15** | pulled up by R88 51K |
| SDIO D2 | **16** | pulled up by R89 51K |
| SDIO D3 | **17** | pulled up by R90 51K |
| C6 reset / enable | **54** | GPIO54 -> R82/R83 0R -> `C6_CHIP_PU` |
| unknown | 6 | `C6_IO2` <-> P4 GPIO6. Purpose not established; not needed for basic bring-up. |

Confidence: the *set* of pins is certain, and CLK=18 / CMD=19 is solid because
the pull-up pattern is diagnostic — SDIO pulls up CMD and DATA but not CLK.
The D0..D3 ordering across 14-17 comes from `esp_hosted`'s ESP32-P4 defaults,
which match this net cluster exactly. Confirm on hardware before trusting the
individual data-line order.

**These are exactly `esp_hosted`'s built-in ESP32-P4 defaults**, so no pin
overrides should be needed. Waveshare copied the ESP32-P4-Function-EV-Board
wiring, which is also why their own wiki instructions add the components and
change nothing but the slave target.

### The SD card shares the peripheral, not the pins

The C6 is on **SDMMC slot 1**. The microSD is on **slot 0** (CLK 43, CMD 44,
D0-D3 39-42) — and on the P4 slot 0's pins are fixed, so the two cannot be
swapped. No pin conflict, but they are the same peripheral: **ESP-IDF issue
16233** breaks SD card and esp_hosted running together over SDMMC. `esp_hosted`
ships `examples/host_sdcard_with_hosted` specifically to carry the workaround.
Relevant because `imx708_snapshot` still compiles SD support, even though
serial capture replaced it in practice.

## 2. How the C6 is reachable

**Not over COM3.** COM3 is the CH343 bridge to the P4 and is the only port the
machine has; the C6's UART does not appear on it. Two real paths:

**a. Serial, via the 4-pin pad.** The schematic shows header **H4** carrying
`GND`, `C6_U0RXD`, `C6_U0TXD`, `C6_IO9` — needs an ESP-Prog or any UART adapter.
Commands for this route are in the appendix at the end; it is the recovery path,
not the normal one.
Note this pad has **no EN pin**, unlike the EV board's `PROG_C6` header, which
is why Waveshare's instruction is "pull `C6_IO9` low while powering on" rather
than the usual auto-reset. The P4 can also drive the C6's reset itself over
GPIO54, which may make this less manual than it looks. The P4 must be held in
bootloader mode meanwhile so it does not talk over SDIO during the flash.

**b. OTA over the SDIO link — no wires at all.** `esp_hosted` can update the
slave from the host; see its `examples/host_performs_slave_ota`. Espressif calls
this the recommended path for everything after initial setup.

**Resolved on hardware: Waveshare does pre-flash the C6**, so (b) is available
from the start and no ESP-Prog is required. The slave reports itself as version
`0.0.0` and still serves scans against a 2.12.0 host, so the version gap is
more forgiving than expected — but Espressif warns it will cause RPC timeouts,
so treat an OTA slave upgrade as the next step, not an optional one.

Host and slave are still meant to run the same esp_hosted version. Pin both
sides explicitly:

```
idf.py add-dependency "espressif/esp_hosted^2.12.12"
idf.py create-project-from-example "espressif/esp_hosted^2.12.12:slave"
```

## 3. ESP-IDF version

**This is the headline: `esp_hosted` 3.x requires ESP-IDF >= 5.5. We are on
5.4.0.**

| | newest usable on IDF 5.4 | newest overall |
| --- | --- | --- |
| `esp_hosted` | **2.12.12** (idf >= 5.3) | 3.0.6 (**idf >= 5.5**) |
| `esp_wifi_remote` | 1.6.4 (idf >= 5.3) | same |

Neither component ships with IDF 5.4; both come from the registry, as expected.

`esp_wifi_remote` 1.6.4 depends on `esp_hosted >= 2.11` with no upper bound, so
the solver will try 3.0.6 first and reject it on the IDF rule before settling on
2.12.12. That should resolve on its own; pinning avoids relying on it.
(Version 1.6.3 pins `>=2.11,<3.0` explicitly, if a stable floor is wanted.)

Also worth knowing: **3.0.0 is the first release to declare `esp32p4` in its
target list.** The 2.x line declares no targets at all, which means "any" rather
than "tested" — and P4-as-host is clearly supported in 2.x, since that line
ships a whole `docs/esp32_p4_function_ev_board.md` guide. Still, 3.x is where
P4 support became explicit.

### The upgrade decision

Three options, and the middle one is probably wrong:

- **Stay on 5.4.0.** Cap at esp_hosted 2.12.12. Nothing else changes. The
  esp_ipa ABI workaround (`ISP_AWB_WINDOW_X_NUM/_Y_NUM`, see the CMakeLists in
  both examples) stays.
- **Upgrade to 5.4.4-ish.** Retires the esp_ipa workaround but does *not* unlock
  esp_hosted 3.x. Pays the cost of an IDF upgrade for one of two benefits.
- **Upgrade to >= 5.5.** Retires the esp_ipa workaround *and* unlocks
  esp_hosted 3.x. One upgrade, both problems — but the largest jump, and the
  camera stack (esp_video, esp_cam_sensor, esp_ipa) would all need revalidating
  against it, including the closed esp_ipa binary that caused the trap in the
  first place.

Recommendation, now backed by a working link: **stay on 5.4.0 with esp_hosted
2.12.12.** The radio works there, so an IDF upgrade buys nothing for the C6 and
would only put the camera stack at risk. Revisit >= 5.5 only if something in
esp_hosted 3.x is actually wanted, and decide it on the esp_ipa workaround's
merits instead — see [[esp32p4-camera-platform-traps]] trap 5.

## Where this leaves things

Done: link up, pin map confirmed, versions settled, `examples/c6_link_check`
committed as the regression test for all of it.

Next, in order:

1. ~~**OTA-upgrade the slave firmware** off `0.0.0`~~ — done, see below.
2. ~~Connecting to an AP~~ — done, `examples/c6_wifi_sta`, see
   [Station connect](#station-connect--associating-with-a-real-ap). Streaming
   frames off the board over WiFi rather than serial is the step after.
3. The SD/SDMMC coexistence workaround only matters if the camera examples and
   the radio end up in one build.

## Sources

Command-by-command recipes for everything below — flashing the slave, running
the link check, connecting to the router — are collected in
[`cli-cookbook.md`](cli-cookbook.md).


- Board schematic: `https://files.waveshare.com/wiki/ESP32-P4-WIFI6/ESP32-P4-WIFI6-datasheet.pdf`
- Waveshare wiki: `https://www.waveshare.com/wiki/ESP32-P4-WIFI6`
- `esp_hosted` 2.12.12 `Kconfig` and `docs/`, from the component registry
- Registry API: `https://components.espressif.com/api/components/espressif/{esp_hosted,esp_wifi_remote}`

## Slave OTA — done 2026-08-29

The as-shipped slave was upgraded from `0.0.0` to **2.12.12** over the SDIO
link. No ESP-Prog, no wires, nothing touched on the H4 pad.

**Method: esp_hosted's `host_performs_slave_ota` example, "Partition" variant.**
The slave firmware is flashed into a dedicated `slave_fw` partition on the
*host's* flash, and the host then streams it to the C6 over the existing
transport, where the C6 writes it into its own OTA partition and reboots into
it. Note "partition" refers to two different chips in that sentence — a staging
partition on the P4, and the app partition on the C6 that actually ends up
running. The name describes only where the host keeps the image; the C6 is
genuinely reflashed either way. LittleFS and HTTPS variants also exist; Partition needs neither a
filesystem nor a network, which makes it the right one for a first upgrade.

Reproducing it, from an activated ESP-IDF shell:

```sh
# 1. Build the slave firmware for the C6, pinned to the host's version.
idf.py create-project-from-example "espressif/esp_hosted^2.12.12:slave"
cd slave && idf.py set-target esp32c6 && idf.py build     # -> build/network_adapter.bin

# 2. Build the OTA host app and stage that binary in it.
idf.py create-project-from-example "espressif/esp_hosted^2.12.12:host_performs_slave_ota"
cp slave/build/network_adapter.bin    host_performs_slave_ota/components/ota_partition/slave_fw_bin/

# 3. Add to its sdkconfig.defaults, then set-target esp32p4, build, flash.
#      CONFIG_OTA_METHOD_PARTITION=y
#      CONFIG_SLAVE_IDF_TARGET_ESP32C6=y
```

`idf.py flash` writes the slave image to the `slave_fw` partition through a
custom CMake target rather than through `flasher_args.json` — so it will not
appear in `flash_files`, and that absence is not a sign anything is wrong.

What the run looks like, and the two lines worth expecting:

```
ota_partition: Firmware verified - Size: 1157664 bytes, Version: 2.12.12
ota_partition: Partition OTA completed successfully - Sent 1157664 bytes
host_performs_slave_ota: OTA completed successfully!
host_performs_slave_ota: Activate API not supported (requires v2.6.0+)
host_performs_slave_ota: Restarting host to resync with slave...
```

The transfer took ~11 s for 1.16 MB. **"Activate API not supported" is expected
and not a failure** — the activate RPC only exists in slave firmware >= 2.6.0,
and the old slave being replaced predates it. The docs are explicit that
activation is not required in that case; the host restart is what resyncs.

Two things confirm it worked, beyond the success line:

1. `Req_GetCoprocessorFwVersion` **used to time out** and now answers. That RPC
   timing out was exactly what esp_hosted's version-mismatch warning predicted,
   so it working is the symptom clearing rather than a separate claim.
2. The `Version mismatch: Host [2.12.0] > Co-proc [0.0.0]` warning is **gone**.
   Re-running `examples/c6_link_check` now boots clean — `Identified slave
   [esp32c6]`, no warnings, 9 APs scanned.

Note the two version readings disagree before an upgrade and this is not a bug:
the transport handshake reports `0.0.0` for a slave too old to populate that
field, while `esp_hosted_get_coprocessor_fwversion()` is an RPC — which is why
it timed out rather than returning a number. Do not read a `0.0.0` as "no
firmware".

## Station connect — associating with a real AP

**Working on hardware 2026-08-29.** Associated, DHCP lease, and a live HTTP
round-trip to the internet, first attempt, no retries:

```
I (2608) c6_wifi_sta: connecting to "TP-Link_14DC" ...
I (3418) RPC_WRAP: ESP Event: Station mode: Connected
I (5087) c6_wifi_sta: got IP 192.168.0.164  netmask 255.255.255.0  gateway 192.168.0.1
I (5094) c6_wifi_sta: associated with "TP-Link_14DC"  bssid a8:29:48:92:14:dc
I (5094) c6_wifi_sta: channel 3  rssi -47 dBm  authmode 3  phy bgn
I (5095) c6_wifi_sta: dns 192.168.0.50
I (5178) c6_wifi_sta: example.com resolved to 172.66.147.243
I (5202) c6_wifi_sta: TCP connected to example.com:80
I (5223) c6_wifi_sta: HTTP response: HTTP/1.1 200 OK

==== done - CONNECTED, internet reachable ====
```

**Boot to HTTP 200 in 5.2 s**, and the breakdown is worth keeping because it
says where the time actually goes:

| Phase | ms | |
| --- | --- | --- |
| boot -> transport up, C6 booted | ~2400 | the SDIO bring-up and slave reset dominate |
| `esp_wifi_connect` -> associated | 810 | |
| associated -> DHCP lease | **1669** | the single largest step after transport |
| DNS resolve | 66 | |
| TCP connect | 24 | |
| HTTP request -> response | 21 | |

So on this board a cold start costs about 5 s before a socket is usable, and
**DHCP is the part to attack** if that ever matters — a static lease or a
stored one would take ~1.7 s off. Once up, round-trips are in the tens of
milliseconds, which is the number that matters for streaming frames later.

`phy bgn` — no `/ax`. The C6 is a WiFi 6 radio but this AP is 802.11n, so the
link negotiated down; nothing to fix, but do not read a WiFi 6 co-processor as
a WiFi 6 link.

No `Version mismatch` warning appeared, which re-confirms the slave OTA to
2.12.12 held.

### Everything below was the design; the log above is the result

`examples/c6_wifi_sta`, on **ESP-IDF 5.4.0**, esp_hosted 2.12.12 /
esp_wifi_remote 1.6.4 — the same versions `c6_link_check` proved. Nothing about
associating needed a newer IDF: `esp_wifi_connect`, `esp_netif`, DHCP and LwIP
all run on the P4, and only the `esp_wifi_*` calls cross the SDIO bus. The
sockets in this example are ordinary LwIP sockets.

**Why a separate example rather than extending `c6_link_check`.** The link
check answers one question — is the radio alive — in about five seconds, with
no credentials and no network. That makes it worth keeping as a regression test
exactly as it is; anything that needs a PSK and a working AP is a different
kind of test, with different ways to fail.

**Scanning is receive-only, so it is weaker evidence than it looks.** A scan
proves the C6 hears. It says nothing about transmit, the 4-way handshake, or
whether the P4's LwIP stack is really wired to the far-side radio. So this
example goes as far as DNS plus a TCP connect to a host on the internet: the
first thing that exercises the whole path in both directions.

### Credentials

The PSK lives in `main/wifi_credentials.h`, which is **gitignored**, written by
hand from the committed `main/wifi_credentials.h.example`. A PSK committed once
stays in git history whatever a later commit says, so this one never enters the
tree.

The header is optional at build time. It is pulled in behind `__has_include`
and the SSID falls back to `""`, checked at run time rather than with `#if` —
so the connect path is always compiled, and a build without credentials still
type-checks all of it. Flashed without one, the firmware says what to write and
exits.

To find the exact SSID string the C6 can see, run `c6_link_check` — its scan
list is the authoritative spelling, and it also shows which of your networks
are on 2.4 GHz.

**The trap this cost a flash cycle:** `__has_include` cannot put a file that
does not exist into the depfile, so **creating `wifi_credentials.h` for the
first time does not invalidate an object compiled without it**. Ninja sees the
source unchanged, reuses the stale object, and the firmware prints "no
credentials" with the header sitting right there. `main/CMakeLists.txt` now
declares the header as an `OBJECT_DEPENDS` when it exists, which covers later
edits; the very first appearance still needs `idf.py fullclean` or a touch of
the source, so the runtime message says as much. Generalises to any optional
header behind `__has_include`.

### Reading the result

The run is over in a few seconds and ends in one of five `==== done` lines:

| Line | Meaning |
| ---- | ------- |
| `no credentials` | the header is missing or blank |
| `link DOWN` | `esp_wifi_init` failed — the C6, not the AP; run `c6_link_check` |
| `NOT connected` | the radio works, the association did not |
| `CONNECTED, but no route out` | associated with an IP, but DNS or the route failed |
| `CONNECTED, internet reachable` | full path, both directions |

**On a failure the reason code is the entire diagnostic**, which is why nothing
on the connect path is `ESP_ERROR_CHECK`ed — aborting would throw away the one
number worth having. The three that come up:

- `WIFI_REASON_NO_AP_FOUND` (201) — **the C6 is a 2.4 GHz-only radio.** A
  5 GHz-only SSID is invisible to it, and a dual-band AP that publishes one
  SSID on both bands will answer on 2.4 only. This looks exactly like a typo in
  the SSID, so check the band before re-reading the spelling.
- `WIFI_REASON_AUTH_FAIL` / `..._HANDSHAKE_TIMEOUT` — a wrong password, nearly
  always.
- `WIFI_REASON_ASSOC_FAIL` — the AP refused; MAC filtering or band steering.

### Two config choices worth keeping

- **`threshold.authmode` is left at its default (open).** Raising it to
  `WPA2_PSK` is a common copy-paste from Espressif's examples and it silently
  filters out APs below that mode — an open or WPA-only network then reports
  `NO_AP_FOUND`, which reads as a missing AP rather than a policy. `sae_pwe_h2e`
  is set to `WPA3_SAE_PWE_BOTH` so WPA3 APs requiring hash-to-element also work.
- **`esp_wifi_connect()` is called from `WIFI_EVENT_STA_START`,** not straight
  after `esp_wifi_start()`. Calling it before the driver finishes starting
  returns `ESP_ERR_WIFI_NOT_STARTED`.

`CONFIG_FREERTOS_HZ=1000` carries over from `c6_link_check` for the same reason
— esp_hosted warns about "bus level jitters" at the 100 Hz default, because
every SDIO transport wait rounds up to a 10 ms tick.

Power save is turned off (`WIFI_PS_NONE`) once connected, with the next step in
mind: streaming frames off the board. Power save has the AP buffer traffic
between beacons, which turns into bursty round-trips.

## Throughput — measured 2026-08-30

`examples/imx708_wifi_snapshot` puts the camera, the radio and an HTTP server in
one image and measures the link. Full write-up and the numbers are in that
example's README; the radio-level headlines:

- **Sustained ceiling 6-7 Mbit/s (~800 KB/s)** over SDIO -> C6 -> air, measured
  with 8 MB and 4 MB payloads. Run-to-run variance is large - two identical 8 MB
  runs differed by 50%.
- **TCP slow start dominates anything small.** The same 260 KB JPEG took 1.70 s
  on a fresh connection and **0.47 s** on a warm one. Hold connections open.
- That ceiling rules MJPEG-at-q90 out for video (~3 fps) and leaves H.264 at
  4 Mbit/s comfortable.

Two traps found doing it, both radio-level:

- **esp_hosted's SDIO mempool is allocated up front from internal DMA-capable
  RAM.** At the default queue sizes it does not fit beside the camera stack, and
  the symptom is an assert in `prvCreateIdleTasks` before `app_main` - the
  *idle task's* stack is the allocation that fails. `CONFIG_ESP_HOSTED_SDIO_TX_Q_SIZE`
  / `..._RX_Q_SIZE` at 10 each halves the pool and it fits.
- **`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y` makes it boot and breaks bulk
  transfer.** Every response past the initial congestion window stalls; small
  ones keep working. Keep the transport's DMA buffers in internal RAM, whatever
  esp_hosted's help text suggests.

## Appendix: flashing the C6 manually over serial

**Not needed in normal use, and NOT TESTED HERE** — the OTA route above worked,
so nothing was ever wired to the pad. Written down because it is the recovery
path if a bad OTA leaves the slave unable to talk, which is exactly when
nobody wants to be deriving it from scratch. Treat the commands as a starting
point, not as verified steps.

**To be clear: the OTA route does flash the C6.** These two are not
"real flashing" versus "not really flashing" — they differ in how much of the
C6's flash is written, and in what has to be alive to receive it:

| | OTA over SDIO | Serial via H4 |
| --- | --- | --- |
| Writes | the C6's **app (OTA) partition** only | the C6's **whole flash** — bootloader, partition table, ota_data, app |
| Written by | the C6's own running slave firmware | the C6's **ROM bootloader** (in silicon, always present) |
| Needs | working firmware already on the C6 | nothing working on the C6 |
| Hardware | none | UART adapter on the pad |

So serial is not the better or more thorough option to reach for by default —
it is the one that still works when there is nothing on the C6 left to receive
an OTA.

You need a USB-UART adapter (an ESP-Prog, or any 3.3 V adapter). The board's
own COM3 cannot reach the C6.

### Wiring

Header **H4** carries four signals — `GND`, `C6_U0RXD`, `C6_U0TXD`, `C6_IO9`.

> **Check the pin order against the silkscreen or the schematic before
> wiring.** The order above is the order the nets appear in the extracted
> schematic text, which is not the same thing as physical pin 1-2-3-4.
> Getting TX/RX backwards is harmless; getting 3.3 V onto the wrong pin is not.

| Adapter | H4 |
| --- | --- |
| GND | `GND` |
| TXD | `C6_U0RXD` |
| RXD | `C6_U0TXD` |
| (see below) | `C6_IO9` |
| **do not connect** | **VDD — the board powers itself** |

**This pad has no EN pin**, unlike the EV board's `PROG_C6` header, so the
adapter cannot drive the usual auto-reset. `C6_IO9` is the C6's boot strap:
hold it low across a power cycle to enter download mode, which is what
Waveshare's own instruction amounts to. Because reset is manual, esptool must
be told not to attempt its own — hence `--before no_reset` below, which differs
from the command `idf.py build` prints.

### Keep the P4 off the bus

The P4 drives the C6's reset line on GPIO54 and talks SDIO to it, so a running
host app will fight the flash. Hold the P4 in its bootloader: either the BOOT +
RST button dance (hold BOOT, tap RST, release BOOT), or over COM3:

```sh
esptool -p COM3 --before default_reset --after no_reset run
```

### Flash

Build the slave firmware first — same as OTA step 1, pinned to the host's
version:

```sh
idf.py create-project-from-example "espressif/esp_hosted^2.12.12:slave"
cd slave && idf.py set-target esp32c6 && idf.py build
```

Then, with `C6_IO9` held low across a power cycle, from `slave/build`
(`<PORT>` is the *adapter's* port, not COM3):

```sh
esptool --chip esp32c6 -p <PORT> -b 460800 --before no_reset --after no_reset   write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m   0x0 bootloader/bootloader.bin   0x8000 partition_table/partition-table.bin   0xd000 ota_data_initial.bin   0x10000 network_adapter.bin
```

Those four offsets and the flash settings are what the esp32c6 slave build
actually printed, so they are right for this firmware. Only the reset flags and
the port are changed from it, for the reasons above. Release `C6_IO9` and power
cycle to run the new firmware.

Confirm it took by reflashing `examples/c6_link_check` on the P4: a clean boot
with no `Version mismatch` line and a scan that returns APs.
