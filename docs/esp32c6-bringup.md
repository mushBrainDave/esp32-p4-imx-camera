# ESP32-C6 bring-up on the Waveshare ESP32-P4-WIFI6 — discovery notes

Answers to the three questions that blocked starting this work: how the C6 is
reachable, what it is wired to, and what ESP-IDF version the software wants.
Gathered 2026-08-29 from the board schematic, the `esp_hosted` component itself,
and the Waveshare wiki. Nothing here has been run on hardware yet.

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
Note this pad has **no EN pin**, unlike the EV board's `PROG_C6` header, which
is why Waveshare's instruction is "pull `C6_IO9` low while powering on" rather
than the usual auto-reset. The P4 can also drive the C6's reset itself over
GPIO54, which may make this less manual than it looks. The P4 must be held in
bootloader mode meanwhile so it does not talk over SDIO during the flash.

**b. OTA over the SDIO link — no wires at all.** `esp_hosted` can update the
slave from the host; see its `examples/host_performs_slave_ota`. Espressif calls
this the recommended path for everything after initial setup.

**The open question is whether (b) is available on first boot**, and it turns on
one unknown: *does Waveshare ship the C6 pre-flashed?* The Function EV board
does (slave firmware v0.0.6), and Waveshare's wiki walkthrough goes straight to
running a WiFi example with no flashing step, which implies theirs is too — but
that is inference, not a documented promise. **Check this before buying an
ESP-Prog.** The cheap test is to build a host app and see whether it finds a
slave at all.

Caveat that makes it matter: **host and slave must run the same esp_hosted
version.** A stock v0.0.6 slave will very likely not talk to a 2.12.x host, so
even a pre-flashed board probably needs one serial flash — unless the OTA path
works across that gap. Pin both sides explicitly:

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

Recommendation: **start on 5.4.0 with esp_hosted 2.12.12.** It needs no upgrade
and answers the only question that matters first — whether the link comes up at
all. Treat the 5.5 upgrade as a separate piece of work, decided once the radio
is known to function, so an IDF upgrade and a first bring-up are never being
debugged at the same time.

## Suggested order

1. Build a host app on 5.4.0 with the pinned versions above, defaults unchanged.
2. See whether it detects a slave. That single result resolves both the
   pre-flashed question and whether the pin map is right.
3. If no slave: flash one over the H4 pad, matching versions.
4. Only then consider the IDF 5.5 upgrade.

## Sources

- Board schematic: `https://files.waveshare.com/wiki/ESP32-P4-WIFI6/ESP32-P4-WIFI6-datasheet.pdf`
- Waveshare wiki: `https://www.waveshare.com/wiki/ESP32-P4-WIFI6`
- `esp_hosted` 2.12.12 `Kconfig` and `docs/`, from the component registry
- Registry API: `https://components.espressif.com/api/components/espressif/{esp_hosted,esp_wifi_remote}`
