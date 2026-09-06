# esp32_bt_speaker

Bluetooth half of a two-ESP32 TuneIn-to-Bluetooth internet radio bridge.

This chip runs no Wi-Fi or TLS at all. It listens for PCM audio arriving over I2S as the
**I2S slave** (BCLK/WS are inputs, DIN only — no clock generation), resamples it as needed, and
streams it out over classic Bluetooth **A2DP source** to a real Bluetooth speaker (the target in
`main/app_config.h` is an EDIFIER R1280DB) via [ESP-ADF](https://github.com/espressif/esp-adf)'s
Bluedroid-based `bluetooth_service`.

With no Wi-Fi/mbedTLS running here, all of this chip's ~200KB+ free heap at boot is available to
Bluedroid alone.

## Companion project

This board is one half of a pair. The other half,
**[esp32_wifi_streamer](https://github.com/ashrockd/esp32_wifi_streamer)**, owns Wi-Fi, resolves
and decodes the TuneIn stream, and sends this chip its PCM over the I2S link described below. The
two firmwares are built and flashed independently but are only useful together — they replace what
was originally a single ESP32 doing Wi-Fi, TLS, decode, *and* Bluetooth/A2DP at once, split across
two chips so each side gets an untouched heap for its own job instead of Wi-Fi/TLS/Bluetooth
fighting over one radio's worth of RAM.

## I2S link (esp32_wifi_streamer → this chip)

Both firmwares' `main/app_config.h` hard-code the same three GPIOs for the cross-board I2S link
(no MCLK wire needed for a direct ESP32-to-ESP32 digital link — just the 3 signal wires + a shared
ground). The sdkconfig on both boards has `CONFIG_ESP_LYRAT_V4_3_BOARD=y` (ESP-ADF's ESP32-LyraT
V4.3 board profile); the pins below are this project's own I2S GPIO assignment, chosen to steer
clear of that board's strapping pins (0, 2, 5, 12, 15), its flash-connected pins (6–11), UART0
(1, 3 — used for flashing/serial console), and the input-only pins (34–39):

| Signal | Role on this chip | GPIO |
|---|---|---|
| BCLK (bit clock) | input (I2S slave) | GPIO26 |
| WS / LRCLK (word select) | input (I2S slave) | GPIO25 |
| DATA (audio) | input — DIN | GPIO27 |
| GND | shared reference | — |

`esp32_wifi_streamer` wires the identical GPIO26 / GPIO25 / GPIO27 trio as its I2S **master**
output — see that repo's README for its side of the link.

Since `esp32_wifi_streamer` picks its I2S output rate per-station (44.1kHz or 48kHz) with no
back-channel to tell this chip which, this chip doesn't assume a fixed input rate: it measures the
real incoming rate live (the existing I2S probe in `main.c`) and retargets `resample_filter`'s
source rate on the fly via `rsp_filter_change_src_info()` roughly once a second, snapping to
whichever of 44.1kHz/48kHz the measurement is nearer to rather than feeding the resampler a raw
(non-table) measured value. When the measured rate rounds to the 44.1kHz Bluetooth output rate,
`resample_filter` is left in the pipeline with `src_rate == dest_rate` (a lossless 1:1 pass) rather
than being torn out live. See `RADIO_I2S_RATE_CANDIDATE_LOW_HZ`/`_HIGH_HZ` in `main/app_config.h`
for the details and reasoning.

## AVRCP → UART command bridge

This chip is the **AVRCP target**: the connected Bluetooth speaker is the controller and can send
transport commands (play/pause/stop/next/previous/volume) from its own remote/physical controls.
Those commands only mean something to whatever is actually driving playback — `esp32_wifi_streamer`
on the other side of the I2S link — so every AVRCP command received here is translated to a stable
ASCII name and forwarded one-way over a dedicated UART link to that chip (which uses next/previous
to change station). See `RADIO_AVRC_UART_PORT`/`RADIO_AVRC_UART_TX_GPIO` in `main/app_config.h` for
the pin and wire format (plain `AVRCP:CMD:<name>:<PRESSED|RELEASED>`-style ASCII lines, no shared
struct/binary framing required on the receiving side). TX only, plus a common ground with the other
board — no RX pin is wired since `esp32_wifi_streamer` has nothing to report back.

## Building

This is an [ESP-ADF](https://github.com/espressif/esp-adf) project (ESP-IDF v5.5.5, target
`esp32`), built the standard ESP-IDF way once both environments are exported:

```sh
. $IDF_PATH/export.sh
. $ADF_PATH/export.sh
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

Because this chip runs no Wi-Fi/HTTP/TLS/JSON pipeline stages at all, only build the ESP-ADF
components this project's `CMakeLists.txt` actually requires by setting `MINIMAL_BUILD=1` before
building, instead of pulling in ESP-ADF's entire component tree. `build.ps1` wraps all of this for
a Windows/PowerShell dev machine (activates the pinned ESP-IDF/ESP-ADF profiles, sets
`MINIMAL_BUILD`, and skips `set-target`'s forced fullclean unless `sdkconfig.defaults` actually
changed) — see its header comment for direct invocation examples and the `-Clean`/`-Flash`/`-Port`
switches.

Update `RADIO_SPEAKER_NAME` and `RADIO_LOCAL_BT_NAME` in `main/app_config.h` to target your own
speaker rather than the EDIFIER R1280DB this project was built against.

## Status

Working end-to-end (I2S in → live rate detection → resample only when needed → A2DP out), with
active tuning around I2S clock-drift/jitter between the two boards — see `main/app_config.h` for
the current, commented constants (including the measured vs. nominal I2S rate) and the reasoning
behind each.
