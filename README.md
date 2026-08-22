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

## Status

Working end-to-end (I2S in → resample → A2DP out), with active tuning around I2S clock-drift/jitter
between the two boards — see `main/app_config.h` for the current, commented constants (including
the measured vs. nominal I2S rate) and the reasoning behind each.
