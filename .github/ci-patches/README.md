# CI patches for the vendored ESP-ADF / ESP-IDF trees

This directory holds every local modification to the vendored `../esp-adf`
and `../esp-idf` trees that `esp32_bt_speaker` actually depends on to build
**correctly** - not just successfully. `esp32_bt_speaker/.github/workflows/
build.yml` applies all of them, in the order listed below, to freshly-
checked-out copies of those trees before running `idf.py build`.

Sibling to `../esp32_wifi_streamer/.github/ci-patches/README.md` - same
mechanism, same reasoning, same vendored trees (shared one level up by every
project in this family), adapted for this project's actual dependency
closure, which is much smaller: no Wi-Fi, no `esp_http_client`, no mbedTLS,
no `esp_aac_dec` - this chip only ever does `i2s_stream` (SLAVE) ->
`filter_resample` -> `bluetooth_service` (A2DP SOURCE + AVRCP TARGET).

## Why this exists

Same as the sibling project: `../esp-adf` and `../esp-idf` are vendored one
level above this project and shared with `esp32_wifi_streamer` (and other
sibling projects). They carry **uncommitted** local edits on the dev
machine - CI starts from a clean upstream checkout every time and has no way
to see those edits unless they're captured and replayed. That's what this
directory is.

## Base versions

Same pin as `esp32_wifi_streamer` (same physical vendored tree on the dev
machine, so necessarily the same commit) - see that project's own
`ci-patches/README.md` "Base versions" table for how this was verified:

| Tree | Base |
|---|---|
| `esp-adf` | commit `eac70fd2` on branch `release/v2.x` (`v2.8-14-geac70fd2`) |
| `esp-idf` actually used by the real build | tag `v5.5.5` exactly, standalone install at `C:\esp\v5.5.5\esp-idf` |

Unlike `esp32_wifi_streamer`, this project does NOT need the `esp-adf-libs`
submodule patch at all - it never includes `esp_aac_dec.h`/anything under
`esp_audio_codec`, only `filter_resample.h` (`esp_codec`), which is compiled
into `esp-adf-libs` unconditionally by that component's own stock
`CMakeLists.txt` (`COMPONENT_SRCS` lists `filter_resample.c` directly, no
patch needed to expose it) - confirmed by reading that file directly rather
than assumed.

## Patches applied (esp-adf)

1. **`esp-adf/0001-i2s-pin-override-lyrat_v4_2.patch`**
   `components/audio_board/lyrat_v4_2/board_pins_config.c` - repoints
   `get_i2s_pins()` from stock LyraT v4.2 pins (mck 0/bck 5/ws 25/dout
   26/din 35) to this project's actual wiring (bck 26/ws 25/din 27, no mclk,
   no dout - RX-only I2S slave). **THE CRITICAL PATCH**, identical
   reasoning to the sibling project's own patch #1: `i2s_stream_idf5.c`'s
   `i2s_driver_startup()` calls this function and `memcpy()`s its result
   straight over whatever `main.c` configured, unconditionally, with no
   public API to override it - an unpatched checkout compiles and boots
   fine, it just silently listens on the wrong GPIOs (confirmed by
   disassembling `get_i2s_pins` in a real built .elf on the dev machine,
   documented at length in this project's own `main/app_config.h`
   `RADIO_I2S_*_GPIO` comment block and this patched file's own header
   comment). Also why this project moved off LyraT v4.3 (ESP-ADF's Kconfig
   default when no board is selected, which is what this project silently
   ran on before 2026-08-31): `esp32_wifi_streamer_520kbram`/
   `-multistation` still select v4.3 and are I2S MASTERS needing GPIO27 as
   `data_out` - a slave on the same board definition needing GPIO27 as
   `data_in` would make IDF's `i2s_std.c` treat `dout == din` as a loopback
   request instead of two independent pins. Moving this project's own board
   selection to v4.2 (a board no I2S-master project in this family uses)
   keeps the two roles from ever colliding in one board definition.

2. **`esp-adf/0002-audio_board-idf_component-drop-ili9341-dep.patch`**
   `components/audio_board/idf_component.yml` - removes a component-manager
   dependency on `esp_lcd_ili9341: "^1"` (replaced with `dependencies: {}`).
   Identical patch to the sibling project's #3 (same shared file, same
   component-wide manifest, not per-board) - safe, verified removal of an
   unused dependency.

3. **`esp-adf/0003-audio_pipeline-cmakelists-werror-return-type.patch`**
   `components/audio_pipeline/CMakeLists.txt` - adds
   `target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=return-type)`.
   Identical to the sibling project's #4 - `audio_pipeline` is a direct
   `REQUIRES` of `main/CMakeLists.txt` here too, so this always builds and
   without this patch the build fails to compile outright (a missing-return
   warning promoted to a hard error somewhere in `audio_pipeline`'s
   sources by this ESP-IDF version's default flags).

4. **`esp-adf/0004-bluetooth_service-avrc_tg_init_order-and-classic_bt_mode.patch`**
   `components/bluetooth_service/bluetooth_service.c` - two independent
   fixes to the shared Bluetooth service helper this project's `main.c`
   drives directly (`bluetooth_service_start()`/`_create_stream()`):
   - `esp_bt_controller_enable(ESP_BT_MODE_BTDM)` -> `ESP_BT_MODE_CLASSIC_BT`
     (plus proper `g_bt_service` cleanup on every early-return failure path
     added alongside it). `ESP_BT_MODE_BTDM` needs BLE controller memory
     that the line right above this one (`esp_bt_controller_mem_release
     (ESP_BT_MODE_BLE)`) has already released, and this project's own
     `sdkconfig.defaults` sets `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY` (classic-
     BT-only, no BLE) - so the two were inconsistent and
     `esp_bt_controller_enable()` failed unconditionally with the original
     `BTDM` argument. **Without this, Bluetooth never starts at all.**
   - Adds `esp_avrc_tg_init()` (and the matching `esp_avrc_tg_deinit()` in
     `bluetooth_service_destroy()`), called before A2DP init for both
     SINK and SOURCE modes. `esp_avrc_api.h`'s own doc comment for
     `esp_avrc_tg_init()` requires AVRC to be initialized *before* A2DP;
     this file previously only called `esp_avrc_ct_init()` here and left
     `esp_avrc_tg_init()` to whichever consumer wanted AVRCP-TG to call
     later - by which point A2DP (and discovery) had already started,
     violating that ordering. This project's `main.c` IS an AVRCP TARGET
     (`start_avrc_bridge()`, forwarding transport/volume commands from the
     paired speaker over UART to `esp32_wifi_streamer`) - **without this
     fix, AVRCP connects but passthrough commands never reach the app
     callback at all**, hardware-confirmed on the sibling
     `esp32_bt_speaker_48khz` project (2026-08-24) where this bug was first
     found and fixed, and equally applicable here since both projects share
     this same file and the same AVRCP-TG usage pattern.

## Patches applied (esp-idf, standalone v5.5.5 tree)

5. **`esp-idf/0001-freertos-xTaskCreateRestrictedPinnedToCore.patch`**
   Identical file to `esp32_wifi_streamer`'s own patch of the same name
   (copied directly - same vendored tree, same base version, so the same
   diff applies) - adds `xTaskCreateRestrictedPinnedToCore()` to IDF's
   FreeRTOS additions layer. Not a project-invented patch - it's ESP-ADF's
   own official patch (`esp-adf/idf_patches/idf_v5.5_freertos.patch`),
   hand-applied here for the same reason documented in the sibling
   project's own patch header: `adf_install_patches.py apply-patch`'s
   `git apply` call has no error checking, so a failed match is silently
   swallowed rather than reported. Needed because `audio_pipeline` (a
   direct `REQUIRES` here too) pulls in `audio_sal/audio_thread.c`, which
   calls this function.

## Deliberately NOT included

- **`esp-adf-libs` submodule patch** (`esp_audio_codec` include-dir
  exposure) - see "Base versions" above for why this project's dependency
  closure never reaches that code at all.
- **`esp-adf/0002-m5stack_atoms3r-board-adc-channel-api-fix.patch`** and
  **`0005-http_stream-connection-reuse-and-live-prefetch.patch`** (sibling
  project's patch numbers) - both target files this project never compiles
  at all: the M5Stack AtomS3R board component (this project selects LyraT
  v4.2, not M5Stack AtomS3R) and `http_stream.c` (this project has no
  `audio_stream` HTTP usage - `REQUIRES audio_stream` here is for
  `i2s_stream`/`filter_resample` only).

## Application order

(see `build.yml`/`ci-scripts/docker-build.sh` for the actual commands)

1. Check out `esp-adf` at `eac70fd2`, init the `esp-adf-libs` AND `esp-sr`
   submodules - `esp-sr` because `components/audio_stream/CMakeLists.txt`
   (a transitive dependency via `main/CMakeLists.txt`'s own
   `REQUIRES audio_stream`) requires it unconditionally, exactly the same
   gotcha `esp32_wifi_streamer`'s own README documents; `MINIMAL_BUILD`
   does not change what an already-in-scope component itself requires.
   `esp-adf-libs` is initialized (even though no patch is needed for it -
   see above) because `filter_resample.c`/`.h` still physically live inside
   that submodule.
2. Apply all 4 `esp-adf/*.patch` files (`git apply`, run from the `esp-adf`
   checkout root) - the build script globs the directory alphabetically, so
   a new patch just needs the right filename prefix, no script change.
3. Apply `esp-idf/0001-*.patch` to the Docker image's baked-in `/opt/esp/idf`
   (run from there), then verify with a grep - same "don't trust
   `adf_install_patches.py`'s unchecked `git apply`" reasoning as the
   sibling project.
4. Only then source IDF's `export.sh` and ADF's `export.sh`.
