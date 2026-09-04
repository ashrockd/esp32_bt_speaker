#!/usr/bin/env bash
#
# Runs INSIDE the espressif/idf:v5.5.5 container (see ../workflows/build.yml,
# which invokes this with `--entrypoint bash`) - sibling to
# esp32_wifi_streamer/.github/ci-scripts/docker-build.sh, same mechanism,
# adapted for this project's target chip (esp32, not esp32s3) and smaller
# dependency closure (no esp-adf-libs submodule patch needed - see
# ci-patches/README.md).
#
# Reproduces, in order, what a human runs locally via esp32_bt_speaker/
# build.ps1, minus the interactive/flash-specific parts:
#   1. activate ESP-IDF (build.ps1: ". $IdfProfile")
#   2. activate ESP-ADF (build.ps1: ". $AdfExport")
#   3. force a clean sdkconfig regen from sdkconfig.defaults
#   4. idf.py set-target esp32
#   5. idf.py build

set -euo pipefail

WORKSPACE="/workspace"
PROJECT_DIR="$WORKSPACE/esp32_bt_speaker"
ADF_DIR="$WORKSPACE/esp-adf"
PATCH_DIR="$PROJECT_DIR/.github/ci-patches"

# Same "dubious ownership" issue as the sibling project's script - see its
# own comment for the full CVE-2022-24765 explanation. Safe to blanket-trust
# in this throwaway, single-run CI container.
git config --global --add safe.directory '*'

echo "::group::Versions"
echo "Container IDF_PATH (baked-in, image-provided): ${IDF_PATH:-<unset>}"
git -C "$IDF_PATH" describe --tags --always 2>&1 || echo "(no .git metadata in this image variant - version is still pinned by the image tag itself, espressif/idf:v5.5.5)"
git -C "$ADF_DIR" rev-parse HEAD
git -C "$ADF_DIR" describe --tags --always
echo "::endgroup::"

# --- Step 0: apply this project's captured local patches -------------------
# See ci-patches/README.md for exactly what each patch does and why - the
# critical one is the LyraT v4.2 I2S pin override (patch #1): an unpatched
# esp-adf checkout compiles and boots fine, it just silently listens on the
# wrong GPIOs, no compile error, no runtime error.
echo "::group::Applying esp-adf local patches"
for p in "$PATCH_DIR"/esp-adf/*.patch; do
    echo "Applying $(basename "$p")"
    git -C "$ADF_DIR" apply --whitespace=nowarn "$p"
done
echo "::endgroup::"

# The freertos patch targets the CONTAINER'S OWN baked-in /opt/esp/idf, not a
# separately-checked-out esp-idf tree - see the sibling project's
# ci-patches/README.md "Which ESP-IDF actually matters" for the full
# reasoning (same standalone v5.5.5 install activation order applies here).
echo "::group::Applying esp-idf (container-baked v5.5.5) FreeRTOS patch"
git -C "$IDF_PATH" apply --whitespace=nowarn \
    "$PATCH_DIR/esp-idf/0001-freertos-xTaskCreateRestrictedPinnedToCore.patch"

if ! grep -q "xTaskCreateRestrictedPinnedToCore" \
        "$IDF_PATH/components/freertos/esp_additions/include/freertos/idf_additions.h"; then
    echo "::error::xTaskCreateRestrictedPinnedToCore is missing from esp-idf's" \
         "idf_additions.h after applying the FreeRTOS patch. ESP-ADF's" \
         "audio_sal/audio_thread.c calls this function - continuing would" \
         "produce a confusing 'undefined reference' LINK error far from this" \
         "root cause, so failing fast here instead."
    exit 1
fi
echo "FreeRTOS patch verified present."
echo "::endgroup::"

# --- Step 1+2: activate ESP-IDF, then ESP-ADF (build.ps1's own order) ------
#
# MINIMAL_BUILD=1, set BEFORE either activation step, exactly where build.ps1
# sets it - restricts CMake's component discovery to main's actual REQUIRES
# closure (audio_pipeline, audio_stream, bt, driver, bluetooth_service - see
# main/CMakeLists.txt) instead of scanning esp-adf's entire components/ tree.
export MINIMAL_BUILD=1
echo "MINIMAL_BUILD=$MINIMAL_BUILD"

echo "::group::Activating ESP-IDF"
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh"
echo "::endgroup::"

# ADF_PATH must be set before esp-adf's export.sh AND before the project's own
# top-level CMakeLists.txt (which FATAL_ERRORs if ADF_PATH is unset).
export ADF_PATH="$ADF_DIR"
echo "::group::Activating ESP-ADF"
# shellcheck disable=SC1091
#
# This ALSO runs tools/adf_install_patches.py apply-patch internally (the
# last lines of esp-adf/export.sh) - harmless here, same reasoning as the
# sibling project's script: it will try to git-apply esp-adf's own
# idf_v5.5_freertos.patch on top of a tree that already has our equivalent
# hand-verified patch applied above, fail to match, and get silently ignored
# by that script's own unchecked subprocess.run() call.
source "$ADF_DIR/export.sh"
echo "::endgroup::"

if ! command -v idf.py >/dev/null 2>&1; then
    echo "::error::idf.py not on PATH after activation - environment setup failed."
    exit 1
fi

# --- Step 3: force a clean sdkconfig regen, exactly like build.ps1 ---------
cd "$PROJECT_DIR"
echo "::group::Regenerating sdkconfig from sdkconfig.defaults"
rm -f sdkconfig
echo "::endgroup::"

echo "::group::idf.py set-target esp32"
idf.py set-target esp32
echo "::endgroup::"

echo "::group::idf.py build"
idf.py build
echo "::endgroup::"

echo "Build succeeded."
