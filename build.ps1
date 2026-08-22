<#
  build.ps1 - one-shot ESP32-WROOM-32U (4MB flash, no PSRAM) build script
  for esp32_bt_speaker - the Bluetooth half of the two-chip split. Reuses
  the ESP-ADF/ESP-IDF checkout vendored one level up in ..\esp-adf rather
  than duplicating it.

  Usage:
    powershell -NoExit -ExecutionPolicy Bypass -File "C:\Users\Ashish\Github\esp32_bt_int_radio\esp32_bt_speaker\build.ps1"
    powershell -NoExit -ExecutionPolicy Bypass -File "...\build.ps1" -Flash -Port COM7
    powershell -NoExit -ExecutionPolicy Bypass -File "...\build.ps1" -Clean
#>

param(
    [switch]$Clean,          # wipe build/ and managed_components/ before building (use after dependency/space issues)
    [switch]$Flash,          # flash + monitor after a successful build
    [string]$Port = "COM7"   # serial port to use with -Flash - NOTE: pick the port for THIS board, not the streamer's
)

$ErrorActionPreference = "Stop"

$ProjectRoot = "C:\Users\Ashish\Github\esp32_bt_int_radio\esp32_bt_speaker"
$AdfExport   = "C:\Users\Ashish\Github\esp32_bt_int_radio\esp-adf\export.ps1"

if (-not (Test-Path $AdfExport)) {
    Write-Error "esp-adf export.ps1 not found at $AdfExport"
    exit 1
}

Set-Location $ProjectRoot

$IdfProfile = "C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1"
if (-not (Test-Path $IdfProfile)) {
    Write-Error "IDF profile script not found at $IdfProfile"
    exit 1
}

# Only build the ESP-ADF components this project's CMakeLists.txt actually
# REQUIRES (no `esp_wifi`/`esp_http_client`/`mbedtls`/`json` at all on this
# chip), instead of ESP-ADF's entire component tree.
$env:MINIMAL_BUILD = "1"

Write-Host "== Activating ESP-IDF v5.5.5 (official profile) ==" -ForegroundColor Cyan
. $IdfProfile

Write-Host "== Activating ESP-ADF (ADF_PATH + patches) ==" -ForegroundColor Cyan
. $AdfExport

if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    Write-Error "idf.py not on PATH after activation - environment setup failed, see output above."
    exit 1
}

if ($Clean) {
    Write-Host "== Clean requested: removing build/, managed_components/, dependencies.lock ==" -ForegroundColor Yellow
    Remove-Item -Recurse -Force "build" -ErrorAction SilentlyContinue
    Remove-Item -Recurse -Force "managed_components" -ErrorAction SilentlyContinue
    Remove-Item -Force "dependencies.lock" -ErrorAction SilentlyContinue
}

# `idf.py set-target` unconditionally triggers a fullclean in stock ESP-IDF
# regardless of whether the target actually changed - so only run it (and
# only delete sdkconfig, which is what forces set-target to regenerate it
# from sdkconfig.defaults) when there's an actual reason to: -Clean was
# requested, sdkconfig doesn't exist yet, or sdkconfig.defaults was edited
# more recently than the sdkconfig it produced. Otherwise skip straight to
# `idf.py build`, which does a real incremental ninja/ccache build. This
# machine is a 2013 dual-core Pentium with no hyperthreading - a full
# 1612-step rebuild on every single invocation, even for a one-line source
# change, is not something to default to.
$sdkconfigPath = Join-Path $ProjectRoot "sdkconfig"
$defaultsPath  = Join-Path $ProjectRoot "sdkconfig.defaults"
$needsRegen = $Clean -or
              (-not (Test-Path $sdkconfigPath)) -or
              ((Get-Item $defaultsPath).LastWriteTime -gt (Get-Item $sdkconfigPath).LastWriteTime)

if ($needsRegen) {
    Write-Host "== sdkconfig missing/stale/-Clean - regenerating from sdkconfig.defaults (this forces a fullclean) ==" -ForegroundColor Cyan
    Remove-Item -Force $sdkconfigPath -ErrorAction SilentlyContinue
    idf.py set-target esp32
    if ($LASTEXITCODE -ne 0) { Write-Error "set-target failed"; exit $LASTEXITCODE }
} else {
    Write-Host "== sdkconfig up to date with sdkconfig.defaults - skipping set-target/fullclean ==" -ForegroundColor DarkGray
}

Write-Host "== Building ==" -ForegroundColor Cyan
idf.py build
if ($LASTEXITCODE -ne 0) { Write-Error "build failed"; exit $LASTEXITCODE }

Write-Host "== Build succeeded ==" -ForegroundColor Green

if ($Flash) {
    Write-Host "== Flashing + monitoring on $Port ==" -ForegroundColor Cyan
    idf.py -p $Port flash monitor
}
