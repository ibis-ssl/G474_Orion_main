# Builds the bootloader and Debug Slot A application, then backs up and writes all Main Flash artifacts.
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildBootloaderScript = Join-Path $scriptDir "build_bootloader.ps1"
$buildSlotAScript = Join-Path $scriptDir "build_slot_a.ps1"
$installScript = Join-Path $scriptDir "install_main_bootloader.ps1"

foreach ($path in @($buildBootloaderScript, $buildSlotAScript, $installScript)) {
  if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
    throw "Required script not found: $path"
  }
}

Write-Output "Building bootloader..."
& $buildBootloaderScript -Rebuild
if ($LASTEXITCODE -ne 0) {
  throw "Bootloader build failed"
}

Write-Output "Building Debug Slot A application and metadata..."
& $buildSlotAScript -Configuration Debug -Rebuild
if ($LASTEXITCODE -ne 0) {
  throw "Slot A application build failed"
}

Write-Output "Backing up target Flash and writing all artifacts..."
& $installScript -Configuration Debug -Execute
if ($LASTEXITCODE -ne 0) {
  throw "All-in-one Flash operation failed"
}

Write-Output "Bootloader, Debug Slot A application, and metadata were written successfully."
