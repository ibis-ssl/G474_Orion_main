param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",

  [string]$ToolchainBin = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin",

  [switch]$Rebuild,

  [int]$Generation = 1
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildDir = Join-Path $repoRoot $Configuration

if (-not (Test-Path $MakePath)) {
  throw "make.exe not found: $MakePath"
}

if (-not (Test-Path $buildDir)) {
  throw "Build directory not found: $buildDir"
}

$makeArgs = @()
if ($Rebuild) {
  $makeArgs += "-B"
}
$makeArgs += "-j4"
$makeArgs += "all"

Push-Location $buildDir
try {
  & $MakePath @makeArgs
  if ($LASTEXITCODE -ne 0) {
    throw "Build failed for $Configuration"
  }
} finally {
  Pop-Location
}

$elfPath = Join-Path $buildDir "G474_Orion_main.elf"
$binPath = Join-Path $buildDir "G474_Orion_main.bin"
$objcopy = Join-Path $ToolchainBin "arm-none-eabi-objcopy.exe"
& python (Join-Path $scriptDir "stamp_fw_version.py") $elfPath --objcopy $objcopy --repo $repoRoot --target main_slot_a --log-dir (Join-Path $scriptDir "Logs\Build")
if ($LASTEXITCODE -ne 0) { throw "FW version stamping failed" }
& $objcopy -O binary $elfPath $binPath
if ($LASTEXITCODE -ne 0) { throw "Binary regeneration failed" }
$metadataPath = Join-Path $buildDir "G474_Orion_main_slot_a.metadata.bin"
& python (Join-Path $scriptDir "generate_boot_metadata.py") $binPath $metadataPath --generation $Generation --slot a
if ($LASTEXITCODE -ne 0) { throw "Slot A metadata generation failed" }
Write-Output "Slot A image: $binPath"
Write-Output "Slot A metadata: $metadataPath"

