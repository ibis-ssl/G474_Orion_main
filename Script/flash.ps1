# Writes a Slot A application and its metadata to a Main board with the bootloader installed.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ArtifactName = "",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [switch]$List,
  [switch]$ConnectOnly,
  [switch]$BootloaderInstalled,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir

if ([string]::IsNullOrWhiteSpace($ArtifactName)) {
  $ArtifactName = Split-Path $repoRoot -Leaf
}

$elfPath = Join-Path $repoRoot "$Configuration\$ArtifactName.elf"

if (-not (Test-Path $ProgrammerPath)) {
  throw "STM32_Programmer_CLI.exe not found: $ProgrammerPath"
}

if ($List) {
  & $ProgrammerPath "-l" "stlink"
  if ($LASTEXITCODE -ne 0) {
    throw "ST-Link listing failed"
  }
  exit 0
}

if ($ConnectOnly) {
  & $ProgrammerPath "-c" "port=SWD mode=UR" "-rst"
  if ($LASTEXITCODE -ne 0) {
    throw "Target connection failed"
  }
  exit 0
}

if (-not (Test-Path $elfPath)) {
  throw "ELF not found: $elfPath"
}

if (-not $BootloaderInstalled) {
  throw "The application is linked for Slot A. Use install_main_bootloader.ps1 -Execute for first installation, or rerun with -BootloaderInstalled."
}

$metadataPath = Join-Path $repoRoot "$Configuration\G474_Orion_main_slot_a.metadata.bin"
if (-not (Test-Path $metadataPath)) {
  throw "Slot A metadata not found: $metadataPath. Run build_slot_a.ps1 first."
}

$args = @(
  "-c", "port=SWD mode=UR",
  "-w", $elfPath
)

if (-not $NoVerify) {
  $args += "-v"
}

& $ProgrammerPath @args
if ($LASTEXITCODE -ne 0) {
  throw "Flash failed for $Configuration"
}

$metadataArgs = @(
  "-c", "port=SWD mode=UR",
  "-w", $metadataPath, "0x08078000"
)
if (-not $NoVerify) {
  $metadataArgs += "-v"
}
if (-not $NoReset) {
  $metadataArgs += "-rst"
}

& $ProgrammerPath @metadataArgs
if ($LASTEXITCODE -ne 0) {
  throw "Slot A metadata Flash failed for $Configuration"
}

