param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ArtifactName = "",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [switch]$List,
  [switch]$ConnectOnly,
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

$args = @(
  "-c", "port=SWD mode=UR",
  "-w", $elfPath
)

if (-not $NoVerify) {
  $args += "-v"
}

if (-not $NoReset) {
  $args += "-rst"
}

& $ProgrammerPath @args
if ($LASTEXITCODE -ne 0) {
  throw "Flash failed for $Configuration"
}

