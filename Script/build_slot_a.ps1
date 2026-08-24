# Builds the G474 application for Slot A and generates validated CRC32C metadata.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",
  [switch]$Rebuild,
  [int]$Generation = 1
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildScript = Join-Path $scriptDir "build.ps1"
$metadataScript = Join-Path $scriptDir "generate_boot_metadata.py"
$buildDir = Join-Path $repoRoot $Configuration
$binPath = Join-Path $buildDir "G474_Orion_main.bin"
$mapPath = Join-Path $buildDir "G474_Orion_main.map"
$metadataPath = Join-Path $buildDir "G474_Orion_main_slot_a.metadata.bin"

$buildArgs = @{ Configuration = $Configuration }
if ($Rebuild) {
  $buildArgs.Rebuild = $true
}
& $buildScript @buildArgs
if ($LASTEXITCODE -ne 0) {
  throw "Slot A application build failed"
}

if (-not (Test-Path $binPath) -or -not (Test-Path $mapPath)) {
  throw "Slot A build artifacts are missing"
}

$flashLine = Select-String -Path $mapPath -Pattern '^FLASH\s+0x08008000\s+0x00038000' | Select-Object -First 1
if ($null -eq $flashLine) {
  throw "Link map is not configured for Slot A (0x08008000, 224KB)"
}

& python $metadataScript $binPath $metadataPath --generation $Generation
if ($LASTEXITCODE -ne 0) {
  throw "Slot A metadata generation failed"
}

if ((Get-Item $binPath).Length -gt 0x38000) {
  throw "Slot A image exceeds 224KB"
}

Write-Output "Slot A image: $binPath"
Write-Output "Slot A metadata: $metadataPath"
