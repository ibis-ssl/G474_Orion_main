# Backs up Main Flash and writes the bootloader, Slot A, and metadata only with -Execute.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",
  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
  [string]$BackupDirectory = "",
  [switch]$Execute
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$bootloaderHex = Join-Path $repoRoot "Bootloader\Build\G474_Orion_bootloader.hex"
$applicationElf = Join-Path $repoRoot "$Configuration\G474_Orion_main.elf"
$metadataBin = Join-Path $repoRoot "$Configuration\G474_Orion_main_slot_a.metadata.bin"
$crcScript = Join-Path $scriptDir "orion_crc32c.py"

if ([string]::IsNullOrWhiteSpace($BackupDirectory)) {
  $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
  $BackupDirectory = Join-Path $scriptDir "Logs\bootloader_install_$timestamp"
}
$resolvedBackupParent = [System.IO.Path]::GetFullPath((Split-Path -Parent $BackupDirectory))
$resolvedScriptLogs = [System.IO.Path]::GetFullPath((Join-Path $scriptDir "Logs"))
if (-not $resolvedBackupParent.StartsWith($resolvedScriptLogs, [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "BackupDirectory must be below $resolvedScriptLogs"
}

foreach ($path in @($ProgrammerPath, $bootloaderHex, $applicationElf, $metadataBin, $crcScript)) {
  if (-not (Test-Path $path)) {
    throw "Required file not found: $path"
  }
}

New-Item -ItemType Directory -Path $BackupDirectory -Force | Out-Null
$flashBackup = Join-Path $BackupDirectory "main_flash_before_bootloader.bin"
$optionBackup = Join-Path $BackupDirectory "option_bytes_before_bootloader.txt"
$crcBackup = Join-Path $BackupDirectory "main_flash_before_bootloader.crc32c.txt"

$connectOutput = & $ProgrammerPath -c "port=SWD mode=HOTPLUG" 2>&1
if ($LASTEXITCODE -ne 0 -or ($connectOutput -join "`n") -notmatch 'Device ID\s+: 0x469') {
  throw "Expected STM32G47x target was not detected"
}

$optionOutput = & $ProgrammerPath -c "port=SWD mode=HOTPLUG" -ob displ 2>&1
if ($LASTEXITCODE -ne 0) {
  throw "Option Bytes read failed"
}
$optionOutput | Set-Content -Encoding UTF8 $optionBackup
if (($optionOutput -join "`n") -notmatch 'DBANK\s+: 0x1') {
  throw "DBANK dual-bank mode is not enabled"
}

& $ProgrammerPath -c "port=SWD mode=HOTPLUG" -u 0x08000000 0x80000 $flashBackup
if ($LASTEXITCODE -ne 0 -or (Get-Item $flashBackup).Length -ne 0x80000) {
  throw "512KB Flash backup failed"
}
$backupCrc = & python $crcScript $flashBackup
if ($LASTEXITCODE -ne 0 -or $backupCrc -notmatch '^0x[0-9A-F]{8}$') {
  throw "Flash backup CRC32C calculation failed"
}
$backupCrc | Set-Content -Encoding ASCII $crcBackup

Write-Output "Flash backup: $flashBackup"
Write-Output "Flash backup CRC32C: $backupCrc"
Write-Output "Option Bytes backup: $optionBackup"
Write-Output "Bootloader: $bootloaderHex"
Write-Output "Slot A app: $applicationElf"
Write-Output "Metadata: $metadataBin -> 0x08078000"

if (-not $Execute) {
  Write-Output "Dry run completed. No Flash write or reset was performed."
  Write-Output "Run again with -Execute only after reviewing the backup and safe IO table."
  exit 0
}

& $ProgrammerPath -c "port=SWD mode=UR" -w $bootloaderHex -v
if ($LASTEXITCODE -ne 0) { throw "Bootloader write failed" }
& $ProgrammerPath -c "port=SWD mode=UR" -w $applicationElf -v
if ($LASTEXITCODE -ne 0) { throw "Slot A application write failed" }
& $ProgrammerPath -c "port=SWD mode=UR" -w $metadataBin 0x08078000 -v -rst
if ($LASTEXITCODE -ne 0) { throw "Metadata write or target reset failed" }

Write-Output "Initial bootloader installation and verification completed."
