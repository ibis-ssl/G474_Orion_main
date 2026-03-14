param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ArtifactName = "",

  [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildScript = Join-Path $scriptDir "build.ps1"
$flashScript = Join-Path $scriptDir "flash.ps1"

if (-not (Test-Path $buildScript)) {
  throw "build.ps1 not found: $buildScript"
}

if (-not (Test-Path $flashScript)) {
  throw "flash.ps1 not found: $flashScript"
}

$buildArgs = @{
  Configuration = $Configuration
}

if ($Rebuild) {
  $buildArgs.Rebuild = $true
}

& $buildScript @buildArgs
if ($LASTEXITCODE -ne 0) {
  throw "Build step failed"
}

$flashArgs = @{
  Configuration = $Configuration
}

if (-not [string]::IsNullOrWhiteSpace($ArtifactName)) {
  $flashArgs.ArtifactName = $ArtifactName
}

& $flashScript @flashArgs
if ($LASTEXITCODE -ne 0) {
  throw "Flash step failed"
}
