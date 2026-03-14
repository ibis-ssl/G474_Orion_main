param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",

  [switch]$Rebuild
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

