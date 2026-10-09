# Generates VS Code compilation databases from the application and bootloader Makefiles without building or flashing.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",
  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",
  [string]$ToolchainBin = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$compiler = Join-Path $ToolchainBin "arm-none-eabi-gcc.exe"
if (-not (Test-Path -LiteralPath $MakePath)) { throw "make.exe not found: $MakePath" }
if (-not (Test-Path -LiteralPath $compiler)) { throw "Compiler not found: $compiler" }

$entries = @(foreach ($directory in @($Configuration, "Bootloader")) {
  $buildDir = Join-Path $repoRoot $directory
  # -n prints recipes; -B includes sources whose objects are already up to date.
  $recipes = & $MakePath -C $buildDir -B -n --no-print-directory all 2>&1
  if ($LASTEXITCODE -ne 0) { throw "Cannot read $directory build commands: $recipes" }
  $count = 0
  foreach ($line in $recipes) {
    if ($line -notmatch '^arm-none-eabi-gcc\s') { continue }
    $arguments = @([regex]::Matches([string]$line, '(?:[^\s"]+|"[^"]*")+') | ForEach-Object {
      $_.Value.Replace('"', '')
    })
    $source = @($arguments | Where-Object { $_ -match '\.c$' })
    if ($source.Count -ne 1 -or $arguments -notcontains '-c') { continue }
    $arguments[0] = $compiler.Replace('\', '/')
    [ordered]@{
      directory = $buildDir.Replace('\', '/')
      file = [IO.Path]::GetFullPath((Join-Path $buildDir $source[0])).Replace('\', '/')
      arguments = $arguments
    }
    $count++
  }
  if ($count -eq 0) { throw "No C compilation commands found for $directory" }
})

$outputPath = Join-Path $repoRoot ".vscode\compile_commands.$Configuration.json"
$json = ConvertTo-Json -InputObject $entries -Depth 5
[IO.File]::WriteAllText($outputPath, $json, (New-Object Text.UTF8Encoding($false)))
Write-Output "Compilation database: $outputPath ($($entries.Count) C files)"
