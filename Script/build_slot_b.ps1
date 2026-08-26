# Builds the G474 application for Slot B by relinking the common objects and generates validated metadata.
param(
  [ValidateSet("Debug", "Release")][string]$Configuration = "Debug",
  [switch]$Rebuild,
  [int]$Generation = 1
)
$ErrorActionPreference = "Stop"
$scriptDir=Split-Path -Parent $MyInvocation.MyCommand.Path
$root=Split-Path -Parent $scriptDir
$buildDir=Join-Path $root $Configuration
$buildArgs=@{Configuration=$Configuration};if($Rebuild){$buildArgs.Rebuild=$true}
& (Join-Path $scriptDir "build.ps1") @buildArgs
if($LASTEXITCODE -ne 0){throw "Common application build failed"}
Push-Location $buildDir
try {
  $elf="G474_Orion_main_slot_b.elf";$map="G474_Orion_main_slot_b.map";$bin="G474_Orion_main_slot_b.bin"
  $linkArgs=@('-o',$elf,'@objects.list','-mcpu=cortex-m4',('-T'+(Join-Path $root 'STM32G474RETX_FLASH.ld')),'--specs=nosys.specs',('-Wl,-Map='+$map),'-Wl,--gc-sections','-Wl,--defsym=APP_FLASH_ORIGIN=0x08040000','-static','--specs=nano.specs','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-mthumb','-u','_printf_float','-Wl,--start-group','-lc','-lm','-Wl,--end-group')
  & arm-none-eabi-gcc @linkArgs
  if($LASTEXITCODE -ne 0){throw "Slot B link failed"}
  & arm-none-eabi-objcopy -O binary $elf $bin
  if($LASTEXITCODE -ne 0){throw "Slot B binary conversion failed"}
  & arm-none-eabi-size $elf
} finally { Pop-Location }
$binPath=Join-Path $buildDir 'G474_Orion_main_slot_b.bin'
$metadataPath=Join-Path $buildDir 'G474_Orion_main_slot_b.metadata.bin'
& python (Join-Path $scriptDir 'generate_boot_metadata.py') $binPath $metadataPath --generation $Generation --slot b
if($LASTEXITCODE -ne 0){throw "Slot B metadata generation failed"}
Write-Output "Slot B image: $binPath"
Write-Output "Slot B metadata: $metadataPath"
