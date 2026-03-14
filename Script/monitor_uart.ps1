param(
  [string]$Port = "COM60",
  [int]$BaudRate = 2000000,
  [int]$DurationSec = 0,
  [int]$MaxLines = 0,
  [string]$LogPath = ""
)

$ErrorActionPreference = "Stop"

$serialPort = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 500
$serialPort.NewLine = "`n"

$writer = $null
if ($LogPath -ne "") {
  $writer = New-Object System.IO.StreamWriter($LogPath, $false, [System.Text.Encoding]::UTF8)
  $writer.AutoFlush = $true
}

try {
  $serialPort.Open()
  $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
  $lineCount = 0

  while ($true) {
    if ($DurationSec -gt 0 -and $stopwatch.Elapsed.TotalSeconds -ge $DurationSec) {
      break
    }

    if ($MaxLines -gt 0 -and $lineCount -ge $MaxLines) {
      break
    }

    try {
      $line = $serialPort.ReadLine().TrimEnd("`r", "`n")
      if ([string]::IsNullOrWhiteSpace($line)) {
        continue
      }

      Write-Output $line
      if ($writer -ne $null) {
        $writer.WriteLine($line)
      }

      $lineCount++
    } catch [System.TimeoutException] {
    }
  }
} finally {
  if ($writer -ne $null) {
    $writer.Dispose()
  }

  if ($serialPort.IsOpen) {
    $serialPort.Close()
  }
}
