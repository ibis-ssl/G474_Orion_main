# G474_Orion_main
## 全体構成（現在）


## ビルド手順（CLI）
### 前提
- STM32CubeIDE 1.17.0 がインストール済み。
- GNU Tools for STM32 が利用可能。

### Debug
```powershell
cd Debug
& "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe" -B -j4 all
```

### Release
```powershell
cd Release
& "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe" -B -j4 all
```

## 書き込み手順（CLI）
### 接続確認
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -l stlink
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -rst
```

### Debugを書き込み
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "Debug\G474_Orion_main.elf" -v -rst
```

### Releaseを書き込み
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "Release\G474_Orion_main.elf" -v -rst
```

### 補足
- `mode=UR` は Under Reset 接続。起動直後にSWDが不安定な場合に有効。
- `-v` は書き込み後ベリファイ。
- `-rst` は書き込み後にリセットを実行。

## スクリプト
`Script` 配下に、ビルドと書き込みをまとめた PowerShell スクリプトを配置している。

### ビルド
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Release -Rebuild
```

### 接続確認
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -List
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -ConnectOnly
```

### 書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Release
```

### ビルドしてから書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Release -Rebuild
```

### UARTログ受信
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM60 -BaudRate 2000000 -DurationSec 5
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM60 -BaudRate 2000000 -MaxLines 100 -LogPath .\uart_log.txt
```

## 出力成果物
- `G474_Orion_main.elf`
- `G474_Orion_main.map`
- `G474_Orion_main.list`

## 注意点
- リンカの RWX 警告は現行リンカ設定由来で、今回のリファクタリング由来ではない。
- 実機評価では、起動シーケンス・校正シーケンス・保護動作・通信周期を必ず回帰確認する。

## 参考ドキュメント
- ハードウェア仕様（コード推定）: `doc/hardware_spec.md`
- 速度制御レイヤ: `doc/hw_control.md`

## ハードウェア仕様更新メモ
- `doc/hardware_spec.md` は `G474_Orion_main.ioc` と `Core` 配下の実装を根拠に再作成した。
- 主制御は TIM7 の 500Hz（2ms）割り込みで動作する。
- 上位系 / CM4 通信は USART2 1Mbps、デバッグ出力は LPUART1 2Mbps。
- サブ基板通信は FDCAN1/FDCAN2 の Classic CAN 約1Mbps。標準 ID を全受信し、ID ごとにモータ、電源、キッカー、マウスセンサ、ボールセンサ情報を処理する。
- IMU は SPI1 接続の ICM20602。現在の制御では yaw ジャイロを主に利用している。
- 文字化けしていた旧 `hardware_spec.md` の内容は復旧せず、現行ソースから読み取れる仕様として整理した。

## 速度制御更新メモ
- `doc/hw_control.md` は速度制御レイヤの説明として、現行実装に合わせて再作成した。
- 通常走行は `maintaskRun()` 内で、上位速度指令、加速度制限、ヨー角制御、オムニホイール目標角度生成、ホイール角度制御、CAN出力の順に処理する。
- 現行の並進速度制御は実速度を直接PIDする構成ではなく、加速度制限済みの内部速度目標 `target.local_vel_now` を作り、各ホイールの角度追従で実現する。
- `linear_velocity_limit` と `SPEED_SCALAR_LIMIT` は現行速度制御では未使用のため、速度上限制御を追加する場合はここを確認する。

## エンコーダーノイズ計測
- `motor.enc_noise_rad[]` は TIM7 の 500Hz 周期で、CAN受信済み角度の周期差分 `motor.angle_diff[]` と、同じCANフレーム由来の `motor.rps[]` から期待される角度差分 `2π*rps/MAIN_LOOP_CYCLE` の絶対差を deg/cycle に変換して更新する。
- `motor.enc_noise_avg_rad[]` は 1 秒ごとの平均値、`motor.enc_noise_max_rad[]` は同じ1秒窓の最大値として更新する。変数名は既存構造体との互換のため `_rad` のままだが、格納値の単位は deg/cycle。
- LPUARTデバッグの `MOTOR` 表示に `EncNzAvgDeg` と `Max` として4輪分を deg/cycle 単位で出力する。
- この値は受信角度の揺れや、角度値と速度値の不整合を確認するための簡易指標であり、CAN周期ずれやBLDC側の速度フィルタ遅れも含む。
