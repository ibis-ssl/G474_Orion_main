# G474_Orion_main
## 全体構成（現在）

- User Flash先頭32 KBに基板専用アプリケーションブートローダーを配置するM1実装を追加した。
- 通常アプリはSlot Aの`0x08008000`へ再配置した。
- 現段階のbootloaderは安全IO、CRC32C、Slot A検証・jumpだけを行い、UART/CAN更新はまだ実装しない。
- 詳細は`doc/bootloader.md`を参照する。

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

## ST-Link接続確認（CLI）
### 接続確認
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -l stlink
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -rst
```

### 補足
- `mode=UR` は Under Reset 接続。起動直後にSWDが不安定な場合に有効。
- 通常アプリ単体をCLIで書き込むと、メタデータとの不整合が生じるため禁止する。書込みは下記スクリプトを使用する。

## スクリプト
`Script` 配下に、ビルドと書き込みをまとめた PowerShell スクリプトを配置している。

### ビルド
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Release -Rebuild
powershell -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -ExecutionPolicy Bypass -File .\Script\build_slot_a.ps1 -Configuration Debug -Rebuild
```

### 接続確認
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -List
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -ConnectOnly
```

### 書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug
# backupと安全IO表の確認後だけ実行する
powershell -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug -Execute
```

### ビルドしてから書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Debug -BootloaderInstalled
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Release -Rebuild -BootloaderInstalled
```

アプリ再配置後の`build_and_flash.ps1`は、初回導入済みの場合だけ`-BootloaderInstalled`を付ける。初回は必ず`install_main_bootloader.ps1`を使用する。

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

## CAN送信FIFO
- FDCAN1/FDCAN2のハードウェア送信FIFOが満杯の場合、各バス独立の固定長リングFIFOへ最大20フレームを退避する。
- リングFIFOは書き込み位置 `head` と読み出し位置 `tail` を分離し、格納順に送信する。送信FIFO空割り込みでは、ハードウェアFIFOの空きがある間、古いフレームから順に転送する。
- ソフトウェアFIFOに未送信フレームがある間は、新規フレームがハードウェアFIFOへ直接追い越さないよう、必ずソフトウェアFIFOの末尾へ追加する。
- FIFO満杯時は新規フレームを破棄し、`can_tx_debug.sw_dropped[]` を加算する。HAL送信登録の成功・失敗、退避回数、左右モーター指令関数の呼び出し時刻差も `can_tx_debug` で計測する。
- CAN送信ヘッダーは送信関数ごとのローカル変数とし、TIM7とFDCAN割り込みがネストした場合のCAN1/CAN2間共有競合を防止する。
- 2026-08-04の実機計測では、右前CAN1送信関数から左後CAN2送信関数までの呼び出し差は変更前が平均3us・最大14us、FIFO化後が平均3us・最大14usだった。FIFO化後の連続監視ではHAL登録エラーとソフトウェアFIFO破棄はいずれも両バス0だった。
- 同計測時は右・左モータードライバーの受信タイムアウト値がともに上限だったため、上記はMCU内部の関数呼び出し時刻差であり、CAN配線上のフレーム到達時刻差や実モーター応答差ではない。

## 直進加速診断ログ
- デバッグLPUARTで `g` を入力すると、`DRIVE_LOG` ページを250Hzで出力する。ページ選択後に `DRV_HEADER`、以後はCSV形式の `DRV` 行を出力する。
- 制御割り込み内でシーケンス番号付きスナップショットを作成し、UART側では同一制御周期の値だけをコピーする。更新と競合した場合は `DRV_RETRY` を出力する。
- 共通列は時刻、cmd_v2速度指令、ローカル最終速度目標、加速度制限中の速度目標、加速度、yaw角、yaw実角速度、yaw目標角速度、yaw減衰値。
- 各モーター0～3について、目標rps、実rps、ドライバー電源ライン電流、角度誤差、rps誤差、Kp項、Kd項、FF項、yaw項、最終出力、最終CAN受信からの経過時間を出力する。
- 電流列 `current0_A`～`current3_A` はCAN ID `0x230`～`0x233` の受信値をA単位、小数1桁で出力する。ドライバー側の分解能が0.1Aのため、細かな電流差ではなく加速開始時の左右差や接地負荷差の傾向確認に使用する。
- 末尾の `real_rf_lf` は `real_rps[0] + real_rps[3]`、`real_rb_lb` は `real_rps[1] + real_rps[2]`。前進時は左輪が負回転なので、正値は右側が速く、負値は左側が速いことを示す。`out_rf_lf` と `out_rb_lb` も同じ符号規則の出力差。
- ログ保存例: `powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM167 -BaudRate 2000000 -LogPath .\drive_log.txt`

## デバッグUART表示ページ
- 表示ページ名と表示周期は `Core/Src/main.c` の `print_page_config[]` に集約している。
- `0`～`9` は従来どおり対応するページを直接選択し、Enterで次ページ、その他の未割当キーで前ページへ移動する。Deleteは `AI_CMD`、`g`は `DRIVE_LOG`へ移動する。
- `q`/`a`はオムニ角度Kp、`w`/`s`はKdを増減するため、ページ移動には使用しない。

## 静止時のオムニ角度誤差クリア
- `clearOmniRotationAngleErrorIfStopped()` は、4輪の角度誤差をオムニホイールの逆運動学に基づいて並進成分と機体回転成分へ分離し、機体回転成分だけを内部目標角度から除去する。並進方向の角度誤差は維持する。
- 最終ローカル速度指令と内部速度目標のXY各成分が `0.01m/s` 未満、各輪目標が `0.05rps` 未満、目標機体角度とIMU yawの誤差が±1degの不感帯内、IMU yaw角速度と目標yaw角速度が `0.05rad/s` 未満、CAN受信間隔が4ms以下の状態が2制御周期（4ms）連続した場合に補正する。
- 実車輪速度はクリア条件に使用しない。機体回転はIMU yaw角速度で判定し、個々のエンコーダー速度ノイズによって補正機会を失わないようにする。
- 停止時に4輪出力を0にする整定状態は使用しない。条件成立中は回転角度誤差を時定数50msの一次遅れで減衰させ、1制御周期の補正量を最大 `0.001rad`（`0.5rad/s`）に制限する。残差が `0.0005rad` 未満になった場合だけ残りを同期する。
- 条件が外れた場合は成立カウントと `angle_clear_active` を即座に解除し、その周期の補正量を0にする。再び2周期連続成立すれば、その時点の残差から補正を再開する。
- `DRIVE_LOG` では、500Hz制御で `angle_clear_count=2` かつ `angle_clear_active=1` が回転角度誤差の補正中を示す。`rotation_angle_error` は補正前の回転成分、`rotation_clear_step` はその周期に各輪目標角度から差し引いた補正量をrad単位で示す。
