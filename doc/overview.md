# G474_Orion_main
## CM4 UART通信の競合レビュー（2026-09-19）

以下は変更前のソースに対する静的レビュー結果。実機検証でTCIEの更新を保護した事例を確認し、現在は案1の標準ATOMIC_SET_BITを4か所に適用済み。通信不安定の全原因がこの競合だけかは未確定。

- USART2は1 Mbps・8N1、TXはDMA1 Channel8の通常モード。NVICはPRIORITYGROUP_4、DMA1 Channel8は優先度0、USART2は1、テレメトリを起動するTIM7は8。DMA転送優先度LOWはNVIC優先度とは別。
- `stm32g4xx_it.c`のUSART2ハンドラー末尾はCR3.EIEとCR1.RXNEIE_RXFNEIEを通常の`SET_BIT`で設定する。`main.c`の受信開始時も同じ。HALのDMA送信完了処理はCR3.DMATをクリアしてCR1.TCIEをセットし、USART2のTC処理で初めてgStateをREADYに戻す。
- 競合の順序は「USART2側がCR1を読む → DMA完了IRQが横取りしてTCIEをセット → USART2側が古いCR1にRXNEIEを加えて書き戻し、TCIEを消す」。送信済みでもgStateがBUSY_TXに残り、後続送信はHAL_BUSYとなり得る。CR3側ではDMA完了でクリアしたDMATを復活させ得る。HAL側だけがatomicでも、呼び出し側の通常のread-modify-writeは保護されない。
- **案1を推奨**：上記4か所を`ATOMIC_SET_BIT`へ変更する。CMSIS実装はLDREXW/STREXWによる再試行であり、割り込みを一律に禁止せず更新競合を防ぐ。定常通信で直接問題になるのはUSART2ハンドラーの2か所。main側はテレメトリ開始前の初期化で同じTX完了競合の主要経路ではないが、統一して保護するのが妥当。実行時間の増加量は未測定。
- **案2は代替策**：DMA IRQをUSART2より低くする（例：数値2）と当該横取りを防げる。同じ数値1でも相互に横取りしないため有効。DMA側は既にatomicなので逆方向の横取りにも対応する。ただし他IRQとの関係やDMA完了通知の遅延が変わる。採用時は`dma.c`だけでなく`.ioc`も更新する。mainのSET_BIT全般を保護する策ではなく、案1実施後にこの競合だけを理由として併用する必要はない。
- **案3は条件付きで有効**：`ai_comm.c`のsendRobotInfoはHAL_UART_Transmit_DMAの戻り値を無視している。HAL_UART_AbortTransmitは送信IRQ/DMA要求を止め、必要に応じてDMA停止とTX FIFO破棄を行い、gStateをREADYへ戻すため、今回のTCIE消失から復旧できる。受信停止は行わないが、送信途中ならフレーム欠落が起こる。搭載HALのHAL_DMA_Abortには待機ループはない。復旧処理とDMA完了IRQの競合を含め、実装時に実行コンテキストを整理する。
- 案3では単発HAL_BUSYでabortせず、正常な送信時間とIRQ遅延を超える継続時間で判定する。128 byteは1 Mbps・8N1で約1.28 ms、テレメトリは目標125 Hz（実際の試行タイミングはprint_timing等で変わる）。送信成功時刻、BUSY開始時刻、完了回数、HAL_BUSY/HAL_ERROR/abort回数を区別し、正常完了で監視状態をリセットする。HAL_UART_GetStateはRxStateも合成するためTX判定にはgState等を使う。
- 現状はstatic送信バッファを作り直してからHALへ渡すため、前回DMAが進行中だとHAL_BUSYを返す前にデータを破壊し得る。バッファ更新前の送信可否判定が必要。同じUARTを使うFWバージョン応答・FW更新応答の正規送信をabortしないよう所有者も区別する。テレメトリ無効時やFW更新中はsendRobotInfoが呼ばれず、ここだけの監視では全経路の自己回復にならない。
- 実機検証では停止時のgState、CR1.TCIE、CR3.DMAT、ISR.TC、DMA残数CNDTRとDMA状態を退避する。`BUSY_TX / TCIE=0 / TC=1 / CNDTR=0`の持続は仮説と整合するが、それだけで原因確定とはしない。案1だけを適用して双方向連続通信を比較し、既存RX ORE/FE/NE/PEカウンターも確認する。案3は別試験で送信完了通知欠落を模擬し、復旧後の次フレーム送信と受信継続を確認する。当該競合が直接説明するのはSTM32からCM4へのTX停止であり、RXのみの停止は別途切り分ける。

### 実機検証結果と正式修正

ユーザー提供の実機ログでは、`IRQ1=182702/10/2 IRQ3=182702/1/0 INIT1=1/0/0 INIT3=1/0/0`を確認した。各組は実行回数／再試行した呼び出し回数／対象ビットの更新を保護した回数。

- CR1で2回、最初の読み取りではTCIE=0、再試行後の書き込み成功時の読み取りではTCIE=1となった。古い値を書き戻していれば失われるTCIEをatomic処理が保持した実績である。DMA IRQの発生元照合は行っていない。
- CR3は再試行1回、DMAT復活防止の検出は0回。同ログの受信フレームは2400/2400有効、チェックサムエラー・PE/FE/NE/OREはすべて0だった。
- 正式修正は`stm32g4xx_it.c`のUSART2 IRQ末尾と`main.c`の受信開始時のCR1/CR3、計4か所を標準の`ATOMIC_SET_BIT`にする。DMA完了IRQによるDMATクリアとTCIEセットを古い値で上書きしないためである。
- 検証用ヘッダー、4組のカウンター、デバッグUARTのATOM表示は削除済み。既存のUART受信診断表示は維持する。IRQ優先度変更やabort復旧は追加していない。
- 計測付き実装の結果は対策の有効性を示すが、計測によるタイミング変化はある。正式修正後も通信停止の再発有無を継続確認する。

## 全体構成（現在）

- User Flash先頭32 KBに基板専用アプリケーションブートローダーを配置するM1実装を追加した。
- 通常アプリはSlot Aの`0x08008000`へ再配置した。
- Main自身のbootloaderは安全IO、CRC32C、Slot A検証・jumpを担当する。通常アプリの`fw_update_gateway.c`はCM4の`OFW2`要求を受信し、CAN1/CAN2上のF303ブートローダーへ配信する。
- Gatewayの`ENTER`ではCAN1/CAN2に別々のnode IDを指定できる。未使用バスは`0xFF`とし、左右BLDCではnode 16/17へ同じdata frameを両FDCANから並列送信する。command応答は各バスの`0x650 + node ID`で個別確認し、statusとcommit offsetが一致した場合だけ成功とする。
- 更新中は通常制御周期とブザーPWMを停止する。全CANノード更新はCM4側で全対象を先にbootloaderへ移し、全imageの確定後にまとめて再起動する。
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

ブートローダーとDebug版Slot Aアプリを再ビルドし、Flash全体を退避してから、ブートローダー、アプリ、metadataを一括で書き込む場合は次を使用する。`flash_all.ps1`は確認オプションなしで直ちに書き込みまで実行するため、対象基板と安全状態を確認してから起動する。

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash_all.ps1
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

## BLDC並列FW更新

- 2026-08-26にCM4→Main→CAN1 node 16/CAN2 node 17の実機並列更新を確認した。
- データフレームは両バスへ同時送信し、BLOCK_BEGIN、BLOCK_END、HELLOの応答、確定offset、再送対象はバスごとに管理する。片側だけ書込み済みの場合は成功側を除外し、未完了側だけをchunk再送する。
- 63,592 byteの更新は通常13.965秒、UART CRC破損とCAN欠落・重複・逆順・payload破損の複合注入時14.047秒で完了した。
- 更新中はTIM5のブザーPWMを停止し、全対象の確定後に再起動する。
- 2026-08-27、左右2台の同時更新を10回連続実施し10/10成功した。14.885～19.591秒、全回CRC32C `0xC22DAE9C`一致。UART応答欠落によるchunk再送88回もすべて回復した。
- 両BLDCはmetadata CONFIRMED、VTOR `0x08004000`、Flash設定のboard ID 0/1保持をST-Linkで確認した。

## Main A/B FW更新

- CM4の`main_ab_updater.py`から非稼働slotへOFW1で書込み、PENDING起動後にCONFIRMする。
- 通常USART2受信もIRQ内でRX FIFOを全量drainし、72-byte FWUP要求と更新モード切替直後の長いOFW frameを取りこぼさない。
- 2026-08-27の最終往復はB→Aが9.808秒、A→Bが9.796秒。Slot A generation 8、Slot B generation 9がともにCONFIRMED、boot attempts 0を確認した。

## CM4 USART2割り込み受信の安定化

- USART2は1 Mbps、RX FIFO有効、threshold 1/8で使用する。起動処理で`HAL_UART_Init()`を二重に呼ぶとFIFOENが消えるため、初期化は`MX_USART2_UART_Init()`だけで行う。
- RXはHALの1-byte受信状態機械を使わず、RXNE/RXFNE割り込みを直接常時有効にする。ISRはFIFOをdrainして2 KBリングへ格納し、parserはmain loopで実行する。ISR末尾でRX interruptとerror interruptを再有効化する。
- FW gateway中は通常robot telemetryのUSART2 DMA送信を停止し、OFW2応答との競合を防ぐ。
- partial frame timeoutでは`uart_last_byte_tick`を先にsnapshotし、その後`HAL_GetTick()`を取得する。判定直前にsnapshotと現在の`uart_last_byte_tick`が一致する場合だけresetする。逆順に評価すると、その間のRX IRQ更新でunsigned減算がunderflowし、受信途中のframeを誤resetする。
- 診断用にISR/queueのbyte countとrolling hash、FIFO drain最大数、ORE/FE/NE/PE、queue overflow、magic/header/frame、CRC、parser timeout、response成功/失敗をRAMへ保持する。
- 2026-08-28、CM4をPL011（`/dev/serial0 -> ttyAMA0`）へ切替後、923-byte最大frameを3,000/3,000で初回応答成功した。median 14.891 ms、p95 16.937 ms、max 17.832 ms。CM4経由でB 10.108秒、A 9.932秒の更新後も1,000/1,000成功した。
- 最終A/Bは同一build ID `1787928269`。Slot A CRC32C `23F1D426`、Slot B `CF4FCD66`、active slot A。

## 開発用FW識別

- Slot A/Bの各アプリ先頭`+0x400`へ`FWVR` magicとUnix秒build IDを配置する。`build_slot_b.ps1`はA/Bを同じbuild IDで生成し、両slotのmetadataも必ず再生成する。
- UARTの72 byte `FWVR`要求を受けると、両slotを直接読出し、CAN ID `0x611`でSub、左右BLDC、Powerへ同時照会する。未応答nodeだけ40 ms間隔で再照会し、250 ms後に6対象のbuild IDとimage CRC32Cを60 byte応答としてCM4へ返す。
- 各ビルドは`Script/Logs/Build/`へGit hashとdirty状態をJSON保存する。2026-08-27にA/B更新後のUART応答と両CANバス集約を実機確認した。

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

## CM4_105 Power更新の復旧（2026-09-16）

- Power更新で3584 byte地点に欠落・順序異常（node status 3）が再現したため、fw_update_gateway.cにPower対象時のみ8 CAN frameごとの1 ms待機を追加した。
- A/B build成功。build ID 1789484648、各84,500 byte、CRC32C A=AEC55D28、B=DB0D0D01。CM4経由のB更新9.652秒、A更新9.972秒、最終active A。両スロットのbuild IDとCRCが配布ファイルに一致した。
- 修正後のPower更新は13.244秒で成功し、CRC32C A07D08B0一致、Sub・左右BLDCを含む全基板のversion応答復帰を確認した。UART再送は発生した。受信FIFO overflow自体は直接計測していない。

## CM4 UART送受信競合の切り分け（2026-09-19）

- COM57のUART RAW画面に、正常フレーム受信からの経過時間、USART2 IRQ/byte数、FIFO最大drain数、PE/FE/NE/ORE数を表示する。
- UART RAW画面では全USART2割り込み数とは別に、RX byteを実際にdrainした割り込み数、72 byte組立完了数、チェックサム正常数も表示し、物理受信とパーサ結果を比較できる。
- COM57から`t`を送信すると、MainからCM4への通常128 byteテレメトリ送信を一時停止・再開できる。FW更新応答には影響しない。
- `Dt`はUARTフレームの受信間隔ではなく、AIコマンドの`check_counter`が最後に変化してからの時間である。UART受信停止の判定にはUART RAW画面の`RX age`を使用する。
- 20秒の実機測定ではRX data IRQ 95,112回、受信95,112 byte、72 byteフレーム1,321個、チェックサム正常1,321個で完全一致した。約4秒周期の停止中は4カウンタがすべて同時に停止し、PE/FE/NEとチェックサムエラーは増加しなかったため、MainのパーサではなくCM4側の物理送信停止と判断した。
