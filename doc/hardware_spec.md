# G474_Orion_main ハードウェア仕様

この文書は `G474_Orion_main.ioc` と `Core` 配下のソースコードから読み取れるハードウェア仕様を整理したものです。
回路図未確認のため、信号名がソース上で未命名の GPIO は「用途未確定」として扱います。

## MCU / クロック

- MCU: STM32G474RETx
- CPU: Arm Cortex-M4F
- SYSCLK / HCLK / APB1 / APB2: 170 MHz
- 外部クロック: HSE を使用
- PLL: HSE / 2 * 85 / 2 により 170 MHz を生成
- SysTick: HAL 標準の 1 ms tick
- 主制御周期: TIM7 割り込み 500 Hz（2 ms）

## 主要な処理周期

- TIM7: メイン制御割り込み。`MAIN_LOOP_CYCLE = 500` として 500 Hz で実行する。
- ロボット情報送信: `ROBOT_INFO_CYCLE = 125`。500 Hz 制御内で間引いて USART2 へ送信する。
- デバッグ表示: 通常 `PRINT_LOOP_CYCLE = 60`、TUI 表示時 `PRINT_TUI_CYCLE = 10`。
- 起動時は IMU 初期化、IMU キャリブレーション、CAN 初期化、アクチュエータ停止指令、キッカー充電設定後に TIM7 を開始する。

## ピン割り当て

| Pin | 信号 / ラベル | 機能 | 備考 |
| --- | --- | --- | --- |
| PF0 | RCC_OSC_IN | HSE 入力 | 外部発振子 |
| PF1 | RCC_OSC_OUT | HSE 出力 | 外部発振子 |
| PA13 | SWDIO | デバッグ | Serial Wire |
| PA14 | SWCLK | デバッグ | Serial Wire |
| PC13 | LED_G | GPIO 出力 | UART / AI 接続状態表示 |
| PC14 | LED_R | GPIO 出力 | イベント / エラー表示 |
| PB7 | LED_B | GPIO 出力 | TIM7 割り込み表示 |
| PA4 | IMU_CS | GPIO 出力 | ICM20602 の SPI CS |
| PA5 | SPI1_SCK | SPI1 | IMU 通信用 |
| PA6 | SPI1_MISO | SPI1 | IMU 通信用 |
| PA7 | SPI1_MOSI | SPI1 | IMU 通信用 |
| PC4 | IMU_INT | GPIO 入力 | 現状は割り込み未使用 |
| PC5 | IMU_FSYNC | GPIO 出力 | 起動時 High に設定 |
| PA2 | LPUART1_TX | LPUART1 | デバッグ UART、2 Mbps |
| PA3 | LPUART1_RX | LPUART1 | デバッグ UART、2 Mbps、Pull-up |
| PB3 | UART2_TX_ETH | USART2 TX | CM4 / 上位系通信、1 Mbps、ピン Swap 有効 |
| PB4 | UART2_RX_ETH | USART2 RX | CM4 / 上位系通信、1 Mbps、Pull-up、ピン Swap 有効 |
| PA11 | FDCAN1_RX | FDCAN1 | Classic CAN |
| PA12 | FDCAN1_TX | FDCAN1 | Classic CAN |
| PB12 | FDCAN2_RX | FDCAN2 | Classic CAN |
| PB13 | FDCAN2_TX | FDCAN2 | Classic CAN |
| PC12 | TIM5_CH2_Buzzer | TIM5 CH2 PWM | ブザー |
| PB0 | V_SENSE | ADC1_IN15 | 電圧検出入力として定義 |
| PB1 | C_SENSE | ADC3_IN1 | 電流検出入力として定義 |
| PA9 | ADC5_IN2 | ADC5 | アナログスイッチ入力 `sys.sw_adc_raw` |
| PB5 | DIP_0 | GPIO 入力 | モードスイッチ bit0 |
| PB6 | DIP_1 | GPIO 入力 | モードスイッチ bit1 |
| PD2 | DIP_3 | GPIO 入力 | モードスイッチ bit2 |
| PA10 | DIP_2 | GPIO 入力 | モードスイッチ bit3 |
| PC0 | 未命名 GPIO | GPIO 出力 | 用途未確定 |
| PA15 | 未命名 GPIO | GPIO 出力 | 用途未確定 |
| PB2 | 未命名 GPIO | GPIO 出力 | 用途未確定 |
| PB10 | 未命名 GPIO | GPIO 出力 | 用途未確定 |
| PB14 | 未命名 GPIO | GPIO 出力 | 用途未確定 |

## DIP スイッチ / アナログスイッチ

- DIP 値は `getModeSwitch()` で `15 - (DIP0 + DIP1*2 + DIP3*4 + DIP2*8)` として読み取る。
- GPIO は内部 Pull-up / Pull-down を設定していないため、外部回路側で論理を確定させる前提。
- アナログスイッチは ADC5_IN2 の 12 bit 生値で判定する。

| 操作 | ADC 生値範囲 |
| --- | --- |
| Center | 0 - 100 |
| Back | 101 - 500 |
| Right | 501 - 2000 |
| Forward | 2001 - 3000 |
| Left | 3001 - 3900 |

## ADC / DMA

| ADC | Channel | Pin | DMA | 用途 |
| --- | --- | --- | --- | --- |
| ADC1 | IN15 | PB0 | DMA1_Channel5 circular | `V_SENSE` |
| ADC3 | IN1 | PB1 | DMA1_Channel3 circular | `C_SENSE` |
| ADC5 | IN2 | PA9 | DMA1_Channel4 circular | アナログスイッチ |

- ADC は 12 bit、右詰め、連続変換、DMA 連続要求を有効化している。
- オーバーサンプリングは 4 倍、右シフト 2 bit。
- サンプリング時間は 640.5 cycles。
- 現状、起動時に DMA 開始しているのは ADC5 のアナログスイッチのみ。

## IMU

- センサ: ICM20602
- 通信: SPI1 master、8 bit、CPOL Low、CPHA 1 edge、MSB first
- SPI 速度: 170 MHz / 32 = 約 5.3125 Mbit/s
- CS: PA4、ソフトウェア制御
- FSYNC: PC5、初期化時 High
- INT: PC4 入力。ただし EXTI 割り込みは使っていない。
- 起動時に `ICM20602_init()` を 2 回実行し、`ICM20602_IMU_calibration2()` で yaw ジャイロをキャリブレーションする。
- 制御周期内では yaw のみを積分して姿勢角として利用する。

## UART

### LPUART1: デバッグ UART

- Pin: PA2 TX / PA3 RX
- Baudrate: 2,000,000 bps
- 8N1、FIFO 無効
- RX: 1 byte 割り込み受信
- TX: `printf_buffer` を DMA 送信
- 用途: 起動ログ、状態表示、デバッグ表示切替、PID ゲイン調整

### USART2: CM4 / 上位系通信

- Pin: PB3 TX / PB4 RX
- Baudrate: 1,000,000 bps
- 8N1、FIFO 無効
- USART の pin swap を有効化している。
- RX: 1 byte 割り込み受信
- TX: DMA1_Channel8 normal
- 受信パケット:
  - 先頭 byte: `0xFE`
  - 長さ: `RX_BUF_SIZE_CM4 = 72` byte
  - 先頭 64 byte: `RobotCommandSerializedV2`
  - 後続 8 byte: カメラ簡易パケット
  - 最終 byte: チェックサム
- 送信パケット:
  - 長さ: 128 byte
  - 先頭: `0xAB, 0xEA`
  - yaw、電圧、ボールセンサ、エラー、温度、推定位置、速度、カメラ情報などを格納する。

## FDCAN

- FDCAN1: PA11 RX / PA12 TX
- FDCAN2: PB12 RX / PB13 TX
- フレーム形式: Classic CAN
- ID: Standard ID
- モード: Normal
- Auto retransmission: Enable
- Bit rate: 約 1 Mbps
- Prescaler: 10
- Nominal TimeSeg1 / TimeSeg2: 14 / 2
- フィルタ: mask 0x000 / 0x000 で全標準 ID を FIFO0 に受信
- TX FIFO が満杯の場合はソフトウェア側の 20 frame スタックバッファに退避する。

### FDCAN バスの使い分け

- FDCAN1: 右側モータ系、ドリブラー、電源 / キッカー系を送信対象に含む。
- FDCAN2: 左側モータ系、電源 / キッカー系を送信対象に含む。
- コメント上は FDCAN1 が power / FC / mouse 系を含む想定だが、受信は両 FDCAN の FIFO0 コールバックで同じ parser に入る。

### 送信 CAN ID

| ID | 送信先 / 指令 | バス | データ |
| --- | --- | --- | --- |
| 0x010 | 電源制御 / パラメータ | FDCAN1, FDCAN2 | byte0: param id、byte1 以降: 値 |
| 0x100 | Front Right モータ | FDCAN1 | float duty |
| 0x101 | Back Right モータ | FDCAN1 | float duty |
| 0x102 | Back Left モータ | FDCAN2 | float duty |
| 0x103 | Front Left モータ | FDCAN2 | float duty |
| 0x104 | ドリブラーモータ | FDCAN1 | float duty |
| 0x110 | キッカー | FDCAN1, FDCAN2 | 充電、電圧、ストレート / チップ、キック強度 |
| 0x300 - 0x304 | モータパラメータ | FDCAN1/FDCAN2 | param id + float |
| 0x310 | モータキャリブレーション | FDCAN1 or FDCAN2 | dummy 8 byte |
| 0x000 | エラー通知 | FDCAN1, FDCAN2 | 8 byte |
| 0x001 | 電源基板リセット | FDCAN1, FDCAN2 | `0x5A, 0xA5` |

### 受信 CAN ID

| ID | 内容 |
| --- | --- |
| 0x000 - 0x001 | エラー ID / info / value |
| 0x200 - 0x204 | モータ・ドリブラーの rps と角度 |
| 0x210 - 0x216 | 各基板の電圧、電源バッテリ、キャパシタ電圧 |
| 0x220 - 0x223 | モータ温度、ドライバ温度 |
| 0x224 | 電源系 FET 温度、コイル温度 |
| 0x230 - 0x234 | 各系統の電流 |
| 0x240 | ボール検出 |
| 0x241 | マウスセンサ raw X/Y と quality |
| 0x500 - 0x503 | モータパラメータ rps |

## タイマ

### TIM7

- 用途: メイン制御割り込み
- Prescaler: 170
- Period: 2000
- 実効周期: 2 ms（500 Hz）
- 割り込み優先度: 8

### TIM5 CH2

- 用途: ブザー PWM
- Pin: PC12
- Prescaler: 170
- Period: 500
- 通常 duty: compare 250 または 0
- `actuator_buzzer_frq_on()` では prescaler を周波数に応じて変更する。

## NVIC 優先度

| 割り込み | 優先度 |
| --- | --- |
| USART2_IRQn | 1 |
| FDCAN1_IT0_IRQn | 3 |
| FDCAN1_IT1_IRQn | 4 |
| FDCAN2_IT0_IRQn | 6 |
| FDCAN2_IT1_IRQn | 7 |
| TIM7_DAC_IRQn | 8 |
| DMA1_Channel3_IRQn | 9 |
| DMA1_Channel4_IRQn | 10 |
| DMA1_Channel5_IRQn | 11 |
| LPUART1_IRQn | 13 |
| DMA1_Channel6_IRQn | 14 |
| DMA1_Channel7_IRQn | 15 |
| DMA1_Channel8_IRQn | 0（ioc 上は未有効扱い） |

## 起動時のハードウェア制御

1. GPIO / DMA / FDCAN / UART / SPI / TIM / ADC を初期化する。
2. TIM5 CH2 の PWM を開始し、起動音を鳴らす。
3. USART2 と LPUART1 の 1 byte 受信割り込みを開始する。
4. ADC5 の DMA 変換を開始し、アナログスイッチ値を `sys.sw_adc_raw` に入れる。
5. IMU を初期化、キャリブレーション、角度クリアする。
6. FDCAN1 / FDCAN2 を開始し、全 ID 受信と TX FIFO empty 通知を有効にする。
7. アクチュエータを停止状態へ設定し、キッカー充電と保護パラメータを設定する。
8. 1 秒の停止要求を入れてから TIM7 の 500 Hz 制御を開始する。

## 保護 / フェイルセーフ

- `LOW_VOLTAGE_LIMIT = 22.5` V を低電圧判定の基準に使う。
- 電源基板へ起動時に以下の保護パラメータを送る。
  - min voltage: 15.0
  - max voltage: 35.0
  - max current: 50.0
  - max FET temp: 90.0
  - max solenoid temp: 90.0
- CAN 受信 timeout は 100 cycle。500 Hz 制御なので約 200 ms。
- エンコーダ / マウス初期化が揃うまでは停止要求を継続する。
- AI 接続 timeout は 0.5 s、CM4 接続 timeout は 1.0 s。
- AI 接続後に CM4 が 6 s 以上途切れると自己リセットする。
- エラー時はメインモードを `MAIN_MODE_ERROR` にし、モータ停止、CAN エラー送信、手動電源リセット待ちに入る。
