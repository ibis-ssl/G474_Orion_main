# Main基板のハードウェア実装

`G474_Orion_main.ioc` と `Core` 配下のソースコードから読み取れるハードウェア仕様をまとめる。回路図は確認していないので、ソース上で信号名のない GPIO は「用途未確定」として扱う。

## MCU / クロック

- MCU: STM32G474RETx
- CPU: Arm Cortex-M4F
- SYSCLK / HCLK / APB1 / APB2: 170 MHz
- 外部クロック: HSE を使用
- PLL: HSE / 2 * 85 / 2 により 170 MHz を生成
- SysTick: HAL 標準の 1 ms tick
- 主制御周期の契約: [対外仕様](../specs/stm/orion_board_spec.md#制御周期)

## 主要な処理周期

- TIM7: メイン制御割り込み。`MAIN_LOOP_CYCLE = 500` として 500 Hz で実行する。
- ロボット情報送信: 制御周期内で`ROBOT_INFO_CYCLE`に従い間引く。目標周期は[feedback契約](../specs/common/feedback_packet.md#送信元と経路)、実際の登録条件は[UART実装](uart_cm4.md#dma送信)を参照。
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

- Pin: PB3 TX / PB4 RX（USART の pin swap を有効化）
- Baudrate: 1,000,000 bps、8N1
- RX: FIFO を有効にし、RXNE/RXFNE 割り込みで FIFO を読み切る。生成コード（`MX_USART2_UART_Init()`）は FIFO を無効にするが、同じ関数の USER CODE で再び有効にしている（`.ioc` には FIFO の設定はない）。
- TX: DMA1_Channel8 normal
- 受信フレーム: 72 byte（`RX_BUF_SIZE_CM4`）。形式は [制御パケット](../specs/common/control_packet.md#cm4main-パケット72-バイト) を参照。
- 送信フレーム: 128 byte。形式は [フィードバックパケット](../specs/common/feedback_packet.md) を参照。
- 受信処理と診断の詳細は [CM4 との UART](uart_cm4.md) を参照。

## FDCAN実装

通信速度・バス割当・CAN IDは[CAN共通契約](../specs/common/orion_can.md)を参照する。ピンは本書のピン表で管理する。

FDCAN1/2はNormal、auto retransmission有効。prescaler 10、Nominal TimeSeg1/2は14/2。mask 0x000 / 0x000で全標準IDをFIFO0へ通す。これはMain側のペリフェラル設定で、共通契約の通信速度を実現する。

### CAN 送信 FIFO

- FDCAN1/FDCAN2 のハードウェア送信 FIFO が満杯のときは、バスごとに独立した固定長のリング FIFO（`CAN_BUF_CAPACITY = 20`）へ退避する。
- 書き込み位置 `head` と読み出し位置 `tail` を分け、格納した順に送る。送信 FIFO 空き割り込みで、ハードウェア FIFO に空きがある間、古いフレームから順に転送する。
- ソフトウェア FIFO に未送信のフレームがある間は、新しいフレームがハードウェア FIFO へ直接追い越さないよう、必ずソフトウェア FIFO の末尾に追加する。
- FIFO が満杯なら新しいフレームを捨て、`can_tx_debug.sw_dropped[]` を加算する。HAL 送信登録の成功・失敗、退避回数、左右モーター指令関数の呼び出し時刻差も `can_tx_debug` で計測する。
- CAN 送信ヘッダーは送信関数ごとのローカル変数にしている。TIM7 と FDCAN の割り込みがネストしたとき、CAN1/CAN2 間でヘッダーを共有して競合するのを防ぐためである。

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
| DMA1_Channel8_IRQn | 0（USART2 TX） |

## 起動時のハードウェア制御

1. GPIO / DMA / FDCAN / UART / SPI / TIM / ADC を初期化する。
2. TIM5 CH2 の PWM を開始し、起動音を鳴らす。
3. USART2 の RX/エラー割り込み（RXNE/RXFNE、EIE）を直接有効にし、LPUART1 の 1 byte 受信割り込みを開始する。
4. ADC5 の DMA 変換を開始し、アナログスイッチ値を `sys.sw_adc_raw` に入れる。
5. IMU を初期化、キャリブレーション、角度クリアする。
6. FDCAN1 / FDCAN2 を開始し、全 ID 受信と TX FIFO empty 通知を有効にする。
7. アクチュエータを停止状態へ設定し、キッカー充電と保護パラメータを設定する。
8. 1 秒の停止要求を入れてから TIM7 の 500 Hz 制御を開始する。

## 保護・停止

外部へ設定する保護値は[対外仕様](../specs/stm/orion_board_spec.md#保護条件)、通信・CAN・初期化未完了の停止条件は[停止判定](../specs/common/safety_stop.md)を参照する。
