# Mainブートローダーの実装と導入

A/B更新契約、Flash配置、PENDING/CONFIRMED、OFW1形式は[ブートローダー対外仕様](../specs/stm/orion_bootloader.md)、CAN中継を含む順序は[MCU更新](../specs/common/firmware_update.md)を参照する。本書は内部実装とローカルのビルド・書込み手順の正本である。

## 起動判定

`Bootloader/Src/main.c` は次の順に処理する。

1. [更新中の安全動作](#更新中の安全動作) に従い、検証中 LED を点ける。
2. バックアップレジスタ `TAMP->BKP0R/BKP1R` から要求を読み、消去する。
   - `BOOT_REQUEST_UPDATE`：UART 更新モードに入る（[UART更新契約](../specs/stm/orion_bootloader.md#uart-更新ofw1)）。
   - `BOOT_REQUEST_CONFIRM`：指定スロットが PENDING なら CONFIRMED にし、control record の試行回数を 0 にする。
3. 起動するスロットを選ぶ（`boot_select_slot()`）。
   - control record が有効ならその優先スロット、無効なら有効なスロットのうち generation が新しいほうを選ぶ。選んだスロットが無効なら、もう一方を試す。
   - PENDINGの試行回数とロールバックは[起動と確定の契約](../specs/stm/orion_bootloader.md#起動と確定の契約)に従う。
4. PENDING のスロットを起動するときは、control record の試行回数を 1 増やす。
5. そのスロットへジャンプする。有効なスロットがなければエラー LED を点けて停止する。

スロットが有効とみなされるのは、次をすべて満たすときである（`boot_slot_is_valid()`）。

- magic `OFW1`、format version、record size が一致する
- slot、image base、image sizeが[イメージ配置](../specs/stm/orion_bootloader.md#flash-配置)およびベクタ検証の条件を満たす
- state が CONFIRMED（起動判定では PENDING も可）
- metadata の CRC32C が一致する
- 初期 MSP が SRAM1 か CCMRAM の中で、8 バイト境界に揃っている
- Reset_Handler が Thumb アドレスで、イメージの範囲内にある
- イメージ全体の CRC32C が一致する


## アプリへジャンプするときのCPU状態

bootloaderはjump準備中に全割り込みを禁止するが、その状態を通常アプリへ引き継いではならない。NVIC pending/enableとSysTickを消去した後、`CONTROL`、`BASEPRI`、`FAULTMASK`、`PRIMASK`を0へ戻し、選択スロットのMSPとVTORを設定してReset_Handlerへ分岐する。


## 更新中の安全動作

上位から見た更新中の安全動作は[ブートローダー契約](../specs/stm/orion_bootloader.md#更新中の安全動作)を参照する。以下はそのGPIO・CPU設定の実装である。検証中はPC13、起動不可時はPC14をHighにする。

## 安全IOの実装

`SystemInit()`から`board_io_init_safe()`を呼び、C runtime/HALより前に設定する。各portのbonding済みGPIOは出力、入力、analog、SWD保持のいずれかに全分類し、maskの重複と未分類を`_Static_assert`で検出する。

| GPIO | bootloader状態 | 理由 |
| --- | --- | --- |
| PA4 | Output High | IMU CS非選択 |
| PA15 | Output Low | 既存未命名出力の安全初期値 |
| PB2/PB10/PB14 | Output Low | 既存未命名出力の安全初期値 |
| PB7 | Output Low | status LED初期消灯 |
| PC0/PC5 | Output Low | 既存出力、IMU FSYNC |
| PC12 | Output Low | buzzer PWMを停止状態へ固定 |
| PC13/PC14 | Output Low後にstatus表示 | boot validating/error LED |
| PA10/PB5/PB6/PC4/PD2 | Input no-pull | DIP、IMU INT。外部回路のlevelを維持 |
| PB8 | Input pull-down | BOOT0兼用pinを実行開始後Low側へ固定 |
| PA13/PA14 | reset状態を保持 | SWDIO/SWCLK |
| その他のbonding済みGPIO | Analog no-pull | bootloaderで不使用。floating digital inputを残さない |

FDCANのPA11/PA12、PB12/PB13とUSART2のPB3/PB4は安全IO初期化ではanalogとし、UART更新に入るときだけPB3/PB4をalternate functionへ切り替える。

更新開始から通常アプリへjumpするまで、TIM5は強制reset後にclock disableし、PC12はGPIO Output Lowを維持する。G474自身の更新中だけでなく、G474がF303更新を中継している間もブザーPWMを開始してはならない。


## ビルド

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_slot_a.ps1 -Configuration Debug -Rebuild
```

`build_slot_a.ps1` は Slot A のベクタと領域を検査し、`.bin` から CRC32C 付きの metadata を生成する。A/B 両方を作るときは `build_slot_b.ps1` を使う（[Main開発文書](overview.md#ビルドと書込み)）。

## 初回導入

最初に dry-run し、Flash 全体、CRC32C、Option Bytes を退避する。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug
```

出力された backup と IO 表を確認したあとでだけ、明示的に `-Execute` を付ける。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug -Execute
```

`-Execute` を付けなければ、Flash への書込みもリセットも行わない。初回導入が終わるまでは、既存の `flash.ps1` を使わない。

導入後に ST-Link で Slot A を更新するときは、先に `build_slot_a.ps1` を実行してから、明示的に次を使う。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Debug -BootloaderInstalled
```

bootloader 領域の書込み保護（WRP）はまだ設定していない。


## 回路図の確認範囲

PA15、PB2、PB10、PB14、PC0は用途未確認で、通常アプリの`MX_GPIO_Init()`と同じLowを暫定安全値としている。初回書込み前に回路図でactive levelを確認する。実機導入時の観測結果は[過去の導入記録](history/bootloader_before_2026-10-20.md)に保存する。
