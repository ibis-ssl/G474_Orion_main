# G474 Main アプリケーションブートローダー

## 目的

CM4からのFW更新に先立ち、User Flash先頭へ基板専用ブートローダーを置き、通常アプリをSlot Aへ再配置する。M1実装ではUART/CANによる更新はまだ行わず、安全IO、metadata、CRC32C、Slot A jumpだけを提供する。

STM32 System Memory bootloader、BOOT0によるROM boot、hardware bank swapの`BFB2`は使用しない。

## Flash配置

| 領域 | アドレス | サイズ | M1での用途 |
| --- | --- | ---: | --- |
| Bootloader | `0x08000000`～`0x08007FFF` | 32 KB | 安全IO、CRC、Slot A検証・jump |
| Slot A | `0x08008000`～`0x0803FFFF` | 224 KB | 現在の通常アプリ |
| Slot B | `0x08040000`～`0x08077FFF` | 224 KB | M3以降。M1では未使用 |
| Metadata | `0x08078000`～`0x0807FFFF` | 32 KB | M1では先頭36 byteを使用 |

通常アプリのリンカ`STM32G474RETX_FLASH.ld`はSlot Aを指し、`system_stm32g4xx.c`の`VECT_TAB_OFFSET`は`0x8000`である。

## 起動判定

metadataの次の項目を全て検証した場合だけSlot Aへjumpする。

- magic `OFW1`、format version、record size
- state=`CONFIRMED`、slot=`A`、base=`0x08008000`
- image sizeが8 byte以上224 KB以下
- metadata CRC32C
- 初期MSPがSRAM1またはCCMRAM内で8 byte alignment
- Reset_HandlerがThumb addressかつimage範囲内
- Slot A image全体のCRC32C

失敗時はPC14のerror LEDをHighにして安全状態で停止する。検証中はPC13をHighにし、成功時は消灯してSlot Aへjumpする。

## M1安全IO表

`SystemInit()`から`board_io_init_safe()`を呼び、C runtime/HALより前に設定する。各portのbonding済みGPIOは出力、入力、analog、SWD保持のいずれかに全分類し、maskの重複と未分類を`_Static_assert`で検出する。

| GPIO | M1 bootloader状態 | 理由 |
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
| その他のbonding済みGPIO | Analog no-pull | M1で不使用。floating digital inputを残さない |

PA11/PA12、PB12/PB13のFDCANとPB3/PB4のUSART2はM1ではanalogであり、M2以降で安全IO設定後にalternate functionへ切り替える。

回路図で用途未確認のPA15、PB2、PB10、PB14、PC0は、現行`MX_GPIO_Init()`と同じLowを暫定安全値としている。実機書込み前に回路図でactive levelを確定する。

## ビルド

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_slot_a.ps1 -Configuration Debug -Rebuild
```

`build_slot_a.ps1`はSlot A vectorと領域を検査し、`.bin`に対するCRC32C metadataを生成する。

## 初回導入

最初にdry-runし、Flash全体、CRC32C、Option Bytesを退避する。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug
```

出力されたbackupとIO表を確認した後だけ、明示的に`-Execute`を付ける。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\install_main_bootloader.ps1 -Configuration Debug -Execute
```

`-Execute`なしではFlash書込みもresetも行わない。初回導入が完了するまでは既存`flash.ps1`を使用しない。

導入後にST-LinkでSlot Aを更新する場合は、先に`build_slot_a.ps1`を実行し、明示的に次を使用する。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Debug -BootloaderInstalled
```

WRPはM1では設定しない。UART更新、rollback、SWD復旧の実機試験完了後にbootloader領域だけを保護する。

