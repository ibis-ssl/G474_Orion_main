> 2026-10-20の整理前の文書を保存した履歴。本文の「現在」「現行」、仕様値、操作手順は当時の記述であり、現在の仕様・手順として使用しない。未採用案や未確認事項も含む。現在の入口は[開発文書](../overview.md)。

# 統合仕様への集約対象レビュー

2026-10-18 時点の文書整理のための調査記録。現行仕様の正本ではない。

対象は、本リポジトリの `README.md`、`AGENTS.md`、`doc/` 内の全 Markdown（本記録追加前の9文書）。比較先は submodule `specs` のコミット `d93a17753d1c3031fe49aa8ae1edd575076e0499`、実装の確認基準は親リポジトリの `dc38359fa57f07e5a85163957230060705039dd0`。外部リポジトリの受信実装や実機の動作は今回確認していない。

## 集約方針

通信形式、層間の責務、停止条件、更新手順は `specs/common/` に集約する。Main 固有でも、現行の動作・ハードウェア・制御・診断の仕様は、すでに用意されている `specs/stm/` に集約する。同じ数値・条件・レイアウトをローカル文書で保守しない。

本リポジトリには、仕様への入口、submodule の扱い、開発環境の設定、変更理由や検証結果の作業記録を残す。移行後も旧リンクを維持する必要がある文書は、本文を対応する `specs` 文書への案内に置き換える。

## ファイル単位の整理対象

| 現在の文書 | `specs` 側にのみ記載する内容 | 集約先 | 本リポジトリでの扱い |
|---|---|---|---|
| [README.md](../../README.md) | ロータリースイッチのモード一覧、LED の意味、デバッグ表示の操作、制御機構成の説明 | [STM 概要](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/overview.md)、[システム構成](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/system_overview.md) | プロジェクトの入口、開発環境・仕様へのリンクだけを残す |
| [ai_cmd_definition.md](https://github.com/ibis-ssl/G474_Orion_main/blob/120b1e7/doc/ai_cmd_definition.md) | 指令の経路と互換性、全フィールドの意味・単位・座標系、mode 3 の解釈、遅延補償時間、停止指令・vision の扱い | [制御パケット](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/control_packet.md)、[停止判定](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/safety_stop.md) | 全文が仕様。移行先への案内に置換可能 |
| [feedback_packet.md](https://github.com/ibis-ssl/G474_Orion_main/blob/120b1e7/doc/feedback_packet.md) | 128 byte の枠、同期値、CRC-8/ATM の計算範囲・パラメータ・検査値、計算順序、受信時の検証、応答領域上書き時の扱い | [フィードバックパケット](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/feedback_packet.md)、必要に応じて [FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md) | **先に集約先の更新が必要**。更新後は案内に置換 |
| [control_system_architecture.md](https://github.com/ibis-ssl/G474_Orion_main/blob/120b1e7/doc/control_system_architecture.md) | 制御機の役割、通信構成図、CAN バスとノード、指令・センサ・状態情報の流れ、更新経路、物理配線未確認という範囲の制約 | [システム構成](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/system_overview.md)、[FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md) | ほぼ収録済み。根拠となるファイル・関数の参照も集約先へまとめ、ローカルは案内にする |
| [board_spec.md](../board_spec.md) | MCU・クロック・周期、ピン、スイッチ閾値、ADC/DMA、IMU、UART、CAN ID、タイマ、NVIC、起動順序、保護値 | [Main ハードウェア仕様](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/board_spec.md) | 全文が仕様。古い UART/FIFO 記述をコピーせず、集約先へ案内する |
| 同上 | AI/CM4/CAN/vision のタイムアウト、停止・リセット条件 | [停止判定](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/safety_stop.md) | タイムアウト値はローカルに残さない。`specs/stm/board_spec.md` 内の重複値もリンクへ整理する候補 |
| [hw_control.md](../hw_control.md) | 制御ブロック図、関数の呼出順、座標変換、加速度制限と boost、ヨー制御、逆運動学、各輪制御式・ゲイン・出力制限、オドメトリとの関係、未使用フィールド | [速度制御](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/speed_control.md) | ほぼ収録済み。旧ゲインを引き継がず案内に置換。停止条件そのものは共通仕様へリンク |
| 同上 | `accelBoost()` の0除算可能性と将来の修正案 | ローカルの課題・レビュー記録 | 現行仕様と分離する。推測や未実装案を仕様として転記しない |
| [bootloader.md](../bootloader.md) | Flash 配置、metadata、起動検証、安全 IO、jump 時 CPU 状態、WRP の現状 | [Main ブートローダー](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/bootloader.md) | 現行 A/B 仕様を使用し、M1 時点の制約は現行仕様から除外 |
| 同上 | OFW2 の形式、chunk、sequence 再送、CAN ID、並列更新、commit offset、更新中の制御・ブザー停止 | [FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md) | ほぼ収録済み。実測結果と分離して案内に置換 |
| 同上 | ビルド、初回導入、ST-Link でのアプリ更新手順 | [Main ブートローダー](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/bootloader.md)、[STM 概要](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/overview.md) | すでに両側にあるため手順は一元化し、ローカルは実行スクリプトと案内を残す |
| 同上 | 日付付き導入結果、PRIMASK 引継ぎ不具合、10回連続試験、所要時間、CRC・Flash readback 結果 | ローカルの作業・実機検証記録 | 保持する。M1 の記述は当時の状況と明示する |
| [overview.md](../overview.md) | 現行構成、ビルド・書込み・ログ取得手順、成果物、UART/FIFO・制御・診断・FW 識別の仕様 | 下記の節単位対応表 | 仕様部分を除き、作業・検証記録へ再編する |
| [vscode.md](../vscode.md) | 集約必須のロボット仕様はない | — | `.vscode/` と `update_code_browse.ps1` の設定・保守手順として維持。`specs/stm/overview.md` から案内する程度でよい |
| [specs.md](../specs.md) | 集約対象はない | — | submodule の取得・更新と正本の所在を示すローカルの運用文書として維持 |
| [AGENTS.md](../../AGENTS.md) | 集約対象はない | — | コーディング・テスト・文書管理の作業指示として維持 |

## `doc/overview.md` の節単位対応

この文書は現行仕様、過去のレビュー、検証結果、操作手順が混在しているため、ファイルごとの移動では分離できない。

| 節 | `specs` へ集約する部分 | ローカルに残す部分 |
|---|---|---|
| CM4 UART通信の競合レビュー／実機検証結果と正式修正 | 現行の atomic 更新と必要な理由 → [UART](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/uart_cm4.md)（収録済み） | 競合仮説、案1〜3の比較、未採用案、カウンタの実測、原因特定の限界 |
| 全体構成（現在） | 現行構成 → [ブートローダー](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/bootloader.md)、[FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md) | M1 導入の経緯を残すなら日付付き履歴にする |
| ビルド手順／ST-Link接続確認／スクリプト／出力成果物／注意点 | [STM 概要](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/overview.md)、[初回導入](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/bootloader.md)（ほぼ収録済み） | 過去のビルド・検証でのみ成立した注意は当時の記録へ |
| 参考ドキュメント | 対応する `specs` 文書へのリンクへ更新 | `doc/hardware_spec.md` への存在しない参照は修正する |
| ハードウェア仕様更新メモ／速度制御更新メモ | 現行の周期・通信・制御構成・未使用項目 → [ハード](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/board_spec.md)、[速度制御](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/speed_control.md) | 文書再作成や文字化け対応の経緯 |
| エンコーダーノイズ計測 | 計算式、更新窓、単位、表示名、指標の限界 → [診断](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/diagnostics.md)（収録済み） | 実測結果が追加される場合は検証記録へ |
| CAN送信FIFO | 順序保証、20 frame、満杯時の破棄、カウンタ、ヘッダー共有回避 → [ハード](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/board_spec.md)、[診断](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/diagnostics.md)（収録済み） | 2026-08-04 の測定値と、配線上の到達差・実応答差を測っていないという試験範囲 |
| BLDC並列FW更新／Main A/B FW更新 | バスごとの進捗・再送、PENDING/CONFIRM、更新中の制御停止 → [FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md)（収録済み） | 更新時間、回数、CRC、generation、VTOR、実機で確認した状態 |
| CM4 USART2割り込み受信の安定化 | FIFO・初期化、通常時と gateway 時の処理、partial frame timeout、診断 → [UART](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/uart_cm4.md)（通常時と gateway 時を区別して収録済み） | PL011 切替後の試験条件・成功数・遅延分布・ビルド識別 |
| 開発用FW識別 | `FWVR` 配置、照会対象・再照会・応答、build ID の生成 → [FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md)、[STM 概要](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/overview.md)（収録済み） | 実機で照会を確認した日時・結果 |
| 直進加速診断ログ／デバッグUART表示ページ | 列定義・単位・符号・周期・スナップショット・キー操作 → [診断](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/diagnostics.md)、[STM 概要](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/overview.md)（収録済み） | 個別機体のログや測定結果 |
| 静止時のオムニ角度誤差クリア | 適用条件、回転成分の分離、時定数・制限・解除条件 → [速度制御](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/speed_control.md)、ログの意味 → [診断](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/diagnostics.md)（収録済み） | 変更理由や実機検証を残す場合は別の記録へ |
| CM4_105 Power更新の復旧 | Power 対象時の8 frameごとの1 ms待機 → [FW 更新](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/common/firmware_update.md)（収録済み） | 欠落の再現条件、所要時間、build ID、CRC、復旧確認、未計測事項 |
| CM4 UART送受信競合の切り分け | UART 診断カウンタ、`t`、`RX age` と `Dt` の意味 → [UART](https://github.com/ibis-ssl/specs/blob/d93a17753d1c3031fe49aa8ae1edd575076e0499/stm/uart_cm4.md)（収録済み） | COM57 での測定値、停止中の観測、原因の切り分け結果 |

## 集約前に解消すべき差分

### 1. フィードバック形式は `specs` の更新が必要

`Core/Src/ai_comm.c` の `sendRobotInfo()` / `feedbackCrc8()` を確認した。ローカルの [feedback_packet.md](https://github.com/ibis-ssl/G474_Orion_main/blob/120b1e7/doc/feedback_packet.md) にある CRC-8 の説明は現行コードと一致する。一方、`specs/common/feedback_packet.md` は旧形式であり、CRC の置換だけでは不足する。

| 項目 | 現在の `specs` の記述 | 現行コード |
|---|---|---|
| byte 2 | byte 3..127 の加算チェックサム | 同範囲の CRC-8/ATM（poly `0x07`、init/xorout `0`、refin/refout false） |
| 送信カウンタ | byte 14 | byte 4。DMA送信登録成功時だけ加算 |
| エラー id/info/value | byte 16..23 | byte 5..12 |
| IMU yaw | byte 4..7 | byte 13..16 |
| ボール検出 | byte 12..13 | byte 17..18、byte 19 は0 |
| vision との角度差 | byte 36..39 | byte 20..23 |
| バッテリ電圧 | byte 8..11 | byte 24..27 |
| kick／電源温度／キャパシタ電圧 | byte 15、33..35、40..43 | byte 28、29..31、32..35 |
| マウス関連 | byte 64以降の配列 | odom: byte 36..43、global velocity: 44..51、quality: 52..55 |
| 電流／モータ温度 | byte 24..27、29..32 | byte 56..59、60..63 |
| 出力速度／モータfeedback／ローカルodom速度 | byte 64以降の配列 | byte 64..71、72..87、88..99 |
| ステア角領域 | 記載なし | byte 100..107 に角度0の符号化値 `7F FF` を各輪に設定 |
| カメラ圧縮値 | byte 60..63 | 現行送信処理では格納していない |
| 統合推定位置 X/Y | byte 44..51 | byte 112..119 |
| グローバルodom速度 X/Y | byte 52..59 | byte 120..127 |

byte 108..111 はゼロ初期化されたまま。`tx_value_array[14]` の旧配置説明は、現行の固定配置に置き換える必要がある。

`specs` の CM4 位置制御が byte 44..51 を現在位置として読むという説明も、この Main の送信配置とは一致しない。受信側が更新済みかは今回未確認なので、受信側の現行コードを照合したうえで `specs/common/feedback_packet.md`、`specs/cm4/position_control.md`、`specs/cm4/bridge.md`、`specs/pc/` 内の関連説明を整合させる。CRC検証の対応状況も同様に確認する。

ローカル文書の「FW gateway 応答で byte 112..126 を上書きした場合」という説明は、現行の推定位置・速度領域と重なる。応答の生成経路と現行コードでの使用有無を確認してから統合する。

### 2. UART 送信バッファの問題説明は古い

`specs/stm/uart_cm4.md` は「DMAの戻り値を見ない」「バッファを作り直してから HAL に渡す」としている。現行の `sendRobotInfo()` はバッファ変更前に `huart2.gState != HAL_UART_STATE_READY` なら戻り、`HAL_UART_Transmit_DMA()` が `HAL_OK` の場合だけカウンタを増やす。旧問題を現在もそのまま存在する問題として記述しない。`doc/overview.md` の競合レビューは変更前の調査として保存する。

### 3. ローカル側が古く、`specs` 側を参照すればよい項目

- `doc/board_spec.md` の USART2「FIFO 無効・1 byte HAL 受信」、CAN の「スタックバッファ」は古い。`specs/stm/board_spec.md` は FIFO drain・リング FIFO として整理済み。
- `doc/board_spec.md` の AI 0.5 s／CM4 1.0 s は、現行コードの実際の比較時間と異なる。`system_time_ms` に対して `MAIN_LOOP_CYCLE * TIMEOUT` を比較するため、現行は250 ms／500 ms。`specs/common/safety_stop.md` がこの差を説明している。
- `doc/hw_control.md` の初期ゲイン50/1に対し、現行 `main.c` の `target` 初期化子は25/2。`specs/stm/speed_control.md` は25/2を記載済み。
- `doc/bootloader.md` の「Slot B 未使用」「M1ではUART更新なし」、`doc/overview.md` の M1 構成は過去の段階。`specs/stm/bootloader.md` は A/B・UART更新・ロールバックまで記載済み。
- README の「文字を送るたび切替、backspaceで戻る」は現行のキー割当を十分に表していない。`specs/stm/overview.md` に一覧がある。
- `doc/overview.md` の全CANノード一括更新の説明は、現行 `specs/common/firmware_update.md` の給電順序の注意と異なる。一括更新の古い説明を現行手順として残さない。

## 整理の実施順序

1. フィードバックの現行レイアウト・CRC・受信側との整合と、UART 送信処理の説明を `specs` 側で更新する。
2. `overview.md` と `bootloader.md` の実機結果・変更経緯を `doc/` 配下の作業記録へ分離する。旧版の制約や未採用案には当時の状況を明記する。
3. 現行仕様の重複本文を削除し、対応する `specs` 文書への案内に置き換える。README も仕様・開発手順への入口に整理する。
4. 文書間リンクを確認し、`specs` 内でもタイムアウトやパケット定義を一か所に集約する。ローカル独自の開発設定は `doc/vscode.md`、submodule 運用は `doc/specs.md` に残す。

今回の作業は対象と差分の整理まで。既存文書の削除・本文移行、`specs` の編集、コード変更は行っていない。
