# CM4 UARTの実装

通信速度・フレーム・接続判定・停止条件は[UART対外仕様](../specs/stm/orion_uart_cm4.md)を参照する。本書はUSART2の内部実装と診断を管理する。

## 初期化と受信

- ピンはPB3 TX / PB4 RX、pin swap有効。RX FIFO thresholdは1/8。`MX_USART2_UART_Init()`のUSER CODEでFIFOを再有効化する。起動側で`HAL_UART_Init()`を重ねて呼ぶとFIFOENが消えるため、初期化は生成関数の呼出しだけとする。
- HALの1バイト受信状態機械を使わず、RXNE/RXFNEとCR3.EIEを直接有効化する。
- `USART2_IRQHandler()`はFIFOを空になるまで読み、`cm4_uart_rx_byte()`から`HAL_UART_RxCpltCallback()`へ渡す。通常のパーサはIRQ内で動作する。検証済み制御指令を`cmd_v2_buf`へ置き、TIM7側で取り込む。
- FWゲートウェイ中は通常パーサへ渡さず、`fw_update_gateway.c`の2 KBリングへ入れ、メインループでOFW2を解釈する。
- ISRでPE/FE/NE/OREを処理し、送信完了等の残りの要因は`HAL_UART_IRQHandler()`へ渡す。

## レジスタ更新の競合防止

`main.c`の受信開始と`USART2_IRQHandler()`末尾のCR1.RXNEIE_RXFNEIE、CR3.EIEは、計4か所とも`ATOMIC_SET_BIT`で更新する。

DMA1 Channel8のNVIC優先度は0、USART2は1、TIM7は8。DMA転送優先度LOWとは区別する。USART2側の通常read-modify-writeの途中にDMA完了IRQが入ると、HALが設定したCR1.TCIEを古い値で消したり、クリア済みのCR3.DMATを復活させたりし得る。TC処理が来なければHALのgStateがBUSY_TXに残る。

CMSISのLDREXW/STREXWによる再試行で他のビット更新を保持する。HAL側だけatomicにしても呼出し側の通常更新は守られない。実機での保護実績と未採用の復旧案は[競合レビューの履歴](history/overview_before_2026-10-20.md)に保存する。

## DMA送信

`sendRobotInfo()`は`huart2.gState != HAL_UART_STATE_READY`なら送信バッファに触れず戻る。staticバッファをゼロ初期化して状態値を詰め、[共通feedbackのCRC](../specs/common/feedback_packet.md)を最後に書き、`HAL_UART_Transmit_DMA()`が`HAL_OK`の場合だけ送信カウンタを進める。カウンタは送信登録成功を数え、物理的な到達・受信を保証しない。

TIM7内で送信を間引く。デバッグ表示タイミングやUARTの状態により実際の登録時刻は変わる。デバッグUARTの`t`で通常送信を切り替えられる。FWゲートウェイ中は通常テレメトリを止める。バージョン応答とゲートウェイ応答も同じUSART2を使うため、送信経路を追加する際は実行コンテキストと送信所有者を確認する。

## 診断

`UART`ページには次を表示する。

- `RX age`：最後の正しい制御指令からの経過時間。`FWVR` / `FWUP`は制御指令の鮮度を更新しない。
- 全USART2 IRQ数、RXバイトを読んだIRQ数、受信バイト数、最大FIFO drain数。
- フレーム組立完了数、チェックサム正常数、PE/FE/NE/ORE数、テレメトリ有効状態。

`AI_CMD`等の`Dt`はフレーム間隔ではなく、`check_counter`の最終変化からの経過時間である。FWゲートウェイ中はISR/queueのbyte数とrolling hash、overflow、header/frame/CRC、parser timeout、応答成功/失敗をRAMに保持する。

## FWゲートウェイの途中フレーム

partial frame timeoutは`uart_last_byte_tick`を先にsnapshotし、そのあと`HAL_GetTick()`を取る。判定直前にsnapshotと現在値が一致するときだけパーサをresetする。逆順ではRX IRQの更新が挟まると符号なし減算がunderflowし、受信中のフレームを誤resetし得る。

## 共通契約との差

[制御パケット](../specs/common/control_packet.md#cm4main-パケット72-バイト)ではbyte 64..70を未定義・CM4側でゼロ初期化とし、カメラ情報はCM4内で扱う。Mainには以下の旧経路が残る。

- `HAL_UART_RxCpltCallback()`はbyte 64..70を`parseCameraPacket()`へ渡す。fps非0の場合だけ`camera_updated_flag`を立てる。
- TIM7でカメラ情報をコピーし、最終更新から200 msを超えたら検出・接続状態を解除する。
- マニュアルモードには[カメラによる横速度上書き](hw_control.md#マニュアルモードのカメラ入力)が残る。

ゼロ送信するCM4ではこのカメラ補正は作動しない。Mainが未定義領域を無視する実装への整理と、マニュアルモードの期待動作の確認が残っている。旧カメラ値を送るCM4との互換を現行契約として保証しない。今回は文書整理だけを行い、ファームウェアは変更していない。
