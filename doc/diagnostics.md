# Orion：診断機能（Main 基板）

Orion（4輪オムニホイール）のデバッグ UART（LPUART1、2 Mbps）で見られる診断情報のうち、仕組みの説明が必要なものをまとめる。以下のログ列、車輪の符号、オムニ角度誤差の処理は Orion の実装である。キー操作の一覧は [Orion：STM 概要](overview.md#デバッグ-uartlpuart12-mbps)、CM4 との受信・送信処理は [Orion：CM4 との UART](uart_cm4.md) を参照。

## 直進加速診断ログ（DRIVE_LOG）

デバッグ UART で `g` を入力すると、`DRIVE_LOG` ページを 250 Hz で出力する。ページを選ぶと最初に `DRV_HEADER` を出し、以後は CSV 形式の `DRV` 行を出す。

- 同じ制御周期の値をまとめて出す。データの更新と競合したときは `DRV_RETRY` を出す。
- 共通の列：時刻、cmd_v2 の速度指令、ローカル最終速度目標、加速度制限中の速度目標、加速度、yaw 角、yaw 実角速度、yaw 目標角速度、yaw 減衰値。
- モーター 0〜3 ごとの列：目標 rps、実 rps、ドライバー電源ラインの電流、角度誤差、rps 誤差、Kp 項、Kd 項、FF 項、yaw 項、最終出力、最後に CAN を受信してからの経過時間。
- 電流列 `current0_A`〜`current3_A` は、各モータの電流を A 単位・小数 1 桁で出す。ドライバー側の分解能が 0.1 A なので、細かな電流差ではなく、加速開始時の左右差や接地負荷の差の傾向を見るのに使う。
- 末尾の `real_rf_lf` は `real_rps[0] + real_rps[3]`、`real_rb_lb` は `real_rps[1] + real_rps[2]`。前進時は左輪が負回転なので、正の値は右側が速く、負の値は左側が速いことを表す。`out_rf_lf` と `out_rb_lb` も同じ符号の規則で出力の差を表す。
- 静止時の角度誤差クリア（[速度制御](hw_control.md#静止時のオムニ角度誤差クリア)）の状態も出す。500 Hz の制御で `angle_clear_count=2` かつ `angle_clear_active=1` なら、回転角度誤差を補正中である。`rotation_angle_error` は補正前の回転成分、`rotation_clear_step` はその周期に各輪の目標角度から差し引いた補正量（rad）。

ログの保存例:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM167 -BaudRate 2000000 -LogPath .\drive_log.txt
```

## エンコーダーノイズ

- `motor.enc_noise_rad[]` は [制御周期](../specs/stm/orion_board_spec.md#制御周期) ごとに更新する。モータから受けた角度の周期差分 `motor.angle_diff[]` と、同時に受けた `motor.rps[]` から期待される角度差分 `2π × rps / 制御周波数` との差の絶対値を、deg/cycle に換算した値である。
- `motor.enc_noise_avg_rad[]` は 1 秒ごとの平均値、`motor.enc_noise_max_rad[]` は同じ 1 秒窓の最大値。変数名は既存の構造体との互換のため `_rad` のままだが、格納している値の単位は deg/cycle。
- デバッグ UART の `MOTOR` 表示に、4 輪分を `EncNzAvgDeg` と `Max` として出す。
- 受信角度の揺れや、角度値と速度値の食い違いを見るための簡易な指標である。CAN 周期のずれや BLDC 側の速度フィルタの遅れも含む。

## CAN 送信の計測

`can_tx_debug` に、送信登録の成功・失敗、送信待ちへの退避回数と破棄数（`sw_dropped[]`）、左右モーター指令関数の呼び出し時刻差を記録する。送信失敗や破棄の発生を調べる診断に使う。この時刻差は MCU 内部の関数呼び出しの差であり、CAN 配線上のフレーム到達時刻の差やモーターの応答の差ではない。
