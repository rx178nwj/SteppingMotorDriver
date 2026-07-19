# SteppingMotorDriver — プロジェクト概要・設計方針

## プロジェクト構成

```
SteppingMotorDriver/
├── schematic/
│   ├── specs/schematic-plan.md      # 回路図設計仕様（APPROVED）
│   └── prompts/hardware_spec.md     # ハードウェア仕様書
└── firmware/
    └── REQUIREMENTS.md              # ファームウェア要件定義（主要ドキュメント）
```

---

## ハードウェア仕様（確定）

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3-WROOM-1 |
| モータードライバ | DRV8825 StepStick 互換モジュール × **3** |
| エンコーダ | **3ch**（A/B/Z 相、差動入力、AM26LV32 経由） |
| ADC | **5ch**（POT×3 + 電流×1 + 電源電圧×1、すべて ADC1） |
| 通信 | USB Type-C（USB-CDC） |
| I2C | SDA=GPIO38 / SCL=GPIO39（拡張用） |
| 電源 | 24V モーター入力 → Buck → 5V → LDO → 3.3V |

### GPIO マッピング（確定）

| GPIO | 信号 | 種別 | 備考 |
|------|------|------|------|
| GPIO1 | POT0 | ADC1_CH0 | ポテンショメーター ch0 |
| GPIO2 | POT1 | ADC1_CH1 | ポテンショメーター ch1 |
| GPIO3 | POT2 | ADC1_CH2 | ポテンショメーター ch2 |
| GPIO4 | MOT_V | ADC1_CH3 | モーター電源電圧モニタ（24V分圧） |
| GPIO5 | CURRENT | ADC1_CH4 | 電流センス（10mΩシャント + LT6106、Rin=120Ω/Rout=5kΩ） |
| GPIO6 | STEP0 | Digital Out | CH0 ステップ |
| GPIO7 | DIR0 | Digital Out | CH0 方向 |
| GPIO8 | STEP1 | Digital Out | CH1 ステップ |
| GPIO9 | DIR1 | Digital Out | CH1 方向 |
| GPIO10 | STEP2 | Digital Out | CH2 ステップ |
| GPIO11 | DIR2 | Digital Out | CH2 方向 |
| GPIO12 | DRV_EN | Digital Out | 全軸共通 |
| GPIO13 | DRV_RESET | Digital Out | 全軸共通 |
| GPIO14 | DRV_SLEEP | Digital Out | 全軸共通 |
| GPIO15 | ENC0_A | Digital In | エンコーダ0 A相 |
| GPIO16 | ENC0_B | Digital In | エンコーダ0 B相 |
| GPIO17 | ENC0_Z | Digital In | エンコーダ0 Z相（インデックス） |
| GPIO18 | ENC1_A | Digital In | エンコーダ1 A相 |
| GPIO19 | USB_D- | USB | 固定 |
| GPIO20 | USB_D+ | USB | 固定 |
| GPIO21 | ENC1_B | Digital In | エンコーダ1 B相 |
| GPIO35 | ENC1_Z | Digital In | エンコーダ1 Z相 |
| GPIO36 | ENC2_A | Digital In | エンコーダ2 A相 |
| GPIO37 | ENC2_B | Digital In | エンコーダ2 B相 |
| GPIO38 | I2C_SDA | I2C | 固定 |
| GPIO39 | I2C_SCL | I2C | 固定 |
| GPIO40 | ENC2_Z | Digital In | エンコーダ2 Z相 |
| GPIO41 | M0 | Digital Out | マイクロステップ MS0 |
| GPIO42 | M1 | Digital Out | マイクロステップ MS1 |
| GPIO43 | UART_TX | UART | デバッグ用 |
| GPIO44 | UART_RX | UART | デバッグ用 |
| GPIO45 | M2 | Digital Out | マイクロステップ MS2 |

### 重要なハードウェア制約

- **DRV_EN / DRV_SLEEP / DRV_RESET は全 3 軸共通の単一 GPIO**。軸単位の独立制御は不可能
- **DRV8825 の FAULT ピンは ESP32 に未接続**（回路図上 no-connect）。ハードウェア FAULT を直接検出できない
- **M0=GPIO41、M1=GPIO42、M2=GPIO45**（GPIO22/23/24 は ESP32-S3 内部 SPI Flash 専用ピンのため使用不可）
- **ADC はすべて ADC1（GPIO1〜GPIO5）に集約**。ADC2 は WiFi と共有のため使用しない
- ESP32-S3 の ADC は非線形特性あり → 必ず `esp_adc_cal` でキャリブレーション適用
- **ポテンショメーター（POT0/1/2）**：分圧回路（20kΩ/10kΩ）＋ RC フィルタ（0.1µF）経由
- **電流センス（CURRENT = GPIO5/ADC4）**：10 mΩ シャント抵抗 + LT6106（Rin=120Ω、Rout=5kΩ、ゲイン≈41.67倍）経由。V_adc[V] = I[A]×0.4167 → I[mA] = V_adc[mV]×2.4
- **電源電圧モニタ（MOT_V = GPIO4/ADC3）**：分圧回路（24V→3.3V 範囲）経由

---

## ファームウェア設計方針（確定）

### 開発フレームワーク

- **ESP-IDF v5.x**（Arduino IDE ではなく公式 SDK）
- FreeRTOS マルチタスク設計
- ステップパルス生成：**RMT ペリフェラル**（80 MHz、12.5 ns 分解能）
- エンコーダデコード：**PCNT ペリフェラル**（4 逓倍）

### タスク構成

| タスク | 優先度 | 周期 | 役割 |
|--------|--------|------|------|
| EncoderTask | 21 | 割り込み | PCNT カウンタ・Z 相処理 |
| MotorControlTask | 20 | 1 ms | 速度プロファイル計算・RMT 更新・脱調検出 |
| ADCTask | 15 | 10 ms | サンプリング・電流換算・過電流判定 |
| CommTask | 10 | 常時 | USB-CDC コマンド受信・応答 |
| StatusTask | 5 | 100 ms | ハートビート・ログ |

### DRV8825 タイミング制約（実装時に必ず守ること）

| パラメータ | 最小値 |
|-----------|--------|
| STEP パルス幅 High/Low | 各 1.9 µs |
| DIR セットアップ時間（STEP 前） | 650 ns |
| DIR ホールド時間（STEP 後） | 650 ns |
| SLEEP → STEP 待機時間 | 1 ms |
| RESET パルス幅 Low | 10 µs |

---

## 主要な設計決定（確定済み）

### フォルト復帰：提案 A「完全手動確認型」を採用

- FAULT 復帰は `CLEAR_FAULT` コマンド 1 本で完結
- ファームウェアは**自動リトライを行わない**
- 復帰前の安全確認（機械・配線・温度）はオペレータ責任
- `CLEAR_FAULT` シーケンス：DRV_RESET 10µs パルス → DRV_SLEEP High → DRV_EN は High 維持（励磁しない）→ 全軸 SLEEP 遷移
- 復帰後に軸を動かすには必ず `ENABLE <axis>` が必要

**却下した選択肢：**
- 提案 B（原因別タイムアウト強制型）：初期フェーズは複雑すぎる
- 提案 C（自動リトライ型）：FAULT ピン未接続で信頼性が低く、機械破損リスクあり

### 起動シーケンス（F-MOT-03）

`app_main()` 先頭で安全初期状態を確定後、以下の完全起動シーケンスを実行する：

```
安全初期状態（最初の処理）：
  DRV_EN → High、DRV_SLEEP → Low、DRV_RESET → Low、STEP×4 → Low

完全起動シーケンス：
  1. DRV_EN → High、DRV_SLEEP → Low、DRV_RESET → Low（安全状態確保）
  2. 待機 1 ms
  3. DRV_RESET → High（リセット解除）
  4. 待機 1 ms（DRV8825 内部初期化完了待ち）
  5. DRV_SLEEP → High（動作可能状態）
  6. 待機 1 ms（内部チャージポンプ安定待ち）
  7. NVS から設定を読み込む（失敗時はデフォルト値を使用しログに記録）
  8. M0（GPIO41）/ M1（GPIO42）/ M2（GPIO45）を NVS 設定値に応じて出力
  9. DRV_EN は ENABLE コマンドまで High を維持（起動直後は励磁しない）
```

### モーションプロファイル

- 台形速度プロファイル（加速 → 定速 → 減速）を使用
- 短距離移動で最高速に到達しない場合は三角プロファイルに自動切り替え
- 速度更新は 1 ms 毎に RMT 周波数を動的更新

### 同時コマンドポリシー（解決済み）

| 受信 | 実行中 | 処理 |
|------|--------|------|
| MOVE / MOVETO | MOVE/MOVETO 実行中 | ERR E008 MOTION_IN_PROGRESS（拒否） |
| VEL | MOVE/MOVETO 実行中 | 減速停止後に VEL モードへ移行、EVT MOVE_ABORTED を送信 |
| VEL | VEL 実行中 | 新速度へ加速/減速（上書き許可） |
| HOME | MOVE/VEL 実行中 | ERR E008（拒否） |
| STOP / ESTOP | 任意 | 常に受付（最優先） |
| SET | 任意モーション中 | ERR E004（拒否） |
| GET | 任意 | 常に受付 |

- **将来検討**：MOVE/MOVETO 実行中の上書きポリシー（現在は拒否）はキュー方式または割り込み許可への変更を将来 Phase で検討する

### 通信ウォッチドッグ（F-COM-04）

- タイムアウト（デフォルト 5000 ms）または USB 切断 → 全軸を台形減速停止 → IDLE 遷移（コイル励磁維持）
- 停止後に `EVT COMM_TIMEOUT` をバッファに書き込む（再接続時にホストが受信）
- 再接続後は `STATUS` コマンド 1 本でセッション再確立、ENABLE 不要でそのまま動作可能
- `SET COMM_TIMEOUT <ms>`（0 = 無効）で変更可能
- USB 切断は `tinyusb_cdcacm` の disconnection コールバックで検知

---

## 未解決事項（実装前に要確認）

| # | 項目 | 優先度 |
|---|------|--------|
| ~~9.6~~ | ~~多軸同期移動（SYNC_MOVE）~~（解決済み：F-MOT-11 として Section 3.1 に追加） | ~~High~~ |
| ~~9.7~~ | ~~ADC ノイズ対策（解決済み：2段階フィルタ構成、移動平均 N=1〜64 NVS保存）~~ | ~~High~~ |
| ~~9.8~~ | ~~RMT カウンタ同期（解決済み：on_trans_done + 1ms 同期 + portENTER_CRITICAL、F-MOT-05 に反映）~~ | ~~High~~ |
| ~~9.9~~ | ~~NVS パラメータの decel / home_offset 追加（解決済み：F-MOT-10 NVS テーブルに 6 パラメータ追加）~~ | ~~Medium~~ |
| ~~9.10~~ | ~~エンコーダ低速域の速度推定方式（解決済み：F-ENC-03 を高速/低速2方式切替に改訂、ゼロ判定200 ms）~~ | ~~Medium~~ |
| 9.11 | I2C 拡張の用途・要件定義 | Medium |
| ~~9.12~~ | ~~DRV8825 FAULT ピン GPIO 未接続確認（解決済み：3.3V プルアップのみ、EVT DRV_FAULT 削除、間接検知方式に確定）~~ | ~~Medium~~ |
| ~~9.13~~ | ~~複数基板同時運用時の基板識別方法（解決済み：factory MAC アドレスを基板固有IDとして採用。`GET BOARD_ID` コマンド・USB `iSerialNumber` 設定・`E012` エラーコードを追加。F-COM-05 参照）~~ | ~~High~~ |

詳細は [firmware/REQUIREMENTS.md](firmware/REQUIREMENTS.md) の Section 9 を参照。

### 直近の検証メモ（2026-06-23）

- SYNC_MOVE の速度スケーリングが `ax->v_max / accel / decel` を上書きしていた不具合を修正済み
- 実行時専用の `motion_v_max / motion_accel / motion_decel` を導入し、軸設定値は保持する方式へ変更
- 実機テスト結果: `137/138 PASS, 0 FAIL, 1 SKIP`
- `SKIP` は T12（`TEST_GPIO`: RMT 排他のため）
- T26 後に axis0..2 の `vmax / accel / decel` が変化しない回帰テストを追加済み

---

## 開発ロードマップ

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | ESP-IDF セットアップ・USB-CDC・1 軸 RMT パルス生成・起動シーケンス | 実装済み（要ハードウェア検証） |
| 2 | 4 軸制御・台形プロファイル・状態機械・FAULT/CLEAR_FAULT | 未着手 |
| 3 | PCNT エンコーダ・Z 相ホーミング・脱調検出 | 未着手 |
| 4 | ADC 電流モニタリング・過電流保護 | 未着手 |
| 5 | コマンドセット完全実装・NVS・総合テスト | 未着手 |
