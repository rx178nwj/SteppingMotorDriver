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
| モータードライバ | DRV8825 StepStick 互換モジュール × 4 |
| エンコーダ | 4ch（A/B/Z 相、JST-XH 5 ピン） |
| ADC | 4ch（NJM2114 オペアンプ経由） |
| 通信 | USB Type-C（USB-CDC） |
| I2C | SDA=GPIO38 / SCL=GPIO39（拡張用） |
| 電源 | 24V モーター入力 → Buck → 5V → LDO → 3.3V |

### GPIO マッピング（確定）

| GPIO | 信号 | GPIO | 信号 |
|------|------|------|------|
| GPIO4 | STEP0 | GPIO5 | DIR0 |
| GPIO6 | STEP1 | GPIO7 | DIR1 |
| GPIO8 | STEP2 | GPIO9 | DIR2 |
| GPIO10 | STEP3 | GPIO11 | DIR3 |
| GPIO12 | DRV_EN（全軸共通） | GPIO13 | DRV_RESET（全軸共通） |
| GPIO14 | DRV_SLEEP（全軸共通） | | |
| GPIO41 | M0/MS0（マイクロステップ） | GPIO42 | M1/MS1（マイクロステップ） |
| GPIO45 | M2/MS2（マイクロステップ） | | |
| GPIO15 | ENC0_A | GPIO16 | ENC0_B |
| GPIO17 | ENC1_A | GPIO18 | ENC1_B |
| GPIO21 | ENC2_A | GPIO35 | ENC2_B |
| GPIO36 | ENC3_A | GPIO37 | ENC3_B |
| GPIO19 | USB_D- | GPIO20 | USB_D+ |
| GPIO38 | I2C_SDA | GPIO39 | I2C_SCL |
| GPIO1 | ADC0 | GPIO2 | ADC1 |
| GPIO3 | ADC2 | GPIO40 | ADC3 |

### 重要なハードウェア制約

- **DRV_EN / DRV_SLEEP / DRV_RESET は全 4 軸共通の単一 GPIO**。軸単位の独立制御は不可能
- **DRV8825 の FAULT ピンは ESP32 に未接続**（回路図上 no-connect）。ハードウェア FAULT を直接検出できない
- **M0/MS0=GPIO41、M1/MS1=GPIO42、M2/MS2=GPIO45** で ESP32-S3 が GPIO 出力制御（GPIO22/23/24 は ESP32-S3 内部 SPI Flash 専用ピンのため使用不可）
- ESP32-S3 の ADC は非線形特性あり → 必ず `esp_adc_cal` でキャリブレーション適用

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
  8. M0（GPIO22）/ M1（GPIO23）/ M2（GPIO24）を NVS 設定値に応じて出力
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

詳細は [firmware/REQUIREMENTS.md](firmware/REQUIREMENTS.md) の Section 9 を参照。

---

## 開発ロードマップ

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | ESP-IDF セットアップ・USB-CDC・1 軸 RMT パルス生成・起動シーケンス | 未着手 |
| 2 | 4 軸制御・台形プロファイル・状態機械・FAULT/CLEAR_FAULT | 未着手 |
| 3 | PCNT エンコーダ・Z 相ホーミング・脱調検出 | 未着手 |
| 4 | ADC 電流モニタリング・過電流保護 | 未着手 |
| 5 | コマンドセット完全実装・NVS・総合テスト | 未着手 |
