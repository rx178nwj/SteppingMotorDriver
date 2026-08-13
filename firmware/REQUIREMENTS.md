# SteppingMotorDriver ファームウェア 要件定義

## 1. システム概要

### 1.1 目的
ESP32-S3-WROOM-1 を搭載した SteppingMotorDriver PCB 上で動作するファームウェア。
4軸ステッピングモーターの高精度制御、エンコーダフィードバック、電流/電圧モニタリングを提供する。

### 1.2 対象ハードウェア
| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3-WROOM-1 |
| モータードライバ | DRV8825 × 3 |
| エンコーダ入力 | 3ch（A/B/Z相、差動入力、AM26LV32 経由） |
| ADC | 5ch（POT×3 + 電流×1 + 電源電圧×1、すべて ADC1） |
| 通信 | USB Type-C（USB-CDC） |
| I2C | SDA=GPIO38 / SCL=GPIO39（拡張用） |
| 電源 | 3.3V（ロジック）、24V（モーター） |

---

## 2. ハードウェアインターフェース

### 2.1 DRV8825 制御信号（3ch共通）

| 信号名 | 方向 | ESP32 GPIO | 説明 |
|--------|------|------------|------|
| STEP_0 | ESP32 → DRV8825 | GPIO6 | ステップパルス（立ち上がりエッジで1ステップ） |
| STEP_1 | ESP32 → DRV8825 | GPIO8 | ステップパルス（立ち上がりエッジで1ステップ） |
| STEP_2 | ESP32 → DRV8825 | GPIO10 | ステップパルス（立ち上がりエッジで1ステップ） |
| DIR_0 | ESP32 → DRV8825 | GPIO7 | 回転方向（H=正転 / L=逆転） |
| DIR_1 | ESP32 → DRV8825 | GPIO9 | 回転方向（H=正転 / L=逆転） |
| DIR_2 | ESP32 → DRV8825 | GPIO11 | 回転方向（H=正転 / L=逆転） |
| DRV_EN | ESP32 → DRV8825 | GPIO12 | 全軸共通イネーブル（アクティブLow） |
| DRV_SLEEP | ESP32 → DRV8825 | GPIO14 | 全軸共通スリープ（アクティブLow） |
| DRV_RESET | ESP32 → DRV8825 | GPIO13 | 全軸共通リセット（アクティブLow、最低 10 µs） |
| M0 | ESP32 → DRV8825 | GPIO41 | マイクロステップ bit0（全軸共通） |
| M1 | ESP32 → DRV8825 | GPIO42 | マイクロステップ bit1（全軸共通） |
| M2 | ESP32 → DRV8825 | GPIO45 | マイクロステップ bit2（全軸共通） |

#### マイクロステップ設定表（DRV8825）
| M2 | M1 | M0 | 分割数 |
|----|----|----|--------|
| L  | L  | L  | フルステップ（1） |
| L  | L  | H  | 1/2 |
| L  | H  | L  | 1/4 |
| L  | H  | H  | 1/8 |
| H  | L  | L  | 1/16 |
| H  | L  | H  | 1/32（デフォルト） |

### 2.2 エンコーダ入力（3ch）

**対応エンコーダ：Stepperonline 17HS15-1504-ME1K（1000 PPR、差動出力）**

差動信号（A+/A−、B+/B−、Z+/Z−）を AM26LV32IDR（低電圧高速 4ch 差動ラインレシーバ）で受信し、
シングルエンド CMOS 信号に変換して ESP32-S3 へ入力する。

| ch | 信号 | ESP32 GPIO | 説明 |
|----|------|-----------|------|
| 0 | ENC0_A | GPIO15 | A相（PCNT 入力） |
| 0 | ENC0_B | GPIO16 | B相（PCNT 入力） |
| 0 | ENC0_Z | GPIO17 | Z相（インデックスパルス・原点検出） |
| 1 | ENC1_A | GPIO18 | A相（PCNT 入力） |
| 1 | ENC1_B | GPIO21 | B相（PCNT 入力） |
| 1 | ENC1_Z | GPIO35 | Z相（インデックスパルス・原点検出） |
| 2 | ENC2_A | GPIO36 | A相（PCNT 入力） |
| 2 | ENC2_B | GPIO37 | B相（PCNT 入力） |
| 2 | ENC2_Z | GPIO40 | Z相（インデックスパルス・原点検出） |

- A相・B相の両エッジで計数（4逓倍モード）→ 実効分解能 4000 カウント/rev
- AM26LV32 の差動入力電圧範囲：±7 V（エンコーダケーブル延長耐ノイズ性確保）

### 2.3 ADCチャンネル（5ch、すべて ADC1）

| ch名 | GPIO | 信号 | 内容 |
|------|------|------|------|
| ADC0 | GPIO1 | POT0 | ポテンショメーター ch0（分圧回路 + RC フィルタ） |
| ADC1 | GPIO2 | POT1 | ポテンショメーター ch1（分圧回路 + RC フィルタ） |
| ADC2 | GPIO3 | POT2 | ポテンショメーター ch2（分圧回路 + RC フィルタ） |
| ADC3 | GPIO4 | MOT_V | モーター電源電圧モニタ（24V → 3.3V 分圧回路経由） |
| ADC4 | GPIO5 | CURRENT | モーター電源電流センス（100 mΩ シャント + 20倍アンプ経由） |

- 分解能：12bit（ESP32-S3 内蔵ADC）
- 入力レンジ：0〜3.3V
- ADC2 は WiFi と共有のため使用しない（全 ch を ADC1 に集約）

### 2.4 USB通信
- インターフェース：USB Type-C
- プロトコル：USB-CDC（仮想シリアルポート）
- ボーレート：115200 bps（または USB Full Speed 固定帯域）
- ESP32-S3 の内蔵 USB Serial/JTAG コントローラを使用
- USB ディスクリプタの `iSerialNumber` に、eFuse factory MAC アドレス由来の16進文字列（例: `AABBCCDDEEFF`）を設定する（F-COM-05）

### 2.5 Status1 LED（USB-CDC接続・システム状態表示）

| 項目 | 内容 |
|------|------|
| GPIO | GPIO46（出力、Status1 LED専用） |
| 用途 | USB-CDC接続状態および軸のFAULT状態をユーザーに一目で提示する表示灯。無線（BLE/WiFi）側の表示灯である Status2 LED（GPIO47、[BLE_WIFI_REQUIREMENTS.md §3.1](BLE_WIFI_REQUIREMENTS.md)）と対になる「有線」側の表示灯 |
| 駆動タスク | StatusTask（100ms周期、既存のハートビート処理と統合） |

#### 表示パターン

以下の優先順位（上ほど優先）で状態を反映する。

| 優先度 | 状態 | LED挙動 | 周期（デフォルト、暫定値） | 条件 |
|--------|------|---------|---------------------------|------|
| 1（最優先） | FAULT | 高速点滅 | 8 Hz（Duty 50%、約62.5ms ON/OFF） | いずれかの軸が FAULT 状態（ESTOP／OVERCURRENT／STALL、§3.1軸状態機械。DRV8825 FAULT ピンは GPIO 未接続のため直接検知は不可、§9.12） |
| 2 | USB通信中 | 点滅 | 2 Hz（Duty 50%、約250ms ON/OFF） | CommTask がコマンドを受信・応答処理中（`comm.c` のコマンド処理区間） |
| 3 | USB接続確立 | 点灯（常時ON） | - | USB-CDC がホストとオープン済み（DTR アサート等で検出） |
| 4（デフォルト） | USB未接続 | 消灯 | - | USB-CDC 未接続 |

- 点滅周期（2Hz／8Hz）は暫定値とし、Status2 LED（BLE_WIFI_REQUIREMENTS.md §3.1）と同一の値を採用して両LEDの見え方を統一する。実機評価後に視認性の観点で調整可能とする（§9未解決事項に追加想定）。
- 全軸が FAULT から復帰（`CLEAR_FAULT`）すれば、優先度2・3・4の判定に戻る。

---

## 3. 機能要件

### 3.1 モーター制御

#### F-MOT-01: ステップパルス生成

**パルスタイミング制約（DRV8825 データシート準拠）**

| パラメータ | 最小値 | 単位 | 備考 |
|-----------|--------|------|------|
| STEP パルス幅（High） | 1.9 | µs | tSTEP_HIGH |
| STEP パルス幅（Low） | 1.9 | µs | tSTEP_LOW |
| DIR セットアップ時間（STEP 立ち上がり前） | 650 | ns | tDIR_SETUP |
| DIR ホールド時間（STEP 立ち上がり後） | 650 | ns | tDIR_HOLD |
| SLEEP → STEP 待機時間 | 1 | ms | tWAKE（内部チャージポンプ安定） |
| RESET パルス幅（Low） | 10 | µs | tRESET |

**パルス生成実装**
- ESP32-S3 の RMT（Remote Control Transceiver）ペリフェラルを使用する
  - RMT キャリアクロック：80 MHz（APB クロック分周）
  - 分解能：12.5 ns/tick（80 MHz 時）→ STEP パルス幅を確実に 2 µs 以上確保
- 各軸に独立した RMT チャンネルを割り当てる（CH0〜CH2 = RMT CH0〜CH2）
- パルス周波数範囲：1 Hz〜200 kHz（連続）
  - 1/32 マイクロステップ時：5 kHz ≒ 約 9.4 RPM（1.8°/step モーター、200 step/rev）
  - 200 kHz 時：約 375 RPM（1/32 マイクロステップ換算）
- RMT の `rmt_transmit_config_t.loop_count` を利用した継続パルス列出力（速度制御モード）
- DIR 変更時は必ず STEP を 1 µs 以上 Low に維持してから DIR を切り替え、さらに 650 ns 以上待機後に次の STEP を送出する

#### F-MOT-02: マイクロステップ設定

**設定表（DRV8825）**

| M2 | M1 | M0 | 分割数 | steps/rev（1.8°モーター） | 最大 STEP 周波数 |
|----|----|----|--------|--------------------------|-----------------|
| L  | L  | L  | 1（フル） | 200 | 250 kHz |
| L  | L  | H  | 1/2 | 400 | 250 kHz |
| L  | H  | L  | 1/4 | 800 | 250 kHz |
| L  | H  | H  | 1/8 | 1,600 | 250 kHz |
| H  | L  | L  | 1/16 | 3,200 | 250 kHz |
| H  | L  | H  | 1/32（デフォルト） | 6,400 | 200 kHz |

- 起動時に M0/M1/M2（GPIO41/42/45）を NVS 設定値に応じて出力し、全軸共通の分割数を確定する
- デフォルト：1/32 分割（最高精度・低振動モード）
- 設定値は NVS（`motor_config` 名前空間）に保存し、再起動後も維持する
- `SET MICROSTEP <div>` コマンドで動的変更可能（全軸停止後にのみ実行可能とする）
- マイクロステップ変更時は M0/M1/M2 GPIO を新設定値に応じて同時に更新する

#### F-MOT-03: イネーブル/スリープ/リセット制御

**信号極性**

| 信号 | アクティブ | 非アクティブ | 説明 |
|------|-----------|------------|------|
| DRV_EN | Low | High | コイル励磁 ON（非励磁がデフォルト安全側） |
| DRV_SLEEP | High | Low | ドライバ動作（Low でスリープ・デフォルト安全側） |
| DRV_RESET | Low | High | 内部ラッチクリア（アクティブ Low、最低 10 µs） |

**起動時 GPIO 安全初期状態（ファームウェア初期化の最初の処理）**

電源 ON 直後は GPIO の状態が未定義のため、`app_main()` の先頭で以下を確実に設定する。
これによりファームウェア初期化中の意図しない励磁・パルス出力を防止する：

```
DRV_EN    → High  （非励磁・安全側）
DRV_SLEEP → Low   （スリープ）
DRV_RESET → Low   （リセット保持）
STEP_0〜2 → Low   （パルス出力停止）
DIR_0〜2  → Low   （任意の安定値）
```

**完全起動シーケンス**

```
1. DRV_EN → High、DRV_SLEEP → Low、DRV_RESET → Low（安全状態確保）
2. 待機 1 ms
3. DRV_RESET → High（リセット解除）
4. 待機 1 ms（DRV8825 内部初期化完了待ち）
5. DRV_SLEEP → High（動作可能状態）
6. 待機 1 ms（内部チャージポンプ安定待ち）
7. NVS から設定を読み込む
   - 読み込み失敗時はデフォルト値を使用し、ログに記録する
8. M0（GPIO41）/ M1（GPIO42）/ M2（GPIO45）を NVS 設定値に応じて出力する
9. DRV_EN は ENABLE コマンドまたはモーションコマンドを受信するまで High を維持する
   （起動直後には励磁しない）
```

**通常動作中の制御シーケンス**

```
停止後（待機モード移行）：
  モーション完了 → アイドルタイムアウト（デフォルト 2000 ms）→ DRV_EN → High → DRV_SLEEP → Low

コマンド受信時（スリープ中）：
  DRV_SLEEP → High → 待機 1 ms → DRV_EN → Low → モーション開始
```

- アイドルタイムアウト時間は NVS に保存し `SET IDLE_TIMEOUT <ms>` コマンドで変更可能とする
- DRV_EN・DRV_SLEEP・DRV_RESET は全軸共通のため、いずれか 1 軸でも動作中は非スリープを維持する

#### F-MOT-04: モーションプロファイル

**台形速度プロファイル（Trapezoidal）**

```
速度
 |        ___________
 |       /           \
 |      /             \
 |_____/_______________\_____ 時間
      ^   ^           ^ ^
      |   |           | |
      加速  最高速到達   減速開始  停止

パラメータ：
  v_start : 開始速度（steps/sec）、デフォルト 0
  v_max   : 最高速度（steps/sec）
  v_end   : 終端速度（steps/sec）、デフォルト 0
  accel   : 加速度（steps/sec²）
  decel   : 減速度（steps/sec²）、デフォルト = accel
```

**計算式**

加速ステップ数：`n_accel = (v_max² - v_start²) / (2 × accel)`  
減速ステップ数：`n_decel = (v_max² - v_end²) / (2 × decel)`  
最高速ステップ数：`n_cruise = total_steps - n_accel - n_decel`

`n_cruise < 0` の場合（短距離移動で最高速に到達しない場合）：  
  到達可能ピーク速度 `v_peak = sqrt((2 × accel × decel × total_steps) / (accel + decel))` を計算して三角プロファイルに移行する

**RMT による可変周波数パルス生成**

```c
// 台形プロファイル: RMT エンコーダコールバックで各ステップの間隔を動的に更新
// 速度 v [steps/sec] → パルス周期 T = 1/v [sec]
// RMT tick 数 = T × RMT_CLK_HZ
uint32_t ticks = RMT_CLK_HZ / current_velocity_steps_per_sec;
```

速度更新周期：1 ms 毎（MotorControlTask 周期に同期）

**パラメータ範囲**

| パラメータ | 最小 | 最大 | デフォルト | 単位 |
|-----------|------|------|-----------|------|
| v_max | 1 | 200,000 | 10,000 | steps/sec |
| accel | 1 | 1,000,000 | 50,000 | steps/sec² |
| decel | 1 | 1,000,000 | 50,000 | steps/sec² |
| v_start | 0 | 1,000 | 0 | steps/sec |

#### F-MOT-05: 位置制御（目標位置への移動）

- 各軸の目標ステップ位置を指定して移動する（絶対座標・相対座標）
- 台形速度プロファイル（F-MOT-04）を使用する
- **位置カウンタ**：ステップ出力数を 32 bit 符号付き整数でカウントする（モーター側位置）
- エンコーダ搭載時は F-MOT-08 のクローズドループ位置制御を使用可能とする
- 移動完了判定：目標ステップ数のパルス出力完了、かつ速度 = 0

**RMT パルスとステップカウンタの同期（9.8 より反映）**

- ステップカウンタは RMT の `rmt_tx_event_callbacks_t.on_trans_done` コールバックで更新する（ソフトウェアカウントではなく送信完了ベース）
- 速度制御モード（連続パルス）中は 1 ms ティック毎に `rmt_get_channel_status` から送信済みシンボル数を取得してカウンタを同期する
- カウンタ更新はクリティカルセクション（`portENTER_CRITICAL`）内で行う

#### F-MOT-06: 速度制御（一定速連続運転）

- 速度指令値（steps/sec）を与えて連続回転する
- 指令速度への到達は台形プロファイルの加速フェーズを使用する
- 速度変更コマンド受信時は現在速度から新たな速度へ加速/減速する
- 停止コマンド（STOP）受信時は減速プロファイルを生成して停止する

#### F-MOT-07: 停止制御

| 停止種別 | コマンド | 動作 |
|---------|---------|------|
| 通常停止 | `STOP <axis>` | 減速プロファイルを実行して停止（アイドルタイムアウト後スリープ） |
| 緊急停止 | `ESTOP` | 全軸即時停止、RMT 出力を即時無効化、コイル励磁維持（DRV_EN = Low） |
| コイル解除停止 | `STOP_FREE <axis>` | 減速後に DRV_EN = High でトルクを解放 |

**ハードウェア制約：DRV8825 FAULT ピンは 3.3V プルアップのみで ESP32 GPIO に未接続のため、ファームウェアは FAULT 信号を直接検知できない。**

緊急停止の発動条件：
- ソフトウェアコマンド `ESTOP`
- ホーミングタイムアウト（F-MOT-09、`HOME` 開始後 `home_timeout_us` 経過）
- 過電流検出（F-ADC-03）
- 脱調検出（F-MOT-08 の偏差閾値超過）

**注意：通信ウォッチドッグタイムアウト（F-COM-04）は FAULT 遷移ではない。** `motor_stop()` による通常の台形減速停止 → IDLE 遷移であり、`EVT COMM_TIMEOUT` を送信するのみで軸は FAULT 状態にならず `CLEAR_FAULT` も不要（`STATUS` 送信のみで再開可能）。

**FAULT 状態への遷移処理（全発動条件共通）：**
1. 全軸 RMT 出力を即時無効化する
2. DRV_EN → High（コイル励磁解除）
3. DRV_SLEEP はそのまま High を維持する（RESET 準備のため）
4. 全軸の状態を FAULT に設定する
5. FAULT 原因と発生時刻（esp_timer_get_time() の µs 値）を内部ログバッファに記録する
6. `EVT FAULT <reason> <axis_mask>` をホストに送信する
   - reason: `ESTOP` / `OVERCURRENT` / `STALL`
   - axis_mask: 影響軸のビットフィールド（例: 全軸 = 0x7）

#### F-MOT-07b: フォルト復帰（CLEAR_FAULT）

**前提：**
- DRV8825 の FAULT ピンは ESP32 に未接続のため、ハードウェア的な復帰確認は不可
- 復帰前の安全確認（機械的障害物・配線・温度）はオペレータの責任とする

**CLEAR_FAULT 実行シーケンス（全軸一括）：**

```
前提確認：
  全軸が FAULT 状態であること（一部軸のみ FAULT も全軸一括復帰）

シーケンス：
  1. DRV_EN → High（念のため非励磁を確認）
  2. DRV_RESET → Low（最低 10 µs 保持）        ← DRV8825 内部ラッチクリア
  3. DRV_RESET → High
  4. 待機 1 ms                                   ← 内部初期化完了待ち
  5. DRV_SLEEP → High
  6. 待機 1 ms                                   ← チャージポンプ安定待ち
  7. DRV_EN は High のまま維持する               ← 励磁は ENABLE コマンドで別途行う
  8. 全軸状態を SLEEP に遷移する
  9. 応答: OK
```

- FAULT 状態以外で `CLEAR_FAULT` を受信した場合は `ERR E010 NOT_IN_FAULT` を返す
- `CLEAR_FAULT` 後に軸を動かすには必ず `ENABLE <axis>` コマンドが必要とする
- ホーミング完了フラグはクリアされない（座標系は保持する）

#### F-MOT-07c: フォルト情報取得（GET FAULT_INFO）

- `GET FAULT_INFO` コマンドで最後の FAULT 情報を返す
- 応答フォーマット: `OK <reason> <axis_mask> <timestamp_us>`
- 例: `OK OVERCURRENT 0x02 1234567890`（軸 1 で過電流、タイムスタンプ µs）
- FAULT が一度も発生していない場合: `OK NONE 0x00 0`

#### F-MOT-08: クローズドループ位置制御（エンコーダ使用時）

**制御方式：位置フィードバック付きステップ補正**

エンコーダカウント（F-ENC-01）とステップカウンタの偏差を監視し、偏差が閾値を超えた場合にエラーとして扱う（DRV8825 はオープンループドライバのため、エンコーダ偏差から脱調を検出する）。

```
偏差 = encoder_position - step_position
```

| 偏差レベル | 処理 |
|-----------|------|
| ±`WARN_THRESHOLD` 以内（デフォルト 32 step） | 正常 |
| ±`ERROR_THRESHOLD`（デフォルト 128 step）超過 | 脱調警告（ホストに通知） |
| ±`FAULT_THRESHOLD`（デフォルト 512 step）超過 | 緊急停止・フォルトフラグ設定 |

- 各閾値は NVS に保存し `SET STALL_WARN/ERROR/FAULT <axis> <steps>` で設定可能
- ホーミング動作（F-MOT-09）はエンコーダを使用する

#### F-MOT-09: ホーミング動作

**シーケンス**（Z 相インデックスパルス使用）

```
1. v_home_coarse（NVS、デフォルト 2,000 steps/sec）でホーミング方向に移動
2. Z 相立ち上がりエッジ検出 → 即時停止
3. 反対方向に back_off_steps（NVS、デフォルト 200 steps）バックオフ
4. v_home_fine（NVS、デフォルト 500 steps/sec）で再アプローチ
5. Z 相立ち上がりエッジ検出 → 即時停止
6. エンコーダカウンタおよびステップカウンタを home_offset_steps（NVS、デフォルト 0）にセット
7. HOME_DONE イベントをホストに送信
```

- ホーミングタイムアウト：デフォルト 30 秒（NVS 設定可能）
- タイムアウト時は緊急停止し `HOME_TIMEOUT` エラーを送信する
- ホーミング方向（正/負）は `SET HOME_DIR <axis> <+1|-1>` で設定する
- 上記パラメータは F-MOT-10 の NVS テーブルで管理する

#### F-MOT-11: 多軸同期移動（SYNC_MOVE）

**コマンド**

```
SYNC_MOVE <n> <axis0> <steps0> [<axis1> <steps1> ...]
```

| 引数 | 型 | 範囲 | 説明 |
|------|----|------|------|
| n | uint8 | 2〜3 | 同期軸数 |
| axisN | uint8 | 0〜2 | 軸番号（重複不可） |
| stepsN | int32 | ±2,000,000 | 相対移動ステップ数（符号付き、0 = 移動なし） |

**速度スケーリング（移動時間一致方式）**

全軸が同一時刻に停止するよう各軸の速度を比率で調整する。

```
1. 参照軸 = |steps| が最大の軸（複数同値は軸番号の小さい方、steps = 0 の軸は対象外）
2. 参照軸はその軸の v_max / accel / decel パラメータをそのまま使用し、
   台形プロファイルで移動完了時間 T_ref を計算する
3. 従軸 i（i ≠ 参照軸）:
     ratio_i  = |steps_i| / |steps_ref|
     v_max_i  = max(1, floor(v_max_ref  × ratio_i))  [steps/sec]
     accel_i  = max(1, floor(accel_ref  × ratio_i))  [steps/sec²]
     decel_i  = max(1, floor(decel_ref  × ratio_i))  [steps/sec²]
4. steps_i = 0 の軸は移動しない（IDLE を維持する）
```

プロファイル形状（台形 or 三角）は参照軸と同じになる。
各従軸のスケール後パラメータが三角プロファイル条件を満たす場合は三角プロファイルに自動切替する（F-MOT-04 と同一ロジック）。

**開始同期（START タイミング）**

```
コマンド受信フェーズ（CommTask）:
  1. 全軸の IDLE 確認（いずれかが非 IDLE → ERR E008）
  2. 全軸の速度パラメータ計算
  3. 全軸の RMT チャンネルをプリロード（パルス出力は未開始）
  4. sync_group_mask（ビットフィールド）を設定し、start_pending フラグを立てる

開始フェーズ（MotorControlTask 次回 1ms ティック先頭）:
  5. start_pending を確認し、sync_group_mask の全 RMT チャンネルを
     同一ティック内でアトミックに enable する
  6. start_pending フラグをクリアする
```

RMT の enable 呼び出しは同一 MotorControlTask ティック内の連続処理で行い、
FreeRTOS タスクスイッチを挟まない（クリティカルセクション不要・処理順序で保証）。

**ソフトリミット監視**

各軸のソフトリミット（min_pos / max_pos）は個別に監視する。
いずれか 1 軸でもリミットに到達した場合：

```
1. sync_group_mask の全軸を台形減速停止する
2. EVT LIMIT_HIT <axis> を送信する
3. EVT SYNC_ABORTED <axis_mask> を送信する
```

**脱調検出（エンコーダ使用時）**

sync_group 内のいずれか 1 軸が FAULT_THRESHOLD を超えた場合、
sync_group 全軸を即時緊急停止（ESTOP 相当）する。

**ESTOP / STOP 受信時の動作**

| 受信コマンド | 処理 |
|------------|------|
| `ESTOP` | sync_group 全軸の RMT を即時無効化（FAULT 遷移） |
| `STOP <axis>`（sync_group 内の軸） | sync_group 全軸に台形減速停止を適用する |
| `STOP ALL` | 全軸停止（既存動作と同様） |

停止後に `EVT SYNC_ABORTED <axis_mask>` を送信する。

**完了処理**

参照軸の移動完了（パルス数到達・速度 = 0）をトリガーとし、
全従軸の完了も同一ティックで確認する。全軸完了後：

```
EVT SYNC_DONE <axis_mask>
```

axis_mask: SYNC_MOVE で指定した軸のビットフィールド（例: 軸 0 + 軸 2 = 0x05）

**エラー条件**

| 条件 | 応答 |
|------|------|
| n が 2〜4 の範囲外 | `ERR E002 INVALID_ARG` |
| 引数の個数不足 | `ERR E002 INVALID_ARG` |
| 軸番号が 0〜2 の範囲外 | `ERR E003` |
| 軸番号が重複している | `ERR E011 DUPLICATE_AXIS` |
| 指定軸のいずれかが IDLE 以外 | `ERR E008 MOTION_IN_PROGRESS` |
| ソフトリミット超過（開始前チェック） | `ERR E006` |

**SYNC_MOVE 実行中の他コマンド受付ポリシー**

| 受信コマンド | 処理 |
|------------|------|
| `SYNC_MOVE` | `ERR E008`（再発行不可） |
| `MOVE / MOVETO / VEL / HOME`（sync_group 内軸） | `ERR E008` |
| `STOP <axis>`（sync_group 内軸） | sync_group 全軸を台形減速停止 |
| `ESTOP` | sync_group 全軸即時停止 |
| `GET` 系 | 常に受付 |
| `SET` 系 | `ERR E004 MOTION_IN_PROGRESS` |

---

#### F-MOT-10: 軸パラメータ設定

各軸に以下のパラメータを NVS に保存する：

| パラメータ名 | 型 | デフォルト | 説明 |
|------------|-----|-----------|------|
| `steps_per_rev` | uint32 | 6400 | 1回転あたりのステップ数（1/32 時 6400） |
| `gear_ratio` | float | 1.0 | 減速比（1.0 = 直結） |
| `encoder_ppr` | uint32 | 1000 | エンコーダ分解能（PPR、4逓倍前） |
| `max_pos` | int32 | 2,000,000 | ソフトリミット上限（steps） |
| `min_pos` | int32 | -2,000,000 | ソフトリミット下限（steps） |
| `v_max` | uint32 | 10,000 | 最高速度（steps/sec） |
| `accel` | uint32 | 50,000 | 加速度（steps/sec²） |
| `decel` | uint32 | 50,000 | 減速度（steps/sec²）（accel と独立して設定） |
| `idle_timeout_ms` | uint32 | 2000 | アイドルスリープ移行時間（ms） |
| `stall_fault_th` | uint32 | 512 | 脱調フォルト閾値（steps） |
| `home_offset_steps` | int32 | 0 | ホーミング後の原点オフセット（steps）（ホーミング完了時に position = home_offset_steps にセット） |
| `v_home_coarse` | uint32 | 2,000 | ホーミング粗探索速度（steps/sec） |
| `v_home_fine` | uint32 | 500 | ホーミング精密探索速度（steps/sec） |
| `back_off_steps` | uint32 | 200 | ホーミングバックオフ距離（steps） |
| `comm_timeout_ms` | uint32 | 5,000 | 通信ウォッチドッグタイムアウト（ms、0 = 無効） |
| `pot_scale_deg` | float | 0.0879 | POT角度換算係数（度/ADCカウント、12bit=4095カウント換算。デフォルトは0〜360°を4095カウントに割り付けた値、軸ごとに実測校正が必要） |
| `pot_zero_offset` | int32 | 0 | POTゼロ位置補正オフセット（ADCカウント、`SET POT_ZERO`で現在値を記録） |

---

#### F-MOT-12: 関節角度出力（度単位）・POTゼロ位置補正

**目的：** 実機組立検証・キャリブレーション時に、同一関節の3種のセンサ（POT・エンコーダ・ステップ位置）が
示す角度を度単位で相互比較できるようにする（robot_arm_monitorの関節検証パネル、[robot_arm_monitor/REQUIREMENTS.md](../../robot_arm_monitor/REQUIREMENTS.md) F-RAM-VERIFYの唯一のデータソース）。

**採用方針：** 角度換算はホスト側（robot_arm_monitor・制御アプリ）では行わず、**ファームウェア側で完結**させる
（steps_per_rev/gear_ratio/encoder_ppr等の較正値はファームウェアが唯一の正であり、ホスト側に二重管理させない）。

**ステップ位置角度（`GET POS_DEG`）・エンコーダ角度（`GET ENC_DEG`）：**
- 既存の F-MOT-10 NVS パラメータ（`steps_per_rev`・`gear_ratio`・`encoder_ppr`）を用いて換算する
- `pos_deg = (pos / steps_per_rev) * 360.0 / gear_ratio`
- `enc_deg = (enc / (encoder_ppr * 4)) * 360.0 / gear_ratio`（PCNTは4逓倍のため`encoder_ppr`を4倍してから割る）
- 新規NVSパラメータの追加は不要（既存 F-MOT-10 の値をそのまま流用）

**POT角度（`GET POT_DEG`）・ゼロ位置補正：**
- POTはステップ/エンコーダと異なり、モーター駆動系と独立した絶対角度センサ（分圧回路経由、[CLAUDE.md](../CLAUDE.md)）のため、
  専用の較正パラメータ（`pot_scale_deg`・`pot_zero_offset`、上記 F-MOT-10 表に追加）を持つ
- 「補正なし」角度：`pot_deg_raw = adc_raw[axis] * pot_scale_deg`（`adc_raw`はADCTaskのフィルタ後生カウント、F-ADC-01）
- 「ゼロ位置補正あり」角度：`pot_deg_zeroed = (adc_raw[axis] - pot_zero_offset) * pot_scale_deg`
- `SET POT_ZERO <axis>` コマンドで、その時点の `adc_raw[axis]` を `pot_zero_offset` としてNVSに保存する
  （multi_i2c_bridgeの「0位置設定」機能と同様の設計思想。取付誤差の補正が目的で、機構上の物理ゼロ位置合わせは
  オペレータが担保する）
- `CLEAR POT_ZERO <axis>` コマンドで `pot_zero_offset` を0にリセットする
- `SET POT_SCALE <axis> <deg_per_count>` コマンドで `pot_scale_deg` を校正する（POTの可動範囲とADCカウント範囲の
  実測に基づき、ホスト側オペレータが算出した値を設定する想定。自動校正ウィザードは本版スコープ外）

**相対・絶対移動の角度指定（`MOVE_DEG`/`MOVETO_DEG`）：**
- 既存の `MOVE`/`MOVETO`（steps単位）と同じモーションロジック（台形プロファイル、F-MOT-04/05）を用い、
  引数のみ度単位で受け取り内部でsteps換算する（`steps = deg * steps_per_rev * gear_ratio / 360.0`、四捨五入）
- 換算後は既存の `MOVE`/`MOVETO` と完全に同じ実行パス・同時コマンドポリシー（E008等）に従う
- 完了イベントも既存と同じ `EVT MOVE_DONE <axis>` を使う（度単位専用の新規イベントは設けない）

**反映先：** Section 3.1（F-MOT-10 NVSテーブル）、Section 4.2（`MOVE_DEG`/`MOVETO_DEG`）、
Section 4.3（`GET POS_DEG`/`GET ENC_DEG`/`GET POT_DEG`）、Section 4.4（`SET POT_SCALE`/`SET POT_ZERO`/`CLEAR POT_ZERO`）

#### F-MOT-13: 保持電流モード（2026-08-09新設）

**目的：** ロボットアーム関節は無励磁になると外力（自重等）で位置がずれるため、アイドル時にも
励磁を維持できる「保持モード」を設ける。DRV8825の実駆動電流（VREF）は各モジュール基板上の
物理トリムポットで固定されており、ESP32からデジタル制御できないため、**`DRV_EN`
（アクティブLow＝励磁、全3軸共通の単一GPIO、[CLAUDE.md](../CLAUDE.md)）をLEDC（ハードウェアPWM、
20kHz固定）でチョッピングし、平均電流を疑似的に下げることで保持電流可変を近似する**方式を採用する。
`DRV_EN`が全軸共通のため、本モードは**軸単位ではなく基板全体で共通のグローバル設定**とする。

**3モード（`SET HOLD_MODE <0|1|2>`）：**

| 値 | 名称 | 動作 |
|---|------|------|
| 0 | NORMAL（デフォルト、既存動作） | `SET IDLE_TIMEOUT`経過で`DRV_EN`=High（無励磁）・`AXIS_SLEEP`遷移（既存挙動のまま） |
| 1 | HOLD_FULL | アイドルタイムアウトによる無励磁遷移を無効化。アイドル中も`DRV_EN`=Low（100%、フル電流）を維持し続ける |
| 2 | HOLD_REDUCED | アイドルタイムアウトを無効化。全軸アイドルになった瞬間から`DRV_EN`を`SET HOLD_CURRENT_PERCENT`指定の比率でPWMチョッピングし、平均電流を下げつつ保持する。いずれかの軸が動き出した瞬間、即座にチョッピングを止め100%（フル電流）へ復帰する |

- `SET HOLD_CURRENT_PERCENT <1-100>`：HOLD_REDUCEDのチョッピング比率（%、100=フル電流相当、既定30）。
- `SET HOLD_MODE`/`SET HOLD_CURRENT_PERCENT`は`SET IDLE_TIMEOUT`と同様、モーション中も変更可能
  （チョッピングは全軸アイドル時のみ作動するため、実行中のモーションを妨げない）。
- チョッピング周波数はDRV8825巻線インダクタンスに対して十分高速（電気的時定数より短周期）かつ可聴域外の
  20kHz固定とし、コマンドでは変更しない（実装定数、`motor_ctrl.c`の`DRV_EN_LEDC_FREQ_HZ`）。
- `SET/GET HOLD_MODE`・`SET/GET HOLD_CURRENT_PERCENT`はNVSへ即時書き込みせず、`SAVE`コマンドで
  永続化する（`SET IDLE_TIMEOUT`と同じパターン）。

**反映先：** Section 4.3（`GET HOLD_MODE`/`GET HOLD_CURRENT_PERCENT`）、
Section 4.4（`SET HOLD_MODE`/`SET HOLD_CURRENT_PERCENT`）

---

#### F-MOT-14: モータータイプ（軸ごとのエンコーダ有無設定、2026-08-13新設）

**目的：** 全3軸が閉ループ（エンコーダ付き）前提だった設計を改め、軸ごとにエンコーダ非搭載の
ステッピングモーター（オープンループ）を選択できるようにする。

**設定（`SET MOTOR_TYPE <axis> <0|1>` / `GET MOTOR_TYPE <axis>`、軸ごと）：**

| 値 | 名称 | 動作 |
|---|------|------|
| 0 | `MOTOR_TYPE_CLOSED_LOOP`（デフォルト） | 既存動作のまま：脱調検出（F-MOT-08）・PCNT/Z相エンコーダ初期化・Z相ホーミング有効 |
| 1 | `MOTOR_TYPE_OPEN_LOOP` | 脱調検出を無効化。PCNT/Z相GPIOは初期化しない（未接続GPIOのフローティングノイズ誤カウント防止）。ホーミングは multi_i2c_bridge（AS5600、F-GEAR系）の絶対角度を用いる |

**オープンループ軸のホーミング（`HOME <axis>`）：**

- gear_monitor（multi_i2c_bridge 経由 AS5600）が示す出力軸の較正済み絶対角度
  （0度＝「0位置設定」機能で磁石取付誤差を補正済みの原点）へ向けて、通常の`MOVETO`と
  同一の台形速度プロファイルで移動する。Z相インデックス探索は行わない
- gear_monitor が `UNAVAILABLE`／未較正（該当軸の角度センサ未 `enabled`／`ok`）の場合は
  `HOME` を拒否する（軸は動かない）
- 移動完了時、`home_offset_steps`（既存 F-MOT-10 パラメータ）を現在位置として確定し、
  蓄積したオープンループ位置ドリフトを補正する。以降は `gear_monitor_mark_home()` に
  よりギア角度モニタの乖離検出（F-GEAR系）の基準点も更新される
- タイムアウトは既存の通信ウォッチドッグ・SOFT_LIMIT等、通常モーションと同じ保護機構に従う
  （Z相ホーミング専用の`HOME_TIMEOUT_US`は適用されない）

**制約：**

- `SET MOTOR_TYPE` はモーション中（`motor_is_moving()`）は拒否する（E008）
- `GET ENC <axis>` / `GET ENC_DEG <axis>` はオープンループ軸に対して `ERR E002 INVALID_PARAM`
  を返す（未装備のため）
- `SET/GET MOTOR_TYPE` はNVSへ即時書き込みせず、`SAVE`コマンドで永続化する
  （`SET ENC_DIR`と同じパターン）
- BLE `JOINT_ANGLE` テレメトリ（F-MOT-12）は、いずれかの軸がオープンループでも他軸分まで
  丸ごと失敗しない。各軸オブジェクトに`enc_available`等の有効性フラグを追加し、
  オープンループ軸は `enc_deg` を無効値として個別に示す

**反映先：** Section 3.2（エンコーダ初期化のスキップ条件）、Section 4.4（`SET/GET MOTOR_TYPE`）

---

### 3.2 エンコーダ読み取り（3ch）

#### F-ENC-01: 位置カウント
- A/B相の4逓倍デコードにより位置カウントを維持する（17HS15-1504-ME1K: 1000 PPR × 4 = 4000 カウント/rev）
- カウンタ：32bit 符号付き整数（オーバーフロー処理あり）
- PCNT ペリフェラルを使用（3ch 独立）

#### F-ENC-02: Z相（インデックス）検出
- Z相の立ち上がりエッジでインデックスイベントを発生させる
- 原点出し（ホーミング）動作のトリガーとして使用する

#### F-ENC-03: 速度推定

速度域に応じて2方式を切り替える：

**高速域（> 500 steps/sec）：差分カウント法**
- 算出式：speed = Δcount / Δt
- 算出周期：10 ms（MotorControlTask に同期）

**低速域（≤ 500 steps/sec）：エッジ間時間計測法**
- 算出式：speed = 1 / T_edge
- エッジ発生時に `esp_timer_get_time()` で µs 精度のタイムスタンプを記録し、連続エッジ間の経過時間 T_edge を速度換算する
- PCNT の割り込み（またはコールバック）でタイムスタンプを更新する

**速度 = 0 の判定：**
- 最後のエッジから 200 ms 以上エッジが来ない場合、速度 = 0 と判定する
- MotorControlTask の 10 ms ループで `esp_timer_get_time() - last_edge_us > 200,000` を確認する

### 3.3 ADCモニタリング

#### F-ADC-01: 定周期サンプリング・ノイズ対策

**ハードウェアフィルタ（実装済み）**
- NJM2114 オペアンプによる 1 kHz 1次ローパスフィルタが各 ADC チャンネルに実装されている
- カットオフ周波数：1 kHz（ハードウェア固定）

**サンプリング**
- 4ch の ADC 値を定周期（1 ms〜10 ms）でサンプリングする
- サンプリング周期はコマンドで設定可能にする
- 各サンプリングごとに `adc_oneshot_read` を 4 回コールし、その平均値を 1 サンプルとして扱う（ESP32-S3 ADC の単発読み取りノイズ低減）

**ソフトウェア移動平均フィルタ**
- 各チャンネル独立に移動平均フィルタを適用する
- 実装：リングバッファ方式（FIFO、窓サイズ N サンプル分のバッファを保持）
- フィルタ出力 = 直近 N サンプルの算術平均
- 窓サイズ N はチャンネルごとに設定可能（NVS 保存）

| パラメータ | 最小 | 最大 | デフォルト | 説明 |
|-----------|------|------|-----------|------|
| `adc_filter_window` | 1 | 64 | 8 | 移動平均窓サイズ（サンプル数）。1 = フィルタ無効（パススルー） |

- 窓サイズ変更時はリングバッファをリセットし、新しいサンプルが蓄積されるまで現在の最終値で埋める（過渡応答を最小化）
- `SET ADC_FILTER <ch|ALL> <N>` コマンドで実行時変更可能

**ADC キャリブレーション**
- 起動時に `esp_adc_cal_characterize`（ESP-IDF v5.x では `adc_cali_create_scheme_*`）を実行し、ADC の非線形補正を適用する
- キャリブレーション係数（gain, offset）は NVS 名前空間 `adc_config` に保存する

#### F-ADC-02: 電流・電圧換算

**電流センス（ADC4 = GPIO5）：**

実機回路：10 mΩ シャント抵抗 + LT6106 電流センスアンプ（Rin=120Ω、Rout=5kΩ）

| 項目 | 値 |
|------|-----|
| シャント抵抗 R_shunt | 10 mΩ (0.010 Ω) |
| 電流センスアンプ | LT6106（Rin=120 Ω、Rout=5000 Ω） |
| アンプゲイン | Rout/Rin = 5000/120 ≈ 41.67 倍 |
| ADC 入力電圧 | V_adc[V] = I[A] × 0.010 × (5000/120) = I[A] × 0.4167 |
| 換算式 | I[mA] = V_adc[mV] × Rin / (R_shunt × Rout) = V_adc[mV] × 2.4 |

```c
// ESP32-S3 ADC: 12 bit, Vref = 3.3 V
// 実機: 10mΩ シャント + LT6106 (Rin=120Ω, Rout=5kΩ) → 換算係数 ×2.4
#define CURRENT_R_SHUNT   0.010f   /* シャント抵抗 [Ω] (10 mΩ) */
#define CURRENT_R_IN      120.0f   /* LT6106 Rin [Ω] */
#define CURRENT_R_OUT     5000.0f  /* LT6106 Rout [Ω] */
#define CURRENT_CONV_FACTOR  (CURRENT_R_IN / (CURRENT_R_SHUNT * CURRENT_R_OUT))  /* = 2.4 mA/mV */

float v_adc_mv = adc_raw * (3300.0f / 4095.0f);
float current_mA = v_adc_mv * CURRENT_CONV_FACTOR;  /* × 2.4 */
```

動作範囲：0 mV → 0 mA、2500 mV → 6000 mA（6 A）、3300 mV → 7920 mA（ADC 満量程）

- 換算係数（Rin/Rout/R_shunt）は NVS（`adc_config` 名前空間）に保存し実行時変更可能とする

**電源電圧モニタ（ADC3 = GPIO4）：**

- MOT_V（24V系）を分圧回路で 3.3V レンジに変換して ADC3 で読み取る
- 換算係数（分圧比）は NVS に保存し `SET VOLT_DIVIDER <ratio>` コマンドで変更可能とする
- 換算式：`V_mot[V] = V_adc[V] × divider_ratio`

#### F-ADC-03: 過電流保護
- 電流値が閾値を超えた場合、該当軸のモーターを緊急停止する
- 閾値はコマンドで設定可能にする

### 3.4 通信・コマンドインターフェース

#### F-COM-01: コマンドパーサ
- USB-CDC 経由でテキストまたはバイナリコマンドを受信・解析する
- コマンドは改行（`\n`）で終端する（テキストモードの場合）

#### F-COM-02: 応答送信
- コマンド実行結果（ACK/NAK、データ値）を返送する
- 非同期イベント（エラー、ホーミング完了等）も通知する

#### F-COM-03: ハートビート
- 100ms周期でホスト向けにステータスパケットを送信する（オプション）

#### F-COM-04: 通信ウォッチドッグ

**動作仕様：**
- 最後のコマンド受信から `comm_timeout_ms`（デフォルト 5000 ms）経過した場合、全軸に STOP（減速停止）を発動する
- タイムアウト発動時：
  1. 全軸を台形減速プロファイルで停止する（STOP ALL 相当）
  2. 停止完了後、全軸状態を IDLE に遷移する（コイル励磁は維持）
  3. `EVT COMM_TIMEOUT` をシリアルバッファに書き込む（USB 再接続時にホストが受信）
- USB 切断イベント検出時（`tinyusb_cdcacm` の disconnection コールバック）：
  上記タイムアウトと同じ減速停止シーケンスを即時開始する
- 再接続後の復帰手順：
  ホストが `STATUS` コマンドを送信してセッション再確立する。全軸 IDLE 状態のため、そのままモーションコマンドを送信可能とする（ENABLE 不要）
- `SET COMM_TIMEOUT <ms>` で変更可能（0 = 無効、無効時は USB 切断イベントのみ監視する）
- `comm_timeout_ms` は NVS に保存し、電源投入時に復元する（F-MOT-10 NVS テーブル参照）
- ウォッチドッグタイマは `SET COMM_TIMEOUT` コマンド受信時もリセットする

#### F-COM-05: 基板固有ID（複数基板識別用）

**目的：** 複数基板を同時に USB 接続する運用（例: 6軸ロボットアームを 3ch 基板 2 枚で構成）において、
ホスト側アプリケーションが COM ポート番号に依存せず基板を一意に識別できるようにする。

**採用方針：** ESP32-S3 の eFuse に工場出荷時点で書き込まれる factory MAC アドレス（48bit、書き換え不可、
チップごとにグローバルユニーク）を基板固有 ID として使用する。

**実装内容：**
- 起動時に `esp_efuse_mac_get_default()`（または `esp_read_mac(mac, ESP_MAC_WIFI_STA)`）で MAC アドレスを取得する
- `GET BOARD_ID` コマンドで 12 桁の 16 進文字列（コロン無し、大文字、例: `AABBCCDDEEFF`）として返す
- 同じ文字列を USB ディスクリプタの `iSerialNumber` にも設定する（Section 2.4）
  - これによりホスト側は USB 列挙情報（Windows のデバイスマネージャ相当、Node.js `serialport.list()` の
    `serialNumber` プロパティ）からシリアルポートを開く前に基板を識別できる
- MAC アドレス取得に失敗した場合（eFuse 未書き込み等の異常系）は `GET BOARD_ID` に対し `ERR E012 BOARD_ID_UNAVAILABLE` を返す

**反映先：** Section 2.4（USB通信）、Section 4.3（GET BOARD_ID コマンド）、Section 4.6（E012 追加）

#### F-COM-06: エラーログ（FAULT/エラー系イベントの履歴保持）

**目的：** `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE` はUARTデバッグコンソール（GPIO43/44）にのみ出力され、
USB-CDC/BLE接続のホストからは参照できない。FAULT発生・コマンド拒否・通信タイムアウト等の異常系イベントを
接続後にさかのぼって確認できるよう、ファームウェア内にリングバッファで履歴を保持する。

**実装内容：**
- 直近 24 件（`ERROR_LOG_CAPACITY`）の異常系イベントをリングバッファに保持する（超過時は最古を上書き）
- 記録対象：全ての `ERR <code> ...` 応答、および `EVT <name> ...` のうち成功系（`MOVE_DONE` / `HOME_DONE` /
  `SYNC_DONE` / `GEAR_RECOVERED` / `GEAR_AVAILABLE`）を除く全イベント（`FAULT` / `COMM_TIMEOUT` /
  `OVERCURRENT` / `LIMIT_HIT` / `MOVE_ABORTED` / `SYNC_ABORTED` / `HOME_TIMEOUT` / `GEAR_DEGRADED` /
  `GEAR_UNAVAILABLE` / `GEAR_DEVIATION_WARN` 等）
- 各エントリは発生時刻（`esp_timer_get_time()`、ブート起点のマイクロ秒）・単調増加シーケンス番号・
  コード・詳細メッセージを保持する
- `GET LOG` で履歴全件を取得、`LOG_CLEAR` で明示的に消去する（`CLEAR_FAULT` と同じく自動クリアは行わない）
- BLE（読み取り専用）にも最新1件+シーケンス番号を通知経由で公開する（[BLE_WIFI_REQUIREMENTS.md §4.1](BLE_WIFI_REQUIREMENTS.md) Error Log キャラクタリスティック参照）。
  複数基板同時運用時にBLE経由で異常発生に気づけるようにする目的で、全履歴ではなく最新1件のみとする
  （BLE MTU制約のため、全履歴が必要な場合はUSB-CDCの `GET LOG` を使用する）。WiFi側は本タスクではスコープ外
  （robot_arm_monitorはBLEのみを使用するため）

**反映先：** Section 4.3（`GET LOG`/`LOG_CLEAR`）、[BLE_WIFI_REQUIREMENTS.md §4.1](BLE_WIFI_REQUIREMENTS.md)（Error Log キャラクタリスティック）

#### F-COM-07: フォルトトレース（STALL等のエラー直前挙動解析用リングバッファ）

**目的：** `GET LOG`（F-COM-06）はイベント発生の事実のみを記録し、発生直前の位置・速度・エンコーダ偏差の
時系列推移は残らない。STALL/OVERCURRENT等の原因切り分けには直前挙動の時系列データが必要なため、
軸ごとに一定間隔でサンプリングしたリングバッファを保持する。

**実装内容：**
- 軸ごとに 10ms 間隔で以下を `fault_trace_sample_t` としてリングバッファに記録する（直近 400 件 = 4 秒分、
  `FAULT_TRACE_CAPACITY`/`FAULT_TRACE_INTERVAL_MS`）：軸状態（`axis_state_t`）・`step_pos`・
  `enc_steps`（F-MOT-08 の脱調判定と同じ換算式）・`diff`（`enc_steps - step_pos`）・符号付き速度・
  電流値（`GET ADC 4` と同一センサ、全軸共通値を参考記録）
- 記録は状態に関わらず常時継続する（FAULT発生時も凍結しない）。ホスト側（control_app）が
  `EVT FAULT` 受信直後に `GET FAULT_TRACE` を発行して取得することを前提とする
- `GET FAULT_TRACE <axis>` で古い→新しい順に全件をJSON配列で返す（サンプル間隔は固定のため個々の
  タイムスタンプは持たず、応答の `interval_ms` から呼び出し側が経過時間を逆算する）

**反映先：** Section 4.3（`GET FAULT_TRACE`）

---

## 4. コマンドセット

### 4.1 フォーマット

```
テキストモード（USB-CDC）：
  要求: <COMMAND> [ARG...]\n
  応答: OK [data]\n  または  ERR <code> <message>\n
  非同期通知: EVT <EVENT> [data]\n

axis: 0〜2  または  ALL
```

### 4.2 モーション制御コマンド

| コマンド | 引数 | 説明 | 応答例 |
|---------|------|------|--------|
| `MOVE <axis> <steps>` | steps: ±int32 | 相対移動（台形プロファイル） | `OK` |
| `MOVETO <axis> <pos>` | pos: int32 | 絶対位置移動 | `OK` |
| `MOVE_DEG <axis> <deg>` | deg: ±float | 相対移動（度単位、F-MOT-12。内部でstepsへ換算し`MOVE`と同一経路で実行） | `OK`（完了時 `EVT MOVE_DONE <axis>`） |
| `MOVETO_DEG <axis> <deg>` | deg: float | 絶対位置移動（度単位、F-MOT-12。内部でstepsへ換算し`MOVETO`と同一経路で実行） | `OK`（完了時 `EVT MOVE_DONE <axis>`） |
| `SYNC_MOVE <n> <ax0> <st0> [<ax1> <st1>...]` | n:2〜3, ax:0〜2, st:±int32 | 多軸同期移動（F-MOT-11） | `OK`（完了時 `EVT SYNC_DONE <mask>`） |
| `VEL <axis> <speed>` | speed: int32（負=逆転） | 速度制御モード（連続回転） | `OK` |
| `STOP <axis>` | ALL 可 | 減速停止 | `OK` |
| `STOP_FREE <axis>` | ALL 可 | 減速停止後コイル解除 | `OK` |
| `ESTOP` | - | 全軸緊急停止（即時） | `OK` |
| `HOME <axis>` | - | ホーミング動作開始 | `OK`（完了時 `EVT HOME_DONE <axis>`） |
| `ENABLE <axis>` | ALL 可 | DRV_EN Low（励磁） | `OK` |
| `DISABLE <axis>` | ALL 可 | DRV_EN High（コイル解除） | `OK` |
| `CLEAR_FAULT` | - | フォルト復帰（全軸一括、F-MOT-07b 参照） | `OK` |

### 4.3 状態取得コマンド

| コマンド | 引数 | 説明 | 応答例 |
|---------|------|------|--------|
| `GET POS <axis>` | - | ステップカウンタ位置取得 | `OK 12800` |
| `GET ENC <axis>` | - | エンコーダカウンタ取得 | `OK 51200` |
| `GET VEL <axis>` | - | 現在速度取得（steps/sec） | `OK 5000` |
| `GET VMAX <axis>` | - | 最高速度設定値取得（steps/sec） | `OK 10000` |
| `GET ACCEL <axis>` | - | 加速度設定値取得（steps/sec²） | `OK 20000` |
| `GET DECEL <axis>` | - | 減速度設定値取得（steps/sec²） | `OK 20000` |
| `GET ADC <ch>` | - | ADC 電流値取得（mA） | `OK 850` |
| `GET STATE <axis>` | - | 軸状態取得 | `OK IDLE` |
| `GET FAULT_INFO` | - | 最後のフォルト情報取得（F-MOT-07c） | `OK OVERCURRENT 0x02 1234567890` |
| `GET LOG` | - | エラーログ履歴取得（最大24件、古い→新しい順、F-COM-06） | `OK [{"t":123456,"seq":3,"code":"E005","msg":"FAULT"},...]` |
| `LOG_CLEAR` | - | エラーログ履歴を消去（F-COM-06） | `OK` |
| `GET FAULT_TRACE <axis>` | - | 10ms間隔・直近400件（4秒分）のstate/step_pos/enc_steps/diff/vel/電流トレース取得（F-COM-07、古い→新しい順、上書き式リングバッファ・凍結なし） | `OK {"interval_ms":10,"axis":0,"count":400,"samples":[[2,12800,12790,-10,5000,850],...]}`（各要素は`[state,step_pos,enc_steps,diff,vel,current_mA]`、stateはaxis_state_t数値） |
| `GET BOARD_ID` | - | 基板固有ID取得（F-COM-05） | `OK AABBCCDDEEFF` |
| `GET POS_DEG <axis>` | - | ステップ位置の角度換算値取得（F-MOT-12） | `OK 45.230` |
| `GET ENC_DEG <axis>` | - | エンコーダ位置の角度換算値取得（F-MOT-12） | `OK 45.180` |
| `GET POT_DEG <axis>` | - | POT角度取得（補正なし・ゼロ位置補正ありの2値、F-MOT-12） | `OK 45.560 0.120`（raw zeroed の順） |
| `GET HOLD_MODE` | - | 保持電流モード取得（F-MOT-13） | `OK 2` |
| `GET HOLD_CURRENT_PERCENT` | - | HOLD_REDUCEDのチョッピング比率取得（F-MOT-13） | `OK 30` |
| `STATUS` | - | 全軸サマリー（JSON） | `OK {...}` |

**GET STATE の返却値**

| 値 | 説明 |
|---|------|
| `IDLE` | 停止・励磁中 |
| `SLEEP` | スリープ中 |
| `ACCEL` | 加速中 |
| `CRUISE` | 最高速維持中 |
| `DECEL` | 減速中 |
| `HOMING` | ホーミング動作中 |
| `FAULT` | フォルト停止中 |

### 4.4 設定コマンド

| コマンド | 引数 | 説明 |
|---------|------|------|
| `SET MICROSTEP <div>` | 1/2/4/8/16/32 | マイクロステップ設定（全軸共通・全停止後のみ） |
| `GET MICROSTEP` | - | マイクロステップ分周比取得（全軸共通） |
| `SET VMAX <axis> <v>` | steps/sec | 最高速度設定（NVS 保存） |
| `SET ACCEL <axis> <a>` | steps/sec² | 加速度設定（NVS 保存） |
| `SET DECEL <axis> <d>` | steps/sec² | 減速度設定（NVS 保存） |
| `SET ADC_FILTER <ch\|ALL> <N>` | N: 1〜64 | ADC 移動平均窓サイズ設定（1=フィルタ無効）（NVS 保存） |
| `SET CURRENT_LIMIT <axis> <mA>` | mA | 過電流閾値設定（`axis`引数は構文上の互換のためのみで実際は全軸共通1センサの閾値、F-ADC-03） |
| `GET CURRENT_LIMIT` | - | 過電流閾値取得（全軸共通） |
| `SET HOME_DIR <axis> <dir>` | +1 or -1 | ホーミング方向設定 |
| `SET IDLE_TIMEOUT <ms>` | ms | アイドルスリープ時間設定 |
| `SET HOLD_MODE <mode>` | 0=NORMAL/1=HOLD_FULL/2=HOLD_REDUCED | 保持電流モード設定（全軸共通、F-MOT-13） |
| `SET HOLD_CURRENT_PERCENT <pct>` | 1〜100 | HOLD_REDUCEDのチョッピング比率設定（NVS 保存、F-MOT-13） |
| `SET COMM_TIMEOUT <ms>` | ms | 通信ウォッチドッグタイムアウト設定（0=無効） |
| `SET STALL_FAULT <axis> <steps>` | steps | 脱調フォルト閾値設定（軸ごと、モーション中は拒否・E004） |
| `GET STALL_FAULT <axis>` | - | 脱調フォルト閾値取得（軸ごと） |
| `SET MOTOR_TYPE <axis> <0\|1>` | 0=CLOSED_LOOP/1=OPEN_LOOP | モータータイプ設定（軸ごと、モーション中は拒否・E008、F-MOT-14） |
| `GET MOTOR_TYPE <axis>` | - | モータータイプ取得（軸ごと、F-MOT-14） |
| `SET POT_SCALE <axis> <deg_per_count>` | float | POT角度換算係数の校正（NVS 保存、F-MOT-12） |
| `SET POT_ZERO <axis>` | - | 現在のPOT ADC生値をゼロ位置オフセットとして記録（NVS 保存、F-MOT-12） |
| `CLEAR POT_ZERO <axis>` | - | POTゼロ位置オフセットを0にリセット（NVS 保存、F-MOT-12） |
| `SAVE` | - | 現在の設定を NVS に保存 |
| `LOAD` | - | NVS から設定を読み込み |
| `RESET_CONFIG` | - | 設定を工場出荷デフォルトに戻す |

### 4.5 非同期イベント通知

| イベント | データ | 説明 |
|---------|--------|------|
| `EVT HOME_DONE <axis>` | - | ホーミング完了 |
| `EVT HOME_TIMEOUT <axis>` | - | ホーミングタイムアウト |
| `EVT STALL_WARN <axis> <err>` | err: step偏差 | 脱調警告（閾値超過） |
| `EVT STALL_FAULT <axis>` | - | 脱調フォルト（緊急停止済み） |
| `EVT OVERCURRENT <axis> <mA>` | mA: 電流値 | 過電流検出（緊急停止済み） |
| `EVT MOVE_DONE <axis>` | - | 位置制御移動完了 |
| `EVT MOVE_ABORTED <axis>` | - | MOVE/MOVETO が VEL コマンドにより中断された |
| `EVT SYNC_DONE <axis_mask>` | mask: ビットフィールド | 多軸同期移動が全軸完了（F-MOT-11） |
| `EVT SYNC_ABORTED <axis_mask>` | mask: ビットフィールド | SYNC_MOVE が ESTOP/STOP/リミットにより中断（F-MOT-11） |
| `EVT LIMIT_HIT <axis>` | - | ソフトリミット到達 |
| `EVT COMM_TIMEOUT` | - | 通信ウォッチドッグ発動（減速停止完了・IDLE 遷移済み） |

### 4.6 エラーコード

| コード | 説明 |
|--------|------|
| `E001` | 不明なコマンド |
| `E002` | 引数不足または型エラー |
| `E003` | 軸番号範囲外（0〜2 以外） |
| `E004` | モーション中に設定変更不可 |
| `E005` | フォルト状態（軸が `AXIS_FAULT`。原因は `ESTOP`コマンド／ホーミングタイムアウト／`OVERCURRENT`／`STALL` のいずれか、F-MOT-07。復帰は `CLEAR_FAULT` のみ、F-MOT-07b。`ESTOP` コマンドの再送では解除されない） |
| `E006` | ソフトリミット到達 |
| `E007` | ホーミング未完了（絶対位置移動不可） |
| `E008` | モーション実行中のモーションコマンド（MOTION_IN_PROGRESS） |
| `E009` | DRV 未有効化状態でのモーションコマンド（NOT_ENABLED、`ENABLE` が必要） |
| `E010` | FAULT 状態ではない（NOT_IN_FAULT） |
| `E011` | SYNC_MOVE 軸番号重複（DUPLICATE_AXIS） |
| `E012` | 基板固有ID取得不可（BOARD_ID_UNAVAILABLE、eFuse 異常等） |

---

## 5. 非機能要件

### 5.1 リアルタイム性
- ステップパルス生成はハードウェアタイマーまたはRMT周辺機能を使用し、ソフトウェアジッタを最小化する
- エンコーダ割り込み応答時間：< 10 µs
- コマンド処理レイテンシ：< 5 ms

### 5.2 安全機能
- 過電流検出時の即時停止（ADC 電流監視による）
- 脱調検出時の即時停止（エンコーダ偏差監視による）
- DRV8825 FAULT ピンは 3.3V プルアップのみで GPIO 未接続のため、ハードウェア FAULT を直接監視できない（DRV8825 の熱保護・過電流・短絡 FAULT はエンコーダ偏差・ADC 電流降下で間接的に検知する）
- ウォッチドッグタイマによるフリーズ検出とシステムリセット
- ソフトリミット（最大位置・最小位置）の設定

### 5.3 信頼性
- NVS（Non-Volatile Storage）に設定パラメータを保存し、電源断後も維持する
- 起動時に全DRV8825の初期化確認を行う

### 5.4 デバッグ機能
- USB-CDC 経由のログ出力（ログレベル切り替え可能）
- JTAG デバッグポート対応（ESP32-S3 内蔵USB-JTAG）

---

## 6. ソフトウェアアーキテクチャ（案）

```
firmware/
├── main/
│   ├── main.c              # エントリポイント、初期化
│   ├── motor_ctrl.c/h      # ステッピングモーター制御（DRV8825）
│   ├── encoder.c/h         # エンコーダ読み取り・デコード
│   ├── adc_monitor.c/h     # ADCサンプリング・電流換算
│   ├── comm.c/h            # USB-CDC 通信・コマンドパーサ
│   ├── config.c/h          # NVS設定パラメータ管理
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig.defaults      # ESP-IDF デフォルト設定
└── REQUIREMENTS.md         # 本ドキュメント
```

### 開発フレームワーク
- **ESP-IDF** v5.x（Espressif 公式 SDK）
- FreeRTOS ベースのマルチタスク設計
- ステップパルス生成：ESP32-S3 RMT（Remote Control Transceiver）または MCPWM 使用
- エンコーダデコード：PCNT（パルスカウンタ）ペリフェラル使用

### タスク構成

| タスク名 | 優先度 | スタック | 周期 | 説明 |
|---------|--------|---------|------|------|
| MotorControlTask | 20（高） | 4096 B | 1 ms | 速度プロファイル計算・RMT 周波数更新・脱調検出 |
| EncoderTask | 21（最高） | 2048 B | 割り込み | PCNT カウンタ読み取り・Z 相処理 |
| ADCTask | 15（中高） | 2048 B | 10 ms | ADC サンプリング・電流換算・過電流判定 |
| CommTask | 10（中） | 4096 B | 常時待機 | コマンド受信・パース・応答送信 |
| StatusTask | 5（低） | 2048 B | 100 ms | ハートビート・ログ出力 |

### 軸状態機械

```
          ┌─────────────────────────────────────────────────────┐
          │                     FAULT                           │
          │              (ESTOP / OVERCURRENT / STALL)          │
          ▼                                                     │
       [FAULT] ──CLEAR_FAULT──► [SLEEP]                         │
          ▲                      │                             │
          │                   ENABLE                           │
          │                      ▼                             │
          │                   [IDLE] ──────────────────────────►│
          │                 ▲  │  ▲                            │
          │    アイドルタイムアウト │  │ STOP完了                     │
          │                 │  │  │                            │
          │              MOVE/VEL │ STOP（減速完了）               │
          │                 │  ▼  │                            │
          │                 │ [ACCEL] ──► [CRUISE] ──► [DECEL] │
          │                 │    ▲          │                  │
          │                 │    └──VEL変更──┘                  │
          │                 │                                  │
          │            HOME開始                                 │
          │                 ▼                                  │
          │             [HOMING] ──Z相検出──► Z相再アプローチ ──► IDLE
          │                 │                                  │
          └─────────────────┘（FAULT 発生時 全状態から遷移）
```

### モーターコントロールタスク詳細設計（参考）

```c
// 1 ms タイマーコールバック内での処理イメージ
void motor_control_1ms_tick(axis_t *ax) {
    // 1. 速度プロファイル計算
    float dv = (ax->target_vel - ax->current_vel);
    float max_dv = ax->accel * 0.001f;  // accel × dt
    ax->current_vel += clamp(dv, -max_dv, max_dv);

    // 2. RMT 周波数更新（周期変更）
    if (ax->current_vel > 0) {
        uint32_t ticks = RMT_CLK_HZ / (uint32_t)ax->current_vel;
        rmt_update_period(ax->rmt_channel, ticks);
    }

    // 3. 脱調検出
    int32_t err = ax->encoder_pos - ax->step_pos;
    if (abs(err) > ax->stall_fault_th) {
        trigger_fault(ax, FAULT_STALL);
    }
}
```

---

## 7. 開発ロードマップ

### Phase 1：基本動作確認
- [x] ESP-IDF v5.x プロジェクトセットアップ（CMakeLists.txt、sdkconfig.defaults）
- [x] USB-CDC 通信確立（CommTask — USB Serial/JTAG VFS stdin/stdout）
- [x] RMT を使用した 3 軸ステップパルス生成（F-MOT-01、motor_ctrl.c）
- [ ] DIR セットアップ時間の実測確認（オシロスコープで 650 ns 以上を確認）
- [x] マイクロステップ設定（M0/M1/M2 GPIO 制御、config.c apply_microstep_gpio）
- [ ] DRV_EN / DRV_SLEEP 制御シーケンス確認（実機通電テスト要）

### Phase 2：3軸制御・モーションプロファイル
- [x] 3 軸独立 RMT チャンネル設定（F-MOT-01）
- [x] 台形加速プロファイル実装（F-MOT-04）
  - [x] 加速・定速・減速フェーズ切り替え（MotorControlTask 1ms ティック）
  - [x] 短距離移動時の三角プロファイル（残りステップ ≤ decel_steps で ACCEL→DECEL 自動遷移）
  - [x] 1 ms ティックでの RMT 周波数動的更新（set_rmt_freq → rmt_kick）
  - [x] DECEL 完了時の rmt_stop_channel によるキュー残バッチ消去（バグ修正）
- [x] 軸状態機械実装（IDLE / ACCEL / CRUISE / DECEL / SLEEP / FAULT）
  - [x] VEL 割り込みシーケンス（MOVE 中 VEL 受信 → DECEL 後に VEL モード再起動・EVT MOVE_ABORTED）
- [x] CLEAR_FAULT コマンド実装（F-MOT-07b：RESET パルス → SLEEP 遷移）
- [x] GET FAULT_INFO コマンド実装（F-MOT-07c）
- [x] アイドルタイムアウト・スリープ制御（F-MOT-03）
- [x] ソフトリミット（F-MOT-10）
  - [x] MOVE/MOVETO 開始時チェック（start_motion でクランプ）
  - [x] VEL モード中の動的監視 → DECEL 強制・EVT LIMIT_HIT 送出
  - [x] SET SOFT_LIMIT コマンド実装

### Phase 3：エンコーダフィードバック・ホーミング
- [x] PCNT によるエンコーダ 4 逓倍デコード（3 軸）（F-ENC-01）
- [x] Z 相インデックスパルス割り込み処理（F-ENC-02）
- [x] エンコーダ速度推定（F-ENC-03）
  - [x] encoder_update_10ms() を MotorControlTask の 1ms ループで 10ms 毎に呼び出し
  - [x] GET ENC コマンド実装（encoder_get_pos → OK \<count\>）
- [x] ホーミング動作実装（F-MOT-09 の 2 段階アプローチシーケンス）
  - [x] motor_home() 関数（AXIS_HOMING 状態遷移・タイムアウト管理）
  - [x] フェーズ 0 coarse: Z 検出で即時停止
  - [x] フェーズ 1 backoff: back_off_steps を逆方向移動
  - [x] フェーズ 2 fine: Z 再検出・位置ゼロセット
  - [x] HOME コマンド・EVT HOME_DONE / HOME_TIMEOUT 送出
  - [x] SET HOME_DIR コマンド実装
- [x] 脱調検出（F-MOT-08 の偏差閾値監視）
  - [x] ACCEL/CRUISE/DECEL 中に encoder_get_pos と step_pos の差分を 1ms 毎に監視
  - [x] |差分| > stall_fault_th → motor_estop(FAULT_STALL)
  - [x] motor_set_stall_fault_th() 関数・SET STALL_FAULT コマンド実装

### Phase 4：ADC モニタリング
- [x] NJM2114 出力 ADC 読み取り（12 bit、esp_adc_cal 適用）
- [x] 電流換算（ゲイン・シャント抵抗パラメータ適用）（LT6106実機回路: I[mA]=V_adc[mV]×2.4）
- [x] 過電流保護・緊急停止連動（F-ADC-03）（ヒステリシス付き、motor_estop(FAULT_OVERCURRENT) 連動）

### Phase 5：コマンドセット・設定管理・統合
- [x] コマンドセット完全実装（Section 4 全コマンド）
- [x] エラーコード・非同期イベント通知実装
- [x] NVS 設定保存・読み込み（全 F-MOT-10 パラメータ：vmax/accel/decel/stall_th/soft_limit/home 各軸）
- [x] 通信ウォッチドッグ実装（F-COM-04：タイムアウト減速停止・EVT COMM_TIMEOUT）
- [x] STATUS コマンドの JSON 応答フォーマット実装
- [x] ハートビート（StatusTask 100ms、HEARTBEAT ON/OFF で切替）
- [x] 多軸同期移動実装（F-MOT-11 SYNC_MOVE：速度スケーリング・同一ティック START・STOP/ESTOP/リミット連動・EVT SYNC_DONE/SYNC_ABORTED）
  - [x] 速度スケーリング値はモーション専用の一時プロファイルへ適用し、軸設定の `v_max / accel / decel` は永続変更しない（2026-06-23 修正）
- [x] 総合テスト（3 軸同時動作、ホーミング、脱調検出、フォルト復帰、SYNC_MOVE 直線補間確認）（T24〜T27 実機 PASS 2026-06-23）
- [x] 回帰テスト（T28〜T30 実機 PASS 2026-06-23）
  - [x] T26 後も axis0..2 の `vmax / accel / decel` が不変であることを確認

### Phase 6：関節角度出力（度単位）・POTゼロ位置補正（F-MOT-12、USB-CDC実装・実機確認済み）
- [x] `GET POS_DEG`/`GET ENC_DEG`（既存 F-MOT-10 パラメータからの角度換算）
- [x] `GET POT_DEG`（補正なし・ゼロ位置補正あり2値）
- [x] `SET POT_SCALE`/`SET POT_ZERO`/`CLEAR POT_ZERO`（NVS 保存）
- [x] `MOVE_DEG`/`MOVETO_DEG`（既存 `MOVE`/`MOVETO` と同一実行経路への角度→steps換算）
- [x] USB-CDC 実機動作確認（COM41 / Board ID `441BF6C8C148`、2026-08-08）
  - [x] T31: 軸0を `MOVETO_DEG 90°` → `MOVETO_DEG -45°` → `MOVE_DEG +45°`で最終0°へ復帰、steps/角度換算・`E008`・エンコーダ追従を確認
  - [x] T32: POTスケール・ゼロ位置設定/解除・引数エラーを確認
  - [x] T33: 単軸動作中は他軸のIDLE時間を算入せず、全軸停止後のみ共通ドライバがSLEEPへ遷移することを確認
- [ ] BLE Joint Angle キャラクタリスティック（[BLE_WIFI_REQUIREMENTS.md §4.1](BLE_WIFI_REQUIREMENTS.md)）への反映
- [ ] 実機での POT スケール・ゼロ位置校正手順の検証（robot_arm_monitor F-RAM-VERIFY パネルでの動作確認）

### Phase 7：保持電流モード（F-MOT-13、DRV_EN LEDC PWMチョッピング）
- [x] `hold_mode_t`（NORMAL/HOLD_FULL/HOLD_REDUCED）・`SET/GET HOLD_MODE`・`SET/GET HOLD_CURRENT_PERCENT`実装
- [x] `DRV_EN`をLEDC（20kHz固定）駆動へ統一し、`drv_en_set_percent()`経由に一本化
- [x] MotorControlTaskの全軸アイドル判定にチョッピング開始/停止ロジックを統合
- [x] NVS永続化（`SET IDLE_TIMEOUT`と同じ即時適用+`SAVE`永続化パターン）
- [x] ESP-IDFビルド確認（`idf.py build`成功）
- [ ] 実機でのDRV_EN波形確認（オシロ/テスタ、20kHz・指定デューティでのチョッピング動作、モーション開始時の即時100%復帰）

---

## 8. 制約事項・注意点

- ESP32-S3 の ADC は 3.3V 単電源のため、負電圧の検出はできない（NJM2114でオフセット処理が必要）
- DRV_EN と DRV_SLEEP は全軸共通信号のため、軸単位のスリープ制御は不可（同じ理由で保持電流モード F-MOT-13 も軸単位ではなく全軸共通のグローバル設定）
- DRV8825の実駆動電流（VREF）は各モジュール基板上の物理トリムポットで固定されており、ESP32からデジタル制御できない（F-MOT-13の保持電流可変はDRV_ENのPWMチョッピングによる近似であり、真の電流値制御ではない）
- ESP32-S3 の ADC は非線形特性があるため、キャリブレーション（esp_adc_cal）を必ず適用する
- USB-CDC は PC 側のドライバ不要（ESP32-S3 は USB Full Speed 対応）

---

## 9. 未解決事項・要件ギャップ（要確認）

本セクションは要件レビューで検出された不足・矛盾点を記録する。実装前に各項目の方針を決定し、上記セクションへ反映すること。

### 9.1 フォルト復帰手順（解決済み：提案 A 採用）

**採用方針：** 完全手動確認型。FAULT 復帰は常にオペレータの明示的なコマンド 1 本で完結する。ファームウェアは自動リトライを行わない。

### 9.2 [解決済み] 通信断時の安全動作（USB ウォッチドッグ）

**採用方針：** 減速停止 → IDLE 維持型。タイムアウトまたは USB 切断で全軸を台形減速停止し、IDLE 状態（コイル励磁維持）に遷移する。再接続後は STATUS コマンド 1 本でセッションを再確立でき、ENABLE 不要でそのままモーションコマンドを送信できる。

**却下した選択肢：**
- 即時停止 → FAULT：過剰に保守的。通信断のたびに CLEAR_FAULT が必要になり、運用が煩雑
- 減速停止 → コイル解除（SLEEP）：重力軸がある場合に落下リスクがあり、本プロジェクトには不適

**反映先：** F-COM-04（Section 3.4）、SET COMM_TIMEOUT（Section 4.4）、EVT COMM_TIMEOUT（Section 4.5）

### 9.3 [解決済み] 起動時 GPIO 初期状態とリセットシーケンス

**問題（解決済み）：**
- 電源 ON 直後（ファームウェア初期化前）の出力状態が未定義
- DRV8825 をアンノーン状態から既知状態にするリセットパルスが起動手順に含まれていない
- 旧 F-MOT-03 起動シーケンスは電源 ON と同時に全軸励磁されるため、意図せぬトルクが発生する

**採用方針：**
- `app_main()` の先頭で DRV_EN=High / DRV_SLEEP=Low / DRV_RESET=Low / STEP×4=Low を設定し、安全状態を確保する
- DRV_RESET パルス（10 µs 以上 Low → High）を含む 9 ステップの完全起動シーケンスを実行する
- 起動後の励磁は `ENABLE` コマンドを明示的に受信するまで行わない

**反映先：** F-MOT-03（Section 3.1）、Section 2.1（DRV_RESET 信号追加）

### 9.4 [解決済み] 同時コマンド処理ポリシー

**採用方針：** 安全優先の単純ポリシー。MOVE/MOVETO 実行中の同種コマンドは即時拒否し、VEL による上書きは減速後に移行する。

**同時コマンド処理ポリシー：**

| 受信コマンド | 実行中コマンド | ポリシー |
|------------|-------------|---------|
| `MOVE` / `MOVETO` | MOVE/MOVETO 実行中 | `ERR E008 MOTION_IN_PROGRESS` を返す（拒否） |
| `VEL` | MOVE/MOVETO 実行中 | 現在の MOVE を中断し、減速プロファイルで停止後に VEL モードへ移行する |
| `VEL` | VEL 実行中 | 現在速度から新速度へ加速/減速（上書き許可） |
| `STOP` / `ESTOP` | 任意 | 常に受付（最優先） |
| `SET` | MOVE/MOVETO/VEL 実行中 | `ERR E004 MOTION_IN_PROGRESS` を返す（拒否） |
| `GET` | 任意 | 常に受付 |
| `HOME` | MOVE/VEL 実行中 | `ERR E008 MOTION_IN_PROGRESS` を返す（拒否） |

**追加したエラーコード：**
- `E008 MOTION_IN_PROGRESS`：モーション実行中の競合コマンドを拒否する（Section 4.6 に追加済み）

**VEL 割り込みシーケンス（MOVE 実行中に VEL を受信した場合）：**
```
1. 現在のステップターゲットをキャンセルする
2. 減速プロファイル（accel パラメータを使用）で速度 0 まで減速する
3. 速度 0 到達後、VEL コマンドの速度に向けて加速フェーズを開始する
4. 軸状態：DECEL → IDLE（瞬時） → ACCEL → CRUISE の順に遷移する
5. EVT MOVE_ABORTED <axis> を送信してホストにキャンセルを通知する
```

**将来拡張（現 Phase では不要）：**
- `MOVE_QUEUE <axis> <steps>`：複数の MOVE コマンドをキューイングして連続実行する
- MOVE/MOVETO 実行中の上書きポリシー（現在は拒否）は、将来の Phase でキュー方式または割り込み許可への変更を検討する

**反映先：** Section 4.5（EVT MOVE_ABORTED 追加）、Section 4.6（E008 追加）

### 9.5 [解決済み] MS0/MS1/MS2 制御主体の矛盾

**採用方針：選択肢 B「GPIO のみ」**

ESP32-S3 が M0/M1/M2 を GPIO 出力で直接制御し、DIP スイッチを廃止する。

**GPIO 割り当て（確定）：**

| 信号 | GPIO | 説明 |
|------|------|------|
| M0（MS0） | GPIO41 | マイクロステップ bit0 |
| M1（MS1） | GPIO42 | マイクロステップ bit1 |
| M2（MS2） | GPIO45 | マイクロステップ bit2 |

**注記：GPIO22/23/24 は ESP32-S3 内部 SPI Flash 専用ピンのため使用不可。**

**機能への影響：**
- `SET MICROSTEP <div>` コマンドで実行時に動的変更可能（全軸停止後のみ）
- 設定値は NVS に保存され再起動後も維持される
- 起動シーケンスの Step 8 で GPIO41/42/45 を NVS 設定値に応じて出力する

**ハードウェア変更（回路図・基板への対応が必要）：**
1. schematic-plan.md の DIP スイッチ（3 ポジション）を削除する
2. MS0/MS1/MS2 ネットを ESP32-S3 の GPIO41/42/45 に直結する
3. MS ラインに 100Ω 程度の直列保護抵抗を追加することを推奨する（GPIO 破損防止）

**却下した選択肢：**
- 選択肢 A（DIP スイッチのみ）：`SET MICROSTEP` による動的変更が不可能になり、運用上の柔軟性が失われる
- 選択肢 C（DIP + GPIO オーバーライド）：回路が複雑になりトラブルシューティングが困難

**反映先：** F-MOT-02（Section 3.1）、起動シーケンス（F-MOT-03 Step 8）、Section 2.1 信号テーブル、CLAUDE.md GPIO マッピング（GPIO41/42/45 確定）

### 9.6 [解決済み] 多軸同期移動の欠如

**採用方針：** 移動時間一致方式（Duration-Matching）を採用。参照軸（最大 steps 軸）の台形プロファイル完了時間 T_ref に合わせ、従軸の v_max / accel / decel を比率スケーリングする。全軸の RMT 開始は MotorControlTask 同一 1ms ティック内でアトミックに実行する。

**反映先：** F-MOT-11（Section 3.1）、SYNC_MOVE コマンド（Section 4.2）、EVT SYNC_DONE / SYNC_ABORTED（Section 4.5）、E011（Section 4.6）、Phase 5 ロードマップ（Section 7）

**設計上の注意点：**
- RMT enable の同一ティック保証は CommTask → MotorControlTask のフラグ渡しで実現し、OS レベルの atomic は不要
- steps = 0 の軸は IDLE 維持（参照軸にはなれない）
- スケール後の v_max_i / accel_i が 1 steps/sec 未満になる場合は 1 に切り上げ（わずかな時間ズレが生じる可能性があるが許容範囲内とする）
- SYNC_MOVE 中のソフトリミット到達・脱調 FAULT は sync_group 全軸を連動停止する

### 9.7 [解決済み] ADC ノイズ対策の要件不足

**採用方針：** 2段階フィルタ構成（ハードウェア LPF + ソフトウェア移動平均）を採用。

**ノイズ対策の構成：**

| 段階 | 実装場所 | 方式 | パラメータ |
|------|---------|------|-----------|
| 第1段 | ハードウェア | NJM2114 による 1 kHz 1次 LPF | 固定（回路設計済み） |
| 第2段 | ファームウェア | 4回平均（単発読み取りノイズ低減） | 固定（4回） |
| 第3段 | ファームウェア | 移動平均フィルタ（リングバッファ） | N=1〜64、デフォルト8、NVS保存・動的変更可 |

**設計上の決定事項：**
- 窓サイズ N=1 でフィルタ無効（パススルー）とし、無効化のための別フラグは設けない
- 窓サイズはチャンネル毎に独立して設定可能（`ALL` 指定で全 ch 一括設定も可）
- 窓サイズ変更時のバッファリセット：新サンプルが蓄積されるまで直前の出力値で埋める
- ADC キャリブレーション（`adc_cali_create_scheme_*`）は起動時に実行し、係数を NVS 保存

**却下した選択肢：**
- 指数移動平均（EMA）：係数の直感的な理解が難しく、窓サイズで応答時間を制御できる単純平均を優先
- 窓サイズを 2の累乗に限定：除算コストは ESP32-S3 では無視できるため制限しない

**反映先：** F-ADC-01（Section 3.3）、SET ADC_FILTER コマンド（Section 4.4）、adc_filter_window パラメータ（F-ADC-01 パラメータ表）

### 9.8 [解決済み] RMT パルスとステップカウンタの同期

**問題：**
- RMT がハードウェアでパルスを生成し、ステップカウンタをソフトウェアで管理すると、実際のパルス数とカウンタがずれる可能性がある

**採用方針：** `on_trans_done` コールバック + 速度制御中の 1 ms 同期を組み合わせ、クリティカルセクションで更新する。

**反映先：** F-MOT-05（Section 3.1）

### ~~9.9~~ [解決済み] F-MOT-10 パラメータ不足

**採用方針：** 以下 6 パラメータを F-MOT-10 NVS テーブルに追加した。

| パラメータ名 | 型 | デフォルト | 説明 |
|------------|-----|-----------|------|
| `decel` | uint32 | 50,000 | 減速度（steps/sec²）（accel と独立して設定） |
| `home_offset_steps` | int32 | 0 | ホーミング後の原点オフセット（steps） |
| `v_home_coarse` | uint32 | 2,000 | ホーミング粗探索速度（steps/sec） |
| `v_home_fine` | uint32 | 500 | ホーミング精密探索速度（steps/sec） |
| `back_off_steps` | uint32 | 200 | ホーミングバックオフ距離（steps） |
| `comm_timeout_ms` | uint32 | 5,000 | 通信ウォッチドッグタイムアウト（ms、0=無効） |

**反映先：** F-MOT-10（Section 3.1 NVS テーブル）、F-MOT-09（ホーミングシーケンス）、F-COM-04（Section 3.4）

### ~~9.10~~ [Medium] ~~エンコーダ速度推定の低速精度問題~~（解決済み：F-ENC-03 を2方式切替に改訂）

~~**F-ENC-03 を以下に改訂：**~~

~~高速域（> 500 steps/sec）：差分カウント法（10 ms 周期）~~
~~低速域（≤ 500 steps/sec）：エッジ間時間計測法（esp_timer_get_time() で µs 精度）~~
~~速度 = 0：最後のエッジから 200 ms 以上経過で判定~~

**反映先：** F-ENC-03（Section 3.2）

### 9.11 [Medium] I2C 拡張インターフェースの要件欠落

Section 1.2 では I2C（GPIO38/39）を言及しているが機能要件が皆無。以下を追加：

```
F-I2C-01: I2C スキャン
  - 起動時に I2C バスをスキャンし、検出デバイスアドレスをログ出力する

F-I2C-02: 拡張デバイス対応（将来拡張）
  - I2C 経由のセンサ/IO エクスパンダ対応は Phase 後の拡張機能として扱う
  - 4.7kΩ プルアップは常時有効とし、バスは常時 400 kHz（Fast モード）で初期化する
```

### ~~9.12~~ [Medium] ~~DRV8825 熱保護 FAULT の区別~~（解決済み：GPIO 未接続確認、間接検知方式に確定）

**ハードウェア確認結果：**
- DRV8825 FAULT ピンは 3.3V プルアップ接続のみで、ESP32 GPIO には未接続
- ファームウェアは FAULT ピンの状態を一切読み取れない

**対応方針（F-MOT-07 / Section 5.2 に反映済み）：**
- DRV8825 の過電流・熱保護・短絡 FAULT は、いずれもドライバ出力遮断→モーター停止→エンコーダ偏差増大または電流降下として間接的に現れる
- 過電流 FAULT → ADC 電流監視（F-ADC-03）で `OVERCURRENT` として検知
- 熱保護・短絡 FAULT → エンコーダ偏差監視（F-MOT-08）で `STALL` として検知（原因種別の区別は不可）
- `EVT DRV_FAULT` イベントは削除（ハードウェア未接続のため発生不可能）
- `EVT FAULT` の reason から `DRV_HW` を削除、`ESTOP` / `OVERCURRENT` / `STALL` の 3 種に確定

**反映先：** F-MOT-07（Section 3.1）、Section 4.5 イベント、Section 5.2 安全機能

### 9.13 [Low] Status1 LED（GPIO46）点滅周期の妥当性

Section 2.5 で追加した Status1 LED の点滅周期（通信中2Hz／FAULT 8Hz）は暫定値。BLE/WiFi側の Status2 LED（[BLE_WIFI_REQUIREMENTS.md §3.1](BLE_WIFI_REQUIREMENTS.md) §9.7）と同一値で揃えているが、実機評価で視認性を確認し、必要に応じて両LED同時に調整する。

### 9.14 [解決済み] MOVE/MOVETO/MOVE_DEG/MOVETO_DEG の失敗理由が E006 SOFT_LIMIT に誤集約

**問題：** `start_motion()`（内部ヘルパー）は「DRV 未有効化（ENABLE 未実行、またはアイドルタイムアウトによる自動無効化後）」「軸が FAULT 状態」「RMT 起動失敗」のいずれでも `false` を返す。従来の `comm.c` はこの `false` を区別せず一律 `ERR E006 SOFT_LIMIT` として返していたため、実際にはソフトリミットに達していない失敗（未 ENABLE・FAULT）まで「ソフトリミット到達」と誤表示していた。なお実際のソフトリミット超過は `start_motion()` 内で `target` を `max_pos`/`min_pos` にクランプして正常に動作を開始するため、単軸コマンドの `false` 応答が本来の意味で SOFT_LIMIT になることはほぼ無い。

**対応：** `MOVE` / `MOVETO` / `MOVE_DEG` / `MOVETO_DEG` の各コマンドで、`motor_move`/`motor_moveto` 呼び出し前に以下の判定を追加：

1. 対象軸が `AXIS_FAULT` 状態 → `ERR E005 FAULT`（`SYNC_MOVE` の既存チェックと同様）
2. DRV 未有効化（新設 `motor_is_enabled()`） → `ERR E009 NOT_ENABLED`（新エラーコード）
3. 上記に該当しない場合のみ `motor_move`/`motor_moveto` を呼び出し、`false` が返れば従来通り `ERR E006 SOFT_LIMIT`

**反映先：** Section 4.2（`MOVE`/`MOVETO`/`MOVE_DEG`/`MOVETO_DEG` の事前チェック順序）、Section 4.6（E009 追加）、`motor_ctrl.h`/`motor_ctrl.c`（`motor_is_enabled()` 追加）

### 9.15 [解決済み] エラーログの蓄積・遡及参照ができない（F-COM-06 追加）

**問題：** 9.14 の調査過程で判明した根本的なギャップ。ファームウェアの異常系イベント（FAULT・コマンド拒否・通信タイムアウト等）は `ESP_LOGI` 相当としてUARTデバッグコンソールにのみ出力され、USB-CDC/BLE接続のホストアプリからは一切参照できなかった。`fault_info_t`（`GET FAULT_INFO`）も直近1件のみの保持で、接続前に発生した異常や複数回のFAULTの履歴は失われていた。

**対応：** `error_log` モジュール（`error_log.h`/`error_log.c`）を新設し、直近24件の異常系イベントをリングバッファで保持。`comm.c` の `comm_send()`（全ての `ERR`/`EVT` 応答が通る唯一のフックポイント）で自動分類・記録するため、個別コマンドハンドラの改修は不要。`GET LOG`/`LOG_CLEAR` コマンド（USB-CDC）と、Error Log キャラクタリスティック（BLE、最新1件+シーケンス番号のみ、読み取り専用）から取得可能にした。`SteppingMotorDriver/monitor_app`（USB接続）は `GET LOG` で全履歴を、`robot_arm_monitor`（BLE接続）は既存の生ログ表示機構（`rawLog`）へ新キャラクタリスティックの通知が自動的に流れ込む形で対応する。

**反映先：** F-COM-06（Section 3.4）、Section 4.3（`GET LOG`/`LOG_CLEAR`）、[BLE_WIFI_REQUIREMENTS.md §4.1](BLE_WIFI_REQUIREMENTS.md)（Error Log キャラクタリスティック）、`error_log.h`/`error_log.c`（新設）
