# ギアアウトプット角度モニタ機能 要件定義・要求仕様・実装仕様

SteppingMotorDriver ファームウェアに、`multi_i2c_bridge` 経由で各軸ギアボックスの
**アウトプット軸回転角度**（AS5600 磁気エンコーダ）を取得・モニタする機能を追加する。

| 項目 | 内容 |
|------|------|
| 対象 | SteppingMotorDriver ファームウェア（ESP32-S3, firmware/） |
| 関連 | [firmware/REQUIREMENTS.md](firmware/REQUIREMENTS.md)（本体要件定義） |
| 関連（外部） | multi_i2c_bridge/docs/[command_spec.md](../multi_i2c_bridge/docs/command_spec.md)（レジスタ確定仕様） / [i2c_architecture.md](../multi_i2c_bridge/docs/i2c_architecture.md)（通信アーキ・タイミング） / [as5600_config.md](../multi_i2c_bridge/docs/as5600_config.md)（センサ設定） / [controller_impl_notes.md](../multi_i2c_bridge/docs/controller_impl_notes.md)（実装注意点） |
| 版 | 0.1（初版） |
| 作成日 | 2026-07-25 |

---

## 1. 概要

### 1.1 目的

各軸（CH0〜CH2）のステッピングモーターはギアボックス経由で最終出力軸を駆動する。
現行ファームウェアはモーター軸に直結したインクリメンタルエンコーダ（PCNT, F-ENC-01〜03）
でモーター側の回転のみを検出しており、**ギアボックスのアウトプット側の実角度**を直接観測する
手段を持たない。

本機能は、`multi_i2c_bridge`（RP2040, I2C スレーブ 0x42, AS5600 磁気エンコーダ ×6ch 集約）を
SteppingMotorDriver の GPIO38(SDA)/GPIO39(SCL) 拡張 I2C バスに接続し、ギアボックスアウトプット
軸に取り付けた AS5600 の絶対角度を **CH0〜CH2 の 3ch**（モーター軸 CH0〜CH2 に 1:1 対応）で
定期取得する。

### 1.2 スコープ（利用用途）

本版でスコープに含める用途は以下の3点（優先度順）：

| # | 用途 | 本版での扱い |
|---|------|------|
| 1 | **モニタリング／テレメトリ** | 実装する（主目的）。`GET GEAR_ANGLE` / ハートビート等でホストに角度・健全性を公開する |
| 2 | **電源投入時の絶対位置認識** | 実装する。起動時に AS5600 の絶対角度を読み、モーター側位置（ステップカウンタ・ホーミング状態）との整合確認・ログ記録に用いる（§4.5, 多回転曖昧性の制約あり） |
| 3 | **将来のクローズドループ位置制御の外側ループ** | 本版では実装しない。データ経路・レジスタ・タスク構造のみ将来拡張を阻害しない形で設計する（§4.9） |

既存の F-MOT-08（モーター軸エンコーダによる脱調検出、PCNT ベース）には**影響を与えない**。
本機能はそれとは独立な、ギアボックス出力側の追加センサ系として位置付ける。

### 1.3 非対象（明示的に対象外）

- ギアボックス出力角度を用いた脱調検出・自動補正（将来検討、本版は対象外）
- 6ch 全チャンネルの利用（本ハードウェアは 3 軸のため CH0〜CH2 のみ使用、CH3〜CH5 は未接続として扱う）
- multi_i2c_bridge 側ファームウェアの変更（bridge は確定仕様のまま使用する前提）

---

## 2. システム構成

### 2.1 全体構成

```
┌─────────────────────────────┐        I2C (400kHz, Master)       ┌──────────────────────────┐
│ SteppingMotorDriver (ESP32-S3) │ ───────────────────────────────► │ multi_i2c_bridge (RP2040) │
│  GPIO38=SDA / GPIO39=SCL       │ ◄─────────────────────────────── │  Slave Addr 0x42          │
│  役割: I2C Master              │   レジスタ read/write（§5）        │  役割: I2C Slave           │
└─────────────────────────────┘                                    └──────────┬───────────────┘
                                                                                │ 下流 I2C (TCA9548A mux)
                                                                    ┌───────────┼───────────┐
                                                                    ▼           ▼           ▼
                                                                 AS5600      AS5600      AS5600
                                                              (ch0=軸0出力) (ch1=軸1出力) (ch2=軸2出力)
```

### 2.2 役割分担

| 主体 | I2C ロール | 説明 |
|------|-----------|------|
| SteppingMotorDriver (ESP32-S3) | **Master** | 本要件で新規実装する側。ホストコントローラとしては USB-CDC 経由で上位 PC と別系統。拡張 I2C バスでは bridge に対して Master となる |
| multi_i2c_bridge (RP2040) | **Slave（0x42）** | 既存の確定仕様（[command_spec.md](../multi_i2c_bridge/docs/command_spec.md)）をそのまま利用。ファームウェア変更なし |
| AS5600 ×3（bridge 下流） | — | bridge が完全に隠蔽。SteppingMotorDriver 側はレジスタ read/write のみ意識する |

### 2.3 チャンネル対応表

| bridge ch | 接続先 | SteppingMotorDriver 軸 | 備考 |
|-----------|--------|------------------------|------|
| CH0 | ギア0 アウトプット軸 AS5600 | axis 0（STEP0/DIR0） | |
| CH1 | ギア1 アウトプット軸 AS5600 | axis 1（STEP1/DIR1） | |
| CH2 | ギア2 アウトプット軸 AS5600 | axis 2（STEP2/DIR2） | |
| CH3〜CH5 | 未接続 | — | bridge の起動時自動検出（`CH_PRESENT`）により無効化される想定。`CH_ENABLE` の明示上書きは行わない（§4.1） |

---

## 3. ハードウェアインタフェース要件

### 3.1 I2C バス仕様

| 項目 | 値 | 根拠 |
|------|-----|------|
| SDA | GPIO38（既存 `GPIO_I2C_SDA`, [gpio_config.h](firmware/main/gpio_config.h)） | 既に拡張用として予約済み |
| SCL | GPIO39（既存 `GPIO_I2C_SCL`） | 同上 |
| バス速度 | 400 kHz（Fast） | bridge 側仕様の確定値（[command_spec.md §1.1](../multi_i2c_bridge/docs/command_spec.md)）。100kHz では bridge 側の 1kHz 一括読み要件を満たせないが、本機能は 10ms(100Hz) 周期読みのため 100kHz でも動作はするが、bridge 側確定仕様に合わせ 400kHz を採用する |
| スレーブアドレス | 0x42（7bit 固定） | [command_spec.md §1.1](../multi_i2c_bridge/docs/command_spec.md) |
| クロックストレッチ | 許容（無効化しない） | ESP-IDF I2C マスタドライバの既定動作のまま。bridge 側 ISR 準備の µs オーダのストレッチに対応（[controller_impl_notes.md §6](../multi_i2c_bridge/docs/controller_impl_notes.md)） |
| プルアップ | SteppingMotorDriver 側 or bridge 側の**どちらか一方のみ**に実装（二重実装禁止） | [controller_impl_notes.md §6](../multi_i2c_bridge/docs/controller_impl_notes.md)。本基板の実装状況を回路図で確認すること（要ハードウェア確認、§11） |
| 電圧 | 3.3V 系で統一 | bridge 側も 3.3V 確定（[i2c_architecture.md §11.1](../multi_i2c_bridge/docs/i2c_architecture.md)）。レベル変換不要 |

### 3.2 制約事項

- GPIO38/39 は本機能専用とし、他機能（他 I2C デバイス）との共用は本版では想定しない。
- 上流バス（USB-CDC、GPIO19/20）とは完全に独立した別バスであり、干渉しない。
- 本 I2C バスの Master は SteppingMotorDriver 側のみ（マルチマスタ構成ではない）。

---

## 4. 機能要件

### F-GEAR-01: bridge 起動確認・疎通（Ready 待ち）

**目的**：bridge は電源投入後に遅延有効化されるため（[command_spec.md §8.2](../multi_i2c_bridge/docs/command_spec.md)）、ready を待ってから通常運用に入る。

**仕様**：
- ESP32-S3 の `app_main()` 起動シーケンス（[CLAUDE.md](CLAUDE.md) の完全起動シーケンス）と並行して、Gear Monitor 初期化を実施する。DRV8825 起動シーケンスをブロックしない（非同期・別タスク）。
- `WHO_AM_I`（offset 0x00）を **10ms 間隔**でポーリングし、`0xB6` が返るまで待つ。NACK は「未 ready」として継続する（異常扱いしない）。
- タイムアウト：**200ms**。超過した場合は bridge 未接続／異常として扱い、`GEAR_STATE = UNAVAILABLE` に遷移する（F-GEAR-04）。以降の角度取得はスキップし、`GET GEAR_ANGLE` には `ERR E013 GEAR_UNAVAILABLE` を返す。
- Ready 確認後、以下を順に実施する：
  1. `VERSION`（0x01）読み出し・major 検証（**期待 major=1**、6ch/v1.0 レジスタマップ前提。不一致は致命として `GEAR_STATE = UNAVAILABLE` とし `ERR E014 GEAR_VERSION_MISMATCH` をログに記録）
  2. `CH_PRESENT`（0x46）読み出し。CH0〜CH2 の bit が全て 1 であることを期待値として確認し、不一致（未接続 ch あり）はログに警告を出す（致命エラーにはしない。§4.4 の DEGRADED 判定に委ねる）
  3. `CONFIG`（0x40）へ `0x00`（RAW_ANGLE 明示選択）を書込（防御的実装、[controller_impl_notes.md §1](../multi_i2c_bridge/docs/controller_impl_notes.md) 推奨事項）
  4. `GEAR_STATE = READY` に遷移し、定期取得（F-GEAR-02）を開始する

**再接続・再試行**：
- 一度 `UNAVAILABLE` になった場合、以降は **30 秒間隔**でバックグラウンド再探索（同じ ready ポーリング手順）を行う。復帰を検出したら `READY` へ復帰する。

### F-GEAR-02: 角度定期取得

**周期**：**10ms（100Hz）**。既存 `ADCTask`（10ms 周期）と同等のカデンスとし、専用タスク `GearMonitorTask` として実装する（§7）。

**取得方式**：
- bridge の**15バイト一括読み**（offset 0x10〜0x1E、[command_spec.md §3.1/§8.1](../multi_i2c_bridge/docs/command_spec.md)）を用いる。1トランザクションで CH0〜CH5 角度 + `STATUS_LO_M` + `STATUS_HI_M` + `SAMPLE_LO` を一貫スナップショットとして取得できるため、角度と健全性フラグの整合が保証される。
- 使用するのは CH0〜CH2（offset 0x10〜0x15）の角度と、`STATUS_LO_M`（bit0〜2）のみ。CH3〜CH5（offset 0x16〜0x1B）は読み捨てる。
- **新旧判別**：`SAMPLE_LO`（offset 0x1E）を前回値と比較する。差分 0（未更新）の場合も異常ではない（[i2c_architecture.md §5.5.3](../multi_i2c_bridge/docs/i2c_architecture.md) のとおり bridge 側巡回周期は 3ch 有効時で概算 0.55〜0.6ms のため、10ms 周期読みでは通常は毎回更新済みになる想定）。

**データ変換**（[command_spec.md §5](../multi_i2c_bridge/docs/command_spec.md) 準拠）：
```c
uint16_t raw = (buf[2*ch+1] << 8 | buf[2*ch]) & 0x0FFF;   // 12bit, LE
if (raw == 0x0FFF && buf[2*ch] == 0xFF && buf[2*ch+1] == 0xFF)  // 無効ch判定（§5.4）
    → 無効値として扱う
float gear_deg_raw = raw * 360.0f / 4096.0f;               // 0-360°, センサ生角度
float gear_deg = normalize(gear_deg_raw * gear_dir_sign[axis] - gear_angle_offset[axis]); // F-GEAR-08
```

### F-GEAR-03: 角度スケーリング・オフセット・方向正規化

- センサ生角度（0〜360°、AS5600 RAW_ANGLE）に対し、軸ごとの **オフセット**（`gear_angle_offset[axis]`, NVS 保存, F-GEAR-08）と **方向反転**（`gear_dir_sign[axis]` = ±1）を適用し、機械的な基準角度系に正規化した `gear_angle_deg[axis]` を算出する。
- **ギア比換算**：既存 NVS パラメータ `gear_ratio`（[F-MOT-10](firmware/REQUIREMENTS.md)）を用い、モーター軸換算角度も参考値として算出可能とする：
  `motor_equiv_deg = gear_angle_deg × gear_ratio`
  （テレメトリ・ログ用途。制御には使用しない、§1.3）
- AS5600 の方向（DIR ピン, bridge 側 `DIR_CONFIG` レジスタ, [as5600_config.md §5](../multi_i2c_bridge/docs/as5600_config.md)）は bridge 既定値（全ch CW増加）のまま変更しない。方向補正は SteppingMotorDriver 側の `gear_dir_sign` ソフトウェア係数で行う（bridge 側 `DIR_CONFIG` への書込は本版では行わない — 複数プロジェクトで bridge を共用する可能性を考慮し、bridge 側設定への書込は最小限にする）。

### F-GEAR-04: 健全性監視

| 状態 | 判定条件 | ホストへの通知 |
|------|---------|--------------|
| `OK` | `STATUS_LO_M` の対象 ch bit = 1 | 通常。角度値は有効 |
| `DEGRADED`（該当軸） | `STATUS_LO_M` の対象 ch bit = 0（bridge 側 `CHn_OK=0`：未接続 or 故障） | `EVT GEAR_DEGRADED <axis>` を1回送信（エッジ検出、連続送信しない）。角度値は無効（前回有効値保持 or `NaN` 扱い、ホスト側は `GET GEAR_STATUS` で判別） |
| `UNAVAILABLE`（全体） | F-GEAR-01 の ready 待ちタイムアウト、または通信連続失敗（後述） | `EVT GEAR_UNAVAILABLE` を送信 |

**通信異常時のリトライ方針**：
- 1回の I2C トランザクション失敗（NACK/タイムアウト）は即座に致命扱いしない。**連続 5 回失敗**（= 50ms 相当、10ms 周期換算）で該当軸を `DEGRADED` とする。
- **連続 20 回失敗**（= 200ms 相当）で bridge 自体を `UNAVAILABLE` とし、F-GEAR-01 の再探索フェーズに戻る。
- `FAULT`（0x04）/`CH_FAULT`（0x05）レジスタは低頻度（1Hz 程度）で別途読み出し、ログに記録する（`STATUS_DECIM` 相当の間引き。bridge 側は自動で ~100Hz 更新のため、ホスト側は 1Hz で十分）。異常ラッチを検出した場合、`CMD=CLEAR_FAULT` を発行してクリアする（診断目的、自動復旧の判断材料）。

### F-GEAR-05: 電源投入時の絶対位置認識

**目的**：AS5600 は単回転絶対角度センサである点を活かし、電源投入直後（ホーミング前）にギアボックス出力軸の絶対角度を取得し、以下に利用する：
1. ログ記録（前回電源断からの位置変化の有無を人が確認できるようにする）
2. ホーミング完了後の整合性チェック（ホーミングで得た `home_offset_steps` 基準の位置と、AS5600 絶対角度から算出した期待角度の差分をログに出し、大きく乖離していれば警告する）

**⚠ 重要な制約（多回転曖昧性）**：
AS5600 は **1回転（0〜360°）のみ**を表す絶対角度センサであり、複数回転を積算するカウンタを持たない。
`gear_ratio > 1`（減速）かつモーター側の可動範囲が出力軸換算で 360° を超える場合、AS5600 単体では
「今何回転目か」を一意に決定できない。したがって：

- 出力軸の可動範囲が **1回転（360°）未満**に収まる軸（例：関節可動域が機構的に制限されている場合）のみ、
  AS5600 絶対角度から**一意に絶対位置を復元**できる。この場合、軸ごとの NVS フラグ
  `gear_absolute_position_capable[axis]`（bool）を立てることで、起動時に AS5600 角度から
  `step_pos` の初期値を直接算出する運用を許可する（F-MOT-09 のホーミング動作を代替 or 省略可能にする、
  ただし本版では**ログ記録・整合性チェックのみ実装**し、`step_pos` への自動反映は将来拡張とする §4.9）。
- 出力軸の可動範囲が 360° を超えうる軸は、AS5600 角度を絶対位置には使用せず、
  **ホーミング結果の妥当性検証（乖離検出）**にのみ用いる。

**実装内容（本版）**：
- 起動後 `GEAR_STATE = READY` 到達時点で、CH0〜CH2 の角度を1回取得し、`boot_gear_angle_deg[axis]` として保持する（`GET GEAR_ANGLE` で参照可能）。
- `HOME_DONE` イベント発生時（F-MOT-09）、その時点の `gear_angle_deg[axis]` を `home_gear_angle_deg[axis]` として記録する。以後、モーター位置（`step_pos / steps_per_rev / gear_ratio × 360`）と `gear_angle_deg[axis]` の差分を算出し、`gear_absolute_position_capable[axis]=true` かつ差分が閾値（NVS `gear_deviation_warn_deg`、デフォルト 5.0°）を超えた場合に `EVT GEAR_DEVIATION_WARN <axis> <deg>` を送信する。

### F-GEAR-06: コマンドインタフェース拡張

USB-CDC 経由の既存コマンドセット（[REQUIREMENTS.md §4](firmware/REQUIREMENTS.md)）に以下を追加する。

**状態取得**

| コマンド | 引数 | 説明 | 応答例 |
|---------|------|------|--------|
| `GET GEAR_ANGLE <axis>` | axis: 0〜2 | ギアアウトプット軸角度取得（正規化後, deg） | `OK 123.45` |
| `GET GEAR_RAW <axis>` | axis: 0〜2 | センサ生角度取得（デバッグ用, 0〜360°, オフセット・方向補正前） | `OK 118.20` |
| `GET GEAR_STATUS` | - | 全軸の健全性サマリー（JSON） | `OK {"bridge":"READY","axes":[{"ok":true,"deg":123.45},...]}` |
| `GET GEAR_DEVIATION <axis>` | axis: 0〜2 | ホーミング時記録角度との差分（F-GEAR-05） | `OK 1.20` |

**設定**

| コマンド | 引数 | 説明 |
|---------|------|------|
| `SET GEAR_OFFSET <axis> <deg>` | deg: float | 角度オフセット設定（NVS 保存） |
| `SET GEAR_DIR <axis> <+1\|-1>` | - | 方向反転係数設定（NVS 保存） |
| `SET GEAR_ABS_CAPABLE <axis> <0\|1>` | - | 単回転絶対位置復元可否フラグ設定（NVS 保存, F-GEAR-05） |
| `SET GEAR_DEVIATION_WARN <deg>` | deg: float | 乖離警告閾値設定（NVS 保存, デフォルト 5.0） |

**非同期イベント（[REQUIREMENTS.md §4.5](firmware/REQUIREMENTS.md) に追加）**

| イベント | データ | 説明 |
|---------|--------|------|
| `EVT GEAR_DEGRADED <axis>` | - | 該当軸のセンサ異常検出（F-GEAR-04） |
| `EVT GEAR_RECOVERED <axis>` | - | 該当軸のセンサ異常から復帰 |
| `EVT GEAR_UNAVAILABLE` | - | bridge 自体が応答不能（F-GEAR-01/04） |
| `EVT GEAR_AVAILABLE` | - | bridge 復帰 |
| `EVT GEAR_DEVIATION_WARN <axis> <deg>` | deg: 乖離角度 | ホーミング位置との乖離が閾値超過（F-GEAR-05） |

**エラーコード追加（[REQUIREMENTS.md §4.6](firmware/REQUIREMENTS.md) に追加）**

| コード | 説明 |
|--------|------|
| `E013` | `GEAR_UNAVAILABLE`（bridge 未接続または応答不能） |
| `E014` | `GEAR_VERSION_MISMATCH`（bridge の `VERSION` major 不一致） |

### F-GEAR-07: エラー処理・堅牢性

| 事象 | 検出 | 対処 |
|------|------|------|
| bridge 未接続／未応答（起動時） | F-GEAR-01 のタイムアウト | `GEAR_STATE=UNAVAILABLE`、30秒毎に再探索、モーター制御系には一切影響させない（本機能は補助系） |
| bridge との通信喪失（運用中） | 連続 I2C 失敗（§4.4） | 軸単位 `DEGRADED` → 全体 `UNAVAILABLE` の2段階。モーター制御は継続（本機能故障で運転停止しない） |
| bridge `VERSION` major 不一致 | 起動時チェック | `UNAVAILABLE` 固定（自動リトライで解消しないため、再探索しても同じ結果になるが、bridge FW 更新後の再起動で復帰する想定） |
| 上流 I2C バス固着 | ESP-IDF I2C ドライバのタイムアウト | ESP-IDF I2C マスタドライバのバス復旧機能（`i2c_master_bus` の再初期化）を利用。**本バスの復旧は SteppingMotorDriver（Master）側の責務**（[controller_impl_notes.md §5](../multi_i2c_bridge/docs/controller_impl_notes.md)） |
| CH_ENABLE の不整合（未接続chを期待） | `CH_PRESENT` 読み出し確認（F-GEAR-01） | ログ警告のみ。`CH_ENABLE` への書込（明示宣言）は本版では行わない（bridge 既定の自動検出に委ねる） |

**重要な設計方針**：本機能の異常（bridge 未接続・通信断）は、既存のモーター制御・安全機構
（F-MOT-07 ESTOP、F-MOT-08 脱調検出、F-COM-04 通信ウォッチドッグ等）に**一切影響を与えない**。
ギアアウトプット角度モニタはあくまで補助的なテレメトリ系として、独立して失敗できる設計とする。

### F-GEAR-08: NVS 設定パラメータ

`config.c`（[REQUIREMENTS.md §6](firmware/REQUIREMENTS.md) ソフトウェアアーキテクチャ参照）に
新規 NVS 名前空間 `gear_config` を追加し、軸ごとに以下を保存する：

| パラメータ名 | 型 | デフォルト | 説明 |
|------------|-----|-----------|------|
| `gear_angle_offset` | float | 0.0 | 角度オフセット（deg） |
| `gear_dir_sign` | int8 | 1 | 方向反転係数（+1 or -1） |
| `gear_abs_capable` | bool | false | 単回転絶対位置復元可否（F-GEAR-05） |
| `gear_deviation_warn_deg` | float | 5.0 | 乖離警告閾値（deg, 全軸共通） |
| `gear_enable` | bool | true | 軸ごとの本機能有効/無効（false でポーリング対象から除外） |

### F-GEAR-09: 将来拡張のための設計制約（本版では実装しない）

クローズドループ位置制御の外側ループへの拡張を将来行う場合に備え、本版の実装は以下を満たす：

- `GearMonitorTask` が保持する `gear_angle_deg[axis]` は、他タスク（`MotorControlTask` 等）から
  読み取り可能な共有変数として設計する（mutex or アトミック読み出しで整合性を保つ。書き込みは
  `GearMonitorTask` のみ）。ただし本版では `MotorControlTask` から参照しない。
- レジスタ読み出し周期（10ms）はモーション制御周期（1ms, F-MOT-04）より粗いため、将来クローズド
  ループに使う場合は周期短縮（最短 1ms、bridge 側 3ch 巡回律速 ~0.55-0.6ms、[i2c_architecture.md §5.5.3](../multi_i2c_bridge/docs/i2c_architecture.md)）が必要になる点を設計メモとして残す。

---

## 5. データフロー・bridge レジスタ対応表

本機能が使用する bridge レジスタ（[command_spec.md §3](../multi_i2c_bridge/docs/command_spec.md) の抜粋）：

| Offset | 名称 | 用途 | 読み出し頻度 |
|--------|------|------|-------------|
| 0x00 | `WHO_AM_I` | 起動 ready 確認（F-GEAR-01） | 起動時のみ（10ms間隔ポーリング） |
| 0x01 | `VERSION` | major 互換チェック（F-GEAR-01） | 起動時のみ |
| 0x40 | `CONFIG` | RAW_ANGLE 明示選択（防御的書込） | 起動時のみ |
| 0x46 | `CH_PRESENT` | 接続確認ログ（F-GEAR-01） | 起動時のみ |
| 0x10〜0x1E | 角度+STATUSミラー+SAMPLE_LO（15バイト一括） | 通常運用の主読み出し（F-GEAR-02） | **10ms 周期** |
| 0x04, 0x05 | `FAULT` / `CH_FAULT` | 診断ログ | 1Hz |
| 0x50 | `CMD` | `CLEAR_FAULT` 発行（診断時のみ） | 必要時のみ |

---

## 6. タスク設計

### 6.1 新規タスク `GearMonitorTask`

| 項目 | 値 |
|------|-----|
| 優先度 | 15（`ADCTask` と同等） |
| スタックサイズ | 3072 B（JSON 応答生成・浮動小数点演算のため `ADCTask` よりやや多め） |
| 周期 | 10 ms |
| 実装ファイル | `main/gear_monitor.c` / `main/gear_monitor.h`（新規） |

既存タスク構成（[REQUIREMENTS.md §6](firmware/REQUIREMENTS.md)）への追加：

| タスク名 | 優先度 | スタック | 周期 | 説明 |
|---------|--------|---------|------|------|
| EncoderTask | 21（最高） | 2048 B | 割り込み | （既存）PCNT カウンタ読み取り・Z 相処理 |
| MotorControlTask | 20（高） | 4096 B | 1 ms | （既存）速度プロファイル計算・RMT 周波数更新・脱調検出 |
| **GearMonitorTask** | **15（中高）** | **3072 B** | **10 ms** | **（新規）bridge 経由ギア出力角度取得・健全性監視** |
| ADCTask | 15（中高） | 2048 B | 10 ms | （既存）ADC サンプリング・電流換算・過電流判定 |
| CommTask | 10（中） | 4096 B | 常時待機 | （既存）コマンド受信・パース・応答送信 |
| StatusTask | 5（低） | 2048 B | 100 ms | （既存）ハートビート・ログ出力 |

### 6.2 内部状態機械

```
        起動
         │
         ▼
    [INIT] ──WHO_AM_I ready 待ち(10ms間隔,timeout 200ms)──► タイムアウト ──► [UNAVAILABLE]
         │ 成功                                                                  │
         ▼                                                                       │
   [VERSION/CH_PRESENT 確認]                                                     │
         │ major不一致 ─────────────────────────────────────────────────────────►│
         │ OK                                                                    │
         ▼                                                                       │
      [READY] ◄────────────────── 30秒毎の再探索成功 ─────────────────────────────┘
         │  10ms周期で15バイト一括読み
         │
         ├─ 個別ch連続失敗5回 ──► 該当軸 DEGRADED（他軸は READY 継続）
         │                              │ 復帰
         │                              ▼
         │                          READY へ復帰（EVT GEAR_RECOVERED）
         │
         └─ 連続失敗20回（全体） ──► [UNAVAILABLE]（EVT GEAR_UNAVAILABLE）
```

---

## 7. 実装仕様（ESP-IDF）

### 7.1 I2C マスタドライバ初期化

ESP-IDF v5.x 新 I2C マスタドライバ（`driver/i2c_master.h`）を使用する。

```c
// gear_monitor.c 抜粋（初期化イメージ）
#define GEAR_I2C_PORT        I2C_NUM_0
#define GEAR_BRIDGE_ADDR     0x42
#define GEAR_I2C_FREQ_HZ     400000
#define GEAR_READY_TIMEOUT_MS  200
#define GEAR_READY_POLL_MS     10

i2c_master_bus_config_t bus_cfg = {
    .i2c_port = GEAR_I2C_PORT,
    .sda_io_num = GPIO_I2C_SDA,     // 38
    .scl_io_num = GPIO_I2C_SCL,     // 39
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,  // §3.1: 二重実装回避。回路図確認結果に応じて設定
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_cfg, &bus_handle);

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = GEAR_BRIDGE_ADDR,
    .scl_speed_hz = GEAR_I2C_FREQ_HZ,
};
i2c_master_dev_handle_t bridge_handle;
i2c_master_bus_add_device(bus_handle, &dev_cfg, &bridge_handle);
```

### 7.2 レジスタ read/write ヘルパー

```c
// コンバインド読み（ptr書込→リピートスタート→n バイト読み, command_spec.md §2.2）
esp_err_t bridge_read_reg(i2c_master_dev_handle_t h, uint8_t reg, uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(h, &reg, 1, buf, n, /*timeout_ms=*/50);
}

esp_err_t bridge_write_reg(i2c_master_dev_handle_t h, uint8_t reg, const uint8_t *data, size_t n) {
    uint8_t frame[1 + 16];
    frame[0] = reg;
    memcpy(&frame[1], data, n);
    return i2c_master_transmit(h, frame, 1 + n, /*timeout_ms=*/50);
}
```

- タイムアウト 50ms は 10ms 周期タスクに対し保守的すぎるため、実装時に見直す（暫定値。§8 未確定事項）。
- クロックストレッチは ESP-IDF I2C マスタドライバの既定挙動（無効化しない）。

### 7.3 起動 ready 待ち（F-GEAR-01）

```c
static bool gear_wait_ready(i2c_master_dev_handle_t h) {
    int64_t deadline_us = esp_timer_get_time() + GEAR_READY_TIMEOUT_MS * 1000;
    uint8_t who_am_i;
    while (esp_timer_get_time() < deadline_us) {
        if (bridge_read_reg(h, 0x00, &who_am_i, 1) == ESP_OK && who_am_i == 0xB6) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(GEAR_READY_POLL_MS));
    }
    return false;
}
```

### 7.4 定期読み出し（F-GEAR-02, 10ms タスクループ）

```c
void gear_monitor_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (gear_state == GEAR_STATE_READY) {
            uint8_t buf[15];
            esp_err_t err = bridge_read_reg(bridge_handle, 0x10, buf, sizeof(buf));
            if (err == ESP_OK) {
                uint8_t status_lo = buf[12];
                uint8_t sample_lo = buf[14];
                for (int ch = 0; ch < NUM_AXES; ch++) {
                    bool ok = (status_lo >> ch) & 0x01;
                    uint16_t raw = (buf[2*ch] | (buf[2*ch+1] << 8)) & 0x0FFF;
                    gear_update_channel(ch, ok, raw);
                }
                gear_fail_count = 0;
            } else {
                gear_handle_comm_failure();   // 連続失敗カウント・DEGRADED/UNAVAILABLE 遷移（§4.4）
            }
        } else if (gear_state == GEAR_STATE_UNAVAILABLE) {
            gear_try_reacquire();   // 30秒毎に §7.3 を再実行
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
```

### 7.5 角度換算（F-GEAR-03）

```c
static float gear_raw_to_deg(uint16_t raw, uint8_t axis) {
    float deg = raw * 360.0f / 4096.0f;
    deg = deg * gear_dir_sign[axis] - gear_angle_offset[axis];
    // 0-360°に正規化
    deg = fmodf(deg, 360.0f);
    if (deg < 0) deg += 360.0f;
    return deg;
}
```

### 7.6 NVS パラメータテーブル追加分

`config.c` に `gear_config` 名前空間として以下のキーを追加する（既存 `motor_config` 名前空間とは分離し、独立して `RESET_CONFIG` 可能にする）：

```c
// NVS keys（軸ごとに suffix "0"/"1"/"2" を付与、例: "gear_off_0"）
"gear_off_%u"      // float, gear_angle_offset
"gear_dir_%u"      // int8,  gear_dir_sign
"gear_abscap_%u"   // uint8, gear_abs_capable
"gear_en_%u"       // uint8, gear_enable
"gear_devwarn"     // float, gear_deviation_warn_deg（全軸共通）
```

---

## 8. 非機能要件

| 項目 | 要件 |
|------|------|
| モーター制御への影響 | `GearMonitorTask`（優先度15）は `MotorControlTask`（優先度20）・`EncoderTask`（優先度21）より低優先度とし、モーション制御のリアルタイム性に影響を与えない |
| I2C バス占有時間 | 15バイト読み1回あたり約 400µs（400kHz 換算, [command_spec.md §3.1](../multi_i2c_bridge/docs/command_spec.md)）。10ms 周期に対し 4% 未満で十分な余裕 |
| 起動時間への影響 | Gear Monitor 初期化は非同期タスクとし、DRV8825 起動シーケンス（F-MOT-03）・USB-CDC 確立をブロックしない。bridge 未接続でもファームウェア全体の起動は正常完了する |
| 耐障害性 | bridge 未接続・通信断・バージョン不一致のいずれも、モーター制御・安全機構（ESTOP・過電流保護・通信ウォッチドッグ）を停止させない（§4.7） |

---

## 9. 開発ロードマップ（Phase 6 案）

既存ロードマップ（[REQUIREMENTS.md §7](firmware/REQUIREMENTS.md)）に対する追加 Phase として位置付ける。

### Phase 6：ギアアウトプット角度モニタ
- [ ] `gear_monitor.c/h` 新規実装（ESP-IDF I2C マスタドライバ, F-GEAR-01/02）
- [ ] 角度変換・オフセット・方向正規化（F-GEAR-03）
- [ ] 健全性監視・DEGRADED/UNAVAILABLE 状態機械（F-GEAR-04）
- [ ] 電源投入時絶対位置認識・ホーミング乖離検出（F-GEAR-05）
- [ ] コマンドセット拡張（`GET GEAR_*` / `SET GEAR_*`, F-GEAR-06）
- [ ] NVS `gear_config` 名前空間実装（F-GEAR-08）
- [ ] 実機疎通確認（bridge 実機との 15バイト一括リード, GPIO38/39 配線確認）
- [ ] 回路図上の I2C プルアップ実装箇所の確認（bridge 側 or SteppingMotorDriver 側、二重実装でないこと）

---

## 10. 未確定事項（実装前に要確認）

| # | 項目 | 優先度 |
|---|------|--------|
| 1 | I2C プルアップの実装箇所（本基板の回路図確認。SteppingMotorDriver 基板と multi_i2c_bridge 基板のどちらが担うか、二重実装でないか） | High |
| 2 | `bridge_read_reg`/`bridge_write_reg` のタイムアウト値（暫定 50ms。10ms 周期タスクとの整合を実機検証で調整） | Medium |
| 3 | 軸ごとの `gear_abs_capable`（単回転絶対位置復元可否）の初期値・機構的な可動域確認（各ギアボックス出力軸の実際の可動範囲がジョイントごとに 360° 未満か要確認） | Medium |
| 4 | multi_i2c_bridge 側の `VERSION` 実機値確認（本書は major=1 / 6ch v1.0 を前提とするが、接続する実機 bridge の FW バージョンとの整合を要確認） | High |
| 5 | F-GEAR-09 の将来クローズドループ拡張時の周期短縮（10ms→1ms 等）の要否判断 | Low（将来） |
