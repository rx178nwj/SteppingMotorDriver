# Codex 実装プロンプト — SteppingMotorDriver firmware 関節角度出力・POTゼロ位置補正（F-MOT-12）

## 役割
あなたは ESP-IDF（ESP32-S3）ファームウェアの実装を担当するエンジニアです。
SteppingMotorDriver ファームウェアに、**関節角度出力（度単位）・POTゼロ位置補正機能（F-MOT-12）**を、
本リポジトリ内の要件定義書に厳密に従って実装してください。

この機能は `robot_arm_monitor` の新規パネル「関節検証パネル（F-RAM-VERIFY）」が、同一関節の
POT角度・エンコーダ角度・ステップ位置角度を並べて表示し、実機組立検証・キャリブレーションに使うための
データソースです。

## 参照ドキュメント（正）
- `SteppingMotorDriver/firmware/REQUIREMENTS.md` §3.1 F-MOT-12（本機能の要件本体、
  角度換算式・POTゼロ位置補正の設計方針の唯一の正）・F-MOT-10（既存NVSパラメータ表、
  `pot_scale_deg`/`pot_zero_offset`を追加済み）・§4.2〜4.4（コマンドセット表、
  `MOVE_DEG`/`MOVETO_DEG`/`GET POS_DEG`/`GET ENC_DEG`/`GET POT_DEG`/`SET POT_SCALE`/
  `SET POT_ZERO`/`CLEAR POT_ZERO`を追加済み）
- `SteppingMotorDriver/firmware/BLE_WIFI_REQUIREMENTS.md` §4.1 GATTサービス構成表（Joint Angle
  キャラクタリスティックを追加済み）
- 既存実装：`comm.c`（コマンドパーサ、既存の`GET POS`/`GET ENC`/`MOVE`/`MOVETO`ハンドラを参考にする）、
  `motor_ctrl.c`（`MOVE`/`MOVETO`の実行経路）、`adc_monitor.c`（ADCフィルタ後の生値取得）、
  `config.c`（NVS読み書き、F-MOT-10パラメータの既存実装パターン）

作業前に上記ファイルを必ず読み込み、内容を実装に反映してください。矛盾する記憶や推測で補完しないこと。

## 重要な前提：BLE Joint Angleキャラクタリスティックの実装は本Phaseの対象外
BLEテレメトリ（NimBLE GATT）は別ロードマップ（`BLE_WIFI_REQUIREMENTS.md` §10）で実装されており、
Joint Angleキャラクタリスティックの追加は**本Phaseでは行わない**（別Phaseで、本Phaseが追加する
`GET POS_DEG`/`GET ENC_DEG`/`GET POT_DEG`の値をJSON配列化してGATTに載せる形になる）。
本Phaseは**USB-CDCコマンドとして正しく動作すること**のみをスコープとする。

## 今回のスコープ
1. `GET POS_DEG <axis>`/`GET ENC_DEG <axis>`（既存NVSパラメータからの角度換算、新規NVS不要）
2. `GET POT_DEG <axis>`（補正なし・ゼロ位置補正ありの2値を返す）
3. `SET POT_SCALE <axis> <deg_per_count>`/`SET POT_ZERO <axis>`/`CLEAR POT_ZERO <axis>`（NVS保存）
4. `MOVE_DEG <axis> <deg>`/`MOVETO_DEG <axis> <deg>`（既存`MOVE`/`MOVETO`と同一実行経路への角度→steps換算）
5. F-MOT-10 NVSテーブルへの`pot_scale_deg`/`pot_zero_offset`追加（ドキュメントは追加済み、コード側の実装）

## 実装要件（詳細）

### 角度換算（`firmware/REQUIREMENTS.md` F-MOT-12）
- `pos_deg = (pos / steps_per_rev) * 360.0 / gear_ratio`
- `enc_deg = (enc / (encoder_ppr * 4)) * 360.0 / gear_ratio`（PCNTは4逓倍のため`encoder_ppr`を4倍してから割る）
- `pot_deg_raw = adc_raw[axis] * pot_scale_deg`（`adc_raw`は`adc_monitor.c`のフィルタ後生カウント、
  POTチャンネル0-2に対応する軸のカウント値を使う）
- `pot_deg_zeroed = (adc_raw[axis] - pot_zero_offset) * pot_scale_deg`
- いずれも`float`または`double`で計算し、応答は小数点以下3桁程度で文字列化する（既存の浮動小数応答が
  ある場合はそのフォーマットに合わせる。既存になければ`%.3f`を採用してよい）

### 新規コマンドハンドラ（`comm.c`）
| コマンド | 引数 | 処理 | 応答 |
|---------|------|------|------|
| `GET POS_DEG <axis>` | axis:0-2 | 上記式で計算 | `OK <deg>` |
| `GET ENC_DEG <axis>` | axis:0-2 | 上記式で計算 | `OK <deg>` |
| `GET POT_DEG <axis>` | axis:0-2 | 上記2式で計算 | `OK <raw_deg> <zeroed_deg>` |
| `SET POT_SCALE <axis> <deg_per_count>` | axis:0-2, float | `pot_scale_deg`をNVS保存 | `OK` |
| `SET POT_ZERO <axis>` | axis:0-2 | 現在の`adc_raw[axis]`を`pot_zero_offset`としてNVS保存 | `OK` |
| `CLEAR POT_ZERO <axis>` | axis:0-2 | `pot_zero_offset`を0にリセットしNVS保存 | `OK` |
| `MOVE_DEG <axis> <deg>` | axis:0-2, float（±） | `steps = round(deg * steps_per_rev * gear_ratio / 360.0)`を計算し、既存`MOVE`と同じ実行関数を呼ぶ | `OK`（完了時`EVT MOVE_DONE <axis>`、既存と同じ） |
| `MOVETO_DEG <axis> <deg>` | axis:0-2, float | 同様に`steps`換算し、既存`MOVETO`と同じ実行関数を呼ぶ | `OK`（完了時`EVT MOVE_DONE <axis>`） |

- `MOVE_DEG`/`MOVETO_DEG`は**既存の`MOVE`/`MOVETO`関数へstepsを渡すラッパー**として実装し、
  同時コマンドポリシー（`E008 MOTION_IN_PROGRESS`等、`REQUIREMENTS.md` §3.1同時コマンドポリシー表）を
  含めモーション実行ロジックを重複実装しないこと。
- 引数のパース失敗・軸番号範囲外は既存コマンドと同様に`E002`/`E003`を返す。
- `GET`系は既存方針どおりモーション中でも常に受付可とする（`REQUIREMENTS.md` §3.1参照）。
- `SET POT_SCALE`/`SET POT_ZERO`/`CLEAR POT_ZERO`はモーション中でも実行を拒否しない
  （キャリブレーション作業は静止中に行う運用だが、ファームウェア側で強制はしない。理由：
  既存の`SET`系コマンドの多くは`E004`でモーション中拒否だが、これらは軸の動作パラメータではなく
  センサ較正パラメータであり、モーション実行に影響しないため）。

### NVS（`config.c`、F-MOT-10）
- `pot_scale_deg`（float、デフォルト0.0879）・`pot_zero_offset`（int32、デフォルト0）を軸ごとに追加する。
- 既存の`SAVE`/`LOAD`/`RESET_CONFIG`コマンドの対象パラメータ集合にこの2つを含める。

### テスト（`firmware_test/`）
- 既存のテストスイート（実機テストランナー、`board_test.c`等）に、以下を確認するテストケースを追加する：
  - `GET POS_DEG`/`GET ENC_DEG`が`steps_per_rev`/`gear_ratio`/`encoder_ppr`の変更に応じて正しく変化する
  - `SET POT_ZERO`実行後、`GET POT_DEG`の`zeroed`側が0近傍になる（`raw`側は変化しない）
  - `CLEAR POT_ZERO`実行後、`zeroed`側が`raw`側と一致する
  - `MOVE_DEG`/`MOVETO_DEG`実行後の`GET POS`（steps）が期待どおりの値になる（角度→steps換算の丸め誤差を許容範囲で確認）
  - `MOVE_DEG`/`MOVETO_DEG`実行中に別の`MOVE`/`MOVETO`/`MOVE_DEG`/`MOVETO_DEG`を送ると`E008`が返る（既存ポリシーの継承確認）
- 既存の統合テストが回帰なくPASSすることを確認する。

## 成果物
- `comm.c`（新規コマンドハンドラ追加）
- `motor_ctrl.c`または該当箇所（角度→steps換算ヘルパー、既存`MOVE`/`MOVETO`実行関数の再利用）
- `adc_monitor.c`（POT角度計算に必要な生値アクセス、既存の取得手段で足りる場合は変更不要）
- `config.c`（`pot_scale_deg`/`pot_zero_offset`のNVS読み書き）
- `firmware_test/`への新規テストケース追加
- `firmware/REQUIREMENTS.md` Phase 6チェックリスト（F-MOT-12）の該当項目を実装済みに更新

## 完了条件（Definition of Done）
- ビルドが通り、既存の統合テストが回帰なくPASSする
- 上記テストケースが実機（またはビルド環境で可能な範囲のシミュレーション）でPASSすることを確認する
  （実機確認ができない場合はその旨を明記する）
- `GET POS_DEG`/`GET ENC_DEG`/`GET POT_DEG`/`SET POT_SCALE`/`SET POT_ZERO`/`CLEAR POT_ZERO`/
  `MOVE_DEG`/`MOVETO_DEG`がUSB-CDC経由で仕様どおりに応答する
- BLE Joint Angleキャラクタリスティックの実装は行わない（別Phase）
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
