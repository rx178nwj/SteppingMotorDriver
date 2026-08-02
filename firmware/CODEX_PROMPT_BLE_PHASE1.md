# Codex 実装プロンプト — SteppingMotorDriver firmware BLEテレメトリ Phase 1

## 役割
あなたは ESP-IDF（ESP32-S3）ファームウェアの実装を担当するエンジニアです。
SteppingMotorDriver ファームウェアに、モニタアプリ（`robot_arm_monitor`）向けの読み取り専用
BLEテレメトリ機能を追加するプロジェクトの **Phase 1**（NimBLE導入・GATTサービス実装）を、
本リポジトリ内の要件定義書に厳密に従って実装してください。

既存のファームウェア（USB-CDCコマンド処理・モーション制御・エンコーダ・ADC、Phase 1〜5実装済み、
統合テスト105/106 PASS）に**手を加えず**、BLEテレメトリ機能を新規タスクとして追加する形にしてください。

## 参照ドキュメント（正）
- `SteppingMotorDriver/firmware/BLE_WIFI_REQUIREMENTS.md` §3（ハードウェアIF）・§4.1（GATTサービス構成）・
  §7.4（リアルタイム性）・§8（ソフトウェアアーキテクチャ）— 本Phaseの要求仕様の唯一の正
- `SteppingMotorDriver/firmware/REQUIREMENTS.md` §4（既存USBコマンドセット、JSON構造の転用元）・
  §6（既存タスク構成・優先度、本Phaseで追加するBleTaskとの優先度関係）
- `SteppingMotorDriver/CLAUDE.md` — ハードウェア仕様・タスク優先度・設計方針の確定事項一覧
- 既存実装：`firmware/main/`配下の`comm.c`（USB JSONシリアライズ処理、GATT側で共用する）

作業前に上記ファイルを必ず読み込み、内容を実装に反映してください。矛盾する記憶や推測で補完しないこと。

## 絶対に守ること（安全制約）
- **BLE経由で書き込み可能なキャラクタリスティックを一切設けない**（CCCDを除く）。
  `BLE_WIFI_REQUIREMENTS.md` §1.2 #4・§4.1・§7.1の恒久方針。将来のPhaseでも例外を作らない。
- BLEテレメトリタスクは既存の`MotorControlTask`（優先度20）・`EncoderTask`（優先度21）より**低い優先度**
  （§7.4・§8：`BleTask`優先度8）で動作させ、モーション制御のリアルタイム性に一切影響を与えないこと。
- BLE初期化・動作異常が既存のモーション制御・USB通信をブロックしないこと（タスク分離を徹底する）。

## 今回のスコープ（Phase 1 のみ）
Phase 2（ペアリング・ボンディング、`GET/SET BLE_ENABLE`）、Phase 3〜4（WiFi）、Phase 5（統合テスト）は
着手しないこと。本Phaseでは**ペアリングなし（暗号化なし）・常時Advertising**の最小構成でGATTサービスが
機能することを目標とする（ペアリングはPhase 2で追加）。

Phase 1 の内容：
1. NimBLEスタック導入（ESP-IDF `bt` コンポーネント、BLE-only構成）
2. GATTサービス実装：Device Info / Axis Status / Power/ADC / Fault Info の4キャラクタリスティック（§4.1）
3. 既存テレメトリ状態（モーション制御・ADCタスクの内部状態）からGATT Notifyへのデータフロー構築
4. `BleTask`の新設・タスク優先度設定

## Phase 1 実装要件（詳細）

### GATTサービス構成（`BLE_WIFI_REQUIREMENTS.md` §4.1）
| キャラクタリスティック | Property | 更新頻度 | ペイロード |
|----------------------|----------|---------|-----------|
| Device Info | Read | 静的（接続時1回） | `{"product":"stepping_motor_driver","board_id":"AABBCCDDEEFF","protocol_version":1,"fw_version":"x.y.z"}` |
| Axis Status | Read, Notify | 10 Hz（100ms） | `[{"axis":0,"state":"IDLE","pos":12800,"vel":0,"enc":51200},...]`（3軸分） |
| Power/ADC | Read, Notify | 10 Hz（100ms） | `{"pot":[v0,v1,v2],"current_mA":850,"voltage_mV":24100}` |
| Fault Info | Read, Notify | 変化時 + 1Hz keepalive | `{"reason":"OVERCURRENT","axis_mask":2,"timestamp_us":1234567890}` |

- Gear Angle キャラクタリスティック（§4.1表の5行目）は**本Phaseでは実装しない**（`GEAR_ANGLE_MONITOR_REQUIREMENTS.md`
  実装後の別Phase。未実装時は`{"state":"UNAVAILABLE"}`を返す固定実装で構わないが、追加作業は最小限にする）。
- 既存の`comm.c`の`GET STATE`/`GET POS`/`GET VEL`/`GET ENC`/`GET ADC`/`GET FAULT_INFO`のJSONシリアライズ
  処理を関数として切り出し、USB応答・BLE Notifyの両方から呼び出せる共通関数にする（§8のデータフロー図の
  「3経路とも同一の内部テレメトリ状態・JSONシリアライズ処理を共用」方針）。新規にBLE専用のJSON生成ロジックを
  重複実装しないこと。
- **ATT MTU**：接続確立後、ATT MTU 185バイト以上への拡張をネゴシエーションする。失敗時はLong Read/
  複数フラグメント送信にフォールバックする（§4.1）。
- **UUID**：プロジェクト専用の128bitベースUUIDを1つ発番し、各キャラクタリスティックはベースUUIDの下位
  バイトをインクリメントして割り当てる（§4.1、§9.1で「実装時に正式発番」とされているため、本Phaseで
  発番して`ble_gatt_uuid.h`等に定義する。他コンポーネントとの衝突確認は目視でよい）。

### Advertising・識別（§4.2、ペアリング以外の部分のみ本Phase対象）
- Device Name：`SMD-<board_id>`（`board_id`は既存`GET BOARD_ID`と同一の12桁16進文字列）
- Advertised Service UUIDに本Phaseで発番したテレメトリサービスUUIDを含める
- Advertising間隔：100〜200ms
- 起動後デフォルトでAdvertising開始（`SET BLE_ENABLE`はPhase 2で実装するため、本Phaseでは常時有効固定でよい）

### 接続管理（§4.3、ペアリング以外）
- Peripheral roleのみ、同時接続Central数は1
- Connection Interval：30〜50msを要求
- 切断検出後は自動的にAdvertisingを再開する
- BLE切断が既存のモーション制御・USB通信・`EVT COMM_TIMEOUT`ロジックに一切影響しないことを確認する
  （BLEとUSBは完全に独立した経路であることをコードレビューで明示すること）

### タスク構成（§8）
- `BleTask`（優先度8、既存`CommTask`優先度10と同等かそれ以下）を新設し、GATTサーバ・Advertising管理・
  Notify送信を担当させる
- 共有テレメトリ状態へのアクセスはmutex保護する（既存タスクの内部状態を壊さないこと）

### 非機能要件（Phase 1 範囲）
- BLEスタック初期化失敗時は`E016 BLE_INIT_FAILED`相当のログを出力し、モーション制御・USB通信は正常動作を継続する
  （BLE初期化失敗で起動全体を止めないこと）
- 既存の統合テスト（`firmware_test/`）が引き続き全てPASSすること（回帰がないことを確認する）

## 成果物
- NimBLE導入（`sdkconfig`のBluetooth関連設定、`CMakeLists.txt`への依存追加）
- `firmware/main/ble_gatt.c`・`.h`（新規、GATTサービス定義・Notify送信ロジック）
- `firmware/main/ble_task.c`・`.h`（新規、`BleTask`本体）
- `comm.c`のJSONシリアライズ関数の切り出し・共通化（既存USB応答ロジックは変更しないこと、リファクタのみ）
- `SteppingMotorDriver/CLAUDE.md`または`firmware/BLE_WIFI_REQUIREMENTS.md`への実装状況追記

## 完了条件（Definition of Done）
- ビルドが通り、既存の統合テスト（`firmware_test/`）が回帰なくPASSする
- BLEスキャンで`SMD-<board_id>`のAdvertisingが確認できる（実機がある場合。無い場合はコード上の
  到達可能性を確認し、その旨を報告する）
- nRF ConnectやBLE Central実装等で接続し、4キャラクタリスティックのRead/Notifyが取得できることを確認する
  （実機確認できない場合は未確認事項として明記する）
- Phase 2以降（ペアリング、`GET/SET BLE_ENABLE`、WiFi）は実装しない
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
