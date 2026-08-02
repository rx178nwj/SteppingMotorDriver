# Codex 実装プロンプト — SteppingMotorDriver firmware BLEテレメトリ Phase 2

## 役割
あなたは ESP-IDF（ESP32-S3）ファームウェアの実装を担当するエンジニアです。
SteppingMotorDriver ファームウェアのBLEテレメトリ機能 **Phase 2**（ペアリング・ボンディング実装、
`GET/SET BLE_ENABLE`コマンド追加）を、本リポジトリ内の要件定義書に厳密に従って実装してください。

**前提条件：Phase 1（NimBLE導入・GATTサービス実装）が完了していること。** 未完了の場合は着手せず、
その旨を報告してください。

## 参照ドキュメント（正）
- `SteppingMotorDriver/firmware/BLE_WIFI_REQUIREMENTS.md` §4.4（ペアリング・セキュリティ）・
  §6.1〜6.3（コマンドセット拡張：`GET BLE_STATUS`/`SET BLE_ENABLE`/エラーコード）
- `SteppingMotorDriver/firmware/REQUIREMENTS.md` §4.1（コマンドフォーマット）・§4.6（既存エラーコード
  一覧、`E015`/`E016`追加時の採番衝突確認）
- Phase 1成果物（`ble_gatt.c`/`ble_task.c`）

## 今回のスコープ（Phase 2 のみ）
Phase 3〜4（WiFi）、Phase 5（統合テスト）は着手しないこと。

Phase 2 の内容：
1. LE Secure Connections + Just Works、ボンディング有効化
2. `GET BLE_STATUS`・`SET BLE_ENABLE <0|1>` コマンド追加（USB-CDC経由）
3. `E016 BLE_INIT_FAILED`エラーコードの正式追加

## Phase 2 実装要件（詳細）

### ペアリング・セキュリティ（`BLE_WIFI_REQUIREMENTS.md` §4.4）
| 項目 | 実装内容 |
|------|---------|
| ペアリング方式 | Just Works（MITM保護なし、`ble_gap_security_ie`で`MITM=0`） |
| 暗号化 | 接続確立後は必ず暗号化リンクを要求する（`SC=1`、平文でのテレメトリ送信を避ける） |
| ボンディング | 有効化し、NVSにボンド情報を保存する（再接続時にペアリングをやり直さない） |

- 未ボンドのCentralからの接続要求はペアリング手続きへ進む。ボンド済みCentralは暗号化リンクの
  再確立のみで再接続できることを確認する。
- ボンド情報のクリア手段（工場出荷リセット等）は本Phaseでは新規UIを設けなくてよいが、既存の
  `RESET_CONFIG`（`firmware/REQUIREMENTS.md` F-PARAM相当）実行時にNVSのBLEボンド情報も
  クリアするかどうかを判断し、報告に明記すること（要件書に明記がないため実装者判断とし、
  安全側＝クリアする方向を推奨するが、既存の`RESET_CONFIG`のNVS消去範囲を確認してから決めること）。

### コマンドセット拡張（§6.1〜6.3）
| コマンド | 引数 | 説明 | 応答例 |
|---------|------|------|--------|
| `GET BLE_STATUS` | - | BLE接続状態取得 | `OK ADVERTISING` / `OK CONNECTED` |
| `SET BLE_ENABLE <0\|1>` | - | BLE Advertisingの有効/無効（NVS保存、デフォルト1） |

- `firmware/REQUIREMENTS.md` §4.1のコマンドフォーマット（既存の`GET`/`SET`パーサ）に従い、
  新規コマンドとして追加する。既存コマンドのパース処理を壊さないこと。
- `SET BLE_ENABLE 0`実行時は即座にAdvertising停止・既存接続があれば切断する。`1`実行時は
  Advertisingを再開する。設定はNVSに保存し次回起動時に反映する。
- エラーコード`E016 BLE_INIT_FAILED`を`firmware/REQUIREMENTS.md` §4.6のエラーコード表に正式追加する
  （Phase 1では暫定ログ出力のみだった箇所を、正式なエラーコード経由の報告に置き換える）。

### 非機能要件（Phase 2 範囲）
- ペアリング処理中も`MotorControlTask`/`EncoderTask`のリアルタイム性に影響を与えないこと（§7.4継続）。
- ボンディング情報の保存・読み込み失敗時にBLEタスクがクラッシュしないこと（NVS異常時はログ出力の上
  ペアリング未実施状態にフォールバックする）。
- 既存の統合テスト（`firmware_test/`）が回帰なくPASSすること。

## 成果物
- `ble_gatt.c`/`ble_task.c`の拡張（ペアリング・ボンディング処理）
- `comm.c`または該当コマンドハンドラへの`GET BLE_STATUS`/`SET BLE_ENABLE`追加
- `firmware/REQUIREMENTS.md` §4.6への`E016`追加（ドキュメント更新）
- `firmware/BLE_WIFI_REQUIREMENTS.md`への実装状況追記

## 完了条件（Definition of Done）
- ビルドが通り、既存の統合テストが回帰なくPASSする
- BLE Central（スマホアプリ等）からペアリング要求→暗号化リンク確立→ボンディング→再接続がボンド情報で
  スキップされることを実機で確認する（未確認の場合はその旨を報告）
- `GET BLE_STATUS`/`SET BLE_ENABLE`がUSB-CDC経由で動作する
- Phase 3以降（WiFi、統合テスト）は実装しない
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
