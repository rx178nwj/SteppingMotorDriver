# Codex 実装プロンプト — SteppingMotorDriver firmware WiFiテレメトリ Phase 4

## 役割
あなたは ESP-IDF（ESP32-S3）ファームウェアの実装を担当するエンジニアです。
SteppingMotorDriver ファームウェアの無線テレメトリ機能 **Phase 4**（WiFi TCPテレメトリサーバ・mDNS）を、
本リポジトリ内の要件定義書に厳密に従って実装してください。

**前提条件：Phase 3（WiFi Station接続・USB-CDC経由プロビジョニング）が完了していること。**
未完了の場合は着手せず、その旨を報告してください。

## 参照ドキュメント（正）
- `SteppingMotorDriver/firmware/BLE_WIFI_REQUIREMENTS.md` §5.3（テレメトリプロトコル）・
  §5.5（ディスカバリ／mDNS）・§7.1（安全性：書き込みコマンド非受理の徹底）
- Phase 1成果物（`ble_gatt.c`のJSON共通シリアライズ関数、§8「3経路とも同一の内部テレメトリ状態・
  JSONシリアライズ処理を共用」方針を踏襲すること）
- Phase 3成果物（`wifi_task.c`、Station接続確立後にTCPサーバを起動する）

## 今回のスコープ（Phase 4 のみ）
Phase 5（統合テスト）は着手しないこと。

Phase 4 の内容：
1. TCPテレメトリサーバ（改行区切りJSON、既定ポート4000）
2. `SET WIFI_TELEMETRY_RATE`の実配信ロジック実装
3. mDNSサービス広告

## Phase 4 実装要件（詳細）

### テレメトリプロトコル（`BLE_WIFI_REQUIREMENTS.md` §5.3）
- トランスポート：生TCPソケット（ESP-IDF lwIPスタック上にTCPサーバ、ブローカー非依存）
- ポート：既定4000/TCP固定（本版では設定コマンド化しない）
- フレーム形式：改行区切りJSONテキスト（`\n`終端）。BLEキャラクタリスティック（Phase 1）と同一のJSON
  構造に`"type"`フィールドを付与して多重化する：

```
{"type":"axis_status","data":[{"axis":0,"state":"IDLE","pos":12800,"vel":0,"enc":51200},...]}\n
{"type":"power","data":{"pot":[...],"current_mA":850,"voltage_mV":24100}}\n
{"type":"fault","data":{"reason":"NONE","axis_mask":0,"timestamp_us":0}}\n
```

- **JSON生成はPhase 1で切り出した共通シリアライズ関数を再利用する**こと。BLE用・WiFi用でJSON構築
  ロジックを重複実装しないこと（`type`フィールドの付与のみラッパーで行う）。
- 配信レート：デフォルト10Hz（BLEと同一）。`SET WIFI_TELEMETRY_RATE <hz>`（範囲1〜100Hz、Phase 3で
  コマンド受理・NVS保存済み想定）で変更可能にする。WiFi経由の設定コマンドは提供しない
  （読み取り専用方針の徹底）。
- 同時接続数：1（複数クライアント接続要求は2本目以降を拒否する）。
- **書き込み方向の扱い**：サーバは接続後、クライアントからの受信データを一切コマンドとして解釈しない
  （受信バイトは破棄する、または一定サイズ超過で接続を切断する）。これは§1.2 #4・§7.1の恒久方針を
  プロトコルレベルで担保する重要な安全要件であり、絶対に妥協しないこと。

### ディスカバリ（§5.5）
- mDNS（`_smd-telemetry._tcp.local.`）でサービスを広告し、TXTレコードに`board_id=<12桁16進>`を含める。
- ESP-IDFの`mdns`コンポーネント（`managed_components/espressif__mdns`が既にリポジトリに存在することを
  確認し、利用する）を使う。

### 非機能要件（Phase 4 範囲）
- `WifiTelemetryTask`（優先度8）は既存タスクより低優先度を維持する。
- TCPクライアント切断・再接続がファームウェアの他機能に影響しないこと。
- 既存の統合テスト（`firmware_test/`）が回帰なくPASSすること。

## 成果物
- `wifi_task.c`の拡張（TCPサーバ、mDNS広告）
- Phase 1のJSON共通シリアライズ関数への`type`ラッパー追加

## 完了条件（Definition of Done）
- ビルドが通り、既存の統合テストが回帰なくPASSする
- WiFi接続確立後、`nc`やNode.jsの`net.Socket`等でTCP:4000へ接続し、改行区切りJSONが指定レートで
  受信できることを実機で確認する（未確認の場合はその旨を報告）
- mDNSで`_smd-telemetry._tcp.local.`が発見でき、TXTレコードに`board_id`が含まれることを確認する
- TCPクライアントからの送信データがコマンドとして解釈されないことを確認する（安全要件、必ず検証する）
- Phase 5（統合テスト・coexistence評価）は実装しない
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
