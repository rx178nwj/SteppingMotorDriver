# Codex 実装プロンプト — SteppingMotorDriver firmware BLE/WiFiテレメトリ Phase 3

## 役割
あなたは ESP-IDF（ESP32-S3）ファームウェアの実装を担当するエンジニアです。
SteppingMotorDriver ファームウェアの無線テレメトリ機能 **Phase 3**（WiFi Station接続・
USB-CDC経由プロビジョニング）を、本リポジトリ内の要件定義書に厳密に従って実装してください。

**前提条件：なし（BLEと独立実装可能、`BLE_WIFI_REQUIREMENTS.md` §10ロードマップの通り
Phase 1・2と並行・別順序で着手してよい）。** ただしBLE機能（Phase 1・2）を壊さないこと。

## 参照ドキュメント（正）
- `SteppingMotorDriver/firmware/BLE_WIFI_REQUIREMENTS.md` §5.2（動作モード）・§5.4（プロビジョニング）・
  §6.2〜6.3（コマンドセット拡張）・§7.2（信頼性、自動再接続）
- `SteppingMotorDriver/firmware/REQUIREMENTS.md` §2.3（ADC、ADC2はWiFiと共有のため未使用の既存確定
  制約。本Phaseの実装がこの制約と矛盾しないことを確認する）

## 今回のスコープ（Phase 3 のみ）
Phase 4（TCPテレメトリサーバ・mDNS）、Phase 5（統合テスト）は着手しないこと。
本PhaseではWiFi接続の確立とプロビジョニングまでを対象とし、**テレメトリ配信（TCPサーバ）は実装しない**。

Phase 3 の内容：
1. WiFi Station mode接続（ESP-IDF `esp_wifi`、SoftAPは実装しない）
2. USB-CDC経由のSSID/パスワード設定コマンド追加
3. 自動再接続（指数バックオフ）

## Phase 3 実装要件（詳細）

### 動作モード（`BLE_WIFI_REQUIREMENTS.md` §5.2）
- Station modeのみ。SoftAP機能は実装しない。
- 起動時デフォルトは**無効**。SSID未設定時に`SET WIFI_ENABLE 1`を受信した場合は
  `ERR E015 WIFI_NOT_CONFIGURED`を返す（BLEがデフォルト有効なのとは対照的な方針、意図しない
  電波送出を避けるため）。

### プロビジョニングコマンド（§5.4、USB-CDC経由のみ）
| コマンド | 引数 | 説明 |
|---------|------|------|
| `SET WIFI_SSID <ssid>` | 文字列（最大32文字） | NVS（`wifi_config`名前空間）に保存 |
| `SET WIFI_PASS <password>` | 文字列（最大64文字） | NVSに保存。`GET WIFI_STATUS`等のいかなる応答にも平文表示しない |
| `SET WIFI_ENABLE <0\|1>` | - | WiFi機能の有効/無効切替。NVS保存、次回起動時に反映（即時接続も試行する） |
| `GET WIFI_STATUS` | - | 応答例：`OK CONNECTED 192.168.1.42 RSSI=-52` / `OK DISCONNECTED` / `OK DISABLED` |

- SSID/パスワード未設定のまま`SET WIFI_ENABLE 1`を受信した場合は`ERR E015 WIFI_NOT_CONFIGURED`を返す。
- パスワードはNVS暗号化パーティション（既存`nvs_flash`の暗号化設定に準拠）に保存する。追加のセキュア
  ストレージは導入しない。
- `SET WIFI_TELEMETRY_RATE`（§5.3）はPhase 4のTCPテレメトリ実装と同時に追加してよい（本Phaseで
  コマンドの受理だけ先行実装し値をNVS保存する分には構わないが、実配信ロジックはPhase 4対象）。

### 信頼性（§7.2）
- WiFi接続断からの自動再接続を実装する（指数バックオフ、上限間隔60秒）。
- WiFiタスクの異常が既存のモーション制御・USB通信・BLEタスクに影響しないこと（タスク優先度は
  `WifiTelemetryTask`優先度8、既存タスクより低優先度、`BLE_WIFI_REQUIREMENTS.md` §8）。

### エラーコード追加
| コード | 説明 |
|--------|------|
| `E015` | `WIFI_NOT_CONFIGURED`（SSID未設定のままWiFi有効化を試みた） |

`firmware/REQUIREMENTS.md` §4.6のエラーコード表に正式追加する（Phase 2で追加した`E016`との採番衝突が
ないことを確認する）。

### 非機能要件（Phase 3 範囲）
- ADC2はWiFiと共有のため未使用という既存確定制約（`CLAUDE.md`）に抵触しないこと（全ADCがADC1に
  集約済みのため通常は問題ないが、WiFi有効化時のADC1計測精度への影響がないか確認する）。
- 既存の統合テスト（`firmware_test/`）が回帰なくPASSすること。

## 成果物
- `firmware/main/wifi_task.c`・`.h`（新規、Station接続・自動再接続ロジック）
- WiFiプロビジョニングコマンドハンドラ追加（`comm.c`または該当箇所）
- `firmware/REQUIREMENTS.md` §4.6への`E015`追加（ドキュメント更新）

## 完了条件（Definition of Done）
- ビルドが通り、既存の統合テストが回帰なくPASSする
- USB-CDC経由で`SET WIFI_SSID`/`SET WIFI_PASS`/`SET WIFI_ENABLE 1`を送信し、実機WiFi環境に接続できる
  ことを確認する（実機確認できない場合はコード上の到達可能性を確認し、その旨を報告する）
- `GET WIFI_STATUS`で接続状態・IPアドレス・RSSIが取得できる
- SSID未設定時の`SET WIFI_ENABLE 1`が`ERR E015`を返すことを確認する
- Phase 4以降（TCPテレメトリサーバ、mDNS、統合テスト）は実装しない
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
