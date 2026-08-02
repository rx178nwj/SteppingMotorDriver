# SteppingMotorDriver Bluetooth(BLE)・WiFi通信 要件定義・要求仕様

SteppingMotorDriver ファームウェアに、モニタアプリ（`robot_arm_monitor`）向けの**読み取り専用テレメトリ経路**として
Bluetooth Low Energy（BLE）を実装し、BLE の帯域では不足する用途向けに WiFi（ステーションモード）による
高帯域テレメトリ経路を追加する。

| 項目 | 内容 |
|------|------|
| 対象 | SteppingMotorDriver ファームウェア（ESP32-S3, `firmware/`） |
| 関連（本体） | [firmware/REQUIREMENTS.md](REQUIREMENTS.md)（USB-CDC 本体要件定義。コマンド・データモデルの元） |
| 関連（外部） | [design/SYSTEM_REQUIREMENTS.md](../../design/SYSTEM_REQUIREMENTS.md) §4（通信アーキテクチャ方針）・§6 #1/#3/#4（本書が解消する未解決事項） |
| 関連（外部） | [robot_arm_monitor/REQUIREMENTS.md](../../robot_arm_monitor/REQUIREMENTS.md)（本書のテレメトリを消費するホスト側アプリ、Phase 6 以降） |
| 版 | 0.2（0位置設定コマンドのスコープ確認を反映） |
| 作成日 | 2026-07-25（初版）／2026-08-01 改訂（§1.3, §4.1: 0位置設定はUSB-CDC限定と確認・明記） |

---

## 1. 概要

### 1.1 目的

現行ファームウェアは USB-CDC のみを通信手段とし、モーション制御コマンドとテレメトリ取得の両方を同一チャネルで扱う。
[design/SYSTEM_REQUIREMENTS.md §4](../../design/SYSTEM_REQUIREMENTS.md) の確定方針により、USB-CDC は**制御アプリが専有**し、
モニタアプリ（`robot_arm_monitor`）は SteppingMotorDriver に対して**無線経由の読み取り専用テレメトリ**のみを持つ。

本書はこの無線テレメトリ経路（BLE・WiFi）を要件化し、以下の未解決事項を解消する：

| 元の未解決事項 | 出典 | 本書での扱い |
|---------------|------|-------------|
| BLEテレメトリサービス未設計（GATTサービス・キャラクタリスティック設計、送信形式未定） | [design/SYSTEM_REQUIREMENTS.md §6 #1](../../design/SYSTEM_REQUIREMENTS.md) | §4.1〜4.3 で解消 |
| Bluetoothデバイス検出・ペアリングのUX未検討 | [同 §6 #3](../../design/SYSTEM_REQUIREMENTS.md) | §4.4 で解消 |
| WiFi追加実装の時期・用途未定 | [同 §6 #4](../../design/SYSTEM_REQUIREMENTS.md) | §5 で解消（用途：高帯域テレメトリ、BLEの代替・追加経路） |

### 1.2 スコープ（利用用途）

| # | 用途 | 本版での扱い |
|---|------|------|
| 1 | BLE：モニタアプリ向け読み取り専用テレメトリ（軸状態・電流電圧・フォルト情報） | 実装する（主目的） |
| 2 | WiFi：BLEでは帯域不足になる高頻度・多軸テレメトリの代替／追加経路 | 実装する（§5） |
| 3 | WiFi：ファームウェア OTA 更新 | 本版では実装しない（別途要件化、§9.4） |
| 4 | BLE/WiFi 経由でのモーション制御コマンド発行 | **恒久的に対象外**（[design/SYSTEM_REQUIREMENTS.md §4](../../design/SYSTEM_REQUIREMENTS.md) の読み取り専用方針、無線への書き込み例外は設けない確定事項） |

### 1.3 非対象（明示的に対象外）

- BLE/WiFi 経由の ENABLE/MOVE/JOG/ESTOP 等の書き込みコマンド（§1.2 #4、恒久対象外）
- BLE/WiFi 経由の `SET GEAR_ZERO`/`GEAR_ZERO_CLEAR`（0位置設定, [GEAR_ANGLE_MONITOR_REQUIREMENTS.md F-GEAR-10](GEAR_ANGLE_MONITOR_REQUIREMENTS.md)）発行。§1.2 #4 の恒久方針どおり書き込みコマンドの一種として扱い、USB-CDC（制御アプリ）限定とする。BLE/WiFi側は較正状態の参照のみ（§4.1 `zero_calibrated`）（2026-08-01 確認・確定）
- WiFi SoftAP モード・Web プロビジョニング UI（§5.4 で USB-CDC 経由のプロビジョニングに確定）
- 制御アプリとのIPC中継（[design/SYSTEM_REQUIREMENTS.md §6 #2](../../design/SYSTEM_REQUIREMENTS.md) は別件、本書の対象外）

---

## 2. システム構成

### 2.1 全体構成

```
                         モニタPC（robot_arm_monitor）
        ┌───────────────────────────────────────────────┐
        │  BLE Central                  WiFi(TCP) Client │
        └──────┬─────────────────────────────┬───────────┘
               │ BLE（読取専用、常時）           │ WiFi（読取専用、任意・追加経路）
               ▼                             ▼
        ┌───────────────────────────────────────────────┐
        │       SteppingMotorDriver (ESP32-S3)            │
        │  BLE: NimBLE GATT Server (Peripheral)           │
        │  WiFi: Station mode, TCP Telemetry Server       │
        │  ※ 同一 2.4GHz 無線を時分割共有（coexistence）  │
        └───────────────────────────────────────────────┘
               ▲
               │ USB-CDC（制御アプリ専有・本書対象外／プロビジョニングのみ本書§5.4で使用）
```

### 2.2 役割分担

| 経路 | ESP32-S3側の役割 | 用途 | 常時/任意 |
|------|------------------|------|-----------|
| BLE | GATT Server（Peripheral） | 標準テレメトリ経路（軸状態・電流電圧・フォルト） | 常時 Advertising（電源投入後デフォルト有効） |
| WiFi | TCP Server（Station mode） | 高帯域テレメトリの代替・追加経路 | 任意（SSID未設定時は無効、§5.4） |

- BLE と WiFi は ESP32-S3 の単一 2.4GHz 無線を共有する（coexistence）。両方同時有効時はレイテンシ・スループットが低下し得る（§7.3）。
- どちらの経路も**読み取り専用**：書き込み可能なキャラクタリスティック／コマンド受付ポートは一切設けない（§1.2 #4）。

---

## 3. ハードウェアインタフェース要件

| 項目 | 内容 |
|------|------|
| BLE スタック | ESP-IDF NimBLE（BLE-only、Bluedroid比で省メモリ。Classic Bluetooth は不要） |
| WiFi | ESP32-S3 内蔵 2.4GHz WiFi（IEEE 802.11 b/g/n、Station mode のみ使用） |
| 専用GPIO | 無線モジュール自体は内蔵RF（ESP32-S3-WROOM-1 モジュール内蔵アンテナ）のため専用GPIOなし。ただし接続状態表示用に GPIO47（Status2 LED）を使用する（§3.1） |
| ADC2 との関係 | 既存確定仕様どおり ADC2 は WiFi と共有のため未使用（全 ADC を ADC1 に集約、[firmware/REQUIREMENTS.md §2.3](REQUIREMENTS.md)）。本書の WiFi 有効化はこの制約と整合する |
| 要確認 | 筐体内でのアンテナ設置・RF性能（金属筐体・他基板との近接配置による減衰）は未検証。実機評価が必要（§9.5） |

### 3.1 Status2 LED（BLE/WiFi接続状態表示）

| 項目 | 内容 |
|------|------|
| GPIO | GPIO47（出力、Status2 LED専用） |
| 用途 | BLE・WiFi無線テレメトリ経路の接続／通信／エラー状態をユーザーに一目で提示する表示灯 |
| 駆動タスク | BleTask・WifiTelemetryTask（§8）が状態を共有変数経由でLED制御タスクへ通知する想定。専用タスクを新設するか既存 StatusTask（[firmware/REQUIREMENTS.md §6](REQUIREMENTS.md)）に統合するかは実装時に決定する |

#### 表示パターン

BLE・WiFiは同一のStatus2 LEDを共有し、以下の優先順位（上ほど優先）で状態を反映する。BLE/WiFiのいずれか一方でも該当条件を満たせばその表示を採用する。

| 優先度 | 状態 | LED挙動 | 周期（デフォルト、暫定値） | 条件 |
|--------|------|---------|---------------------------|------|
| 1（最優先） | エラー | 高速点滅 | 8 Hz（Duty 50%、約62.5ms ON/OFF） | `E015 WIFI_NOT_CONFIGURED`以外のBLE/WiFi関連エラー（`E016 BLE_INIT_FAILED`、WiFi接続失敗・認証エラー等） |
| 2 | データ通信中 | 点滅 | 2 Hz（Duty 50%、約250ms ON/OFF） | BLE Notify送信中、またはWiFi TCPテレメトリ配信中（§4.1・§5.3） |
| 3 | 接続確立 | 点灯（常時ON） | - | BLE CentralとConnection確立中、またはWiFi TCPクライアント接続中（データ送信の合間を含む） |
| 4（デフォルト） | 未接続 | 消灯 | - | BLE未接続（Advertising中含む）かつWiFi未接続（無効・未接続いずれも） |

- 点滅周期（2Hz／8Hz）は暫定値とし、実機評価後に見やすさ・視認性の観点で調整可能とする（§9.7、未解決事項に追加）。
- BLEとWiFiが同時に異なる状態（例：BLE接続済み・WiFi通信中）の場合は、上表の優先順位に従い高い方（この例では「データ通信中」）を表示する。
- 本LEDは読み取り専用テレメトリ経路の状態表示に限定し、モーション制御・USB通信の状態には連動しない（§1.2 #4の恒久方針と整合、他LEDがあれば役割分担する）。

---

## 4. BLE要件

### 4.1 GATTサービス構成

**方針：** [firmware/REQUIREMENTS.md §4](REQUIREMENTS.md) の既存 USB コマンド（`GET STATE`/`GET POS`/`GET VEL`/`GET ENC`/`GET ADC`/`GET FAULT_INFO`/`STATUS`）が返す JSON 構造をそのまま各キャラクタリスティックのペイロードに転用する。新規バイナリプロトコルを起こさず、`comm.c` の既存 JSON シリアライズ処理を共用できる形にする。

| キャラクタリスティック | Property | 更新頻度 | ペイロード（JSON、UTF-8） | 対応する既存USBコマンド |
|----------------------|----------|---------|--------------------------|------------------------|
| Device Info | Read | 静的（接続時1回） | `{"product":"stepping_motor_driver","board_id":"AABBCCDDEEFF","protocol_version":1,"fw_version":"x.y.z"}` | `IDENTITY` / `GET BOARD_ID` 相当 |
| Axis Status | Read, Notify | 10 Hz（100ms、デフォルト） | `[{"axis":0,"state":"IDLE","pos":12800,"vel":0,"enc":51200},...]`（3軸分の配列） | `GET STATE`/`GET POS`/`GET VEL`/`GET ENC` の集約 |
| Power/ADC | Read, Notify | 10 Hz（100ms、デフォルト） | `{"pot":[v0,v1,v2],"current_mA":850,"voltage_mV":24100}` | `GET ADC` の集約 |
| Fault Info | Read, Notify | 変化時 + 1Hz keepalive | `{"reason":"OVERCURRENT","axis_mask":2,"timestamp_us":1234567890}` | `GET FAULT_INFO` |
| Gear Angle | Read, Notify | 10 Hz（gear monitor 有効時のみ、[GEAR_ANGLE_MONITOR_REQUIREMENTS.md](GEAR_ANGLE_MONITOR_REQUIREMENTS.md) 実装後） | `[{"axis":0,"angle_deg":123.4,"state":"OK","zero_calibrated":true},...]`（未実装時は `{"state":"UNAVAILABLE"}`） | `GET GEAR_ANGLE`／`GET GEAR_ZERO_STATUS`（将来, [GEAR_ANGLE_MONITOR_REQUIREMENTS.md F-GEAR-10](GEAR_ANGLE_MONITOR_REQUIREMENTS.md)） |

**書き込み可能なキャラクタリスティックは設けない**（CCCD＝Notify有効化のためのクライアント設定記述子を除く）。これにより §1.2 #4 の恒久対象外方針をプロトコルレベルで担保する。

**UUID：** プロジェクト専用の 128bit ベースUUIDを発番し、各キャラクタリスティックはベースUUIDの下位バイトをインクリメントして割り当てる（実装時に正式発番、本書では仮UUIDとして扱う）。`multi_i2c_bridge`・`robot_arm_monitor` 等の他コンポーネントと衝突しないことを実装時に確認する。

**ATT MTU：** JSON ペイロード（Axis Status で最大 150 バイト程度）を1パケットで送るため、接続確立後に ATT MTU 185 バイト以上への拡張（`ble_gatts_mtu` ネゴシエーション）を要求する。ネゴシエーション失敗時（相手側非対応）は JSON を複数キャラクタリスティックに分割しない代わりに、ATT の Long Read/複数フラグメント送信にフォールバックする。

### 4.2 Advertising・識別

- **Device Name：** `SMD-<board_id>`（`board_id` は `GET BOARD_ID` と同一の12桁16進文字列、[firmware/REQUIREMENTS.md F-COM-05](REQUIREMENTS.md)）。USB接続時と同じ基板固有IDを無線側でも一貫して使用する。
- **Advertised Service UUID：** 本書 §4.1 のテレメトリサービスUUIDを含める。ホスト側（robot_arm_monitor）はこの Service UUID でスキャンフィルタし、`multi_i2c_bridge` のUSB VID一次分類（[design_spec.md §3.1](../../robot_arm_monitor/docs/design_spec.md) `VID_HINT`）に相当する一次分類手段とする。
- **Advertising間隔：** 100ms〜200ms（デフォルト、発見性と消費電力のバランス。本用途はACアダプタ駆動前提のため省電力最適化は優先度低）。
- **接続後の識別確定：** Device Info キャラクタリスティック読み取りで `product=stepping_motor_driver` を検証してから接続を確定する（USB `identity` コマンドの検証方式、[usb_serial_spec.md](../../multi_i2c_bridge/docs/usb_serial_spec.md) 相当パターンを踏襲）。

### 4.3 接続管理

- ESP32-S3側はPeripheral roleのみ、同時接続Central数は1（1基板につき1つのモニタアプリインスタンスからのみ接続を受け付ける）。
- Connection Interval：30〜50ms を要求（低レイテンシ用途ではないため、消費電力・無線帯域とのバランスを優先）。
- 切断検出後は自動的にAdvertisingを再開する（USB切断時の `EVT COMM_TIMEOUT`（[firmware/REQUIREMENTS.md F-COM-04](REQUIREMENTS.md)）とは独立した経路のため、BLE切断はモーション制御に一切影響しない）。

### 4.4 ペアリング・セキュリティ（[design/SYSTEM_REQUIREMENTS.md §6 #3](../../design/SYSTEM_REQUIREMENTS.md) の解消）

**採用方針：LE Secure Connections + Just Works、ボンディング有効**

| 項目 | 採用 | 理由 |
|------|------|------|
| ペアリング方式 | Just Works（MITM保護なし） | ESP32-S3側にディスプレイ・キーボード等の入出力デバイスがなくPasskey表示ができない。読み取り専用テレメトリのため、盗聴防止（暗号化）のみを目的としMITM対策の必要性は低いと判断 |
| 暗号化 | 接続確立後は必ず暗号化リンクを要求（`ble_gap_security_ie` で `MITM=0, SC=1, encryption必須`） | 平文でのテレメトリ送信（関節位置等）を避ける |
| ボンディング | 有効（NVS にボンド情報を保存） | 再接続の都度ペアリングをやり直さない運用性確保 |
| 発見・選択UX | ユーザーが robot_arm_monitor 上のBLEスキャン結果から `SMD-<board_id>` を選択して接続操作を行う（自動全接続はしない） | USB側の「候補ポート選択→接続」方式（[robot_arm_monitor/REQUIREMENTS.md F-RAM-CONN-01](../../robot_arm_monitor/REQUIREMENTS.md)）と一貫させる |

---

## 5. WiFi要件

### 5.1 用途（[design/SYSTEM_REQUIREMENTS.md §6 #4](../../design/SYSTEM_REQUIREMENTS.md) の解消）

**確定用途：BLEの代替・追加の高帯域テレメトリ経路。** BLEのスループット・接続間隔の制約でトレンドグラフ用途等の高頻度・多チャンネル配信が不足する場合に使用する。データ内容はBLEと同一の読み取り専用方針を維持し、更新頻度・同時配信チャンネル数のみBLEより高くできる。

ファームウェアOTA更新用途は本版のスコープ外とする（§1.3、§9.4で別途要件化予定）。

### 5.2 動作モード

- **Station modeのみ**（既存のホームネットワーク/ラボ内WiFiに接続する）。SoftAP機能は実装しない。
- 起動時デフォルトは**無効**。SSID未設定時に有効化しようとした場合は `ERR E015 WIFI_NOT_CONFIGURED` を返す（BLEがデフォルト有効なのとは対照的に、意図しない電波送出・攻撃面拡大を避けるため明示的な設定を要求する）。

### 5.3 テレメトリプロトコル

- **トランスポート：** 生TCPソケット（ESP32-S3 lwIPスタック上にTCPサーバを立て、robot_arm_monitor（Node.js `net.Socket`）がクライアントとして接続する）。MQTT等のブローカー依存を避け、既存USB/BLEと同様の「アプリが直接デバイスに接続する」構成を維持する。
- **ポート：** 既定 4000/TCP固定（本版では設定コマンド化しない、§9.2）。
- **フレーム形式：** 改行区切りJSONテキスト（`\n` 終端）。§4.1 のBLEキャラクタリスティックと同一のJSON構造に `"type"` フィールドを付与して多重化する。

```
{"type":"axis_status","data":[{"axis":0,"state":"IDLE","pos":12800,"vel":0,"enc":51200},...]}\n
{"type":"power","data":{"pot":[...],"current_mA":850,"voltage_mV":24100}}\n
{"type":"fault","data":{"reason":"NONE","axis_mask":0,"timestamp_us":0}}\n
```

- **配信レート：** デフォルト 10Hz（BLEと同一）だが、`SET WIFI_TELEMETRY_RATE <hz>`（範囲 1〜100Hz）でBLEより高頻度に設定可能とする（USB-CDC経由のみで設定、WiFi経由の設定コマンドは提供しない＝読み取り専用方針の徹底）。
- **同時接続数：** 1（複数クライアント接続要求は2本目以降を拒否する）。
- **書き込み方向の扱い：** サーバは接続後、クライアントからの受信データを一切コマンドとして解釈しない（受信バイトは破棄する、または一定サイズ超過で接続を切断する）。これにより §1.2 #4 の恒久対象外方針をプロトコルレベルで担保する（BLEの「書き込みキャラクタリスティックを設けない」と同じ考え方）。

### 5.4 接続情報のプロビジョニング（USB-CDC経由、確定）

WiFi用のSSID・パスワードは **USB-CDC接続時にのみ** 設定可能とする（SoftAP・BLEプロビジョニングは実装しない）。

| コマンド | 引数 | 説明 |
|---------|------|------|
| `SET WIFI_SSID <ssid>` | 文字列（最大32文字） | NVS（`wifi_config`名前空間）に保存 |
| `SET WIFI_PASS <password>` | 文字列（最大64文字） | NVS に保存。`GET WIFI_STATUS` 等のいかなる応答にも平文表示しない |
| `SET WIFI_ENABLE <0\|1>` | - | WiFi機能の有効/無効切替。NVS保存、次回起動時に反映（即時接続も試行する） |
| `SET WIFI_TELEMETRY_RATE <hz>` | 1〜100 | WiFiテレメトリ配信レート（§5.3） |
| `GET WIFI_STATUS` | - | 応答例：`OK CONNECTED 192.168.1.42 RSSI=-52` または `OK DISCONNECTED` または `OK DISABLED` |

- SSID/パスワード未設定のまま `SET WIFI_ENABLE 1` を受信した場合は `ERR E015 WIFI_NOT_CONFIGURED` を返す。
- パスワードは NVS 暗号化パーティション（既存 `nvs_flash` の暗号化設定に準拠、追加のセキュアストレージは本版では導入しない）に保存する。

### 5.5 ディスカバリ

- mDNS（`_smd-telemetry._tcp.local.`）でサービスを広告し、TXTレコードに `board_id=<12桁16進>` を含める。robot_arm_monitor はIPアドレスに依存せず基板を発見できる（DHCP環境でのIP変化に対応）。

---

## 6. コマンドセット拡張（USB-CDC、[firmware/REQUIREMENTS.md §4](REQUIREMENTS.md) への追加）

### 6.1 状態取得コマンド追加

| コマンド | 引数 | 説明 | 応答例 |
|---------|------|------|--------|
| `GET BLE_STATUS` | - | BLE接続状態取得 | `OK ADVERTISING` / `OK CONNECTED` |
| `GET WIFI_STATUS` | - | WiFi接続状態取得（§5.4） | `OK CONNECTED 192.168.1.42 RSSI=-52` |

### 6.2 設定コマンド追加

| コマンド | 引数 | 説明 |
|---------|------|------|
| `SET BLE_ENABLE <0\|1>` | - | BLE Advertising の有効/無効（NVS保存、デフォルト1） |
| `SET WIFI_SSID <ssid>` | 文字列 | §5.4 |
| `SET WIFI_PASS <password>` | 文字列 | §5.4 |
| `SET WIFI_ENABLE <0\|1>` | - | §5.4 |
| `SET WIFI_TELEMETRY_RATE <hz>` | 1〜100 | §5.3 |

### 6.3 エラーコード追加（[firmware/REQUIREMENTS.md §4.6](REQUIREMENTS.md) への追加）

| コード | 説明 |
|--------|------|
| `E015` | `WIFI_NOT_CONFIGURED`（SSID未設定のまま WiFi 有効化を試みた） |
| `E016` | `BLE_INIT_FAILED`（NimBLEスタック初期化失敗、起動時内部エラー） |

---

## 7. 非機能要件

### 7.1 安全性（最重要）

- BLE・WiFiのいずれの経路からも、モーション制御・設定変更コマンドを一切受け付けない（§4.1・§5.3で機構的に担保）。この制約は [design/SYSTEM_REQUIREMENTS.md §4](../../design/SYSTEM_REQUIREMENTS.md) の確定方針であり、将来のPhaseでも例外を設けない。
- ESTOPを含むモーション制御操作はモニタアプリから制御アプリへのIPC中継のみを経由する（本書の対象外、[同 §6 #2](../../design/SYSTEM_REQUIREMENTS.md)）。BLE/WiFiが停止・切断してもモーション制御には一切影響しない（テレメトリ経路とモーション制御経路は完全に独立）。

### 7.2 信頼性

- BLE切断・WiFi切断はいずれもファームウェアの動作（モーション制御・USB通信）に影響を与えない（テレメトリ配信タスクの異常はモータ制御タスクをブロックしない設計とする、§8のタスク優先度参照）。
- WiFi接続断からの自動再接続を実装する（指数バックオフ、上限間隔60秒）。

### 7.3 無線共存（coexistence）

- BLEとWiFiは同一2.4GHz無線をESP32-S3内部で時分割共有する。両方同時有効時、BLE Notify・WiFi配信の双方でジッタ・遅延が増加し得る（ESP-IDFの内蔵coexistenceアービトレーションに依存）。
- 典型運用では「BLEのみ」または「WiFiのみ」を推奨するが、両方同時有効化を禁止はしない（実装後に実機で許容可能なジッタ量を評価する、§9.3）。

### 7.4 リアルタイム性

- BLE/WiFiテレメトリタスクは既存の `MotorControlTask`（優先度20）・`EncoderTask`（優先度21）より**低い優先度**で動作させ、モーション制御のリアルタイム性に影響を与えない（§8のタスク優先度）。

---

## 8. ソフトウェアアーキテクチャ（案）

### タスク構成追加

| タスク名 | 優先度 | 周期 | 説明 |
|---------|--------|------|------|
| BleTask | 8（低、CommTaskと同等） | Notify: 10Hz | GATTサーバ・Advertising管理・Notify送信 |
| WifiTelemetryTask | 8（低） | 設定可（1〜100Hz） | TCPサーバ・JSON配信・mDNS広告 |

既存タスク（`MotorControlTask`/`EncoderTask`/`ADCTask`/`CommTask`/`StatusTask`、[firmware/REQUIREMENTS.md §6](REQUIREMENTS.md)）より低優先度とし、モーション制御への影響を排除する。

### データフロー

```
MotorControlTask/EncoderTask/ADCTask（内部状態更新）
        │
        ▼
   共有テレメトリ状態（mutex保護）
        │
   ┌────┴────┬─────────────┐
   ▼         ▼             ▼
CommTask   BleTask   WifiTelemetryTask
(USB JSON) (BLE Notify JSON) (TCP JSON)
```

3経路とも同一の内部テレメトリ状態・JSONシリアライズ処理を共用し、実装・保守コストを抑える。

---

## 9. 未解決事項

| # | 項目 | 優先度 | 備考 |
|---|------|--------|------|
| 9.1 | GATT UUID の正式発番 | Low | 実装時に生成、他コンポーネントとの衝突がないことを確認 |
| 9.2 | WiFiテレメトリのTCPポート番号を設定可能にすべきか（現状固定4000） | Low | 複数基板を同一ホストから同時接続する際にポート競合が起きないか要検証（IPアドレスで区別されるため通常は問題ないと想定） |
| 9.3 | BLE/WiFi同時有効時の実機ジッタ許容量 | Medium | §7.3、実機評価が必要 |
| 9.4 | ファームウェアOTA更新の要件化時期 | Low | 本書スコープ外、別途要件化 |
| 9.5 | 筐体内アンテナ性能（金属筐体・近接配置による減衰） | Medium | §3、実機評価が必要 |
| 9.6 | WiFi接続の認証・トークンを設けないことの妥当性 | Low | 読み取り専用・信頼済みローカルネットワーク前提のため本版では未導入。将来トークン認証を追加する余地を残す設計とする |
| 9.7 | Status2 LED（§3.1）の点滅周期（2Hz／8Hz）の妥当性 | Low | 暫定値。実機評価で視認性を確認し、必要に応じて調整する |

---

## 10. 開発ロードマップ（案）

| Phase | 内容 | 前提 |
|-------|------|------|
| 1 | NimBLE導入・GATTサービス実装（Device Info/Axis Status/Power/Fault Info、§4.1） | firmware Phase 1〜5完了（実装済み） |
| 2 | BLEペアリング・ボンディング実装（§4.4）、`GET/SET BLE_ENABLE`（§6） | Phase 1完了 |
| 3 | WiFi Station接続・USB-CDC経由プロビジョニング（§5.2, §5.4） | なし（BLEと独立実装可能） |
| 4 | WiFi TCPテレメトリサーバ・mDNS（§5.3, §5.5） | Phase 3完了 |
| 5 | 統合テスト（BLE/WiFi同時有効時のcoexistence評価、実機アンテナ性能評価、§9.3/§9.5） | Phase 2・4完了 |

Phase 1完了後、[design/SYSTEM_REQUIREMENTS.md §6 #1](../../design/SYSTEM_REQUIREMENTS.md) がクローズし、`robot_arm_monitor` Phase 6（BLEアダプタ実装）に着手可能となる（[robot_arm_monitor/REQUIREMENTS.md §9](../../robot_arm_monitor/REQUIREMENTS.md)）。
