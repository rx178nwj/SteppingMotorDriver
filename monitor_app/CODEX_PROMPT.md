# Codex 実装プロンプト — monitor_app Phase 1

## 役割
あなたは Electron デスクトップアプリの実装を担当するエンジニアです。
`SteppingMotorDriver` 基板（ESP32-S3、USB-CDC通信）向けのモニターアプリを、
本リポジトリ内の要件定義書に厳密に従って実装してください。

## 参照ドキュメント（正）
- `SteppingMotorDriver/monitor_app/REQUIREMENTS.md` — 本アプリの要求仕様書（唯一の正）
- `SteppingMotorDriver/firmware/REQUIREMENTS.md` — 通信プロトコル（コマンド/応答フォーマット）の唯一の正。
  Section 4 のコマンド体系・応答フォーマットをそのまま実装すること。本アプリ側でコマンド文字列を変更・拡張しない。
- `SteppingMotorDriver/CLAUDE.md` — ハードウェア構成・設計方針の背景情報（実装対象はソフトウェアのみ）

作業前に上記3ファイルを必ず読み込み、内容を実装に反映してください。矛盾する記憶や推測で補完しないこと。

## 今回のスコープ（Phase 1 のみ）
`monitor_app/REQUIREMENTS.md` Section 8 のロードマップに従い、**Phase 1 のみ**を実装してください。
Phase 2 以降（複数基板、トレンドグラフ、パラメータ設定、ログ記録等）は着手しないこと。

Phase 1 の内容：
1. Electron プロジェクトの雛形作成
2. 単一基板との USB-CDC 接続（`serialport` パッケージ、メインプロセスに集約）
3. `PING` → `OK PONG` による疎通確認
4. 疎通確認後に `STATUS` コマンドを送信し、初期状態を取得（F-CONN-03）

## 確定済みの技術方針（変更しないこと）
| 項目 | 採用 | 参照 |
|------|------|------|
| フレームワーク | Electron | Section 2 |
| メインプロセス | Node.js + `serialport`。シリアル I/O はメインプロセスに集約し、レンダラーは IPC 経由でのみ操作 | Section 2, F-CONN-01〜03 |
| レンダラー | HTML/CSS/JS（プレーンJS。TypeScript化やフレームワーク導入はこの段階では行わない） | Section 2 |
| グラフ描画ライブラリ | uPlot（Phase 3 まで未使用。Phase 1 では導入不要） | Section 7.3 |
| 設定永続化 | `electron-store`（Phase 4 の複数基板対応まで未使用。Phase 1 では導入不要） | Section 7.4 |
| パッケージ化 | 行わない。`npm start` による開発機直接実行のみ | Section 7.2 |
| ボーレート | 115200 bps 固定 | F-CONN-03 |

## Phase 1 実装要件（詳細）

### F-CONN-01: シリアルポート一覧・選択
- 起動時に `serialport.list()` で COM ポート一覧を取得し、レンダラーに表示する
- ユーザーがドロップダウン等でポートを選択し、「接続」ボタンで接続できる
- 「更新」ボタンで一覧を再取得できる

### F-CONN-03: 接続確立シーケンス
- 接続確立後、`PING` を送信し `OK PONG` が返るまで待つ（タイムアウトあり。応答が無い場合は接続失敗として UI にエラー表示）
- 疎通確認後に `STATUS` を送信し、応答（JSON）をコンソール的に画面表示する（この段階では整形 UI は不要、生応答表示で可）
- ボーレートは 115200 固定でハードコードしてよい

### F-CONN-05: 送受信ログ（デバッグ用）
- 送受信した生コマンド・応答を時刻付きでレンダラー画面に一覧表示するデバッグパネルを設ける（開閉不要、Phase 1 では常時表示で可）

### アーキテクチャ制約（Section 4.4 保守性）
- コマンド文字列の送信・応答パースは、後続 Phase での再利用を見据えて **通信レイヤー（例: `src/main/serial-session.js` 等）に集約**し、
  UI ロジック（レンダラー側）と分離すること
- Electron のセキュリティベストプラクティスに従うこと：
  - `contextIsolation: true`、`nodeIntegration: false`
  - `preload.js` で `contextBridge` を使い、IPC 経由の限定APIのみレンダラーに公開する
  - シリアルポートへの直接アクセスをレンダラーに渡さない

### 非機能要件（Phase 1 範囲）
- シリアル通信タイムアウトやパースエラーでアプリ全体がクラッシュしないこと（Section 4.3）
- 接続失敗時は当該操作 UI が分かる形でエラー表示する

## 成果物
- `package.json`（`electron`, `serialport` を依存関係に追加。スクリプトに `npm start` を定義）
- `src/main/main.js`（BrowserWindow 生成、IPC ハンドラ登録）
- `src/main/serial-session.js`（シリアル接続・PING/STATUS シーケンス・送受信ログを管理する通信レイヤー）
- `src/preload/preload.js`（contextBridge 経由の限定 API 公開）
- `src/renderer/index.html` / `renderer.js` / 最小限の CSS（ポート一覧・接続操作・生ログ表示）
- 簡単な `README.md` 追記（起動方法: `npm install` → `npm start`）

## 完了条件（Definition of Done）
- `npm install && npm start` でアプリが起動する
- COM ポート一覧が表示され、選択→接続操作ができる（実機が無い場合はコード上の到達可能性を確認し、その旨を報告する）
- PING/STATUS のシーケンスが実装されており、送受信ログがレンダラーに表示される
- Phase 2 以降の機能（複数基板、グラフ、パラメータ設定、ログ記録）は実装しない
- 実装後、変更ファイル一覧と動作確認方法（または未確認事項）を簡潔に報告すること
