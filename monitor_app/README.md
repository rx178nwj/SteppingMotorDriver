# SteppingMotorDriver Monitor

ESP32-S3 ベースの SteppingMotorDriver 基板へ USB-CDC で接続する Electron モニターです。Phase 3 として、単一基板の接続・状態監視・基本モーション操作・リアルタイムトレンドまで実装しています。

主な機能:

- COM ポート選択、115200 bps 接続、`PING` / `STATUS` 疎通確認
- 50〜1000 ms周期のステータス監視（既定100 ms）
- 3軸の状態、位置、エンコーダ、偏差、速度表示
- 電源電圧、基板電流、フォルト、非同期イベント表示
- ENABLE / DISABLE / STOP / STOP_FREE / ESTOP / CLEAR_FAULT / HOME
- 押下中ジョグ、相対MOVE、絶対MOVETO
- 時刻付き送受信ログ
- uPlotによる速度、位置/エンコーダ、偏差、電流/電圧のリアルタイムトレンド
- 10〜300秒の表示時間切替、表示一時停止、カーソル値確認、しきい値ライン

## 起動

Windows 10/11 と Node.js 22 以降を想定しています。

```powershell
npm install
npm start
```

起動後に対象 COM ポートを選び「接続」を押してください。モーション操作前に対象軸と機械の安全を確認してください。DRV_EN / SLEEP / RESET は基板上の全軸で共通です。

## テスト

```powershell
npm test
```

テストは疑似シリアルポートを使うため、基板なしで実行できます。実機確認では、接続後にログへ `TX PING`、`RX OK PONG`、`TX STATUS`、`RX {...}` が順に表示され、初期ステータス欄へ JSON が表示されることを確認してください。

シリアル I/O は Electron のメインプロセス内に限定し、レンダラーには preload の限定 API のみを公開しています。
