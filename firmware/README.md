# SteppingMotorDriver ファームウェアのコンパイル

このファームウェアは Arduino IDE ではなく、ESP-IDF v5.5 でコンパイルする。
以下は Windows 上の Claude Code から PowerShell コマンドとして実行する手順である。

## 通常のコンパイル

Claude Code を `RobotArm2` で起動している場合は、次のコマンドを同じシェル呼び出し内で実行する。
環境変数と ESP-IDF の有効化は、別々のシェル呼び出しに分けないこと。

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
$env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.5"
$env:IDF_TOOLS_PATH = "C:\Espressif"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\python_env\idf5.5_py3.11_env"
. "C:\Espressif\frameworks\esp-idf-v5.5\export.ps1"
Set-Location ".\SteppingMotorDriver\firmware"
idf.py build
```

Claude Code を `SteppingMotorDriver` で起動している場合は、`Set-Location` だけを次に置き換える。

```powershell
Set-Location ".\firmware"
```

終了時に次のような表示が出ればコンパイル成功である。

```text
Project build complete.
```

主な生成物は `firmware/build/` に出力される。

- `stepping_motor_driver.bin` — アプリケーション本体
- `bootloader/bootloader.bin` — ブートローダー
- `partition_table/partition-table.bin` — パーティションテーブル

## Claude Code への依頼例

```text
SteppingMotorDriver/firmware/README.md の手順に従ってファームウェアをコンパイルし、
成功または失敗を報告してください。失敗した場合は最初の原因となったエラーも示してください。
```

## 初回セットアップとトラブル対応

このリポジトリではターゲットが既に `esp32s3` に設定されているため、通常は
`idf.py set-target` を実行する必要はない。

`idf.py` が見つからない場合は、上記の環境有効化と `idf.py build` を同じシェル呼び出しで
実行しているか確認する。PowerShell のスクリプト実行が禁止されている環境では、先頭の
`Set-ExecutionPolicy -Scope Process ...` が必要である。この設定は現在の PowerShell
プロセスだけに適用され、ユーザーやシステム全体の実行ポリシーは変更しない。

依存関係や CMake キャッシュが壊れた疑いがある場合だけ、次を実行してから再コンパイルする。
`fullclean` は `build/` を消去するので、通常のビルドでは使わない。

```powershell
idf.py fullclean
idf.py build
```

ESP-IDF のインストール自体がない場合は、Espressif の Windows Installer で ESP-IDF v5.5 と
Python 環境を導入してから実行する。このPCで確認済みのインストール先は、上記コマンドに
記載した `C:\Espressif` である。
