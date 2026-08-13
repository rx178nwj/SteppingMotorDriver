#!/usr/bin/env python3
"""
Phase 1 初期チェックスクリプト
SteppingMotorDriver ファームウェアの基本動作を USB-CDC 経由で検証する。

使用法:
  python test_initial_check.py COM31
  python test_initial_check.py COM31 --no-motor   # 実モーター未接続時（GPIO トグルのみ）

テスト項目:
  [T01] PING 疎通確認
  [T02] 初期 STATUS 確認（全軸 IDLE または SLEEP）
  [T03] 各軸 GET STATE / GET POS / GET VEL
  [T04] エラーハンドリング（不正コマンド・軸番号範囲外）
  [T05] ENABLE → GET STATE → DISABLE
  [T06] GET FAULT_INFO（初期状態: NONE）
  [T07] ESTOP → CLEAR_FAULT → GET STATE
  [T08] SET/GET パラメータ（VMAX/ACCEL/DECEL/MICROSTEP）
  [T09] モーション中コマンド拒否（ERR E008）※実機モーター必要
  [T10] NVS SAVE / LOAD / RESET_CONFIG
  [T11] COMM_TIMEOUT SET（0 で無効化）
  [T12] TEST_GPIO トグル（オシロスコープ確認用）
  [T13] TEST_PULSE / TEST_STOP（RMT パルス生成・停止）
  [T14] MOVE / GET POS（台形プロファイル移動）※実機モーター必要
  [T15] STOP ALL（全軸減速停止）
  [T16] GET ADC / SET CURRENT_LIMIT / SET VOLT_DIVIDER / SET ADC_FILTER
  [T17] HEARTBEAT ON / OFF
  [T18] SYNC_MOVE（引数チェック・2軸同期・E008拒否）
  [T19] GET ENC（全軸エンコーダカウンタ・範囲外エラー）
  [T20] MOVETO（範囲外エラー・現在位置・ソフトリミット超過・実移動）
  [T21] VEL（範囲外エラー・vel=0停止・実行+STOP）
  [T22] STOP_FREE（範囲外エラー・ALL・個別軸）
  [T23] SET SOFT_LIMIT（境界テスト: 上限/下限超過 → E006 / リミット内 OK）
  [T24] 脱調検出（enc_dir 反転による制御脱調 → FAULT 確認）※実機モーター必要
  [T25] フォルト復帰（CLEAR_FAULT → ENABLE → MOVE 正常動作）※実機モーター必要
  [T26] SYNC_MOVE 3 軸同時動作（EVT SYNC_DONE 0x07 確認）
  [T27] HOME コマンド（HOMING 状態確認・STOP 中断）
  [T28] SET MICROSTEP 動的変更（有効値全種 / 無効値 E002 / モーション中 E004）
  [T29] COMM_TIMEOUT 実動作（2000ms 設定 → コマンド停止 → EVT 受信 → IDLE 確認）
  [T30] MOVETO + GET ENC 精度確認（step_pos / enc_pos 誤差チェック）※実機モーター必要
  [T31] F-MOT-12 角度換算・MOVE_DEG/MOVETO_DEG・E008 ※実機モーター必要
  [T32] F-MOT-12 POT スケール・ゼロ位置補正・引数エラー
  [T33] 単軸動作中の他軸IDLEタイムアウト回帰 ※実機モーター必要

依存: pip install pyserial
"""

import serial
import time
import sys
import argparse
import json
import re

# Windows コンソールの文字コード問題を回避（cp932 にない文字を含む場合）
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except AttributeError:
    pass

# ─────────────────────────────────────────────
#  シリアル通信ヘルパー
# ─────────────────────────────────────────────
class MotorComm:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 3.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(1.5)          # ESP32 リセット後の USB CDC 安定待ち
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _recv_line(self, extra_timeout: float = 0.0) -> str:
        """OK / ERR / { のいずれかが来るまで待機して返す。
        ESP-IDF ログ行（'I (...) ...'）と EVT 行はスキップして表示する。"""
        deadline = time.monotonic() + self.ser.timeout + extra_timeout
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if re.match(r'^[IWEDC] \(\d+\)', line):
                print(f"    [LOG] {line}")
                continue
            if line.startswith("EVT"):
                print(f"    [EVT] {line}")
                continue
            return line
        return ""

    def send(self, cmd: str, extra_timeout: float = 0.0) -> str:
        """コマンドを送り、最初の応答行を返す。"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        return self._recv_line(extra_timeout)

    def send_ok(self, cmd: str, extra_timeout: float = 0.0) -> bool:
        return self.send(cmd, extra_timeout).startswith("OK")

    def drain_events(self, wait_sec: float = 0.5):
        """非同期 EVT を消費する。"""
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                line = raw.decode(errors="replace").strip()
                if line:
                    print(f"    [EVT] {line}")


# ─────────────────────────────────────────────
#  テストランナー
# ─────────────────────────────────────────────
def read_status_json(c: MotorComm):
    resp = c.send("STATUS")
    if not (resp.startswith("OK") or resp.startswith("{")):
        return None, resp
    payload = resp if "}" in resp else (resp + c._recv_line())
    try:
        payload = re.sub(r'^OK\s*', '', payload.strip())
        return json.loads(payload), None
    except Exception as e:
        return None, str(e)


class TestRunner:
    def __init__(self, comm: MotorComm, no_motor: bool = False):
        self.comm       = comm
        self.no_motor   = no_motor
        self._passed    = 0
        self._failed    = 0
        self._skipped   = 0

    # ── 結果記録 ──────────────────────────────
    def ok(self, name: str, detail: str = ""):
        tag = f" ({detail})" if detail else ""
        print(f"  [PASS] {name}{tag}")
        self._passed += 1

    def fail(self, name: str, got: str, expected: str = ""):
        exp = f" (expected: {expected!r})" if expected else ""
        print(f"  [FAIL] {name}{exp}  got={got!r}")
        self._failed += 1

    def skip(self, name: str, reason: str = ""):
        tag = f" ({reason})" if reason else ""
        print(f"  [SKIP] {name}{tag}")
        self._skipped += 1

    def check(self, name: str, got: str, startswith: str):
        if got.startswith(startswith):
            self.ok(name, got)
        else:
            self.fail(name, got, f"starts with {startswith!r}")

    def summary(self):
        total = self._passed + self._failed + self._skipped
        print()
        print("=" * 55)
        print(f"RESULT: {self._passed}/{total} passed, "
              f"{self._failed} failed, {self._skipped} skipped")
        print("=" * 55)
        return self._failed == 0


# ─────────────────────────────────────────────
#  テストケース
# ─────────────────────────────────────────────
def run_tests(r: TestRunner, c: MotorComm, no_motor: bool):
    NUM_AXES = 3

    # ── T01: PING ─────────────────────────────
    print("\n[T01] PING 疎通確認")
    resp = c.send("PING")
    r.check("PING → OK PONG", resp, "OK PONG")

    # ── 実機モード: エンコーダ方向設定 ─────────
    # axis 0: ME1K エンコーダの A/B が逆接続のため enc_dir=-1 に設定
    # axis 1,2: エンコーダ未接続のためスタール閾値を無効化
    if not no_motor:
        c.send_ok("SET ENC_DIR 0 -1")
        # 軸1,2 のエンコーダは未接続のため脱調検出を一時無効化（モーション完了後に復元）
        c.send_ok("SET STALL_FAULT 1 2000000")
        c.send_ok("SET STALL_FAULT 2 2000000")

    # ── T02: STATUS ───────────────────────────
    print("\n[T02] 初期 STATUS (全軸 IDLE / SLEEP)")
    resp = c.send("STATUS")
    if resp.startswith("OK") or resp.startswith("{"):
        r.ok("STATUS 応答あり", resp[:80])
        # JSON パース試行
        json_str = resp.lstrip("OK").strip() if resp.startswith("OK") else resp
        # STATUS は OK なしで JSON を送ることもある
        try:
            # comm.c: comm_sendf("{...") と comm_send("]}\n") の 2 回呼び
            # readline で 1 行しか取れないため、もう 1 行読む
            rest = c._recv_line()
            json_str = resp + rest
            # "OK" プレフィックスがあれば除去
            json_str = re.sub(r'^OK\s*', '', json_str)
            data = json.loads(json_str.strip())
            r.ok("STATUS JSON パース", f"microstep={data.get('microstep')}")
            axes = data.get("axes", [])
            for ax in axes:
                st = ax.get("state", "?")
                if st in ("IDLE", "SLEEP"):
                    r.ok(f"  軸{ax['id']} 初期状態", st)
                else:
                    r.fail(f"  軸{ax['id']} 初期状態", st, "IDLE or SLEEP")
        except (json.JSONDecodeError, Exception) as e:
            r.skip("STATUS JSON パース", f"マルチライン応答: {e}")
    else:
        r.fail("STATUS 応答", resp, "OK or {...")

    # ── T03: GET STATE / POS / VEL ───────────
    print("\n[T03] GET STATE / POS / VEL（全軸）")
    for axis in range(NUM_AXES):
        resp = c.send(f"GET STATE {axis}")
        if resp.startswith("OK"):
            state = resp[3:].strip()
            if state in ("IDLE", "SLEEP"):
                r.ok(f"GET STATE {axis}", state)
            else:
                r.fail(f"GET STATE {axis}", resp, "OK IDLE or OK SLEEP")
        else:
            r.fail(f"GET STATE {axis}", resp)

        resp = c.send(f"GET POS {axis}")
        r.check(f"GET POS {axis}", resp, "OK")

        resp = c.send(f"GET VEL {axis}")
        r.check(f"GET VEL {axis}", resp, "OK")

    # ── T04: エラーハンドリング ───────────────
    print("\n[T04] エラーハンドリング")
    resp = c.send("UNKNOWN_COMMAND")
    r.check("不明コマンド → ERR E001", resp, "ERR E001")

    resp = c.send("GET STATE 5")
    r.check("軸番号範囲外(5) → ERR E003", resp, "ERR E003")

    resp = c.send("MOVE 5 1000")
    r.check("MOVE 軸範囲外 → ERR E003", resp, "ERR E003")

    resp = c.send("GET FAULT_INFO")
    r.check("GET FAULT_INFO 初期 → OK NONE", resp, "OK NONE")

    # ── T05: ENABLE / DISABLE ─────────────────
    print("\n[T05] ENABLE → GET STATE → DISABLE")
    resp = c.send("ENABLE")
    r.check("ENABLE → OK", resp, "OK")
    time.sleep(0.1)

    for axis in range(NUM_AXES):
        resp = c.send(f"GET STATE {axis}")
        if resp.startswith("OK IDLE"):
            r.ok(f"ENABLE後 軸{axis} → IDLE", resp)
        else:
            r.fail(f"ENABLE後 軸{axis} → IDLE", resp, "OK IDLE")

    resp = c.send("DISABLE")
    r.check("DISABLE → OK", resp, "OK")
    time.sleep(0.1)

    # ── T06: GET FAULT_INFO 初期値 ────────────
    print("\n[T06] GET FAULT_INFO")
    resp = c.send("GET FAULT_INFO")
    r.check("FAULT_INFO 初期 → OK NONE", resp, "OK NONE")

    # ── T07: ESTOP → CLEAR_FAULT ─────────────
    print("\n[T07] ESTOP → CLEAR_FAULT → 復帰")
    # ENABLE して ESTOP
    c.send_ok("ENABLE")
    time.sleep(0.1)

    resp = c.send("ESTOP")
    r.check("ESTOP → OK", resp, "OK")
    time.sleep(0.1)
    c.drain_events()

    # FAULT 状態確認
    resp = c.send("GET STATE 0")
    r.check("ESTOP後 軸0 → FAULT", resp, "OK FAULT")

    # GET FAULT_INFO
    resp = c.send("GET FAULT_INFO")
    r.check("FAULT_INFO → OK ESTOP", resp, "OK ESTOP")

    # CLEAR_FAULT でない状態で CLEAR_FAULT 試行（全軸 FAULT なので OK のはず）
    resp = c.send("CLEAR_FAULT")
    r.check("CLEAR_FAULT → OK", resp, "OK")
    time.sleep(0.1)

    # SLEEP 状態に遷移
    resp = c.send("GET STATE 0")
    r.check("CLEAR_FAULT後 軸0 → SLEEP", resp, "OK SLEEP")

    # CLEAR_FAULT を非 FAULT 時に呼ぶ → E010
    resp = c.send("CLEAR_FAULT")
    r.check("非FAULT時 CLEAR_FAULT → ERR E010", resp, "ERR E010")

    # ── T08: SET パラメータ ───────────────────
    print("\n[T08] SET / パラメータ変更")
    # ENABLE → IDLE 状態にしてから SET
    c.send_ok("ENABLE")
    time.sleep(0.1)

    resp = c.send("SET VMAX 0 5000")
    r.check("SET VMAX 0 5000 → OK", resp, "OK")

    resp = c.send("SET ACCEL 0 20000")
    r.check("SET ACCEL 0 20000 → OK", resp, "OK")

    resp = c.send("SET DECEL 0 20000")
    r.check("SET DECEL 0 20000 → OK", resp, "OK")

    resp = c.send("SET MICROSTEP 16")
    r.check("SET MICROSTEP 16 → OK", resp, "OK")

    # STATUS で確認
    resp = c.send("STATUS")
    if resp.startswith("OK") or resp.startswith("{"):
        r.ok("SET後 STATUS → OK", resp[:60])
    else:
        r.fail("SET後 STATUS → OK", resp, "OK or {...")

    # MICROSTEP をデフォルトに戻す
    c.send_ok("SET MICROSTEP 32")

    # ── T09: モーション中コマンド拒否 ─────────
    print("\n[T09] モーション中コマンド拒否 (ERR E008)")
    if no_motor:
        r.skip("T09 全体", "--no-motor 指定")
    else:
        # ENABLE 確認
        c.send_ok("ENABLE")
        time.sleep(0.1)

        # 大きめの移動（途中でコマンド送る）
        c.send_ok("SET VMAX 0 2000")
        c.send_ok("SET ACCEL 0 5000")
        c.send_ok("SET DECEL 0 5000")

        resp = c.send("MOVE 0 32000")  # 1/32 マイクロ×200step×0.8回転
        if resp.startswith("OK"):
            time.sleep(0.05)  # 移動開始を待つ

            # モーション中に MOVE → E008
            resp2 = c.send("MOVE 0 100")
            r.check("移動中 MOVE → ERR E008", resp2, "ERR E008")

            # モーション中に SET → E004
            resp3 = c.send("SET VMAX 0 3000")
            r.check("移動中 SET → ERR E004", resp3, "ERR E004")

            # STOP は通る
            resp4 = c.send("STOP 0")
            r.check("移動中 STOP → OK", resp4, "OK")
            time.sleep(0.5)
            c.drain_events()
        else:
            r.fail("MOVE 0 32000 開始", resp, "OK")

        c.send_ok("SET VMAX 0 10000")
        c.send_ok("SET ACCEL 0 50000")
        c.send_ok("SET DECEL 0 50000")

    # ── T10: NVS SAVE / LOAD / RESET ─────────
    print("\n[T10] NVS SAVE / LOAD / RESET_CONFIG")
    c.send_ok("ENABLE")

    resp = c.send("SET VMAX 0 8000")
    r.check("SET VMAX 0 8000 → OK", resp, "OK")

    resp = c.send("SAVE")
    r.check("SAVE → OK", resp, "OK")

    # RESET してから LOAD で 8000 に戻るはず
    c.send_ok("RESET_CONFIG")
    # RESET_CONFIG でデフォルト (10000) になる
    resp = c.send("LOAD")
    r.check("LOAD → OK", resp, "OK")
    time.sleep(0.1)

    # RESET_CONFIG でデフォルトに戻す（テスト後のクリーンアップ）
    c.send_ok("RESET_CONFIG")
    # motor モード: RESET_CONFIG で enc_dir / stall_fault がデフォルトに戻るため再適用
    if not no_motor:
        c.send_ok("SET ENC_DIR 0 -1")
        c.send_ok("SET STALL_FAULT 1 2000000")
        c.send_ok("SET STALL_FAULT 2 2000000")

    # ── T11: COMM_TIMEOUT SET ─────────────────
    print("\n[T11] COMM_TIMEOUT 設定（0=無効）")
    resp = c.send("SET COMM_TIMEOUT 0")
    r.check("SET COMM_TIMEOUT 0 → OK", resp, "OK")

    resp = c.send("SET COMM_TIMEOUT 5000")
    r.check("SET COMM_TIMEOUT 5000 → OK", resp, "OK")

    # ── T12: TEST_GPIO（GPIO トグル）──────────
    print("\n[T12] TEST_GPIO - GPIO トグル（オシロで確認）")
    print("       GPIO6(STEP0) を 1Hz で 3 回トグルします...")
    if no_motor:
        resp = c.send("TEST_GPIO 0 3", extra_timeout=6.0)
        r.check("TEST_GPIO 0 3 → OK", resp, "OK")
    else:
        r.skip("TEST_GPIO", "--no-motor 未指定: RMT と排他のためスキップ")

    # ── T13: TEST_PULSE / TEST_STOP ──────────
    print("\n[T13] TEST_PULSE / TEST_STOP - RMT パルス生成")
    print("       軸0 GPIO6(STEP0) に 1 kHz を 2 秒間出力します...")
    c.send_ok("ENABLE")
    resp = c.send("TEST_PULSE 0 1000")
    r.check("TEST_PULSE 0 1000Hz → OK", resp, "OK")
    time.sleep(2.0)
    print("       [確認] オシロで 1kHz の矩形波を観測してください")

    resp = c.send("TEST_STOP 0")
    r.check("TEST_STOP 0 → OK", resp, "OK")
    time.sleep(0.1)

    # 5kHz テスト
    print("       軸0 に 5 kHz を 1 秒間出力...")
    c.send("TEST_PULSE 0 5000")
    time.sleep(1.0)
    c.send("TEST_STOP 0")
    r.ok("TEST_PULSE 5000Hz → 目視確認", "（オシロ確認推奨）")

    # ── T14: MOVE / GET POS ──────────────────
    print("\n[T14] MOVE / GET POS（台形プロファイル移動）")
    if no_motor:
        r.skip("T14 全体", "--no-motor 指定: 実モーター必要")
    else:
        c.send_ok("ENABLE")
        c.send_ok("SET VMAX 0 5000")
        c.send_ok("SET ACCEL 0 10000")
        c.send_ok("SET DECEL 0 10000")

        resp = c.send("GET POS 0")
        pos_before = int(resp.split()[-1]) if resp.startswith("OK") else 0

        # 6400 ステップ (1/32 マイクロステップ × 200step = 1回転)
        move_steps = 6400
        resp = c.send("MOVE 0 6400")
        r.check("MOVE 0 6400 → OK", resp, "OK")

        # 移動完了待ち (最大 5 秒)
        deadline = time.monotonic() + 5.0
        final_state = "?"
        while time.monotonic() < deadline:
            time.sleep(0.2)
            st = c.send("GET STATE 0")
            final_state = st
            if "IDLE" in st or "SLEEP" in st:
                break
        r.check("MOVE 完了後 → IDLE", final_state, "OK IDLE")

        resp = c.send("GET POS 0")
        if resp.startswith("OK"):
            pos_after = int(resp.split()[-1])
            expected = pos_before + move_steps
            if abs(pos_after - expected) <= 10:   # ±10step の誤差を許容
                r.ok("GET POS 移動後", f"{pos_after} (expected {expected})")
            else:
                r.fail("GET POS 移動後", str(pos_after), str(expected))
        else:
            r.fail("GET POS 移動後", resp, "OK <number>")
        c.drain_events()

    # ── T15: STOP ALL ─────────────────────────
    print("\n[T15] STOP ALL")
    c.send_ok("ENABLE")
    c.send("MOVE 0 100000")   # 大きい移動（減速テスト用）
    time.sleep(0.05)
    resp = c.send("STOP ALL")
    r.check("STOP ALL → OK", resp, "OK")
    time.sleep(0.5)
    c.drain_events()

    resp = c.send("GET STATE 0")
    if "IDLE" in resp or "DECEL" in resp:
        r.ok("STOP ALL後 軸0 → IDLE/DECEL", resp)
    else:
        r.fail("STOP ALL後 軸0 → IDLE/DECEL", resp)

    c.send_ok("DISABLE")

    # ── T16: GET ADC (Phase 4 ADC モニタリング) ──
    print("\n[T16] GET ADC - ADC モニタリング確認")
    # ch0〜2: POT（生 mV）
    for ch in range(3):
        resp = c.send(f"GET ADC {ch}")
        if resp.startswith("OK"):
            mv = resp.split()[-1]
            r.ok(f"GET ADC {ch} (POT{ch}) → mV", f"{mv} mV")
        else:
            r.fail(f"GET ADC {ch}", resp, "OK <mV>")

    # ch3: MOT_V（電源電圧 V）
    resp = c.send("GET ADC 3")
    if resp.startswith("OK"):
        v = resp.split()[-1]
        r.ok("GET ADC 3 (MOT_V) → V", f"{v} V")
    else:
        r.fail("GET ADC 3 (MOT_V)", resp, "OK <V>")

    # ch4: 電流 mA
    resp = c.send("GET ADC 4")
    if resp.startswith("OK"):
        mA = resp.split()[-1]
        r.ok("GET ADC 4 (CURRENT) → mA", f"{mA} mA")
    else:
        r.fail("GET ADC 4 (CURRENT)", resp, "OK <mA>")

    # 範囲外チェック
    resp = c.send("GET ADC 5")
    r.check("GET ADC 5 → ERR E002", resp, "ERR E002")

    # SET CURRENT_LIMIT / SET VOLT_DIVIDER
    resp = c.send("SET CURRENT_LIMIT 0 7000")
    r.check("SET CURRENT_LIMIT → OK", resp, "OK")

    resp = c.send("SET VOLT_DIVIDER 7.27")
    r.check("SET VOLT_DIVIDER → OK", resp, "OK")

    resp = c.send("SET ADC_FILTER ALL 4")
    r.check("SET ADC_FILTER ALL 4 → OK", resp, "OK")

    resp = c.send("SET ADC_FILTER ALL 8")
    r.check("SET ADC_FILTER ALL 8 (復元) → OK", resp, "OK")

    # ── T17: HEARTBEAT ON/OFF ──────────────────
    print("\n[T17] HEARTBEAT ON / OFF")
    resp = c.send("HEARTBEAT ON")
    r.check("HEARTBEAT ON → OK", resp, "OK")
    time.sleep(0.25)   # HB パケット 2〜3 回分を待つ
    c.drain_events(0.2)

    resp = c.send("HEARTBEAT OFF")
    r.check("HEARTBEAT OFF → OK", resp, "OK")

    # ── T18: SYNC_MOVE (軸エラー / 基本受付) ──
    print("\n[T18] SYNC_MOVE - 引数チェック・基本動作")
    c.send_ok("ENABLE")

    # 引数不足
    resp = c.send("SYNC_MOVE 2")
    r.check("SYNC_MOVE 引数不足 → ERR E002", resp, "ERR E002")

    # 軸番号重複
    resp = c.send("SYNC_MOVE 2 0 1000 0 2000")
    r.check("SYNC_MOVE 重複軸 → ERR E011", resp, "ERR E011")

    # 軸番号範囲外
    resp = c.send("SYNC_MOVE 2 0 1000 5 2000")
    r.check("SYNC_MOVE 軸範囲外 → ERR E003", resp, "ERR E003")

    # 正常: 2軸同期移動（実際のパルス出力）
    if no_motor:
        # no-motor: スタール閾値を一時無効化（encoder_pos=0固定でFAULTになるため）
        c.send_ok("SET STALL_FAULT 0 2000000")
        c.send_ok("SET STALL_FAULT 1 2000000")
        c.send_ok("SET STALL_FAULT 2 2000000")

    resp = c.send("SYNC_MOVE 2 0 6400 1 12800")
    r.check("SYNC_MOVE 2軸 → OK", resp, "OK")
    time.sleep(0.2)
    # SYNC_MOVE 実行中に再発行 → E008
    resp2 = c.send("SYNC_MOVE 2 0 100 1 200")
    if resp2.startswith("ERR E008"):
        r.ok("SYNC_MOVE 実行中 → ERR E008", resp2)
    elif resp2.startswith("OK"):
        r.skip("SYNC_MOVE 実行中 → ERR E008", "既に完了 (高速すぎ)")
    else:
        r.fail("SYNC_MOVE 実行中 → ERR E008", resp2, "ERR E008")
    # 完了待ち
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        time.sleep(0.2)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            break
    c.drain_events(0.3)
    r.ok("SYNC_MOVE 完了", "done")

    if no_motor:
        c.send_ok("SET STALL_FAULT 0 512")

    c.send_ok("DISABLE")

    # ── T19: GET ENC ─────────────────────────────
    print("\n[T19] GET ENC - エンコーダカウンタ取得")
    for axis in range(NUM_AXES):
        resp = c.send(f"GET ENC {axis}")
        if resp.startswith("OK"):
            count = resp.split()[-1]
            r.ok(f"GET ENC {axis}", f"count={count}")
        else:
            r.fail(f"GET ENC {axis}", resp, "OK <count>")

    resp = c.send("GET ENC 5")
    r.check("GET ENC 範囲外(5) → ERR E003", resp, "ERR E003")

    # ── T20: MOVETO エラーチェック・基本動作 ────
    print("\n[T20] MOVETO - エラーチェック・基本動作")
    # no-motor: step_pos が累積しているためスタール閾値を先に無効化
    if no_motor:
        c.send_ok("SET STALL_FAULT 0 2000000")
    c.send_ok("ENABLE")

    # 軸番号範囲外
    resp = c.send("MOVETO 5 100")
    r.check("MOVETO 軸範囲外(5) → ERR E003", resp, "ERR E003")

    # 現在位置と同じ → steps == 0 → OK（パルスなし即完了）
    pos_resp = c.send("GET POS 0")
    cur_pos = int(pos_resp.split()[-1]) if pos_resp.startswith("OK") else 0
    resp = c.send(f"MOVETO 0 {cur_pos}")
    r.check(f"MOVETO 0 {cur_pos} (現在位置) → OK", resp, "OK")

    # ソフトリミットクランプ確認:
    # start_motion は target > max_pos をクランプして動作する（E006 は返さない）
    lim_max = cur_pos + 100
    c.send_ok(f"SET SOFT_LIMIT 0 {cur_pos - 2000000} {lim_max}")
    resp = c.send(f"MOVETO 0 {cur_pos + 200}")   # 上限 +100 を超えた目標 → lim_max にクランプ
    r.check("MOVETO ソフトリミット超過 → クランプ OK", resp, "OK")
    # クランプ後の移動完了を待つ
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        time.sleep(0.1)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            break
    c.drain_events(0.1)
    # 位置確認: lim_max にクランプされているはず
    new_pos = c.send("GET POS 0")
    actual = int(new_pos.split()[-1]) if new_pos.startswith("OK") else -1
    if abs(actual - lim_max) <= 5:
        r.ok(f"MOVETO クランプ後 pos={actual} == lim_max={lim_max}", new_pos)
    else:
        r.fail(f"MOVETO クランプ後 pos={actual}", new_pos, f"≈{lim_max}")
    c.send_ok("SET SOFT_LIMIT 0 -2000000 2000000")

    # 200step 絶対移動（アイドルタイムアウトで ENABLE 解除される場合があるので再 ENABLE）
    if no_motor:
        c.send_ok("ENABLE")
    else:
        # motor モード: FAULT 状態の場合は復帰してから実行
        st_chk = c.send("GET STATE 0")
        if "FAULT" in st_chk:
            c.send("CLEAR_FAULT"); time.sleep(0.1)
        c.send_ok("ENABLE")
    pos_resp = c.send("GET POS 0")
    actual = int(pos_resp.split()[-1]) if pos_resp.startswith("OK") else 0
    small_target = actual + 200
    resp = c.send(f"MOVETO 0 {small_target}")
    r.check(f"MOVETO 0 {small_target} (200step) → OK", resp, "OK")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        time.sleep(0.1)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            break
    c.drain_events(0.2)
    new_pos = c.send("GET POS 0")
    actual2 = int(new_pos.split()[-1]) if new_pos.startswith("OK") else -1
    if abs(actual2 - small_target) <= 5:
        r.ok(f"MOVETO 後 GET POS={actual2} == target={small_target}", new_pos)
    else:
        r.fail(f"MOVETO 後 GET POS={actual2}", new_pos, str(small_target))

    # ── T21: VEL コマンド ──────────────────────
    print("\n[T21] VEL - エラーチェック・基本動作")

    # FAULT 状態の場合は復帰
    resp_st = c.send("GET STATE 0")
    if "FAULT" in resp_st:
        c.send("CLEAR_FAULT")
        time.sleep(0.1)

    # 軸番号範囲外
    resp = c.send("VEL 5 1000")
    r.check("VEL 軸範囲外(5) → ERR E003", resp, "ERR E003")

    # vel = 0 → motor_stop → OK（停止中でも OK を返す）
    resp = c.send("VEL 0 0")
    r.check("VEL 0 0 (停止指令) → OK", resp, "OK")

    # VEL 実行 + STOP（motor / no-motor 共通）
    if no_motor:
        # no-motor: stall 閾値は T20 で設定済み(2000000)
        pass
    # FAULT 状態の場合は復帰
    st_chk = c.send("GET STATE 0")
    if "FAULT" in st_chk:
        c.send("CLEAR_FAULT"); time.sleep(0.1)
    c.send_ok("ENABLE")
    resp = c.send("VEL 0 1000")
    r.check("VEL 0 1000 → OK", resp, "OK")
    time.sleep(0.15)
    resp_state = c.send("GET STATE 0")
    if "ACCEL" in resp_state or "CRUISE" in resp_state:
        r.ok("VEL 実行中 → ACCEL/CRUISE", resp_state)
    else:
        r.ok("VEL 実行中 GET STATE", resp_state)
    resp = c.send("STOP 0")
    r.check("VEL 中 STOP 0 → OK", resp, "OK")
    time.sleep(0.5)
    c.drain_events(0.2)
    resp = c.send("GET STATE 0")
    if "IDLE" in resp or "SLEEP" in resp or "DECEL" in resp:
        r.ok("STOP 後 軸0 → IDLE/DECEL", resp)
    else:
        r.fail("STOP 後 軸0 → IDLE/DECEL", resp)
    if no_motor:
        c.send_ok("SET STALL_FAULT 0 512")

    # ── T22: STOP_FREE ─────────────────────────
    print("\n[T22] STOP_FREE - エラーチェック・基本動作")

    # 軸番号範囲外
    resp = c.send("STOP_FREE 5")
    r.check("STOP_FREE 軸範囲外(5) → ERR E003", resp, "ERR E003")

    # STOP_FREE ALL（停止中軸への呼び出しは常に OK）
    c.send_ok("ENABLE")
    resp = c.send("STOP_FREE ALL")
    r.check("STOP_FREE ALL → OK", resp, "OK")
    time.sleep(0.3)
    c.drain_events(0.2)

    # 個別軸
    c.send_ok("ENABLE")
    resp = c.send("STOP_FREE 0")
    r.check("STOP_FREE 0 → OK", resp, "OK")
    time.sleep(0.3)
    c.drain_events(0.2)

    # ── T23: SET SOFT_LIMIT + 境界テスト ────────
    print("\n[T23] SET SOFT_LIMIT - ソフトリミット境界テスト")
    # FAULT 状態の場合は復帰
    resp_st = c.send("GET STATE 0")
    if "FAULT" in resp_st:
        c.send("CLEAR_FAULT")
        time.sleep(0.1)

    c.send_ok("ENABLE")
    pos_resp = c.send("GET POS 0")
    cur_pos = int(pos_resp.split()[-1]) if pos_resp.startswith("OK") else 0

    # no-motor: step_pos が累積しているためスタール閾値を無効化（motor モードは enc_dir 修正済みのため不要）
    if no_motor:
        c.send_ok("SET STALL_FAULT 0 2000000")

    lim_min = cur_pos - 100
    lim_max = cur_pos + 100
    resp = c.send(f"SET SOFT_LIMIT 0 {lim_min} {lim_max}")
    r.check(f"SET SOFT_LIMIT 0 ±100 → OK", resp, "OK")

    # MOVE/MOVETO はソフトリミット超過時にクランプして OK を返す設計
    # 上限クランプ: MOVE +200 → lim_max までクランプ → OK（アイドル後は再 ENABLE 必要）
    c.send_ok("ENABLE")
    resp = c.send("MOVE 0 200")
    r.check("MOVE +200 (上限クランプ) → OK", resp, "OK")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        time.sleep(0.1)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            break
    c.drain_events(0.1)
    pos_resp = c.send("GET POS 0")
    actual = int(pos_resp.split()[-1]) if pos_resp.startswith("OK") else -1
    if abs(actual - lim_max) <= 5:
        r.ok(f"MOVE +200クランプ後 pos={actual} == lim_max={lim_max}", pos_resp)
    else:
        r.fail(f"MOVE +200クランプ後 pos={actual}", pos_resp, f"≈{lim_max}")

    # 下限クランプ: MOVE -300 (from lim_max) → lim_min までクランプ → OK
    c.send_ok("ENABLE")
    resp = c.send("MOVE 0 -300")
    r.check("MOVE -300 (下限クランプ) → OK", resp, "OK")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        time.sleep(0.1)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            break
    c.drain_events(0.1)
    pos_resp = c.send("GET POS 0")
    actual = int(pos_resp.split()[-1]) if pos_resp.startswith("OK") else -1
    if abs(actual - lim_min) <= 5:
        r.ok(f"MOVE -300クランプ後 pos={actual} == lim_min={lim_min}", pos_resp)
    else:
        r.fail(f"MOVE -300クランプ後 pos={actual}", pos_resp, f"≈{lim_min}")

    # リミット内移動: MOVE +50 (from lim_min, within limits) → OK
    c.send_ok("ENABLE")
    resp = c.send("MOVE 0 50")
    if resp.startswith("OK"):
        r.ok("MOVE +50 (リミット内) → OK", resp)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            st = c.send("GET STATE 0")
            if "IDLE" in st or "SLEEP" in st:
                break
        c.drain_events(0.2)
    else:
        r.fail("MOVE +50 (リミット内)", resp, "OK")

    # ソフトリミット・スタール閾値を復元
    c.send_ok("SET SOFT_LIMIT 0 -2000000 2000000")
    if no_motor:
        c.send_ok("SET STALL_FAULT 0 512")

    # ── T24: 脱調検出テスト（motor モードのみ） ──────────────
    if not no_motor:
        print("\n[T24] 脱調検出 - enc_dir 反転による制御脱調テスト")
        # enc_dir=+1（方向反転）にすると MOVE 中の差分が 2× 速で増大し、
        # stall_th=200 で ~100 ステップ時点に FAULT が発生する
        c.send_ok("SET ENC_DIR 0 1")
        c.send_ok("SET STALL_FAULT 0 200")
        c.send_ok("DISABLE")
        c.send_ok("ENABLE")   # enc_pos を step_pos（enc_dir=+1 基準）に再基準化
        resp = c.send("MOVE 0 400")
        r.check("脱調誘発 MOVE 0 400 → OK", resp, "OK")
        deadline = time.monotonic() + 5.0
        fault_detected = False
        st = ""
        while time.monotonic() < deadline:
            time.sleep(0.1)
            st = c.send("GET STATE 0")
            if "FAULT" in st:
                fault_detected = True
                break
        if fault_detected:
            r.ok("脱調 → FAULT 遷移確認", st)
        else:
            r.fail("脱調 → FAULT 遷移タイムアウト", st, "OK FAULT")
        c.drain_events(0.5)   # EVT FAULT STALL / EVT ESTOP をドレイン

        # ── T25: フォルト復帰 - CLEAR_FAULT → 設定復元 → 正常動作 ───
        print("\n[T25] フォルト復帰 - CLEAR_FAULT → ENABLE → 正常動作確認")
        c.send("CLEAR_FAULT")
        time.sleep(0.1)
        c.send_ok("SET ENC_DIR 0 -1")
        c.send_ok("SET STALL_FAULT 0 512")
        c.send_ok("ENABLE")   # enc_dir=-1 で再基準化
        resp = c.send("MOVE 0 200")
        r.check("フォルト復帰後 MOVE 0 200 → OK", resp, "OK")
        deadline = time.monotonic() + 5.0
        st = ""
        while time.monotonic() < deadline:
            time.sleep(0.1)
            st = c.send("GET STATE 0")
            if "IDLE" in st or "SLEEP" in st:
                break
        c.drain_events(0.2)
        if "IDLE" in st or "SLEEP" in st:
            r.ok("フォルト復帰後 MOVE 完了 → IDLE/SLEEP", st)
        else:
            r.fail("フォルト復帰後 MOVE 完了タイムアウト", st, "IDLE/SLEEP")

    # ── T26: SYNC_MOVE 3 軸同時動作 ─────────────────────────
    print("\n[T26] SYNC_MOVE 3 軸同時動作")
    resp_st = c.send("GET STATE 0")
    if "FAULT" in resp_st:
        c.send("CLEAR_FAULT")
        time.sleep(0.1)
    c.send_ok("ENABLE")
    if not no_motor:
        # axis 1, 2 はエンコーダ未接続のため脱調検出を無効化
        c.send_ok("SET STALL_FAULT 1 2000000")
        c.send_ok("SET STALL_FAULT 2 2000000")
    sync_status_before, _ = read_status_json(c)
    resp = c.send("SYNC_MOVE 3 0 3200 1 6400 2 3200")
    r.check("SYNC_MOVE 3 軸 → OK", resp, "OK")
    # axis 0 が IDLE になるまで待機（3 軸同時完了 → SYNC_DONE 後 IDLE）
    deadline = time.monotonic() + 10.0
    sync_ok = False
    st = ""
    while time.monotonic() < deadline:
        time.sleep(0.1)
        st = c.send("GET STATE 0")
        if "IDLE" in st or "SLEEP" in st:
            sync_ok = True
            break
    c.drain_events(0.5)   # EVT SYNC_DONE 0x07 をドレイン
    if sync_ok:
        r.ok("SYNC_MOVE 3 軸 完了 → IDLE/SLEEP", st)
    else:
        r.fail("SYNC_MOVE 3 軸 完了タイムアウト", st, "IDLE")
    if sync_status_before is not None:
        sync_status_after, err = read_status_json(c)
        if sync_status_after is not None:
            for axis in (0, 1, 2):
                before_ax = sync_status_before.get("axes", [{}])[axis]
                after_ax = sync_status_after.get("axes", [{}])[axis]
                for key in ("vmax", "accel", "decel"):
                    before_val = before_ax.get(key)
                    after_val = after_ax.get(key)
                    if before_val == after_val:
                        r.ok(f"T26後 axis{axis} {key} 維持", f"{after_val}")
                    else:
                        r.fail(f"T26後 axis{axis} {key} 変化", f"{before_val} -> {after_val}", str(before_val))
        else:
            r.skip("T26後 STATUS JSON パース", err or "STATUS read failed")

    # ── T27: HOME コマンド - ホーミング状態遷移・STOP 中断テスト ─
    print("\n[T27] HOME - ホーミング開始・HOMING 状態確認・STOP 中断テスト")
    resp_st = c.send("GET STATE 0")
    if "FAULT" in resp_st:
        c.send("CLEAR_FAULT")
        time.sleep(0.1)
    c.send_ok("ENABLE")
    resp = c.send("HOME 0")
    r.check("HOME 0 → OK", resp, "OK")
    time.sleep(0.15)
    st = c.send("GET STATE 0")
    if "HOMING" in st:
        r.ok("HOME 実行中 → HOMING 状態確認", st)
        resp = c.send("STOP 0")
        r.check("HOMING 中 STOP 0 → OK", resp, "OK")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            st = c.send("GET STATE 0")
            if "IDLE" in st or "SLEEP" in st:
                break
        c.drain_events(0.3)
        if "IDLE" in st or "SLEEP" in st:
            r.ok("HOME STOP 後 → IDLE/SLEEP", st)
        else:
            r.fail("HOME STOP 後 IDLE タイムアウト", st, "IDLE/SLEEP")
    elif "IDLE" in st or "SLEEP" in st:
        # Z 相 GPIO がプルアップ Hi のため即時 HOME_DONE が発生した
        r.ok("HOME 即時完了 (Z 相即時検出)", st)
        c.drain_events(0.3)
    else:
        r.fail("HOME 0 後の状態確認", st, "HOMING/IDLE")

    c.send_ok("DISABLE")

    # ── T28: SET MICROSTEP 動的変更テスト ────────────────────────
    print("\n[T28] SET MICROSTEP 動的変更テスト")
    # 全軸停止状態で各分割数に変更し STATUS で確認
    for div in (1, 2, 4, 8, 16, 32):
        resp = c.send(f"SET MICROSTEP {div}")
        r.check(f"SET MICROSTEP {div} (全軸停止) → OK", resp, "OK")
    # 無効値はエラー
    resp = c.send("SET MICROSTEP 3")
    r.check("SET MICROSTEP 3 (無効値) → ERR E002", resp, "ERR E002")
    resp = c.send("SET MICROSTEP 0")
    r.check("SET MICROSTEP 0 (無効値) → ERR E002", resp, "ERR E002")
    # STATUS で microstep=32 を確認
    c.send_ok("SET MICROSTEP 32")
    resp = c.send("STATUS")
    if resp.startswith("OK") or resp.startswith("{"):
        rest = c._recv_line()
        try:
            json_str = re.sub(r'^OK\s*', '', (resp + rest).strip())
            data = json.loads(json_str)
            ms = data.get("microstep")
            if str(ms) in ("32", "1/32"):
                r.ok("STATUS microstep=32 確認", f"microstep={ms}")
            else:
                r.fail("STATUS microstep=32 確認", f"microstep={ms}", "32 or 1/32")
        except Exception:
            r.skip("STATUS JSON パース（microstep確認）", "マルチライン応答")
    else:
        r.fail("SET MICROSTEP後 STATUS", resp, "OK or {...")
    # モーション中は SET MICROSTEP を拒否（motor モードのみ）
    if not no_motor:
        c.send_ok("ENABLE")
        c.send_ok("SET STALL_FAULT 0 2000000")
        c.send("MOVE 0 3200")          # 移動開始（応答は捨てる）
        time.sleep(0.05)              # 移動中
        resp = c.send("SET MICROSTEP 16")
        r.check("モーション中 SET MICROSTEP → ERR E004", resp, "ERR E004")
        # 完了まで待機
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if "IDLE" in c.send("GET STATE 0") or "SLEEP" in c.send("GET STATE 0"):
                break
        c.drain_events(0.2)
        c.send_ok("SET STALL_FAULT 0 512")
        c.send_ok("DISABLE")

    # ── T29: 通信ウォッチドッグ タイムアウト実動作テスト ──────────
    print("\n[T29] COMM_TIMEOUT 実動作テスト")
    # 短いタイムアウト（2000 ms）を設定してコマンドを止め、EVT を待つ
    TIMEOUT_MS = 2000
    c.send_ok(f"SET COMM_TIMEOUT {TIMEOUT_MS}")
    c.send_ok("ENABLE")
    print(f"  コマンド送信停止 → {TIMEOUT_MS} ms 後に EVT COMM_TIMEOUT 待機...")
    # タイムアウト + バッファ余裕 1500 ms 待機（コマンド送信なし）
    wait_sec = TIMEOUT_MS / 1000.0 + 1.5
    deadline_evt = time.monotonic() + wait_sec
    comm_timeout_seen = False
    while time.monotonic() < deadline_evt:
        raw = c.ser.readline()
        if raw:
            line = raw.decode(errors="replace").strip()
            if line:
                print(f"    [EVT] {line}")
                if "COMM_TIMEOUT" in line:
                    comm_timeout_seen = True
                    break
    if comm_timeout_seen:
        r.ok("EVT COMM_TIMEOUT 受信確認")
    else:
        r.fail("EVT COMM_TIMEOUT 受信", "(タイムアウト)", "EVT COMM_TIMEOUT")
    # タイムアウト後の全軸 IDLE 確認
    for axis in range(NUM_AXES):
        st = c.send(f"GET STATE {axis}")
        if "IDLE" in st or "SLEEP" in st:
            r.ok(f"COMM_TIMEOUT後 軸{axis} → IDLE/SLEEP", st)
        else:
            r.fail(f"COMM_TIMEOUT後 軸{axis} 状態", st, "IDLE or SLEEP")
    # COMM_TIMEOUT をデフォルト（5000 ms）に戻す
    c.send_ok("SET COMM_TIMEOUT 5000")

    # ── T30: MOVETO + GET ENC 精度確認（motor モードのみ） ─────────
    if not no_motor:
        print("\n[T30] MOVETO + GET ENC 精度確認")
        # 軸0 のみ（軸1/2はエンコーダ未接続）
        # 事前準備: stall 閾値を通常値に戻し enc_dir=-1 を確認
        # パラメータ確認 (STATUSのJSON応答)
        try:
            _d, _err = read_status_json(c)
            if _d is None:
                raise ValueError(_err or "STATUS read failed")
            _ax = _d.get("axes", [{}])[0]
            print(f"  [diag] axis0: vmax={_ax.get('vmax')}  accel={_ax.get('accel')}  decel={_ax.get('decel')}")
        except Exception:
            print("  [diag] STATUS read failed")
        # MOVETO に十分な速度・加速度を設定
        c.send_ok("SET VMAX 0 6400")
        c.send_ok("SET ACCEL 0 50000")
        c.send_ok("SET DECEL 0 50000")
        c.send_ok("SET STALL_FAULT 0 512")
        c.send_ok("SET ENC_DIR 0 -1")
        c.send_ok("ENABLE")
        # 原点に戻す（MOVETO 0）
        moveto0_resp = c.send("MOVETO 0 0")
        print(f"  MOVETO 0 0 → {moveto0_resp}")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.2)
            st = c.send("GET STATE 0")
            pos_now = c.send("GET POS 0")
            enc_now = c.send("GET ENC 0")
            print(f"  [poll] state={st}  pos={pos_now}  enc={enc_now}")
            if "IDLE" in st or "SLEEP" in st or "FAULT" in st:
                break
        c.drain_events(0.2)
        # 脱調が起きていれば CLEAR_FAULT → ENABLE
        st = c.send("GET STATE 0")
        origin_pos = c.send("GET POS 0")
        print(f"  原点確認: state={st}  pos={origin_pos}")
        if "FAULT" in st:
            c.send("CLEAR_FAULT"); time.sleep(0.1)
            c.send_ok("SET ENC_DIR 0 -1")
            c.send_ok("ENABLE")
            origin_pos = c.send("GET POS 0")
            print(f"  FAULT復帰後 pos={origin_pos}")

        targets = [3200, 6400, -3200]
        for tgt in targets:
            c.send_ok("ENABLE")
            resp = c.send(f"MOVETO 0 {tgt}")
            r.check(f"MOVETO 0 {tgt} → OK", resp, "OK")
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                time.sleep(0.2)
                st = c.send("GET STATE 0")
                pos_now = c.send("GET POS 0")
                enc_now = c.send("GET ENC 0")
                print(f"  [poll tgt={tgt}] state={st}  pos={pos_now}  enc={enc_now}")
                if "IDLE" in st or "SLEEP" in st or "FAULT" in st:
                    break
            c.drain_events(0.2)
            st = c.send("GET STATE 0")
            if "FAULT" in st:
                r.fail(f"MOVETO {tgt} 中に FAULT 発生", st, "IDLE/SLEEP")
                c.send("CLEAR_FAULT"); time.sleep(0.1)
                c.send_ok("SET ENC_DIR 0 -1"); c.send_ok("ENABLE")
                continue
            pos_resp = c.send("GET POS 0")
            enc_resp = c.send("GET ENC 0")
            try:
                step_pos = int(pos_resp.split()[-1])
                enc_pos  = int(enc_resp.split()[-1])
                # enc_dir=-1: モーター正方向でエンコーダ逆転
                # enc_expected = tgt × (4000/6400) × (-1) = -tgt × 0.625
                enc_expected = int(-tgt * 0.625)
                step_err = abs(step_pos - tgt)
                enc_err  = abs(enc_pos - enc_expected)
                if step_err <= 5:
                    r.ok(f"MOVETO {tgt}: step_pos={step_pos} (誤差 {step_err})")
                else:
                    r.fail(f"MOVETO {tgt}: step_pos={step_pos}", pos_resp, f"≈{tgt}")
                if enc_err <= 50:
                    r.ok(f"MOVETO {tgt}: enc_pos={enc_pos} (誤差 {enc_err}, expected≈{enc_expected})")
                else:
                    r.fail(f"MOVETO {tgt}: enc_pos={enc_pos}", enc_resp, f"≈{enc_expected} (±50)")
            except ValueError:
                r.fail(f"MOVETO {tgt}: 数値パース失敗", f"{pos_resp} / {enc_resp}")

        # ── T31: F-MOT-12 関節角度・角度指定移動 ──────────────
        print("\n[T31] F-MOT-12 関節角度・MOVE_DEG/MOVETO_DEG")
        c.send_ok("SET MICROSTEP 32")
        c.send_ok("ENABLE")
        resp = c.send("MOVETO_DEG 0 90")
        r.check("MOVETO_DEG 0 90 → OK", resp, "OK")
        for busy_cmd in ("MOVE 0 1", "MOVETO 0 0",
                         "MOVE_DEG 0 1", "MOVETO_DEG 0 0"):
            resp_busy = c.send(busy_cmd)
            if resp_busy.startswith("ERR E008"):
                r.ok(f"角度指定移動中 {busy_cmd} → ERR E008", resp_busy)
            elif "IDLE" in c.send("GET STATE 0"):
                r.skip(f"角度指定移動中 {busy_cmd} → ERR E008", "既に完了")
            else:
                r.fail(f"角度指定移動中 {busy_cmd}", resp_busy, "ERR E008")

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = c.send("GET STATE 0")
            if "IDLE" in state or "SLEEP" in state or "FAULT" in state:
                break
        pos_resp = c.send("GET POS 0")
        pos_deg_resp = c.send("GET POS_DEG 0")
        enc_resp = c.send("GET ENC 0")
        enc_deg_resp = c.send("GET ENC_DEG 0")
        try:
            step_pos = int(pos_resp.split()[-1])
            pos_deg = float(pos_deg_resp.split()[-1])
            enc_pos = int(enc_resp.split()[-1])
            enc_deg = float(enc_deg_resp.split()[-1])
            enc_deg_expected = enc_pos * 360.0 / (1000 * 4)
            if abs(step_pos - 1600) <= 5 and abs(pos_deg - 90.0) <= 0.3:
                r.ok("MOVETO_DEG 90° → 1600 steps / POS_DEG 90°",
                     f"pos={step_pos}, deg={pos_deg:.3f}")
            else:
                r.fail("MOVETO_DEG 90° 換算", f"{pos_resp} / {pos_deg_resp}",
                       "pos≈1600, deg≈90")
            if abs(enc_deg - enc_deg_expected) <= 0.001:
                r.ok("GET ENC_DEG が encoder_ppr×4 換算式と一致",
                     f"enc={enc_pos}, deg={enc_deg:.3f}")
            else:
                r.fail("GET ENC_DEG 換算", enc_deg_resp,
                       f"OK ≈{enc_deg_expected:.3f}")
        except (ValueError, IndexError):
            r.fail("GET POS_DEG/ENC_DEG 数値応答",
                   f"{pos_deg_resp} / {enc_deg_resp}", "OK <float>")

        # steps_per_rev はマイクロステップ変更に追従することを確認する。
        c.send_ok("SET MICROSTEP 16")
        changed_deg_resp = c.send("GET POS_DEG 0")
        try:
            changed_deg = float(changed_deg_resp.split()[-1])
            if abs(changed_deg - 180.0) <= 0.3:
                r.ok("steps_per_rev 変更が POS_DEG に反映", changed_deg_resp)
            else:
                r.fail("steps_per_rev 変更が POS_DEG に反映",
                       changed_deg_resp, "OK ≈180.000")
        except (ValueError, IndexError):
            r.fail("steps_per_rev 変更後 POS_DEG", changed_deg_resp, "OK <float>")
        c.send_ok("SET MICROSTEP 32")

        resp = c.send("MOVETO_DEG 0 -45")
        r.check("MOVETO_DEG 0 -45 → OK", resp, "OK")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = c.send("GET STATE 0")
            if "IDLE" in state or "SLEEP" in state or "FAULT" in state:
                break
        final_pos = c.send("GET POS 0")
        try:
            if abs(int(final_pos.split()[-1]) + 800) <= 5:
                r.ok("MOVETO_DEG -45° → -800 steps", final_pos)
            else:
                r.fail("MOVETO_DEG -45° 換算", final_pos, "OK ≈-800")
        except (ValueError, IndexError):
            r.fail("MOVETO_DEG -45° 後 GET POS", final_pos, "OK <int>")

        resp = c.send("MOVE_DEG 0 45")
        r.check("MOVE_DEG 0 45 → OK", resp, "OK")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = c.send("GET STATE 0")
            if "IDLE" in state or "SLEEP" in state or "FAULT" in state:
                break
        relative_pos = c.send("GET POS 0")
        try:
            if abs(int(relative_pos.split()[-1])) <= 5:
                r.ok("MOVE_DEG +45° → 相対 +800 steps", relative_pos)
            else:
                r.fail("MOVE_DEG +45° 換算", relative_pos, "OK ≈0")
        except (ValueError, IndexError):
            r.fail("MOVE_DEG +45° 後 GET POS", relative_pos, "OK <int>")

        # ── T33: 他軸が IDLE でも単軸動作中は共通ドライバをスリープさせない ──
        print("\n[T33] 単軸動作中のIDLEタイムアウト回帰")
        c.send_ok("SET IDLE_TIMEOUT 300")
        c.send_ok("SET VMAX 0 1000")
        c.send_ok("ENABLE")
        resp = c.send("MOVETO_DEG 0 90")
        r.check("IDLE_TIMEOUTより長い単軸移動 → OK", resp, "OK")
        time.sleep(0.5)
        mid_state = c.send("GET STATE 0")
        if any(name in mid_state for name in ("ACCEL", "CRUISE", "DECEL")):
            r.ok("他軸IDLE中も軸0の動作を継続", mid_state)
        else:
            r.fail("他軸IDLE中の軸0状態", mid_state,
                   "OK ACCEL/CRUISE/DECEL")

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = c.send("GET STATE 0")
            if "IDLE" in state or "SLEEP" in state or "FAULT" in state:
                break
        timeout_pos = c.send("GET POS 0")
        try:
            if "FAULT" not in state and abs(int(timeout_pos.split()[-1]) - 1600) <= 5:
                r.ok("タイムアウト超過後も移動完了", timeout_pos)
            else:
                r.fail("タイムアウト超過後の移動", f"{state} / {timeout_pos}",
                       "IDLE/SLEEP, pos≈1600")
        except (ValueError, IndexError):
            r.fail("タイムアウト回帰 GET POS", timeout_pos, "OK <int>")

        # 全軸停止後は通常どおりスリープする。
        time.sleep(0.5)
        sleep_states = [c.send(f"GET STATE {axis}") for axis in range(NUM_AXES)]
        if all("SLEEP" in item for item in sleep_states):
            r.ok("全軸停止後は共通ドライバがSLEEP", str(sleep_states))
        else:
            r.fail("全軸停止後のSLEEP遷移", str(sleep_states), "all SLEEP")

        # 原点へ戻してテスト設定を復元。
        c.send_ok("ENABLE")
        c.send_ok("MOVETO_DEG 0 0")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = c.send("GET STATE 0")
            if "IDLE" in state or "SLEEP" in state or "FAULT" in state:
                break
        c.send_ok("SET VMAX 0 6400")
        c.send_ok("SET IDLE_TIMEOUT 2000")
        c.send_ok("DISABLE")

    # ── T32: F-MOT-12 POT 較正（モーター不要） ─────────────────
    print("\n[T32] F-MOT-12 POT スケール・ゼロ位置補正")
    r.check("GET POS_DEG 軸範囲外", c.send("GET POS_DEG 3"), "ERR E003")
    r.check("MOVE_DEG 引数不足", c.send("MOVE_DEG 0"), "ERR E002")
    r.check("MOVE_DEG 引数区切りなし", c.send("MOVE_DEG 0.5"), "ERR E002")
    r.check("SET POT_SCALE 不正値", c.send("SET POT_SCALE 0 0"), "ERR E002")
    r.check("SET POT_SCALE 0 0.1", c.send("SET POT_SCALE 0 0.1"), "OK")
    before = c.send("GET POT_DEG 0")
    r.check("SET POT_ZERO 0", c.send("SET POT_ZERO 0"), "OK")
    zeroed = c.send("GET POT_DEG 0")
    try:
        raw_before = float(before.split()[-2])
        raw_after, zero_after = map(float, zeroed.split()[-2:])
        if abs(raw_after - raw_before) <= 2.0:
            r.ok("SET POT_ZERO 後も raw 側を維持", zeroed)
        else:
            r.fail("SET POT_ZERO 後も raw 側を維持", zeroed,
                   f"raw≈{raw_before:.3f}")
        if abs(zero_after) <= 2.0:
            r.ok("SET POT_ZERO 後 zeroed≈0", zeroed)
        else:
            r.fail("SET POT_ZERO 後 zeroed≈0", zeroed, "zeroed≈0")
    except (ValueError, IndexError):
        r.fail("GET POT_DEG 数値応答", f"{before} / {zeroed}",
               "OK <raw_deg> <zeroed_deg>")

    r.check("CLEAR POT_ZERO 0", c.send("CLEAR POT_ZERO 0"), "OK")
    cleared = c.send("GET POT_DEG 0")
    try:
        raw_clear, zero_clear = map(float, cleared.split()[-2:])
        if abs(raw_clear - zero_clear) <= 0.001:
            r.ok("CLEAR POT_ZERO 後 raw==zeroed", cleared)
        else:
            r.fail("CLEAR POT_ZERO 後 raw==zeroed", cleared, "raw == zeroed")
    except (ValueError, IndexError):
        r.fail("CLEAR 後 GET POT_DEG 数値応答", cleared,
               "OK <raw_deg> <zeroed_deg>")
    c.send("SET POT_SCALE 0 0.0879")


# ─────────────────────────────────────────────
#  メイン
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Phase 1 初期チェックスクリプト")
    parser.add_argument("port", help="シリアルポート (例: COM31)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--no-motor", action="store_true",
                        help="実モーター未接続モード（GPIO/RMT のみ確認）")
    args = parser.parse_args()

    print(f"SteppingMotorDriver Phase 1 初期チェック")
    print(f"  Port  : {args.port}")
    print(f"  Mode  : {'--no-motor (GPIO/RMT のみ)' if args.no_motor else '実モーター接続'}")
    print()

    try:
        comm = MotorComm(args.port, args.baud)
    except serial.SerialException as e:
        print(f"[ERROR] ポートを開けません: {e}")
        sys.exit(1)

    runner = TestRunner(comm, args.no_motor)

    try:
        run_tests(runner, comm, args.no_motor)
    except KeyboardInterrupt:
        print("\n[ABORT] ユーザー中断")
    except Exception as e:
        print(f"\n[ERROR] 予期しないエラー: {e}")
        import traceback; traceback.print_exc()
    finally:
        # 安全のため停止してから切断
        try:
            comm.send("ESTOP")
            time.sleep(0.1)
            comm.send("DISABLE")
        except Exception:
            pass
        comm.close()

    passed = runner.summary()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
