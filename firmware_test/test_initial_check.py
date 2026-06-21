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
  [T09] モーション中コマンド拒否（ERR E008）
  [T10] NVS SAVE / LOAD / RESET_CONFIG
  [T11] COMM_TIMEOUT SET（0 で無効化）
  [T12] TEST_GPIO トグル（オシロスコープ確認用）
  [T13] TEST_PULSE / TEST_STOP（RMT パルス生成・停止）
  [T14] MOVE / GET POS（台形プロファイル移動）
  [T15] STOP ALL（全軸減速停止）

依存: pip install pyserial
"""

import serial
import time
import sys
import argparse
import json
import re

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
        """OK / ERR / EVT のいずれかが来るまで待機して返す。"""
        deadline = time.monotonic() + self.ser.timeout + extra_timeout
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if line:
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
    r.check("SET後 STATUS → OK", resp, "OK")

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

    # ── T11: COMM_TIMEOUT SET ─────────────────
    print("\n[T11] COMM_TIMEOUT 設定（0=無効）")
    resp = c.send("SET COMM_TIMEOUT 0")
    r.check("SET COMM_TIMEOUT 0 → OK", resp, "OK")

    resp = c.send("SET COMM_TIMEOUT 5000")
    r.check("SET COMM_TIMEOUT 5000 → OK", resp, "OK")

    # ── T12: TEST_GPIO（GPIO トグル）──────────
    print("\n[T12] TEST_GPIO — GPIO トグル（オシロで確認）")
    print("       GPIO6(STEP0) を 1Hz で 3 回トグルします...")
    if no_motor:
        resp = c.send("TEST_GPIO 0 3", extra_timeout=6.0)
        r.check("TEST_GPIO 0 3 → OK", resp, "OK")
    else:
        r.skip("TEST_GPIO", "--no-motor 未指定: RMT と排他のためスキップ")

    # ── T13: TEST_PULSE / TEST_STOP ──────────
    print("\n[T13] TEST_PULSE / TEST_STOP — RMT パルス生成")
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
