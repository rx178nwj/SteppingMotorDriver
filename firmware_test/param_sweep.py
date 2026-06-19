#!/usr/bin/env python3
"""
SteppingMotorDriver パラメータ最適化スクリプト
マイクロステップ・最大速度・加速度の最適値を組み合わせテストで決定する。

使用法:
  python param_sweep.py COM3          # 全スイープ
  python param_sweep.py COM3 --quick  # 代表値のみ
  python param_sweep.py COM3 --find-max-speed --microstep 32  # 最大速度探索

依存: pip install pyserial
"""

import serial
import time
import sys
import csv
import argparse
from dataclasses import dataclass, field
from typing import Optional, List
from datetime import datetime

# ────────────────────────────────────────────────────────────────────────────
#  テスト設定
# ────────────────────────────────────────────────────────────────────────────

AXIS = 0                  # テスト対象軸
TEST_STEPS = 3200         # 往復の片道ステップ数
IDLE_TIMEOUT_S = 30.0     # IDLE 待機タイムアウト [秒]
POLL_INTERVAL_S = 0.05    # 状態ポーリング間隔 [秒]

# --- フルスイープ用テストマトリクス ---
MICROSTEPS_ALL  = [1, 2, 4, 8, 16, 32]
VMAX_ALL        = [200, 500, 1000, 2000, 5000, 10000, 20000, 50000]  # steps/sec
ACCEL_ALL       = [200, 500, 1000, 2000, 5000, 10000]                 # steps/sec²

# --- クイックスイープ（代表値のみ） ---
MICROSTEPS_QUICK = [8, 16, 32]
VMAX_QUICK       = [500, 1000, 2000, 5000, 10000]
ACCEL_QUICK      = [500, 1000, 2000, 5000]

# ────────────────────────────────────────────────────────────────────────────
#  シリアル通信ラッパー
# ────────────────────────────────────────────────────────────────────────────

class MotorComm:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(1.5)  # CDC 安定待ち（再接続時のウォッチドッグ応答など）
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def flush(self):
        """受信バッファの残留データを捨てる（多行応答の余剰行など）。"""
        self.ser.reset_input_buffer()

    def _recv_line(self) -> str:
        """OK または ERR で始まる行が来るまで読み続ける。内部用。
        ESP_LOG 行・STATUS の継続行など OK/ERR 以外は全てスキップする。"""
        deadline = time.monotonic() + self.ser.timeout * 5
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if line.startswith("OK") or line.startswith("ERR"):
                return line
            # それ以外 (ESP_LOG, STATUS 継続行, EVT, 空行) はスキップ
        return ""

    def send(self, cmd: str) -> str:
        """バッファをフラッシュしてコマンドを送信、OK/ERR 応答行を返す。"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        return self._recv_line()

    def send_noflush(self, cmd: str) -> str:
        """バッファをフラッシュせずにコマンドを送信（wait_idle 内部用）。"""
        self.ser.write((cmd + "\n").encode())
        return self._recv_line()

    def send_ok(self, cmd: str) -> bool:
        resp = self.send(cmd)
        if not resp.startswith("OK"):
            print(f"  [WARN] cmd={cmd!r} resp={resp!r}")
            return False
        return True

    def get_state(self, axis: int = AXIS) -> str:
        resp = self.send(f"GET STATE {axis}")
        if resp.startswith("OK STATE "):
            return resp[9:]
        return "UNKNOWN"

    def get_pos(self, axis: int = AXIS) -> Optional[int]:
        resp = self.send(f"GET POS {axis}")
        if resp.startswith("OK POS "):
            try:
                return int(resp[7:])
            except ValueError:
                pass
        return None

    def wait_idle(self, axis: int = AXIS, timeout: float = IDLE_TIMEOUT_S) -> bool:
        """モーターが IDLE になるまでポーリング（バッファフラッシュなし）。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # send_noflush: ポーリング中に前のレスポンスを捨てない
            self.ser.write(f"GET STATE {axis}\n".encode())
            resp = self._recv_line()
            state = resp[9:] if resp.startswith("OK STATE ") else "UNKNOWN"
            if state in ("IDLE", "FAULT"):
                return state == "IDLE"
            time.sleep(POLL_INTERVAL_S)
        return False  # タイムアウト

    def ping(self, retries: int = 3) -> bool:
        for _ in range(retries):
            resp = self.send("PING")
            if resp == "OK PONG":
                return True
            time.sleep(0.3)
        return False

# ────────────────────────────────────────────────────────────────────────────
#  テスト実施
# ────────────────────────────────────────────────────────────────────────────

@dataclass
class TestResult:
    microstep: int
    vmax: int
    accel: int
    passed: bool
    pos_error: int = 0       # 往復後の残留位置誤差 [steps]
    elapsed_ms: float = 0.0  # 往復所要時間 [ms]
    note: str = ""


def apply_params(comm: MotorComm, microstep: int, vmax: int, accel: int) -> bool:
    ok = True
    ok &= comm.send_ok(f"SET MICROSTEP {microstep}")
    ok &= comm.send_ok(f"SET VMAX {AXIS} {vmax}")
    ok &= comm.send_ok(f"SET ACCEL {AXIS} {accel}")
    ok &= comm.send_ok(f"SET DECEL {AXIS} {accel}")
    return ok


def run_single_test(comm: MotorComm, microstep: int, vmax: int, accel: int,
                    steps: int = TEST_STEPS) -> TestResult:
    result = TestResult(microstep=microstep, vmax=vmax, accel=accel, passed=False)

    # 開始位置を記録（MOVETO 0 は行わず現在地を原点とする）
    origin = comm.get_pos()
    if origin is None:
        result.note = "GET POS failed"
        return result

    # パラメータ適用
    if not apply_params(comm, microstep, vmax, accel):
        result.note = "SET param failed"
        return result

    t0 = time.monotonic()

    # 正方向移動
    comm.send_ok(f"MOVE {AXIS} {steps}")
    if not comm.wait_idle():
        comm.send_ok("ESTOP")
        result.note = "forward move timeout (stall?)"
        # 開始位置へ復帰試行
        comm.send_ok(f"MOVETO {AXIS} {origin}")
        comm.wait_idle()
        return result

    # 逆方向移動（開始位置へ）
    comm.send_ok(f"MOVE {AXIS} -{steps}")
    if not comm.wait_idle():
        comm.send_ok("ESTOP")
        result.note = "return move timeout (stall?)"
        comm.send_ok(f"MOVETO {AXIS} {origin}")
        comm.wait_idle()
        return result

    elapsed_ms = (time.monotonic() - t0) * 1000.0

    # 位置誤差確認（開始位置との差分）
    final_pos = comm.get_pos()
    if final_pos is None:
        result.note = "final GET POS failed"
        return result

    result.pos_error = abs(final_pos - origin)
    result.elapsed_ms = elapsed_ms
    result.passed = (result.pos_error == 0)
    if not result.passed:
        result.note = f"pos error={final_pos - origin:+d} steps"
        # 開始位置へ復帰
        comm.send_ok(f"MOVETO {AXIS} {origin}")
        comm.wait_idle()
    return result


# ────────────────────────────────────────────────────────────────────────────
#  最大速度探索（バイナリサーチ）
# ────────────────────────────────────────────────────────────────────────────

def find_max_speed(comm: MotorComm, microstep: int, accel: int,
                   lo: int = 100, hi: int = 50000) -> int:
    """指定マイクロステップ・加速度で動作可能な最大速度をバイナリサーチで求める。"""
    print(f"\n  [MaxSpeed] microstep=1/{microstep} accel={accel} "
          f"search [{lo}..{hi}] steps/sec")
    best = lo
    while lo <= hi:
        mid = (lo + hi) // 2
        r = run_single_test(comm, microstep, mid, accel)
        status = "OK" if r.passed else "FAIL"
        print(f"    vmax={mid:6d}  {status}  err={r.pos_error}  {r.note}")
        if r.passed:
            best = mid
            lo = mid + 1
        else:
            hi = mid - 1
        time.sleep(0.2)
    return best


# ────────────────────────────────────────────────────────────────────────────
#  結果集計・表示
# ────────────────────────────────────────────────────────────────────────────

def print_summary(results: List[TestResult]):
    print("\n" + "=" * 72)
    print("テスト結果サマリー")
    print("=" * 72)
    print(f"{'ustep':>6} {'vmax':>7} {'accel':>7} {'結果':>5} "
          f"{'pos_err':>8} {'ms':>8}  note")
    print("-" * 72)
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"  1/{r.microstep:<3} {r.vmax:>7} {r.accel:>7} {status:>5} "
              f"{r.pos_error:>8} {r.elapsed_ms:>8.1f}  {r.note}")

    # PASS した組み合わせの中で最大速度を抽出
    passed = [r for r in results if r.passed]
    if passed:
        print("\n── 合格した組み合わせ（速度降順） ──")
        for r in sorted(passed, key=lambda x: -x.vmax)[:10]:
            print(f"  microstep=1/{r.microstep:<3}  vmax={r.vmax:>7}  "
                  f"accel={r.accel:>7}  time={r.elapsed_ms:.0f}ms")
        best = max(passed, key=lambda x: x.vmax)
        print(f"\n★ 推奨設定: microstep=1/{best.microstep}  "
              f"vmax={best.vmax}  accel={best.accel}")
    else:
        print("\n[警告] 全テスト失敗。速度・加速度を下げてください。")


def save_csv(results: List[TestResult], path: str):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["microstep", "vmax", "accel", "passed",
                    "pos_error", "elapsed_ms", "note"])
        for r in results:
            w.writerow([r.microstep, r.vmax, r.accel, r.passed,
                        r.pos_error, f"{r.elapsed_ms:.1f}", r.note])
    print(f"結果を保存しました: {path}")


# ────────────────────────────────────────────────────────────────────────────
#  メイン
# ────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SteppingMotorDriver パラメータスイープ")
    parser.add_argument("port", help="シリアルポート (例: COM3 / /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--axis", type=int, default=0, help="テスト軸 (0-3)")
    parser.add_argument("--quick", action="store_true", help="代表値のみテスト")
    parser.add_argument("--find-max-speed", action="store_true",
                        help="バイナリサーチで最大速度を探索")
    parser.add_argument("--microstep", type=int, default=None,
                        help="--find-max-speed 時のマイクロステップ")
    parser.add_argument("--accel", type=int, default=1000,
                        help="--find-max-speed 時の加速度")
    parser.add_argument("--vmax-min", type=int, default=100,
                        help="最大速度探索の下限 (デフォルト 100)")
    parser.add_argument("--vmax-max", type=int, default=200000,
                        help="最大速度探索の上限 (デフォルト 200000)")
    parser.add_argument("--steps", type=int, default=TEST_STEPS,
                        help="往復ステップ数 (デフォルト 3200)")
    parser.add_argument("--no-save", action="store_true", help="CSV を保存しない")
    args = parser.parse_args()

    global AXIS
    AXIS = args.axis

    print(f"接続中: {args.port} @ {args.baud} baud  軸={AXIS}")
    try:
        comm = MotorComm(args.port, args.baud)
    except serial.SerialException as e:
        print(f"[ERROR] シリアルポート接続失敗: {e}")
        sys.exit(1)

    if not comm.ping():
        print("[ERROR] PING 失敗。ポートとファームウェアを確認してください。")
        comm.close()
        sys.exit(1)
    print("PING OK")

    # ENABLE
    comm.send_ok("ENABLE")
    time.sleep(0.1)
    print(f"STATUS: {comm.send('STATUS')}")

    results: List[TestResult] = []

    try:
        if args.find_max_speed:
            # ── 最大速度探索モード ──
            targets = [args.microstep] if args.microstep else MICROSTEPS_ALL
            for ms in targets:
                vmax = find_max_speed(comm, ms, args.accel,
                                      lo=args.vmax_min, hi=args.vmax_max)
                print(f"  → microstep=1/{ms}  最大速度: {vmax} steps/sec")

        else:
            # ── スイープモード ──
            microsteps = MICROSTEPS_QUICK if args.quick else MICROSTEPS_ALL
            vmaxs      = VMAX_QUICK       if args.quick else VMAX_ALL
            accels     = ACCEL_QUICK      if args.quick else ACCEL_ALL

            total = len(microsteps) * len(vmaxs) * len(accels)
            print(f"\nスイープ開始: {total} 組合せ  steps={args.steps}")
            print("-" * 72)

            n = 0
            for ms in microsteps:
                for vm in vmaxs:
                    for ac in accels:
                        n += 1
                        r = run_single_test(comm, ms, vm, ac, steps=args.steps)
                        results.append(r)
                        status = "PASS" if r.passed else "FAIL"
                        print(f"[{n:3d}/{total}] 1/{ms:<3} vmax={vm:>7} "
                              f"accel={ac:>7} → {status}"
                              + (f"  {r.note}" if r.note else ""))
                        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n中断されました")
        comm.send_ok("ESTOP")
    finally:
        comm.send_ok("DISABLE")
        comm.close()

    if results:
        print_summary(results)
        if not args.no_save:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_csv(results, f"sweep_result_{ts}.csv")

    return 0


if __name__ == "__main__":
    sys.exit(main())
