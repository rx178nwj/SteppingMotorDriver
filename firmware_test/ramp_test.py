#!/usr/bin/env python3
"""
手動ランプテスト — 速度を段階的に上げてモーターの脱調を耳・振動で確認する。

使用法:
  python ramp_test.py COM31
  python ramp_test.py COM31 --microstep 16 --start 500 --step 500 --max 30000

操作:
  Enter   → 次のステップへ (今の速度で問題なし)
  s       → この速度で停止・記録 (ここが上限)
  q       → テスト終了

依存: pip install pyserial
"""

import serial
import time
import sys
import argparse

AXIS = 0

class MotorComm:
    def __init__(self, port, baud=115200, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(1.5)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _recv_line(self):
        deadline = time.monotonic() + self.ser.timeout * 5
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if line.startswith("OK") or line.startswith("ERR"):
                return line
        return ""

    def send(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        return self._recv_line()

    def send_ok(self, cmd):
        return self.send(cmd).startswith("OK")

    def ping(self):
        for _ in range(3):
            if self.send("PING") == "OK PONG":
                return True
            time.sleep(0.3)
        return False


def main():
    parser = argparse.ArgumentParser(description="手動ランプテスト")
    parser.add_argument("port", help="シリアルポート (例: COM31)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--axis", type=int, default=0)
    parser.add_argument("--microstep", type=int, default=16,
                        help="マイクロステップ (デフォルト 16)")
    parser.add_argument("--accel", type=int, default=2000,
                        help="加速度 steps/sec² (デフォルト 2000)")
    parser.add_argument("--start", type=int, default=500,
                        help="開始速度 steps/sec (デフォルト 500)")
    parser.add_argument("--step", type=int, default=500,
                        help="速度増加ステップ steps/sec (デフォルト 500)")
    parser.add_argument("--max", type=int, default=20000,
                        help="上限速度 steps/sec (デフォルト 20000)")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="各速度の保持時間 [秒] (デフォルト 2.0)")
    args = parser.parse_args()

    global AXIS
    AXIS = args.axis

    print(f"接続中: {args.port} microstep=1/{args.microstep} accel={args.accel}")
    comm = MotorComm(args.port, args.baud)

    if not comm.ping():
        print("[ERROR] PING 失敗")
        comm.close()
        sys.exit(1)

    comm.send_ok("ENABLE")
    comm.send_ok(f"SET MICROSTEP {args.microstep}")
    comm.send_ok(f"SET ACCEL {AXIS} {args.accel}")
    comm.send_ok(f"SET DECEL {AXIS} {args.accel}")

    print()
    print("操作方法:")
    print("  Enter → 次の速度へ")
    print("  s     → この速度が上限として記録・停止")
    print("  q     → 終了")
    print()
    print("モーターを観察しながら脱調（異音・振動増加・停止）を確認してください。")
    print()

    best_speed = 0
    current_speed = args.start

    try:
        while current_speed <= args.max:
            comm.send_ok(f"SET VMAX {AXIS} {current_speed}")
            comm.send_ok(f"VEL {AXIS} {current_speed}")

            # 1/16 microstep, 200steps/rev モーターの場合の RPM
            rpm = current_speed / (200 * args.microstep) * 60
            print(f"  速度: {current_speed:>7} steps/sec  ({rpm:5.1f} RPM)  "
                  f"[Enter=次へ / s=ここが上限 / q=終了] ", end="", flush=True)

            time.sleep(args.hold)

            try:
                import msvcrt
                # Windows: ノンブロッキング入力
                key = ""
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    if msvcrt.kbhit():
                        key = msvcrt.getch().decode(errors="replace").lower()
                        break
                    time.sleep(0.05)
            except ImportError:
                key = input()

            if key in ("s", "s\n"):
                print(f"\n  ★ {current_speed} steps/sec を上限として記録")
                best_speed = current_speed
                break
            elif key in ("q", "q\n"):
                print("\n終了")
                break
            else:
                print("OK")
                best_speed = current_speed
                current_speed += args.step

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print(f"\nモーター停止中...")
        comm.send_ok(f"STOP {AXIS}")
        time.sleep(1.0)
        comm.send_ok("DISABLE")
        comm.close()

    if best_speed > 0:
        safe = int(best_speed * 0.8)
        print()
        print("=" * 50)
        print(f"確認済み最大速度: {best_speed} steps/sec")
        print(f"推奨 vmax (80%):  {safe} steps/sec")
        print()
        print("NVS に保存するコマンド:")
        print(f"  SET MICROSTEP {args.microstep}")
        print(f"  SET VMAX {AXIS} {safe}")
        print(f"  SET ACCEL {AXIS} {args.accel}")
        print(f"  SET DECEL {AXIS} {args.accel}")
        print(f"  SAVE")


if __name__ == "__main__":
    main()
