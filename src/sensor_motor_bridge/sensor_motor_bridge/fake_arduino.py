"""Fake Arduino: speaks the lab serial protocol on a virtual serial port.

Lets you develop and demo the ROS 2 side with no hardware. Creates a pseudo
terminal and prints its path; point the bridge's `port` parameter at it.

    ros2 run sensor_motor_bridge fake_arduino            # prints e.g. /dev/pts/5
    ros2 run sensor_motor_bridge fake_arduino --link /tmp/fake_arduino

The sensor names here are placeholders. The real firmware can send whatever
fields it likes, as long as it follows PROTOCOL.md.
"""
import argparse
import math
import os
import pty
import random
import time
import tty

HEADER = [
    "mode", "button",
    "sensors/pot_adc", "sensors/pot_deg",
    "sensors/ir_cm", "sensors/ir_cm_filtered",
    "sensors/slot_count",
    "motors/servo_deg",
    "motors/dc_deg", "motors/dc_rpm", "motors/dc_target_deg", "motors/dc_pwm",
    "motors/stepper_deg", "motors/stepper_last_cmd_deg",
]
RATE_HZ = 50
STEP_DEG = 1.8 / 8  # EasyDriver default 1/8 microstepping


class Sim:
    def __init__(self):
        self.mode = 0
        self.servo = 90.0
        self.dc_deg = 0.0
        self.dc_rpm = 0.0
        self.dc_target = 0.0
        self.dc_vel_mode = False
        self.dc_vel_target = 0.0
        self.step_deg = 0.0
        self.step_target = 0.0
        self.step_last = 0.0
        self.ir_filt = 30.0
        self.slot_count = 0
        self.t0 = time.monotonic()

    def handle(self, line):
        parts = line.split()
        if not parts:
            return None
        cmd, args = parts[0].upper(), parts[1:]
        try:
            nums = [float(a) for a in args]
        except ValueError:
            return f"E,bad args: {line}"
        if cmd == "?":
            return "H," + ",".join(HEADER)
        if cmd == "M" and nums:
            self.mode = 1 if nums[0] else 0
            return f"A,mode {'SENSOR' if self.mode else 'GUI'}"
        if cmd == "PID" and len(nums) == 3:
            return f"A,pid {nums[0]} {nums[1]} {nums[2]}"
        if cmd == "Z":
            self.dc_deg = self.dc_target = self.step_deg = self.step_target = 0.0
            return "A,zeroed"
        if self.mode == 1 and cmd in ("SV", "DP", "DV", "DS", "SR"):
            return "E,ignored: SENSOR mode"
        if cmd == "SV" and nums:
            self.servo = max(0.0, min(180.0, nums[0]))
            return f"A,servo {self.servo:.1f}"
        if cmd == "DP" and nums:
            self.dc_vel_mode = False
            self.dc_target = self.dc_deg + nums[0]
            return f"A,dc pos {self.dc_target:.1f}"
        if cmd == "DV" and nums:
            self.dc_vel_mode = True
            self.dc_vel_target = max(-60.0, min(60.0, nums[0]))
            return f"A,dc vel {self.dc_vel_target:.1f}"
        if cmd == "DS":
            self.dc_vel_mode = False
            self.dc_target = self.dc_deg
            return "A,dc stop"
        if cmd == "SR" and nums:
            steps = round(nums[0] / STEP_DEG)
            self.step_last = steps * STEP_DEG
            self.step_target += self.step_last
            return f"A,stepper {self.step_last:.3f}"
        return f"E,unknown: {line}"

    def tick(self, dt):
        t = time.monotonic() - self.t0
        pot_deg = 135 + 120 * math.sin(t * 0.3)
        pot_adc = round(pot_deg / 270 * 1023)
        ir_true = 45 + 30 * math.sin(t * 0.2)
        ir_raw = ir_true + random.gauss(0, 2.0)
        self.ir_filt += 0.15 * (ir_raw - self.ir_filt)  # EMA, like the real filter would
        self.slot_count += random.random() < 0.05

        if self.mode == 1:  # sensor control: each motor driven by a different sensor
            self.servo = pot_deg * 180 / 270
            self.dc_vel_mode, self.dc_vel_target = True, (self.ir_filt - 45) * 2
            self.step_target = self.slot_count * 15.0

        if self.dc_vel_mode:
            rpm_cmd = self.dc_vel_target
        else:
            rpm_cmd = max(-60.0, min(60.0, 0.8 * (self.dc_target - self.dc_deg)))
        self.dc_rpm += (rpm_cmd - self.dc_rpm) * min(1.0, dt * 8)  # motor lag
        self.dc_deg += self.dc_rpm * 6 * dt
        pwm = round(max(-255, min(255, rpm_cmd * 4)))

        err = self.step_target - self.step_deg
        self.step_deg += max(-180 * dt, min(180 * dt, err))

        vals = [self.mode, 0, pot_adc, pot_deg, ir_raw, self.ir_filt, self.slot_count,
                self.servo, self.dc_deg, self.dc_rpm, self.dc_target, pwm,
                self.step_deg, self.step_last]
        return "T," + ",".join(f"{v:.2f}" if isinstance(v, float) else str(int(v))
                               for v in vals)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--link", help="also create this symlink to the pty, e.g. /tmp/fake_arduino")
    args, _ = ap.parse_known_args()  # ignore ROS args

    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if args.link:
        if os.path.islink(args.link):
            os.remove(args.link)
        os.symlink(path, args.link)
        path = f"{args.link} -> {path}"
    print(f"Fake Arduino on {path}", flush=True)

    sim, rx = Sim(), b""
    out = ["H," + ",".join(HEADER)]  # like the real board announcing on boot
    period = 1.0 / RATE_HZ
    last = time.monotonic()
    try:
        while True:
            try:
                rx += os.read(master, 1024)
            except (BlockingIOError, OSError):
                pass
            while b"\n" in rx:
                line, rx = rx.split(b"\n", 1)
                reply = sim.handle(line.decode("ascii", "ignore").strip())
                if reply:
                    print(f"<- {line.decode(errors='ignore').strip():20s} -> {reply}", flush=True)
                    out.append(reply)
            now = time.monotonic()
            if now - last >= period:
                out.append(sim.tick(now - last))
                last = now
                for msg in out:
                    try:
                        os.write(master, (msg + "\r\n").encode())
                    except (BlockingIOError, OSError):
                        pass  # nobody reading yet; drop, like a real UART
                out.clear()
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.remove(args.link)


if __name__ == "__main__":
    main()
