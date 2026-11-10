#!/usr/bin/env python3
"""Stream synthetic sensor CSV through a Linux virtual serial port."""

import math
import os
import pty
import select
import sys
import time
import tty


def sensor_samples():
    """Generate demo readings with synthetic conversions."""
    start = time.monotonic()
    filtered = 600.0
    was_blocked = False
    count = 0
    while True:
        elapsed = time.monotonic() - start
        pot = round(511.5 * (1 + math.sin(elapsed * 0.5)))
        raw = round(600 + 200 * math.sin(elapsed * 0.8)
                    + 25 * math.sin(elapsed * 19))
        filtered += 0.15 * (raw - filtered)
        bend = max(0, min(90, (filtered - 375) * 90 / 450))
        distance = round(350 + 250 * math.sin(elapsed * 0.4))
        if int(elapsed) % 12 == 11:
            distance = -1
        blocked = int(elapsed // 2) % 2 == 1
        if blocked and not was_blocked:
            count += 1
        was_blocked = blocked
        yield (f'POT,{pot},{pot * 180 / 1023:.2f}\n'
               f'FLEX,{raw},{filtered:.2f},{bend:.2f}\n'
               f'TOF_MM,{distance}\n'
               f'SLOT,{int(blocked)},{count}\n')


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    port = os.ttyname(slave)
    print(f'Fake Arduino port: {port}', flush=True)
    print(f'ros2 run arduino_bridge arduino_bridge.py --ros-args -p port:={port}')
    print('Sensors stream at about 100 Hz; TOF is invalid for 1 s every 12 s.')
    print('Type b + Enter to simulate the enable button; q + Enter to quit.')
    print('PC commands are printed locally; motors are not simulated.')

    samples = sensor_samples()
    buffer = b''
    enabled = False
    next_enable_report = time.monotonic()
    next_sample = time.monotonic()
    inputs = [master]
    if sys.stdin.isatty():
        inputs.append(sys.stdin)
    try:
        while True:
            ready, _, _ = select.select(
                inputs, [], [], max(0, next_sample - time.monotonic()))
            if master in ready:
                buffer += os.read(master, 4096)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    command = line.decode(errors='replace').strip()
                    if command:
                        print(f'PC -> Uno: {command}', flush=True)
                    if command in ('ENABLE,0', 'ENABLE,1'):
                        enabled = command == 'ENABLE,1'
                        next_enable_report = 0
            if sys.stdin in ready:
                key = sys.stdin.readline().strip().lower()
                if key == 'q':
                    break
                if key == 'b':
                    enabled = not enabled
                    try:
                        os.write(master, f'ENABLE,{int(enabled)}\n'.encode())
                    except BlockingIOError:
                        pass
            if time.monotonic() >= next_sample:
                try:
                    os.write(master, next(samples).encode())
                except BlockingIOError:
                    pass  # Skip this sample if nobody is draining the port.
                next_sample = time.monotonic() + 0.01
            if time.monotonic() >= next_enable_report:
                try:
                    os.write(master, f'ENABLE,{int(enabled)}\n'.encode())
                except BlockingIOError:
                    pass
                next_enable_report = time.monotonic() + 1
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
