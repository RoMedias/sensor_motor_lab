#!/usr/bin/env python3
"""Minimal ROS 2 <-> Arduino serial bridge.

After sourcing your ROS 2 installation, install pyserial (e.g. on Ubuntu:
    sudo apt install python3-serial
Run:
    python3 arduino_bridge.py --ros-args -p port:=/dev/ttyACM0

PC -> Arduino: std_msgs/String on /arduino/command, WITHOUT a newline:
    DC_MOVE,45       signed relative degrees
    DC_VEL,-30      signed RPM; zero stops
    SERVO,90        absolute degrees
    STEPPER_MOVE,-45 signed relative degrees
    MODE,GUI        or MODE,SENSOR
    ENABLE,1        or ENABLE,0

Arduino -> PC: std_msgs/String on /arduino/data, WITHOUT the newline:
    POT,512,90          raw ADC, degrees
    FLEX,620,605,35     raw ADC, filtered ADC, degrees
    TOF_MM,250          distance in mm; -1 if invalid
    SLOT,1,12           blocked, event count
    ENABLE,0           physical button changed enable state

Arduino must terminate each outgoing message with a real newline.
This bridge transports messages only. Sensor-to-motor mapping belongs in
your controller (ROS or Arduino); it is not implemented here.
Opening the port can reset an Uno: commands are rejected during startup.
No automatic reconnect or command replay. Restart after a disconnect.
"""

from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('startup_delay', 2.0)

        self.port = serial.Serial(
            port=self.get_parameter('port').value,
            baudrate=self.get_parameter('baud').value,
            timeout=0,
            write_timeout=0,
        )
        self.ready_at = time.monotonic() + max(
            0.0, float(self.get_parameter('startup_delay').value)
        )
        self.rx = bytearray()
        self.dropping_line = False
        self.tx = deque()
        self.data_pub = self.create_publisher(String, '/arduino/data', 10)
        self.command_sub = self.create_subscription(
            String, '/arduino/command', self.on_command, 10
        )
        self.timer = self.create_timer(0.01, self.poll_serial)
        self.get_logger().info(
            f'Opened {self.port.port} at {self.port.baudrate} baud. '
            'Commands: /arduino/command; readings: /arduino/data'
        )

    def on_command(self, msg):
        """Validate and queue exactly one command; never wait for serial I/O."""
        line = msg.data.strip()
        fields = line.split(',')
        if len(fields) != 2 or '\n' in line or '\r' in line:
            self.get_logger().warning('Expected one COMMAND,value message.')
            return

        command, value = fields
        valid = False
        if command == 'MODE':
            valid = value in ('GUI', 'SENSOR')
        elif command == 'ENABLE':
            valid = value in ('0', '1')
        elif command in ('DC_MOVE', 'DC_VEL', 'SERVO', 'STEPPER_MOVE'):
            try:
                valid = math.isfinite(float(value))
            except ValueError:
                pass
        if not valid or not line.isascii() or len(line) > 80:
            self.get_logger().warning(f'Invalid command: {line!r}')
            return
        if not self.port.is_open:
            self.get_logger().error('Serial disconnected; restart this node.')
            return
        if time.monotonic() < self.ready_at:
            self.get_logger().warning('Arduino starting; resend command shortly.')
            return
        if len(self.tx) >= 50:
            self.get_logger().warning('Serial command queue full; command rejected.')
            return
        self.tx.append((line + '\n').encode('ascii'))

    def poll_serial(self):
        """Handle partial writes and partial/multiple incoming lines."""
        try:
            # Process at most one outgoing command and 1024 input bytes per tick.
            if self.tx:
                sent = self.port.write(self.tx[0])
                self.tx[0] = self.tx[0][sent:]
                if not self.tx[0]:
                    self.tx.popleft()

            chunk = self.port.read(min(self.port.in_waiting, 1024))
            for byte in chunk:
                if byte == 10:  # Real newline, not the literal characters \\n.
                    if not self.dropping_line:
                        try:
                            line = self.rx.decode('ascii').strip()
                        except UnicodeDecodeError:
                            line = ''
                        if line:
                            self.data_pub.publish(String(data=line))
                    self.rx.clear()
                    self.dropping_line = False
                elif not self.dropping_line:
                    self.rx.append(byte)
                    if len(self.rx) > 256:
                        self.rx.clear()
                        self.dropping_line = True
                        self.get_logger().warning('Discarded oversized serial line.')
        except (serial.SerialException, OSError) as exc:
            # Do not retry a relative move: part of it might already be sent.
            self.get_logger().error(f'Serial error: {exc}. Restart this node.')
            self.tx.clear()
            self.timer.cancel()
            self.port.close()

    def destroy_node(self):
        self.port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
