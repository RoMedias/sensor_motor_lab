#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PORT = '/dev/ttyACM0'  # Change to your Arduino's port.
BAUD = 115200


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', PORT)
        port = self.get_parameter('port').value
        self.serial = serial.Serial(port, BAUD, timeout=0)
        self.buffer = b''

        self.publisher = self.create_publisher(String, '/arduino/data', 10)
        self.subscription = self.create_subscription(
            String, '/arduino/command', self.send_command, 10
        )
        self.timer = self.create_timer(0.01, self.read_serial)

    def send_command(self, msg):
        # Example ROS message: DC_MOVE,45
        self.serial.write((msg.data.strip() + '\n').encode())

    def read_serial(self):
        # Keep incomplete lines until the remaining bytes arrive.
        self.buffer += self.serial.read(self.serial.in_waiting)
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            text = line.decode(errors='replace').strip()
            if text:
                self.publisher.publish(String(data=text))


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
