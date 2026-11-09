"""ROS 2 <-> Arduino serial bridge for the Sensors & Motor Control Lab.

Sensor-agnostic: every field the firmware lists in its H line is published as
std_msgs/Float64 on /lab/<field name>. For example, a header field
"sensors/ir_cm" becomes the topic /lab/sensors/ir_cm. Adding a sensor on the
Arduino needs no change here.

Extra topics derived from well-known field names (if the firmware sends them):
  /lab/mode_name             std_msgs/String  "GUI" | "SENSOR"     (from "mode")
  /lab/motors/joint_states   sensor_msgs/JointState, rad and rad/s
                             (from motors/servo_deg, motors/dc_deg, motors/dc_rpm,
                              motors/stepper_deg)
  /lab/status                std_msgs/String  acks / errors from the board
  /lab/link_ok               std_msgs/Bool    false if no telemetry for 1 s

Subscribes (commands, forwarded to the board):
  /lab/cmd/sensor_mode      std_msgs/Bool     true = SENSOR, false = GUI
  /lab/cmd/servo_deg        std_msgs/Float64  absolute angle
  /lab/cmd/dc_rel_deg       std_msgs/Float64  relative to current position
  /lab/cmd/dc_vel_rpm       std_msgs/Float64  signed
  /lab/cmd/dc_stop          std_msgs/Empty
  /lab/cmd/stepper_rel_deg  std_msgs/Float64  relative, board rounds to steps
  /lab/cmd/zero             std_msgs/Empty
  /lab/cmd/pid              geometry_msgs/Vector3 (x=kp, y=ki, z=kd)
  /lab/cmd/raw              std_msgs/String   passthrough, e.g. "SV 45"
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

import serial

from . import protocol as P

MODE_NAMES = {0: "GUI", 1: "SENSOR"}
JOINTS = [  # (joint name, position field, velocity field or None, vel scale to rad/s)
    ("servo", "motors/servo_deg", None, 0.0),
    ("dc", "motors/dc_deg", "motors/dc_rpm", 2 * math.pi / 60),
    ("stepper", "motors/stepper_deg", None, 0.0),
]


class LabBridge(Node):
    def __init__(self):
        super().__init__("sensor_motor_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.ser = None
        self.port, self.baud = port, baud
        self.rx = bytearray()
        self.header = None
        self.last_rx = 0.0
        self.last_header_req = 0.0

        self._pubs = {}
        self.js_pub = self.create_publisher(JointState, "/lab/motors/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/lab/status", 10)
        self.link_pub = self.create_publisher(Bool, "/lab/link_ok", 10)

        sub = self.create_subscription
        sub(Bool, "/lab/cmd/sensor_mode", lambda m: self.send(P.cmd_mode(m.data)), 10)
        sub(Float64, "/lab/cmd/servo_deg", lambda m: self.send(P.cmd_servo(m.data)), 10)
        sub(Float64, "/lab/cmd/dc_rel_deg", lambda m: self.send(P.cmd_dc_rel(m.data)), 10)
        sub(Float64, "/lab/cmd/dc_vel_rpm", lambda m: self.send(P.cmd_dc_vel(m.data)), 10)
        sub(Empty, "/lab/cmd/dc_stop", lambda m: self.send(P.cmd_dc_stop()), 10)
        sub(Float64, "/lab/cmd/stepper_rel_deg",
            lambda m: self.send(P.cmd_stepper_rel(m.data)), 10)
        sub(Empty, "/lab/cmd/zero", lambda m: self.send(P.cmd_zero()), 10)
        sub(Vector3, "/lab/cmd/pid", lambda m: self.send(P.cmd_pid(m.x, m.y, m.z)), 10)
        sub(String, "/lab/cmd/raw", lambda m: self.send(m.data.strip() + "\n"), 10)

        self.create_timer(0.005, self.poll_serial)  # 200 Hz poll
        self.create_timer(0.5, self.watchdog)
        self.open_serial()

    # ---------------- serial link ----------------
    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.rx.clear()
            self.header = None
            self.get_logger().info(f"Opened {self.port} @ {self.baud}")
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f"Cannot open {self.port}: {e} (retrying)",
                                   throttle_duration_sec=5.0)

    def close_serial(self, why):
        self.get_logger().error(f"Serial link lost: {why}")
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send(self, text):
        if self.ser is None:
            self.get_logger().warn(f"Not connected, dropped: {text.strip()}")
            return
        self.get_logger().info(f"-> {text.strip()}")
        try:
            self.ser.write(text.encode("ascii"))
        except (serial.SerialException, OSError) as e:
            self.close_serial(e)

    def poll_serial(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.rx += self.ser.read(n)
        except (serial.SerialException, OSError) as e:
            self.close_serial(e)
            return
        if len(self.rx) > 4096 and b"\n" not in self.rx:
            self.rx.clear()  # garbage with no line endings (e.g. wrong baud)
        while b"\n" in self.rx:
            line, _, rest = self.rx.partition(b"\n")
            self.rx = bytearray(rest)
            self.handle_line(line.decode("ascii", errors="ignore"))

    def handle_line(self, line):
        kind, data = P.parse_line(line)
        if kind == "H":
            if data != self.header:
                self.get_logger().info(f"Header: {len(data)} fields: {', '.join(data)}")
            self.header = data
        elif kind == "T":
            d = P.decode_telemetry(self.header, data)
            if d is None:
                self.request_header()
                return
            self.last_rx = time.monotonic()
            self.publish_telemetry(d)
        elif kind in ("A", "E"):
            self.status_pub.publish(String(data=f"{kind}: {data}"))
            # separate call sites: rclpy forbids one call site logging at two severities
            if kind == "E":
                self.get_logger().warn(f"board: {data}")
            else:
                self.get_logger().info(f"board: {data}")

    def request_header(self):
        # Telemetry without a matching header: ask again, at most twice a second.
        now = time.monotonic()
        if now - self.last_header_req > 0.5:
            self.last_header_req = now
            self.send(P.cmd_request_header())

    def watchdog(self):
        if self.ser is None:
            self.open_serial()
        ok = self.ser is not None and time.monotonic() - self.last_rx < 1.0
        self.link_pub.publish(Bool(data=ok))

    # ---------------- publishing ----------------
    def publish_telemetry(self, d):
        for name, value in d.items():
            topic = f"/lab/{name}"
            if topic not in self._pubs:
                self._pubs[topic] = self.create_publisher(Float64, topic, 10)
            self._pubs[topic].publish(Float64(data=value))

        if "mode" in d:
            self.pub_string("/lab/mode_name", MODE_NAMES.get(int(d["mode"]), "?"))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for joint, pos_key, vel_key, scale in JOINTS:
            if pos_key in d:
                js.name.append(joint)
                js.position.append(math.radians(d[pos_key]))
                js.velocity.append(d[vel_key] * scale if vel_key in d else 0.0)
        if js.name:
            self.js_pub.publish(js)

    def pub_string(self, topic, text):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(String, topic, 10)
        self._pubs[topic].publish(String(data=text))


def main():
    rclpy.init()
    node = LabBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
