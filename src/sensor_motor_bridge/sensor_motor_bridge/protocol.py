"""Serial line protocol between the Arduino firmware and the ROS 2 bridge.

Pure Python (no ROS) so it can be unit-tested without a ROS install.
See PROTOCOL.md at the package root for the contract the firmware must follow.

Arduino -> laptop, one ASCII line each, terminated by '\n':
  H,<name>,<name>,...     header: the names of the telemetry fields, in order
  T,<val>,<val>,...       telemetry: numbers, same order and count as the header
  A,<text>                ack for a command
  E,<text>                error / rejected command

Laptop -> Arduino:
  ?                       ask the board to resend its H line
  M 0|1                   control mode: 0 = GUI, 1 = SENSOR
  SV <deg>                servo absolute angle
  DP <deg>                DC motor: move <deg> relative to current position
  DV <rpm>                DC motor: hold signed velocity
  DS                      DC motor: stop
  SR <deg>                stepper: move <deg> relative (board rounds to steps)
  Z                       zero the DC and stepper position counters
  PID <kp> <ki> <kd>      DC motor gains
"""


def parse_line(line):
    """Return (kind, data) for one received line.

    ("H", [names])      header
    ("T", [floats])     telemetry values (caller zips with the header)
    ("A"|"E", text)     ack / error
    (None, None)        blank, garbage or corrupted line
    """
    line = line.strip()
    if len(line) < 2 or line[1] != ",":
        return None, None
    kind, body = line[0], line[2:]
    if kind == "H":
        names = [n.strip() for n in body.split(",")]
        if not names or any(not _valid_name(n) for n in names):
            return None, None
        return "H", names
    if kind == "T":
        try:
            return "T", [float(v) for v in body.split(",")]
        except ValueError:
            return None, None
    if kind in ("A", "E"):
        return kind, body
    return None, None


def _valid_name(name):
    # Becomes part of a ROS topic name: letters, digits, '_' and '/' only.
    return bool(name) and name[0].isalpha() and all(c.isalnum() or c in "_/" for c in name)


def decode_telemetry(header, values):
    """Zip a T line's values with the header. Returns None if the counts differ
    (truncated frame, or the firmware changed and we have a stale header)."""
    if header is None or len(values) != len(header):
        return None
    return dict(zip(header, values))


def cmd_request_header():     return "?\n"
def cmd_mode(sensor_mode):    return f"M {1 if sensor_mode else 0}\n"
def cmd_servo(deg):           return f"SV {deg:.2f}\n"
def cmd_dc_rel(deg):          return f"DP {deg:.2f}\n"
def cmd_dc_vel(rpm):          return f"DV {rpm:.2f}\n"
def cmd_dc_stop():            return "DS\n"
def cmd_stepper_rel(deg):     return f"SR {deg:.3f}\n"
def cmd_zero():               return "Z\n"
def cmd_pid(kp, ki, kd):      return f"PID {kp:.4f} {ki:.4f} {kd:.4f}\n"
