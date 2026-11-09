# Serial protocol: Arduino ⇄ ROS 2 bridge

This is the contract between the Arduino firmware and the ROS 2 bridge. If the firmware follows it, the bridge publishes the data with no ROS code changes. That holds even when you add, remove or rename sensors.

- **Settings:** 115200 baud, ASCII text, one message per line, each line ending in `\n` (`Serial.println` is fine).
- **Units:** send sensor values already converted to physical units (cm, deg, N, °C, RPM…). The lab grades physical units, and the bridge does no conversion.

## Arduino → laptop

| Line | Meaning | When to send |
|---|---|---|
| `H,name1,name2,...` | Names of the telemetry fields, in order | Once in `setup()`, and again whenever the board receives `?` |
| `T,v1,v2,...` | Numbers, in the **same order and count** as `H` | Every loop at a fixed rate (≈20–50 Hz) |
| `A,text` | A command was accepted | After each command |
| `E,text` | A command was rejected (bad args, or in SENSOR mode) | Instead of `A` |

Each header name becomes a topic: `sensors/ir_cm` → `/lab/sensors/ir_cm` (`std_msgs/Float64`). Names may only contain letters, digits, `_` and `/`, and must start with a letter.

**Field names the bridge uses for the GUI.** Please use these exact names:

| Name | Value |
|---|---|
| `mode` | 0 = GUI control, 1 = SENSOR control |
| `button` | debounced switch state, 0/1 |
| `motors/servo_deg` | servo angle |
| `motors/dc_deg`, `motors/dc_rpm` | DC motor position and velocity (from the encoder) |
| `motors/stepper_deg` | stepper position |
| `motors/stepper_last_cmd_deg` | actual angle of the last stepper move after rounding to steps (the spec requires showing this) |

Name everything else however you like under `sensors/...` or `motors/...`. For the filter requirement, send the raw value and the filtered value as two fields, e.g. `sensors/ir_cm` and `sensors/ir_cm_filtered`. For the transfer-function plot, also send the raw ADC value, e.g. `sensors/ir_adc`.

Keep the line short: at 115200 baud, a 120-character `T` line at 50 Hz uses about half the link. Send 2 decimals at most, and drop to 20 Hz if you have many fields.

## Laptop → Arduino

| Command | Meaning |
|---|---|
| `?` | Resend the `H` line |
| `M 0` / `M 1` | Switch to GUI / SENSOR control |
| `SV <deg>` | Servo to an absolute angle |
| `DP <deg>` | DC motor: move `<deg>` relative to the current position (PID position) |
| `DV <rpm>` | DC motor: hold a signed velocity (PID velocity) |
| `DS` | DC motor: stop |
| `SR <deg>` | Stepper: move `<deg>` relative, rounded to the nearest step |
| `Z` | Zero the DC and stepper positions |
| `PID <kp> <ki> <kd>` | Set the DC motor gains |

In SENSOR mode, reply `E,ignored: SENSOR mode` to motor commands (`SV DP DV DS SR`).

## Minimal firmware skeleton

```cpp
const char HEADER[] = "H,mode,button,sensors/ir_cm,sensors/ir_cm_filtered,"
                      "motors/servo_deg,motors/dc_deg,motors/dc_rpm,"
                      "motors/stepper_deg,motors/stepper_last_cmd_deg";
char buf[48]; uint8_t len = 0;

void setup() { Serial.begin(115200); Serial.println(HEADER); }

void handleCommand(char *line) {
  if (line[0] == '?') { Serial.println(HEADER); return; }
  // parse "SV 90", "DP -45", ... with strtok/atof, then Serial.println("A,ok") or "E,..."
}

void loop() {
  while (Serial.available()) {            // non-blocking line reader
    char c = Serial.read();
    if (c == '\n') { buf[len] = 0; handleCommand(buf); len = 0; }
    else if (c != '\r' && len < sizeof(buf) - 1) buf[len++] = c;
  }
  static unsigned long last = 0;
  if (millis() - last >= 20) {            // 50 Hz telemetry
    last = millis();
    Serial.print("T,"); Serial.print(mode); Serial.print(',');
    Serial.print(button); Serial.print(',');
    Serial.print(irCm, 2); Serial.print(','); /* ...one value per header field... */
    Serial.println();
  }
}
```

Never use `delay()` or blocking `Serial.readString()` in `loop()`. Either one stalls the PID loop and the telemetry.
