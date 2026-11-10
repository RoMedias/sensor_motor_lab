# Sensor and motor lab

Arduino Uno ↔ USB serial ↔ ROS 2 bridge ↔ Flask GUI. The bridge only transports
newline-delimited strings on `/arduino/command` and `/arduino/data` using
`std_msgs/String`. Sensor-to-motor mapping runs on the Arduino.
All runnable Python files are in `ros_node/src/arduino_bridge/scripts/`:
`arduino_bridge.py`, `flask_gui.py`, and `fake_arduino.py`. The GUI template is
in `scripts/templates/index.html`.

## Run the ROS bridge and GUI

Source your ROS 2 installation, install the package dependencies (`rclpy`,
`std_msgs`, `python3-serial`, `python3-flask`), then build:

```bash
cd ros_node
colcon build --packages-select arduino_bridge
source install/setup.bash
ros2 run arduino_bridge arduino_bridge.py
```

Close Arduino Serial Monitor first. The bridge uses `/dev/ttyACM0` at 115200 baud.
Wait about two seconds after starting the bridge before sending commands because
opening the port can reset the Uno. There is no reconnect or acknowledgment logic.

In a second terminal, source ROS 2 and `ros_node/install/setup.bash`, then run:

```bash
ros2 run arduino_bridge flask_gui.py
```

Open <http://127.0.0.1:5000>, set GUI mode, and enable motors. To use the stepper,
restart the GUI with `--step-degrees VALUE`, replacing VALUE with the confirmed
degrees per driver pulse, including microstepping and gearing. It must match
Bill's rounding calculation. The GUI rounds half steps away from zero and sends
the achievable relative angle. Without this setting only stepper input is disabled.

The GUI displays pot degrees and raw ADC, flex bend degrees and raw/filtered ADC,
TOF millimeters (`-1` means unavailable), and slot state/interruption count.
Each sensor has a live graph covering the last 60 seconds: pot angle, flex raw
and filtered ADC on the same graph, TOF distance, and slot interruption count.
Graphs sample the current readings every 50 ms (20 Hz); invalid TOF values leave gaps.
Plot history stays in the browser and clears when the page reloads.
Raw potentiometer values remain available on `/arduino/data` for calibration plots.
The last GUI commands are requests, not measured motor positions or completion
reports. Servo limits are enforced by the servo module, so a requested angle can
be clamped there. Mode/enable selections are locally tracked; physical-button
`ENABLE` messages update the enable display. Use this GUI as the sole PC command
source. Restart the GUI after resetting the Uno, and select mode/enable again.

## Test sensors without hardware (Linux / WSL)

From the repository root, run:

```bash
python3 ros_node/src/arduino_bridge/scripts/fake_arduino.py
```

The script prints a virtual serial port and the bridge command to use it. After
rebuilding the ROS package and sourcing its setup file, run that command in a
second terminal, for example:

```bash
ros2 run arduino_bridge arduino_bridge.py --ros-args -p port:=/dev/pts/3
```

Use the port actually printed by the script. Start the Flask GUI as usual in
another terminal. Pot, flex, TOF, and slot readings stream at about 100 Hz. Flex includes
raw noise and a filtered value; TOF sends `-1` for one second every 12 seconds.
Slot interruption count increases on open → blocked transitions.

In the fake Arduino terminal, type `b` then Enter to simulate the enable button,
or `q` then Enter to quit. PC commands are printed there; the script sends no ACKs
or motor reports and does not simulate motor movement. Values and conversions
are synthetic test data, not measured calibration. No Arduino or ROS installation
is needed to run the fake script itself.

## Arduino integration

`firmware/main/main.ino` provides command parsing, mode/enable behavior, sensor
sampling, telemetry, and mapping calls. It starts in GUI mode with motors disabled.
Only enabled GUI mode accepts direct motor commands. Disabling cancels DC/stepper
motion and disables servo output. Changing modes cancels DC/stepper motion.
The debounced button toggles enable in either mode.

The sketch **cannot link or operate hardware yet**: teammate module implementations
are absent. Add their `.cpp`/`.ino` files to the sketch directory and include
`lab_modules.h` for shared types and declarations. The header follows
`function_sigs.txt`; do not define those structs twice. Each Begin function is
called from setup. Module reads, PID updates, and step generation must return
promptly; TOF reads must return the latest measurement or `-1`, without waiting
for a new conversion. `buttonUpdate()` must debounce and return true once per
press. `dcStop()` must cancel moves/reset PID; `stepperStop()` must cancel motion.

Sensor mode applies the following every 50 ms:

| Input | Motor command |
|---|---|
| Pot raw ADC | `pot_to_servo_angle()` → servo absolute angle |
| Calibrated flex bend | Clamped mapping → `stepperSetPosition()` absolute target |
| TOF millimeters | Clamped mapping → DC continuous RPM magnitude |
| Slot blocked/open | Opposite DC velocity signs |
| Debounced button | Enable/disable all motors |

Fill in the TOF and flex mapping endpoints and slot direction sign at the top of
`main.ino` after confirmation. They are deliberately unconfigured; missing values
stop the affected motor in SENSOR mode. Invalid TOF also stops the DC motor.
The stepper module must treat repeated absolute targets as the same target and
round relative commands to achievable steps. GUI DC position commands remain
relative; TOF never commands DC position.

Sensor data is sent approximately every 100 ms, one record at a time only when
the Uno's transmit buffer has room. Slot transitions and button presses are
polled every loop. Serial parsing uses an 80-byte buffer, discards oversized lines
through newline, and limits work per loop. No motor telemetry or ACKs are sent.

| Direction | CSV records (each terminated by an actual newline) |
|---|---|
| PC → Uno | `DC_MOVE,45`, `DC_VEL,-30`, `SERVO,90`, `STEPPER_MOVE,-45` |
| PC → Uno | `MODE,GUI`, `MODE,SENSOR`, `ENABLE,1`, `ENABLE,0` |
| Uno → PC | `POT,512,90`, `FLEX,620,605,35`, `TOF_MM,250`, `SLOT,1,12` |
| Uno → PC | `ENABLE,0` or `ENABLE,1` after a physical-button change |

## Remaining lab evidence

Confirm wiring in `PINOUT.md`, hardware limits, measured calibration constants,
step resolution, and module timing on the Uno. No physical motor integration has
been verified. The assignment also requires measured sensor conversion errors,
a transfer-function plot based on physical measurements, a raw/filtered comparison,
and a live unified demonstration. Real measurements are still needed.

Work split: Aaryan owns potentiometer/DC encoder/PID; Aswarth owns ROS, Flask,
serial and main-loop integration; Bill owns flex/stepper; Ji owns slot/servo;
Khushi owns TOF/button. Each teammate must contribute their module through their
own GitHub account as required by the assignment.

Run the GUI integration tests in a sourced ROS environment:

```bash
cd ros_node/src/arduino_bridge
python3 -m pytest test/test_gui.py
```
