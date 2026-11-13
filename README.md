# Sensor and motor lab

Arduino Uno ↔ USB serial ↔ ROS 2 bridge ↔ Flask GUI. The bridge only transports
newline-delimited strings on `/arduino/command` and `/arduino/data` using
`std_msgs/String`. Sensor-to-motor mapping runs on the Arduino.
All runnable Python files are in `ros_node/src/arduino_bridge/scripts/`:
`arduino_bridge.py`, `flask_gui.py`, and `fake_arduino.py`. The GUI template is
in `scripts/templates/index.html`, with styling in `scripts/static/dashboard.css`.

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
**Reset DC motor** calls the existing `dcStop()` function: it stops the DC motor,
cancels its current command, and resets PID state. It does not zero the encoder.
Reset uses the same enabled GUI mode requirement as the other motor commands.

The GUI displays pot degrees and raw ADC, flex bend degrees and raw/filtered ADC,
TOF millimeters (`-1` means unavailable), and slot state/interruption count.
Each sensor has a live graph covering the last 60 seconds: pot angle, flex raw
and filtered ADC on the same graph, TOF distance, and slot interruption count.
Graphs request the current readings every 10 ms (100 Hz target); invalid TOF
values leave gaps. Actual refresh rate depends on browser and request latency.
The fake Arduino and real Uno telemetry both target 100 Hz. The real TOF sensor
keeps its 50 ms measurement timing (about 20 Hz); intervening packets repeat its
latest available reading.
Plot history stays in the browser and clears when the page reloads.
Raw potentiometer values remain available on `/arduino/data` for calibration plots.
The last GUI commands are requests, not measured motor positions or completion
reports. Servo limits are enforced by the servo module, so a requested angle can
be clamped there. Mode is locally tracked; GUI enable requests update the display,
and Arduino `ENABLE` reports synchronize it at startup, after enable changes, and
once per second. A green/red status indicator and outlined enable/disable button
reflect the enable state, including physical-button presses. Use this GUI as the sole PC command
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

Sensor mode applies the following every 10 ms:

| Input | Motor command |
|---|---|
| Pot raw ADC | `pot_to_servo_angle()` → servo absolute angle |
| Calibrated flex bend | Clamped mapping → `stepperSetPosition()` absolute target |
| TOF millimeters | 9–345 mm → 0–100% DC power; outside this range stops DC |
| Slot blocked/open | Opposite DC velocity signs |
| Debounced button | Enable/disable all motors |

TOF endpoints use the observed near/far hand positions. Near is stopped and far
is full PWM power, so SENSOR mode does not assume an unmeasured maximum RPM.
Blocked slot means positive rotation; open means negative. Invalid or out-of-range
TOF stops the DC motor. GUI velocity commands remain encoder PID in signed RPM.
DC encoder feedback uses pin-change interrupts on A2/A3, with the same quadrature
direction as the component test. Counts/output revolution is calibrated to 361.4
from 3,614 encoder counts over 10 observed shaft turns on 2026-11-11. Temporary
encoder telemetry is disabled after calibration. PID gains still need tuning on
the actual motor.
DC position and velocity control currently use P only. At a position target the
L298 brakes instead of coasting; disabling motors releases the bridge outputs.
Very low RPM may still stall without measured friction compensation.
The stepper module must treat repeated absolute targets as the same target and
round relative commands to achievable steps. GUI DC position commands remain
relative; TOF never commands DC position.

Sensor sampling and telemetry cycles target 100 Hz (every 10 ms) at 115200 baud.
The four sensor records are sent one at a time only when the Uno's transmit
buffer has room. A cycle is timed from its start, and delayed cycles are not
replayed in a burst. Actual rates depend on loop timing and serial capacity.
TOF measurement timing remains 50 ms; telemetry repeats the latest reading
between measurements. Slot transitions and button presses are
polled every loop. Serial parsing uses an 80-byte buffer, discards oversized lines
through newline, and limits work per loop. No motor telemetry or ACKs are sent.

| Direction | CSV records (each terminated by an actual newline) |
|---|---|
| PC → Uno | `DC_MOVE,45`, `DC_VEL,-30`, `SERVO,90`, `STEPPER_MOVE,-45` |
| PC → Uno | `MODE,GUI`, `MODE,SENSOR`, `ENABLE,1`, `ENABLE,0` |
| PC → Uno | `DC_RESET,0` (stop DC motor, cancel motion, and reset PID) |
| Uno → PC | `POT,512,90`, `FLEX,620,605,35`, `TOF_MM,250`, `SLOT,1,12` |
| Uno → PC | `ENABLE,0` or `ENABLE,1` at startup, after enable changes, and once per second |

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

## Individual contributions

| Member | Contributions |
|---|---|
| Aaryan | Potentiometer module, DC motor + encoder module and PID, component test sketch, `PINOUT.md`, pot transfer-function plot |
| Aswarth | ROS 2 serial bridge, Flask GUI, fake Arduino, function signatures and `main.ino` integration, DC encoder rework and calibration, flex calibration |
| Bill | Flex sensor and stepper modules, flex/stepper limits, initial ROS bridge and protocol spec |
| Jihyun | Slot sensor and servo modules, wiring diagram, and electrical schematic |
| Khushi | TOF sensor and button modules |

Everyone contributed to hardware assembly and wiring.

## Issues faced

| Problem | Fix |
|---|---|
| Wiring was messy, and much of the work was troubleshooting connections | Tidied the wiring with an extra breadboard and organized it with tape. Checked each component on its own with `test_components/test_components.ino` and kept `PINOUT.md` matched to the actual wiring |
| The DC motor was wired incorrectly, so the encoder returned no data | Rewired the motor and encoder, then confirmed the counts with the component test sketch |
| Flex sensor readings were noisy | A 5-sample median filter followed by an EMA (alpha = 0.2) at 100 Hz, plus a two-point calibration taken from median readings at 0 and 90 deg |
| The original DC code polled the encoder once per loop, which can miss edges when the loop is busy with serial and sensor work. Its direction sign was also opposite to the test sketch, and the CPR was a placeholder | Pin-change interrupt with 4x quadrature decoding, the sign matched to the test sketch, and CPR calibrated to 361.4 |
| RPM measured over a 10 ms window was noisy because of count quantization | Encoder counts averaged over a 100 ms window before the EMA |
| PID tuning was tedious with no good way to measure angles or partial rotations | CPR calibrated by counting 10 full output-shaft turns; control reduced to P-only, with the L298 braking at the position target |
| Maximum DC RPM was never measured, so TOF could not map to an RPM | TOF maps to PWM duty (0-100%) instead of RPM |
| The TOF sensor measures at about 20 Hz, while telemetry runs at 100 Hz | Telemetry repeats the latest TOF reading between measurements |
| 100 Hz telemetry at 115200 baud could overrun the serial buffer | Each record is sent only when the Uno's transmit buffer has room, and delayed cycles are not replayed |
