# Arduino Uno pin layout

Source of truth: `test_components.ino`. If a pin changes, update that sketch and this file together.

| Pin | Connects to | Mode | Notes |
|---|---|---|---|
| D0, D1 | **Leave empty.** USB serial to the laptop (ROS 2 bridge) | — | Anything wired here breaks uploads and telemetry |
| D2, D3 | Spare | — | Free (hardware-interrupt pins, currently unused) |
| D4 | Omron EE-SX1042 slot sensor collector (Ji) | `INPUT_PULLUP` | HIGH = blocked, LOW = clear |
| D5 | L298 **E1-2 enable** (PWM speed) (Aaryan) | `OUTPUT`, `analogWrite` | Timer0 PWM; still works while the Servo library is in use |
| D6 | EasyDriver **STEP** (Bill) | `OUTPUT` | |
| D7 | EasyDriver **DIR** (Bill) | `OUTPUT` | |
| D8 | L298 **I1** (direction) (Aaryan) | `OUTPUT` | Forward: I1=HIGH, I2=LOW |
| D9 | Servo signal (Ji) | Servo library | Servo library uses Timer1. This **disables PWM on D9 and D10**, so the DC motor must not use them |
| D10 | Spare | — | Digital only (no PWM while the servo is attached) |
| D11 | L298 **I2** (direction) (Aaryan) | `OUTPUT` | Reverse: I1=LOW, I2=HIGH; both LOW = stop |
| D12 | Pushbutton, other leg to GND (Khushi) | `INPUT_PULLUP` | Pressed = LOW; debounced in software |
| D13 | Onboard LED (status) | — | |
| A0 | Potentiometer wiper (Aaryan) | analog in | |
| A1 | Flex sensor (Bill) | `INPUT_PULLUP` | See flex note below |
| A2 | DC motor encoder **A** (Aaryan) | `INPUT_PULLUP` | PC2 / PCINT10, pin-change interrupt |
| A3 | DC motor encoder **B** (Aaryan) | `INPUT_PULLUP` | PC3 / PCINT11, pin-change interrupt; A+B give 4× quadrature counts |
| A4 / A5 | VL53L1X TOF I2C SDA / SCL (Khushi) | `Wire` | Hardware I2C lives only on these pins; default address 0x29 |

## Encoder interrupt

A2/A3 share the port-C pin-change vector `PCINT1_vect` (enabled via `PCMSK1` bits
PCINT10/PCINT11 and `PCICR` bit PCIE1). Only one `ISR(PCINT1_vect)` may exist in
the firmware, so no other module may use pin-change interrupts on A0–A5.

## Power and grounds

- **DC motor:** bench supply (12 V) → L298 motor power. **Stepper:** bench supply → EasyDriver `M+`. **Servo:** a separate 5–6 V supply. Don't power any motor from the Arduino's 5 V pin.
- **Tie every supply's GND to the Arduino GND.** If you skip this, nothing works reliably.
- **Encoder:** powered from the Arduino 5 V/GND. The test sketch relies on the internal pull-ups (~20–50 kΩ); the lab notes recommend an external 4.7–10 kΩ pull-up from each of A and B to 5 V for sharp edges.
- **EasyDriver:** set the current-limit pot **before** connecting the stepper. MS1/MS2 left unconnected = 1/8 microstepping = **0.225°/step** (1600 steps/rev); `stepper.cpp` assumes this (`STEPPER_DEG_PER_STEP`). Stepper wires: A=red, C=blue, B=black, D=green (see the lab guidelines; the datasheet is wrong).
- **Flex sensor:** the test sketch enables the internal pull-up on A1. If the flex is also wired in an external 10 kΩ divider, the internal pull-up sits in parallel with the top leg and shifts readings; confirm the wiring and match `flex.cpp`'s `pinMode` to it.

## Paste into the firmware

```cpp
// pins.h
#define PIN_POT        A0
#define PIN_FLEX       A1
#define PIN_ENC_A      A2   // PCINT10
#define PIN_ENC_B      A3   // PCINT11
#define PIN_SLOT       4
#define PIN_DC_EN      5    // PWM
#define PIN_STEP       6
#define PIN_DIR        7
#define PIN_DC_IN1     8
#define PIN_SERVO      9
#define PIN_DC_IN2     11
#define PIN_BUTTON     12
// TOF: I2C on A4 (SDA) / A5 (SCL), via Wire
```
