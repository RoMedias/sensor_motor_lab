# Arduino Uno pin layout

| Pin | Connects to | Why this pin |
|---|---|---|
| D0, D1 | **Leave empty.** USB serial to the laptop (ROS 2 bridge) | Anything wired here breaks uploads and telemetry |
| D2 | DC motor encoder **A** | One of only two hardware-interrupt pins (INT0) |
| D3 | DC motor encoder **B** | The other interrupt pin (INT1); both channels give 4× quadrature counts |
| D4 | Pushbutton (other leg to GND), `INPUT_PULLUP` (Khushi) | Debounced enable/disable switch; no external resistor needed |
| D5 | Spare | Free |
| D6 | L298 **enable** (PWM speed) (Aaryan) | Timer0 PWM; still works while the Servo library is in use |
| D7 | L298 **input 1** (direction) | |
| D8 | L298 **input 2** (direction) | |
| D9 | Servo signal (Ji) | Servo library uses Timer1. This **disables PWM on D9 and D10**, so the DC motor must not use them |
| D10 | Slot sensor output (Ji) | Plain digital input is fine here; only PWM is lost |
| D11 | EasyDriver **STEP** (Bill) | |
| D12 | EasyDriver **DIR** (Bill) | |
| D13 | Onboard LED (status) | |
| A0 | Potentiometer wiper (Aaryan) | |
| A1 | Flex sensor divider: 5V – flex – A1 – 10 kΩ – GND (Bill) | |
| A2, A3 | Spare | |
| A4 / A5 | TOF sensor I2C SDA / SCL (Khushi) | Hardware I2C lives only on these pins |

## Power and grounds

- **DC motor:** bench supply (12 V) → L298 motor power. **Stepper:** bench supply → EasyDriver `M+`. **Servo:** a separate 5–6 V supply. Don't power any motor from the Arduino's 5 V pin.
- **Tie every supply's GND to the Arduino GND.** If you skip this, nothing works reliably.
- **Encoder:** powered from the Arduino 5 V/GND. Put a 4.7–10 kΩ pull-up from each of A and B to 5 V, as the lab notes require.
- **EasyDriver:** set the current-limit pot **before** connecting the stepper. MS1/MS2 left unconnected = 1/8 microstepping = **0.225°/step**; `stepper.cpp` assumes this (`STEPPER_DEG_PER_STEP`). Stepper wires: A=red, C=blue, B=black, D=green (see the lab guidelines; the datasheet is wrong).
- **Flex divider:** flex sensor + 10 kΩ fixed resistor between 5 V and GND, with the midpoint going to A1.

## Paste into the firmware

```cpp
// pins.h
#define PIN_ENC_A      2
#define PIN_ENC_B      3
#define PIN_BUTTON     4
#define PIN_DC_EN      6   // PWM
#define PIN_DC_IN1     7
#define PIN_DC_IN2     8
#define PIN_SERVO      9
#define PIN_SLOT       10
#define PIN_STEP       11
#define PIN_DIR        12
#define PIN_POT        A0
#define PIN_FLEX       A1
// TOF: I2C on A4 (SDA) / A5 (SCL), via Wire
```
