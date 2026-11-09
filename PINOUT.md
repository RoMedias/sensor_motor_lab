# Arduino Uno pin layout

| Pin | Connects to | Why this pin |
|---|---|---|
| D0, D1 | **Leave empty.** USB serial to the laptop (ROS 2 bridge) | Anything wired here breaks uploads and telemetry |
| D2 | DC motor encoder **A** | One of only two hardware-interrupt pins (INT0) |
| D3 | DC motor encoder **B** | The other interrupt pin (INT1); both channels give 4× quadrature counts |
| D4 | Pushbutton (other leg to GND), `INPUT_PULLUP` | Debounced mode switch; no external resistor needed |
| D5 | Spare digital sensor (e.g. TSOP IR receiver, ultrasonic) | Free |
| D6 | L298 **enable** (PWM speed) | Timer0 PWM; still works while the Servo library is in use |
| D7 | L298 **input 1** (direction) | |
| D8 | L298 **input 2** (direction) | |
| D9 | Servo signal | Servo library uses Timer1. This **disables PWM on D9 and D10**, so the DC motor must not use them |
| D10 | Slot sensor output (spare digital) | Plain digital input is fine here; only PWM is lost |
| D11 | EasyDriver **STEP** | |
| D12 | EasyDriver **DIR** | |
| D13 | Onboard LED (status) | |
| A0 | Analog sensor 1 (e.g. potentiometer wiper) | |
| A1 | Analog sensor 2 (e.g. Sharp IR, FSR divider, thermistor divider) | |
| A2, A3 | Spare analog sensors | |
| A4 / A5 | I2C SDA / SCL, **only if you use a TOF sensor** | Hardware I2C lives only on these pins |

## Power and grounds

- **DC motor:** bench supply (12 V) → L298 motor power. **Stepper:** bench supply → EasyDriver `M+`. **Servo:** a separate 5–6 V supply. Don't power any motor from the Arduino's 5 V pin.
- **Tie every supply's GND to the Arduino GND.** If you skip this, nothing works reliably.
- **Encoder:** powered from the Arduino 5 V/GND. Put a 4.7–10 kΩ pull-up from each of A and B to 5 V, as the lab notes require.
- **EasyDriver:** set the current-limit pot **before** connecting the stepper. MS1/MS2 left unconnected = 1/8 microstepping = 0.225°/step, which is what the bridge's fake board assumes. Stepper wires: A=red, C=blue, B=black, D=green (the datasheet is wrong).
- **Analog dividers:** FSR or thermistor + 10 kΩ fixed resistor between 5 V and GND, with the midpoint going to the A-pin.

## Paste into the firmware

```cpp
// pins.h
#define PIN_ENC_A      2
#define PIN_ENC_B      3
#define PIN_BUTTON     4
#define PIN_SPARE_DIG  5
#define PIN_DC_EN      6   // PWM
#define PIN_DC_IN1     7
#define PIN_DC_IN2     8
#define PIN_SERVO      9
#define PIN_SLOT       10
#define PIN_STEP       11
#define PIN_DIR        12
#define PIN_ANALOG_1   A0
#define PIN_ANALOG_2   A1
```
