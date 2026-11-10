#pragma once
// Team header. Every .cpp in this sketch folder implements part of it;
// main.ino calls the functions. Keep in sync with function_sigs.txt.

#include <Arduino.h>
void potBegin();
void dcBegin();
void flexBegin();
void stepperBegin();
void slotBegin();
void servoBegin();
void tofBegin();
void buttonBegin();
// ---------- Sensor return types ----------

struct PotReading {
    uint16_t raw_adc;
    float angle_deg;
};

struct FlexReading {
    uint16_t raw_adc;
    float filtered_adc;
    float angle_deg;
};

struct SlotReading {
    bool blocked;
    // Open → blocked transitions
    uint32_t count;
};


//
// Potentiometer
//

// Returns raw ADC and calibrated angle.
PotReading potRead();


//
// DC motor
//

// Starts a signed relative move in degrees — GUI mode.
void dcMoveRelative(float delta_deg);

// Runs continuously at signed RPM; 0 stops.
// GUI: user input. SENSOR: TOF speed + slot direction.
void dcSetVelocity(float rpm);

// Nonblocking encoder/PID update; call every loop.
void dcUpdate();

// Stops output, cancels movement, resets PID.
void dcStop();


//
// Flex sensor
//

// Reads sensor, updates filter, returns ADCs and bend angle.
FlexReading flexRead();


//
// Stepper motor
//

// Starts a signed relative move — GUI mode.
// Returns the achievable move in degrees after step rounding.
float stepperMoveRelative(float delta_deg);

// Sets an absolute target — SENSOR mode.
// Repeated identical targets do not accumulate movement.
void stepperSetPosition(float target_deg);

// Nonblocking step generation; call every loop.
void stepperUpdate();

// Stops stepping and cancels pending movement.
void stepperStop();

// Current shaft angle from home in degrees (for sendSensorData).
float stepperGetPositionDeg();

// Achievable size of the last stepperMoveRelative() in degrees
// (the spec requires displaying the actual angle moved).
float stepperGetLastMoveDeg();

// Declares the current position to be 0 deg (stops first).
void stepperZero();


//
// Slot sensor
//

// Reads slot state and updates interruption count.
SlotReading slotRead();


//
// Servo motor
//

// Maps potentiometer ADC to the servo's safe angle range.
float pot_to_servo_angle(uint16_t raw_adc);

// Commands servo angle, clamped to safe limits.
void set_servo_angle(float angle_deg);

// Enables servo output or releases it when disabled.
void servoSetEnabled(bool enabled);


//
// TOF sensor
//

// Returns distance in mm; -1 means invalid/unavailable.
int16_t tofReadMm();


//
// Button
//

// Updates debounce; returns true once per valid press.
// Main loop toggles system enable on true.
bool buttonUpdate();


//
// Mapping and system control
//

enum class ControlMode : uint8_t {
    GUI,
    SENSOR
};

// Maps valid TOF distance to bounded speed magnitude in RPM.
float distanceToRpm(int16_t distance_mm);

// Maps flex bend angle to a stepper target in degrees.
float bendToStepperDegrees(float bend_deg);

// Sets GUI or SENSOR control mode.
void setControlMode(ControlMode mode);

// Enables/disables all motors; disabling cancels active motion.
void setSystemEnabled(bool enabled);

// Applies sensor mappings while enabled in SENSOR mode.
void applySensorControl();

// Reads and dispatches complete serial commands without blocking.
void readSerialCommands();

// Sends sensor readings periodically without blocking.
void sendSensorData();



