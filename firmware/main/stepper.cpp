// Hardware: SM-42BYG011-25 (1.8 deg/step) on a SparkFun EasyDriver
//   1/8 microstepping (MS1/MS2 unconnected) -> 0.225 deg/step
//   Wires: A=red, C=blue, B=black, D=green (the datasheet is wrong)
//   Set the EasyDriver current-limit pot BEFORE connecting the motor
// Library: AccelStepper (Arduino Library Manager)

#include <Arduino.h>
#include <AccelStepper.h>
#include "function_sigs.h"

static const uint8_t STEPPER_STEP_PIN = 11;
static const uint8_t STEPPER_DIR_PIN = 12;

// MOTOR SETTINGS
// full step / microsteps
static const float STEPPER_DEG_PER_STEP = 1.8f / 8.0f;

// steps/s (1 rev/s)
static const float STEPPER_MAX_SPEED = 1600.0f;

// steps/s^2
static const float STEPPER_ACCEL = 3200.0f;

// SENSOR mode: ignore target changes < 4 steps (0.9 deg) so noise doesn't cause jitter
static const long STEPPER_DEADBAND_STEPS = 4;

// STEPPER STATE
static AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
static float lastMoveDeg = 0.0f;

void stepperBegin() {
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);

    // power-on position = 0 deg
    stepper.setCurrentPosition(0);
}

// GUI mode: signed relative move, returns the achievable move after step rounding (e.g. 10 -> 9.900)
float stepperMoveRelative(float delta_deg) {
    long steps = lround(delta_deg / STEPPER_DEG_PER_STEP);
    stepper.moveTo(stepper.targetPosition() + steps);
    lastMoveDeg = steps * STEPPER_DEG_PER_STEP;
    return lastMoveDeg;
}

// SENSOR mode: absolute target in degrees from home repeated or tiny changes are ignored (deadband)
void stepperSetPosition(float target_deg) {
    long target = lround(target_deg / STEPPER_DEG_PER_STEP);

    if (labs(target - stepper.targetPosition()) < STEPPER_DEADBAND_STEPS) return;

    stepper.moveTo(target);
}

void stepperUpdate() {
    stepper.run();
}

void stepperStop() {
    // target = here, speed = 0
    stepper.setCurrentPosition(stepper.currentPosition());
}

float stepperGetPositionDeg() {
    return stepper.currentPosition() * STEPPER_DEG_PER_STEP;
}

float stepperGetLastMoveDeg() {
    return lastMoveDeg;
}

void stepperZero() {
    stepper.setCurrentPosition(0);
}
