#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lab_modules.h"

// Fill these from measured calibration and agreed motor limits.
// NAN / 0 leave the corresponding mapping unavailable.
const float TOF_NEAR_MM = NAN;
const float TOF_FAR_MM = NAN;
const float TOF_NEAR_RPM = NAN;
const float TOF_FAR_RPM = NAN;
const float FLEX_MIN_BEND_DEG = NAN;
const float FLEX_MAX_BEND_DEG = NAN;
const float STEPPER_MIN_TARGET_DEG = NAN;
const float STEPPER_MAX_TARGET_DEG = NAN;
const int8_t SLOT_BLOCKED_SIGN = 0;  // Confirm +1 or -1.

const uint32_t SENSOR_PERIOD_MS = 10;  // 100 Hz sampling and telemetry target.

ControlMode controlMode = ControlMode::GUI;
bool systemEnabled = false;
bool enableStatePending = true;
bool sensorsReady = false;
PotReading pot;
FlexReading flex;
SlotReading slot;
int16_t distanceMm = -1;

float mapClamped(float value, float inMin, float inMax,
                 float outMin, float outMax) {
    if (!isfinite(value) || !isfinite(inMin) || !isfinite(inMax) ||
        !isfinite(outMin) || !isfinite(outMax) || inMax <= inMin)
        return NAN;
    value = constrain(value, inMin, inMax);
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}

float distanceToRpm(int16_t distance_mm) {
    if (distance_mm < 0 || TOF_NEAR_RPM < 0 || TOF_FAR_RPM < 0)
        return NAN;
    return mapClamped(distance_mm, TOF_NEAR_MM, TOF_FAR_MM,
                      TOF_NEAR_RPM, TOF_FAR_RPM);
}

float bendToStepperDegrees(float bend_deg) {
    return mapClamped(bend_deg, FLEX_MIN_BEND_DEG, FLEX_MAX_BEND_DEG,
                      STEPPER_MIN_TARGET_DEG, STEPPER_MAX_TARGET_DEG);
}

void setControlMode(ControlMode mode) {
    if (mode == controlMode) return;
    dcStop();
    stepperStop();
    controlMode = mode;
}

void setSystemEnabled(bool enabled) {
    systemEnabled = enabled;
    enableStatePending = true;
    if (!enabled) {
        dcStop();
        stepperStop();
    }
    servoSetEnabled(enabled);
}

void applySensorControl() {
    if (!systemEnabled || controlMode != ControlMode::SENSOR || !sensorsReady)
        return;
    float servoAngle = pot_to_servo_angle(pot.raw_adc);
    if (isfinite(servoAngle)) set_servo_angle(servoAngle);
    float stepperTarget = bendToStepperDegrees(flex.angle_deg);
    if (isfinite(stepperTarget))
        stepperSetPosition(stepperTarget);  // Absolute target, never a relative move.
    else
        stepperStop();
    float rpm = distanceToRpm(distanceMm);
    if (!isfinite(rpm) || (SLOT_BLOCKED_SIGN != 1 && SLOT_BLOCKED_SIGN != -1))
        dcStop();
    else
        dcSetVelocity(rpm * (slot.blocked ? SLOT_BLOCKED_SIGN : -SLOT_BLOCKED_SIGN));
}

void parseCommand(char* message) {
    char* comma = strchr(message, ',');
    if (comma == nullptr) return;
    *comma = '\0';
    char* command = message;
    char* argument = comma + 1;
    if (strcmp(command, "MODE") == 0) {
        if (strcmp(argument, "GUI") == 0)
            setControlMode(ControlMode::GUI);
        else if (strcmp(argument, "SENSOR") == 0)
            setControlMode(ControlMode::SENSOR);
        return;
    }
    if (strcmp(command, "ENABLE") == 0) {
        if (strcmp(argument, "1") == 0)
            setSystemEnabled(true);
        else if (strcmp(argument, "0") == 0)
            setSystemEnabled(false);
        return;
    }
    char* end;
    float value = (float)strtod(argument, &end);
    if (end == argument || *end != '\0' || !isfinite(value))
        return;
    if (!systemEnabled || controlMode != ControlMode::GUI) return;
    if (strcmp(command, "DC_MOVE") == 0)
        dcMoveRelative(value);
    else if (strcmp(command, "DC_VEL") == 0)
        dcSetVelocity(value);
    else if (strcmp(command, "SERVO") == 0)
        set_servo_angle(value);
    else if (strcmp(command, "STEPPER_MOVE") == 0)
        stepperMoveRelative(value);
    else if (strcmp(command, "DC_RESET") == 0 && value == 0.0f)
        dcStop();  // Existing stop also cancels motion and resets the DC PID.
}

void readSerialCommands() {
    static char buffer[80];
    static uint8_t index = 0;
    static bool overflow = false;
    // Bound work per loop so serial traffic cannot starve motor updates.
    uint8_t remaining = sizeof(buffer);
    while (remaining-- > 0 && Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buffer[index] = '\0';
            if (!overflow) parseCommand(buffer);
            index = 0;
            overflow = false;
        } else if (!overflow) {
            if (index < sizeof(buffer) - 1)
                buffer[index++] = c;
            else
                overflow = true;
        }
    }
}

void sendSensorData() {
    // Each record fits the Uno's 63-byte TX space; never wait for it to drain.
    if (Serial.availableForWrite() < 63) return;
    static uint32_t lastEnableSent = 0;
    if (enableStatePending || millis() - lastEnableSent >= 1000) {
        Serial.print(F("ENABLE,"));
        Serial.println(systemEnabled ? 1 : 0);
        enableStatePending = false;
        lastEnableSent = millis();
        return;
    }
    static uint32_t lastSent = 0;
    static uint8_t field = 0;
    if (!sensorsReady) return;
    // Time between cycle starts, so sending four records adds no extra period.
    if (field == 0) {
        uint32_t now = millis();
        if (now - lastSent < SENSOR_PERIOD_MS) return;
        lastSent = now;
    }
    switch (field) {
        case 0:
            Serial.print(F("POT,"));
            Serial.print(pot.raw_adc);
            Serial.print(',');
            Serial.println(pot.angle_deg, 2);
            break;
        case 1:
            Serial.print(F("FLEX,"));
            Serial.print(flex.raw_adc);
            Serial.print(',');
            Serial.print(flex.filtered_adc, 2);
            Serial.print(',');
            Serial.println(flex.angle_deg, 2);
            break;
        case 2:
            Serial.print(F("TOF_MM,"));
            Serial.println(distanceMm);
            break;
        case 3:
            Serial.print(F("SLOT,"));
            Serial.print(slot.blocked ? 1 : 0);
            Serial.print(',');
            Serial.println(slot.count);
            break;
    }
    field = (field + 1) % 4;
}

void setup() {
    Serial.begin(115200);
    potBegin();
    flexBegin();
    slotBegin();
    tofBegin();
    buttonBegin();
    dcBegin();
    servoBegin();
    stepperBegin();
    setSystemEnabled(false);
}

void loop() {
    readSerialCommands();
    if (buttonUpdate()) {
        setSystemEnabled(!systemEnabled);
    }
    slot = slotRead();  // Count transitions even while disabled.
    static uint32_t lastRead = 0;
    if (!sensorsReady || millis() - lastRead >= SENSOR_PERIOD_MS) {
        lastRead = millis();
        pot = potRead();
        flex = flexRead();
        distanceMm = tofReadMm();
        sensorsReady = true;
        applySensorControl();
    }
    dcUpdate();
    stepperUpdate();
    sendSensorData();
}
