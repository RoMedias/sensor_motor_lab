#pragma once

#include <Arduino.h>

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
    uint32_t count;
};

enum class ControlMode : uint8_t { GUI, SENSOR };

// Teammate modules include this header and provide these definitions.
void potBegin();
void flexBegin();
void slotBegin();
void tofBegin();
void buttonBegin();
void dcBegin();
void servoBegin();
void stepperBegin();
PotReading potRead();
FlexReading flexRead();
SlotReading slotRead();
int16_t tofReadMm();
bool buttonUpdate();
void dcMoveRelative(float delta_deg);
void dcSetVelocity(float rpm);
void dcUpdate();
void dcStop();
float pot_to_servo_angle(uint16_t raw_adc);
void set_servo_angle(float angle_deg);
void servoSetEnabled(bool enabled);
float stepperMoveRelative(float delta_deg);
void stepperSetPosition(float target_deg);
void stepperUpdate();
void stepperStop();
