// RC servo signal on D9. The Servo library uses Timer1 on the Uno.
// Power the servo from an external 5-6 V supply with common ground.

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "lab_modules.h"

static const uint8_t SERVO_PIN = 9;

// TODO: recheck limits
static const uint16_t POT_MIN_ADC = 0;
static const uint16_t POT_MAX_ADC = 1023;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;

static Servo servo;
static bool servoEnabled = false;
static float servoTargetDeg = 90.0f;

void servoBegin() {
    servoEnabled = false;
    servoTargetDeg = 90.0f;
}

float pot_to_servo_angle(uint16_t raw_adc) {
    const uint16_t adc = constrain(raw_adc, POT_MIN_ADC, POT_MAX_ADC);
    const float fraction = (float)(adc - POT_MIN_ADC) /
                           (float)(POT_MAX_ADC - POT_MIN_ADC);
    return SERVO_MIN_DEG + fraction * (SERVO_MAX_DEG - SERVO_MIN_DEG);
}

void set_servo_angle(float angle_deg) {
    if (!isfinite(angle_deg)) return;
    servoTargetDeg = constrain(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    if (servoEnabled) servo.write((int)lroundf(servoTargetDeg));
}

void servoSetEnabled(bool enabled) {
    if (enabled == servoEnabled) return;
    servoEnabled = enabled;
    if (enabled) {
        servo.attach(SERVO_PIN);
        servo.write((int)lroundf(servoTargetDeg));
    } else {
        servo.detach();
    }
}
