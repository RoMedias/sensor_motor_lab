// Hardware: SparkFun 10k linear rotary potentiometer (~300 deg travel)
//   5V -- pot -- GND, wiper -> POT_PIN
//   angle_deg = POT_M * adc + POT_B (linear fit)

#include <Arduino.h>
#include "lab_modules.h"

static const uint8_t POT_PIN = A0;

// FILTER SETTINGS
// back-to-back reads averaged per call (~0.1 ms each)
static const uint8_t POT_SAMPLES = 4;

// CALIBRATION (TO DO IN PERSON)
// Currently nominal: 300 deg over the full ADC range, 0 deg at ADC 0
// Replace with a least-squares fit of own protractor measurements
// (same data as the transfer-function plot; report max and RMS error)

// deg per ADC count
static const float POT_M = 300.0f / 1023.0f;

// deg at ADC 0
static const float POT_B = 0.0f;

void potBegin() {
    pinMode(POT_PIN, INPUT);
}

// returns the rounded mean of POT_SAMPLES reads and its calibrated angle
PotReading potRead() {
    // 4 * 1023 fits in 16 bits
    uint16_t sum = 0;
    for (uint8_t i = 0; i < POT_SAMPLES; i++) sum += analogRead(POT_PIN);

    PotReading r;
    r.raw_adc = (sum + POT_SAMPLES / 2) / POT_SAMPLES;
    r.angle_deg = POT_M * r.raw_adc + POT_B;
    return r;
}
