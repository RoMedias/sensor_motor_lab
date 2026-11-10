// Hardware: Cytron SPG30E DC gear motor + Hall quadrature encoder on a Solarbotics L298
//   EN = D5 (PWM, Timer0 so the Servo library on Timer1 doesn't affect it)
//   I1 = D8, I2 = D11 (direction)
//   Encoder A = A2, B = A3 (polled every loop, counts rising edges of A, B gives direction)
//   POSITION mode: PID on angle error, VELOCITY mode: PID on RPM error

#include <Arduino.h>
#include "lab_modules.h"

static const uint8_t DC_PIN_EN    = 5;
static const uint8_t DC_PIN_IN1   = 8;
static const uint8_t DC_PIN_IN2   = 11;
static const uint8_t DC_PIN_ENC_A = A2;
static const uint8_t DC_PIN_ENC_B = A3;

// Placeholders
// TODO - Calibrate these

// Counts per rev - measure via serial reads
// rising edges of A per output-shaft rev = test_components.ino count / 4
// if forward PWM gives a negative count, swap the encoder A/B wires
static const float DC_CPR = 90.0f;

// LOOP SETTINGS
// us conversion * 1/Frequency
static const uint32_t DC_DT_US = 1000000 * 1/100; 

// EMA weight on the speed estimate: higher = faster, lower = smoother
static const float DC_RPM_ALPHA = 0.3f;

// POSITION PID (error in deg, output in PWM -255..255)
// TODO - Tune these numbers.
static const float DC_KP = 4.0f;
static const float DC_KI = 0.0f;
static const float DC_KD = 0.0f;

// VELOCITY PID (error in RPM, output in PWM -255..255)
// TODO - Tune these numbers.
static const float DC_KP_V = 1.0f;
static const float DC_KI_V = 4.0f;
static const float DC_KD_V = 0.0f;

// DC STATE
enum DcMode : uint8_t { DC_IDLE, DC_POSITION, DC_VELOCITY };

static long     dcEncCount    = 0;
static bool     dcLastA       = false;
static DcMode  dcMode        = DC_IDLE;
static long     dcTargetCount = 0;
static float    dcRpmTarget   = 0.0f;
static float    dcInteg       = 0.0f;
static float    dcPrevErr     = 0.0f;
static long     dcLastCount   = 0;
static float    dcRpmF        = 0.0f;
static uint32_t dcLastUs      = 0;

// on each rising edge of A: B low -> +1, B high -> -1
static void dcPollEncoder() {
    bool a = digitalRead(DC_PIN_ENC_A);
    if (a && !dcLastA) {
        if (digitalRead(DC_PIN_ENC_B)) dcEncCount--;
        else dcEncCount++;
    }
    dcLastA = a;
}

// u in [-255, 255], sign = direction
static void dcDrive(float u) {
    int pwm = (int)constrain(u, -255.0f, 255.0f);
    digitalWrite(DC_PIN_IN1, pwm > 0 ? HIGH : LOW);
    digitalWrite(DC_PIN_IN2, pwm < 0 ? HIGH : LOW);
    analogWrite(DC_PIN_EN, abs(pwm));
}

static void dcResetPid() {
    dcInteg = 0.0f;
    dcPrevErr = 0.0f;
}

void dcBegin() {
    pinMode(DC_PIN_EN, OUTPUT);
    pinMode(DC_PIN_IN1, OUTPUT);
    pinMode(DC_PIN_IN2, OUTPUT);
    dcDrive(0);

    pinMode(DC_PIN_ENC_A, INPUT_PULLUP);
    pinMode(DC_PIN_ENC_B, INPUT_PULLUP);

    dcLastA = digitalRead(DC_PIN_ENC_A);
    dcEncCount = 0;
    dcLastCount = 0;
    dcLastUs = micros();
    dcMode = DC_IDLE;
}

// GUI mode: signed relative move, measured from the shaft's current position
// holds the target until another command or dcStop()
void dcMoveRelative(float delta_deg) {
    dcTargetCount = dcEncCount + lround(delta_deg * DC_CPR / 360.0f);
    dcResetPid();
    dcMode = DC_POSITION;
}

// GUI or SENSOR mode: signed RPM, 0 stops
void dcSetVelocity(float rpm) {
    if (rpm == 0.0f) {
        dcStop();
        return;
    }
    if (dcMode != DC_VELOCITY) dcResetPid();
    dcRpmTarget = rpm;
    dcMode = DC_VELOCITY;
}

// output off, movement cancelled, PID reset
void dcStop() {
    dcDrive(0);
    dcMode = DC_IDLE;
    dcRpmTarget = 0.0f;
    dcResetPid();
}

// polls the encoder every call
// runs the PID every DC_DT_US, otherwise returns immediately
void dcUpdate() {
    dcPollEncoder();

    uint32_t now = micros();
    uint32_t elapsed = now - dcLastUs;
    if (elapsed < DC_DT_US) return;
    dcLastUs = now;
    float dt = elapsed * 1e-6f;

    // speed estimate runs in every mode so dcGetRpm() is always valid
    long c = dcEncCount;
    long dcount = c - dcLastCount;
    dcLastCount = c;
    float rpmRaw = (dcount / DC_CPR) / dt * 60.0f;
    dcRpmF += DC_RPM_ALPHA * (rpmRaw - dcRpmF);

    if (dcMode == DC_IDLE) return;

    // error in deg (position) or RPM (velocity)
    float e = (dcMode == DC_POSITION)
                  ? (dcTargetCount - c) * 360.0f / DC_CPR
                  : dcRpmTarget - dcRpmF;

    dcInteg += e * dt;
    float deriv = (e - dcPrevErr) / dt;
    dcPrevErr = e;

    float u = (dcMode == DC_POSITION)
                  ? DC_KP * e + DC_KI * dcInteg + DC_KD * deriv
                  : DC_KP_V * e + DC_KI_V * dcInteg + DC_KD_V * deriv;
    dcDrive(u);
}

// current shaft angle from power-on in degrees
float dcGetPositionDeg() {
    return dcEncCount * 360.0f / DC_CPR;
}

// filtered shaft speed in RPM
float dcGetRpm() {
    return dcRpmF;
}