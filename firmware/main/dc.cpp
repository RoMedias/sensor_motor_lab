// Hardware: Cytron SPG30E DC gear motor + Hall quadrature encoder on a Solarbotics L298
//   EN = D5 (PWM, Timer0 so the Servo library on Timer1 doesn't affect it)
//   I1 = D8, I2 = D11 (direction)
//   Encoder A = A2, B = A3 (pin-change interrupt, 4x quadrature decoding)
//   POSITION/VELOCITY: encoder P control. SENSOR duty mode: signed motor power.

#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "lab_modules.h"

static const uint8_t DC_PIN_EN    = 5;
static const uint8_t DC_PIN_IN1   = 8;
static const uint8_t DC_PIN_IN2   = 11;
static const uint8_t DC_PIN_ENC_A = A2;
static const uint8_t DC_PIN_ENC_B = A3;

// Placeholders
// TODO - Calibrate these

// Output-shaft calibration, 2026-11-11: 3614 raw quadrature counts over
// 10 observed turns, including the final coast: 3614 / 10 = 361.4 counts/rev.
// Positive PWM must produce positive counts; swap A/B if direction is reversed.
static const float DC_CPR = 361.4f;
static const long DC_POSITION_TOL_COUNTS = 1;  // About one degree.

// LOOP SETTINGS
// us conversion * 1/Frequency
static const uint32_t DC_DT_US = 10000UL;
// Average encoder counts over 100 ms to reduce RPM quantization noise.
static const uint32_t DC_RPM_WINDOW_US = 100000UL;

// EMA weight on the speed estimate: higher = faster, lower = smoother
static const float DC_RPM_ALPHA = 0.3f;

// P-only gains (position error in degrees, velocity error in RPM).
// Tune these numbers on the actual motor.
static const float DC_KP = 4.0f;
static const float DC_KP_V = 4.0f;  // Stronger initial response; tune on the motor.

// DC STATE
enum DcMode : uint8_t { DC_IDLE, DC_POSITION, DC_VELOCITY, DC_DUTY };

static volatile long dcEncCount = 0;
static volatile uint8_t dcEncPrev = 0;
static volatile uint32_t dcEncEdges = 0;
static DcMode  dcMode        = DC_IDLE;
static long     dcTargetCount = 0;
static int8_t   dcPositionDirection = 0;
static float    dcRpmTarget   = 0.0f;
static long     dcLastCount   = 0;
static float    dcRpmF        = 0.0f;
static uint32_t dcLastUs      = 0;
static uint32_t dcLastRpmUs   = 0;

// Match the quadrature direction used by test_components/test_components.ino.
// The previous polling decoder used the opposite sign from that test sketch.
ISR(PCINT1_vect) {
    static const int8_t transitions[16] = {
         0, -1,  1,  0,  1,  0,  0, -1,
        -1,  0,  0,  1,  0,  1, -1,  0
    };
    uint8_t current = (PINC >> 2) & 0x03;
    if (current != dcEncPrev) ++dcEncEdges;
    dcEncCount += transitions[(dcEncPrev << 2) | current];
    dcEncPrev = current;
}

static long dcReadEncoder() {
    long count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = dcEncCount;
    }
    return count;
}

// u in [-255, 255], sign = direction
static void dcDrive(float u) {
    int pwm = (int)constrain(u, -255.0f, 255.0f);
    digitalWrite(DC_PIN_IN1, pwm > 0 ? HIGH : LOW);
    digitalWrite(DC_PIN_IN2, pwm < 0 ? HIGH : LOW);
    analogWrite(DC_PIN_EN, abs(pwm));
}

// L298 fast stop: equal direction inputs with enable HIGH brakes the motor.
// Ordinary disable/stop still uses dcDrive(0), releasing the bridge outputs.
static void dcBrake() {
    digitalWrite(DC_PIN_IN1, LOW);
    digitalWrite(DC_PIN_IN2, LOW);
    analogWrite(DC_PIN_EN, 255);
}

void dcBegin() {
    pinMode(DC_PIN_EN, OUTPUT);
    pinMode(DC_PIN_IN1, OUTPUT);
    pinMode(DC_PIN_IN2, OUTPUT);
    dcDrive(0);

    pinMode(DC_PIN_ENC_A, INPUT_PULLUP);
    pinMode(DC_PIN_ENC_B, INPUT_PULLUP);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dcEncPrev = (PINC >> 2) & 0x03;
        dcEncCount = 0;
        dcEncEdges = 0;
        PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
        PCIFR = _BV(PCIF1);
        PCICR |= _BV(PCIE1);
    }
    dcLastCount = 0;
    dcLastUs = micros();
    dcLastRpmUs = dcLastUs;
    dcMode = DC_IDLE;
}

// GUI mode: signed relative move, measured from the shaft's current position
// Stops output at the nearest achievable encoder target.
void dcMoveRelative(float delta_deg) {
    if (!isfinite(delta_deg)) return;
    long current = dcReadEncoder();
    dcTargetCount = current + lround(delta_deg * DC_CPR / 360.0f);
    dcPositionDirection = dcTargetCount > current ? 1 : dcTargetCount < current ? -1 : 0;
    dcDrive(0);
    dcMode = DC_POSITION;
}

// GUI or SENSOR mode: signed RPM, 0 stops
void dcSetVelocity(float rpm) {
    if (!isfinite(rpm)) return;
    if (rpm == 0.0f) {
        dcStop();
        return;
    }
    if (dcMode != DC_VELOCITY || (rpm > 0) != (dcRpmTarget > 0)) {
        dcDrive(0);
    }
    dcRpmTarget = rpm;
    dcMode = DC_VELOCITY;
}

// SENSOR mode: signed fraction of full motor power, -1..1.
// This supports "far = maximum" without assuming an unmeasured maximum RPM.
void dcSetDuty(float duty) {
    if (!isfinite(duty)) return;
    duty = constrain(duty, -1.0f, 1.0f);
    if (duty == 0.0f) {
        dcStop();
        return;
    }
    dcMode = DC_DUTY;
    dcDrive(255.0f * duty);
}

// Output off and movement cancelled.
void dcStop() {
    dcDrive(0);
    dcMode = DC_IDLE;
    dcRpmTarget = 0.0f;
    dcPositionDirection = 0;
}

// Runs the P controller every DC_DT_US, otherwise returns immediately.
void dcUpdate() {
    uint32_t now = micros();
    uint32_t elapsed = now - dcLastUs;
    if (elapsed < DC_DT_US) return;
    dcLastUs = now;

    // speed estimate runs in every mode so dcGetRpm() is always valid
    long c = dcReadEncoder();
    uint32_t rpmElapsed = now - dcLastRpmUs;
    if (rpmElapsed >= DC_RPM_WINDOW_US) {
        long dcount = c - dcLastCount;
        dcLastCount = c;
        dcLastRpmUs = now;
        float rpmRaw = (dcount / DC_CPR) / (rpmElapsed * 1e-6f) * 60.0f;
        dcRpmF += DC_RPM_ALPHA * (rpmRaw - dcRpmF);
    }

    if (dcMode == DC_IDLE || dcMode == DC_DUTY) return;

    if (dcMode == DC_POSITION) {
        bool reached = labs(dcTargetCount - c) <= DC_POSITION_TOL_COUNTS ||
                       (dcPositionDirection > 0 && c >= dcTargetCount) ||
                       (dcPositionDirection < 0 && c <= dcTargetCount);
        if (reached) {
            dcStop();
            dcBrake();
            return;
        }
    }

    // error in deg (position) or RPM (velocity)
    float e = (dcMode == DC_POSITION)
                  ? (dcTargetCount - c) * 360.0f / DC_CPR
                  : dcRpmTarget - dcRpmF;

    float kp = dcMode == DC_POSITION ? DC_KP : DC_KP_V;
    float u = kp * e;
    dcDrive(u);
}

// Raw count, independent of the provisional counts-per-revolution scale.
long dcGetEncoderCount() {
    return dcReadEncoder();
}

uint8_t dcGetEncoderAB() {
    return (PINC >> 2) & 0x03;
}

uint32_t dcGetEncoderEdges() {
    uint32_t edges;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { edges = dcEncEdges; }
    return edges;
}

// current shaft angle from power-on in degrees
float dcGetPositionDeg() {
    return dcReadEncoder() * 360.0f / DC_CPR;
}

// filtered shaft speed in RPM
float dcGetRpm() {
    return dcRpmF;
}
