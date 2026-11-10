// SparkFun VL53L1X TOF sensor, Arduino Uno I2C: SDA=A4, SCL=A5.
// Requires "SparkFun VL53L1X 4m Laser Distance Sensor" Arduino library.
// Nonblocking: returns most recent distance in mm, or -1 if unavailable.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include "lab_modules.h"

static SFEVL53L1X tof;
static bool tofReady = false;
static int16_t lastDistanceMm = -1;
static uint32_t lastValidMs = 0;
static const uint32_t TOF_STALE_MS = 250;

void tofBegin() {
    Wire.begin();
    tofReady = (tof.begin() == 0);
    lastDistanceMm = -1;
    lastValidMs = 0;
    if (tofReady) {
        tof.setDistanceModeLong();
        tof.setTimingBudgetInMs(50);
        tof.setIntermeasurementPeriod(50);
        tof.startRanging();
    }
}

int16_t tofReadMm() {
    if (!tofReady) return -1;
    if (tof.checkForDataReady()) {
        const uint16_t distance = tof.getDistance();
        const uint8_t status = tof.getRangeStatus();
        tof.clearInterrupt();
        if (status == 0 && distance > 0 && distance <= 4000) {
            lastDistanceMm = (int16_t)distance;
            lastValidMs = millis();
        } else {
            lastDistanceMm = -1;
        }
    }
    if (lastValidMs == 0 || (uint32_t)(millis() - lastValidMs) > TOF_STALE_MS)
        return -1;
    return lastDistanceMm;
}
