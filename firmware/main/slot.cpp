// HIGH = beam blocked, LOW = clear

#include <Arduino.h>
#include "lab_modules.h"

static const uint8_t SLOT_PIN = 4;
// TODO: tune debouncing
static const uint32_t SLOT_DEBOUNCE_MS = 5;

static bool slotLastRaw = false;
static bool slotBlocked = false;
static uint32_t slotLastChangeMs = 0;
static uint32_t slotCount = 0;

void slotBegin() {
    pinMode(SLOT_PIN, INPUT_PULLUP);
    slotLastRaw = (digitalRead(SLOT_PIN) == HIGH);
    slotBlocked = slotLastRaw;
    slotLastChangeMs = millis();
    slotCount = 0; // Defined as open to blocked only
}

SlotReading slotRead() {
    const bool rawBlocked = (digitalRead(SLOT_PIN) == HIGH);
    const uint32_t now = millis();

    if (rawBlocked != slotLastRaw) {
        slotLastRaw = rawBlocked;
        slotLastChangeMs = now;
    }
    if (rawBlocked != slotBlocked &&
        (uint32_t)(now - slotLastChangeMs) >= SLOT_DEBOUNCE_MS) {
        slotBlocked = rawBlocked;
        if (slotBlocked) ++slotCount;
    }

    SlotReading reading = {slotBlocked, slotCount};
    return reading;
}
