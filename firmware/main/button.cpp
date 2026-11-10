// Pushbutton: D12 to GND, using INPUT_PULLUP.
// Nonblocking debounce; one true event per press.
#include <Arduino.h>
#include "lab_modules.h"

static const uint8_t BUTTON_PIN = 12;
static const unsigned long DEBOUNCE_MS = 40;
static bool lastRaw = HIGH;
static bool stableState = HIGH;
static unsigned long lastChangeMs = 0;

void buttonBegin() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastRaw = digitalRead(BUTTON_PIN);
    stableState = lastRaw;
    lastChangeMs = millis();
}

bool buttonUpdate() {
    bool raw = digitalRead(BUTTON_PIN);
    unsigned long now = millis();
    if (raw != lastRaw) {
        lastChangeMs = now;
        lastRaw = raw;
    }
    if ((now - lastChangeMs) >= DEBOUNCE_MS && raw != stableState) {
        stableState = raw;
        return stableState == LOW; // one event on press, not release
    }
    return false;
}
