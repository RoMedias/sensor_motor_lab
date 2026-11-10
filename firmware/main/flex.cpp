// Hardware: Adafruit Long Flex Sensor (~10k flat, ~20k bent) in a divider
//   5V -- flex -- FLEX_PIN -- 10k -- GND
//   flat: 5 * 10k/(10k+10k) = 2.50 V -> ADC 512
//   bent: 5 * 10k/(20k+10k) = 1.67 V -> ADC 341

#include <Arduino.h>
#include "function_sigs.h"

static const uint8_t FLEX_PIN = A1;

// FILTER SETTINGS
// for spike rejection window
static const uint8_t FLEX_MEDIAN_N = 5;

// EMA weight: higher = faster, lower = smoother
static const float FLEX_ALPHA = 0.2f;

// ms -> 100 Hz, so filter timing doesn't depend on how fast loop() runs
static const uint16_t FLEX_SAMPLE_PERIOD = 10;    
                                                  
// CALIBRATION (TO DO IN PERSON)
// ADC counts (ASCENDING) -> bend angle in degrees, piecewise-linear
// Currently nominal datasheet values
// Replace with own protractor measurements (same data as the transfer-function plot)
// More points handle the sensor's nonlinearity better
static const uint8_t FLEX_CAL_N = 2;
static const float FLEX_CAL_ADC[FLEX_CAL_N] = { 341.0f, 512.0f };
static const float FLEX_CAL_DEG[FLEX_CAL_N] = {  90.0f,   0.0f };

// FILTER STATE
static uint16_t flexBuf[FLEX_MEDIAN_N];
static uint8_t flexBufCount = 0;
static uint8_t flexBufIdx = 0;
static float flexEma = 0.0f;
static bool flexEmaInit  = false;
static uint32_t flexLastSampleMs = 0;
static FlexReading flexLast = {0, 0.0f, 0.0f};

static float flexMedian() {
    uint16_t tmp[FLEX_MEDIAN_N];
    for (uint8_t i = 0; i < flexBufCount; i++) tmp[i] = flexBuf[i];

    // insertion sort, N is tiny
    for (uint8_t i = 1; i < flexBufCount; i++) { 
        uint16_t v = tmp[i];
        int8_t j = i - 1;

        while (j >= 0 && tmp[j] > v) { 
            tmp[j + 1] = tmp[j]; j--; 
        }
        tmp[j + 1] = v;
    }

    uint8_t m = flexBufCount / 2;

    // odd count: middle value, even count: average of the two middle values
    return (flexBufCount % 2) ? tmp[m] : (tmp[m - 1] + tmp[m]) / 2.0f;
}

static float flexAdcToDeg(float adc) {
    if (adc <= FLEX_CAL_ADC[0]) return FLEX_CAL_DEG[0];

    if (adc >= FLEX_CAL_ADC[FLEX_CAL_N - 1]) return FLEX_CAL_DEG[FLEX_CAL_N - 1];

    for (uint8_t i = 0; i < FLEX_CAL_N - 1; i++) {
        if (adc <= FLEX_CAL_ADC[i + 1]) {
            float t = (adc - FLEX_CAL_ADC[i]) / (FLEX_CAL_ADC[i + 1] - FLEX_CAL_ADC[i]);
            return FLEX_CAL_DEG[i] + t * (FLEX_CAL_DEG[i + 1] - FLEX_CAL_DEG[i]);
        }
    }
    return FLEX_CAL_DEG[FLEX_CAL_N - 1];
}

void flexBegin() {
    pinMode(FLEX_PIN, INPUT);
    flexBufCount = 0;
    flexBufIdx = 0;
    flexEmaInit = false;
}

// samples every FLEX_SAMPLE_PERIOD ms
// otherwise returns the latest reading
FlexReading flexRead() {
    uint32_t now = millis();
    if (flexEmaInit && (now - flexLastSampleMs) < FLEX_SAMPLE_PERIOD) {
        return flexLast;
    }
    flexLastSampleMs = now;

    uint16_t raw = analogRead(FLEX_PIN);

    // median of the last N samples (rejects single-sample spikes)
    flexBuf[flexBufIdx] = raw;
    flexBufIdx = (flexBufIdx + 1) % FLEX_MEDIAN_N;
    if (flexBufCount < FLEX_MEDIAN_N) flexBufCount++;
    float med = flexMedian();

    // exponential moving average (smooths remaining noise)
    if (!flexEmaInit) { 
        flexEma = med; 
        flexEmaInit = true; 
    }
    else { 
        flexEma += FLEX_ALPHA * (med - flexEma); 
    }

    flexLast.raw_adc = raw;
    flexLast.filtered_adc = flexEma;
    flexLast.angle_deg = flexAdcToDeg(flexEma);
    return flexLast;
}
