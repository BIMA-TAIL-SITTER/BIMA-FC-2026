#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include <stdint.h>
#include "Filter.h"

// MS4525D sensor characteristic
const uint8_t MS4525DAddress = 0x28;
const int16_t MS4525FullScaleRange = 1;  // 1 psi
const int16_t MS4525MinScaleCounts = 1617;                                            // Voff
const int16_t MS4525FullScaleCounts = 14569;                                          // Vfso
const int16_t MS4525Span = MS4525FullScaleCounts - MS4525MinScaleCounts;              // span
const int16_t MS4525ZeroCounts = (MS4525MinScaleCounts - MS4525FullScaleCounts) / 2;  //?

extern KalmanFilter kalmanFilter;

class Airspeed {
public:
    Airspeed() : press_psi(0), psi_offset(0), v_ms(0), v_kmh(0), prspsi(0), init_press(20.0f) {}

    void initAirspeed();
    void updateAirspeed();
    void airspeedThread();
    void calibrateAirspeed();

    float getVelocityMS() const { return v_ms; }
    float getVelocityKMH() const { return v_kmh; }
    float getPressurePSI() const { return press_psi; }

    float press_psi, psi_offset, v_ms, v_kmh, prspsi;

private:
    byte stats;
    volatile uint16_t press_data, temp_data;
    float init_press;

    byte getRawAirspeed();
};