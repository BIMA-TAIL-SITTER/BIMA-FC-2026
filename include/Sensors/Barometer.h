#pragma once

#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

#include "drivers/ms5611.h"
#include "Filter.h"
#include <Filter/Filter.h>

extern LowPassFilterFloat lpfbaro;
extern DerivativeFilterFloat_Size7 _climb_rate_filter;
extern KalmanFilter kalman;

class Barometer {
public:
    bool init_baro();
    void thread_baro();
    void update_baro();
    float altitude;
    float get_climb_rate();
    float getEAS2TAS();

    void readPressure(bool compensation);
    void readTemperature(bool compensation);

private:
    double sea_level;
    double pressure;
    double temperature;

    float source = 1013.25;
    float ground_pressure;
    float ground_altitude;
    float virtual_temp = 0;
    float _last_altitude_EAS2TAS;
    float _last_alt;
    float _EAS2TAS;
    float highest_altitude = 0;
    bool is_reach_approach = false;

    const float ALT_DROP_THRESHOLD = 0.001;

    void determineGroundPress(uint8_t n);
    void hypsometricEquation(float current_press);
    void calcAltitude(double press, double reference_press);
    void calcSeaLevel(double press, double relative_alt);
    bool reached_approach_alt(float current_alt);
};