/**
 * @file gps.h
 * @brief Provides functionality to interface with a GPS module using the TinyGPSPlus library.
 */
#pragma once

#include <AP_Math.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include "Barometer.h"

extern Barometer baro;

// Enumerasi Status GPS
enum StatusGPS {
    GPS_NO_FIX,
    GPS_FIX_2D,
    GPS_FIX_3D
};

// Kelas GPS
class Gepees {
public:
    Gepees(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
        : gps_status(GPS_NO_FIX), latitude(0), longitude(0), hdop(0), satellites(0),
          speed_mps(0), velocity(), age(0), gps(), ss(rxPin, txPin), baudRate(baudRate),
          alt_m(0), alt_cm(0), speed_kmph(0), course(0), prev_alt(0), prev_time(0) {}

    void initGPS();
    void updateGPS();

    int line = 0;

    StatusGPS  gps_status;
    float      latitude;
    float      longitude;
    float      hdop;
    float      satellites;
    float      speed_mps;
    Vector3f   velocity;
    uint32_t   age;
    TinyGPSPlus    gps;
    SoftwareSerial ss;
    uint32_t   baudRate;

    float      getLatitude()     const { return latitude; }
    float      getLongitude()    const { return longitude; }
    float      getAltitudeM()    const { return alt_m; }
    int32_t    getAltitudeCM()   const { return alt_cm; }
    float      getHDOP()         const { return hdop; }
    float      getSatellites()   const { return satellites; }
    float      getSpeedMPS()     const { return speed_mps; }
    float      getSpeedKMPH()    const { return speed_kmph; }
    float      getCourse()       const { return course; }
    Vector3f   getVelocity()     const { return velocity; }
    StatusGPS  getStatus()       const { return gps_status; }

private:
    float    alt_m;
    int32_t  alt_cm;
    float    speed_kmph;
    float    course;
    float    prev_alt;
    uint32_t prev_time;

    void parseLocation();
    void parseSpeed();
    void parseSatDOP();
    void calculateVelocity();
    void checkStatus();
};