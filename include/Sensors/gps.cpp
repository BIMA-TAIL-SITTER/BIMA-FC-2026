#include "gps.h"

Barometer baro;

void Gepees::parseLocation() {
    if (gps.location.isValid() || gps.location.isUpdated()) {
        latitude  = (float)gps.location.lat();
        longitude = (float)gps.location.lng();
    }

    if (gps.course.isValid() || gps.course.isUpdated()) {
        course = (float)gps.course.deg();
    }
}

void Gepees::parseSpeed() {
    if (gps.speed.isValid() || gps.speed.isUpdated()) {
        age        = gps.speed.age();
        speed_kmph = (float)gps.speed.kmph();
        speed_mps  = (float)gps.speed.mps();
    }
}

void Gepees::parseSatDOP() {
    if (gps.hdop.isValid() || gps.hdop.isUpdated()) {
        hdop = (float)gps.hdop.hdop();
    }

    if (gps.satellites.isValid() || gps.satellites.isUpdated()) {
        satellites = (float)gps.satellites.value();
    }
}

void Gepees::calculateVelocity() {
    if (gps.speed.isUpdated()) {
        float    horizontal_speed = (float)gps.speed.mps();
        uint32_t curr_time        = millis();
        float    time_interval    = (curr_time - prev_time) / 1000.0f;  // detik
        prev_time = curr_time;

        velocity.x = horizontal_speed * cos(radians(course));
        velocity.y = horizontal_speed * sin(radians(course));
        velocity.z = (baro.altitude - prev_alt) / time_interval;
        prev_alt   = baro.altitude;
    }
}

void Gepees::checkStatus() {
    if ((hdop > 0.0f && hdop < 1.0f) && satellites > 6) {
        gps_status = GPS_FIX_3D;
    } else if ((hdop > 0.0f && hdop < 1.0f) || satellites > 6) {
        gps_status = GPS_FIX_2D;
    } else {
        gps_status = GPS_NO_FIX;
    }
}

void Gepees::initGPS() {
    ss.begin(baudRate);
    Serial.println(TinyGPSPlus::libraryVersion());
}

void Gepees::updateGPS() {
    while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
            parseLocation();
            parseSpeed();
            parseSatDOP();
            calculateVelocity();
            checkStatus();
            line++;
        }
    }
}