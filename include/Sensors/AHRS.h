/**
 * @file AHRS.h
 * @brief Sensor fusion & state estimation menggunakan Extended Kalman Filter (EKF).
 *
 * Mengintegrasikan data IMU, GPS, barometer, dan airspeed sensor untuk
 * estimasi posisi, kecepatan, dan attitude wahana. Termasuk drift correction,
 * estimasi kecepatan angin, serta kalkulasi AOA dan SSA.
 *
 * Dependencies:
 *   Locations.h, Arduino.h, AP_Math.h
 *   Barometer.h, NavEKF.h, airspeed.h, bno055.h, gps.h
 */
// include/Sensors/AHRS.h
#pragma once

#include <AP_Math.h>
#include <Arduino.h>
#include <Locations.h>

#include "Barometer.h"
#include "NavEKF.h"
#include "airspeed.h"
#include "bno055.h"
#include "gps.h"

// ─── Global sensor instances ──────────────────────────────────────
// Didefinisikan di AHRS.cpp; di-extern agar file lain bisa akses
extern Gepees gepees;
extern Airspeed arspd;
extern IMU imu;

// ─── AHRS class ───────────────────────────────────────────────────
class AHRS {
   public:
    Vector3f accel_gravity;
    Vector3f accel_ef;
    Vector3f accel_bf;
    Matrix3f dcm_matrix;
    float    groundspeed;
    bool     is_airspeed_sensor_enabled;

    AHRS();

    void            init_ahrs();
    void            update_ahrs();
    bool            haveGPS() const;
    const Matrix3f &get_rotation_body_to_ned() const;
    Vector3f        get_velocity_ned() const;
    bool            get_position(Locations &loc);
    void            setHome(Locations &loc);
    bool            airspeedEstimate(float *airspeed_ret) const;
    void            estimateGroundspeedVelocity(float sin_h, float cos_h);
    void            estimatePositionEKF();
    void            driftCorrection();
    void            estimate_windspeed();
    void            updateAOASSA();
    void            get_relative_position_D_home(float &posD) const;
    Vector3f        get_accel_ef() const;
    Vector3f        get_accel_bf() const;
    void            threadAHRS();

   private:
    bool      is_gps_lock;
    nav_ekf   ekf;
    Locations home_loc;
    Vector3f  velocity_vector;
    Vector3f  last_velocity;
    float     airspeed;
    float     last_groundspeed;
    float     lp;
    float     hp;
    int32_t   last_latitude;
    int32_t   last_longitude;
    uint32_t  last_aoa_update;
    float     AOA;
    float     SSA;
    Vector3f  last_fuse;
    uint32_t  last_wind_time;
    Vector3f  windspeed;
    float     windspeed_x;
    float     last_airspeed;

    Vector3f getNormAccel();
    Matrix3f getRotationMatFromQuat() const;
};

// Global AHRS instance
extern AHRS ahrs;