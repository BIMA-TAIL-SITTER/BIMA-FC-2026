/**
 * @file bno055.h
 * @brief Provides functions and data structures for interacting with a BNO055 IMU sensor.
 *
 *! Key features:
 *  - Handles conversion of raw sensor data from the BNO055 IMU
 *  - Extracts Euler angles (roll, pitch, heading), gyroscope data and accelerometer data
 *  - Converts raw data to degrees and radians
 *  - Provides functions to invert heading and calculate yaw
 *  - Includes necessary header files and data structures to interface with the BNO055 sensor
 *  - Implements a dedicated thread for continuous IMU updates
 *
 *! Dependencies:
 *  - Arduino.h
 *  - AP_Math.h
 *  - BNO055_support.h
 *  - TeensyThreads.h
 *  - I2CDev.h
 */

#pragma once

#include <Arduino.h>
#include <AP_Math.h>
#include <BNO055_support.h>
#include "I2CDev.h"

// Representation Unit Settings
#define ACCEL_TO_MS2      100
#define ACCEL_TO_MG       1
#define MAG_TO_MIKROTESLA 16
#define GYRO_TO_DPS       16
#define GYRO_TO_RPS       900
#define EULER_TO_DEG      16
#define EULER_TO_RAD      900
#define QUAT_UNIT_LESS    16384
#define GRAVITY_TO_MS2    100
#define GRAVITY_TO_MG     1
#define TEMP_TO_CELC      1
#define TEMP_TO_FAHR      2

class IMU {
public:
    Vector3f _accel;
    Vector3f _linear_acc;
    float roll, pitch, heading, yaw;
    float rad_roll, rad_pitch, rad_yaw;
    float last_roll, last_pitch, last_yaw;
    float delta_roll, delta_pitch, delta_yaw;
    float gyro_x, gyro_y, gyro_z;
    int pitch_mode;

    void init_imu();
    void update_imu();
    void check_imu_calibration();
    void imu_thread();
    void imu_task();

private:
    struct bno055_t            bno_data;
    struct bno055_euler        bno_euler;
    struct bno055_gyro         bno_gyro;
    struct bno055_accel        bno_accel;
    struct bno055_quaternion   bno_quat;
    struct bno055_linear_accel bno_linear;

    unsigned char  sys_calib_status = 0;
    unsigned char  mag_calib_status = 0;
    unsigned long  last_time = 0;

    float invert_heading(float raw_h);
    float calc_yaw(float real_h);
    void  extract_euler();
    void  extract_gyroscope();
    void  extract_accelerometer();
    void  extract_linear_accelerometer();
};