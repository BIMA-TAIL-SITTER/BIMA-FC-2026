#include "bno055.h"

void IMU::check_imu_calibration() {
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial2.print("Time Stamp: ");
            Serial2.println(last_time);
            Serial2.print("Magnetometer Calibration Status: ");
            Serial2.println(mag_calib_status);
            Serial2.print("System Calibration Status: ");
            Serial2.println(sys_calib_status);
            last_time = millis();
        }
    }
}

float IMU::invert_heading(float raw_h) {
    float real_h = raw_h - 180.0f;
    if (real_h < 0) real_h += 360.0f;
    return real_h;
}

float IMU::calc_yaw(float real_h) {
    return real_h > 180.0f ? real_h - 360.0f : real_h;
}

void IMU::extract_euler() {
    bno055_read_euler_hrp(&bno_euler);

    //? Convert to degrees
    roll    = (float)bno_euler.r / EULER_TO_DEG;
    pitch   = (float)bno_euler.p / EULER_TO_DEG;
    heading = invert_heading((float)bno_euler.h / EULER_TO_DEG);
    yaw     = calc_yaw(heading);

    //? Convert to radians
    rad_roll  = (float)bno_euler.r / EULER_TO_RAD;
    rad_pitch = (float)bno_euler.p / EULER_TO_RAD;
    rad_yaw   = radians(yaw);
}

void IMU::extract_gyroscope() {
    bno055_read_gyro_xyz(&bno_gyro);
    gyro_x =      (float)bno_gyro.x / GYRO_TO_DPS;
    gyro_y = -1 * (float)bno_gyro.y / GYRO_TO_DPS;
    gyro_z =      (float)bno_gyro.z / GYRO_TO_DPS;
}

void IMU::extract_accelerometer() {
    bno055_read_accel_xyz(&bno_accel);
    _accel.x = (float)bno_accel.x;
    _accel.y = (float)bno_accel.y;
    _accel.z = (float)bno_accel.z;
}

void IMU::extract_linear_accelerometer() {
    bno055_read_linear_accel_xyz(&bno_linear);
    _linear_acc.x = (float)bno_linear.x / ACCEL_TO_MS2;
    _linear_acc.y = (float)bno_linear.y / ACCEL_TO_MS2;
    _linear_acc.z = (float)bno_linear.z / ACCEL_TO_MS2;
}

void IMU::init_imu() {
    _I2C.init();
    BNO_Init(&bno_data);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1 << 9);
}

void IMU::update_imu() {
    extract_linear_accelerometer();
    extract_euler();
    extract_gyroscope();
    extract_accelerometer();

    delta_roll  = abs(roll  - last_roll);
    delta_pitch = abs(pitch - last_pitch);
    delta_yaw   = abs(yaw   - last_yaw);

    last_roll  = roll;
    last_pitch = pitch;
    last_yaw   = yaw;
}