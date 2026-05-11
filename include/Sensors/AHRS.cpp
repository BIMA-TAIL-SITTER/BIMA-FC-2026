// include/Sensors/AHRS.cpp
#include "AHRS.h"

// ─── Global sensor instance definitions ───────────────────────────
Gepees  gepees(0, 1, 38400);
Airspeed arspd;
IMU      imu;

// ─── Global AHRS instance ─────────────────────────────────────────
AHRS ahrs;

// ─── Constructor ──────────────────────────────────────────────────
AHRS::AHRS()
    : accel_gravity(0.0f, 0.0f, -9.80665f),
      accel_ef(),
      accel_bf(),
      dcm_matrix(),
      groundspeed(0.0f),
      is_airspeed_sensor_enabled(false),
      is_gps_lock(false),
      ekf(),
      home_loc(),
      velocity_vector(),
      last_velocity(),
      airspeed(0.0f),
      last_groundspeed(0.0f),
      lp(0.0f),
      hp(0.0f),
      last_latitude(0),
      last_longitude(0),
      last_aoa_update(0),
      AOA(0.0f),
      SSA(0.0f),
      last_fuse(),
      last_wind_time(0),
      windspeed(),
      windspeed_x(0.0f),
      last_airspeed(0.0f) {}

// ─── Public: status & getters ─────────────────────────────────────

bool AHRS::haveGPS() const {
    return gepees.gps_status > GPS_NO_FIX;
}

Vector3f AHRS::get_accel_bf() const {
    return accel_bf;
}

Vector3f AHRS::get_accel_ef() const {
    return accel_ef;
}

const Matrix3f &AHRS::get_rotation_body_to_ned() const {
    return dcm_matrix;
}

Vector3f AHRS::get_velocity_ned() const {
    return velocity_vector;
}

void AHRS::get_relative_position_D_home(float &posD) const {
    posD = -baro.altitude;
}

// ─── Public: position & home ──────────────────────────────────────

bool AHRS::get_position(Locations &loc) {
    ekf.get_pos_ned(loc);
    loc.lat = gepees.latitude  * 10000000;
    loc.lng = gepees.longitude * 10000000;
    loc.alt = baro.altitude    * 100;
    return true;
}

void AHRS::setHome(Locations &loc) {
    if (gepees.gps_status >= GPS_FIX_3D) {
        home_loc.lat = loc.lat = gepees.latitude  * 10000000;
        home_loc.lng = loc.lng = gepees.longitude * 10000000;
        home_loc.alt = loc.alt = baro.altitude    * 100;

        ekf.set_origin(home_loc);
        ekf.set_origin(loc);
    }
}

// ─── Public: airspeed ─────────────────────────────────────────────

bool AHRS::airspeedEstimate(float *airspeed_ret) const {
    if (haveGPS()) {
        *airspeed_ret = last_airspeed;
        return true;
    }
    return false;
}

// ─── Private: velocity & position estimation ──────────────────────

void AHRS::estimateGroundspeedVelocity(float sin_h, float cos_h) {
    const float dt   = 0.1f;
    const float beta = 0.1f;
    const float alpha = 1.0f - beta;
    const Vector3f acc_bf = getNormAccel();

    lp = gepees.speed_mps * beta + alpha * lp;
    hp = ((groundspeed - last_groundspeed) + acc_bf.x * cos_h * dt) + alpha * hp;
    groundspeed = (hp + lp) * 10000;

    if (groundspeed < 0.0f) groundspeed = 0.0f;

    if (is_gps_lock) {
        last_groundspeed = gepees.speed_mps;
        last_velocity    = gepees.velocity;
    } else {
        last_groundspeed = groundspeed;
        last_velocity    = velocity_vector;
    }

    velocity_vector.x = groundspeed * cos_h;
    velocity_vector.y = groundspeed * sin_h;
    velocity_vector.z = 0.0f; // TODO: implementasi climb rate
}

void AHRS::estimatePositionEKF() {
    if (haveGPS() && home_loc.lat != 0 && home_loc.lng != 0) {
        Locations loc;
        loc.lat = gepees.latitude  * 10000000;
        loc.lng = gepees.longitude * 10000000;
        Vector2f vel(velocity_vector.x, velocity_vector.y);
        ekf.estimate_pos_and_vel(loc, vel);
        last_latitude  = loc.lat;
        last_longitude = loc.lng;
    } else if (home_loc.lat != 0 && home_loc.lng != 0) {
        Locations last_loc(last_latitude, last_longitude, 0,
                           Locations::AltFrame::ABOVE_HOME);
        Vector2f vel(velocity_vector.x, velocity_vector.y);
        ekf.estimate_pos_and_vel(last_loc, vel);
    }
}

void AHRS::driftCorrection() {
    float sin_heading = sinf(radians(imu.heading));
    float cos_heading = cosf(radians(imu.heading));
    bool  is_gps_fix  = gepees.age > 0 && gepees.age < 3000;

    if (is_gps_fix || gepees.gps_status >= GPS_FIX_3D || gepees.gps.speed.isUpdated()) {
        groundspeed     = gepees.speed_mps; // cek lagi saat terbang, jangan *1000
        velocity_vector = gepees.velocity;
        is_gps_lock     = true;
    } else if (haveGPS()) {
        estimateGroundspeedVelocity(sin_heading, cos_heading);

        airspeed = is_airspeed_sensor_enabled
                       ? arspd.v_ms
                       : groundspeed - windspeed_x;
        last_airspeed = MAX(airspeed, 0.0f);
        is_gps_lock   = false;
    } else {
        groundspeed = 0.0f;
        is_gps_lock = false;
    }

    estimatePositionEKF();
}

// ─── Private: wind & AOA/SSA ──────────────────────────────────────

void AHRS::estimate_windspeed() {
    // Berdasarkan algoritma wind estimation MatrixPilot oleh Bill Premerlani,
    // diadaptasi untuk ArduPilot oleh Jon Challinger.
    const Vector3f fuselage_direction      = dcm_matrix.colx();
    const Vector3f fuselage_direction_diff = fuselage_direction - last_fuse;
    const uint32_t now = millis();

    if (now - last_wind_time > 1000) {
        last_wind_time = now;
        last_fuse      = fuselage_direction;
        last_velocity  = velocity_vector;
        return;
    }

    float diff_length = fuselage_direction_diff.length();
    if (diff_length > 0.2f) {
        const Vector3f velocity_diff = velocity_vector - last_velocity;
        float V = velocity_diff.length() / diff_length;

        const Vector3f fuselage_direction_sum = fuselage_direction + last_fuse;
        const Vector3f velocity_sum           = velocity_vector + last_velocity;
        last_fuse     = fuselage_direction;
        last_velocity = velocity_vector;

        const float theta    = atan2f(velocity_diff.y, velocity_diff.x)
                             - atan2f(fuselage_direction_diff.y, fuselage_direction_diff.x);
        const float sintheta = sinf(theta);
        const float costheta = cosf(theta);

        Vector3f wind;
        wind.x = velocity_sum.x - V * (costheta * fuselage_direction_sum.x
                                      - sintheta * fuselage_direction_sum.y);
        wind.y = velocity_sum.y - V * (sintheta * fuselage_direction_sum.x
                                      + costheta * fuselage_direction_sum.y);
        wind.z = velocity_sum.z - V * fuselage_direction_sum.z;
        wind  *= 0.5f;

        if (wind.length() < windspeed.length() + 20.0f) {
            windspeed = windspeed * 0.95f + wind * 0.05f;
        }

        last_wind_time = now;
        windspeed_x    = sqrtf(windspeed.x * windspeed.x + windspeed.y * windspeed.y);
    }
}

void AHRS::updateAOASSA() {
    const uint32_t now = millis();
    if (now - last_aoa_update < 50) return;
    last_aoa_update = now;

    Vector3f aoa_velocity = gepees.velocity;
    if (aoa_velocity.length() == 0.0f) return;

    Vector3f aoa_wind = windspeed;

    const Matrix3f &rot = getRotationMatFromQuat();
    aoa_velocity = rot.mul_transpose(aoa_velocity);
    aoa_wind     = rot.mul_transpose(aoa_wind);

    aoa_velocity = aoa_velocity - aoa_wind;
    const float vel_len = aoa_velocity.length();

    if (vel_len < 2.0f) {
        AOA = 0.0f;
        SSA = 0.0f;
        return;
    }

    AOA = (aoa_velocity.x > 0.0f)
              ? degrees(atanf(aoa_velocity.z / aoa_velocity.x))
              : 0.0f;
    SSA = degrees(safe_asin(aoa_velocity.y / vel_len));
}

// ─── Private: helpers ─────────────────────────────────────────────

Matrix3f AHRS::getRotationMatFromQuat() const {
    Quaternion q;
    Matrix3f   rot;
    q.from_euler(imu.rad_roll, imu.rad_pitch, imu.rad_yaw);
    q.rotation_matrix(rot);
    return rot;
}

Vector3f AHRS::getNormAccel() {
    accel_bf = accel_ef - accel_gravity;
    return accel_bf;
}

// ─── Public: init & update ────────────────────────────────────────

void AHRS::init_ahrs() {
    imu.init_imu();
    gepees.initGPS();
    arspd.initAirspeed();
    baro.init_baro();
}

void AHRS::update_ahrs() {
    dcm_matrix = getRotationMatFromQuat();
    accel_ef   = dcm_matrix * imu._accel;

    driftCorrection();
    updateAOASSA();
}

// void AHRS::threadAHRS() {
//     threads.addThread([]() { IMU_thread(); }, 2);
//     threads.addThread([]() { BARO_thread(); }, 1);
//     threads.addThread([]() { GPS_thread(); });
//     threads.addThread([]() { ARSPD_thread(); }, 1);
// }