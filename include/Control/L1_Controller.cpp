#include "L1_Controller.h"

float grspd;

float AP_L1_Control::get_yaw(void) {
    if (_reverse) {
        return wrap_PI(M_PI + imu.yaw);
    }
    return imu.rad_yaw;
}

int32_t AP_L1_Control::get_yaw_sensor(void) {
    if (_reverse) {
        return wrap_180_cd(18000 + imu.yaw);
    }
    return imu.yaw * 100;
}

int32_t AP_L1_Control::nav_roll_cd(void) {
    float pitchLimL1 = radians(60.0f);
    float pitchL1    = constrain(imu.rad_pitch, -pitchLimL1, pitchLimL1);
    float ret = degrees(atanf(_latAccDem * (1.0f / (GRAVITY_MSS * cosf(pitchL1))))) * 100.0f;
    ret = constrain(ret, -9000, 9000);
    return ret;
}

float AP_L1_Control::lateral_acceleration(void) {
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) {
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) {
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) {
    return wrap_180_cd(_target_bearing_cd);
}

float AP_L1_Control::turn_distance(float wp_radius) {
    wp_radius *= sq(baro.getEAS2TAS());
    return MIN(wp_radius, _L1_dist);
}

float AP_L1_Control::turn_distance(float wp_radius, float turn_angle) {
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90.0f) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_L1_Control::loiter_radius(float radius) {
    float sanitized_bank_limit    = constrain(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;
    float nominal_velocity_sea_level = get_target_airspeed();
    float eas2tas_sq = sq(baro.getEAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            return radius * eas2tas_sq;
        } else {
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void) {
    return _WPcircle;
}

void AP_L1_Control::_prevent_indecision(float &Nu) {
    float Nu_limit = 0.9f * M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        Nu = _last_Nu;
    }
}

void AP_L1_Control::update_waypoint(const Locations &prev_WP, const Locations &next_WP, float dist_min) {
    Locations _current_loc;
    float Nu, xtrackVel, ltrackVel;

    uint32_t now = micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 1.0f) {
        _L1_xtrack_i = 0.0f;
    }
    if (dt > 0.1f) dt = 0.1f;
    _last_update_waypoint_us = now;

    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    if (ahrs.get_position(_current_loc) == false) {
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = {ahrs.get_velocity_ned().x, ahrs.get_velocity_ned().y};
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    float groundSpeed = ahrs.groundspeed;
    grspd = _groundspeed_vector.angle();

    bool moving_forwards = fabsf(wrap_PI(_groundspeed_vector.angle() - get_yaw())) < M_PI_2;
    _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;

    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    float AB_length = AB.length();

    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);
    _crosstrack_error = A_air % AB;

    float WP_A_dist      = A_air.length();
    float alongTrackDist = A_air * AB;

    if (WP_A_dist > _L1_dist && alongTrackDist / MAX(WP_A_dist, 1.0f) < -0.7071f) {
        Vector2f A_air_unit = (A_air).normalized();
        xtrackVel   = _groundspeed_vector % (-A_air_unit);
        ltrackVel   = _groundspeed_vector * (-A_air_unit);
        Nu          = atan2f(xtrackVel, ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y, -A_air_unit.x);

    } else if (alongTrackDist > AB_length + groundSpeed * 3.0f) {
        const Vector2f B_air      = next_WP.get_distance_NE(_current_loc);
        Vector2f       B_air_unit = (B_air).normalized();
        xtrackVel   = _groundspeed_vector % (-B_air_unit);
        ltrackVel   = _groundspeed_vector * (-B_air_unit);
        Nu          = atan2f(xtrackVel, ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y, -B_air_unit.x);

    } else {
        xtrackVel     = _groundspeed_vector % AB;
        ltrackVel     = _groundspeed_vector * AB;
        float Nu2     = atan2f(xtrackVel, ltrackVel);
        float sine_Nu1 = _crosstrack_error / MAX(_L1_dist, 0.1f);
        sine_Nu1      = constrain(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1     = asinf(sine_Nu1);

        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain, _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i          = 0.0f;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5.0f)) {
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;
            _L1_xtrack_i  = constrain(_L1_xtrack_i, -0.1f, 0.1f);
        }

        Nu1 += _L1_xtrack_i;
        Nu   = Nu1 + Nu2;
        _nav_bearing = wrap_PI(atan2f(AB.y, AB.x) + Nu1);
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu        = constrain(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    _WPcircle  = false;
    _bearing_error = Nu;
    _data_is_stale = false;
}

void AP_L1_Control::update_loiter(const Locations &center_WP, float radius, int8_t loiter_direction) {
    Locations _current_loc;

    radius = loiter_radius(fabsf(radius));

    float omega = (6.2832f / _L1_period);
    float Kx    = omega * omega;
    float Kv    = 2.0f * _L1_damping * omega;
    float K_L1  = 4.0f * _L1_damping * _L1_damping;

    if (ahrs.get_position(_current_loc) == false) {
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = {ahrs.get_velocity_ned().x, ahrs.get_velocity_ned().y};
    float groundSpeed = MAX(_groundspeed_vector.length(), 1.0f);

    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {