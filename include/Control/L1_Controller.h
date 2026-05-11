#pragma once

#include "Arduino.h"
#include "../Sensors/AHRS.h"
#include "../lib/Math/AP_Math.h"
#include "../lib/Common/Locations.h"
#include "../lib/Common/Common.h"
#include "definitions.h"
#include "../Sensors/gps.h"
#include "Auto_setup.h"
#include "../Sensors/bno055.h"
#include "../lib/Math/vector2.h"

extern float grspd;

/*
 * @file    L1_Controller.h
 * @brief   L1 Control algorithm.
 * Originally written by Brandon Jones 2013
 * Modified by Paul Riseborough 2013
 */

class AP_L1_Control {
public:
    // Tuning parameters
    float _L1_period            = 22.0f;
    float _L1_damping           = 0.73f;
    float _L1_xtrack_i_gain     = 0.2f;
    float _loiter_bank_limit    = 45.0f;

    // Navigation outputs
    int32_t nav_roll_cd(void);
    float   lateral_acceleration(void);
    int32_t nav_bearing_cd(void);
    int32_t bearing_error_cd(void);
    int32_t target_bearing_cd(void);
    float   turn_distance(float wp_radius);
    float   turn_distance(float wp_radius, float turn_angle);
    float   loiter_radius(float radius);
    bool    reached_loiter_target(void);

    // Control update functions
    void update_waypoint(const Locations &prev_WP, const Locations &next_WP, float dist_min);
    void update_loiter(const Locations &center_WP, float radius, int8_t loiter_direction);
    void update_heading_hold(int32_t navigation_heading_cd);
    void update_level_flight(void);

    // State management
    void set_default_period(float period) { _L1_period = period; }
    void set_data_is_stale(void)          { _data_is_stale = true; }
    bool data_is_stale(void)              { return _data_is_stale; }
    void set_reverse(bool reverse)        { _reverse = reverse; }

private:
    float    _L1_xtrack_i_gain_prev = 0.0f;
    uint32_t _last_update_waypoint_us = 0;
    bool     _data_is_stale = true;

    float   _latAccDem;
    float   _L1_dist;
    bool    _WPcircle;
    float   _nav_bearing;
    float   _bearing_error;
    float   _crosstrack_error;
    int32_t _target_bearing_cd;
    float   _last_Nu;
    float   _L1_xtrack_i;
    bool    _reverse = false;

    float   get_yaw(void);
    int32_t get_yaw_sensor(void);
    void    _prevent_indecision(float &Nu);
};