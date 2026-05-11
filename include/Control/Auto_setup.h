#pragma once
#ifndef AUTO_SETUP_H
#define AUTO_SETUP_H

#include <Locations.h>
// #include "FW_config.h"
#include <AP_Math.h>
#include "FW_ControlModes.h"
// #include "Radio.h"

// =============================================================================
// Flight mode flags
// =============================================================================
extern bool auto_throttle_mode;
extern bool auto_navigation_mode;
extern bool auto_landing_mode;
extern bool auto_loiter_mode;
extern bool auto_takeoff_mode;
extern bool safety_mode;
extern bool stabilize_lock;

// =============================================================================
// General state variables
// =============================================================================
extern float target_airspeed;
extern float relative_alt;
extern int   flag_wp;
extern int   flag_land;
extern int   flag_takeoff;
extern int   wp_sum;
extern int   land_sum;
extern int   takeoff_sum;

// =============================================================================
// External setpoint from Raspberry Pi (MAVLink SET_POSITION_TARGET_*)
// =============================================================================
extern bool      rpi_external_setpoint_active;
extern uint32_t  rpi_external_setpoint_last_ms;
extern Locations rpi_external_setpoint_target;
extern float     rpi_external_setpoint_yaw_deg;
extern bool      rpi_external_setpoint_yaw_valid;

// =============================================================================
// Waypoint / location arrays & objects
// =============================================================================
extern Locations waypoint[100];
// extern Locations landing[1];  // conflict declaration
extern Locations _takeoff[1];   // conflict declaration

extern Locations prev_WP_loc;   ///< previous waypoint
extern Locations current_loc;   ///< current plane location
extern Locations next_WP_loc;   ///< next waypoint
extern Locations home_wp_loc;   ///< Home/Launch waypoint

extern float acceptance_distance_m;

// =============================================================================
// Misc scalars
// =============================================================================
extern int   value;
extern float value2;
extern float landing_pitch_degree;
extern float takeoff_pitch_degree;

// =============================================================================
// FBWB state
// =============================================================================
struct FbwbState
{
    int8_t climb_rate;
    float  airspeed;
};
extern FbwbState fbwb_state;

// =============================================================================
// Auto (navigation) state
// =============================================================================
struct AutoState
{
    /** should we crosstrack on the next wp? */
    bool     next_wp_crosstrack : 1;
    bool     crosstrack         : 1;
    float    distance_next_wp;
    float    next_wp_altitude;
    float    bearing;
    float    wp_proportion;
    float    next_turn_angle;
    Vector3f forced_rpy;
    float    baro_takeoff_alt;
};
extern AutoState auto_state;

// =============================================================================
// Auto-takeoff state
// =============================================================================
struct AutoTakeoffState
{
    float    highest_airspeed;
    int32_t  target_altitude;
    int8_t   pitch_limit_takeoff;
    bool     complete;
    bool     doing;
    int32_t  throttle;
    uint32_t now;
    uint32_t dtime;
    float    head_setpoint;
    float    nav_head;
    uint32_t last_tkoff_arm_time;
    uint32_t last_check_ms;
    uint32_t rudder_takeoff_warn_ms;
    uint32_t last_report_ms;
    bool     launchTimerStarted;
    uint8_t  accel_event_counter;
    uint32_t accel_event_ms;
    uint32_t start_time_ms;
    bool     waiting_for_rudder_neutral;
    bool     ready_for_throttle;
};
extern AutoTakeoffState auto_takeoff_state;

// =============================================================================
// Altitude control state
// =============================================================================
struct TargetAltitude
{
    /** target altitude above sea level in cm (barometric navigation) */
    float    amsl_cm;
    /** target altitude from ground level in cm (Lidar) */
    float    ground_alt_lidar_cm;
    /** altitude difference between prev and current waypoint in cm (glide slope) */
    int32_t  offset_cm;
    /** last input for FBWB/CRUISE height control */
    float    last_elevator_input;
    /** last time we checked for pilot control of height */
    uint32_t last_elev_check_us;
};
extern TargetAltitude target_altitude;

// =============================================================================
// Attitude / navigation limits
// =============================================================================
extern float max_yaw_angle;
extern float roll_limit_deg;
extern float pitch_limit_min;
extern float pitch_limit_max;
extern float airspeed_min;

// =============================================================================
// Loiter state
// =============================================================================
struct LoiterState
{
    Locations center;            ///< center of loiter

    uint16_t  wp_radius;         ///< loiter wp radius
    uint16_t  in_loit_rad;
    float     loiter_radius;     ///< loiter radius

    Locations wp_loiter[100];
    int       num_wp_loiter;
    int       flag_wp;

    int32_t  old_target_bearing_cd; ///< previous target bearing, used to update sum_cd
    int32_t  sum_cd;                ///< total angle completed in the loiter so far
    int8_t   direction;             ///< 1 = clockwise, -1 = counter-clockwise
    uint32_t start_time_ms;         ///< start time of the loiter (ms)
    uint32_t time_max_ms;           ///< how long to stay in loiter (ms)
    int      wp_needed;
    int      counter;
};
extern LoiterState loiter;

// =============================================================================
// Function declarations
// =============================================================================

/**
 * Returns the height above field elevation (AFE).
 * Passes rangefinder data to TECS during landing so it can detect the flare.
 */
float tecs_hgt_afe(void);

#endif // AUTO_SETUP_H