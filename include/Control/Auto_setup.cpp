#include "Auto_setup.h"

// =============================================================================
// Flight mode flags
// =============================================================================
bool auto_throttle_mode   = false;
bool auto_navigation_mode = false;
bool auto_landing_mode    = false;
bool auto_loiter_mode     = false;
bool auto_takeoff_mode    = false;
bool safety_mode          = false;
bool stabilize_lock       = false;

// =============================================================================
// General state variables
// =============================================================================
float target_airspeed = 0.0f;
float relative_alt    = 0.0f;
int   flag_wp         = -1;
int   flag_land       = -1;
int   flag_takeoff    = -1;
int   wp_sum          =  0;
int   land_sum        =  1;
int   takeoff_sum     =  1;

// =============================================================================
// External setpoint from Raspberry Pi (MAVLink SET_POSITION_TARGET_*)
// =============================================================================
bool      rpi_external_setpoint_active    = false;
uint32_t  rpi_external_setpoint_last_ms   = 0;
Locations rpi_external_setpoint_target    = {};
float     rpi_external_setpoint_yaw_deg   = 0.0f;
bool      rpi_external_setpoint_yaw_valid = false;

// =============================================================================
// Waypoint / location arrays & objects
// =============================================================================
Locations waypoint[100];
// Locations landing[1];   // conflict declaration
Locations _takeoff[1];

Locations prev_WP_loc = {};  ///< previous waypoint
Locations current_loc = {};  ///< current plane location
Locations next_WP_loc = {};  ///< next waypoint
Locations home_wp_loc = {};  ///< Home/Launch waypoint

float acceptance_distance_m = 0.0f;

// =============================================================================
// Misc scalars
// =============================================================================
int   value                = 0;
float value2               = 0.0f;
float landing_pitch_degree = 0.0f;
float takeoff_pitch_degree = 0.0f;

// =============================================================================
// FBWB state
// =============================================================================
FbwbState fbwb_state = {
    .climb_rate = FBWB_CLIMB_RATE,
    .airspeed   = MIN_AIRSPEED
};

// =============================================================================
// Auto (navigation) state
// =============================================================================
AutoState auto_state = {};

// =============================================================================
// Auto-takeoff state
// =============================================================================
AutoTakeoffState auto_takeoff_state = {
    .highest_airspeed    = 0.0f,
    .target_altitude     = 29 * M_TO_CM,
    .pitch_limit_takeoff = 15,
    .complete            = false,
    .doing               = false,
    .throttle            = 0,
    .ready_for_throttle  = false
    // remaining members zero/false-initialised by default
};

// =============================================================================
// Altitude control state
// =============================================================================
TargetAltitude target_altitude = {};

// =============================================================================
// Attitude / navigation limits
// =============================================================================
float max_yaw_angle   = 45.0f;
float roll_limit_deg  = HEAD_MAX;
float pitch_limit_min = PITCH_MIN;
float pitch_limit_max = PITCH_MAX;
float airspeed_min    = MIN_AIRSPEED;

// =============================================================================
// Loiter state
// =============================================================================
LoiterState loiter = {
    .wp_radius   = WP_RADIUS_DEFAULT,
    .in_loit_rad = WP_RADIUS_DEFAULT,  // original: in_loit_rad = wp_radius
    .sum_cd      = 0,
    .direction   = -1,
    .flag_wp     = 0,
    .counter     = 0
    // remaining members zero/default-initialised
};

// =============================================================================
// Function implementations
// =============================================================================

/**
 * Returns the height above field elevation (AFE).
 *
 * During landing, TECS receives rangefinder data so it can detect the
 * flare point. In normal flight the height is simply relative altitude
 * to home.
 */
float tecs_hgt_afe(void)
{
    float hgt_afe;
    // when in normal flight we pass the hgt_afe as relative altitude to home
    hgt_afe = relative_alt;
    return hgt_afe;
}