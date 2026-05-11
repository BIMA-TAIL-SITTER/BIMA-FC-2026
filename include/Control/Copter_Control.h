#ifndef COPTER_CONTROL_H
#define COPTER_CONTROL_H

#include <Arduino.h>
#include "../Sensors/AHRS.h"
#include "ControlModes.h"
#include "Copter_Config.h"
#include "../Telemetry/Radio.h"
#include "bno055.h"
#include "pos_hold.h"
#include "../Communication/TimingUtils.h"

// =============================================================================
// Correction limits
// =============================================================================
extern int max_roll_corr;
extern int max_pitch_corr;
extern int max_yaw_corr;
extern int min_roll_corr;
extern int min_pitch_corr;
extern int min_yaw_corr;

// =============================================================================
// Frame mixer matrix  (WAHANA TD 2025)
// F450 frame konfigurasi-x (commented out), WAHANA TD 2025 aktif
// =============================================================================
extern const double A_invers[4][4];

// =============================================================================
// State & parameter
// =============================================================================
extern float         heading_target;
extern float         yaw_error, yaw_ref, yaw_offset;
extern float         error_roll, error_pitch;
extern unsigned long last_t;
extern float         i_yaw_acc, i_alt_acc;
extern float         error_heading;
extern float         p_yaw, d_yaw, i_yaw;
extern float         u1, u2, u3, u4;
extern float         w1, w2, w3, w4;
extern float         roll_int, pitch_int;
extern unsigned long tnow, tbefore;
extern unsigned long calc_time, last_calc_time;
extern float         delta_calc_time;
extern float         alt_ref, heading_now, last_alt, last_heading, alt_now;
extern float         alt_target, z_velocity;
extern float         roll_cmd, pitch_cmd, yaw_cmd;
extern float         min_roll;
extern float         max_roll;
extern float         min_pitch;
extern float         max_pitch;
extern float         min_yaw;
extern float         max_yaw;
extern int           PID_max_roll;
extern int           PID_max_pitch;
extern int           PID_max_yaw;
extern int           PID_min_roll;
extern int           PID_min_pitch;
extern int           PID_min_yaw;
extern float         omega2[4];
extern float         lat_pos, lon_pos;
extern double        roll_pos, pitch_pos;
extern bool          alt_hold_on;
extern bool          pilot_has_yaw_input;
extern float         trim_roll;
extern float         trim_pitch;
extern float         trim_yaw;
extern float         tesdata;
extern float         tesgz, tesgx;

// =============================================================================
// Gains struct
// =============================================================================
struct gains
{
    float k_alt        = 6.50f;
    float k_z_velocity = 3.0f;
    float k_z_vel      = 1.0f;
    float k_pos        = 1.0f;
    int16_t roll_rmt, pitch_rmt, yaw_rmt;

    // Stabilize (angle → rate), P-only
    float stab_roll_P  = 4.5f;
    float stab_pitch_P = 4.0f;
    float stab_yaw_P   = 5.0f;

    // Rate PID + Imax
    float rate_roll_P    = 5.00f;
    float rate_roll_I    = 0.01f;
    float rate_roll_D    = 0.05f;
    float rate_roll_IMAX = 500.0f;

    float rate_pitch_P    = 5.0f;
    float rate_pitch_I    = 0.01f;
    float rate_pitch_D    = 0.05f;
    float rate_pitch_IMAX = 500.0f;

    float rate_yaw_P    = 30.00f;
    float rate_yaw_I    = 0.01f;
    float rate_yaw_D    = 0.05f;
    float rate_yaw_IMAX = 8.0f;

    // Batas rate target (deg/s)
    float max_rate_r = 220.0f;
    float max_rate_p = 220.0f;
    float max_rate_y = 150.0f;
};
extern gains gain;

// =============================================================================
// Rate loop state
// =============================================================================
extern float    roll_rate_sp;
extern float    pitch_rate_sp;
extern float    yaw_rate_sp;
extern float    i_rate_roll;
extern float    i_rate_pitch;
extern float    i_rate_yaw;

// (file-scope statics — defined in .cpp, not exposed as extern)

// =============================================================================
// Constants
// =============================================================================
static constexpr int YAW_DEADZONE_RC = 15;

// =============================================================================
// Function declarations
// =============================================================================

/**
 * Integrator update for roll/pitch angle loop.
 * @param ch_thr  throttle channel raw value
 * @param roll    current roll angle  (deg)
 * @param pitch   current pitch angle (deg)
 * @param yaw     current yaw angle   (deg)
 */
void copter_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw);

/**
 * Compute position-hold angle corrections from GPS distance error.
 * @param distance  horizontal distance to target (cm)
 * @param z         bearing delta to target (deg)
 */
void Error_poshold(double distance, double z);

/**
 * Full-state feedback copter controller.
 * Runs stabilize (angle→rate) outer loop + rate PID inner loop + mixer.
 */
void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw,
                        float gy,   float gx,    float gz);

#endif // COPTER_CONTROL_H