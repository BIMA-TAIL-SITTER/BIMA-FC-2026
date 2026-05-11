#ifndef FW_CONTROL_FBWA_H
#define FW_CONTROL_FBWA_H

#include <Arduino.h>
#include "../Telemetry/Radio.h"
#include "../Sensors/AHRS.h"

// =============================================================================
// Gain parameters for fixed-wing controller
// =============================================================================
struct FW_gain
{
    // Rate PID + Imax — Roll
    float rate_roll_P    = 10.00f;   // 2.50f
    float rate_roll_I    =  0.00f;   // 0.01f
    float rate_roll_D    =  0.00f;   // 0.13f
    float rate_roll_IMAX = 500.0f;

    // Rate PID + Imax — Pitch
    float rate_pitch_P    = 10.00f;  // 2.50f
    float rate_pitch_I    =  0.00f;  // 0.01f
    float rate_pitch_D    =  0.20f;  // 0.13f
    float rate_pitch_IMAX = 500.0f;

    // Rate PID + Imax — Yaw
    float rate_yaw_P    =  0.00f;   // 20.0f
    float rate_yaw_I    =  0.00f;   // 0.01f
    float rate_yaw_D    =  0.00f;   // 1.1581
    float rate_yaw_IMAX =  8.00f;

    // Rate PID — Throttle
    float rate_thr_P =  35.00f;     // 20.0f
    float rate_thr_I =   0.00f;     // 0.01f
    float rate_thr_D =   0.00f;     // 1.1581
};

// =============================================================================
// Fixed-wing controller class
// =============================================================================
class FW_CONTROL
{
public:
    /** Control surface selector matrix [servo][roll/pitch/yaw] */
    signed char selector[4][3] = {
        { 1,  0,  0},   // aileron right
        { 1,  0,  0},   // aileron left
        { 0,  1,  0},   // elevator  (trainer +, wahana -)
        { 0,  0,  1}    // rudder
    };

    /**
     * Fly-by-wire A (FBWA) update — pilot RC input mapped to stabilised
     * angle commands, then rate-PID drives surfaces.
     */
    void updateFBWA_FW(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                       float roll, float pitch, float yaw,
                       float gy, float gx, float gz);

    /**
     * Auto mode update — navigation demands replace RC angle commands.
     */
    void updateAUTO_FW(float roll, float pitch, float yaw,
                       float gy, float gx, float gz);

    /** Returns auto-computed throttle PWM value. */
    int get_auto_throttle();

    FW_gain fw_gain;
    int servos[4];
    int auto_throttle;

private:
    /** Integrator update (roll & pitch). */
    void getIntegral(int16_t ch_thr, float roll, float pitch, float yaw);

    float u_roll  = 0.0f;
    float u_pitch = 0.0f;
    float u_yaw   = 0.0f;

    unsigned long calc_time      = 0;
    unsigned long last_calc_time = 0;
    unsigned long tnow           = 0;
    unsigned long tbefore        = 0;
    float tdelta                 = 0.0f;
    float delta_calc_time        = 0.0f;

    float roll_cmd   = 0.0f;
    float pitch_cmd  = 0.0f;
    float yaw_cmd    = 0.0f;
    float roll_int   = 0.0f;
    float pitch_int  = 0.0f;

    float min_roll   = -35.0f;
    float max_roll   =  35.0f;
    float min_pitch  = -35.0f;
    float max_pitch  =  35.0f;

    int min_roll_corr  = -512;
    int max_roll_corr  =  512;
    int min_pitch_corr = -512;
    int max_pitch_corr =  512;

    float trim_roll   = 0.0f;
    float trim_pitch  = 0.0f;
    float trim_yaw    = 0.0f;

    float demanded_pitch     =   0.0f;
    float demanded_pitch_min = -20.0f;
    float demanded_pitch_max =  20.0f;
    float demanded_roll_min  = -40.0f;
    float demanded_roll_max  =  40.0f;

    float auto_throttle_min = 1200.0f;
    float auto_throttle_max = 1700.0f;
};

// =============================================================================
// Global instance
// =============================================================================
extern FW_CONTROL fw_control;

#endif // FW_CONTROL_FBWA_H