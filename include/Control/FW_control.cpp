#include "FW_control.h"
#include "Auto_setup.h"   // auto_state, nav_roll_cd(), get_pitch_demand()

// =============================================================================
// Global instance
// =============================================================================
FW_CONTROL fw_control;

// =============================================================================
// FW_CONTROL::getIntegral  (private)
// =============================================================================
void FW_CONTROL::getIntegral(int16_t ch_thr, float roll, float pitch, float yaw)
{
    tbefore = tnow;
    tnow    = millis();
    tdelta  = (tnow - tbefore) / 1000.0f;

    roll_int  += fw_gain.rate_roll_I  * roll  * tdelta;
    pitch_int += fw_gain.rate_pitch_I * pitch * tdelta;

    roll_int  = constrain(roll_int,  -15.0f, 15.0f);
    pitch_int = constrain(pitch_int, -15.0f, 15.0f);

    // Reset integrators when throttle is low (disarmed / idle)
    if (ch_thr < 1100) {
        roll_int  = 0.0f;
        pitch_int = 0.0f;
    }
}

// =============================================================================
// FW_CONTROL::updateFBWA_FW
// =============================================================================
void FW_CONTROL::updateFBWA_FW(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                                float roll, float pitch, float yaw,
                                float gy,   float gx,    float gz)
{
    // last_calc_time = calc_time;
    // calc_time      = micros();
    // delta_calc_time = (calc_time - last_calc_time) / 1000000.0f;

    getIntegral(ch_thr, roll, pitch, yaw);

    // RC → angle command
    roll_cmd  = 1.3f * (map(ch_r - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    pitch_cmd = 1.1f * (map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
    // yaw_cmd = 1.0f * (map(ch_y - 1500, min_roll_corr, max_roll_corr, min_roll, max_roll));

    // Rate controller outputs
    u_roll  = (-fw_gain.rate_roll_P  * (-roll  + roll_int  - ( roll_cmd + trim_roll)))
              + (-fw_gain.rate_roll_D  * gy);

    u_pitch = ( fw_gain.rate_pitch_P * (-pitch + pitch_int - (-pitch_cmd + trim_pitch)))
              + (-fw_gain.rate_pitch_D * gx);

    // Surface mixer
    servos[0] = selector[0][0]*u_roll + selector[0][1]*u_pitch + selector[0][2]*u_yaw; // aileron right
    servos[1] = selector[1][0]*u_roll + selector[1][1]*u_pitch + selector[1][2]*u_yaw; // aileron left
    servos[2] = selector[2][0]*u_roll + selector[2][1]*u_pitch + selector[2][2]*u_yaw; // elevator
    servos[3] = selector[3][0]*u_roll + selector[3][1]*u_pitch + selector[3][2]*u_yaw; // rudder

    servos[0] = constrain(servos[0], -400, 400);
    servos[1] = constrain(servos[1], -400, 400);
    servos[2] = constrain(servos[2], -400, 400);
    servos[3] = constrain(servos[3], -400, 400);

    // Debug prints (disabled)
    // Serial.print("Roll :");   Serial.print(roll);
    // Serial.print(" |Pitch:"); Serial.print(pitch);
    // Serial.print(" |u_roll:"); Serial.print(u_roll);
    // Serial.print(" |u_pitch:"); Serial.print(u_pitch);
    // Serial.print(" |ail_L:"); Serial.print(servos[1]);
    // Serial.print(" |ail_R:"); Serial.print(servos[0]);
    // Serial.print(" |ele:");   Serial.print(servos[2]);
    // Serial.print(" |rud:");   Serial.println(servos[3]);
}

// =============================================================================
// FW_CONTROL::updateAUTO_FW
// =============================================================================
void FW_CONTROL::updateAUTO_FW(float roll, float pitch, float yaw,
                                float gy,   float gx,    float gz)
{
    int ch_thr = get_auto_throttle();
    getIntegral(ch_thr, roll, pitch, yaw);

    demanded_pitch = constrain(get_pitch_demand(), demanded_pitch_min, demanded_pitch_max);

    // Navigation demands replace RC commands
    roll_cmd  = 1.3f * constrain(nav_roll_cd() / 100.0f, demanded_roll_min,  demanded_roll_max);
    pitch_cmd = 1.1f * demanded_pitch;

    u_roll  = (-fw_gain.rate_roll_P  * (-roll  + roll_int  - ( roll_cmd + trim_roll)))
              + (-fw_gain.rate_roll_D  * gy);

    u_pitch = ( fw_gain.rate_pitch_P * (-pitch + pitch_int - (-pitch_cmd + trim_pitch)))
              + (-fw_gain.rate_pitch_D * gx);

    // Surface mixer
    servos[0] = selector[0][0]*u_roll + selector[0][1]*u_pitch + selector[0][2]*u_yaw; // aileron right
    servos[1] = selector[1][0]*u_roll + selector[1][1]*u_pitch + selector[1][2]*u_yaw; // aileron left
    servos[2] = selector[2][0]*u_roll + selector[2][1]*u_pitch + selector[2][2]*u_yaw; // elevator
    servos[3] = selector[3][0]*u_roll + selector[3][1]*u_pitch + selector[3][2]*u_yaw; // rudder

    servos[0] = constrain(servos[0], -400, 400);
    servos[1] = constrain(servos[1], -400, 400);
    servos[2] = constrain(servos[2], -400, 400);
    servos[3] = constrain(servos[3], -400, 400);
}

// =============================================================================
// FW_CONTROL::get_auto_throttle
// =============================================================================
int FW_CONTROL::get_auto_throttle()
{
    // P-controller: throttle proportional to distance to next waypoint
    auto_throttle = static_cast<int>(fw_gain.rate_thr_P * auto_state.distance_next_wp);
    auto_throttle = constrain(auto_throttle,
                              static_cast<int>(auto_throttle_min),
                              static_cast<int>(auto_throttle_max));
    return auto_throttle;
}