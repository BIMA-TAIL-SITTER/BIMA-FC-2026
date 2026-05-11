#include "Telemetry.h"

#include "Control/Auto_setup.h"    // auto_state, auto_throttle_mode, auto_navigation_mode,
                                   // target_altitude, target_airspeed, flag_wp, wp_sum,
                                   // current_loc, next_WP_loc, acceptance_distance_m
#include "Sensors/AHRS.h"          // imu, ahrs, baro, arspd, gepees
#include "Radio.h"        // ch_roll, ch_pitch, ch_yaw, ch_throttle, arming
#include "Actuator/Actuator.h"     // pwm_ail_R, pwm_ele, pwm_rud_R, motor_pwm,
                                   // m1_pwm..m4_pwm, outroll
#include "Control/Mode_Manager.h"  // modeCode4(), mode_phase
#include "Control/Navigation.h"    // nav_roll_cd(), nav_roll_deg, nav_pitch_deg,
                                   // get_pitch_demand(), get_throttle_demand(),
                                   // _crosstrack_error, _bearing_error
#include "Control/FW_control.h"    // gain (copter gains — sesuaikan jika beda)
#include "Control/flywing_control.h" // flywingcontrol
#include "Actuator/Voltage.h"      // display_batt()
#include "Communication/WP_EEPROM.h" // display_wp(), scaleToAngle()

// =============================================================================
// Global state
// =============================================================================
int linee      = 0;
int milisawall = 0;

// =============================================================================
// Telemetry::pid_gain_tuning
// =============================================================================
void Telemetry::pid_gain_tuning()
{
    if (Serial2.available()) {
        char selector = Serial2.read();

        switch (selector) {
            case 'Q': case 'q': modifygain(gain.k_alt,         0.25f);  break;
            case 'A': case 'a': modifygain(gain.k_alt,        -0.25f);  break;
            case 'W': case 'w': modifygain(gain.k_z_velocity,  0.25f);  break;
            case 'S': case 's': modifygain(gain.k_z_velocity, -0.25f);  break;
            case 'E': case 'e': modifygain(gain.rate_roll_I,   0.25f);  break;
            case 'D': case 'd': modifygain(gain.rate_roll_D,  -0.25f);  break;
            case 'R': case 'r': modifygain(flywingcontrol.gain_flywing.k_pitch_rate,  0.25f); break;
            case 'F': case 'f': modifygain(flywingcontrol.gain_flywing.k_pitch_rate, -0.25f); break;
            default: break;
        }
    }

    Serial2.print(" |Mode: ");   Serial2.print(modeCode4(current));
    Serial2.print(" |sat: ");    Serial2.print(gepees.satellites);
    Serial2.print(" |head: ");   Serial2.print(imu.heading);
    Serial2.print(" |alt = ");   Serial2.print(baro.altitude);
    Serial2.print(" |aspd = ");  Serial2.print(arspd.v_ms);
    Serial2.print(" wpsum: ");   Serial2.print(wp_sum);
    Serial2.print(" wp: ");      Serial2.print(flag_wp);
    Serial2.print(" dist: ");    Serial2.print(auto_state.distance_next_wp);
    Serial2.print(" m1: ");      Serial2.print(m1_pwm);
    Serial2.print(" m2: ");      Serial2.print(m2_pwm);
    Serial2.print(" m3: ");      Serial2.print(m3_pwm);
    Serial2.print(" m4: ");      Serial2.print(m4_pwm);
    Serial2.println(" ");
}

// =============================================================================
// Telemetry::print_log_fw_telem
// =============================================================================
void Telemetry::print_log_fw_telem()
{
    String desc[29];
    float  data[29];

    desc[0]  = "";          desc[1]  = " phse:"; desc[2]  = " r:";
    desc[3]  = " p:";       desc[4]  = " y:";    desc[5]  = " alt:";
    desc[6]  = " tim:";     desc[7]  = " sp:";   desc[8]  = " droll:";
    desc[9]  = " hdop:";    desc[10] = " fout:";
    desc[11] = " gsp:";     desc[12] = " sat:";  desc[13] = " wp:";
    desc[14] = " NRD:";     desc[15] = " PEL:";  desc[16] = " PTH:";
    desc[17] = " head:";    desc[18] = " dist:"; desc[19] = " lng:";
    desc[20] = " lat:";     desc[21] = " ar:";   desc[22] = " ap:";
    desc[23] = " at:";

    data[0]  = arming;
    data[1]  = mode_phase;
    data[2]  = imu.roll;
    data[3]  = imu.pitch;
    data[4]  = imu.yaw;
    data[5]  = baro.altitude;
    data[6]  = millis();
    data[7]  = scaleToAngle(ch_roll);
    data[8]  = imu.delta_roll;
    data[9]  = gepees.hdop;
    data[10] = outroll;
    data[11] = ahrs.groundspeed;
    data[12] = gepees.satellites;
    data[13] = flag_wp;
    data[14] = nav_roll_deg;
    data[15] = ch_pitch;
    data[16] = motor_pwm;
    data[17] = imu.heading;
    data[18] = current_loc.get_distance(next_WP_loc);
    data[19] = gepees.longitude;
    data[20] = gepees.latitude;
    data[21] = nav_roll_deg;
    data[22] = get_pitch_demand();
    data[23] = get_throttle_demand();

    for (int i = 0; i < 24; i++) {
        Serial2.print(desc[i]);
        if (i < 19) Serial2.print(data[i]);
        else        Serial2.print(data[i], 6);
    }
    Serial2.println();
}

// =============================================================================
// Telemetry::send_vehicle_attitude_data
// =============================================================================
void Telemetry::send_vehicle_attitude_data()
{
    gcs.SendFWBuffer(
        next_WP_loc.alt,
        0,                                          // velocity_target
        current_loc.get_distance(next_WP_loc),
        imu.roll,
        imu.pitch,
        imu.heading,
        baro.altitude,
        arspd.v_ms,                                 // airspeed
        ahrs.groundspeed,
        gepees.latitude,
        gepees.longitude,
        display_batt(),                             // percentage
        modeCode4(current),                         // MODE
        arming,
        mode_phase,
        imu.delta_roll,                             // aileron
        0,                                          // elevator
        0,                                          // rudder
        display_wp(),
        nav_roll_deg,                               // roll_demand
        nav_pitch_deg,                              // pitch_demand
        0,                                          // yaw demand
        next_WP_loc.alt,                            // alt_demand
        get_throttle_demand(),                      // aspd_demand
        outroll,
        0,                                          // pitch_control
        0,                                          // yaw_control
        0,                                          // throttle_control
        scaleToAngle(ch_roll),                      // roll_radio
        ch_pitch,                                   // pitch_radio
        ch_yaw,                                     // yaw_radio
        ch_throttle,                                // throttle_radio
        _crosstrack_error,                          // Xtrack_err
        gepees.hdop,
        gepees.satellites,
        auto_state.next_turn_angle,
        0,                                          // bearing to next wp
        acceptance_distance_m,
        0,                                          // lateral_acceleration()
        0,                                          // climbRate — need implementation
        pwm_ail_R,                                  // ail_as_servo
        pwm_ele,                                    // ele_as_servo
        pwm_rud_R,                                  // rud_as_servo
        ch_throttle,                                // mtr_as_servo
        HEAD_MAX
    );
}

// =============================================================================
// Telemetry::print_tecs_debug
// =============================================================================
void Telemetry::print_tecs_debug()
{
    static uint32_t last_debug_ms = 0;
    uint32_t now = millis();

    // Print setiap 500 ms agar tidak membanjiri serial
    if (now - last_debug_ms < 500) return;
    last_debug_ms = now;

    Serial2.println("\n========== TECS DEBUG INFO ==========");

    // Altitude control
    float altitude_error = (target_altitude.amsl_cm / 100.0f) - baro.altitude;
    Serial2.print("ALT_TARGET: ");  Serial2.print(target_altitude.amsl_cm / 100.0f, 2);
    Serial2.print(" m | ALT_CURRENT: "); Serial2.print(baro.altitude, 2);
    Serial2.print(" m | ALT_ERROR: ");   Serial2.println(altitude_error, 2);

    // Velocity & climb rate
    Serial2.print("TARGET_AIRSPEED: "); Serial2.print(target_airspeed, 2);
    Serial2.print(" m/s | ACTUAL_AIRSPEED: "); Serial2.print(arspd.v_ms, 2);
    Serial2.print(" m/s | CLIMB_RATE: ");
    Serial2.println(baro.get_climb_rate() * 1.0e-3f, 2);   // convert to m/s

    // Control demands
    Serial2.print("PITCH_DEMAND: ");    Serial2.print(get_pitch_demand(), 1);
    Serial2.print(" cd | THROTTLE_DEMAND: "); Serial2.print(get_throttle_demand(), 1);
    Serial2.println(" %");

    // Attitude
    Serial2.print("ROLL: ");  Serial2.print(imu.roll,  2);
    Serial2.print(" deg | PITCH: "); Serial2.print(imu.pitch, 2);
    Serial2.print(" deg | YAW: ");   Serial2.println(imu.yaw, 2);

    // Waypoint info
    Serial2.print("WP_CURRENT: "); Serial2.print(flag_wp);
    Serial2.print(" / ");          Serial2.print(wp_sum);
    Serial2.print(" | WP_DISTANCE: ");
    Serial2.print(current_loc.get_distance(next_WP_loc), 2);
    Serial2.println(" m");

    // Mode & flying state
    Serial2.print("MODE: ");          Serial2.print(modeCode4(current));
    Serial2.print(" | ARMED: ");      Serial2.print(arming);
    Serial2.print(" | AUTO_THROTTLE: "); Serial2.print(auto_throttle_mode);
    Serial2.print(" | AUTO_NAV: ");   Serial2.println(auto_navigation_mode);

    Serial2.println("=====================================\n");
}

// =============================================================================
// Stub implementations (body di-comment di file asli, tetap disertakan)
// =============================================================================
void Telemetry::tuning_gain_pid()    { /* legacy — lihat pid_gain_tuning() */ }
void Telemetry::receive_command_from_gcs() { /* TODO */ }
void Telemetry::print_wp()           { /* TODO */ }
void Telemetry::calibrate_compass()  { /* TODO */ }
void Telemetry::raw_imu()            { /* TODO */ }
void Telemetry::send_telemetry()     { /* TODO */ }