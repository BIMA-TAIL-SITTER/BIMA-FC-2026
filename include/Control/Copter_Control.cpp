#include "Copter_Control.h"

// =============================================================================
// Correction limits
// =============================================================================
int max_roll_corr  =  512;  // 511
int max_pitch_corr =  512;  // 513
int max_yaw_corr   =  512;  // 510
int min_roll_corr  = -512;  // -514
int min_pitch_corr = -512;  // -510
int min_yaw_corr   = -512;  // -517

// =============================================================================
// Frame mixer matrix  (WAHANA TD 2025)
// =============================================================================
// F450 frame konfigurasi x (tidak aktif)
// const double A_invers[4][4] = {
//   {292600,  1300300,  1300300,  6283300},
//   {292600,  1300300, -1300300, -6283300},
//   {292600, -1300300, -1300300,  6283300},
//   {292600, -1300300,  1300300, -6283300}
// };

// WAHANA TD 2025 (aktif)
const double A_invers[4][4] = {
    {1131000,  2381000,  2381000,  1166822},
    {1131000,  2381000, -2381000, -1166822},
    {1131000, -2381000, -2381000,  1166822},
    {1131000, -2381000,  2381000, -1166822}
};

// const double A_invers[4][4] = {
//   {1,  20,  1,  1},
//   {1,  20, -1, -1},
//   {1, -20, -1,  1},
//   {1, -20,  1, -1}
// };

// =============================================================================
// State & parameter
// =============================================================================
float         heading_target    = 0.0f;
float         yaw_error         = 0.0f;
float         yaw_ref           = 0.0f;
float         yaw_offset        = 0.0f;
float         error_roll        = 0.0f;
float         error_pitch       = 0.0f;
unsigned long last_t            = 0;
float         i_yaw_acc         = 0.0f;
float         i_alt_acc         = 0.0f;
float         error_heading     = 0.0f;
float         p_yaw             = 0.0f;
float         d_yaw             = 0.0f;
float         i_yaw             = 0.0f;
float         u1                = 0.0f;
float         u2                = 0.0f;
float         u3                = 0.0f;
float         u4                = 0.0f;
float         w1                = 0.0f;
float         w2                = 0.0f;
float         w3                = 0.0f;
float         w4                = 0.0f;
float         roll_int          = 0.0f;
float         pitch_int         = 0.0f;
unsigned long tnow              = 0;
unsigned long tbefore           = 0;
unsigned long calc_time         = 0;
unsigned long last_calc_time    = 0;
float         delta_calc_time   = 0.0f;
float         alt_ref           = 0.0f;
float         heading_now       = 0.0f;
float         last_alt          = 0.0f;
float         last_heading      = 0.0f;
float         alt_now           = 0.0f;
float         alt_target        = 0.0f;
float         z_velocity        = 0.0f;
float         roll_cmd          = 0.0f;
float         pitch_cmd         = 0.0f;
float         yaw_cmd           = 0.0f;
float         min_roll          = -15.0f;
float         max_roll          =  15.0f;
float         min_pitch         = -35.0f;
float         max_pitch         =  35.0f;
float         min_yaw           = -35.0f;
float         max_yaw           =  35.0f;
int           PID_max_roll      =  400;
int           PID_max_pitch     =  400;
int           PID_max_yaw       =  400;
int           PID_min_roll      = -400;
int           PID_min_pitch     = -400;
int           PID_min_yaw       = -400;
float         omega2[4]         = {};
float         lat_pos           = 0.0f;
float         lon_pos           = 0.0f;
double        roll_pos          = 0.0;
double        pitch_pos         = 0.0;
bool          alt_hold_on       = false;
bool          pilot_has_yaw_input = false;
float         trim_roll         = 0.0f;  // (+) bales kanan, (-) bales kiri
float         trim_pitch        = 0.0f;  // (+) bales maju,  (-) bales mundur
float         trim_yaw          = 0.0f;  // (+) bales kanan, (-) bales kiri
float         tesdata           = 0.0f;
float         tesgz             = 0.0f;
float         tesgx             = 0.0f;

// =============================================================================
// Gains instance
// =============================================================================
gains gain;

// =============================================================================
// Rate loop state
// =============================================================================
float roll_rate_sp  = 0.0f;
float pitch_rate_sp = 0.0f;
float yaw_rate_sp   = 0.0f;
float i_rate_roll   = 0.0f;
float i_rate_pitch  = 0.0f;
float i_rate_yaw    = 0.0f;

// File-scope statics (not exposed in header)
static float    gy_prev      = 0.0f;
static float    gx_prev      = 0.0f;
static float    gz_prev      = 0.0f;
static uint32_t last_us_rate = 0;

// =============================================================================
// copter_getIntegral
// =============================================================================
void copter_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw)
{
    tbefore = tnow;
    tnow    = millis();
    float tdelta = (tnow - tbefore) / 1000.0f;

    // set 0 agar tidak double-control (outer loop P-only)
    roll_int  += 0.0f * error_roll  * tdelta;
    pitch_int += 0.0f * error_pitch * tdelta;

    roll_int  = constrain(roll_int,  -15.0f, 15.0f);
    pitch_int = constrain(pitch_int, -15.0f, 15.0f);
    i_yaw     = constrain(i_yaw,     -20.0f, 20.0f);

    if (ch_thr < 1100) { roll_int = pitch_int = i_yaw = 0.0f; }
}

// =============================================================================
// Error_poshold
// =============================================================================
void Error_poshold(double distance, double z)
{
    // distance cm, z deg (delta heading ke target)
    double e_north = distance * cos(z * DEG_TO_RAD);
    double e_east  = distance * sin(z * DEG_TO_RAD);

    e_north *= gain.k_pos;
    e_east  *= gain.k_pos;

    pitch_pos = constrain(e_north / 100.0, -5.0, 5.0);
    roll_pos  = constrain(e_east  / 100.0, -5.0, 5.0);
}

// =============================================================================
// copter_ControlFSFB
// =============================================================================
void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw,
                        float gy,   float gx,    float gz)
{
    // ===== time step untuk rate PID =====
    uint32_t now_us = micros();
    float dt = (last_us_rate == 0) ? 0.0f : (now_us - last_us_rate) * 1e-6f;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;  // lock I/D jika timing jelek
    last_us_rate = now_us;
    tesdata = last_us_rate;

    // KALAU MAU CEK GYRO YANG DIBALIK:
    // gx = -gx;
    gz = -gz;
    // tesgx = gx;
    // tesgz = gz;

    // ===== altitude =====
    if (alt_hold) {
        if (ch_thr > 1600 && ch_thr < 1800) {
            if (!alt_hold_on) { alt_ref = baro.altitude; alt_hold_on = true; }
            alt_now    = baro.altitude;
            alt_target = alt_now - alt_ref;
            z_velocity = baro.get_climb_rate() / 100.0f;
        } else {
            alt_hold_on = false;
            z_velocity  = 0;
        }
    } else {
        alt_hold_on = false;
        z_velocity  = 0;
    }

    // ===== POS HOLD =====
    if (pos_hold) {
        if (ch_r > 1490 && ch_r < 1510 && ch_p > 1490 && ch_p < 1510) {
            static bool pos_locked = false;
            if (!pos_locked) {
                lat_pos    = gepees.latitude;
                lon_pos    = gepees.longitude;
                pos_locked = true;
            }
            double distance = haversineDistanceCM(gepees.latitude, gepees.longitude, lat_pos, lon_pos);
            double z        = calculateHeading(imu.heading, gepees.latitude, gepees.longitude, lat_pos, lon_pos);
            Error_poshold(distance, z);
        } else {
            static bool pos_locked = true;
            if (pos_locked) {
                pos_locked = false;
                roll_pos   = 0.0;
                pitch_pos  = 0.0;
            }
        }
    } else {
        roll_pos  = 0.0;
        pitch_pos = 0.0;
    }

    // ===== RC → angle command =====
    roll_cmd  = 1.3f * (map(ch_r - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    pitch_cmd = 1.1f * (map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
    yaw_cmd   = 1.3f * (map(ch_y - 1500, min_yaw_corr,   max_yaw_corr,   min_yaw,   max_yaw));

    // koreksi pos-hold
    roll_cmd  += roll_pos;
    pitch_cmd += pitch_pos;

    // ===== error sudut (cmd - meas) =====
    const float e_roll  = (-roll_cmd  + trim_roll)  - roll;
    const float e_pitch = ( pitch_cmd + trim_pitch) - (-pitch);

    // ===== Stabilize outer loop (angle → rate), P-only =====
    roll_rate_sp  = constrain(e_roll  * gain.stab_roll_P,  -gain.max_rate_r, gain.max_rate_r);
    pitch_rate_sp = constrain(e_pitch * gain.stab_pitch_P, -gain.max_rate_p, gain.max_rate_p);

    // Heading-hold yaw (dinonaktifkan, manual saja):
    // const bool yaw_stick = (abs(ch_y - 1500) > YAW_DEADZONE_RC);
    // ...

    // Yaw manual: rate langsung dari stick, tanpa heading-hold
    const bool yaw_stick = (abs(ch_y - 1500) > YAW_DEADZONE_RC);
    if (yaw_stick) {
        yaw_rate_sp = constrain(
            map(ch_y, 1000, 2000, -gain.max_rate_y, gain.max_rate_y),
            -gain.max_rate_y, gain.max_rate_y
        );
    } else {
        yaw_rate_sp = 0.0f;  // stick netral → yaw diam, tidak heading lock
    }

    // ===== Rate PID inner loop (P + I + D_on_meas) =====
    const float er = (roll_rate_sp  - gy);   // gy untuk roll
    const float ep = (pitch_rate_sp - gx);   // gx untuk pitch
    const float ey = (yaw_rate_sp   - gz);   // gz untuk yaw

    const float dgy = (dt > 0) ? (gy - gy_prev) / dt : 0.0f; gy_prev = gy;
    const float dgx = (dt > 0) ? (gx - gx_prev) / dt : 0.0f; gx_prev = gx;
    const float dgz = (dt > 0) ? (gz - gz_prev) / dt : 0.0f; gz_prev = gz;

    const bool allow_I = (dt > 0) && (ch_thr >= 1100);
    if (allow_I) {
        i_rate_roll  += gain.rate_roll_I  * er * dt;
        i_rate_pitch += gain.rate_pitch_I * ep * dt;
        i_rate_yaw   += gain.rate_yaw_I   * ey * dt;

        i_rate_roll  = constrain(i_rate_roll,  -gain.rate_roll_IMAX,  gain.rate_roll_IMAX);
        i_rate_pitch = constrain(i_rate_pitch, -gain.rate_pitch_IMAX, gain.rate_pitch_IMAX);
        i_rate_yaw   = constrain(i_rate_yaw,   -gain.rate_yaw_IMAX,   gain.rate_yaw_IMAX);
    } else {
        i_rate_roll = i_rate_pitch = i_rate_yaw = 0.0f;
    }

    // output inner-loop (skala tetap)
    u2 = (gain.rate_roll_P  * er + i_rate_roll  - gain.rate_roll_D  * dgy) / 10'000'000.0f;
    u3 = (gain.rate_pitch_P * ep + i_rate_pitch - gain.rate_pitch_D * dgx) / 10'000'000.0f;
    u4 = (gain.rate_yaw_P   * ey + i_rate_yaw   - gain.rate_yaw_D   * dgz) / 10'000'000.0f;

    // ===== Throttle / altitude =====
    u1 = (-gain.k_alt * (alt_target / 1.000f)
          + (-gain.k_z_velocity * (z_velocity) / 100.0f)) / 10'000'000.0f;

    // ===== MIXER =====
    omega2[0] = A_invers[0][0]*u1 + A_invers[0][1]*u2 + A_invers[0][2]*u3 + A_invers[0][3]*u4;
    omega2[1] = A_invers[1][0]*u1 + A_invers[1][1]*u2 + A_invers[1][2]*u3 + A_invers[1][3]*u4;
    omega2[2] = A_invers[2][0]*u1 + A_invers[2][1]*u2 + A_invers[2][2]*u3 + A_invers[2][3]*u4;
    omega2[3] = A_invers[3][0]*u1 + A_invers[3][1]*u2 + A_invers[3][2]*u3 + A_invers[3][3]*u4;

    last_heading = imu.heading;
}