// include/Actuator/Actuator.cpp
#include "Actuator.h"

// ─────────────────────────────────────────────
// Servo & motor object definitions
// ─────────────────────────────────────────────
Servo motor1, motor2, motor3, motor4;
Servo aileron_L;
Servo aileron_R;
Servo elevator;
Servo rudder_L;
Servo rudder_R;
Servo pusher;
Servo payload;
Servo mtr_1;
Servo mtr_2;

// ─────────────────────────────────────────────
// Payload drop timing
// ─────────────────────────────────────────────
uint32_t payload_drop_start_time = 0;
const uint32_t PAYLOAD_DROP_DURATION = 500; // ms
bool payload_dropping = false;

// ─────────────────────────────────────────────
// Control output variables
// ─────────────────────────────────────────────
float u_roll, u_pitch, u_yaw, u_yaw_cmd;
float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
float servo_L, servo_R;
int   servo_yaw;
float high_out = 90;
int   basePWM  = 1500;

/*NOTE: remote yang dipakai tahun lalu punya koreksi sebesar 12
  jadi semua PWM defaultnya 1000 (min) dan 2000 (max) ditambah/dikurangi 12,
  tapi nilai median PWM tetap 1500. */
uint16_t servo_max = 2012;
uint16_t servo_min = 988;
uint16_t pwm_ail_L;
uint16_t pwm_ail_R;
uint16_t pwm_ele;
uint16_t pwm_rud;
uint16_t pwm_rud_L;
uint16_t pwm_rud_R;
uint16_t throttle;

int32_t commanded_throttle;

// ─────────────────────────────────────────────
// Utility conversions
// ─────────────────────────────────────────────

// Scale PWM → -45 … +45 (servo angle)
float scaleToAngle(uint16_t ch) {
    return 0.0879f * ch - 131.46f;
}

// Scale PWM → 0 … 100 (throttle %)
float scaleToPercent(uint16_t ch) {
    return 0.0976f * ch - 96.067f;
}

float scaleToPercent_reversed(uint16_t ch) {
    return (ch + 100.0f) * (2012 - 988) / 200.0f + 988.0f;
}

// Angle → PWM
uint16_t angleToPwm(float angle) {
    return (uint16_t)(11.378f * angle + 1500);
}

uint16_t angleToPwm_reversed(float angle) {
    return (uint16_t)(11.378f * (-angle) + 1500);
}

float PercenttoPWM(float Percent) {
    return map(Percent, 0, 100, 988, 2012);
}

// ─────────────────────────────────────────────
// Initialisation
// ─────────────────────────────────────────────

void servo_idle() {
    servo_pwm_ail_L  = 0;
    servo_pwm_ail_R  = 0;
    servo_pwm_ele    = 0;
    servo_pwm_rud_L  = 0;
    servo_pwm_rud_R  = 0;
}

void init_actuator() {
    pinMode(SERVO_AIL_L,   OUTPUT);
    pinMode(SERVO_ELE,     OUTPUT);
    pinMode(SERVO_RUD_L,   OUTPUT);
    pinMode(SERVO_AIL_R,   OUTPUT);
    pinMode(SERVO_RUD_R,   OUTPUT);
    pinMode(THROTTLE,      OUTPUT);
    pinMode(MOTOR_1_PIN,   OUTPUT);
    pinMode(MOTOR_2_PIN,   OUTPUT);
    pinMode(MOTOR_3_PIN,   OUTPUT);
    pinMode(MOTOR_4_PIN,   OUTPUT);
    pinMode(SERVO_PAYLOAD, OUTPUT);

    aileron_L.attach(SERVO_AIL_L);
    elevator.attach(SERVO_ELE);
    rudder_L.attach(SERVO_RUD_L);
    aileron_R.attach(SERVO_AIL_R);
    rudder_R.attach(SERVO_RUD_R);
    payload.attach(SERVO_PAYLOAD);
    pusher.attach(THROTTLE);
    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);

    motor1.writeMicroseconds(988);
    motor2.writeMicroseconds(988);
    motor3.writeMicroseconds(988);
    motor4.writeMicroseconds(988);
    pusher.writeMicroseconds(988);
    payload.writeMicroseconds(1000);
    servo_pwm_payload = 1000;

    Serial.println("Actuator setup complete");
}

// ─────────────────────────────────────────────
// Motor outputs
// ─────────────────────────────────────────────

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}

void motor_loop_nyt(int pwm1, int pwm2, int pwm3, int pwm4,
                    int pwm_yaw_l, int pwm_yaw_r) {
    aileron_L.writeMicroseconds(pwm_yaw_l);
    aileron_R.writeMicroseconds(pwm_yaw_r);
}

void motorCopterOff(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
    pusher.writeMicroseconds(988);
}

// ─────────────────────────────────────────────
// Copter control outputs
// ─────────────────────────────────────────────

void copter_calcOutput(int16_t ch_thr) {
    throttle = ch_thr;
    int control1 = constrain((int)(omega2[0] * M_CONST), -400, 400);
    int control2 = constrain((int)(omega2[1] * M_CONST), -400, 400);
    int control3 = constrain((int)(omega2[2] * M_CONST), -400, 400);
    int control4 = constrain((int)(omega2[3] * M_CONST), -400, 400);

    if (arming) {
        m1_pwm = control1 + throttle;
        m2_pwm = control2 + throttle;
        m3_pwm = control3 + throttle;
        m4_pwm = control4 + throttle;
    } else {
        m1_pwm = m2_pwm = m3_pwm = m4_pwm = 988;
    }
    m1_pwm = constrain(m1_pwm, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(m2_pwm, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(m3_pwm, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(m4_pwm, PWM_MIN, PWM_MAX);
}

void copter_nyt_calcOutput(int16_t ch_thr) {
    throttle  = ch_thr;
    int control1 = constrain((int)(omega2[0] * M_CONST), -400, 400);
    int control2 = constrain((int)(omega2[1] * M_CONST), -400, 400);
    int control3 = constrain((int)(omega2[2] * M_CONST), -400, 400);
    int control4 = constrain((int)(omega2[3] * M_CONST), -400, 400);
    servo_yaw    = constrain((int)(u4 * M_CONST * 600000000), -512, 512);

    if (arming) {
        m1_pwm = control1 + throttle;
        m2_pwm = control2 + throttle;
        m3_pwm = control3 + throttle;
        m4_pwm = control4 + throttle;
    } else {
        m1_pwm = m2_pwm = m3_pwm = m4_pwm = 988;
    }
    m1_pwm = constrain(m1_pwm, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(m2_pwm, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(m3_pwm, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(m4_pwm, PWM_MIN, PWM_MAX);

    servo_L = basePWM - servo_yaw;
    servo_R = basePWM - servo_yaw;
}

// ─────────────────────────────────────────────
// Fixed-wing servo outputs
// ─────────────────────────────────────────────

void servos_out_manual() {
    aileron_L.writeMicroseconds(ch_roll);
    aileron_R.writeMicroseconds(ch_roll);
    elevator.writeMicroseconds(ch_pitch);
    rudder_L.writeMicroseconds(ch_yaw);
    rudder_R.writeMicroseconds(ch_yaw);
}

void servos_out_fbwa() {
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
    pwm_ele   = angleToPwm(u_pitch);
    pwm_rud_L = angleToPwm(u_yaw_cmd);
    pwm_rud_R = angleToPwm(u_yaw_cmd);

    aileron_L.writeMicroseconds(pwm_ail_L);
    aileron_R.writeMicroseconds(pwm_ail_R);
    elevator.writeMicroseconds(pwm_ele);
    rudder_L.writeMicroseconds(pwm_rud_L);
    rudder_R.writeMicroseconds(pwm_rud_R);
}

void servos_out_fbwa_auto() {
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
    pwm_ele   = angleToPwm(u_pitch);
    pwm_rud_L = angleToPwm(u_yaw_cmd);
    pwm_rud_R = angleToPwm(u_yaw_cmd);

    aileron_L.writeMicroseconds(pwm_ail_L);
    aileron_R.writeMicroseconds(pwm_ail_R);
    elevator.writeMicroseconds(pwm_ele - 42); // trim koreksi auto
    rudder_L.writeMicroseconds(pwm_rud_L);
    rudder_R.writeMicroseconds(pwm_rud_R);
}

void fw_servos_out_fbwa() {
    pwm_ail_L = fw_control.servos[0] + 1500;
    pwm_ail_R = fw_control.servos[1] + 1500;
    pwm_ele   = fw_control.servos[2] + 1500;
    pwm_rud   = fw_control.servos[3] + 1500;

    aileron_L.writeMicroseconds(pwm_ail_L);
    aileron_R.writeMicroseconds(pwm_ail_R);
    elevator.writeMicroseconds(pwm_ele);
    rudder_L.writeMicroseconds(pwm_rud);
    rudder_R.writeMicroseconds(pwm_rud);
}

// ─────────────────────────────────────────────
// Flywing control outputs
// ─────────────────────────────────────────────

void servos_out_manual_flywing() {
    aileron_L.writeMicroseconds(abs(basePWM - (ch_pitch + ch_roll)));
    aileron_R.writeMicroseconds(abs(basePWM - (ch_pitch - ch_roll)));
}

void servos_out_fbwa_flywing() {
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
    pwm_ele   = angleToPwm(u_pitch);

    aileron_L.writeMicroseconds(abs(basePWM - (pwm_ele + pwm_ail_L)));
    aileron_R.writeMicroseconds(abs(basePWM - (pwm_ele - pwm_ail_R)));

    float throttle_pwm = ch_throttle;
    float yaw_diff     = u_yaw_cmd * 5.0f; // gain yaw, sesuaikan saat tuning

    m1_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX); // motor kanan
    m3_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX); // motor kiri
    m2_pwm = 988;
    m4_pwm = 988;

    motor_loop(arming ? m1_pwm : 988,
               arming ? m2_pwm : 988,
               arming ? m3_pwm : 988,
               arming ? m4_pwm : 988);
}

void flywing_calcout() {
    servo_R = constrain((int)(flywingcontrol.servos[0] * M_CONST), -400, 400);
    servo_L = constrain((int)(flywingcontrol.servos[1] * M_CONST), -400, 400);
}

void flywing_servos_out(int pwm_L, int pwm_R) {
    aileron_L.writeMicroseconds(pwm_L + basePWM);
    aileron_R.writeMicroseconds(pwm_R + basePWM);

    float throttle_pwm = ch_throttle;
    float yaw_diff     = (ch_yaw - 1500) * 0.5f;

    m1_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX);

    motor_loop(arming ? m1_pwm : 988,
               arming ? m2_pwm : 988,
               arming ? m3_pwm : 988,
               arming ? m4_pwm : 988);
}

// ─────────────────────────────────────────────
// Payload control
// ─────────────────────────────────────────────

void payload_loop() {
    if (!arming) {
        servo_pwm_payload = 1000;
    } else {
        servo_pwm_payload = (ch_vehicle_mode > 1500) ? 2000 : 1000;
    }
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

void autopayload_loop() {
    if (!arming) {
        servo_pwm_payload     = 1000;
        payload_drop_command  = false;
    } else {
        if (payload_drop_command) {
            servo_pwm_payload    = 2000;
            payload_drop_command = false;
            Serial.println("AUTOPAYLOAD: Payload dropped via MAVLink command");
        } else {
            servo_pwm_payload = 1000;
        }
    }
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

void payload_control_unified() {
    uint32_t current_time = millis();

    if (!arming) {
        servo_pwm_payload    = 1000;
        payload_drop_command = false;
        payload_dropping     = false;
        payload_drop_start_time = 0;
    } else {
        // Prioritas 1: AUTO drop dari Raspberry Pi via MAVLink
        if (payload_drop_command) {
            if (!payload_dropping) {
                payload_dropping        = true;
                payload_drop_start_time = current_time;
                servo_pwm_payload       = 2000;
            }
            if (current_time - payload_drop_start_time >= PAYLOAD_DROP_DURATION) {
                payload_drop_command = false;
                payload_dropping     = false;
            } else {
                servo_pwm_payload = 2000;
            }
        }
        // Prioritas 2: MANUAL drop dari remote
        else if (ch_vehicle_mode > 1500) {
            servo_pwm_payload = 3000; // drop (dikonstrain ke 2012 di bawah)
            payload_dropping  = false;
        } else {
            servo_pwm_payload = 1000;
            payload_dropping  = false;
        }
    }

    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

// ─────────────────────────────────────────────
// Throttle outputs
// ─────────────────────────────────────────────

void plane_motors_out() {
    int motor_pwm = ch_throttle;
    if (mode_fbwa_plane) {
        motor_pwm = constrain(motor_pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    }
    pusher.writeMicroseconds(arming ? motor_pwm : 988);
}

void plane_motors_out_auto(float pwm) {
    int motor_pwm = constrain((int)pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    // yaw differential via dual motor bila diperlukan
    (void)motor_pwm; // placeholder
}

void dual_motor_throttle_out(int pwm_throttle) {
    pwm_throttle = constrain(pwm_throttle, 988, 2012);
    if (arming) {
        mtr_1.writeMicroseconds(pwm_throttle);
        mtr_2.writeMicroseconds(pwm_throttle);
    } else {
        mtr_1.writeMicroseconds(988);
        mtr_2.writeMicroseconds(988);
    }
}

void dual_motor_throttle_out_yaw() {
    int motor_pwm = (mode_now == 3) ? fw_control.get_auto_throttle() : ch_throttle;
    float yaw_diff = (ch_yaw - 1500) * 0.3f;
    int pwm_right  = constrain((int)(motor_pwm + yaw_diff), 988, 2012);
    int pwm_left   = constrain((int)(motor_pwm - yaw_diff), 988, 2012);

    if (arming) {
        mtr_1.writeMicroseconds(pwm_right);
        mtr_2.writeMicroseconds(pwm_left);
    } else {
        mtr_1.writeMicroseconds(988);
        mtr_2.writeMicroseconds(988);
    }
}

void calc_throttle() {
    // Placeholder – implementasi TECS throttle demand di sini bila diperlukan
}