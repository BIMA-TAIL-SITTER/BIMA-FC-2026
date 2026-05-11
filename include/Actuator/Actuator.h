#pragma once
// TUNINGAN WAHANA UTAMA LOMBA REGIONAL
// include/Actuator/Actuator.h
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <Servo.h>

// Control layer headers
#include "../Control/FW_ControlModes.h"   // header-only: enum & flags
#include "../Control/Copter_Control.h"
#include "../Control/FW_control.h"
#include "../Control/flywing_control.h"
#include "../Control/TECS.h"
#include "../Control/Copter_Config.h"     // header-only: PWM_MIN, PWM_MAX, dll

// ─── Pin FC BARU ──────────────────────────────────────────────────
#define SERVO_AIL_L    2   // MTR 10
#define SERVO_ELE      4   // MTR 11
#define THROTTLE       5   // MTR              (Tidak dipakai)
#define SERVO_RUD_L    3   // MTR 7
#define SERVO_AIL_R    14  // MTR 6
#define SERVO_RUD_R    10  // MTR 8
#define SERVO_PAYLOAD  12  // MTR 5
#define MOTOR_1_PIN    31  // MTR 4            (Tidak dipakai)
#define MOTOR_2_PIN    32  //                  (Tidak dipakai – cek pin FC)
#define MOTOR_3_PIN    33  // FC pin 9         (Tidak dipakai)
#define MOTOR_4_PIN    13  // FC pin 10        (Tidak dipakai)

// Servo & motor objects
extern Servo motor1, motor2, motor3, motor4;
extern Servo aileron_L;
extern Servo aileron_R;
extern Servo elevator;
extern Servo rudder_L;
extern Servo rudder_R;
extern Servo pusher;
extern Servo payload;
extern Servo mtr_1;
extern Servo mtr_2;

// Payload drop timing
extern uint32_t payload_drop_start_time;
extern const uint32_t PAYLOAD_DROP_DURATION;
extern bool payload_dropping;

// Control outputs
extern float u_roll, u_pitch, u_yaw, u_yaw_cmd;
extern float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
extern float servo_L, servo_R;
extern int   servo_yaw;
extern float high_out;
extern int   basePWM;

// PWM limits & channel values
extern uint16_t servo_max;
extern uint16_t servo_min;
extern uint16_t pwm_ail_L;
extern uint16_t pwm_ail_R;
extern uint16_t pwm_ele;
extern uint16_t pwm_rud;
extern uint16_t pwm_rud_L;
extern uint16_t pwm_rud_R;
extern uint16_t throttle;

extern int32_t commanded_throttle;

// Utility conversions
float    scaleToAngle(uint16_t ch);
float    scaleToPercent(uint16_t ch);
float    scaleToPercent_reversed(uint16_t ch);
uint16_t angleToPwm(float angle);
uint16_t angleToPwm_reversed(float angle);
float    PercenttoPWM(float Percent);

// Initialisation
void init_actuator();
void servo_idle();

// Motor outputs
void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4);
void motor_loop_nyt(int pwm1, int pwm2, int pwm3, int pwm4,
                    int pwm_yaw_l = 0, int pwm_yaw_r = 0);
void motorCopterOff(int pwm1, int pwm2, int pwm3, int pwm4);

// Copter control outputs
void copter_calcOutput(int16_t ch_thr);
void copter_nyt_calcOutput(int16_t ch_thr);

// Fixed-wing servo outputs
void servos_out_manual();
void servos_out_fbwa();
void servos_out_fbwa_auto();
void fw_servos_out_fbwa();

// Flywing control outputs
void servos_out_manual_flywing();
void servos_out_fbwa_flywing();
void flywing_calcout();
void flywing_servos_out(int pwm_L, int pwm_R);

// Payload control
void payload_loop();
void autopayload_loop();
void payload_control_unified();

// Throttle outputs
void plane_motors_out();
void plane_motors_out_auto(float pwm);
void dual_motor_throttle_out(int pwm_throttle);
void dual_motor_throttle_out_yaw();
void calc_throttle();

#endif // ACTUATOR_H