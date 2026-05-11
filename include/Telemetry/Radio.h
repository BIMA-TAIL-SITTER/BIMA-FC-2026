#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <sbus.h>

extern bfs::SbusRx   sbus_rx;
extern bfs::SbusData data;

extern bool     signal_lost;
extern uint16_t ch_roll, ch_pitch, ch_throttle, ch_yaw;
extern uint16_t ch_mode, ch_mode_backup, ch_vehicle_mode;
extern uint16_t ch5, ch6, ch7, ch8;
extern bool     arming;

extern bool mode_fbwa;
extern bool alt_hold;
extern bool mode_hover;
extern bool pos_hold;
extern bool mode_manual;
extern bool mode_fbwa_plane;
extern bool mode_fbwb;
extern bool transition_phase1;
extern bool transition_phase2;
extern bool transition_phase3;
extern bool mode_vtol;
extern bool mode_safety;
extern bool mode_vtol_plane;
extern int  mode_now, prev_mode;

float outputScaler(uint16_t ch);
void  remote_setup();
void  remote_loop();

#endif