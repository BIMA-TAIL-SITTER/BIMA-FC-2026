#pragma once

#include "FW_controller.h"
#include "Auto_setup.h"
#include "TECS.h"
#include "L1_Controller.h"
#include "../Actuator/Actuator.h"
#include "Mode_setup.h"
#include "FW_ControlModes.h"
#include "Fuzzy_FW_Roll.h"

// Forward declaration for waypoint notification
void notifyWaypointReached(uint16_t seq);

extern bool     enable_throttle_nudge;
extern int16_t  throttle_nudge;
extern uint32_t last_home_set_ms;
extern float    groundspeed_undershoot;
extern int      autonav_state;
extern bool     enterauto;

void calc_groundspeed_undershoot();
void nav_gps();
void update_wp_nav();
void setup_turn_angle();
void set_next_WP(struct Locations &loc);
void update_alt();
void update_speed_height();
void updateAuto_FW();
void navigate();
void add_waypoint(int32_t lat, int32_t lng, int32_t alt, int index);
int  display_wp();

float   get_next_ground_course(float default_angle);