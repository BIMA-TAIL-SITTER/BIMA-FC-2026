#include "Radio.h"

bfs::SbusRx   sbus_rx(&Serial8);
bfs::SbusData data;

bool     signal_lost    = false;
uint16_t ch_roll        = 0;
uint16_t ch_pitch       = 0;
uint16_t ch_throttle    = 0;
uint16_t ch_yaw         = 0;
uint16_t ch_mode        = 0;
uint16_t ch_mode_backup = 0;
uint16_t ch_vehicle_mode = 0;
uint16_t ch5 = 0, ch6 = 0, ch7 = 0, ch8 = 0;
bool     arming         = false;

bool mode_fbwa         = false;
bool alt_hold          = false;
bool mode_hover        = false;
bool pos_hold          = false;
bool mode_manual       = false;
bool mode_fbwa_plane   = false;
bool mode_fbwb         = false;
bool transition_phase1 = false;
bool transition_phase2 = false;
bool transition_phase3 = false;
bool mode_vtol         = false;
bool mode_safety       = false;
bool mode_vtol_plane   = false;

// Definisi mode_now & prev_mode ada di translation unit lain (Mode_setup / main)
// jika belum didefinisikan di tempat lain, uncomment baris berikut:
// int mode_now = 0, prev_mode = 0;

// Scaling sinyal remote ke rentang PWM 1000–2000
float outputScaler(uint16_t ch) {
    return 0.002f * ch - 3.0f;
}

void remote_setup() {
    sbus_rx.Begin();

    if (sbus_rx.Read()) {
        data        = sbus_rx.data();
        arming      = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        if (arming) {
            Serial.println("Please disarm the remote for safety");
            while (arming) {
                if (sbus_rx.Read()) {
                    data        = sbus_rx.data();
                    arming      = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }

        if (signal_lost) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data        = sbus_rx.data();
                    arming      = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }

    Serial.println("Remote setup complete");
}

void remote_loop() {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        // Remote Juang — scaling ke PWM
        ch_roll     = data.ch[0] * 0.6267f + 877.91f;
        ch_pitch    = data.ch[1] * 0.6251f + 878.83f;
        ch_throttle = data.ch[2] * 0.6274f + 879.25f;
        ch_yaw      = data.ch[3] * 0.6304f + 876.19f;

        ch_roll     = constrain(ch_roll,     988, 2012);
        ch_pitch    = constrain(ch_pitch,    988, 2012);
        ch_throttle = constrain(ch_throttle, 988, 2012);
        ch_yaw      = constrain(ch_yaw,      988, 2012);

        arming      = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        ch_mode         = data.ch[5] * 0.6267f + 877.91f;
        ch_mode_backup  = data.ch[6] * 0.6267f + 877.91f;
        ch_vehicle_mode = data.ch[7] * 0.6267f + 877.91f;

        // Mode selection via CH7: low=MANU, mid=FBWA, high=AUTO
        if (ch_mode_backup <= 1000) {
            mode_now = 1;
        } else if (ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
            mode_now = 2;
        } else {
            mode_now = 3;
        }
    }
}