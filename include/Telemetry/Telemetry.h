#ifndef TELEM_H
#define TELEM_H

// ===================== Sensors =====================
#include "Sensors/airspeed.h"
#include "Sensors/gps.h"

// ===================== Actuator =====================
#include "Actuator/Voltage.h"

// ===================== Communication =====================
#include "Communication/TimingUtils.h"
#include <Protobuf/Protobuf.h>     // external library

// ===================== Control =====================
#include "Control/Fuzzy_FW_Roll.h"
#include "Control/Mode_setup.h"
#include "Control/flywing_control.h"
// #include "Transition.h"          // tidak ada di directory — hapus atau buat dulu
// #include "Mode_Transisi.h"       // tidak ada di directory — cek nama file

// ===================== Other =====================
// #include "Attitude.h"            // tidak ada di directory listing — cek lokasi

// =============================================================================
// Helper — modifikasi gain (inline karena sangat kecil, satu baris)
// =============================================================================
inline void modifygain(float &prev_gain, float new_gain)
{
    prev_gain += new_gain;
}

// =============================================================================
// Global state (definisi di Telemetry.cpp)
// =============================================================================
extern int linee;
extern int milisawall;

// =============================================================================
// Telemetry class
// =============================================================================
class Telemetry
{
public:
    /** Serial print ke putty dengan modul telemetry */
    void print_log_fw_telem();

    /** Tuning gain lewat keyboard */
    void tuning_gain_pid();

    /** Kirim data attitude kendaraan ke GCS via protobuf */
    void send_vehicle_attitude_data();

    /** Terima perintah dari GCS */
    void receive_command_from_gcs();

    /** Serial print waypoint */
    void print_wp();

    /** Cek kalibrasi kompas */
    void calibrate_compass();

    /** Cek kalibrasi IMU */
    void raw_imu();

    /** Program tuning gain baru, diuji via PuTTY */
    void pid_gain_tuning();

    /** Kirim data telemetry ke GCS */
    void send_telemetry();

    /** TECS detailed debugging */
    void print_tecs_debug();
};

// =============================================================================
// Global instances (definisi di masing-masing .cpp)
// =============================================================================
extern Telemetry telem;
extern ModeId    current;

#endif // TELEM_H