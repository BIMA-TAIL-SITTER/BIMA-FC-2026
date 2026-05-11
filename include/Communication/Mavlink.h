#ifndef MAV_H
#define MAV_H

/**
 * ============================================================
 *  Mavlink.h — MAVLink Handler + ELRS RC Channel Reader
 *  Teensy 4.1 | Autopilot FC
 * ============================================================
 *  RC Priority (sesuai ELRS MAVLink mode):
 *    1. ID 70  RC_CHANNELS_OVERRIDE  ← ELRS utama
 *    2. ID 65  RC_CHANNELS           ← fallback
 *    3. ID 35  RC_CHANNELS_RAW       ← fallback legacy
 *
 *  RC Mapping:
 *    ch_roll     = CH1  (aileron)
 *    ch_pitch    = CH2  (elevator)
 *    ch_throttle = CH3  (throttle)
 *    ch_yaw      = CH4  (rudder)
 *    ch5 – ch8           (aux channels)
 *
 *  Wiring ELRS -> Teensy:
 *    ELRS TX  ->  Pin 9  (RX2 / Serial2)
 *    ELRS RX  ->  Pin 10 (TX2 / Serial2)  [opsional]
 * ============================================================
 */

#include "TimingUtils.h"
#include "../Sensors/bno055.h"
#include "../Sensors/AHRS.h"
#include "../Telemetry/Radio.h"
#include "../Sensors/gps.h"
#include "../Sensors/airspeed.h"
#include "../Control/Mode_setup.h"
#include "../Control/Auto_setup.h"
#include "../Actuator/Voltage.h"
#include <common/mavlink.h>
#include <cstring>
#include <Locations.h>
#include "WP_EEPROM.h"

// External globals
extern Locations waypoint[100];
extern int       wp_sum;
extern Airspeed  arspd;

// ============================================================
//  RC STATUS
// ============================================================
struct RCStatus {
    bool     valid        = false;
    uint8_t  sourceId     = 0;
    uint32_t lastUpdateMs = 0;
    uint8_t  rssi         = 255;
    uint32_t rcMsgCount   = 0;
};
extern RCStatus g_rcStatus;

static const uint32_t RC_TIMEOUT_MS = 500;

inline bool rcIsHealthy() {
    return g_rcStatus.valid &&
           (millis() - g_rcStatus.lastUpdateMs < RC_TIMEOUT_MS);
}

// ============================================================
//  MISSION STATE
// ============================================================
enum MissionState { MISSION_IDLE, MISSION_RECEIVING };

// ============================================================
//  PORT IDs
// ============================================================
enum PortId { PORT_USB = 0, PORT_TLM = 1, PORT_RPI = 2 };

// ============================================================
//  MavlinkHandler CLASS
// ============================================================
class MavlinkHandler {
public:
    MavlinkHandler();

    void mavlinkTask();
    void handlePorts();

    void     sendCustomTextToGCS(const char* text);
    void     notifyWaypointReached(uint16_t seq);

    uint16_t getMissionCount() const { return wp_sum; }
    bool     rcValid()         const { return rcIsHealthy(); }

private:
    // --- Constants ---
    static const uint8_t  SYSTEM_ID    = 1;
    static const uint8_t  COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;

    static const unsigned long HEARTBEAT_MS      = 1000;
    static const unsigned long ATT_MS            = 200;
    static const unsigned long ALT_MS            = 500;
    static const unsigned long GPS_MS            = 1000;
    static const unsigned long VFR_MS            = 500;
    static const unsigned long BATTERY_MS        = 5000;
    static const unsigned long NAV_CONTROLLER_MS = 1000;

    static constexpr float     ALERT_THRESHOLD_DEG = 40.0f;
    static const unsigned long ALERT_DEBOUNCE_MS   = 2000;

    static const uint16_t MAX_MISSION = 100;

    // --- Timers ---
    unsigned long lastHb            = 0;
    unsigned long lastAtt           = 0;
    unsigned long lastAlt           = 0;
    unsigned long lastGps           = 0;
    unsigned long lastVfr           = 0;
    unsigned long lastBattery       = 0;
    unsigned long lastNavController = 0;
    unsigned long lastAlert         = 0;
    unsigned long lastRcPrint       = 0;

    // --- MAVLink buffers ---
    mavlink_status_t  st_usb, st_tlm, st_rpi;
    mavlink_message_t rxmsg;

    // --- Mission state ---
    MissionState mission_state      = MISSION_IDLE;
    uint16_t     expected_count     = 0;
    uint16_t     current_upload_seq = 0;

    // --- Output helpers ---
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUSB(Stream& port,      const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);

    // --- Telemetry senders ---
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void sendHeartbeat();
    void sendAlert(const char* txt);
    void sendAltitude(float altitude);
    void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad);
    void sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon,
                       uint16_t eph, uint16_t vel, uint16_t cog,
                       uint8_t satellites_visible);
    void sendBatteryStatus(float voltage, int16_t current_amp, int8_t battery_remaining);
    void sendNavControllerOutput(float nav_roll, float nav_pitch,
                                 float nav_bearing, float target_bearing,
                                 float wp_dist, float alt_error,
                                 float aspd_error, float xtrack_error);
    void sendVfrHud(float airspeed, float groundspeed, float heading,
                    float altitude, float climb_rate);
    void sendMissionItemReached(uint16_t seq);

    uint8_t  getBaseMode()   const;
    uint32_t getCustomMode() const;

    // --- Incoming message handlers ---
    void handleCommandLong(const mavlink_message_t& msg);
    void handleStatusText(const mavlink_message_t& msg);
    void handleMissionRequestList(const mavlink_message_t& msg);
    void handleMissionRequest(const mavlink_message_t& msg);
    void handleMissionRequestInt(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItemInt(const mavlink_message_t& msg);
    void handleMissionItem(const mavlink_message_t& msg);
    void handleMissionClearAll(const mavlink_message_t& msg);
    void handleSetPositionTargetLocalNed(const mavlink_message_t& msg);
    void handleSetPositionTargetGlobalInt(const mavlink_message_t& msg);
    void processAsFC(const mavlink_message_t& msg, PortId from);
    void forwardToOthers(const mavlink_message_t& msg, PortId from);

    // --- RC Channel handlers ---
    void handleRcChannelsOverride(const mavlink_message_t& msg);
    void handleRcChannels(const mavlink_message_t& msg);
    void handleRcChannelsRaw(const mavlink_message_t& msg);
    void handleRadioStatus(const mavlink_message_t& msg);
    void applyRcToGlobals(uint16_t c1, uint16_t c2, uint16_t c3,
                          uint16_t c4, uint16_t c5, uint16_t c6,
                          uint16_t c7, uint16_t c8,
                          uint8_t src_id);
    void printRCStatus();
};

// Global helper (dipanggil dari luar class)
void notifyWaypointReached(uint16_t seq);

#endif  // MAV_H