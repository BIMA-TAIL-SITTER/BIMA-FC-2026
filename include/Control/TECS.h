#pragma once

#include <Sensors/Barometer.h>
#include "FW_config.h"
#include <../lib/Math/AP_Math.h>
#include <Sensors/AHRS.h>
#include <../lib/Math/Filter/AverageFilter.h>
#include <Sensors/bno055.h>
#include "quaternion.h"
#include <../lib/Math/Filter/LowPassFilter.h>

// ============================================================
//  TECS TUNING PARAMETERS & STATE — extern declarations
// ============================================================
extern int     tecs_min_throtlle;
extern int     tecs_max_throttle;
extern float   tes;
extern bool    TECS_DEBUG;
extern uint32_t _last_debug_ms;

extern uint64_t _update_50hz_last_usec;
extern uint64_t _update_speed_last_usec;
extern uint64_t _update_pitch_throttle_last_usec;

extern float _hgtCompFiltOmega;
extern float _spdCompFiltOmega;
extern float _maxClimbRate;
extern float _minSinkRate;
extern float _maxSinkRate;
extern float _timeConst;
extern float _landTimeConst;
extern float _ptchDamp;
extern float _land_pitch_damp;
extern float _landDamp;
extern float _thrDamp;
extern float _land_throttle_damp;
extern float _integGain;
extern float _integGain_takeoff;
extern float _integGain_land;
extern float _vertAccLim;
extern float _rollComp;
extern float _spdWeight;
extern float _spdWeightLand;
extern float _landThrottle;
extern float _landAirspeed;
extern float _land_sink;
extern float _land_sink_rate_change;
extern int8_t _pitch_max;
extern int8_t _pitch_min;
extern int8_t _land_pitch_max;
extern float  _maxSinkRate_approach;
extern int8_t _pitch_max_limit;

extern float _height;
extern float _throttle_dem;
extern float _pitch_dem;
extern float _climb_rate;

struct HeightFilter {
    float dd_height;
    float height;
};
extern HeightFilter _height_filter;

extern float _integDTAS_state;
extern float _TAS_state;
extern float _integTHR_state;
extern float _integSEB_state;
extern float _last_throttle_dem;
extern float _last_pitch_dem;
extern float _vel_dot;
extern float _EAS;
extern float _TASmax;
extern float _TASmin;
extern float _TAS_dem;
extern float _EAS_dem;
extern float _hgt_dem;
extern float _hgt_dem_in_old;
extern float _hgt_dem_adj;
extern float _hgt_dem_adj_last;
extern float _hgt_rate_dem;
extern float _hgt_dem_prev;
extern float _land_hgt_dem;
extern float _TAS_dem_adj;
extern float _TAS_rate_dem;
extern float _STEdotErrLast;

typedef struct {
    bool underspeed : 1;
    bool badDescent : 1;
    bool reset      : 1;
} tecs_flags;

typedef union {
    tecs_flags _flags;
    uint8_t    _flags_byte;
} tecs_flags_union;

extern tecs_flags_union my_flags;

enum class clipStatus : int8_t {
    MIN  = -1,
    NONE =  0,
    MAX  =  1,
};

extern clipStatus _thr_clip_status;
extern float      _integKE;
extern float      _integSEBdot;
extern clipStatus _SEBdot_dem_clip;
extern uint32_t   _underspeed_start_ms;
extern float      _pitch_dem_unc;
extern float      _STEdot_max;
extern float      _STEdot_min;
extern float      _THRmaxf;
extern float      _THRminf;
extern float      _PITCHmaxf;
extern float      _PITCHminf;
extern float      _SPE_dem;
extern float      _SKE_dem;
extern float      _SPEdot_dem;
extern float      _SKEdot_dem;
extern float      _SPE_est;
extern float      _SKE_est;
extern float      _SPEdot;
extern float      _SKEdot;
extern float      _STE_error;
extern float      _DT;
extern uint8_t    _flare_counter;
extern float      hgt_dem_lag_filter_slew;
extern float      _path_proportion;
extern float      _distance_beyond_land_wp;
extern float      _land_pitch_min;
extern float      _SKE_weighting;
extern float      _TAS_rate_dem_lpf;
extern float      _vel_dot_lpf;

struct TECSLogging {
    float SKE_weighting;
    float SPE_error;
    float SKE_error;
    float SEB_delta;
};
extern TECSLogging logging_tecs;

extern float _load_factor;
extern LowPassFilterFloat _pitch_demand_lpf;
extern LowPassFilterFloat _pitch_measured_lpf;
extern int8_t  _use_synthetic_airspeed;
extern bool    _use_synthetic_airspeed_once;
extern AverageFilterFloat_Size5 _vdot_filter;

// ============================================================
//  PUBLIC INTERFACE
// ============================================================
int32_t get_throttle_demand(void);
float   get_pitch_demand(void);
float   get_VXdot(void);
float   get_target_airspeed(void);
float   get_max_climbrate(void);
void    reset_pitch_I(void);
float   get_land_sinkrate(void);
float   get_land_airspeed(void);
float   get_height_rate_demand(void);
void    set_path_proportion(float path_proportion);
void    set_pitch_max_limit(int8_t pitch_limit);
void    use_synthetic_airspeed(void);
void    set_tecs_min_throttle(int value);
void    set_tecs_max_throttle(int value);
int     get_tecs_min_throttle(void);
int     get_tecs_max_throttle(void);
float   tecs_hgt_afe(void);
float   timeConstant(void);

void update_50hz(void);
void update_pitch_throttle(int32_t hgt_dem_cm, float EAS_dem_cm,
                            int16_t throttle_nudge, float hgt_afe,
                            float load_factor);