#include "TECS.h"

// ============================================================
//  GLOBAL STATE DEFINITIONS
// ============================================================
int     tecs_min_throtlle = 30;
int     tecs_max_throttle = 85;
float   tes               = 0.0f;
bool    TECS_DEBUG        = false;
uint32_t _last_debug_ms   = 0;

uint64_t _update_50hz_last_usec              = 0;
uint64_t _update_speed_last_usec             = 0;
uint64_t _update_pitch_throttle_last_usec    = 0;

float _hgtCompFiltOmega    = 5.0f;
float _spdCompFiltOmega    = 2.0f;
float _maxClimbRate        = 5.0f;
float _minSinkRate         = 2.0f;
float _maxSinkRate         = 5.0f;
float _timeConst           = 5.0f;
float _landTimeConst       = 2.0f;
float _ptchDamp            = 0.0f;
float _land_pitch_damp     = 0.0f;
float _landDamp            = 0.5f;
float _thrDamp             = 0.5f;
float _land_throttle_damp  = 0.0f;
float _integGain           = 0.1f;
float _integGain_takeoff   = 0.0f;
float _integGain_land      = 0.0f;
float _vertAccLim          = 7.0f;
float _rollComp            = 10.0f;
float _spdWeight           = 1.0f;
float _spdWeightLand       = -1.0f;
float _landThrottle        = -1.0f;
float _landAirspeed        = -1.0f;
float _land_sink           = 0.25f;
float _land_sink_rate_change = 0.0f;
int8_t _pitch_max          = 0;
int8_t _pitch_min          = 0;
int8_t _land_pitch_max     = 10;
float  _maxSinkRate_approach = 0.0f;
int8_t _pitch_max_limit    = 90;

float _height              = 0.0f;
float _throttle_dem        = 0.0f;
float _pitch_dem           = 0.0f;
float _climb_rate          = 0.0f;

HeightFilter _height_filter = {0.0f, 0.0f};

float _integDTAS_state     = 0.0f;
float _TAS_state           = 0.0f;
float _integTHR_state      = 0.0f;
float _integSEB_state      = 0.0f;
float _last_throttle_dem   = 0.0f;
float _last_pitch_dem      = 0.0f;
float _vel_dot             = 0.0f;
float _EAS                 = 0.0f;
float _TASmax              = 0.0f;
float _TASmin              = 0.0f;
float _TAS_dem             = 0.0f;
float _EAS_dem             = 0.0f;
float _hgt_dem             = 0.0f;
float _hgt_dem_in_old      = 0.0f;
float _hgt_dem_adj         = 0.0f;
float _hgt_dem_adj_last    = 0.0f;
float _hgt_rate_dem        = 0.0f;
float _hgt_dem_prev        = 0.0f;
float _land_hgt_dem        = 0.0f;
float _TAS_dem_adj         = 0.0f;
float _TAS_rate_dem        = 0.0f;
float _STEdotErrLast       = 0.0f;

tecs_flags_union my_flags  = {};

clipStatus _thr_clip_status   = clipStatus::NONE;
float      _integKE           = 0.0f;
float      _integSEBdot       = 0.0f;
clipStatus _SEBdot_dem_clip   = clipStatus::NONE;
uint32_t   _underspeed_start_ms = 0;
float      _pitch_dem_unc     = 0.0f;
float      _STEdot_max        = 0.0f;
float      _STEdot_min        = 0.0f;
float      _THRmaxf           = 0.0f;
float      _THRminf           = 0.0f;
float      _PITCHmaxf         = 0.0f;
float      _PITCHminf         = 0.0f;
float      _SPE_dem           = 0.0f;
float      _SKE_dem           = 0.0f;
float      _SPEdot_dem        = 0.0f;
float      _SKEdot_dem        = 0.0f;
float      _SPE_est           = 0.0f;
float      _SKE_est           = 0.0f;
float      _SPEdot            = 0.0f;
float      _SKEdot            = 0.0f;
float      _STE_error         = 0.0f;
float      _DT                = 0.0f;
uint8_t    _flare_counter     = 0;
float      hgt_dem_lag_filter_slew = 0.0f;
float      _path_proportion   = 0.0f;
float      _distance_beyond_land_wp = 0.0f;
float      _land_pitch_min    = -90.0f;
float      _SKE_weighting     = 0.0f;
float      _TAS_rate_dem_lpf  = 0.0f;
float      _vel_dot_lpf       = 0.0f;

TECSLogging logging_tecs      = {};
float      _load_factor       = 0.0f;

LowPassFilterFloat _pitch_demand_lpf;
LowPassFilterFloat _pitch_measured_lpf;
int8_t  _use_synthetic_airspeed      = false;
bool    _use_synthetic_airspeed_once = false;
AverageFilterFloat_Size5 _vdot_filter;

// ============================================================
//  PUBLIC INTERFACE
// ============================================================
int32_t get_throttle_demand(void) {
    return int32_t(_throttle_dem * 100.0f);
}

float get_pitch_demand(void) {
    return _pitch_dem * 57.295781f;
}

float get_VXdot(void) {
    return _vel_dot;
}

float get_target_airspeed(void) {
    return _TAS_dem / baro.getEAS2TAS();
}

float get_max_climbrate(void) {
    return _maxClimbRate;
}

void reset_pitch_I(void) {
    _integSEB_state = 0.0f;
}

float get_land_sinkrate(void) {
    return _land_sink;
}

float get_land_airspeed(void) {
    return _landAirspeed;
}

float get_height_rate_demand(void) {
    return _hgt_rate_dem;
}

void set_path_proportion(float path_proportion) {
    _path_proportion = constrain(path_proportion, 0.0f, 1.0f);
}

void set_pitch_max_limit(int8_t pitch_limit) {
    _pitch_max_limit = pitch_limit;
}

void use_synthetic_airspeed(void) {
    _use_synthetic_airspeed_once = true;
}

void set_tecs_min_throttle(int value) { tecs_min_throtlle = value; }
void set_tecs_max_throttle(int value) { tecs_max_throttle = value; }
int  get_tecs_min_throttle(void)      { return tecs_min_throtlle; }
int  get_tecs_max_throttle(void)      { return tecs_max_throttle; }

float timeConstant(void) {
    return (_timeConst < 0.1f) ? 0.1f : _timeConst;
}

// ============================================================
//  INTERNAL HELPERS
// ============================================================
float _get_i_gain(void) {
    return _integGain;
}

void _update_STE_rate_lim(void) {
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = -_minSinkRate * GRAVITY_MSS;
}

void _initialise_states(float hgt_afe) {
    if (_DT > 1.0f) {
        _integTHR_state    = 0.0f;
        _integSEB_state    = 0.0f;
        _integSEBdot       = 0.0f;
        _integKE           = 0.0f;
        _last_throttle_dem = THROTTLE_CRUISE * 0.01f;
        _last_pitch_dem    = imu.pitch;
        _hgt_dem_adj_last  = hgt_afe;
        _hgt_dem_adj       = _hgt_dem_adj_last;
        _hgt_dem_prev      = _hgt_dem_adj_last;
        _hgt_dem_in_old    = _hgt_dem_adj_last;
        _TAS_dem_adj       = _TAS_dem;
        my_flags._flags.underspeed = false;
        my_flags._flags.badDescent = false;
        _DT                = 0.2f;

        if (TECS_DEBUG) {
            char buf[60];
            snprintf(buf, sizeof(buf), "[TECS] INIT hgt_afe:%.2fm", hgt_afe);
            Serial2.println(buf);
        }
    }
}

void _update_speed(float DT) {
    if (my_flags._flags.reset) {
        _vdot_filter.reset();
        _vel_dot_lpf = _vel_dot;
    } else {
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        float temp  = rotMat.c.x * GRAVITY_MSS + imu._accel.x;
        _vel_dot    = _vdot_filter.apply(temp);
        float alpha = DT / (DT + timeConstant());
        _vel_dot_lpf = _vel_dot_lpf * (1.0f - alpha) + _vel_dot * alpha;
    }

    bool use_airspeed = _use_synthetic_airspeed_once || ahrs.is_airspeed_sensor_enabled;

    float EAS2TAS = baro.getEAS2TAS();
    _TAS_dem = _EAS_dem * EAS2TAS;

    if (my_flags._flags.reset || !use_airspeed) {
        _TASmax = MAX_AIRSPEED * EAS2TAS;
    } else if (_thr_clip_status == clipStatus::MAX) {
        const float velRateMin = 0.5f * _STEdot_min / MAX(_TAS_state, MIN_AIRSPEED * EAS2TAS);
        _TASmax += _DT * velRateMin;
        _TASmax = MAX(_TASmax, AIRSPEED_CRUISE * EAS2TAS);
    } else {
        const float velRateMax = 0.5f * _STEdot_max / MAX(_TAS_state, MIN_AIRSPEED * EAS2TAS);
        _TASmax += _DT * velRateMax;
    }
    _TASmax = MIN(_TASmax, MAX_AIRSPEED * EAS2TAS);
    _TASmin = MIN_AIRSPEED * EAS2TAS;

    if (_TASmax < _TASmin) _TASmax = _TASmin;

    if (!ahrs.is_airspeed_sensor_enabled) {
        _EAS = constrain(AIRSPEED_CRUISE, MIN_AIRSPEED, MAX_AIRSPEED);
    }

    float min_airspeed = 3.0f;

    if (my_flags._flags.reset) {
        _TAS_state       = MAX(_EAS * EAS2TAS, MIN_AIRSPEED);
        _integDTAS_state = 0.0f;
        return;
    }

    float aspdErr         = (_EAS * EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    if (_TAS_state < 3.1f) {
        integDTAS_input = MAX(integDTAS_input, 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input  = _integDTAS_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _TAS_state       = MAX(_TAS_state + TAS_input * DT, min_airspeed);
}

void _update_speed_demand(void) {
    if (my_flags._flags.badDescent || my_flags._flags.underspeed) {
        _TAS_dem = _TASmin;
    }
    _TAS_dem = constrain(_TAS_dem, _TASmin, _TASmax);

    const float velRateMax     = 0.5f * _STEdot_max / _TAS_state;
    const float velRateMin     = 0.5f * _STEdot_min / _TAS_state;
    const float TAS_dem_previous = _TAS_dem_adj;
    const float dt             = 0.1f;

    if ((_TAS_dem - TAS_dem_previous) > (velRateMax * dt)) {
        _TAS_dem_adj  = TAS_dem_previous + velRateMax * dt;
        _TAS_rate_dem = velRateMax;
    } else if ((_TAS_dem - TAS_dem_previous) < (velRateMin * dt)) {
        _TAS_dem_adj  = TAS_dem_previous + velRateMin * dt;
        _TAS_rate_dem = velRateMin;
    } else {
        _TAS_rate_dem = (_TAS_dem - TAS_dem_previous) / dt;
        _TAS_dem_adj  = _TAS_dem;
    }
    _TAS_dem_adj = constrain(_TAS_dem_adj, _TASmin, _TASmax);
}

void _update_height_demand(void) {
    _hgt_dem        = 0.5f * (_hgt_dem + _hgt_dem_in_old);
    _hgt_dem_in_old = _hgt_dem;

    float max_sink_rate = _maxSinkRate;

    if ((_hgt_dem - _hgt_dem_prev) > (_maxClimbRate * 0.1f)) {
        _hgt_dem = _hgt_dem_prev + _maxClimbRate * 0.1f;
    } else if ((_hgt_dem - _hgt_dem_prev) < (-max_sink_rate * 0.1f)) {
        _hgt_dem = _hgt_dem_prev - max_sink_rate * 0.1f;
    }
    _hgt_dem_prev = _hgt_dem;

    _hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;

    float new_hgt_dem         = _hgt_dem_adj;
    hgt_dem_lag_filter_slew   = 0.0f;
    _hgt_dem_adj_last         = _hgt_dem_adj;
    _hgt_dem_adj              = new_hgt_dem;

    float hgt_error           = _hgt_dem_adj - _height;
    float adaptive_time_const = _timeConst;
    if (fabsf(hgt_error) < 3.0f) {
        adaptive_time_const = MAX(1.5f, fabsf(hgt_error) * 0.5f + 1.0f);
    }
    _hgt_rate_dem = constrain(hgt_error / adaptive_time_const, -_minSinkRate, _maxClimbRate);

    if (TECS_DEBUG && (millis() - _last_debug_ms > 1000)) {
        char buf[100];
        snprintf(buf, sizeof(buf), "[ALT] H:%.2f T:%.2f E:%.2f RD:%.2f",
                 _height, _hgt_dem_adj, hgt_error, _hgt_rate_dem);
        Serial2.println(buf);
    }
}

void _detect_underspeed(void) {
    if (my_flags._flags.underspeed &&
        _TAS_state >= _TASmin * 1.15f &&
        millis() - _underspeed_start_ms > 3000U) {
        my_flags._flags.underspeed = false;
    }

    if (((_TAS_state < _TASmin * 0.9f) && (_throttle_dem >= _THRmaxf * 0.95f)) ||
        ((_height < _hgt_dem_adj) && my_flags._flags.underspeed)) {
        my_flags._flags.underspeed = true;
        if (_TAS_state < _TASmin * 0.9f) {
            _underspeed_start_ms = millis();
        }
    } else {
        my_flags._flags.underspeed = false;
    }
}

void _update_energies(void) {
    _SPE_dem    = _hgt_dem_adj * GRAVITY_MSS;
    _SKE_dem    = 0.5f * _TAS_dem_adj * _TAS_dem_adj;
    _SKEdot_dem = _TAS_state * (_TAS_rate_dem - _TAS_rate_dem_lpf);
    _SPE_est    = _height * GRAVITY_MSS;
    _SKE_est    = 0.5f * _TAS_state * _TAS_state;
    _SPEdot     = _climb_rate * GRAVITY_MSS;
    _SKEdot     = _TAS_state * (_vel_dot - _vel_dot_lpf);
}

void _update_throttle_with_airspeed(void) {
    float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem;
    float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;

    _STE_error = constrain((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem   = constrain((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    STEdot_error   = 0.2f * STEdot_error + 0.8f * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    if (my_flags._flags.underspeed) {
        _throttle_dem = 1.0f;
    } else {
        float K_STE2Thr = 1.0f / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));
        float ff_throttle = 0.0f;
        float nomThr      = TECS_CRUISE_THROTTLE * 0.01f;
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        float cosPhi      = sqrtf((rotMat.a.y * rotMat.a.y) + (rotMat.b.y * rotMat.b.y));
        STEdot_dem        = STEdot_dem + _rollComp * (1.0f / constrain(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
        ff_throttle       = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        float throttle_damp = _thrDamp;
        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;
        _throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);

        float THRminf_clipped_to_zero = constrain(_THRminf, 0.0f, _THRmaxf);

        if (THR_SLEW_RATE != 0) {
            float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * THR_SLEW_RATE * 0.01f;
            _throttle_dem     = constrain(_throttle_dem,
                                          _last_throttle_dem - thrRateIncr,
                                          _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        float maxAmp   = 0.5f * (_THRmaxf - THRminf_clipped_to_zero);
        float integ_max = constrain((_THRmaxf - _throttle_dem + 0.1f), -maxAmp, maxAmp);
        float integ_min = constrain((_THRminf - _throttle_dem - 0.1f), -maxAmp, maxAmp);

        _integTHR_state = constrain(
            _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr,
            integ_min, integ_max);
        _throttle_dem = _throttle_dem + _integTHR_state;
    }
    _throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);
}

void _update_throttle_without_airspeed(int16_t throttle_nudge) {
    float nomThr = (TECS_CRUISE_THROTTLE + throttle_nudge) * 0.01f;

    if (_pitch_dem > 0.0f && _PITCHmaxf > 0.0f) {
        _throttle_dem = nomThr + (_THRmaxf - nomThr) * _pitch_dem / _PITCHmaxf;
    } else if (_pitch_dem < 0.0f && _PITCHminf < 0.0f) {
        _throttle_dem = nomThr + (_THRminf - nomThr) * _pitch_dem / _PITCHminf;
    } else {
        _throttle_dem = nomThr;
    }

    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    float cosPhi    = sqrtf((rotMat.a.y * rotMat.a.y) + (rotMat.b.y * rotMat.b.y));
    float STEdot_dem = _rollComp * (1.0f / constrain(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
    _throttle_dem   = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
}

void _detect_bad_descent(void) {
    float STEdot = _SPEdot + _SKEdot;
    if ((!my_flags._flags.underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) &&
         (_throttle_dem >= _THRmaxf * 0.9f)) ||
        (my_flags._flags.badDescent && !my_flags._flags.underspeed && (_STE_error > 0.0f))) {
        my_flags._flags.badDescent = true;
    } else {
        my_flags._flags.badDescent = false;
    }
}

void _update_pitch(void) {
    _SKE_weighting = constrain(_spdWeight, 0.0f, 2.0f);
    if (!ahrs.is_airspeed_sensor_enabled || _use_synthetic_airspeed) {
        _SKE_weighting = 0.0f;
    }
    float SPE_weighting = MIN(2.0f - _SKE_weighting, 1.0f);
    _SKE_weighting      = MIN(_SKE_weighting, 1.0f);

    float SEB_dem   = _SPE_dem * SPE_weighting - _SKE_dem * _SKE_weighting;
    float SEB_est   = _SPE_est * SPE_weighting - _SKE_est * _SKE_weighting;
    float SEB_error = SEB_dem - SEB_est;

    float SEBdot_dem = _hgt_rate_dem * GRAVITY_MSS * SPE_weighting + SEB_error / timeConstant();
    if (SEBdot_dem < -_maxSinkRate * GRAVITY_MSS) {
        SEBdot_dem        = -_maxSinkRate * GRAVITY_MSS;
        _SEBdot_dem_clip  = clipStatus::MIN;
    } else if (SEBdot_dem > _maxClimbRate * GRAVITY_MSS) {
        SEBdot_dem        = _maxClimbRate * GRAVITY_MSS;
        _SEBdot_dem_clip  = clipStatus::MAX;
    } else {
        _SEBdot_dem_clip  = clipStatus::NONE;
    }

    float SEBdot_est   = _SPEdot * SPE_weighting - _SKEdot * _SKE_weighting;
    float SEBdot_error = SEBdot_dem - SEBdot_est;
    float pitch_damp   = _ptchDamp;
    float SEBdot_dem_total = SEBdot_dem + SEBdot_error * pitch_damp;
    float gainInv      = _TAS_state * GRAVITY_MSS;

    float integSEBdot_min = (gainInv * (_PITCHminf - radians(5.0f))) - SEBdot_dem_total;
    float integSEBdot_max = (gainInv * (_PITCHmaxf + radians(5.0f))) - SEBdot_dem_total;
    float integSEB_range  = integSEBdot_max - integSEBdot_min;
    float integSEB_delta  = constrain(SEBdot_error * _get_i_gain() * _DT,
                                      -integSEB_range * 0.1f, integSEB_range * 0.1f);

    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + integSEB_delta + _integKE) / gainInv;

    bool inhibit_integrator = ((_pitch_dem_unc > _PITCHmaxf) && integSEB_delta > 0.0f) ||
                              ((_pitch_dem_unc < _PITCHminf) && integSEB_delta < 0.0f);

    float alt_error_signed = _hgt_dem_adj - _height;
    if (alt_error_signed < -0.2f && integSEB_delta > 0.0f) {
        inhibit_integrator = true;
        if (TECS_DEBUG && (millis() - _last_debug_ms > 1000)) {
            Serial2.println("[TECS] Windup inhibit");
        }
    }

    if (!inhibit_integrator) {
        _integSEBdot += integSEB_delta;
        _integKE     += (_SKE_est - _SKE_dem) * _SKE_weighting * _DT / timeConstant();
    } else {
        float coef    = 1.0f - _DT / (_DT + timeConstant());
        _integSEBdot *= coef;
        _integKE     *= coef;
    }

    _integSEBdot = constrain(_integSEBdot, integSEBdot_min, integSEBdot_max);
    float KE_integ_limit = 0.25f * (_PITCHmaxf - _PITCHminf) * gainInv;
    _integKE = constrain(_integKE, -KE_integ_limit, KE_integ_limit);

    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + _integKE) / gainInv;
    _pitch_dem     = constrain(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    float ptchRateIncr = _DT * _vertAccLim / _TAS_state;
    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    } else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }
    _last_pitch_dem = _pitch_dem;
}

// ============================================================
//  PUBLIC: update_50hz
// ============================================================
void update_50hz(void) {
    ahrs.get_relative_position_D_home(_height);
    _height *= -1.0f;

    uint64_t now = micros();
    float DT     = (now - _update_50hz_last_usec) * 1.0e-6f;
    my_flags._flags.reset = DT > 1.0f;

    if (my_flags._flags.reset) {
        _climb_rate                  = 0.0f;
        _height_filter.dd_height     = 0.0f;
        DT                           = 0.02f;
        _vdot_filter.reset();
    }
    _update_50hz_last_usec = now;

    Vector3f velned = ahrs.get_velocity_ned();
    _climb_rate = -velned.z;

    _update_speed(DT);
}

// ============================================================
//  PUBLIC: update_pitch_throttle
// ============================================================
void update_pitch_throttle(int32_t hgt_dem_cm, float EAS_dem_cm,
                            int16_t throttle_nudge, float hgt_afe,
                            float load_factor) {
    uint64_t now = micros();
    _DT          = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _update_pitch_throttle_last_usec = now;

    _hgt_dem = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm;

    static bool first_call = true;
    if (TECS_DEBUG && first_call && _DT < 1.0f) {
        char buf[80];
        snprintf(buf, sizeof(buf), "[TECS] TGT:%.2fcm CUR:%.2fm EAS:%.1fm/s",
                 (float)hgt_dem_cm, hgt_afe, EAS_dem_cm);
        Serial2.println(buf);
        first_call = false;
    }

    _update_speed(load_factor);

    _THRmaxf = tecs_max_throttle * 0.01f;
    _THRminf = tecs_min_throtlle * 0.01f;
    _THRmaxf = MAX(_THRmaxf, _THRminf + 0.01f);

    _PITCHmaxf = (_pitch_max == 0)    ? PITCH_MAX : MIN(_pitch_max, PITCH_MAX);
    _PITCHminf = (_pitch_min >= 0)    ? PITCH_MIN : MAX(_pitch_min, PITCH_MIN);

    if (_pitch_max_limit < 90) {
        _PITCHmaxf     = constrain(_PITCHmaxf, -90.0f, (float)_pitch_max_limit);
        _PITCHminf     = constrain(_PITCHminf, -(float)_pitch_max_limit, _PITCHmaxf);
        _pitch_max_limit = 90;
    }

    _PITCHmaxf = radians(_PITCHmaxf);
    _PITCHminf = radians(_PITCHminf);
    _PITCHmaxf = MAX(_PITCHmaxf, _PITCHminf);

    _initialise_states(hgt_afe);
    _update_STE_rate_lim();
    _update_speed_demand();
    _update_height_demand();
    _detect_underspeed();
    _update_energies();

    if ((ahrs.is_airspeed_sensor_enabled && _use_synthetic_airspeed) ||
        _use_synthetic_airspeed_once) {
        _update_throttle_with_airspeed();
        _use_synthetic_airspeed_once = false;
    } else {
        _update_throttle_without_airspeed(throttle_nudge);
    }

    _detect_bad_descent();
    _update_pitch();

    if (TECS_DEBUG && (millis() - _last_debug_ms > 1000)) {
        char buf[80];
        snprintf(buf, sizeof(buf), "[PIT] P:%.1f IS:%.2f IK:%.2f T:%d%%",
                 _pitch_dem * 57.3f, _integSEBdot, _integKE,
                 (int)(_throttle_dem * 100));
        Serial2.println(buf);
        _last_debug_ms = millis();
    }
}