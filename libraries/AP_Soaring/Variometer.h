
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <DataFlash/DataFlash.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Soaring/AP_Soaring.h>

#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

class SoaringController;

class Variometer {

    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &_aparm;
    VarioSavitzkyGolayFilter _vario_sg_filter{};
    SoaringController *_sc;

    float _last_alt;
    float _aspd_filt;
    float _expected_thermalling_sink;
    float _last_aspd;
    float _last_roll;
    float _last_total_E;

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    AverageFilterFloat_Size5 _sp_filter;

    // low pass filter @ 30s time constant
    LowPassFilter<float> _climb_filter;

public:
    Variometer(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, SoaringController *sc);
    float alt;
    float aspd_filt_constrained;
    float reading;
    float filtered_reading;
    float filtered_reading_rate;
    float displayed_reading;
    float raw_climb_rate;
    float smoothed_climb_rate;

    void update(float dt);
    float calculate_aircraft_sinkrate(float phi);

    void reset_filter(float value) { _climb_filter.reset(value); }

    float get_airspeed(void) { return _aspd_filt; };

    float get_exp_thermalling_sink(void) { return _expected_thermalling_sink; };

    float calculate_circling_time_constant();
};