/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "Variometer.h"

Variometer::Variometer(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, SoaringController *sc) :
    _ahrs(ahrs),
    _aparm(parms),
    _sc(sc)
{
    _climb_filter = LowPassFilter<float>(1.0/60.0);
}

void Variometer::update(float dt)
{
    _sc->get_altitude_wrt_home(&alt);
	/*
    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
    // take 5 point moving average
    float dsp = _vdot_filter.apply(temp);
	*/


    float aspd = _sc->get_aspd();
    
    if (_sc->aspd_src == 2)
    {
        float tau = 2;
        float aspd_dot = (aspd - _aspd_filt) / tau;
        _aspd_filt += aspd_dot * dt;
    }
    else
    {
        _aspd_filt = _sp_filter.apply(aspd);
    }
    
    // Constrained airspeed.
    const float minV = sqrtf(_sc->polar_K/1.5);
    aspd_filt_constrained = _aspd_filt > minV ? _aspd_filt : minV;

    Vector3f velned;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        raw_climb_rate = -velned.z;
    }
    float tau = calculate_circling_time_constant();
    _climb_filter.set_cutoff_frequency(1/tau);
    smoothed_climb_rate = _climb_filter.apply(raw_climb_rate, dt);

    // Compute still-air sinkrate
    float roll = _ahrs.roll;
    float sinkrate = calculate_aircraft_sinkrate(roll);

    float total_E = alt + 0.5 * aspd_filt_constrained * aspd_filt_constrained / GRAVITY_MSS;   // Work out total energy
    
    if (_sc->debug_mode)
    {
        reading = _sc->_debug_in[DBG_VARIO];
    } 
    else
    {
        //reading = raw_climb_rate + dsp * aspd_filt_constrained/GRAVITY_MSS + sinkrate;
        reading = (total_E - _last_total_E) / dt + sinkrate;    // Unfiltered netto rate
    }

    /*
    // This comes from SoaringController. All SC's variables need to be prefixed with "_sc->"
    if (run_timing_test == 8)
    {
        Vector2f thml_offset = location_diff(_test_thml_loc, current_loc);
        float dist_sqr = thml_offset.x * thml_offset.x + thml_offset.y * thml_offset.y;
        float thml_w = _test_thml_w * expf(-dist_sqr / powf(_test_thml_r, 2));
        _vario.reading += thml_w;
    }
    */
    
    if (_sc->sg_filter == 1)
    {
        _vario_sg_filter.prediction(dt, _sc->_ekf_buffer, EKF_MAX_BUFFER_SIZE, _sc->_ptr, &filtered_reading, &filtered_reading_rate);
    }
    else
    {
        filtered_reading = TE_FILT * reading + (1 - TE_FILT) * filtered_reading;         // Apply low pass timeconst filter for noise
    }

    displayed_reading = TE_FILT_DISPLAYED * reading + (1 - TE_FILT_DISPLAYED) * displayed_reading;

    float expected_roll = atanf(powf(aspd_filt_constrained,2)/(GRAVITY_MSS*_aparm.loiter_radius));
    _expected_thermalling_sink = calculate_aircraft_sinkrate(expected_roll);

    _last_total_E = total_E;
    /*
    DataFlash_Class::instance()->Log_Write("VAR", "TimeUS,aspd_raw,aspd_filt,alt,roll,raw,filt,cl,fc,exs", "Qfffffffff",
                       AP_HAL::micros64(),
                       (double)0.0,
                       (double)aspd_filt_constrained,
                       (double)alt,
                       (double)roll,
                       (double)reading,
                       (double)filtered_reading,
                       (double)raw_climb_rate,
                       (double)smoothed_climb_rate,
                       (double)_expected_thermalling_sink);
                       */
}


float Variometer::calculate_aircraft_sinkrate(float phi)
{
	// The vario type for POMDSoar *must* be 1 
    if (_sc->vario_type == 0) 
    {
        // Remove aircraft sink rate
        float CL0;  // CL0 = 2*W/(rho*S*V^2)
        float C1;   // C1 = CD0/CL0
        float C2;   // C2 = CDi0/CL0 = B*CL0
        CL0 = _sc->polar_K / (aspd_filt_constrained * aspd_filt_constrained);
        C1 = _sc->polar_CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
        C2 = _sc->polar_B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank
        float cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
        return aspd_filt_constrained * (C1 + C2 / (cosphi * cosphi));
    }
    else if (_sc->vario_type == 1)
    {
        float cosphi;
        cosphi = (1 - phi * phi / 2); // first two terms of McLaurin series for cos(phi)
        return (- (_sc->poly_a * aspd_filt_constrained * aspd_filt_constrained + _sc->poly_b * aspd_filt_constrained + _sc->poly_c) / cosphi);
    }
    else if (_sc->vario_type == 2)
    {
        float az = fabsf(AP::ins().get_accel().z);
        float n = az / GRAVITY_MSS;
        return (- (_sc->poly_a * aspd_filt_constrained * aspd_filt_constrained + _sc->poly_b * aspd_filt_constrained + _sc->poly_c) * powf(n, 1.5));
    }
    return 0;
}


float Variometer::calculate_circling_time_constant()
{
    // Calculate a time constant to use to filter quantities over a full thermal orbit.
    // This is used for rejecting variation in e.g. climb rate, or estimated climb rate
    // potential, as the aircraft orbits the thermal.
    // Use the time to circle - variations at the circling frequency then have a gain of 25%
    // and the response to a step input will reach 64% of final value in three orbits.
    return 3 *_aparm.loiter_radius * 2 * M_PI/aspd_filt_constrained;
}
