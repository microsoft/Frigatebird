// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "AP_Soaring.h"
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>
extern const AP_HAL::HAL& hal;


// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] =
{
    // @Param: ENABLE
    // @DisplayName: Is the soaring mode enabled or not
    // @Description: Toggles the soaring mode on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, SoaringController, soar_active, 0),

    // @Param: VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("VSPEED", 2, SoaringController, thermal_vspeed, 0.7f),

    // @Param: Q1
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for thermal strength
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for thermal position and radius
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units:
    // @Range: 0 10
    // @User: Advanced

    AP_GROUPINFO("R", 5, SoaringController, thermal_r, 0.45f),

    // @Param: DIST_AHEAD
    // @DisplayName: Distance to thermal center
    // @Description: Initial guess of the distance to the thermal center. If it is non-positive, the initial guess is placed at the UAV's current location.
    // @Units: metres
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_AHEAD", 6, SoaringController, thermal_distance_ahead, 5.0f),

    // @Param: MIN_THML_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling (between cruising sessions)
    // @Units: seconds
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising (between thermalling sessions)
    // @Units: seconds
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_CRSE_S", 8, SoaringController, min_cruise_s, 30),

    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coefficient
    // @Description: Zero lift drag coefficient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, polar_CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_B", 10, SoaringController, polar_B, 0.031),

    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor (2*m*g/(rho*S))
    // @Units: m*m/s/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, polar_K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude
    // @Description: Maximum soaring altitude, relative to the home location.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 12, SoaringController, alt_max, 350.0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude
    // @Description: Minimum soaring altitude, relative to the home location.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 13, SoaringController, alt_min, 50.0),

    // @Param: ALT_CUTOFF
    // @DisplayName: Target power altitude
    // @Description: The altitude (relative to the home location) to which the sailplane UAV should climb under power and then shut the motor down.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // @Param: ENABLE_CH
    // @DisplayName: (Optional) RC channel that toggles the soaring controller on and off
    // @Description: Toggles the soaring controller on and off. This parameter has any effect only if SOAR_ENABLE is set to 1 and this parameter is set to a valid non-zero channel number. When set, soaring will be activated when RC input to the specified channel is greater than or equal to 1700.
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("ENABLE_CH", 15, SoaringController, soar_active_ch, 0),

    // @Param: POMDP_ON
    // @DisplayName: Is the POMDSoar algorithm on?
    // @Description: If 1, the soaring controller uses the POMDSoar algorithm. If 0, the soaring controller uses the ArduSoar algorithm.
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_ON", 16, SoaringController, pomdp_on, 0),

    // @Param: POMDP_N
    // @DisplayName: Number of samples per action trajectory used by POMDSoar
    // @Description: Number of samples per action trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_N", 17, SoaringController, _pomdsoar.pomdp_n, 10),

    // @Param: POMDP_K
    // @DisplayName: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Description: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POMDP_K", 18, SoaringController, _pomdsoar.pomdp_k, 5),

    // @Param: POMDP_HORI
    // @DisplayName: POMDP planning horizon used by POMDSoar.
    // @Description: POMDP planning horizon used by POMDSoar.
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_HORI", 19, SoaringController, _pomdsoar.pomdp_hori, 4.0),

    // @Param: SG_FILTER
    // @DisplayName: use Savitzky Golay vario filter
    // @Description: 0 = 1st order filter (don't use the SG filter), 1 = SG filter
    // @Units: none
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SG_FILTER", 20, SoaringController, sg_filter, 1),

    // @Param: GPS_SYNC
    // @DisplayName: Enable synchronization between vario updates and GPS updates.
    // @Description: Enable synchronization between vario updates and GPS updates. 0 = off, 1 = sync vario update with GPS update.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("GPS_SYNC", 21, SoaringController, gps_sync, 1),

    // @Param: POMDP_STEP_T
    // @DisplayName:POMDP planning step solve time
    // @Description: The amount of computation time the POMDP solver has for computing the next action
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_STEP", 22, SoaringController, _pomdsoar.pomdp_step_t, 1),

    // @Param: POMDP_LOOP
    // @DisplayName: Number of POMDP solver's inner loop executions per planning step
    // @Description: Number of POMDP solver's inner loop executions per planning step (see also the POMDP_STEP_T parameter)
    // @Units:
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_LOOP", 23, SoaringController, _pomdsoar.pomdp_loop_load, 1),

    // @Param: DEBUG
    // @DisplayName: Debug the POMDP solver
    // @Description: Turn on POMDP solver debugging mode. WARNING: to be used on the ground only. Make sure it is set to 0 before takeoff.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 24, SoaringController, debug_mode, 0),

    // @Param: POMDP_ROLL1
    // @DisplayName: POMDP's maximum commanded roll angle.
    // @Description: Maximum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL1", 25, SoaringController, _pomdsoar.pomdp_roll1, 15),

    // @Param: POMDP_ROLL2
    // @DisplayName: POMDP's minimum commanded roll angle.
    // @Description: Minimum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_ROLL2", 26, SoaringController, _pomdsoar.pomdp_roll2, 45),

    // @Param: POMDP_RRATE
    // @DisplayName: The sailplane UAV's roll rate increment used by POMDSoar
    // @Description: The sailplane UAV's roll rate increment used by POMDSoar.
    // @Units: degrees/second
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("POMDP_RRATE", 27, SoaringController, _pomdsoar.pomdp_roll_rate, 75),

    // @Param: POMDP_N_ACT
    // @DisplayName: POMDP number of actions
    // @Description: Number of actions in the POMDP used by POMDSoar. The roll angle input commands corresponding to actions are endpoints of (POMDP_N_ACT-1) equal intervals between POMDP_ROLL2 and POMDP_ROLL1 (inclusive). 
    // @Units: seconds
    // @Range: 1 254
    // @User: Advanced
    AP_GROUPINFO("POMDP_N_ACT", 28, SoaringController, _pomdsoar.pomdp_n_actions, 2),

    // @Param: I_MOMENT
    // @DisplayName: I-moment coefficient
    // @Description: Airframe-specific I-moment coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("I_MOMENT", 29, SoaringController, _pomdsoar.I_moment, 0.00257482),

    // @Param: K_AILERON
    // @DisplayName: Aileron K coefficient
    // @Description: Airframe-specific aileron K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units: seconds
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_AILERON", 30, SoaringController, _pomdsoar.k_aileron, 1.44833047),

    // @Param: K_ROLLDAMP
    // @DisplayName: Roll dampening K coefficient
    // @Description: Airframe-specific roll-dampening K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_ROLLDAMP", 31, SoaringController, _pomdsoar.k_roll_damping, 0.41073589),

    // @Param: ROLL_CLP
    // @DisplayName: CLP coefficient
    // @Description: Airframe-specific CLP coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("ROLL_CLP", 32, SoaringController, _pomdsoar.c_lp, -1.12808702679),

    // @Param: VARIO_TYPE
    // @DisplayName: Vario algorithm type
    // @Description: 0=ArduSoar's, 1=Edwards with cosine bank correction, 2=Edwards with internal bank correction
    // @Units: 
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("VARIO_TYPE", 33, SoaringController, vario_type, 1),

    // @Param: POLY_A
    // @DisplayName: Sink polynomial coefficient a
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_A", 34, SoaringController, poly_a, -0.03099261),

    // @Param: POLY_B
    // @DisplayName: Sink polynomial coefficient b
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_B", 35, SoaringController, poly_b, 0.44731854),

    // @Param: POLY_C
    // @DisplayName: Sink polynomial coefficient c
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units: 
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_C", 36, SoaringController, poly_c, -2.30292972),

    // @Param: POMDP_TH
    // @DisplayName: POMDSoar's threshold on tr(P) for switching between explore and max-lift modes.
    // @Description: POMDSoar's threshold on the P matrix trace for switching between explore and max-lift modes.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_PTH", 37, SoaringController, _pomdsoar.pomdp_pth, 50),

    // @Param: ASPD_SRC
    // @DisplayName: Airspeed source
    // @Description: 0 = airspeed sensor, 1 = wind corrected ground speed
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ASPD_SRC", 38, SoaringController, aspd_src, 1),

    // @Param: EXIT_MODE
    // @DisplayName: Thermal exit mode
    // @Description: Thermal exit mode. 0 = ArduSoar, 1 (recommended) or 2 = POMDP. It's possible to use ArduSoar's thermal exit mode with POMDSoar, but ArduSoar can only use its own thermal exit mode, 0.
    // @Units:
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("EXIT_MODE", 39, SoaringController, exit_mode, 0),

    // @Param: POMDP_NORM
    // @DisplayName: Normalize the P matrix trace when solving the POMDP
    // @Description: Normalize the trace of the P matrix used for switching between explore and max-lift modes in POMDSoar. 0 = no normalizing, 1 = normalize tr(P)
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_NORM", 40, SoaringController, _pomdsoar.pomdp_norm_pth, 0),

    // @Param: NO_STALLPRV
    // @DisplayName: No Stall Prevention
    // @Description: 0 = stall prevention as per configuration, 1 = stall prevention off
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("NO_STALLPRV", 41, SoaringController, disable_soar_prevention, 1),

    // @Param: MCCREADY
    // @DisplayName: McCready vspeed
    // @Description: Min rate of climb to trigger themal exit
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("MCCREADY", 42, SoaringController, mccready_vspeed, 0.7),

    // @Param: GEOFENCE
    // @DisplayName: Enable geofence
    // @Description: Enable geofence. 0 = off, 1 = on
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("GEOFENCE", 43, SoaringController, enable_geofence, 0),

    // @Param: DELTA_MODE
    // @DisplayName: Enable delta actions
    // @Description: Enable delta actions, whereby an action's roll angle is added to the UAV's current roll angle. 0 = off, 1 = on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DELTA_MODE", 44, SoaringController, _pomdsoar.pomdp_delta_mode, 0),

    // @Param: POMDP_EXT
    // @DisplayName: Enable action duration extension in POMDSoar's max-lift mode compared to the explore mode
    // @Description: 0 = off, > 1 = multiplicative factor by which to extend action duration in max-lift compared to the explore mode.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("POMDP_EXT", 45, SoaringController, _pomdsoar.pomdp_extend, 0),

    // @Param: POMDP_PLN
    // @DisplayName: Enable deterministic trajectory planning mode for the POMDP
    // @Description: Enable deterministic trajectory planning mode for the POMDP. 0 = off, 1 on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("POMDP_PLN", 46, SoaringController, _pomdsoar.pomdp_plan_mode, 0),

    // @Param: RUN_TEST
    // @DisplayName: Run a timing test
    // @Description: 0 = off, 1 = exp test
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RUN_TEST", 47, SoaringController, run_timing_test, 0),

    // @Param: ARSP_CMD
    // @DisplayName: Commanded airspeed
    // @Description: Commanded airspeed.
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("ARSP_CMD", 48, SoaringController, aspd_cmd, 9),

    // @Param: TEST_DIST
    // @DisplayName: Initial distance to the test thermal
    // @Description: Initial distance to the test thermal's center from the UAV along the UAV's (straight) path.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_DIST", 49, SoaringController, test_dist, 20),

    // @Param: TEST_OFFSET
    // @DisplayName: Offset of the test thermal
    // @Description: Offset of the test thermal from the UAV's path.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_OFFSET", 50, SoaringController, test_offset, 10),

    // @Param: TEST_RADIUS
    // @DisplayName: Radius of the test thermal
    // @Description: Radius of the test thermal.
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_RADIUS", 51, SoaringController, test_radius, 20),

    // @Param: TEST_STRENGTH
    // @DisplayName: Strength of the test thermal
    // @Description: Strength of the test thermal.
    // @Units: m/s
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TEST_W", 52, SoaringController, test_strength, 2.5),

    AP_GROUPEND
};


SoaringController::SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, AP_Vehicle::FixedWing &parms, AP_RollController  &rollController, AP_Float &scaling_speed) :
    _ahrs(ahrs),
    _spdHgt(spdHgt),
    _aparm(parms),
    _gps(AP::gps()),
    _new_data(false),
    _loiter_rad(parms.loiter_radius),
    _throttle_suppressed(true),
    _prev_stall_prevention(parms.stall_prevention),
    _pomdsoar(this, rollController, scaling_speed),
    _prev_run_timing_test(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    _prev_update_time = AP_HAL::micros64();
    _prev_vario_update_time = AP_HAL::micros64();
    VectorN<float, 3> X = (const float[]) { 0, 0, 0 };
    MatrixN<float, 3> P = (const float[]) { 0.5, 0.5, 0.5 };
    MatrixN<float, 3> Q = (const float[]) { 0.0001, 0.001, 0.001 };
    float R = 0.5;
    _wind_ekf.reset(X, P, Q, R);
}


void
SoaringController::get_target(Location &wp)
{
    if (_pomdsoar.are_computations_in_progress())
    {
        // Do nothing -- POMDSoar doesn't use waypoints, and doesn't modify them. Therefore, don't change wp's value
        // that the caller initialized it to.
    }
    else
    {
        wp = _prev_update_location;
        location_offset(wp, _ekf.X[2], _ekf.X[3]);
    }
}


bool
SoaringController::suppress_throttle()
{
    float alt = 0;
    get_altitude_wrt_home(&alt);

    if (_throttle_suppressed && (alt < alt_min))
    {
        // Time to throttle up
        _throttle_suppressed = false;
    } 
    else if ((!_throttle_suppressed) && (alt > alt_cutoff))
    {
        // Start glide
        _throttle_suppressed = true;
        // Zero the pitch integrator - the nose is currently raised to climb, we need to go back to glide.
        _spdHgt.reset_pitch_I();
        _cruise_start_time_us = AP_HAL::micros64();
        // Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
        // leading to false positives.
        _filtered_vario_reading = 0;
    }

    return _throttle_suppressed;
}


bool
SoaringController::check_thermal_criteria()
{
    //gcs().send_text((MAV_SEVERITY_INFO, "_cruise_start_time_us %lluus %lluus", _cruise_start_time_us, AP_HAL::micros64() - _cruise_start_time_us);
    return (soar_active && !_inhibited
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && ((sg_filter == 1 && _filtered_vario_reading > thermal_vspeed && _filtered_vario_reading_rate < 0)
            || (sg_filter == 0 && _filtered_vario_reading > thermal_vspeed))
            && _alt < alt_max
            && _alt > alt_min);
}

    
bool
SoaringController::check_cruise_criteria()
{
    float thermalability = -1e6;

    if (uses_POMDSoar() && (exit_mode == 1 || exit_mode == 2))
    {
        thermalability = _pomdsoar.assess_thermalability(uint8_t(exit_mode));
    } 
    else 
    {
        _loiter_rad = _aparm.loiter_radius;
        thermalability = (_ekf.X[0] * expf(-powf(_loiter_rad / _ekf.X[1], 2))) - EXPECTED_THERMALLING_SINK;
    }

    _msg_rate++;

    if (_inhibited)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Thermalling inhibited");
        _soaring = false;
        return true;
    }
    else if (soar_active && (AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6) && thermalability < McCready(_alt))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Thml weak: w %f W %f R %f", (double)thermalability, (double)_ekf.X[0], (double)_ekf.X[1]);
        gcs().send_text(MAV_SEVERITY_INFO, "Thml weak: th %f alt %f Mc %f", (double)thermalability, (double)_alt, (double)McCready(_alt));
        _soaring = false;
        return true;
    } 
    else if (soar_active && (_alt>alt_max || _alt<alt_min))
    {
        gcs().send_text(MAV_SEVERITY_ALERT, "Out of allowable altitude range, beginning cruise. Alt = %f\n", (double)_alt);
        _soaring = false;
        return true;
    }
    else if (_msg_rate == 50)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Thermal: w %f W %f R %f", (double)thermalability, (double)_ekf.X[0], (double)_ekf.X[1]);
        _msg_rate = 0;
    }

    return false;
}


bool
SoaringController::is_in_thermal_locking_period()
{
    return ((AP_HAL::micros64() - _thermal_start_time_us) < ((unsigned)min_thermal_s * 1e6));
}


void
SoaringController::init_ekf()
{
    // Calculate filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r = powf(thermal_r, 2);
    float cov_q1 = powf(thermal_q1, 2); // State covariance
    float cov_q2 = powf(thermal_q2, 2); // State covariance
    const float init_q[4] = { cov_q1, cov_q2, cov_q2, cov_q2 };
    const MatrixN<float, 4> q{ init_q };
    const float init_p[4] = { INITIAL_STRENGTH_COVARIANCE, INITIAL_RADIUS_COVARIANCE, INITIAL_POSITION_COVARIANCE, INITIAL_POSITION_COVARIANCE };
    const MatrixN<float, 4> p{ init_p };
    float ground_course = radians(AP::gps().ground_course());
    float head_sin = sinf(ground_course); //sinf(_ahrs.yaw);
    float head_cos = cosf(ground_course); //cosf(_ahrs.yaw);

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    if (thermal_distance_ahead < 0)
    {
        const float init_xr[4] = { INITIAL_THERMAL_STRENGTH,
            INITIAL_THERMAL_RADIUS,
            0,
            0 }; // Thermal location is placed at current location
        const VectorN<float, 4> xr{ init_xr };
        _ekf.reset(xr, p, q, r);
        _nsamples = (unsigned)-thermal_distance_ahead * rate_hz;
        if (_nsamples > EKF_MAX_BUFFER_SIZE) _nsamples = EKF_MAX_BUFFER_SIZE;
        int k = _ptr - _nsamples;
        if (k < 0) k += EKF_MAX_BUFFER_SIZE; // wrap around

        for (unsigned i = 0; i < _nsamples; i++)
        {
            _ekf.update(_ekf_buffer[k][0], _ekf_buffer[k][1], _ekf_buffer[k][2]);
            //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: buff %d %f %f %f", i, (double)_ekf_buffer[k][0], (double)_ekf_buffer[k][1], (double)_ekf_buffer[k][2]);
            k = (k + 1) % EKF_MAX_BUFFER_SIZE;
        }
    }
    else
    {
        const float init_xr[4] = { INITIAL_THERMAL_STRENGTH,
            INITIAL_THERMAL_RADIUS,
            thermal_distance_ahead * head_cos,
            thermal_distance_ahead * head_sin };
        const VectorN<float, 4> xr{ init_xr };
        // Also reset covariance matrix p so filter is not affected by previous data
        _ekf.reset(xr, p, q, r);
    }
}


void
SoaringController::init_thermalling()
{
    _thermal_id++; // bind logs entries to current thermal. First thermal: _thermal_id = 1
    _soaring = true;
    _prev_stall_prevention = _aparm.stall_prevention;

    if (disable_soar_prevention)
    {
        _aparm.stall_prevention = 0;
    }

    get_position(_prev_update_location);
    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
    _pomdsoar.init_thermalling();
}


void
SoaringController::init_cruising()
{
    if (is_active() && suppress_throttle())
    {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        _throttle_suppressed = true;
    }
}


void
SoaringController::get_wind_corrected_drift(const Location *current_loc, const Location *prev_loc,  const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy,
    float *gdx, float *gdy)
{
    Vector2f diff = location_diff(*prev_loc, *current_loc); // get distances from previous update
    *gdx = diff.x;
    *gdy = diff.y;
    
    // Wind correction
    if (debug_mode)
    {
        *wind_drift_x = _debug_in[DBG_WINDX] * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
        *wind_drift_y = _debug_in[DBG_WINDY] * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
    }
    else
    {
        *wind_drift_x = wind->x * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
        *wind_drift_y = wind->y * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
    }

    *dx = *gdx - *wind_drift_x;
    *dy = *gdy - *wind_drift_y;
}


void
SoaringController::get_altitude_wrt_home(float *alt) const
{
    if (debug_mode)
    {
        *alt = _debug_in[DBG_ALT];
    }
    else
    {
        _ahrs.get_relative_position_D_home(*alt);
        *alt *= -1.0f;
    }
}


bool
SoaringController::is_set_to_continue_past_thermal_locking_period()
{
    return _pomdsoar.is_set_to_continue_past_thermal_locking_period();
}


void
SoaringController::update_thermalling()
{
    struct Location current_loc;
    get_position(current_loc);

    if (soar_active		
        && uses_POMDSoar()
        && _pomdsoar.are_computations_in_progress()
        && (is_in_thermal_locking_period() || _pomdsoar.is_set_to_continue_past_thermal_locking_period()))
    {
        bool is_ok_to_stop = _pomdsoar.update_thermalling(current_loc);

        if (is_ok_to_stop && check_cruise_criteria())
        {
            _pomdsoar.stop_computations();
        }
    }
    else 
    {
        _pomdsoar.stop_computations();
    }

    if (_new_data)
    {
        // write log - save the data.
        DataFlash_Class::instance()->Log_Write("SOAR", "TimeUS,id,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt,dx_w,dy_w", "QQfffffffLLfff",
                                               AP_HAL::micros64(),
                                                _thermal_id,
                                               (double)_vario_reading,
                                               (double)_dx,
                                               (double)_dy,
                                               (double)_ekf.X[0],
                                               (double)_ekf.X[1],
                                               (double)_ekf.X[2],
                                               (double)_ekf.X[3],
                                               current_loc.lat,
                                               current_loc.lng,
                                               (double)_alt,
                                               (double)_dx_w,
                                               (double)_dy_w);
        _ekf.update(_vario_reading,_dx, _dy); // update the filter
        _prev_update_location = current_loc; // save for next time
        _prev_update_time = AP_HAL::micros64();
        _new_data = false;
    }
}


void SoaringController::update_cruising()
{
    // Reserved for future tasks that need to run continuously while in FBWB or AUTO mode,
    // for example, calculation of optimal airspeed and flap angle.
}


void SoaringController::get_heading_estimate(float *hdx, float *hdy) const
{
    if (debug_mode)
    {
        *hdx = _debug_in[DBG_GNDDX] - _debug_in[DBG_WINDX];
        *hdy = _debug_in[DBG_GNDDY] - _debug_in[DBG_WINDY];
    }
    else
    {
        Vector2f gnd_vel = _ahrs.groundspeed_vector();
        Vector3f wind = _ahrs.wind_estimate();
        *hdx = gnd_vel.x - wind.x;
        *hdy = gnd_vel.y - wind.y;
    }
}


void SoaringController::get_velocity_estimate(float dt, float *v0) const
{
    float hdx, hdy;
    get_heading_estimate(&hdx, &hdy);
    *v0 = sqrtf(hdx * hdx + hdy * hdy);
}


bool SoaringController::update_vario()
{
    uint64_t now = AP_HAL::micros64();
    const uint32_t vario_rate_dt_ms = (1.0f / rate_hz) * 1e3;
    const uint32_t sched_rate_dt_ms = (1.0f / 50.0f) * 1e3;
    const uint32_t dropout_delay_ms = 1000;
    const uint32_t sched_jitter_ms = sched_rate_dt_ms * 3;
    uint32_t time_since_last_gps_fix_ms = now * 1e-3 - AP::gps().last_fix_time_ms();
    uint32_t time_since_last_vario_update_ms = (now - _prev_vario_update_time) * 1e-3;
    bool gps_drop_out = time_since_last_gps_fix_ms > dropout_delay_ms;

    if ( (gps_sync > 0 && !gps_drop_out && ((time_since_last_vario_update_ms > vario_rate_dt_ms - sched_jitter_ms && time_since_last_gps_fix_ms < sched_jitter_ms)
        || time_since_last_vario_update_ms > vario_rate_dt_ms + sched_jitter_ms))
        || (now - _prev_vario_update_time) > (1.0f / rate_hz) * 1e6)
    {    
        float dt = (now - _prev_vario_update_time) * 1e-6;
        //gcs().send_text(MAV_SEVERITY_INFO, "var: %f %u v0: %f  fix: %ums", dt, _update_rate_ms, _wind_corrected_gspd, AP_HAL::millis() - AP::gps().last_fix_time_ms());
        _vario_updated = true;
        Location current_loc;
        get_position(current_loc);
        get_altitude_wrt_home(&_alt);

        // Both filtered total energy rates and unfiltered are computed for the thermal switching logic and the EKF
        float aspd = 0;
        float roll = get_roll();
        Vector2f gnd_vel = _ahrs.groundspeed_vector();
        float aspd_sensor = 0;

        if (!_ahrs.airspeed_estimate(&aspd_sensor))
        {
            aspd_sensor = _aparm.airspeed_cruise_cm / 100.0f;
        }

        if (debug_mode)
        {
            aspd_sensor = _debug_in[DBG_ASPD];
            gnd_vel.x =_debug_in[DBG_GNDDX];
            gnd_vel.y =_debug_in[DBG_GNDDY];
        }

        if (fabsf(gnd_vel.x) < 100.0f && fabsf(gnd_vel.y) < 100.0f) // prevent wind ekf from getting GPS glitches
        {
            _wind_ekf.update(gnd_vel.x, gnd_vel.y, aspd_sensor);
        }
        //gcs().send_text(MAV_SEVERITY_INFO, "Wind EKF in : %f %f %f",diff.x, diff.y, aspd_sensor);
        //gcs().send_text(MAV_SEVERITY_INFO, "Wind EKF out: %f %f %f",_wind_ekf.X[0],_wind_ekf.X[1],_wind_ekf.X[2]);
        aspd = get_aspd();
        
        if (aspd_src == 2)
        {
            float tau = 2;
            float aspd_dot = (aspd - _aspd_filt) / tau;
            _aspd_filt += aspd_dot * dt;
        }
        else
        {
            _aspd_filt = ASPD_FILT * aspd + (1 - ASPD_FILT) * _aspd_filt;
        }

        float total_E = _alt + 0.5 *_aspd_filt * _aspd_filt / GRAVITY_MSS;   // Work out total energy
        float sinkrate = correct_netto_rate(vario_type, 0.0f, (roll + _last_roll) / 2, _aspd_filt);   // Compute still-air sink rate

        if (debug_mode)
        {
            _vario_reading = _debug_in[DBG_VARIO];
        } 
        else
        {
            _vario_reading = (total_E - _last_total_E) / ((now - _prev_vario_update_time) * 1e-6) + sinkrate;    // Unfiltered netto rate
        }

        if (run_timing_test == 8)
        {
            Vector2f thml_offset = location_diff(_test_thml_loc, current_loc);
            float dist_sqr = thml_offset.x * thml_offset.x + thml_offset.y * thml_offset.y;
            float thml_w = _test_thml_w * expf(-dist_sqr / powf(_test_thml_r, 2));
            _vario_reading += thml_w;
        }

        if (sg_filter == 1)
        {
            _vario_sg_filter.prediction(dt, _ekf_buffer, EKF_MAX_BUFFER_SIZE, _ptr, &_filtered_vario_reading, &_filtered_vario_reading_rate);
        }
        else
        {
            _filtered_vario_reading = TE_FILT * _vario_reading + (1 - TE_FILT) * _filtered_vario_reading;  // Apply low pass timeconst filter for noise
        }

        _displayed_vario_reading = TE_FILT_DISPLAYED * _vario_reading + (1 - TE_FILT_DISPLAYED) * _displayed_vario_reading;
        Vector3f wind = _ahrs.wind_estimate();
        get_wind_corrected_drift(&current_loc, &_prev_vario_update_location, &wind, &_dx_w, &_dy_w, &_dx, &_dy, &_gdx, &_gdy);
        
        if (is_zero(_dx) && is_zero(_dy))
        {
            _dx = 1e-6f;
            _dy = 1e-6f;
        }

        //gcs().send_text(MAV_SEVERITY_INFO, "old lat %d lng %d", _prev_vario_update_location.lat,_prev_vario_update_location.lng);
        //gcs().send_text(MAV_SEVERITY_INFO, "new lat %d lng %d", current_loc.lat,current_loc.lng);
        //gcs().send_text(MAV_SEVERITY_INFO, "dx %f dy %f dt %f", _dx * 1000, _dy * 1000, _mavlink_dt * 1e-6);
        _ekf_buffer[_ptr][0] = _vario_reading;
        _ekf_buffer[_ptr][1] = _dx;
        _ekf_buffer[_ptr][2] = _dy;
        _ekf_buffer[_ptr][3] = _gdx;
        _ekf_buffer[_ptr][4] = _gdy;
        _ptr = (_ptr + 1) % EKF_MAX_BUFFER_SIZE;
        
        if (dt > 0)
        {
            get_velocity_estimate(dt, &_wind_corrected_gspd);
        }
        else
        {
            _wind_corrected_gspd = 0.01;
        }

        if (!_soaring)
        {
            init_ekf();
        }

        // Store variables
        _last_alt = _alt;
        _last_roll = roll;
        _last_aspd = aspd;
        _last_total_E = total_E;
        _prev_vario_update_time = now;
        _prev_vario_update_location = current_loc;
        _new_data = true;
        _vario_updated_reset_random = true;

        DataFlash_Class::instance()->Log_Write("VAR", "TimeUS,aspd_raw,aspd_filt,alt,roll,raw,filt,wx,wy,dx,dy,polar,az", "QffffffffffBf",
                                               AP_HAL::micros64(),
                                               (double)aspd,
                                               (double)_aspd_filt,
                                               (double)_alt,
                                               (double)roll,
                                               (double)_vario_reading,
                                               (double)_filtered_vario_reading,
                                               (double)wind.x,
                                               (double)wind.y,
                                               (double)_dx,
                                               (double)_dy,
                                               (uint8_t)vario_type,
                                               (double)AP::ins().get_accel().z);
        DataFlash_Class::instance()->Log_Write("VEKF", "TimeUS,x0,x1,x2,x3,p0,p1,p2,p3,wx0,wx1,wx2", "Qfffffffffff",
                                               AP_HAL::micros64(),
                                               (double)_ekf.X[0],
                                               (double)_ekf.X[1],
                                               (double)_ekf.X[2],
                                               (double)_ekf.X[3],
                                               (double)_ekf.P(0, 0),
                                               (double)_ekf.P(1, 1),
                                               (double)_ekf.P(2, 2),
                                               (double)_ekf.P(3, 3),
                                               (double)_wind_ekf.X[0],
                                               (double)_wind_ekf.X[1], 
                                               (double)_wind_ekf.X[2]);
        return true;
    } 
    else
    {
        return false;
    }
}


float
SoaringController::correct_netto_rate(int type, float climb_rate, float phi, float aspd) const
{
    if (type == 0) 
    {
        // Remove aircraft sink rate
        float CL0 = 0;  // CL0 = 2*W/(rho*S*V^2)
        float C1 = 0;   // C1 = CD0/CL0
        float C2 = 0;   // C2 = CDi0/CL0 = B*CL0
        float netto_rate;
        float cosphi;
        CL0 = polar_K / (aspd * aspd);
        C1 = polar_CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
        C2 = polar_B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

        cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
        netto_rate = climb_rate + aspd * (C1 + C2 / (cosphi * cosphi));  // effect of aircraft drag removed
        return netto_rate;
    }
    else if (type == 1)
    {
        float netto_rate;
        float cosphi;
        cosphi = (1 - phi * phi / 2); // first two terms of McLaurin series for cos(phi)
        netto_rate = climb_rate - (poly_a * aspd* aspd + poly_b * aspd + poly_c) / cosphi;
        return netto_rate;
    }
    else if (type == 2)
    {
        float netto_rate;
        float az = fabsf(AP::ins().get_accel().z);
        float n = az / GRAVITY_MSS;
        netto_rate = climb_rate - (poly_a * aspd * aspd + poly_b * aspd + poly_c) * powf(n, 1.5);
        return netto_rate;
    }
    return 0;
}


float
SoaringController::McCready(float alt)
{
    // A method shell to be filled in later
    return mccready_vspeed;
}


bool
SoaringController::is_active() const
{
    if (!soar_active)
    {
        return false;
    }

    if (soar_active_ch <= 0)
    {
        // no activation channel
        return true;
    }
    // active when above 1700
    return hal.rcin->read(soar_active_ch-1) >= 1700;
}


bool SoaringController::POMDSoar_active()
{
    return _pomdsoar.are_computations_in_progress();
}


bool
SoaringController::uses_POMDSoar()
{
    return pomdp_on == 1;
}


void
SoaringController::soaring_policy_computation()
{
    if (uses_POMDSoar())
    {
        _pomdsoar.update_internal_state();
    }
}


void
SoaringController::soaring_policy_computation2()
{
    if (uses_POMDSoar())
    {
        _pomdsoar.update_internal_state_test();
    }
}


void
SoaringController::stop_computation()
{
    _pomdsoar.stop_computations();
}


float
SoaringController::get_roll_cmd()
{
    return _pomdsoar.get_action();
}


bool
SoaringController::vario_updated()
{
    if (_vario_updated)
    {
        _vario_updated = false;
        return true;
    }
    else
    {
        return false;
    }
}


void
SoaringController::handle_test_in_msg(mavlink_message_t* msg)
{
    uint64_t now = AP_HAL::micros64();
    _mavlink_dt = now - _prev_mav_test_msg_us;
    _prev_mav_test_msg_us = now;
    _debug_in_mode = mavlink_msg_soar_test_in_get_mode(msg);
    mavlink_msg_soar_test_in_get_data(msg, _debug_in);
}


void
SoaringController::handle_control_msg(mavlink_message_t* msg)
{
    mavlink_soar_control_t packet;
    mavlink_msg_soar_control_decode(msg, &packet);
    _inhibited = packet.inhibit;
}


void
SoaringController::send_status_msg(mavlink_channel_t chan)
{
    float X[4] = { _ekf.X[0], _ekf.X[1], _ekf.X[2], _ekf.X[3] };
    float P[16] = {
    _ekf.P(0, 0), _ekf.P(0,1), _ekf.P(0,2), _ekf.P(0,3),
    _ekf.P(1,0), _ekf.P(1, 1), _ekf.P(1,2), _ekf.P(1,3),
    _ekf.P(2,0), _ekf.P(2,1), _ekf.P(2, 2), _ekf.P(2,3),
    _ekf.P(3,0), _ekf.P(3,1), _ekf.P(3,2), _ekf.P(3, 3) };
    uint8_t soaring_state = 0;

    if(_throttle_suppressed)
    {
        if (_soaring)
        {
            soaring_state = 2;
        }
        else if (_filtered_vario_reading > thermal_vspeed)
        {
            soaring_state = 1;
        }
    }

    uint8_t pomdp_mode = 0;

    if (_pomdsoar.are_computations_in_progress())
    {
        pomdp_mode = 1 + _pomdsoar.get_curr_mode();
    }

    mavlink_msg_soar_status_send(
        chan,
        AP_HAL::micros64(), //<field type="uint64_t" name="time_usec">Timestamp (micros since boot or Unix epoch)</field>
        (uint8_t)soaring_state, //<field name="soaring" type="uint8_t">soaring state: 0 = searching 1 = detected 2 = soaring</field>
        (uint8_t)_inhibited, //<field name="inhibited" type="uint8_t">soaring inhibited=1 uninhibited=0</field>
        pomdp_mode, //<field name="pomdp_mode" type="uint8_t">pomdp mode: 0 = inactive 1 = explore 2 = max_lift </field>
        _pomdsoar.get_latest_pomdp_solve_time(), //<field name="pomdp_solve_time" type="uint64_t">POMDP solve time</field>
        X, //<field name="X" type="float[4]">Thermal state</field>
        P, //<field name="P" type="float[16]">EKF P matrix diagonal</field>
        _vario_reading, //<field name="vario" type="float">Netto vario signal</field>
        _filtered_vario_reading //<field name="vario_filt" type="float">Filtered netto vario signal</field>
        ); 
}


void
SoaringController::send_test_out_msg(mavlink_channel_t chan)
{
    _pomdsoar.send_test_out_msg(chan);
}


float
SoaringController::get_roll() const
{
    if (debug_mode)
    {
        return _debug_in[DBG_ROLL];
    }
    else
    {
        return _ahrs.roll;
    }
}


float
SoaringController::get_rate() const
{
    if (debug_mode)
    {
        return _debug_in[DBG_ROLL_RATE];
    }
    else
    {
        return _ahrs.get_gyro().x;
    }
}


void
SoaringController::get_position(Location& loc)
{
    if (debug_mode)
    {
        int32_t *lat_temp = (int32_t *)(&_debug_in[DBG_LAT]);
        loc.lat = *lat_temp;
        int32_t *lng_temp = (int32_t *)(&_debug_in[DBG_LNG]);
        loc.lng = *lng_temp;
        loc.alt = (int32_t)(_debug_in[DBG_ALT] * 100);
    }
    else
    {
        _ahrs.get_position(loc);
    }
}


float
SoaringController::get_aspd() const
{
    // initialize to an obviously invalid value, which should get overwritten.
    float aspd = -100.0f;

    if (debug_mode)
    {
        aspd = _debug_in[DBG_ASPD];
    }
    else if (aspd_src == 0)
    {
        if (!_ahrs.airspeed_estimate(&aspd))
        {
            aspd = 0.5f*(_aparm.airspeed_min + _aparm.airspeed_max);
        }
    } 
    else if (aspd_src == 1)
    {
        aspd = _wind_corrected_gspd;
    }
    else if (aspd_src == 2)
    {
        if (!_ahrs.airspeed_estimate(&aspd))
        {
            aspd = _aparm.airspeed_cruise_cm / 100.0f;
        }

        aspd -= _wind_ekf.X[0]; // correct sensor aspd for estimated bias
    }

    return aspd;
}


void SoaringController::get_relative_position_wrt_home(Vector2f &vec) const
{
    _ahrs.get_relative_position_NE_home(vec);
}


float
SoaringController::get_eas2tas() const
{
    if (debug_mode)
    {
        return 1.0f;
    }
    else
    {
        return _ahrs.get_EAS2TAS();
    }
}


void
SoaringController::restore_stall_prevention()
{
    _aparm.stall_prevention = _prev_stall_prevention;
}


bool 
SoaringController::soaring()
{
    return _soaring;
}


void
SoaringController::set_soaring(bool state)
{
    _soaring = state;
}


bool
SoaringController::inhibited()
{
    return _inhibited;
}


bool
SoaringController::set_geofence_point(int i, Location& p)
{
    if (i < MAX_NUM_GEOFENCE_POINTS)
    {
        for(int j = 0; j < _num_geofence_points; j++)
        {
            if (_geofence_points[j].lat == p.lat && _geofence_points[j].lng == p.lng)
            {
                return false;
            }
        }

        _geofence_points[i].lat = p.lat;
        _geofence_points[i].lng = p.lng;
        _last_geofence_update_time_ms = AP_HAL::millis();

        if (i >= _num_geofence_points)
        {
            _num_geofence_points = i + 1;
        }

        return true;
    }
    else
    {
        return false;
    }
}


bool
SoaringController::outside_geofence()
{    
    // This implementation assumes a convex geofence
    if (enable_geofence == 0 || _num_geofence_points < 3)
    {
        return false;
    }
    
    Location p;
    get_position(p);
    int pos = 0;
    int neg = 0;
    
    for (int i = 0; i < _num_geofence_points; i++)
    {
        int j = (i + 1) % _num_geofence_points;
        Vector2f diff = location_diff(p, _geofence_points[i]); // get vec from p to point i
        float x1 = diff.y;
        float y1 = diff.x;
        diff = location_diff(_geofence_points[i], _geofence_points[j]); // get vec from point i to point j
        float x2 = diff.y;
        float y2 = diff.x;
        float d = x1 * y2 - y1 * x2;

        if (d < 0)
        {
            neg++;
        }
        else
        {
            pos++;
        }
    }

    if (pos == _num_geofence_points || neg == _num_geofence_points)
    {
        return false;
    } 
    else
    {
        return true;
    }
}


void
SoaringController::run_tests()
{
    if (run_timing_test == 8 && _prev_run_timing_test != 8)
    {
        run_timing_test8();
    }
    else
    {
        _pomdsoar.run_tests();
    }

    _prev_run_timing_test = run_timing_test;
}


void
SoaringController::run_timing_test8()
{
    float hdx, hdy;
    get_heading_estimate(&hdx, &hdy);
    float l = sqrtf(hdx * hdx + hdy * hdy);

    if (l > 0)
    {
        hdx /= l;
        hdy /= l;
    }

    _test_thml_x = hdx * test_dist;
    _test_thml_y = hdy * test_dist;
    _test_thml_x += hdy * test_offset;
    _test_thml_y += -hdx * test_offset;
    _test_thml_r = test_radius;
    _test_thml_w = test_strength;
    get_position(_test_thml_loc);
    location_offset(_test_thml_loc, _test_thml_x, _test_thml_y);
    gcs().send_text(MAV_SEVERITY_INFO, "Injecting Thermal: xd %f yd %f", _test_thml_x, _test_thml_y);
}