// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

/*
  Provides a layer between the thermal centring algorithm and the main
  code for managing navigation targets, data logging, tuning parameters,
  algorithm inputs and eventually other soaring strategies such as
  speed-to-fly. AP_TECS libary used for reference.
*/

#ifndef AP_Soaring_h
#define AP_Soaring_h

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <APM_Control/APM_Control.h>
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>
#include "ExtendedKalmanFilter.h"
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include "WindExtendedKalmanFilter.h"
#include "VarioSavitzkyGolayFilter.h"
#include "Variometer.h"
#include "POMDSoar.h"


#define EXPECTED_THERMALLING_SINK 0.7
#define INITIAL_THERMAL_STRENGTH 2.5
#define INITIAL_THERMAL_RADIUS 30.0
#define INITIAL_STRENGTH_COVARIANCE 0.05
#define INITIAL_RADIUS_COVARIANCE 2500.0
#define INITIAL_POSITION_COVARIANCE 300.0
#define EKF_MAX_BUFFER_SIZE 300
#define MAX_NUM_GEOFENCE_POINTS 12
#define SOARING_RND_UPDATE_RATE 20 // 260 microseconds of Box Miller 4D rnd samples

#define DBG_ASPD 1
#define DBG_LAT 2
#define DBG_LNG 3
#define DBG_ALT 4
#define DBG_ROLL 5
#define DBG_ROLL_RATE 6
#define DBG_WINDX 7
#define DBG_WINDY 8
#define DBG_VARIO 9
#define DBG_GNDDX 10
#define DBG_GNDDY 11

class POMDSoarAlgorithm;

//
// Soaring Controller class
//
class SoaringController
{
    friend class POMDSoarAlgorithm;

    ExtendedKalmanFilter _ekf{};
    WindExtendedKalmanFilter _wind_ekf{};
    VarioSavitzkyGolayFilter _vario_sg_filter{};
    AP_AHRS &_ahrs;
    AP_SpdHgtControl &_spdHgt;
    AP_Vehicle::FixedWing &_aparm;
    const AP_GPS &_gps;
    const float rate_hz = 5;

    // store aircraft location at last update
    struct Location _prev_update_location;
    struct Location _prev_vario_update_location;

    // store time thermal was entered for hysteresis
    uint64_t _thermal_start_time_us;

    // store time cruise was entered for hysteresis
    uint64_t _cruise_start_time_us;

    // store time of last update
    uint64_t _prev_update_time;

    // store time of last update of the vario
    uint64_t _prev_vario_update_time;

    float _vario_reading;
    bool _vario_updated = false;
    float _filtered_vario_reading;
    float _filtered_vario_reading_rate;
    float _last_alt;
    float _alt;
    float _last_aspd;
    float _last_roll;
    float _last_total_E;
    bool _new_data;
    bool _throttle_suppressed;
    float _ekf_buffer[EKF_MAX_BUFFER_SIZE][5];
    unsigned _nsamples;
    unsigned _ptr = 0; // index into the _ekf_buffer
    bool _soaring = false;
    bool _inhibited = false;
    uint64_t _thermal_id = 0;
    float _wind_corrected_gspd = 0.01;
    float _displayed_vario_reading;
    float _aspd_filt;
    float correct_netto_rate(int type, float climb_rate, float phi, float aspd) const;
    float McCready(float alt);
    void get_wind_corrected_drift(const Location *current_loc, const Location *prev_loc, const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy, float *gdx, float *gdy);
    void get_altitude_wrt_home(float *alt) const;
    int _msg_rate = 0;
    float _debug_in[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t _debug_in_mode;
    int8_t _prev_stall_prevention;
    float _dx = 0;
    float _dy = 0;
    float _dx_w = 0;
    float _dy_w = 0;
    float _gdx = 0;
    float _gdy = 0;
    Location _geofence_points[MAX_NUM_GEOFENCE_POINTS];
    int _num_geofence_points = 0;
    uint32_t _last_geofence_update_time_ms = 0;
    uint64_t _prev_mav_test_msg_us = 0;
    uint64_t _mavlink_dt = 1;
    bool _vario_updated_reset_random = false;
    float _test_thml_x = 20;
    float _test_thml_y = 20;
    float _test_thml_r = 20;
    float _test_thml_w = 2.5;
    Location _test_thml_loc;
    uint8_t _prev_run_timing_test;
    POMDSoarAlgorithm _pomdsoar;

protected:	
    AP_Int8 soar_active;
    AP_Int8 soar_active_ch;
    AP_Float thermal_vspeed;
    AP_Float thermal_q1;
    AP_Float thermal_q2;
    AP_Float thermal_r;
    AP_Float thermal_distance_ahead;
    AP_Int16 min_thermal_s;
    AP_Int16 min_cruise_s;
    AP_Float polar_CD0;
    AP_Float polar_B;
    AP_Float polar_K;
    AP_Float alt_max;
    AP_Float alt_min;
    AP_Float alt_cutoff;
    AP_Int8 debug_mode;
    AP_Int8 pomdp_on;
    AP_Int8 vario_type;
    AP_Float poly_a;
    AP_Float poly_b;
    AP_Float poly_c;
    AP_Int8 aspd_src;
    AP_Int8 exit_mode;
    AP_Int8 disable_soar_prevention;
    AP_Float mccready_vspeed;
    AP_Int8 enable_geofence;
    AP_Int8 run_timing_test;
    AP_Int8 sg_filter;
    AP_Int8 gps_sync;
    AP_Float aspd_cmd;
    AP_Float test_dist;
    AP_Float test_offset;
    AP_Float test_radius;
    AP_Float test_strength;

public:
    SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, AP_Vehicle::FixedWing &parms, AP_RollController &rollController, AP_Float &scaling_speed);
    
    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    void get_target(Location & wp);
    bool suppress_throttle();
    bool check_thermal_criteria();
    bool check_cruise_criteria();
    bool is_in_thermal_locking_period();
    void init_ekf();
    void init_thermalling();
    void init_cruising();
    void update_thermalling();
    void update_cruising();
    bool is_set_to_continue_past_thermal_locking_period();
    bool is_active() const;
    bool get_throttle_suppressed() const { return _throttle_suppressed; }
    void set_throttle_suppressed(bool suppressed) { _throttle_suppressed = suppressed;  }
    float get_vario_reading() { return _displayed_vario_reading; }
    bool update_vario();
    void soaring_policy_computation();
    void soaring_policy_computation2();
    void stop_computation();
    bool POMDSoar_active();
    bool uses_POMDSoar();
    bool vario_updated();
    float get_roll_cmd();
    void send_test_out_msg(mavlink_channel_t chan);
    void send_status_msg(mavlink_channel_t chan);
    void handle_control_msg(mavlink_message_t* msg);
    void handle_test_in_msg(mavlink_message_t* msg);
    void get_relative_position_wrt_home(Vector2f &vec) const;
    float get_aspd() const;
    void restore_stall_prevention();
    bool soaring();
    void set_soaring(bool state);
    bool inhibited();
    void clear_geofence() { _num_geofence_points = 0; }
    bool set_geofence_point(int i, Location& p);
    uint32_t get_last_geofence_update_time() { return _last_geofence_update_time_ms; };
    bool outside_geofence();
    void get_position(Location& loc);
    float get_rate() const;
    float get_roll() const;
    float get_eas2tas() const;
    void run_tests();
    void get_heading_estimate(float *hdx, float *hdy) const;
    void get_velocity_estimate(float dt, float *v0) const;
    void run_timing_test8();
};

#endif