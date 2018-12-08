// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include <GCS_MAVLink/GCS.h>
#include "AP_Soaring.h"
#include "POMDSoar.h"

POMDSoarAlgorithm::POMDSoarAlgorithm(const SoaringController *sc, AP_RollController &rollController, AP_Float &scaling_speed) :
    _sc(sc),
    _gains(rollController.get_gains()),
    _scaling_speed(scaling_speed)
{
    _prev_pomdp_update_time = AP_HAL::micros64();
}


void
POMDSoarAlgorithm::init_actions(bool mode)
{
    _n_actions = MIN(pomdp_n_actions, MAX_ACTIONS);
    float max_roll = fmax(pomdp_roll1 * _sign, pomdp_roll2 * _sign);
    float min_roll = fmin(pomdp_roll1 * _sign, pomdp_roll2 * _sign);

    if (mode)
    {
        float new_max_roll = _pomdp_roll_cmd + pomdp_roll_rate;
        float new_min_roll = _pomdp_roll_cmd - pomdp_roll_rate;

        if (new_max_roll > max_roll)
        {
            new_max_roll = max_roll;
            new_min_roll = max_roll - 2 * pomdp_roll_rate;
        }

        if (new_min_roll < min_roll)
        {
            new_min_roll = min_roll;
            new_max_roll = min_roll + 2 * pomdp_roll_rate;
        }

        float roll = new_max_roll;
        float roll_rate = (new_max_roll - new_min_roll) / (_n_actions - 1);
        for (int i = 0; i < _n_actions; i++) {
            _roll_cmds[i] = roll;
            //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
            roll -= roll_rate;
        }
    }
    else
    {
        float roll = max_roll;
        float roll_rate = (max_roll - min_roll) / (_n_actions - 1);
        for (int i = 0; i < _n_actions; i++)
        {
            _roll_cmds[i] = roll;
            //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
            roll -= roll_rate;
        }
    }

    _prev_n_actions = _n_actions;
}


void
POMDSoarAlgorithm::init_thermalling()
{
    float ground_course = radians(AP::gps().ground_course());
    float head_sin = sinf(ground_course);
    float head_cos = cosf(ground_course);
    float xprod = _sc->_ekf.X[3] * head_cos - _sc->_ekf.X[2] * head_sin;
    _sign = xprod <= 0 ? -1.0 : 1.0;
    _pomdp_roll_cmd = pomdp_roll1 * _sign;
    init_actions(pomdp_delta_mode);
    float aspd = _sc->get_aspd();
    float eas2tas = _sc->get_eas2tas();
    // This assumes that SoaringController called get_position(_prev_update_location) right before this call to init_thermalling
    _pomdp_wp = _sc->_prev_update_location;
    _sc->get_relative_position_wrt_home(_pomdp_vecNE);
    //printf("_pomdp_wp = %f %f\n", ((double)_pomdp_wp.lat) * 1e-7, ((double)_pomdp_wp.lng) * 1e-7);
    float hdx, hdy;
    _sc->get_heading_estimate(&hdx, &hdy);
    float wind_corrected_heading = atan2f(hdy, hdx);

    // Prepare everything necessary for generating action trajectories (by modelling trajectories resulting from various commanded roll angles)
    _solver.set_pid_gains(_gains.P, _gains.I, _gains.D, _gains.FF, _gains.tau, _gains.imax, _gains.rmax, _scaling_speed);
    _solver.set_polar(float(_sc->poly_a), float(_sc->poly_b), float(_sc->poly_c));
    _n_action_samples = pomdp_hori * pomdp_k;
    float pomdp_aspd = aspd;

    if (_sc->aspd_cmd > 0)
    {
        pomdp_aspd = _sc->aspd_cmd;

        if (_sc->aspd_src == 2)
        {
            pomdp_aspd -= _sc->_wind_ekf.X[0]; // correct for sensor bias using wind ekf
        }
    }

    _solver.generate_action_paths(pomdp_aspd, eas2tas, wind_corrected_heading, degrees(_sc->get_roll()),
        degrees(_sc->get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
        pomdp_step_t, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), 0);
    _m = 0;
    _solver.init_step(pomdp_loop_load, pomdp_n, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, false);
    // This assumes that SoaringController updated _thermal_start_time_us right before this call to init_thermalling
    _prev_pomdp_update_time = _sc->_thermal_start_time_us;
    _prev_pomdp_wp = _sc->_prev_update_location;
    _pomdp_active = true;
    _pomdp_mode = 0;
    _prev_pomdp_action = _sign > 0 ? _n_actions - 1 : 0;
}


float
POMDSoarAlgorithm::assess_thermalability(uint8_t exit_mode)
{
    float thermalability = -1e6;
    float aspd = _sc->get_aspd();

    if (exit_mode == 1)
    {
        float expected_thermalling_sink = _sc->correct_netto_rate(1, 0.0f, radians(_pomdp_roll_cmd), aspd);
        float dist_sqr = _sc->_ekf.X[2] * _sc->_ekf.X[2] + _sc->_ekf.X[3] * _sc->_ekf.X[3];
        thermalability = (_sc->_ekf.X[0] * expf(-dist_sqr / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink;
    }
    else if (exit_mode == 2)
    {
        int n_samps = 20;
        float theta_step = fmax(pomdp_roll1, pomdp_roll2) / (float)n_samps;
        float theta = 0;

        for (int i = 0; i < n_samps; i++)
        {
            float expected_thermalling_sink = _sc->correct_netto_rate(1, 0.0f, radians(theta), aspd);
            float r = (aspd * aspd) / (GRAVITY_MSS * tanf(radians(theta)));
            thermalability = MAX(thermalability, (_sc->_ekf.X[0] * expf(-(r*r) / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink);
            theta += theta_step;
        }
    }

    return thermalability;
}


bool
POMDSoarAlgorithm::are_computations_in_progress()
{
    return _pomdp_active;
}


void
POMDSoarAlgorithm::update_internal_state()
{
    if (_solver.running())
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.update();
        _pomp_loop_time = AP_HAL::micros64() - start_time;
        //gcs().send_text(MAV_SEVERITY_INFO, "slice time: %lluus  samps: %d", _pomp_loop_time, samps);
        _pomp_loop_min_time = (_pomp_loop_min_time > _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_min_time;
        _pomp_loop_max_time = (_pomp_loop_max_time < _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_max_time;
        _pomp_loop_av_time = _pomp_loop_av_time * 0.9 + _pomp_loop_time * 0.1;
    }
}


void
POMDSoarAlgorithm::update_internal_state_test()
{
    _solver.update_test();
}


void
POMDSoarAlgorithm::stop_computations()
{
    _pomdp_active = false;
}


bool
POMDSoarAlgorithm::is_set_to_continue_past_thermal_locking_period()
{
    return (pomdp_pth > 0);
}


int
POMDSoarAlgorithm::get_curr_mode()
{
    return _pomdp_mode;
}


uint64_t
POMDSoarAlgorithm::get_latest_pomdp_solve_time()
{
    return _pomdp_solve_time;
}


float
POMDSoarAlgorithm::get_action()
{
    return _pomdp_roll_cmd * 100;
}


void
POMDSoarAlgorithm::send_test_out_msg(mavlink_channel_t chan)
{
    /*
    0: pomdp_wp.lat (y0)
    4: pomdp_wp.lng (x0)
    8:  v0
    10: psi0
    12: x1,y1,psi1,x2,y2,psi2,x3,y3,psi3,x4,y4,psi4
    36: x1,y1,psi1,x2,y2,psi2,x3,y3,psi3,x4,y4,psi4
    60: a1
    61: a2
    62-63: 2 bytes spare
    */
    uint8_t *_debug_out_bytes = (uint8_t *)&_debug_out[0];
    uint8_t *a1 = &_debug_out_bytes[60];
    uint8_t *a2 = &_debug_out_bytes[61];

    if (_pomdp_active && _m < _solver.actions_generated())
    {
        _debug_out_mode = 0;
        int32_t *pos_ptr = (int32_t *)(&_debug_out[0]);
        pos_ptr[0] = _pomdp_wp.lat;
        pos_ptr[1] = _pomdp_wp.lng;

        int16_t *v0 = (int16_t *)&_debug_out_bytes[8];
        *v0 = (int16_t)(_solver.get_action_v0() * 256);

        int16_t *psi0 = (int16_t *)&_debug_out_bytes[10];
        *psi0 = (int16_t)(_solver.get_action_path_psi(0, 0) * 256);

        int16_t *action1 = (int16_t *)&_debug_out_bytes[12];
        int16_t *action2 = (int16_t *)&_debug_out_bytes[36];

        int k_step = _n_action_samples / 4;
        *a1 = _m;
        int k = k_step;

        for (int i = 0; i < 12; i += 3)
        {
            action1[i] = (int16_t)(_solver.get_action_path_x(_m, k) * 256);
            action1[i + 1] = (int16_t)(_solver.get_action_path_y(_m, k) * 256);
            action1[i + 2] = (int16_t)(_solver.get_action_path_psi(_m, k) * 256);
            k += k_step;
        }

        _m++;

        if (_m < _n_actions && _m < _solver.actions_generated())
        {
            *a2 = _m;
            k = k_step;

            for (int i = 0; i < 12; i += 3) {
                action2[i] = (int16_t)(_solver.get_action_path_x(_m, k) * 256);
                action2[i + 1] = (int16_t)(_solver.get_action_path_y(_m, k) * 256);
                action2[i + 2] = (int16_t)(_solver.get_action_path_psi(_m, k) * 256);
                k += k_step;
            }
        }
        else
        {
            *a2 = 255;
        }

        _m++;

        if (_m >= _n_actions)
        {
            _m = 0;
        }
    }
    else
    {
        _debug_out_mode = 0;
        *a1 = 255; // action = 255 to signal no more action data
        *a2 = 255; // action = 255 to signal no more action data
    }

    mavlink_msg_soar_test_out_send(
        chan,
        _debug_out_mode,
        _debug_out); //<field name = "debug" type = "float[16]">Debug results< / field>
}


bool
POMDSoarAlgorithm::update_thermalling(const Location &current_loc)
{
    bool is_ok_to_stop = false;

    if (!_solver.running())
    {
        uint8_t action = 254;
        _pomdp_solve_time = AP_HAL::micros64() - _prev_pomdp_update_time;
        float pdx = 0;
        float pdy = 0;
        /*gcs().send_text(MAV_SEVERITY_INFO, "Soaring: loop %dms %lluus %lluus %lluus",
        (int)((double)(AP_HAL::micros64() - _prev_pomdp_update_time) * (double)1e-3),
        _pomp_loop_av_time,
        _pomp_loop_min_time,
        _pomp_loop_max_time);*/
        //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: X %f %f %f %f", (double)_ekf.X[0], (double)_ekf.X[1], (double)_ekf.X[2], (double)_ekf.X[3]);
        //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: P %f %f %f %f", (double)_ekf.P(0, 0), (double)_ekf.P(1, 1), (double)_ekf.P(2, 2), (double)_ekf.P(3, 3));
        float eas2tas = _sc->get_eas2tas();
        float aspd = _sc->get_aspd();
        action = (uint8_t)_solver.get_best_action();
        _pomdp_roll_cmd = _roll_cmds[action];
        pdx = _pomdp_vecNE.y;
        pdy = _pomdp_vecNE.x;
        _sc->get_relative_position_wrt_home(_pomdp_vecNE);
        float hdx, hdy;
        _sc->get_heading_estimate(&hdx, &hdy);
        float wind_corrected_heading = atan2f(hdy, hdx);
        //gcs().send_text(MAV_SEVERITY_INFO, "head %f %f %f", hdx, hdy, wind_corrected_heading);
        float n[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

        if (pomdp_norm_pth)
        {
            n[0] = fabsf(_sc->_ekf.X[0]) > 0.0f ? fabsf(_sc->_ekf.X[0]) : 1.0f;
            n[1] = fabsf(_sc->_ekf.X[1]) > 0.0f ? fabsf(_sc->_ekf.X[1]) : 1.0f;
            n[2] = n[1];
            n[3] = n[1];
        }

        float trP = _sc->_ekf.P(0, 0) / n[0] + _sc->_ekf.P(1, 1) / n[1] + _sc->_ekf.P(2, 2) / n[2] + _sc->_ekf.P(3, 3) / n[3];
        bool max_lift = trP < pomdp_pth && pomdp_pth > 0.0f;
        init_actions(max_lift || pomdp_delta_mode);
        int extend = 0;
        _n_action_samples = pomdp_hori * pomdp_k;

        if (max_lift)
        {
            extend = pomdp_extend;
            _n_action_samples = MIN(MAX_ACTION_SAMPLES, int(pomdp_hori * pomdp_k * extend));
        }

        int n_samples = pomdp_n;
        float step_w = 1.0f;

        if (max_lift && pomdp_plan_mode)
        {
            n_samples = 1;
            step_w = 1.0f / pomdp_n;
        }

        float pomdp_aspd = aspd;

        if (_sc->aspd_cmd > 0) {
            pomdp_aspd = _sc->aspd_cmd;

            if (_sc->aspd_src == 2) {
                pomdp_aspd -= _sc->_wind_ekf.X[0]; // correct for sensor bias using wind ekf
            }
        }

        _solver.generate_action_paths(pomdp_aspd, eas2tas, wind_corrected_heading, degrees(_sc->get_roll()), degrees(_sc->get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
            pomdp_step_t * step_w, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), extend);
        _m = 0;
        _solver.init_step(pomdp_loop_load, n_samples, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, max_lift);

        if (max_lift)
        {
            _pomdp_mode = 1;
            //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDP maxlift %f",(double)trP);
        }
        else
        {
            _pomdp_mode = 0;
            //gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDP explore %f",(double)trP);
        }

        _prev_pomdp_update_time = AP_HAL::micros64();
        _prev_pomdp_action = action;

        if (is_set_to_continue_past_thermal_locking_period())
        {
            is_ok_to_stop = true;
        }

        DataFlash_Class::instance()->Log_Write("POMP", "TimeUS,id,n,k,action,x,y,sign,lat,lng,rlat,rlng,roll,mode,Q", "QQHHBfffLLLLfB",
            AP_HAL::micros64(),
            _sc->_thermal_id,
            pomdp_n,
            pomdp_k,
            action,
            (double)pdx,
            (double)pdy,
            (double)_sign,
            current_loc.lat,
            current_loc.lng,
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_pomdp_roll_cmd,
            (uint8_t)max_lift,
            (double)_solver.get_action_Q(action));
        DataFlash_Class::instance()->Log_Write("POMT", "TimeUS,id,loop_min,loop_max,loop_time,solve_time,load,n,k", "QQQQQQHHH",
            AP_HAL::micros64(),
            _sc->_thermal_id,
            _pomp_loop_min_time,
            _pomp_loop_max_time,
            _pomp_loop_time,
            _pomdp_solve_time,
            pomdp_loop_load,
            pomdp_n,
            pomdp_k
        );
        DataFlash_Class::instance()->Log_Write("PDBG", "TimeUS,lat,lng,v0,psi,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6", "QLLffffffffffffff",
            AP_HAL::micros64(),
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_solver.get_action_v0(),
            (double)_solver.get_action_path_psi(action, 0),
            (double)_solver.get_action_path_x(action, 1),
            (double)_solver.get_action_path_y(action, 1),
            (double)_solver.get_action_path_x(action, 2),
            (double)_solver.get_action_path_y(action, 2),
            (double)_solver.get_action_path_x(action, 3),
            (double)_solver.get_action_path_y(action, 3),
            (double)_solver.get_action_path_x(action, 4),
            (double)_solver.get_action_path_y(action, 4),
            (double)_solver.get_action_path_x(action, 5),
            (double)_solver.get_action_path_y(action, 5),
            (double)_solver.get_action_path_x(action, 6),
            (double)_solver.get_action_path_y(action, 6)
        );

        _pomdp_wp = current_loc;
        _prev_pomdp_wp = current_loc;
        _pomp_loop_min_time = (unsigned long)1e12;
        _pomp_loop_max_time = 0;
    }

    return is_ok_to_stop;
}


void
POMDSoarAlgorithm::run_tests()
{
    if (_sc->run_timing_test == 1)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_exp_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring exp test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 2)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_fast_exp_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fast exp test: %llu us", total_time);

    }
    else if (_sc->run_timing_test == 3)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.fill_random_array();
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 4)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_rnd_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring rnd test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 5)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_ekf_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring EKF test: %llu us", total_time);
    }
    else if (_sc->run_timing_test == 6)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, false);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, true);
        uint64_t maxlift_total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring loop test: %llu us ML: %llu us", total_time, maxlift_total_time);
    }
    else if (_sc->run_timing_test == 7)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_multivariate_normal_sample_test(1000);
        uint64_t total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring multivariate_normal test: %llu us", total_time);
    } // test #8 is run by the SoaringController class itself
    else if (_sc->run_timing_test == 9)
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.run_trig_box_muller_test(1000);
        uint64_t total_trig_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_polar_box_muller_test(1000);
        uint64_t total_polar_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring box-muller trig: %llu us polar: %llu us", total_trig_time, total_polar_time);
    }

    _prev_run_timing_test = _sc->run_timing_test;
}