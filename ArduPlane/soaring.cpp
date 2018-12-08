// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "Plane.h"
/*
*  ArduSoar support function
*/
void Plane::update_soaring()
{
    if (!g2.soaring_controller.is_active()) {
        return;
    }
    
    g2.soaring_controller.update_vario();
    g2.soaring_controller.run_tests();
    
    if(mission.num_commands() > 2 && (mission.last_change_time_ms() > g2.soaring_controller.get_last_geofence_update_time() + 500 ||
       g2.soaring_controller.get_last_geofence_update_time() == 0)) 
    {
        AP_Mission::Mission_Command cmd;
        int geofence_start_index=0;
        
        // find the start of the geofence waypoints
        for(int cmd_index=0; cmd_index < mission.num_commands(); cmd_index++)
        {
            if(mission.read_cmd_from_storage(cmd_index, cmd))
            {
                if(cmd.id == MAV_CMD_DO_JUMP)
                {
                    geofence_start_index++;
                    break;
                }
            }
            geofence_start_index++;
        }
        
        int i = 0;
        int cmd_index = geofence_start_index;
        g2.soaring_controller.clear_geofence();
        // copy waypoints after the first jump cmd into the geofence
        while (cmd_index < mission.num_commands()) {
            if(mission.read_cmd_from_storage(cmd_index, cmd))
            {
                if(cmd.id == MAV_CMD_NAV_WAYPOINT)
                {
                    if (g2.soaring_controller.set_geofence_point(i,cmd.content.location))
                    {
                        i++;
                    }
                }
            }
            cmd_index++;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Updated %d soaring geofence points", i);
    }
    
    // check the geofence
    if ((control_mode == FLY_BY_WIRE_B || control_mode == LOITER) && g2.soaring_controller.outside_geofence())
    {
        gcs().send_text(MAV_SEVERITY_ALERT, "Outside Geofence Forcing AUTO");
        set_mode(AUTO, MODE_REASON_UNKNOWN);
        return;
    }
    
    if( g2.soaring_controller.soaring() &&
       !((control_mode == FLY_BY_WIRE_B && g2.soaring_controller.POMDSoar_active()) || control_mode == LOITER))
    {
        g2.soaring_controller.set_soaring(false);
        g2.soaring_controller.restore_stall_prevention();
    }
    
    // Check for throttle suppression change.
    switch (control_mode){
    case RTL:
        g2.soaring_controller.set_throttle_suppressed(true);
        break;
    case GUIDED:
            if (previous_mode == MANUAL) {
                gcs().send_text(MAV_SEVERITY_ALERT, "Forcing MANUAL overide of GUIDED mode");
                set_mode(MANUAL, MODE_REASON_UNKNOWN);
                return;
            }
            else if (previous_mode == FLY_BY_WIRE_A) {
                gcs().send_text(MAV_SEVERITY_ALERT, "Forcing FBWA overide of GUIDED mode");
                set_mode(FLY_BY_WIRE_A, MODE_REASON_UNKNOWN);
                return;
            }
    case AUTO:
        g2.soaring_controller.suppress_throttle();
        g2.soaring_controller.stop_computation();
        break;
    case FLY_BY_WIRE_B:
    case CRUISE:
        if (!g2.soaring_controller.suppress_throttle())
        {
            gcs().send_text(MAV_SEVERITY_ALERT, "Out of allowable altitude range, beginning cruise.");
            set_mode(AUTO, MODE_REASON_SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case LOITER:
        // Do nothing. We will switch back to auto/rtl before enabling throttle.
        break;
    default:
        // This does not affect the throttle since suppressed is only checked in the above three modes. 
        // It ensures that the soaring always starts with throttle suppressed though.
        g2.soaring_controller.set_throttle_suppressed(true);
        break;
    }

    // Nothing to do if we are in powered flight
    if (!g2.soaring_controller.get_throttle_suppressed() && aparm.throttle_max > 0)
    {
        return;
    }

    switch (control_mode)
    {
    case AUTO:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case GUIDED:
        // Test for switch into thermalling mode
        g2.soaring_controller.update_cruising();
        if (g2.soaring_controller.POMDSoar_active())
        {
            g2.soaring_controller.update_thermalling();
            g2.soaring_controller.get_target(next_WP_loc);

            if (!g2.soaring_controller.POMDSoar_active())
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar completed");

                if (g2.soaring_controller.is_set_to_continue_past_thermal_locking_period())
                {
                    switch (previous_mode)
                    {
                        case FLY_BY_WIRE_B:
                            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar ended, entering AUTO");
                            // AUTO is our default autonomous-operation mode
                            set_mode(AUTO, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                            break;
                            
                        case CRUISE: {
                            // return to cruise with old ground course
                            CruiseState cruise = cruise_state;
                            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar ended, restoring CRUISE");
                            set_mode(CRUISE, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                            cruise_state = cruise;
                            set_target_altitude_current();
                            break;
                        }
                            
                        case AUTO:
                            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar ended, restoring AUTO");
                            set_mode(AUTO, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                            break;
                            
                        case GUIDED:
                            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar ended, restoring GUIDED");
                            set_mode(GUIDED, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                            break;
                            
                        default:
                            break;
                    }
                }
                else
                {
                    enum FlightMode temp_mode = previous_mode;
                    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Entering Loiter");
                    set_mode(LOITER, MODE_REASON_SOARING_THERMAL_DETECTED);
                    previous_mode = temp_mode; // restore back to original mode, not FBWB of POMDSoar phase
                }
            }
        }
        else if (g2.soaring_controller.check_thermal_criteria())
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected");
            if (g2.soaring_controller.uses_POMDSoar())
            {
                // Start POMDSoar
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: POMDSoar started");
                g2.soaring_controller.init_thermalling();
                g2.soaring_controller.get_target(next_WP_loc);
                set_mode(FLY_BY_WIRE_B, MODE_REASON_SOARING_THERMAL_DETECTED);
            }
            else 
            {
                // Start ArduSoar
                g2.soaring_controller.init_thermalling();
                g2.soaring_controller.get_target(next_WP_loc);
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Entering Loiter");
                set_mode(LOITER, MODE_REASON_SOARING_THERMAL_DETECTED);
            }
        }
        break;

    case LOITER:
        // Stop POMDSoar's computations, if they are in progress
        g2.soaring_controller.stop_computation();
        // Update thermal estimate and check for switch back to AUTO
        g2.soaring_controller.update_thermalling();  // Update estimate

        if (g2.soaring_controller.check_cruise_criteria())
        {
            // Exit as soon as thermal state estimate deteriorates
            switch (previous_mode) {
            case FLY_BY_WIRE_B:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, entering AUTO");
                // AUTO is our default autonomous-operation mode
                set_mode(AUTO, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            case CRUISE: {
                // return to cruise with old ground course
                CruiseState cruise = cruise_state;
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring CRUISE");
                set_mode(CRUISE, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                cruise_state = cruise;
                set_target_altitude_current();
                break;
            }

            case AUTO:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring AUTO");
                set_mode(AUTO, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;
                    
            case GUIDED:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring GUIDED");
                set_mode(GUIDED, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            default:
                break;
            }
        } else {
            // still in thermal - need to update the wp location
            g2.soaring_controller.get_target(next_WP_loc);
        }
        break;
        
    case MANUAL:
        g2.soaring_controller.stop_computation();

    default:
        // nothing to do
        break;
    }
}


void Plane::soaring_policy_computation()
{
    g2.soaring_controller.soaring_policy_computation();
}


void Plane::soaring_policy_computation2()
{
    g2.soaring_controller.soaring_policy_computation2();
}