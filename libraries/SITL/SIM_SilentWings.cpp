/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of SilentWings
*/

#include "SIM_SilentWings.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

SilentWings::SilentWings(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_data_time_ms(0),
    first_pkt_timestamp_ms(0),
    time_base_us(0),
    sock(true),
    home_initialized(false),
    inited_first_pkt_timestamp(false)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // SilentWings keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("127.0.0.1", 6060);

    sock.reuseaddress();
    sock.set_blocking(false);
    
    // TO DO: Force ArduPlane to use sensor data from SilentWings as the actual state,
    // without using EKF, i.e., using "fake EKF". Disable gyro calibration
    // For some reason, setting these parameters succeeds but has no effect...
    /*
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("EK2_ENABLE", 0);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);
    AP_Param::set_default_by_name("ARSPD_ENABLE", 1);
    AP_Param::set_default_by_name("ARSPD_USE", 1);
    */
}

/*
  decode and send servos
*/
void SilentWings::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    // Turn off direct joystick input to the simulator. All joystick commands
    // should go through MissionPlanner and get properly fused with ArduPlane's
    // control inputs when in automatic flight modes.
    float joystick = 0.0f;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    float flap     = 0.0f;

    asprintf(&buf,
             "JOY %f\n"
             "AIL %f\n"
             "ELE %f\n"
             "RUD %f\n"
             "THR %f\n"
             "FLP %f\n",
             joystick, aileron, elevator, rudder, throttle, flap);

    ssize_t buflen = strlen(buf);
    
    ssize_t sent = sock.sendto(buf, buflen, "127.0.0.1", 6070);
    free(buf);

    /*
    printf("Sent AIL %f, ELE %f, RUD %f, THR %f, FLP %f\n",
             aileron, elevator, rudder, throttle, flap);
    */

    if (sent < 0) {
        fprintf(stderr, "Fatal: Failed to send on control socket\n"),
        exit(1);
    }
    
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}


/*
  receive an update from the FDM
  This is a blocking function
 */
bool SilentWings::recv_fdm(void)
{
    fdm_packet pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint32_t now = AP_HAL::millis();

    // TO DO: so far this logic, copied over from X-Plane's SITL, doesn't work very well.
    // Figure out why and enable it?
    /*
    uint32_t wait_time_ms = 1;
    // if we are about to get another frame from SilentWings then wait longer
    if (sw_frame_time > wait_time_ms && now + wait_time_ms >= last_data_time_ms + sw_frame_time) {
        wait_time_ms = 10;
    }
    
    ssize_t nread = sock.recv(&pkt, sizeof(pkt), wait_time_ms);
    */
    
    ssize_t nread = sock.recv(&pkt, sizeof(pkt), 0);
    
    // nread == -1 (255) means no data has arrived
    if (nread != sizeof(pkt)) {
        return finalize_failure();
        //return false;
    }    
    
    // pkt.timestamp is the time of day in SilentWings, measured in ms 
    // since midnight.  
    // TO DO: check what happens when a flight in SW crosses midnight
    if (inited_first_pkt_timestamp) {
        uint64_t tus = (pkt.timestamp - first_pkt_timestamp_ms) * 1.0e3f;
        
        if (tus + time_base_us <= time_now_us) {
            uint64_t tdiff = time_now_us - (tus + time_base_us);
            
            if (tdiff > 1e6f) {
                printf("SilentWings time reset %lu\n", (unsigned long)tdiff);
            }
            
            time_base_us = time_now_us - tus;
        }
        
        time_now_us = time_base_us + tus;
    }
    else {
        first_pkt_timestamp_ms = pkt.timestamp;
        inited_first_pkt_timestamp = true;
    }
    
    dcm.from_euler(radians(pkt.roll), radians(pkt.pitch), radians(pkt.yaw));    
    accel_body = Vector3f(pkt.ax * GRAVITY_MSS, pkt.ay * GRAVITY_MSS, pkt.az * GRAVITY_MSS); // This is g-load.
    gyro = Vector3f(radians(pkt.d_roll), radians(pkt.d_pitch), radians(pkt.d_yaw));
    // SilentWings provides velocity in body frame.
    velocity_ef = dcm * Vector3f(pkt.vx, pkt.vy, pkt.vz);
    // SilentWings also provides velocity in body frame w.r.t. the wind, from which we can infer the wind.
    wind_ef = dcm * (Vector3f(pkt.vx, pkt.vy, pkt.vz) - Vector3f(pkt.vx_wind, pkt.vy_wind, pkt.vz_wind));
    airspeed = pkt.v_eas;
    airspeed_pitot = pkt.v_eas;
    curr_location.lat = pkt.position_latitude * 1.0e7;
    curr_location.lng = pkt.position_longitude * 1.0e7;
    curr_location.alt = pkt.altitude_msl * 100.0f;
    ground_level = curr_location.alt * 0.01f - pkt.altitude_ground;
    Vector3f posdelta = location_3d_diff_NED(home, curr_location);
    position.x = posdelta.x;
    position.y = posdelta.y;
    position.z = posdelta.z;
    update_position();
    
    if (get_distance(curr_location, location) > 4 || abs(curr_location.alt - location.alt)*0.01f > 2.0f || !home_initialized) {
        printf("SilentWings home reset dist=%f alt=%.1f/%.1f\n",
               get_distance(curr_location, location), curr_location.alt*0.01f, location.alt*0.01f);
        // reset home location
        home.lat = curr_location.lat;
        home.lng = curr_location.lng;
        // Resetting altitude reference point in flight can throw off a bunch
        // of important calculations, so let the home altitude always be 0m MSL
        home.alt = 0;
        position.x = 0;
        position.y = 0;
        position.z = -curr_location.alt;
        home_initialized = true;
        update_position();
    }
    
    // Auto-adjust to SilentWings' frame rate
    // This affects the data rate (without this adjustment, the data rate is
    // low no matter what the output_udp_rate in SW's options.dat file is.
    double deltat = (AP_HAL::millis() - last_data_time_ms) / 1000.0f;
    
    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }
    
    if (now > last_data_time_ms && now - last_data_time_ms < 100) {
        sw_frame_time = now - last_data_time_ms;
    }
    
    last_data_time_ms = AP_HAL::millis();
    
    report.data_count++;
    report.frame_count++;
    
    if (0) {
        printf("Delta: %f Time: %" PRIu64 "\n", deltat, time_now_us);
        printf("Accel.x %f\n", accel_body.x);
        printf("Accel.y %f\n", accel_body.y);
        printf("Accel.z %f\n", accel_body.z);
        printf("Gyro.x  %f\n", gyro.x);
        printf("Gyro.y  %f\n", gyro.y);
        printf("Gyro.z  %f\n", gyro.z);
        printf("Pos.x %f\n",   position.x);
        printf("Pos.y %f\n",   position.y);
        printf("Pos.z %f\n",   position.z);
        printf("Roll %f\n",    pkt.roll);
        printf("Pitch %f\n",   pkt.pitch);
        printf("Yaw %f\n",     pkt.yaw);
    }
    
    // printf("Ground level: %f; pos-x: %f; pos-y: %f; pos-z: %f; location-z(alt) in meters: %f; curr_location-z(alt) in meters: %f; alt_msl: %f; alt_ground: %f\n", ground_level, position.x, position.y, position.z, location.alt*0.01f, curr_location.alt*0.01f, pkt.altitude_msl, pkt.altitude_ground);
    // printf("Delta: %f; Time: %d; Lat: %f; Lon: %f; Airspeed: %f; Altitude AGL: %f; Accel-z: %f; Vel-z_ef: %f\n", deltat, pkt.timestamp, pkt.position_latitude, pkt.position_longitude, airspeed, pkt.altitude_ground, pkt.az, velocity_ef[2]);
    
    // data received successfully
    return true;
}


bool SilentWings::finalize_failure()
{
    if (AP_HAL::millis() - last_data_time_ms > 200) {
        // don't extrapolate beyond 0.2s
        return false;
    }

    // advance time by 1ms
    frame_time_us = 1000;
    float delta_time = frame_time_us * 1e-6f;
    time_now_us += frame_time_us;
    extrapolate_sensors(delta_time);
    update_position();
    report.frame_count++;
    return true;
}


/*
  update the SilentWings simulation by one time step
 */
void SilentWings::update(const struct sitl_input &input)
{   
    if (recv_fdm()) {
        send_servos(input);
    }
    
    time_advance();
    update_mag_field_bf();
    
    uint32_t now = AP_HAL::millis();
    
    if (report.last_report_ms == 0) {
        report.last_report_ms = now;
    }
    
    // printf("TIME NOW: %d, TIME OF LAST REPORT: %d\n", now, report.last_report_ms);
    if (now - report.last_report_ms > 5000) {
        float dt = (now - report.last_report_ms) * 1.0e-3f;
        printf("Data rate: %.1f FPS  Frame rate: %.1f FPS\n",
              report.data_count/dt, report.frame_count/dt);
        report.last_report_ms = now;
        report.data_count = 0;
        report.frame_count = 0;
    }    
}

} // namespace SITL
