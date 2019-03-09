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
  simulator connection for ardupilot version of CRRCSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a SilentWings simulator
 */
class SilentWings : public Aircraft {
public:
    SilentWings(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new SilentWings(home_str, frame_str);
    }

private:

    /*
      reply packet sent from SilentWings to ArduPilot
     */
    struct PACKED fdm_packet {
           unsigned int timestamp;          // Millisec  Timestamp
           double position_latitude;        // Degrees   Position latitude,
           double position_longitude;       // Degrees            longitude,
           float  altitude_msl;             // m         Altitude - relative to Sea-level
           float  altitude_ground;          // m         Altitude above gnd
           float  altitude_ground_45;       // m         gnd 45 degrees ahead (NOT IMPLEMENTED YET),
           float  altitude_ground_forward;  // m         gnd straight ahead (NOT IMPLEMENTED YET).
           float  roll;                     // Degrees
           float  pitch;                    // Degrees
           float  yaw;                      // Degrees
           float  d_roll;                    // Deg/sec   Roll speed.
           float  d_pitch;                   // Deg/sec   Pitch speed.
           float  d_yaw;                     // Deg/sec   Yaw speed.
           float  vx;                        // m/sec     Speed vector in body-axis
           float  vy; 
           float  vz;                
           float  vx_wind;                   // m/sec     Speed vector in body-axis, relative to wind
           float  vy_wind;
           float  vz_wind; 
           float  v_eas;                     // m/sec     Equivalent (indicated) air speed. 
           float  ax;                        // m/sec2    Acceleration vector in body axis
           float  ay;
           float  az;
           float  angle_of_attack;          // Degrees   Angle of attack
           float  angle_sideslip;           // Degrees   Sideslip angle
           float  vario;                     // m/sec     TE-compensated variometer.
           float  heading;                   // Degrees   Compass heading.
           float  rate_of_turn;              // Deg/sec   Rate of turn.
           float  airpressure;               // pascal    Local air pressure (at aircraft altitude).
           float  density;                   // Air density at aircraft altitude.
           float  temperature;               // Celcius   Air temperature at aircraft altitude.
    } pkt;
    
    uint32_t sw_frame_time;
    struct {
        int32_t last_report_ms;
        uint32_t data_count;
        uint32_t frame_count;
    } report;

    bool recv_fdm(void);
    void process_packet();
    bool interim_update();
    void send_servos(const struct sitl_input &input);

    uint32_t last_data_time_ms;
    uint32_t first_pkt_timestamp_ms;
    bool inited_first_pkt_timestamp;
    uint64_t time_base_us;
    
    SocketAPM sock;

    Location curr_location;
    bool home_initialized;
};

} // namespace SITL
