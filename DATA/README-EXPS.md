# Reproducing the experimental setup from the RSS-2018 paper

The data files in <https://github.com/Microsoft/Frigatebird/DATA> directory allow reproducing the experiment setup from

I. Guilliard, R. Rogahn, J. Piavis, A. Kolobov. "Autonomous Thermalling as a Partially Observable Markov Decision Process." Robotics: Science and Systems conference, 2018. <https://arxiv.org/pdf/1805.09875.pdf> 

Specifically,

- File [RSS-2018_Radian_Pro_parameters.param](https://github.com/Microsoft/Frigatebird/DATA/RSS-2018_Radian_Pro_parameters.param) is an ArduPilot parameter file that contains parameter values for configuring a Radian Pro airframe equipped as described in the Empirical Evaluation section of the paper above into a sailplane sUAV running POMDSoar or, if the SOAR_POMDP_ON parameter is set to 0, running ArduSoar as the thermalling controller. This file can be uploaded to the sUAV's flight controller (e.g., Pixhawk) using ground control station software such as [Mission Planner](http://ardupilot.org/planner/).

  NOTE #1: For safety reasons, parameters related to battery, AHRS system, compass, inertial navigation system, and airspeed calibration have been deliberately removed from the .param file. They are set using calibration procedures for a given sUAV as described in [ArduPlane documentation here](http://ardupilot.org/plane/docs/common-accelerometer-calibration.html), [here](http://ardupilot.org/plane/docs/common-compass-calibration-in-mission-planner.html), and [here](http://ardupilot.org/copter/docs/common-power-module-configuration-in-mission-planner.html) for a given airframe and battery choice. Please follow these procedures exactly.
  
  NOTE #2: Parameter SOAR_ALT_MAX, which determines the maximum soaring altitude, has been set to 120 meters for safety and compliance with US federal regulations, which limit sUAV operating altitudes to 400 feet above ground level in the absense of ground structures. If the your test site has structures, or if you are flying in a different country with different sUAV regulations, you may be able to fly higher, and set SOAR_ALT_MAX accordingly. Consult your territory's applicable regulations.

- File [Example_geofenced_waypoint_course.waypoints](https://github.com/Microsoft/Frigatebird/DATA/Example_geofenced_waypoint_course.waypoints) is an ArduPilot waypoints file that contains a sequence of waypoints the sUAV is supposed to follow when not thermalling, and a geofence around them determining the area where the sUAV is allowed to fly while thermalling. Starting with this file, you can carry over the waypoints to your test site and add new ones. A waypoint file can be uploaded to a flight controller running ArduPlane using ground control station software such as [Mission Planner](http://ardupilot.org/planner/).


## Developer Information ##

- Github repository: <https://github.com/Microsoft/Frigatebird>

- The main site (<http://ardupilot.org>), developer wiki (<http://dev.ardupilot.org>), and discussion page (<http://discuss.ardupilot.org>) of the parent project, ArduPilot, is useful for understanding how to modify Frigatebird.

## License ##

Project Frigatebird is licensed under the [GNU General Public
License, version 3](https://github.com/Microsoft/Frigatebird/blob/master/LICENSE)

## Maintainers ##

- [Iain Guilliard](https://github.com/iainguilliard), main developer of Frigatebird's POMDSoar thermalling controller
- [Andrey Kolobov](https://github.com/akolobov), project lead

## Acknowledgements ##

We would like to say a big "thank you!" to the developer community of ArduPilot. Their code and advice has greatly helped in starting Project Frigatebird.
