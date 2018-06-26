# Project Frigatebird

Project Frigatebird is aimed at enabling small fixed-wing UAVs to travel long distances by soaring -- taking advantage of rising air regions the way human sailplane
pilots and bird species like frigatebirds do -- using AI planning and learning techniques.

Currenty, this repository contains an implementation of POMDSoar, an algorithm that allows a sailplane UAV to detect and gain altitude in thermals, "columns" of rising air that occur over certain
regions of Earth's surface. This approach, based on solving a Partially Observable Markov Decision Process, is described in

    I. Guilliard, R. Rogahn, J. Piavis, A. Kolobov. "Autonomous Thermalling as a Partially Observable Markov Decision Process." Robotics: Science and Systems conference, 2018. https://arxiv.org/pdf/1805.09875.pdf

We implementated POMDSoar as a soaring controller integrated into a fork of ArduPlane version 3.8.2, the fixed-wing flavor of ArduPilot open-source drone autopilot suite ([original ArduPlane code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](http://ardupilot.org/plane/index.html)). The code in the Frigatebird repository allows building this modified version of ArduPlane. Please see [BUILD.md](https://github.com/Microsoft/Frigatebird/BUILD.md) for building instructions.

The [DATA](https://github.com/Microsoft/Frigatebird/DATA) subdirectory contains ArduPlane parameter files and an example geofenced waypoint course for running POMDSoar on a Radian Pro sUAV as in the experiments in the above paper. 



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
