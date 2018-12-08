# Building ArduPlane with POMDSoar

Please follow [ArduPilot's build instructions for your OS](http://ardupilot.org/dev/docs/building-the-code.html), replacing "ArduCopter" with "ArduPlane" and "https://github.com/ArduPilot/ardupilot.git" with "https://github.com/Microsoft/Frigatebird.git"

On Windows 10, we recommend [building using Waf on Cygwin64](http://ardupilot.org/dev/docs/building-setup-windows-cygwin.html) as opposed to using Make. We also strongly recommend building the firmware on top of ChibiOS rather than NuttX. After setting up your build environment as described at the above link, cloning Frigatebird, and running

        git submodule update --init --recursive

from your local Frigatebird repo's root directory, you can build the ChibiOS flavor of ArduPlane/POMDSoar firmware for, e.g., Pixhawk by running the following commands from the same location:

        ./waf distclean
        ./waf configure --board fmuv2
        ./waf plane

As another example, you can build a SITL flavor of it by running

        ./waf distclean
        ./waf configure --board sitl
        ./waf plane


Additional notes for building the firmware on Windows:

-  Before proceeeding with other instructions, make sure you have the most recent [Git for Windows](https://gitforwindows.org/) installed. Otherwise, you may see errors such as
    
        SSL routines:SSL23_GET_SERVER_HELLO:tlsv1 alert protocol version
    
that will prevent you from performing basic Git operations.

