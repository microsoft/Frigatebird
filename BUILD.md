# Building ArduPlane with POMDSoar#

Please follow [ArduPilot's build instructions for your OS](http://ardupilot.org/dev/docs/building-the-code.html), replacing "ArduCopter" with "ArduPlane" and "https://github.com/ArduPilot/ardupilot.git" with "https://github.com/Microsoft/Frigatebird.git"

__Important__: After initialzing the Git submodules, copy the file Frigatebird/ardupilotmega.xml, in the root of the Frigatebird repository, over the one in the MAVLink submodule: mavlink/message_definitions/v1.0/ardupilotmega.xml.

For Windows 10, at this time we recommend [building using Make](http://ardupilot.org/dev/docs/building-px4-with-make.html#building-px4-with-make), as opposed to [building using Waf](http://ardupilot.org/dev/docs/building-setup-windows-cygwin.html), since the latter does not seem to be fully worked out for Windows yet.

Additional notes for building on Windows:

-  Before proceeeding with other instructions, make sure you have the most [Git for Windows](https://gitforwindows.org/) installed. Otherwise, you may see errors such as
    
        SSL routines:SSL23_GET_SERVER_HELLO:tlsv1 alert protocol version
    
   that will prevent you from performing basic Git operations.
   
-  If when building from [PX4Console](http://ardupilot.org/dev/docs/building-px4-with-make.html#building-px4-with-make) you encounter issues with creating directories, close the PX4Console window and restart it as Administrator.
