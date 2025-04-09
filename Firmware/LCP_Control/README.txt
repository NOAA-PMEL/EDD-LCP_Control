Name:
=====
 lcpcontrol_lowpower


Description:
============
 Example of the app running under FreeRTOS.


This example implements LED task within the FreeRTOS framework. It monitors
three On-board buttons, and toggles respective on-board LEDs in response.
To save power, this application is compiled without print
statements by default. To enable them, add the following project-level
macro definitions.

AM_DEBUG_PRINTF

If enabled debug messages will be sent over UART.


******************************************************************************


State of the world (project):

Requirements:
Install git, cmake, ninja, and the arm-none-eabi gcc compiler. Ensure that everything is in your PATH

Get the SDK:
Clone the AmbiqSuiteSDK in the parent directory of this project:
cd ..
git clone --recursive https://github.com/sparkfun/AmbiqSuiteSDK.git

Build with the following commands in the ./Firmware/LCP_Control/ directory (windows and linux):
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="../arm-gcc-toolchain.cmake" -G "Ninja"
ninja bootload_asb (or whichever target is being used to flash)