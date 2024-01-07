# MECAPI ESP32 for LIDAR and MetaSenseA010

Get data from a LD06 LIDAR and analyze / compute points to get resulting adversary robots positions
Robots positions are then send to PIC via serial.
This code is destined to be executed on a ESP32.

If you want to plot data we advise to go on https://teleplot.fr/ instead of using vscode extension as it is bugged

# How to clone with submodules letest versions

``git submodule update --init --recursive``  
``git submodule foreach --recursive git fetch``   
``git submodule foreach git checkout  main``
