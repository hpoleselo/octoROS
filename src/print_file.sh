#!/bin/bash
if [[ "$1" = *.gcode* ]] 
then 
    roslaunch octo_ros connect_to_printer.launch arg1:="$1"
else
    echo "[INFO]: Provide the file name (.gcode format) you want to print as argument when you execute this program."
fi
