#!/bin/bash

default_x=0
default_y=0

x="${1:-$default_value1}"
y="${2:-$default_value2}"
catkin build
source devel/setup.bash
clear
roslaunch ./src/simple_planner/launch/simple_planner.launch x:=$x y:=$y
