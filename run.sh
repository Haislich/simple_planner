#!/bin/bash

catkin build
source devel/setup.bash
roslaunch ./src/simple_planner/launch/simple_planner.launch
