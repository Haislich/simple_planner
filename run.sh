#!/bin/bash

# Default values for the arguments
DEFAULT_X=0
DEFAULT_Y=0

# Initialize variables with default values
x=$DEFAULT_X
y=$DEFAULT_Y

# Parse named parameters
while [[ $# -gt 0 ]]; do
    case $1 in
        --x)
            x=$2
            shift 2
            ;;
        --y)
            y=$2
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done


catkin build
source devel/setup.bash
roslaunch ./src/simple_planner/launch/simple_planner.launch x:=$x y:=$y
