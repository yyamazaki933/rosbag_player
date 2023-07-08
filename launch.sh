#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1

python3 $SCRIPT_DIR/player.py $1