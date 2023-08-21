#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds.xml

python3 $SCRIPT_DIR/player.py $1