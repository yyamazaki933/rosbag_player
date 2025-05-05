#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

screen -S rosbag_player -ADm bash -l -c "python3 $SCRIPT_DIR/player.py $1"
