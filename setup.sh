#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

sudo cp $SCRIPT_DIR/img/rosbag_player.png /usr/share/pixmaps/
cat $SCRIPT_DIR/desktop/rosbag_player.desktop | sed -e "s?PATH?$SCRIPT_DIR?" > ~/.local/share/applications/rosbag_player.desktop

ROW="application/vnd.sqlite3=rosbag_player.desktop"
sed -i -e '/^$/i'$ROW';' $HOME/.config/mimeapps.list
sed -i -e '$a'$ROW $HOME/.config/mimeapps.list
