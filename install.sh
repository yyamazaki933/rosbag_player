#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

pip install PyYAML flet

sudo cp $SCRIPT_DIR/img/rosbag2_player.png /usr/share/pixmaps/
cat $SCRIPT_DIR/desktop/rosbag2_player.desktop | sed -e "s?PATH?$SCRIPT_DIR?" > ~/.local/share/applications/rosbag2_player.desktop

if ! grep -q "rosbag2_player.desktop" $HOME/.config/mimeapps.list; then
    ROW="application/vnd.sqlite3=rosbag2_player.desktop"
    sed -i -e '/^$/i'$ROW';' $HOME/.config/mimeapps.list
    sed -i -e '$a'$ROW $HOME/.config/mimeapps.list
fi