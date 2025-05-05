#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0);pwd)

pip install PyQt5

sudo cp $SCRIPT_DIR/img/rosbag_player.png /usr/share/pixmaps/
cat $SCRIPT_DIR/desktop/rosbag_player.desktop | sed -e "s?PATH?$SCRIPT_DIR?" > $HOME/.local/share/applications/rosbag_player.desktop

cd $HOME/Desktop
cp $HOME/.local/share/applications/rosbag_player.desktop .
gio set rosbag_player.desktop metadata::trusted true
sudo chmod a+x rosbag_player.desktop

cd $HOME/.config
if ! grep -q "rosbag_player.desktop" mimeapps.list; then
    ROW="application/octet-stream=rosbag_player.desktop"
    sed -i -e '/^$/i'$ROW';' mimeapps.list
    sed -i -e '$a'$ROW mimeapps.list
fi