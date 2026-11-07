#!/bin/bash
# 3-tab web interface (ADR-0014/0018):
#   display :0  -> RViz2        -> noVNC at :6080 (vnc.html?autoconnect=true)
#   display :1  -> rqt_graph    -> noVNC at :6081
#   ttyd at :7681 = terminal inside the ROS2 environment
#   tabs page at :80 (served from /srv/web)
set -e

export DISPLAY_NUM=0

Xvfb :0 -screen 0 1280x800x24 -ac &
Xvfb :1 -screen 1 1280x800x24 -ac &
sleep 1

x11vnc -display :0 -forever -shared -nopw -quiet &
x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5901 &

websockify --web /usr/share/novnc 6080 localhost:5900 &
websockify --web /usr/share/novnc 6081 localhost:5901 &

ttyd -p 7681 bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros/humble/install/setup.bash && exec bash' &

python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/setup.bash
source /opt/ros/humble/install/setup.bash

DISPLAY=:0 ros2 run rviz2 rviz2 &
DISPLAY=:1 ros2 run rqt_graph rqt_graph &

wait
