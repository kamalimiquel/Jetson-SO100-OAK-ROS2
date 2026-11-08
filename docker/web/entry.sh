#!/bin/bash
# Headless GUI services (ADR-0014/0018): RViz2 on display :0 (noVNC :6080),
# rqt_graph on display :1 (noVNC :6081), ttyd terminal :7681 (WRITABLE!),
# tabs page :80.
set -e

Xvfb :0 -screen 0 1280x800x24 -ac &
Xvfb :1 -screen 1 1280x800x24 -ac &
sleep 1
x11vnc -display :0 -forever -shared -nopw -quiet &
x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5901 &
websockify --web /usr/share/novnc 6080 localhost:5900 &
websockify --web /usr/share/novnc 6081 localhost:5901 &
ttyd -W -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash && [ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash && exec bash' &
python3 /shutdown_server.py &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
[ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash
[ -f /opt/rqt_ws/setup.bash ] && source /opt/rqt_ws/setup.bash
[ -f /opt/rosbridge_ws/setup.bash ] && source /opt/rosbridge_ws/setup.bash
DISPLAY=:0 ros2 run rviz2 rviz2 -d /srv/web/rviz.rviz &
DISPLAY=:1 ros2 run rqt_graph rqt_graph &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

wait
