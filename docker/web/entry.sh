#!/bin/bash
# Headless GUI services (ADR-0014/0018): RViz2 on display :10 (noVNC :6080),
# rqt_graph on display :11 (noVNC :6081), ttyd terminal :7681 (WRITABLE!),
# tabs page :80.
#
# Displays start at :10 on purpose: the Xavier's own desktop runs Xorg on :0,
# and with --network host the X11 abstract sockets are shared, so a container
# Xvfb on :0/:1 dies with "server already running".
set -e

Xvfb :10 -screen 0 1280x800x24 -ac &
Xvfb :11 -screen 1 1280x800x24 -ac &
sleep 2
x11vnc -display :10 -forever -shared -nopw -quiet -rfbport 5900 &
x11vnc -display :11 -forever -shared -nopw -quiet -rfbport 5901 &
websockify --web /usr/share/novnc 6080 localhost:5900 &
websockify --web /usr/share/novnc 6081 localhost:5901 &
ttyd -W -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash && [ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash && exec bash' &
python3 /shutdown_server.py &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
[ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash
[ -f /opt/rqt_ws/setup.bash ] && source /opt/rqt_ws/setup.bash
DISPLAY=:10 ros2 run rviz2 rviz2 -d /srv/web/rviz.rviz &
command -v rqt_graph >/dev/null 2>&1 && DISPLAY=:11 ros2 run rqt_graph rqt_graph &

wait
