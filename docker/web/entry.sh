#!/bin/bash
# RViz2 headless over noVNC (ADR-0014/0018): display :0 -> noVNC at :6080,
# ttyd terminal :7681, tabs page :80. RViz2 comes from /opt/rviz_ws (built
# from source in the image's builder stage).
set -e

Xvfb :0 -screen 0 1280x800x24 -ac &
sleep 1
x11vnc -display :0 -forever -shared -nopw -quiet &
websockify --web /usr/share/novnc 6080 localhost:5900 &
ttyd -W -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash && [ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash && exec bash' &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
[ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash
DISPLAY=:0 ros2 run rviz2 rviz2 -d /srv/web/rviz.rviz &

wait
