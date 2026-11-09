#!/bin/bash
# Headless GUI services (ADR-0014/0018): RViz2 on display :10 (noVNC :6080),
# rqt_graph on display :11 (noVNC :6081), ttyd terminal :7681 (WRITABLE!),
# tabs page :80.
#
# Displays start at :10 on purpose: the Xavier's own desktop runs Xorg on :0,
# and with --network host the X11 abstract sockets are shared, so a container
# Xvfb on :0/:1 dies with "server already running".
set -e

# A container restart can leave stale X locks behind (the previous Xvfb was
# killed without cleaning up) — Xvfb then refuses with "server already running".
rm -f /tmp/.X10-lock /tmp/.X11-lock /tmp/.X11-unix/X10 /tmp/.X11-unix/X11 2>/dev/null || true

Xvfb :10 -screen 0 1280x800x24 -ac &
Xvfb :11 -screen 1 1280x800x24 -ac &
# Wait for X to accept connections instead of a fixed sleep (advisorily from the
# reference patch: xdpyinfo returns only once the server is ready).
for d in :10 :11; do
  for i in $(seq 1 40); do xdpyinfo -display $d >/dev/null 2>&1 && break; sleep 0.25; done
done
x11vnc -display :10 -forever -shared -nopw -quiet -rfbport 5900 &
x11vnc -display :11 -forever -shared -nopw -quiet -rfbport 5901 &
websockify --web /usr/share/novnc 6080 localhost:5900 &
websockify --web /usr/share/novnc 6081 localhost:5901 &
ttyd -W -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash && [ -f /opt/rviz_ws/setup.bash ] && source /opt/rviz_ws/setup.bash && exec bash' &
python3 /shutdown_server.py &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
DISPLAY=:10 ros2 run rviz2 rviz2 -d /srv/web/rviz.rviz &
command -v rqt_graph >/dev/null 2>&1 && DISPLAY=:11 ros2 run rqt_graph rqt_graph &

wait
