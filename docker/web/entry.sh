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
# The two VNC ports are 5910/5911 on purpose: 5900 is taken on the host by
# vino-server (GNOME's screen sharing), and with --network host our x11vnc
# cannot bind it — websockify would then happily proxy the HOST's desktop,
# which is exactly the "VNC shows an Ubuntu login" symptom after a reboot.
x11vnc -display :10 -forever -shared -nopw -quiet -rfbport 5910 &
x11vnc -display :11 -forever -shared -nopw -quiet -rfbport 5911 &
websockify --web /usr/share/novnc 6080 localhost:5910 &
websockify --web /usr/share/novnc 6081 localhost:5911 &
# The ROS env is a nicety here: if the source fails the shell must still open
# (a failing `&&` chain exits bash and ttyd shows "press to reconnect").
ttyd -W -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash 2>/dev/null; exec bash' &
python3 /shutdown_server.py &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
DISPLAY=:10 ros2 run rviz2 rviz2 -d /srv/web/rviz.rviz &
command -v rqt_graph >/dev/null 2>&1 && {
  DISPLAY=:11 ros2 run rqt_graph rqt_graph &
  # rqt opens in a small window; wait for it, then stretch it to the display.
  (for i in $(seq 1 30); do
     id=$(DISPLAY=:11 xdotool search --class rqt_graph 2>/dev/null | head -1)
     if [ -n "$id" ]; then
       DISPLAY=:11 xdotool windowsize "$id" 100% 100% windowmove "$id" 0 0 2>/dev/null && break
     fi
     sleep 2
   done) &
}

wait
