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
# :11 was started as `-screen 1`, i.e. a server with no screen 0: clients fell back to the
# default 1280x1024x24 (which is why the Graph tab is 1280x1024, not the 1280x800 above) and
# Qt could not size its window properly. Define screen 0 explicitly.
Xvfb :11 -screen 0 1280x1024x24 -ac &
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
# The terminal tab shows the XAVIER HOST, not this container (user request). ttyd stays here
# on :7681 but SSHes to the host's maju over loopback (we share its netns). The password is
# read from a secret mounted from the host (like shutdown-secret, 600, never in the repo);
# if it is missing the terminal falls back to this container's shell with a clear message.
ttyd -W -p 7681 bash -lc 'P="$(cat /run/secrets/terminal-secret 2>/dev/null)"; \
  if [ -n "$P" ]; then exec sshpass -p "$P" ssh -o StrictHostKeyChecking=no \
       -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 maju@127.0.0.1; \
  else echo "[terminal-secret no montado: caigo al shell de so100-web]"; exec bash; fi' &
python3 /shutdown_server.py &
# wristcam focus test (:8093): the HTTP end is always up, the subscriber+scorer only run
# while the tab is open (POST /start, /stop) — see docker/web/wrist_focus.py.
python3 /srv/web/wrist_focus.py --csv /srv/state/wristcam-focus.csv &
# board temperatures for the "Xavier NX Temp" tab (:8094). Thirteen sysfs reads a second:
# it costs nothing, so unlike the focus test it simply always runs.
python3 /srv/web/xavier_temp.py &
# both wrist cameras composed into one image for RViz (:no port). It is always up because
# starting and stopping it with RViz raced with SIGTERM and left a dead node behind; instead
# it composes only when something is subscribed, so a closed RViz costs exactly nothing.
python3 /srv/web/wrist_pair.py >> /tmp/wrist_pair.log 2>&1 &
python3 -m http.server 80 --directory /srv/web &

source /opt/ros/humble/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
# RViz is NOT started here: with software GL it eats two of the Xavier's six cores, so the
# tabs page asks for it on :8099/rviz/start when its tab is opened and stops it on leave.
# The X display :10 and its VNC front-end stay up, so starting it is a ~10 s window instead
# of a container restart.
command -v rqt_graph >/dev/null 2>&1 && {
  # rqt_gui rejects -geometry (it has its own argparse), so the window is stretched with
  # xdotool. It fits the graph once, while it is still empty and the window is still small,
  # and never re-fits by itself, so after the stretch the graph stays microscopic — wait for
  # the cell's nodes and toggle "Fit" off/on to re-run the fit at the real window size.
  DISPLAY=:11 ros2 run rqt_graph rqt_graph &
  (for i in $(seq 1 30); do
     id=$(DISPLAY=:11 xdotool search --class rqt_graph 2>/dev/null | head -1)
     if [ -n "$id" ] && DISPLAY=:11 xdotool windowsize "$id" 100% 100% \
        windowmove "$id" 0 0 2>/dev/null; then break; fi
     sleep 2
   done
   # `[ ... ] && break` would end this subshell on the first pass under `set -e`
   for i in $(seq 1 90); do
     if [ "$(ros2 node list 2>/dev/null | wc -l)" -ge 12 ]; then break; fi
     sleep 2
   done
   # rqt fits the graph before the cell's nodes exist and never re-fits by itself, so the tab
   # opened with a microscopic graph. A fixed delay does not help: the graph keeps growing while
   # the containers come up, and a fit taken mid-growth zooms in on a handful of nodes. Wait for
   # the node list to *stop changing* (and to have the whole cell in it), then toggle "Fit" off
   # and on so the second click re-runs it at the real window size.
   prev=""
   for i in $(seq 1 60); do
     now=$(ros2 node list 2>/dev/null | sort)
     if [ "$now" = "$prev" ] && [ "$(printf "%s" "$now" | grep -c .)" -ge 12 ]; then break; fi
     prev="$now"
     sleep 4
   done
   sleep 8
   # The tab opens in the view the cell is read in: "Nodes/Topics (all)" (the default
   # "Nodes only" hides every node whose topics have no partner, which is most of the arm
   # side) and no "Dead sinks" filter. rqt does not persist these, so they are clicked here.
   DISPLAY=:11 xdotool mousemove 100 40 click 1 >/dev/null 2>&1 || true
   sleep 1
   DISPLAY=:11 xdotool key Down >/dev/null 2>&1 || true
   DISPLAY=:11 xdotool key Down >/dev/null 2>&1 || true
   DISPLAY=:11 xdotool key Return >/dev/null 2>&1 || true
   sleep 2
   DISPLAY=:11 xdotool mousemove 52 103 click 1 >/dev/null 2>&1 || true
   sleep 2
   # rqt builds its graph once at startup and only patches it afterwards, so nodes missed in
   # that first pass (deg2rad, ik) never appear until the graph is re-queried — that is the
   # toolbar reload at 509,64.
   # The reload re-queries the graph and, with "Fit" still ticked (the default), that update is
   # what re-runs the fit at the full window size. It is repeated because rqt's discovery of the
   # quieter participants (arm_bridge, ik, deg2rad) takes tens of seconds: the first reloads find
   # only the camera nodes, a later one finds the whole cell.
   for round in 1 2 3 4; do
     DISPLAY=:11 xdotool mousemove 509 64 click 1 >/dev/null 2>&1 || true
     sleep 15
   done) &
}

wait
