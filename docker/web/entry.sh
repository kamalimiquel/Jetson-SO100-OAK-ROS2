#!/bin/bash
# Browser interface: foxglove_bridge (WebSocket :8765), ttyd terminal (:7681),
# tabs page (:80). All rendering happens client-side in the browser.
set -e

source /opt/ros/humble/install/setup.bash
[ -f /opt/foxglove_ws/setup.bash ] && source /opt/foxglove_ws/setup.bash

ros2 run foxglove_bridge foxglove_bridge &
ttyd -p 7681 bash -lc 'source /opt/ros/humble/install/setup.bash && [ -f /opt/foxglove_ws/setup.bash ] && source /opt/foxglove_ws/setup.bash && exec bash' &
python3 -m http.server 80 --directory /srv/web &

wait
