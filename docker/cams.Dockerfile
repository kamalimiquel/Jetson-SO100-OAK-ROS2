# cams service — depthai wheel in-container (ADR-0007 fallback, 2.x Pipeline API).
# Installed into the image's python3.8: cp38 aarch64 wheels stop at 2.30.0.0
# (2.31+ requires Python >=3.9), so the pin is 2.30 — same API as the 2.33
# source build for everything this node uses.
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# The dusty image ships a ROS apt repo whose key is expired (EXPKEYSIG) — any
# apt-get update fails until the repo file is dropped. We don't need ROS apt
# packages here (ROS is baked into /opt/ros), so remove it.
RUN rm -f /etc/apt/sources.list.d/*ros* \
    && apt-get update \
    && pip3 install --no-cache-dir "depthai==2.30.0.0" opencv-python-headless \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
