# cams service — depthai wheel in-container (ADR-0007 fallback, API-identical to
# the source build). Installed into the image's python3.8 (a cp38 aarch64 wheel
# exists) so the camera node runs single-process with the image's rclpy.
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# The dusty image ships a ROS apt repo whose key is expired (EXPKEYSIG) — any
# apt-get update fails until the repo file is dropped. We don't need ROS apt
# packages here (ROS is baked into /opt/ros), so remove it.
RUN rm -f /etc/apt/sources.list.d/*ros* \
    && apt-get update \
    && pip3 install --no-cache-dir "depthai==2.33.0.0" opencv-python-headless \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
