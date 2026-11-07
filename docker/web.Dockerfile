# web service — 3-tab interface (ADR-0018): RViz2 (noVNC), rqt_graph (noVNC),
# terminal (ttyd inside the ROS2 environment), plus the tabs page on port 80.
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# Renew the expired ROS apt key (EXPKEYSIG), then install the GUI stack.
# The image's ROS is source-built in /opt/ros/humble/install; the apt packages
# (rviz2, rqt_graph) land in the standard /opt/ros/humble layout — both are
# humble on focal, so they coexist; source the apt setup first, then install/.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        curl xvfb x11vnc websockify novnc mesa-utils libgl1-mesa-dri \
        ros-humble-rviz2 ros-humble-rqt-graph \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

COPY web/entry.sh /entry.sh
RUN chmod +x /entry.sh

CMD ["/entry.sh"]
