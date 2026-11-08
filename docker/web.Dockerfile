# web service — literal RViz2 (ADR-0014/0018, restored 2026-11-07 by user decision):
#   RViz2 built FROM SOURCE in the builder stage (no humble debs exist for
#   focal), shown headless via Xvfb + noVNC at :6080; ttyd terminal :7681;
#   tabs page :80. Foxglove route removed.

# ---- builder: rviz2 + its source-only deps against the image's source-built ROS
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1 AS builder
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential git python3-pip ca-certificates \
        qtbase5-dev qtbase5-dev-tools libqt5svg5-dev \
        libx11-dev libxt-dev libxaw7-dev libxrandr-dev libx11-xcb-dev \
        libgl1-mesa-dev libglu1-mesa-dev \
        libfreetype6-dev libzzip-dev libpng-dev libjpeg-dev libtiff-dev \
        libcppunit-dev doxygen graphviz zlib1g-dev pkg-config \
    && pip3 install colcon-common-extensions \
    && mkdir -p /opt/src && cd /opt/src \
    && git clone --depth 1 --branch humble https://github.com/ros2/rviz.git rviz \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/tango_icons_vendor.git tango_icons_vendor \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/qt_gui_core.git qt_gui_core \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/interactive_markers.git interactive_markers \
    && git clone --depth 1 --branch humble https://github.com/ros-perception/laser_geometry.git laser_geometry \
    && git clone --depth 1 --branch humble https://github.com/ros/resource_retriever.git resource_retriever \
    && . /opt/ros/humble/install/setup.bash \
    && colcon build --packages-up-to rviz2 --install-base /opt/rviz_ws \
    && rm -rf /opt/src /var/lib/apt/lists/*

# ---- runtime: Xvfb + noVNC + ttyd + the Qt/OGRE runtime libs rviz2 needs
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        curl xvfb x11vnc websockify novnc \
        libqt5core5a libqt5gui5 libqt5widgets5 libqt5svg5 libqt5x11extras5 \
        libgl1 libgl1-mesa-dri libzzip-0-13 libcppunit-1.15-0 \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/rviz_ws /opt/rviz_ws
COPY docker/web/entry.sh /entry.sh
RUN chmod +x /entry.sh

CMD ["/entry.sh"]
