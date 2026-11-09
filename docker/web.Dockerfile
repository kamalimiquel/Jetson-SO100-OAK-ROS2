# web service — literal RViz2 (ADR-0014/0018, restored 2026-11-07 by user decision):
#   RViz2 built FROM SOURCE in the builder stage (no humble debs exist for
#   focal), shown headless via Xvfb + noVNC at :6080; ttyd terminal :7681;
#   tabs page :80. Foxglove route removed.

# ---- builder: rviz2 + its source-only deps against the image's source-built ROS.
# If docker/prebuilt/rviz_ws.tar.gz exists in the build context (produced on the
# Jetson by the rviz-builder container — see README), it is extracted instead
# of recompiling ~2.5h; the from-source recipe below stays as the fallback.
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
    && pip3 install colcon-common-extensions
COPY docker/prebuilt /opt/prebuilt
RUN if [ -s /opt/prebuilt/rviz_ws.tar.gz ]; then \
        echo "== using prebuilt rviz_ws ==" && tar xzf /opt/prebuilt/rviz_ws.tar.gz -C /; \
    else \
        echo "== building rviz2 from source ==" \
        && mkdir -p /opt/src && cd /opt/src \
        && git clone --depth 1 --branch humble https://github.com/ros2/rviz.git rviz \
        && git clone --depth 1 --branch humble https://github.com/ros-visualization/tango_icons_vendor.git tango_icons_vendor \
        && git clone --depth 1 --branch humble https://github.com/ros-visualization/qt_gui_core.git qt_gui_core \
        && git clone --depth 1 --branch humble https://github.com/ros-visualization/interactive_markers.git interactive_markers \
        && git clone --depth 1 --branch humble https://github.com/ros-perception/laser_geometry.git laser_geometry \
        && git clone --depth 1 --branch humble https://github.com/ros/resource_retriever.git resource_retriever \
        && . /opt/ros/humble/install/setup.bash \
        && colcon build --packages-up-to rviz2 --install-base /opt/rviz_ws \
        && rm -rf /opt/src; \
    fi && rm -rf /var/lib/apt/lists/*

# ---- builder2: rqt_graph stack (pure python — builds in minutes)
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1 AS rqtbuilder
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git python3-pip python3-pyqt5 python3-pyqtgraph python3-catkin-pkg \
    && pip3 install colcon-common-extensions \
    && mkdir -p /opt/src && cd /opt/src \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/qt_gui_core.git qt_gui_core \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/rqt.git rqt \
    && git clone --depth 1 --branch humble https://github.com/ros-visualization/rqt_graph.git rqt_graph \
    && touch qt_gui_core/qt_gui_cpp/COLCON_IGNORE \
    && . /opt/ros/humble/install/setup.bash \
    && (colcon build --packages-select qt_gui qt_gui_py_common rqt_py_common rqt_gui rqt_gui_py rqt_graph --install-base /opt/rqt_ws || echo "rqt partial — Graph tab degraded") \
    && rm -rf /opt/src /var/lib/apt/lists/*

# ---- runtime: Xvfb + noVNC + ttyd + the Qt/OGRE runtime libs rviz2 needs
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        curl xvfb x11vnc websockify novnc sshpass openssh-client \
        libqt5core5a libqt5gui5 libqt5widgets5 libqt5svg5 libqt5x11extras5 \
        python3-pyqt5 python3-pyqtgraph python3-catkin-pkg \
        python3-pip \
        libgl1 libgl1-mesa-dri libzzip-0-13 libcppunit-1.15-0 \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/rviz_ws /opt/rviz_ws
COPY --from=rqtbuilder /opt/rqt_ws /opt/rqt_ws
COPY docker/web/entry.sh /entry.sh
COPY docker/web/shutdown_server.py /shutdown_server.py
RUN chmod +x /entry.sh

CMD ["/entry.sh"]
