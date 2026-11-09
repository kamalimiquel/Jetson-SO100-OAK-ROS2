# web service — literal RViz2 (ADR-0014/0018), headless via Xvfb + noVNC (:6080),
# rqt_graph (:6081), ttyd terminal (:7681), tabs page (:80).
#
# Base = the dusty DESKTOP image: it ships rviz2 and rqt_graph prebuilt. That
# removes the whole builder stages the focal "humble-ros-base" image needed
# (no humble debs for focal → a multi-hour rviz2 source build; see git history).
FROM dustynv/ros:humble-desktop-l4t-r35.4.1

RUN rm -f /etc/apt/sources.list.d/*ros* 2>/dev/null; \
    apt-get update \
    && apt-get install -y --no-install-recommends \
        curl xvfb x11vnc x11-utils xauth websockify novnc \
        sshpass openssh-client \
        python3-pyqt5 python3-pyqtgraph libignition-math6-dev \
        libgl1 libgl1-mesa-dri libzzip-0-13 libcppunit-1.15-0 \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

COPY docker/web/entry.sh /entry.sh
COPY docker/web/shutdown_server.py /shutdown_server.py
RUN chmod +x /entry.sh

CMD ["/entry.sh"]
