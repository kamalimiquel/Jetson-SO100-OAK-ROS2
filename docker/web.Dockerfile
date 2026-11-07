# web service — 3-tab interface (ADR-0018): RViz stream (noVNC), rqt_graph (noVNC),
# terminal (ttyd). RViz2/rqt_graph wiring lands with the control phase; this image
# carries the servers + the tabs page.
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

RUN rm -f /etc/apt/sources.list.d/*ros* \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        curl xvfb x11vnc websockify novnc \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
