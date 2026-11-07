# web service — browser tabs (ADR-0014/0018, amended 2026-11-07):
#   tab "3D"       -> Foxglove Studio (browser) over foxglove_bridge ws://:8765
#   tab "Graph"    -> same, Foxglove's Rosgraph panel
#   tab "Terminal" -> ttyd :7681 (shell inside the ROS2 environment)
#   tabs page      -> :80
# Literal RViz2 is infeasible on focal (ROS2 ships no humble debs for Ubuntu
# 20.04 — the apt repo only carries foxy) — user chose the Foxglove route.

# ---- build stage: compile foxglove_bridge against the image's source-built ROS
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1 AS builder
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git python3-pip libwebsocketpp-dev libasio-dev libzstd-dev nlohmann-json3-dev libcurl4-openssl-dev \
    && pip3 install colcon-common-extensions \
    && mkdir -p /opt/src && cd /opt/src \
    && git clone --depth 1 --branch 0.8.2 https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge \
    && git clone --depth 1 --branch humble https://github.com/ros/resource_retriever.git resource_retriever \
    && git clone --depth 1 --branch humble https://github.com/ros/curl_vendor.git libcurl_vendor \
    && . /opt/ros/humble/install/setup.bash \
    && colcon build --packages-select libcurl_vendor resource_retriever foxglove_bridge --install-base /opt/foxglove_ws \
    && rm -rf /opt/src /var/lib/apt/lists/*

# ---- runtime stage
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends curl \
    && curl -L -o /usr/local/bin/ttyd https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64 \
    && chmod +x /usr/local/bin/ttyd \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/foxglove_ws /opt/foxglove_ws
COPY web/entry.sh /entry.sh
RUN chmod +x /entry.sh

CMD ["/entry.sh"]
