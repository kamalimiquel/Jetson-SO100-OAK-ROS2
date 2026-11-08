# rosbridge standalone service (MuJoCo twin transport, WebSocket :9090).
# rosbridge_suite humble is Python >=3.9 — it cannot run on the focal/py3.8
# containers, so it gets its own jammy (Ubuntu 22.04) image with official
# binary packages. DDS (domain 42) reaches the focal containers over the LAN.
FROM arm64v8/ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
