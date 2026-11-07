# cams service — depthai SDK in-container (ADR-0007 fallback wheel, API-identical
# to the source build), on a uv-managed py3.12 venv (the image's python3.8 is too old).
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# The dusty image ships a ROS apt repo whose key is expired (EXPKEYSIG) — any
# apt-get update fails until the repo file is dropped. We don't need ROS apt
# packages here (ROS is baked into /opt/ros), so remove it.
RUN rm -f /etc/apt/sources.list.d/*ros* \
    && apt-get update \
    && apt-get install -y --no-install-recommends curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

RUN uv venv /opt/oak/.venv --python 3.12 \
    && uv pip install --python /opt/oak/.venv/bin/python "depthai<3" opencv-python-headless

# Probe entrypoint (USB access comes from compose: /dev/bus/usb + cgroup rule).
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/opt/oak/.venv/bin/python /opt/ws/tools/oak_cams_probe.py"]
