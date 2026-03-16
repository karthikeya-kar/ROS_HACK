# ─────────────────────────────────────────────────────────────
#  ROS_HACK Drone Simulation — Dockerfile
#  Base: ROS 2 Jazzy + Gazebo Harmonic (gz-harmonic)
# ─────────────────────────────────────────────────────────────
FROM ros:jazzy

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# ── Install dependencies ────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS ↔ Gazebo integration (installs gz-harmonic transitively)
    ros-jazzy-ros-gz \
    # Image conversion (ROS Image ↔ OpenCV)
    ros-jazzy-cv-bridge \
    # Python OpenCV + NumPy
    python3-opencv \
    python3-numpy \
    # Useful debugging tools
    ros-jazzy-rqt-image-view \
    ros-jazzy-tf2-tools \
    # Build tools
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ── Create workspace and copy source ────────────────────────
WORKDIR /ros2_ws/src
COPY . /ros2_ws/src/ros_hack/

# ── Build ───────────────────────────────────────────────────
WORKDIR /ros2_ws
RUN /bin/bash -c '\
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select ros_hack --symlink-install \
'

# ── Entrypoint ──────────────────────────────────────────────
COPY <<'ENTRYPOINT_SCRIPT' /ros_entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
ENTRYPOINT_SCRIPT
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "ros_hack", "drone.launch.py"]
