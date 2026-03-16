# AirBots — Autonomous Gate Navigation | Xpecto 26 | IIT Mandi

---

## Overview

Autonomous drone simulation for the Xpecto 2026 ROS Hackathon. The X4 quadrotor detects and flies through red gates in a Gazebo arena using OpenCV computer vision and a PID state machine, running inside Docker with ROS 2 Jazzy and Gazebo Harmonic.

---

## Prerequisites

- Docker v24+
- GPU (recommended) or software rendering
- X11 display server
- Ubuntu 24.04+ or Linux Mint 22+
- Git

---

## Quick Start

Terminal 1 — Clone and start simulation
```
git clone https://github.com/karthikeya-kar/ROS_HACK.git
cd ROS_HACK
xhost +local:docker
sudo docker compose up sim_gui
```
Wait for Gazebo to fully load.

Terminal 2 — Launch the drone
```
sudo docker compose up auto
```

Terminal 3 — Start camera bridge
```
sudo docker exec ros_hack_sim_gui bash -c "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge '/world/aerial_nav_world/model/X4/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image' --ros-args --remap '/world/aerial_nav_world/model/X4/link/base_link/sensor/camera_front/image:=/X4/camera/image'"
```

The drone will automatically take off, find red gates, and fly through them.

---

## Project Structure

```
ROS_HACK/
├── Dockerfile                  # ROS 2 Jazzy + Gazebo Harmonic image
├── docker-compose.yml          # Services: sim_gui, sim_headless, auto
├── package.xml                 # ROS 2 package definition
├── setup.py                    # Python package setup
├── launch/
│   ├── sim.launch.py           # Starts Gazebo + ROS-Gazebo bridge
│   └── auto.launch.py          # Starts autonomous drone node
├── src/
│   └── autonomous_x4.py        # Main navigation logic
└── worlds/
    └── aerial_nav.world        # Arena with 10 red gates and X4 drone
```

---

## ROS 2 Topics

| Topic | Direction | Purpose |
|---|---|---|
| `/X4/camera/image` | Gazebo → ROS | Front camera feed |
| `/X4/imu` | Gazebo → ROS | Orientation and angular velocity |
| `/X4/baro` | Gazebo → ROS | Altitude estimate |
| `/X4/gazebo/command/twist` | ROS → Gazebo | Velocity commands |
| `/X4/enable` | ROS → Gazebo | Arm drone motors |

---

## Troubleshooting

- Drone doesn't move → run the camera bridge in Terminal 3
- Black camera feed → confirm Terminal 3 bridge is running
- Docker permission error → `sudo usermod -aG docker $USER && newgrp docker`
- No display → `xhost +local:docker`

---

## Team

AirBots — IIT Mandi | Xpecto 26 ROS Hackathon 2026
