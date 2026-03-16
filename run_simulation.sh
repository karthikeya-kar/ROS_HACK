#!/bin/bash

# ==============================================================================
# Drone Simulation Launcher — Unified Execution Script (No GPU Required)
# ==============================================================================
# This script automatically builds the docker images, launches the simulation
# environment in the background, waits for it to initialize, and then runs
# the autonomous drone logic.
# ==============================================================================

# 1. Clean up any existing containers
echo "🧹 Cleaning up old containers..."
sudo docker compose down

# 2. Allow Docker to attach to the local X11 server for GUI rendering
echo "🖥️  Configuring X11 forwarding..."
xhost +local:docker 2>/dev/null

# 3. Build and Start the Simulation Environment (Gazebo) in the background
echo "🏗️  Building and starting Simulation Environment (sim_gui)..."
sudo docker compose build sim_gui
sudo docker compose up -d sim_gui

# 4. Wait for Gazebo and ROS bridges to fully initialize before starting the drone
echo "⏳ Waiting 10 seconds for Gazebo to load..."
sleep 10

# 5. Build and Start the Autonomous Drone Control node
echo "🚁 Starting Autonomous Drone Node..."
sudo docker compose build auto
sudo docker compose up auto

echo "✅ Execution finished."
