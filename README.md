
# ROS2 Rover Simulation

A simple 4-wheeled rover simulation in Gazebo with ROS2 control integration.

## Packages

- **rover_description**: Contains URDF files and world definitions
- **rover_launch**: Launch files and controller configurations

## Start

### 1. Build the workspace
```bash
colcon build
source install/setup.bash
```

### 2. Launch the simulation
```bash
# Option A: Using ros2_control (recommended)
ros2 launch rover_launch gz_rover_with_controls.launch.py

# Option B: Using Gazebo built-in diff drive
ros2 launch rover_launch rover_gz.launch.py

# Option C: Visualization only (RViz)
ros2 launch rover_launch display_rover.launch.py
```

### 3. Control the rover
```bash
# For ros2_control approach

# Or use keyboard teleop
```

## Files Overview

### URDF Files
- `rover_control.urdf` - Rover with ros2_control integration
- `rover_diff.urdf` - Rover with Gazebo built-in diff drive plugin

### Launch Files
- `gz_rover_with_controls.launch.py` - Full simulation with ros2_control
- `rover_gz.launch.py` - Simulation with Gazebo diff drive plugin
- `display_rover.launch.py` - RViz visualization only

### Configuration
- `rover_ros2_control.yaml` - ros2_control controller configuration
- `wheel_direct.yaml` - Direct wheel control configuration
- `empty_local.sdf` - Simple Gazebo world

