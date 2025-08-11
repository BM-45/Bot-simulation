# Rover Description & Launch (ROS 2 Humble)

This repository contains:
- **`rover_description`** — URDF model of a basic 4-wheeled rover.
- **`rover_launch`** — Launch file to visualize the rover in RViz2 using `robot_state_publisher` and `joint_state_publisher_gui`.

---

## 📦 Requirements

ROS 2 Humble with the following packages installed:

```bash
sudo apt update
sudo apt install \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2
