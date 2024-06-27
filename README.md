# Robotic Arm Project

## Overview

This project is a robotic arm designed to simulate movements using Gazebo and RViz. The arm utilizes inverse kinematics to accurately reach specific points in 3D space. This repository includes all necessary files and instructions to set up, simulate, and control the robotic arm.

## Features

- **Simulation**: Utilize Gazebo for realistic physics simulation.
- **Visualization**: Use RViz to visualize the robotic arm's movements.
- **Control**: Implement inverse kinematics for precise control of the arm's end-effector in 3D space.

## Dependencies

- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation)
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
- [RViz](http://wiki.ros.org/rviz)

## Installation

1. **Clone the repository:**
    ```bash
    git clone https://github.com/yourusername/robotic-arm-project.git
    cd robotic-arm-project
    ```

2. **Install dependencies:**
    Ensure you have ROS, Gazebo, and RViz installed. You can follow the official installation guides for each.

3. **Build the project:**
    ```bash
    catkin_make
    source devel/setup.bash
    ```

## Usage

1. **Launch the Gazebo simulation:**
    ```bash
    roslaunch robotic_arm_project gazebo_simulation.launch
    ```

2. **Visualize in RViz:**
    ```bash
    roslaunch robotic_arm_project rviz_visualization.launch
    ```

3. **Control the robotic arm using the Python script:**
    ```bash
    rosrun robotic_arm_project move_arm.py
    ```

## Inverse Kinematics

The `move_arm.py` script is responsible for moving the robotic arm to a specific point in 3D using inverse kinematics. Make sure to customize the target coordinates in the script as per your requirement.

```python
# move_arm.py

# Example target coordinates
target_x = 0.5
target_y = 0.2
target_z = 0.3

# Call the function to move the arm
move_to_point(target_x, target_y, target_z)
