
# RobotControlNode using ROS2

## Overview

The Node is designed to control a robot's velocity based on user input and ensure the robot stays within a predefined boundary.
It subscribes to the robot's odometry topic (`/odom`) to monitor its position and publishes velocity commands to the `/cmd_vel` topic leveraging ROS 2's subscription and publication mechanisms.
If the robot moves out of the defined boundaries, the node will stop the robot and provide a warning.

## Features

- **Real-Time Control**: The user can input linear and angular velocities, which the node publishes as velocity commands (`geometry_msgs/msg/Twist`).
- **Boundary Check**: The robot's position is continuously monitored, and if the robot moves outside the specified boundaries, the robot is stopped.
- **Multithreading**: The node uses a separate thread to accept user input while the main loop manages the robot's movements.

## Node Details

- **Name**: `robot_control_node`
- **Topics**:
  - **Subscription**:
    - `/odom`: `nav_msgs/msg/Odometry` – Receives the robot's odometry data, including position and orientation.
  - **Publication**:
    - `/cmd_vel`: `geometry_msgs/msg/Twist` – Publishes velocity commands to control the robot.

## Boundary Conditions

The robot operates within a rectangular boundary, defined by the following limits:
- `x_min_`: 1.0 meters
- `x_max_`: 9.0 meters
- `y_min_`: 1.0 meters
- `y_max_`: 9.0 meters

If the robot's position exceeds any of these limits, it will stop moving.

## Example Output

- Upon receiving input, the node will continuously publish velocity commands to the `/cmd_vel` topic and output this message:
  ```
  Commands will be published until boundary conditions are met.
  ```

- If the robot exceeds the boundary, the following warning message will be displayed:
  ```
  Robot out of bounds! Stopping.
  ```

## Node Components

### 1. `odomCallback`:
   - This function subscribes to the `/odom` topic and updates the robot's position.
   - It checks whether the robot is within the defined boundaries.

### 2. `publishCommand`:
   - This function publishes the velocity commands to the `/cmd_vel` topic.

### 3. `stopRobot`:
   - This function stops the robot by publishing a zero-velocity command when the robot is out of bounds.

### 4. `getUserInput`:
   - This function runs in a separate thread to gather user input for linear and angular velocities while the node is running.

## Dependencies
- `robot_urdf` package
