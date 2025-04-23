# Trajectory Planner
ROS2 path and trajectory planner repository(Team Elite).

## Main Contributor
Sachin ([@ann2716s](https://git.hs-coburg.de/ann2716s))

## Component Description

The TrajectoryPlanner component is responsible for autonomously controlling a vehicle along a predefined path using Ackermann steering. It operates in real-time to process sensor inputs, evaluate the driving environment, and issue drive commands accordingly.

---

<div align="center">
    <img src="Images/tp_image.jpg" height=500, width=800>
</div>

---

## Table of Contents
- [Nodes](#nodes)
- [RQT_graph](#rqt_graph)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)
- [License](#license)


## Nodes
### Node: `path and trajectory Planner`
#### Topics

### 🧾 TrajectoryPlanner – Topic Interface Table

| **Topic**                   | **Input/Output** | **Message Type**               | *Description**                                                  |
|----------------------------|---------|---------------------------------|------------------------------------------------------------------|
| `/ego_pose`                | Input   | `geometry_msgs/PoseStamped`     | Current position and orientation of the vehicle.                 |
| `/ego_twist`               | Input   | `geometry_msgs/TwistStamped`    | Current velocity (linear & angular) of the vehicle.              |
| `/path_data`               | Input   | `nav_msgs/Path`                 | Path that the vehicle is expected to follow.                     |
| `/obstacle_detected`       | Input   | `std_msgs/Bool`                 | Signals if an obstacle is detected in the path.                  |
| `/vehicle_state`           | Input   | `std_msgs/String`               | Vehicle state: "Idle","Driving","Boarding","Drop-Off"            |
| `/ackermann_drive_feedback`| Input   | `ackermann_msgs/AckermannDrive` | Feedback of actual speed and steering.                           |
| `/ackermann_drive`         | Output  | `ackermann_msgs/AckermannDrive` | Publishes target speed and steering to control the vehicle.      |

## RQT_graph
---
<div align="center">
    <img src="Images/rqt_pic.png" height=500, width=800>
</div>
---

## Functionality

This ROS 2 node (`TrajectoryPlanner`) manages autonomous trajectory execution for an Ackermann-steered vehicle. It combines path-following, vehicle_state, speed control, and obstacle handling with real-time vehicle state feedback to achieve safe and structured navigation.

---

## Features

### Vehicle State Management
- Supports three vehicle states: `"Idle"`, `"Boarding"`, and `"Driving"`.
- Vehicle halts if state is `"Idle"` or `"Boarding"`.

### Path Tracking
- Subscribes to planned paths via the `/path_data` topic.
- Determines when the vehicle reaches its goal based on a positional threshold.

### Speed Control (Ramp-Up Logic)
- Gradually increases speed over **5 seconds** to a maximum of **3.0 m/s**.
- Publishes drive commands with a fixed **steering angle of 0.0** (straight path).
- Motion continues unless:
  - The vehicle reaches the goal zone.
  - An obstacle is detected.
  - The state is not `"Driving"`.

### Obstacle Detection
- Listens to the `/obstacle_detected` topic.
- Immediately stops the vehicle when an obstacle is reported.

### Feedback Integration
- Subscribes to `/ackermann_drive_feedback` to track real-time:
  - Speed
  - Steering angle
- Logs target vs. actual values and their errors for debugging purposes.

### Student Location Broadcasting
- Once the vehicle enters the goal zone:
  - **X ∈ [3.0, 4.5]**
  - **Y ∈ [0.0, 1.8]**
- Publishes a one-time message: `"First Student"` to the `/student_location` topic.


## Configuration

### Velocity Settings
- `min_speed = 1.0` m/s
- `max_speed = 4.0` m/s
- `ramp_up_time = 5.0` seconds





## Installation
1. Clone the repository:
```bash
 git clone https://git.hs-coburg.de/Team_ELITE/EL_Path_Planner.git
```
2. Build the package:
```bash
 colcon build --packages-select tp_package
```
3. Source the workspace:
```bash
 source install/setup.bash
```

## Usage
### Launching the Nodes
To launch all of the nodes in lateral control package, run the following command:

```bash
ros2 run tp_package tp_planner 
```

## Testing
### Unit Tests
To run the unit tests for this package, use the following command:

```bash
colcon test --packages-select tj_planner
```

## License

This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.