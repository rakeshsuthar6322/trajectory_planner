# Path and Trajectory Planner
ROS2 path and trajectory planner repository(Team Elite).

## Main Contributor
Sachin ([@ann2716s](https://git.hs-coburg.de/ann2716s))

## Component Description

The Path and Trajectory Planner is a core component of an Autonomous Pupil Transport System. It is responsible for planning and executing a safe and efficient route for the autonomous school shuttle.It receives real-time vehicle motion data, student pickup, and school locations, then computes an optimized path while avoiding obstacles. The planner publishes velocity commands to navigate the shuttle efficiently from the starting point to the destination.

---

<div align="center">
    <img src="Images/TP_image.png" height=500, width=800>
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

| **Topic Name**            | **Input/Output**    | **Message Type**             | **Description** |
|---------------------------|---------------------|------------------------------|-----------------|
| `/current_pose`           | **Input** (Subscribe) | `geometry_msgs/PoseStamped`  | Provides the shuttle's real-time position. |
| `/student_position_xyz`   | **Input** (Subscribe) | `el_msgs/msg/studentLocationXYZ.msg`  | Receives both the student pickup location and school destination. |
| `/obstacle_detection`     | **Input** (Subscribe) | `sensor_msgs/LaserScan`      | Detects obstacles along the planned route. |
| `/vehicle_motion_state`   | **Input** (Subscribe) | `std_msgs/String`            | Retrieves real-time motion state from the master controller. |
| `/path_data`     | **Output** (Publish)  | `nav_msgs/Path`              | Publishes the computed trajectory from start to destination. |
| `/target_velocity`                | **Output** (Publish)  | `std_msgs/Float64`        | Sends velocity commands for shuttle movement. |
| `/target_steering_angle`          | **Output** (Publish)  | `std_msgs/Float64`        | Sends steering angle commands for shuttle movement. |
## RQT_graph
---
<div align="center">
    <img src="Images/rqt_graph.png" height=500, width=800>
</div>
---


## Installation
1. Clone the repository:
```bash
 git clone https://git.hs-coburg.de/Team_ELITE/EL_Path_Planner.git
```
2. Build the package:
```bash
 colcon build --packages-select tj_planner
```
3. Source the workspace:
```bash
 source install/setup.bash
```

## Usage
### Launching the Nodes
To launch all of the nodes in lateral control package, run the following command:

```bash
ros2 run tj_planner tj_package 
```

## Testing
### Unit Tests
To run the unit tests for this package, use the following command:

```bash
colcon test --packages-select tj_planner
```

## License

This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.