
# ROS 2 Simulation task

- This repository contains a complete ROS 2-based simulation setup for a 4-wheeled differential drive rover with LiDAR and camera sensors, emergency stop logic.
- The gazebo simulation requires xacro file and gazebo plugins
---

## Project Structure and commands to use
- Use commands :
- ros2 launch rover_controller gazebo_rviz.launch.py to launch gazebo and rviz and spawn the entity
- ros2 launch rover_controller launch_nodes.launch.py to launch the three nodes for the stopping logic
```
src/rover_controller
├── model/
│   ├── robot.xacro          # The XACRO-based robot model
│   ├── robot.gazebo         # Gazebo-specific extensions
│   ├── empty_world.world    # Custom world for simulation Ive added a wall to it 
├── launch/
│   └── launch_nodes.launch.py #launches all three of the nodes 
│   └── gazebo_rviz.launch.py # Launches Gazebo, robot, RViz2
├── rviz/
│   └── rover_task_lidar_camera.rviz
├── src/
│   ├── lidar_processor.cpp
│   ├── proximity_warning.cpp
│   └── emergency_stop.cpp
```

---

## How It Works

### 🔧 XACRO and URDF

- `robot.xacro` describes the physical structure (links, joints) of the robot in a modular way.
- It supports parameters and math operations to build a compact URDF.
- The file is processed at launch using:
  ```bash
  xacro robot.xacro > robot.urdf
  ```

### Gazebo Extensions (`robot.gazebo`)

- Adds material, friction (`mu1`, `mu2`), sensor visuals, and plugins to the model.
- Plugin: `libgazebo_ros_diff_drive.so` is used to convert `cmd_vel` commands into joint movements.
- Example:
  ```xml
  <plugin name="wheel_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <left_joint>back_left_wheel_joint</left_joint>
    <right_joint>back_right_wheel_joint</right_joint>
    ...
  </plugin>
  ```

### World File

- `empty_world.world` includes `sun`, `ground_plane`, and a `wall`
- Made in gazebo and saved file

---

## ROS 2 Nodes and Communication

### 1. **lidar_processor.cpp**
- Subscribes to `/scan` (LaserScan).
- Publishes minimum distance to `/min_distance`.

### 2. **proximity_warning.cpp**
- Subscribes to `/min_distance`.
- Publishes `STOP` message to `/proximity_warning` if object < 0.9m.

### 3. **emergency_stop.cpp**
- Subscribes to `/proximity_warning`.
- Logs FATAL warning and can be extended to send a `cmd_vel` stop command.

```bash
ros2 run rover_bot lidar_processor
ros2 run rover_bot proximity_warning
ros2 run rover_bot emergency_stop
```

---

## Launching the Simulation

###  Build the package, Change the src_rover to src
```bash
cd ~/ws_rover (or the name you give)
colcon build
source install/setup.bash
```


## RViz2 Setup

- RViz2 config file: `rover_task_lidar_camera.rviz`
- It automatically sets:
  - Fixed frame: `odom`
  - Displays for `/scan`, TF, camera
  - 

---

## Important Commands

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /scan
ros2 topic echo /min_distance
ros2 topic echo /proximity_warning
```


---

## QoS

| Topic               | Reliability | Durability       | Reason                                  |
|--------------------|-------------|------------------|-----------------------------------------|
| `/scan`            | Best Effort | Volatile         | High-rate sensor, latest value is enough |
| `/min_distance`    | Reliable    | Volatile         | Used for control decisions              |
| `/proximity_warning`| Reliable    | Transient Local  | Critical command for stopping           |


---
## TF
- It means transform , its a coordinate tracking system in ROS
- It tracks multiple frames such as body_link lidar_link camera_link (in this project)
- It is so that every link knows where it is wrt the other at a certain point of time
- Each transform has a translation , rotation and a timestamp

- Nodes use tf2_ros::TransformBroadcaster to publish transforms
- To listen they use tf2_ros::Buffer and tf2_ros::TransformListener
---
