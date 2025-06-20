
# ROS 2 Simulation task

- This repository contains a complete ROS 2-based simulation setup for a 4-wheeled differential drive rover with lidar and camera sensors, emergency stop logic.
- The gazebo simulation requires xacro file and gazebo plugins
---

## Project Structure and commands to use
- Use commands :
- `ros2 launch rover_controller gazebo_rviz.launch.py` to launch gazebo and rviz and spawn the entity
- `ros2 launch rover_controller launch_nodes.launch.py` to launch the three nodes for the stopping logic
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

## Gazebo Extensions (`robot.gazebo`)

Adds Gazebo-specific configurations to the robot model:

### 1. Visual & Physical Properties
```xml
<gazebo reference="body_link">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <material>Gazebo/Red</material>
</gazebo>
```

- Adds friction and color to the body and wheels.
- Add fricition for more efficient stopping

### 2. Differential Drive Plugin
```xml
<plugin name="wheel_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <left_joint>back_left_wheel_joint</left_joint>
  <right_joint>back_right_wheel_joint</right_joint>
  ...
</plugin>
```
- Converts `cmd_vel` velocity commands into joint movements.
- Used for both differential drive and skid steering (by pairing wheel joints).

### 3. LiDAR Sensor Plugin
```xml
<plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <argument>~/out:=scan</argument>
  </ros>
  <frame_name>laser_frame</frame_name>
</plugin>
```
- Publishes `/scan` topic.
- Shows LiDAR rays after enabling `<visualize>true</visualize>` inside the `<sensor>`.
- You can see the lidar rays in the rviz.

### 4. Camera Plugin
```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <cameraName>rrbot/camera1</cameraName>
  <imageTopicName>image_raw</imageTopicName>
  ...
</plugin>
```
- Publishes image stream via `/image_raw`.

---

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
- Logs FATAL warning and extended to send a `cmd_vel` stop command.


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
  - The rover
  - TF Frames
  - Camera feed
  - LaserScan
  - Odometry

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

---
### Robot_State_Publisher
- Publishes transforms based on URDF/XACRO joint angles.
- It allows Rviz to know the rovers kinematic state
- Works with `joint_state_publisher` to simulate joint motion.
- To listen they use tf2_ros::Buffer and tf2_ros::TransformListener

 # Working
- Robots are described in URDF/Xacro as a tree of links connected by joints.
- Each joint has a variable position (like angle for revolute joints, or displacement for prismatic ones). However, the URDF is static — it doesn’t update during motion.

- That’s where the Robot State Publisher comes in:
- It reads the joint values from the /joint_states topic
- It uses forward kinematics to compute the pose of each link.
- It publishes the full TF tree via /tf.
---
