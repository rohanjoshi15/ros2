
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
### URDF & GAZEBO OVERVIEW
---
## URDF
- The unified robotics description format (URDF) is an extensible markup language (XML) file type that includes the physical description of a robot.
- a 3-D model with information around joints, motors, mass

### 🔗 `<link>` Tag

Represents a rigid body with inertia, visual, and collision properties.

```xml
<link name="base_link">
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 0.5 0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 0.5 0.2"/>
    </geometry>
  </collision>
</link>
```

### `<joint>` Tag

Defines a connection between two links.

```xml
<joint name="front_left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="front_left_wheel"/>
  <origin xyz="0.5 0.4 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

- `type`: fixed, continuous, revolute, prismatic, planar, floating
- `axis`: Axis of rotation or movement

### Geometry Tags

Used inside `visual` or `collision`.

- `<box size="x y z"/>`
- `<cylinder length="h" radius="r"/>`
- `<sphere radius="r"/>`
- `<mesh filename="package://..." scale="x y z"/>`

### `<material>` Tag

Used to define color or texture.

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```
---

## GAZEBO PLUGINS
- Gazebo plugins are shared libraries loaded by the Gazebo simulator to add custom behavior to models, sensors, and the world.

---

## Types of Gazebo Plugins

1. **World Plugins** – Affect the entire world (e.g., control weather, lighting).
2. **Model Plugins** – Affect a specific model (e.g., move a robot).
3. **Sensor Plugins** – Used for simulating sensors (e.g., cameras, lidars).
4. **System Plugins** – Run globally, can interact with the whole system.

---

## Structure of a Plugin Tag in URDF/XACRO

```xml
<gazebo>
  <plugin name="plugin_name" filename="libplugin_file.so">
    <!-- Plugin-specific parameters -->
  </plugin>
</gazebo>
```

- `name`: Logical name of the plugin instance
- `filename`: Compiled `.so` shared object file
- Contents: Configuration parameters specific to the plugin

---

## Commonly Used Plugins in Robot URDF

### 1. `gazebo_ros_control`

Integrates ROS control with Gazebo.

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/>
</gazebo>
```

Requires `transmission` definitions in URDF for each joint.

---
## Creating a Custom Plugin

### C++ Skeleton

```cpp
#include <gazebo/gazebo.hh>

namespace gazebo {
  class MyPlugin : public ModelPlugin {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      std::cout << "Plugin loaded!" << std::endl;
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}
```

### Build with CMake

```cmake
add_library(my_plugin SHARED src/my_plugin.cpp)
target_link_libraries(my_plugin ${GAZEBO_LIBRARIES})
```
---
## References

- [Gazebo ROS Plugins](http://gazebosim.org/tutorials?tut=ros_plugins)
- [ROS Control](http://wiki.ros.org/ros_control)
- [Custom Plugin Example](http://gazebosim.org/tutorials?tut=plugins_model)
---

## How It Works in our rover

### XACRO and URDF

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
- Its a Node that already exists within ros2.
- We pass the urdf as a parameter to this node so the robot state publisher will know what are the links and joints so its can know how to publish the tf.
- The joint_states topic sends/publishes the state of the joint to the rsp.
- Once we have the urdf and joint_states the rsp can publish the /tf.
- During simulation we use a fake joint state pub , but irl we have node which will read data from the hardware and publish it on the joint state publisher.
- Static transforms are unconditionally published to the /tf_static topic, and that the static transforms are published in a transient_local Quality of Service.
- To listen they use tf2_ros::Buffer and tf2_ros::TransformListener

 # Working
- Robots are described in URDF/Xacro as a tree of links connected by joints.
- Each joint has a variable position (like angle for revolute joints, or displacement for prismatic ones). However, the URDF is static — it doesn’t update during motion.)
- That’s where the Robot State Publisher comes in:
- It reads the joint values from the /joint_states topic
- It uses forward kinematics to compute the pose of each link.
- It publishes the full TF tree via /tf.
---

### rosbag2
- Rosbag2 — the tool for recording and playback of communications in ROS 2 systems.
- A "rosbag" is simply a file full of timestamped messages. The first goal of the tool is efficient recording, to support complex systems in real time. Its second goal is efficent playback, to allow for viewing and using recorded data.
# How to use verbs
- ros2 bag burst/ convert/ info/ list/ play/ record/ reindex

  
---
### Communication in ROS2
- The subscriptions and publications mechanisms in ROS 2 fall in two categories:
- intra-process: messages are sent from a publisher to subscriptions via in-process memory.
- inter-process: messages are sent via the underlying ROS 2 middleware layer.
- The current implementation is based on the creation of a ring buffer for each Publisher and on the publication of meta-messages through the middleware layer.
- When a Publisher has to publish intra-process, it will pass the message to the IntraProcessManager. Here the message will be stored in the ring buffer associated with the Publisher. In order to extract a message from the IntraProcessManager two pieces of information are needed:
- the id of the Publisher (in order to select the correct ring buffer)
- the position of the message within its ring buffer.
- A meta-message with this information is created and sent through the ROS 2 middleware to all the Subscriptions, which can then retrieve the original message from the IntraProcessManager.
- The current implementation can’t be used when the QoS durability value is set to `Transient Local`.

- 





