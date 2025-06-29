# ROS 2 (Robot Operating System 2)
## For the 4 Wheel Differential drive rover go to master branch

ROS 2 is an upgrade over ROS 1, utilizing DDS (Data Distribution System) for real-time, multi-platform, and decentralized communication.
- I have provided the resources for further references below.
- Prior knowledge of Cpp , Python , OOPs , Linux commands needed.
---

## Workspace

In ROS 2, a **workspace** is where you keep all your packages.

### Create a Workspace
In ROS 2, a workspace is where you keep all your packages. You create a workspace using:
```bash
mkdir -p ~/tasks_ws/src    # 'tasks_ws' is the workspace name which i have given , It can be custom
cd ~/tasks_ws
colcon build --symlink-install
source install/setup.bash
```
## What "symlink-install" does is that it tells colcon to symlink (create symbolic links) instead of copying files when installing built packages to the install/ directory.
## Benefits of --symlink-install:
- Faster builds: Files are not copied; just linked.
- Easier debugging: If you modify source files, the changes are immediately visible in the install/ space because it's linked, not copied.
- Efficient for development: You don’t have to rebuild every time you make minor changes to scripts or launch files.
---

## Package

A **package** in ROS 2 is a basic unit of code organization. It contains source files, headers, `CMakeLists.txt`, and `package.xml`.

### Create a C++ Package

```bash
cd ~/tasks_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
```

> Place your `.cpp` files in the `src` folder of your package.

---
## To run the example code use this
- ros2 launch number_square launch_nodes.py
---
## Nodes

A **Node** is the smallest running process in ROS 2.

- The wheel is an **object**.
- The node is the **software** that controls or reads from that object.

Define a node by creating a class that inherits from `rclcpp::Node` and implement its functionality in the constructor or private methods.

---

## Topics

A **topic** is like a channel used by ROS 2 nodes to send and receive data. It’s part of the publish–subscribe communication model.
- One or more Publisher can be connected to one or more Subscriber.
- **Publisher**: Create a publisher and call `publish()` with the message.
- **Subscriber**: Use `create_subscription()` and pass a **callback function**.
- **Send data** : You create a publisher and call publish() with the message. The message type must be one of the existing standard messages or your custom .msg files.
- **Recieve data** : you create a Subscriber using create_subscription() and pass a callback function. The callback function is where you do anything with the received message.


### Example Question :- Get a number and display its Square and Cube.
---
## Publisher Node: `NumberPublisher`

### Included Headers

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
```

### Class Definition

```cpp
class NumberPublisher : public rclcpp::Node {
```

- Inherits from `rclcpp::Node` to become a ROS 2 node.

### Constructor

```cpp
NumberPublisher() : Node("number_publisher"), num(1) {
```

- Initializes the node with the name `number_publisher`.
- Initializes `num` to 1.

### Publisher Creation

```cpp
publisher_ = this->create_publisher<std_msgs::msg::Int32>("sending_nums", 10);
```

- Publishes messages to the topic `sending_nums`.
- `10` is the queue size.
- `this->` is needed to access base class methods from within the derived class. (so basically its a 'this pointer')

### Timer
- It calls the function given as parameter in the timer bind repeatedly at a fixed interval
```cpp
timer = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&NumberPublisher::publish_number, this));
```

- Sets a timer to call `publish_number()` every second. (can change the timers duration too)
- `std::bind(..., this)` binds the method to the instance of the class.

### Publish Callback

```cpp
void publish_number() {
    auto msg = std_msgs::msg::Int32();
    msg.data = num++;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
    publisher_->publish(msg);
}
```

- Constructs the message, sets `data`, logs it, and publishes it.

---

## Subscriber Node: `NumberSubscriber`

### Subscriptions

```cpp
subscription_1 = this->create_subscription<std_msgs::msg::Int32>(
    "sending_nums", 10,
    std::bind(&NumberSubscriber::topic_callback_1, this, std::placeholders::_1));
```

- Subscribes to `sending_nums` topic. Topic names are custom made
- Uses `std::bind` with `_1` placeholder for callback argument.

### Multiple Subscriptions

Two subscriptions to the same topic but with different callbacks:

- `topic_callback_1`: calculates and logs the **square**.
- `topic_callback_2`: calculates and logs the **cube**.

### Callback Functions

```cpp
void topic_callback_1(const std_msgs::msg::Int32::SharedPtr msg) {
    int square = msg->data * msg->data;
    RCLCPP_INFO(this->get_logger(), "Received: %d, Square: %d", msg->data, square);
}
```
---

### Why is the Callback Function Necessary?

- The **callback function** is what runs when a new message arrives — it's where you process, print, log, use, or modify the data.
- Without a callback, the subscription does nothing.

---



## Launch Files

Launch files allow you to start **multiple nodes** with a single command. Launch files can be written in **Python** (recommended), regardless of the package language.
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='number_square',
            executable='pub_node',
            name='number_publisher'
        ),
        Node(
            package='number_square',
            executable='sub_node',
            name='number_subscriber'
        )
    ])
```
---

## CMakeLists.txt

### What Does CMakeLists.txt Do?

- Automates compiling and creates executable `.cpp` files.
- Make sure to update the dependencies (`rclcpp`, `sensor_msgs`, etc.)
- Make sure to add executables and use `ament_target_dependencies()`.
- Install the executable via `install(TARGETS ..)`.

---

## Shared Pointers

- They are smart pointers which are used for dynamic memory allocation for pointers
- `std::shared_ptr<T>` holds a pointer to an object and automatically frees memory when the object goes out of scope.
- In ROS 2, messages are usually passed around using shared pointers `std::shared_ptr<std_msgs::msg::String>`.

---

## Services

Used for request–response tasks.

- Define a **service server** using `create_service()` and pass a callback.
- The **client** makes a request; the **server** responds.

---

## Actions

- The **client node** sends a **goal** to the **action server**.
- While the task is running, the **server sends feedback** to the client to keep it updated on the progress.
- Once the task is finished, the **server sends the final result** back to the client.
- So this is a mix of services and topics.

  
---


## DDS (Data Distribution System)

Replaces the ROS master. DDS enables:

1. Message serialization (e.g., Fast-CDR)
2. Decentralized communication (peer-to-peer)
3. Discovery process

### Decentralized Communication (Unlike ROS1)
- replaces the ros master
- The DDS system is decentralized, meaning that the participants are not managed by any one entity.
- Instead the system works peer to peer, allowing nodes to find each other.
- You create a message in your ROS 2 node (std_msgs::msg::String).
- When the message is published:
- The ROS 2 middleware calls DDS.
- DDS uses a serialization library to convert the message into a CDR (Common Data Representation) byte stream.
- DDS sends the byte stream over the network.
- The subscriber's DDS middleware receives the byte stream and deserializes it back into the original ROS 2 message.

### Discovery Process

Participants (nodes) send periodic multicast announcement messages over the network with their unicast addresses, and then all participants share discovery information (such as topics, and action servers.) with all other participants.

---

## Quality of Service (QoS)

QoS defines a group of settings called **policies** that dictate how a message is transmitted between nodes.

- **Publisher QoS**: Defines the *maximum* level of quality available.
- **Subscriber QoS**: Defines the *required* level of quality for the subscription.

---

## Publisher QoS

```cpp
rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
qos_profile
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .keep_last(15)
    .deadline(rclcpp::Duration::from_seconds(2))
    .lifespan(rclcpp::Duration::from_seconds(4))
    .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
    .liveliness_lease_duration(rclcpp::Duration::from_seconds(3));
```

##  Subscriber QoS
## To test out differences between these subscriptions run them separately after a certain amount of time, If you use a launch file they will be launched together and no difference will be noticieable.
---

###  Subscriber 1 QoS: for Squaring the number

```cpp
rclcpp::QoS qos_1(rclcpp::KeepLast(10));
qos_1
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

- **Reliable + Transient Local**: Subscriber receives *reliable* delivery and even older messages that were published before the subscriber joined.

---

### Subscriber 2 QoS: for Cubing the number

```cpp
rclcpp::QoS qos_2(rclcpp::KeepLast(5));
qos_2
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
```

- **Best Effort + Volatile**: Subscriber does **not** get guaranteed delivery or old messages. It only gets messages that happen to arrive when it's active.


- The DDS middleware will try to **match** the QoS between publisher and subscriber.
  - If incompatible (e.g., Reliable publisher vs Best Effort subscriber), messages may not be delivered.
---



### QoS Policies

1. **History**  
   Controls how a message is stored:
   - `Keep last`: Stores only the last *N* messages.
   - `Keep all`: Stores all messages (until resource limits are reached).

2. **Depth**  
   - Defines the size of the message queue.  
   - Only applies when history is set to `Keep last`.

3. **Reliability**  
   - `Best Effort`: Sends the message once without confirming delivery. Useful for streaming data like sensor readings.
   - `Reliable`: Retries sending the message until it is successfully delivered. Better for large or critical messages.

4. **Durability**  
   - `Transient Local`: The publisher keeps a copy of messages to deliver to new subscribers that join late.
   - `Volatile`: The publisher does not store any messages. Late subscribers only receive messages published after they join.

5. **Deadline**  
   - Specifies the maximum time allowed between two consecutive messages. If exceeded, it's assumed something has gone wrong.

6. **Lifespan**  
   - Defines how long a message is valid after being published. If not received within this time, it is discarded.

7. **Liveliness**  
   - `Automatic`: The system considers the publisher "alive" as long as messages are being published.
   - `Manual by topic`: The publisher must manually assert it is alive, even if not actively publishing.

8. **Lease Duration**  
   - Maximum time a publisher can remain silent before it is considered dead or inactive.

---

### QoS Compatibility Tables
    
- You can find the tables on https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
---
# Important Point
- Subscriptions request a QoS profile that is the “minimum quality” that it is willing to accept, and publishers offer a QoS profile that is the “maximum quality” that it is able to provide. Connections are only made if every policy of the requested QoS profile is not more stringent than that of the offered QoS profile. Multiple subscriptions can be connected to a single publisher simultaneously even if their requested QoS profiles are different.
- ROS 1, any publisher and subscriber with the same message type on the same topic would be connected. The possibility of incompatible requested and offered QoS profiles is something new to be aware of when using ROS 2.

## Compatibility of Reliability QoS Policies

| Publisher     | Subscription   | Compatible | 
|---------------|----------------|------------|
| Best effort   | Best effort    | Yes        |
| Best effort   | Reliable       | No         |
| Reliable      | Best effort    | Yes        |
| Reliable      | Reliable       | Yes        |

## Compatibility of Durability QoS Policies

| Publisher       | Subscription     | Compatible | Result               |
|------------------|------------------|------------|----------------------|
| Volatile         | Volatile         | Yes        | New messages only    |
| Volatile         | Transient local  | No         | No communication     |
| Transient local  | Volatile         | Yes        | New messages only    |
| Transient local  | Transient local  | Yes        | New and old messages |

## Compatibility of Deadline QoS Policies

_Assume x and y are arbitrary valid duration values._

| Publisher | Subscription         | Compatible |
|-----------|----------------------|------------|
| Default   | Default              | Yes        |
| Default   | x                    | No         |
| x         | Default              | Yes        |
| x         | x                    | Yes        |
| x         | y (where y > x)      | Yes        |
| x         | y (where y < x)      | No         |

## Compatibility of Liveliness QoS Policies

| Publisher         | Subscription        | Compatible |
|--------------------|---------------------|------------|
| Automatic          | Automatic           | Yes        |
| Automatic          | Manual by topic     | No         |
| Manual by topic    | Automatic           | Yes        |
| Manual by topic    | Manual by topic     | Yes        |

## Compatibility of Lease Duration QoS Policies

_Assume x and y are arbitrary valid duration values._

| Publisher | Subscription         | Compatible |
|-----------|----------------------|------------|
| Default   | Default              | Yes        |
| Default   | x                    | No         |
| x         | Default              | Yes        |
| x         | x                    | Yes        |
| x         | y (where y > x)      | Yes        |
| x         | y (where y < x)      | No         |

---

### ROS2 Vs ROS1

### 1. Communication Middleware

- **ROS 1**: Uses custom communication protocols (TCPROS and UDPROS).
- **ROS 2**: Built on DDS (Data Distribution Service), enabling real-time communication, scalability, and quality of service (QoS) settings.

### 2. Real-Time Capability

- **ROS 1**: Lacks real-time support.
- **ROS 2**: Designed with real-time capabilities, suitable for time-critical applications.

### 3. Multi-Robot Support

- **ROS 1**: Limited support; requires workarounds for multi-robot systems.
- **ROS 2**: Native support for multi-robot systems with improved namespace management.

### 4. Security

- **ROS 1**: No built-in security features.
- **ROS 2**: Incorporates DDS security, including authentication, encryption, and access control.

---


| ROS 2 QoS Policy        | ROS 1 Equivalent (if any)                 |
|-------------------------|-----------------------------------------|
| History + Depth         | Queue size                              |
| Reliability (Best Effort)| UDPROS (roscpp)                        |
| Reliability (Reliable)  | TCPROS (default in ROS 1)               |
| Durability (Transient Local) | Latching publishers                |
| Others (Deadline, Lifespan, etc.) | No equivalent; ROS 2 exclusive |







### Resources & Documentations 
1. https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
2. https://design.ros2.org/articles/ros_on_dds.html
3. https://design.ros2.org/articles/intraprocess_communications.html
4. https://www.youtube.com/@aleksandarhaber
5. https://www.youtube.com/@robotisim
6. https://www.youtube.com/@kevinwoodrobotics

---

### ROS2 Command Line Interface (CLI) 
 The tables below show only a feew of the commands which have been used till ***yet***, there are more please refer to chatgpt or google too ;)
 
## General Format
```bash
ros2 <command> <subcommand> [options]
```

---

## Build and Workspace

| Command | Description |
|--------|-------------|
| `colcon build` | Builds all packages in the workspace |
| `source install/setup.bash` | Sources the environment after building |
| `ros2 pkg create <name> --build-type <type>` | Creates a new package |

---

## Nodes

| Command | Description |
|--------|-------------|
| `ros2 run <package> <executable>` | Runs a node from a given package |
| `ros2 node list` | Lists all running nodes |
| `ros2 node info <node_name>` | Displays publishers, subscribers, services, etc. of a node |

---

## Topics

| Command | Description |
|--------|-------------|
| `ros2 topic list` | Lists all available topics |
| `ros2 topic echo <topic>` | Prints messages from a topic |
| `ros2 topic info <topic>` | Shows type, publishers, and subscribers |
| `ros2 topic pub <topic> <type> "<data>"` | Publishes messages to a topic |
| `ros2 topic type <topic>` | Shows the message type of a topic |
| `ros2 topic hz <topic>` | Displays the publish rate of a topic , It can also send msgs at a wanted rate |


---

## Services

| Command | Description |
|--------|-------------|
| `ros2 service list` | Lists all active services |
| `ros2 service type <service>` | Shows the service type |

---

## Actions

| Command | Description |
|--------|-------------|
| `ros2 action list` | Lists available actions |
| `ros2 action send_goal <action> <type> "<goal>"` | Sends a goal to an action server |
| `ros2 action cancel` | Cancels a running goal |
| `ros2 action show <action>` | Shows the type of an action |

---

## Packages

| Command | Description |
|--------|-------------|
| `ros2 pkg list` | Lists all available packages |
| `ros2 pkg executables <package>` | Lists executables in a package |
