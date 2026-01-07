# Quickstart Guide for Module 2: Robotic Nervous System with ROS 2

## Overview

This quickstart guide provides a rapid introduction to the key concepts and tools you'll encounter in Module 2: Robotic Nervous System with ROS 2. This guide will help you set up your environment and run your first ROS 2 examples using Python.

## Prerequisites

Before starting with Module 2, ensure you have:

1. **Basic Python Knowledge**: Understanding of Python programming concepts
2. **Module 1 Knowledge**: Completion of Module 1 (Foundations) or equivalent
3. **Development Environment**: Access to a Linux-based system or Docker container
4. **Hardware Requirements**: At least 8GB RAM, 50GB free disk space

## Environment Setup

### Installing ROS 2 Humble Hawksbill

For Ubuntu 22.04:

```bash
# Set locale
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### Setting up the ROS 2 Environment

```bash
# Source the ROS 2 installation
source /opt/ros/humble/setup.bash

# Add to your bashrc to automatically source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Installing Gazebo Garden

```bash
# Add Gazebo Garden repository
wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
sudo mv /tmp/gazebo.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/garden/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/garden.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden
```

## First ROS 2 Node Example

Create your first ROS 2 Python node:

```python
# talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

To run this example:

```bash
# Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a package for your examples
colcon build
source install/setup.bash

# Run the talker node in one terminal
python3 talker.py

# In another terminal, run the listener
ros2 run demo_nodes_cpp listener
```

## Working with Topics

Topics are the primary communication mechanism for streaming data:

```python
# Subscriber example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Working with Services

Services provide request-response communication:

```python
# Service client example
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()

    client = ServiceClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## URDF Basics

Create a simple URDF robot model:

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Launch Files

Create a launch file to start multiple nodes:

```python
# my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='my_talker',
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='my_listener',
        ),
    ])
```

## Running Gazebo Simulation

Launch a simple simulation:

```bash
# Launch Gazebo
gz sim -r -v 4 empty.sdf

# Or use ROS 2 integration
ros2 launch gazebo_ros empty_world.launch.py
```

## Common Commands

- `ros2 node list` - List active nodes
- `ros2 topic list` - List active topics
- `ros2 service list` - List active services
- `ros2 param list <node_name>` - List parameters of a node
- `ros2 run <package_name> <executable_name>` - Run a node
- `ros2 topic echo <topic_name>` - Listen to a topic
- `ros2 topic pub <topic_name> <msg_type> <args>` - Publish to a topic

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure you've sourced your workspace: `source install/setup.bash`

2. **Permission denied**: Check that ROS 2 is properly installed and in your PATH

3. **Nodes not communicating**: Verify that nodes are on the same ROS_DOMAIN_ID

4. **Gazebo not launching**: Ensure proper graphics drivers are installed

## Next Steps

After completing this quickstart, you should:

1. Practice creating your own simple nodes with publishers and subscribers
2. Experiment with services and actions
3. Create simple URDF models
4. Try basic Gazebo simulations
5. Move on to Chapter 1 of Module 2 for deeper understanding

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Garden Documentation](https://gazebosim.org/docs/garden)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

This quickstart provides the foundational knowledge needed to begin working with the concepts covered in Module 2. Each chapter will build upon these basics to develop more sophisticated understanding of the robotic nervous system using ROS 2.