# Gazebo Simulation Environment Setup

## Learning Objectives
By the end of this chapter, readers will be able to:
- Configure Gazebo simulation environments with appropriate physics parameters
- Create and customize world models for robot testing
- Understand the relationship between simulation parameters and real-world physics
- Set up basic robot models in Gazebo simulation environment

## Prerequisites
- Completion of Module 1: Foundations of Physical AI & Humanoid Robotics
- Completion of Module 2: Robotic Nervous System
- Basic understanding of ROS/ROS 2 concepts
- Fundamental knowledge of robot kinematics and dynamics

## Introduction

Gazebo is a powerful 3D simulation environment that plays a critical role in robotics development. It provides realistic physics simulation, high-quality rendering, and convenient programmatic interfaces. For humanoid robotics development, Gazebo serves as a safe, cost-effective environment where complex robot behaviors can be tested without risk to physical hardware.

This chapter focuses on setting up Gazebo simulation environments specifically tailored for humanoid robotics applications. We'll explore the physics engine parameters, world creation techniques, and best practices for creating realistic simulation environments.

## 1. Installing and Configuring Gazebo Garden

### 1.1 Gazebo Garden Installation

Gazebo Garden (Harmonic) is the latest stable version with ongoing support and is fully compatible with ROS 2 Humble Hawksbill. To install Gazebo Garden, follow these steps:

```bash
# Add Gazebo repository
curl -sSL http://get.gazebosim.org | sh

# Install Gazebo Garden
sudo apt-get install gz-harmonic
```

After installation, verify that Gazebo is properly installed by running:

```bash
gz --version
```

### 1.2 Gazebo Physics Engine Configuration

Gazebo supports multiple physics engines including ODE, Bullet, and DART. For humanoid robotics simulation, we recommend using the ODE (Open Dynamics Engine) physics engine due to its stability and accuracy for legged locomotion.

The physics engine is configured in the world file using the `<physics>` tag:

```xml
<sdf version='1.7'>
  <world name='humanoid_world'>
    <!-- Physics Engine Configuration -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Rest of the world definition -->
  </world>
</sdf>
```

### 1.3 Key Physics Parameters

- **max_step_size**: The maximum time step size for the physics engine. Smaller values provide more accurate simulation but require more computation. For humanoid robotics, 0.001 seconds is recommended.
- **real_time_factor**: The target factor at which the simulation should run in relation to real time. A value of 1.0 means the simulation should run at real-time speed.
- **real_time_update_rate**: The rate at which the physics engine updates. Higher values provide smoother simulation.
- **gravity**: The gravitational acceleration vector (x, y, z) in m/s². Earth's gravity is approximately 9.8 m/s² downward.

## 2. Creating Basic World Files

### 2.1 World File Structure

A Gazebo world file is an SDF (Simulation Description Format) file that defines the simulation environment. Here's a basic structure:

```xml
<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='humanoid_basic_world'>
    <!-- Physics Engine -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene Configuration -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sky -->
    <plugin filename="gz-sim-sky-spherical_coordinates-system" name="gz::sim::systems::SkySphericalCoordinates">
      <time_daylight>12:00</time_daylight>
      <sunrise>6:00</sunrise>
      <sunset>18:00</sunset>
    </plugin>
  </world>
</sdf>
```

### 2.2 Customizing Environments

World customization allows for creating diverse testing environments for humanoid robots. Key customization options include:

- **Terrain**: Creating uneven surfaces, ramps, stairs, or obstacles
- **Objects**: Adding furniture, walls, or interactive elements
- **Lighting**: Adjusting intensity, direction, and color of light sources
- **Weather**: Simulating different atmospheric conditions

## 3. Spawning Robots in Gazebo

### 3.1 Basic Robot Spawning

Robots can be spawned in Gazebo using the `gz model` command or programmatically through ROS 2 services. Here's an example of spawning a robot from the command line:

```bash
# Spawn a robot model from a local file
gz model -f /path/to/robot/model.sdf -m robot_name

# Spawn a robot model from a gazebo model database
gz model -m robot_name --model-name robot_instance_name -z 1.0
```

### 3.2 Programmatic Robot Spawning with ROS 2

Using ROS 2, robots can be spawned programmatically:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')

    def spawn_robot(self, robot_xml, robot_name, initial_pose):
        request = SpawnEntity.Request()
        request.xml = robot_xml
        request.name = robot_name
        request.initial_pose = initial_pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {robot_name}')
        else:
            self.get_logger().error(f'Failed to spawn {robot_name}')

def main():
    rclpy.init()
    spawner = RobotSpawner()

    # Example: spawn a simple box robot
    robot_xml = """
    <sdf version='1.7'>
      <model name='simple_box_robot'>
        <link name='chassis'>
          <pose>0 0 0.1 0 0 0</pose>
          <collision name='collision'>
            <geometry>
              <box>
                <size>0.5 0.5 0.2</size>
              </box>
            </geometry>
          </collision>
          <visual name='visual'>
            <geometry>
              <box>
                <size>0.5 0.5 0.2</size>
              </box>
            </geometry>
          </visual>
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx>
              <iyy>0.01</iyy>
              <izz>0.01</izz>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyz>0</iyz>
            </inertia>
          </inertial>
        </link>
      </model>
    </sdf>
    """

    initial_pose = ...
    spawner.spawn_robot(robot_xml, 'simple_box_robot', initial_pose)

    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. World Customization Techniques

### 4.1 Creating Obstacle Courses

For humanoid robotics testing, creating obstacle courses helps evaluate locomotion and navigation capabilities:

```xml
<!-- Example: Simple obstacle course -->
<model name='obstacle_1'>
  <pose>2 0 0.1 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry>
        <box>
          <size>0.2 2.0 0.5</size>
        </box>
      </geometry>
    </collision>
    <visual name='visual'>
      <geometry>
        <box>
          <size>0.2 2.0 0.5</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 4.2 Terrain Generation

For more complex terrain, you can create heightmaps or use procedural generation techniques. Gazebo supports various terrain types including flat planes, heightmaps, and custom meshes.

## 5. Simulation Validation

### 5.1 Physics Accuracy Verification

To verify that the simulation physics accurately represent real-world behavior:

1. **Gravity validation**: Drop objects and measure fall time to verify gravitational acceleration
2. **Collision detection**: Test object interactions to ensure proper collision handling
3. **Friction modeling**: Validate friction coefficients by testing sliding behaviors

### 5.2 Performance Optimization

For smooth simulation performance:
- Keep triangle counts reasonable for meshes
- Use appropriate physics update rates
- Limit the number of active sensors
- Optimize rendering settings based on hardware capabilities

## 6. Practical Example: Setting Up a Basic Humanoid Testing Environment

Let's create a complete example of a basic humanoid testing environment:

```xml
<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='humanoid_test_world'>
    <!-- Physics Configuration -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene Settings -->
    <scene>
      <ambient>0.3 0.3 0.3 1</ambient>
      <background>0.6 0.6 0.6 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Simple Ramp for Testing -->
    <model name='ramp'>
      <pose>3 0 0 0 0.2 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://meshes/ramp.dae</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle Course -->
    <model name='stepping_stone_1'>
      <pose>5 0 0.05 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## 7. Summary

This chapter introduced the fundamentals of setting up Gazebo simulation environments for humanoid robotics. We covered:

- Installing and configuring Gazebo Garden with appropriate physics parameters
- Creating and customizing world files for robot testing
- Spawning robots programmatically and through command-line tools
- World customization techniques for diverse testing scenarios
- Simulation validation and performance optimization

With a properly configured Gazebo environment, you can now proceed to model humanoid robots with appropriate physics properties and joint constraints, which will be covered in the next chapter.

## 8. Exercises and Practice

Complete the following exercises to reinforce your understanding of Gazebo simulation environment setup:

1. [Chapter 1 Exercises](./exercises.md) - Practice problems covering Gazebo environment creation and customization
2. [Chapter 1 Solutions](./solutions.md) - Complete implementations and solution guides