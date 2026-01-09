# Understanding URDF for Humanoid Robot Description and Control

## Learning Objectives
By the end of this chapter, readers will be able to:
- Explain the fundamentals of URDF (Unified Robot Description Format) and its role in ROS 2
- Create URDF files that describe humanoid robot structures and kinematics
- Implement joint definitions and transformations for humanoid robot models
- Use xacro macros to simplify complex humanoid robot descriptions
- Visualize and debug URDF models in RViz and Gazebo simulation
- Integrate URDF models with ROS 2 control systems for humanoid robot control

## Prerequisites
- Completion of Chapter 1: Understanding ROS 2 Architecture and Communication Patterns
- Completion of Chapter 2: Bridging Python-based AI Agents to Robot Controllers
- Basic understanding of 3D coordinate systems and transformations
- Knowledge of robot kinematics fundamentals from Module 1

## Introduction
The Unified Robot Description Format (URDF) is a fundamental component of the ROS ecosystem that allows for the description of robot models in XML format. For humanoid robotics, URDF provides the essential framework for defining the physical structure, kinematic properties, and visual representation of complex multi-jointed robots. This chapter explores how to create and use URDF files specifically for humanoid robot models, enabling proper simulation and control in ROS 2 environments.

URDF serves as the bridge between the abstract concept of a robot and its concrete representation in simulation and real-world applications. For humanoid robots, which typically feature complex structures with multiple degrees of freedom, proper URDF definition is crucial for successful simulation, visualization, and control.

## 1. URDF Fundamentals for Humanoid Robots

### 1.1 URDF Structure and Components
URDF (Unified Robot Description Format) is an XML-based format that describes robot models by defining links, joints, and their relationships. For humanoid robots, this structure becomes particularly important as it must represent complex kinematic chains such as arms, legs, and torso.

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.8"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.8"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

### 1.2 Links and Joints in Humanoid Models
In humanoid robots, links represent rigid bodies while joints define the connections and degrees of freedom between them. The typical humanoid structure includes:
- Base link (torso/trunk)
- Limbs (arms and legs) with multiple segments
- Joints that enable movement (revolute, continuous, prismatic)

```xml
<!-- Example of a simple humanoid arm -->
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>

<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0.25 0.15 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
</joint>
```

## 2. Creating Humanoid Robot Models

### 2.1 Basic Humanoid Structure
Creating a humanoid robot model requires careful attention to the kinematic structure. A basic humanoid typically includes:
- Torso (base link)
- Head
- Two arms (each with shoulder, elbow, and wrist joints)
- Two legs (each with hip, knee, and ankle joints)

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>
</robot>
```

### 2.2 Kinematic Chains and Degrees of Freedom
Humanoid robots require careful consideration of kinematic chains to ensure proper movement and stability. Each limb should be modeled as a series of connected links with appropriate joint types and limits.

```xml
<!-- Right leg with 3-DOF (hip, knee, ankle) -->
<link name="right_hip">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>

<joint name="right_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="right_hip"/>
  <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="0.5" effort="100.0" velocity="0.5"/>
</joint>

<link name="right_thigh">
  <visual>
    <geometry>
      <cylinder radius="0.06" length="0.4"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.06" length="0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
  </inertial>
</link>

<joint name="right_knee_joint" type="revolute">
  <parent link="right_hip"/>
  <child link="right_thigh"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.57" effort="150.0" velocity="0.5"/>
</joint>

<link name="right_shin">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
  </inertial>
</link>

<joint name="right_ankle_joint" type="revolute">
  <parent link="right_thigh"/>
  <child link="right_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="0.5" effort="80.0" velocity="0.5"/>
</joint>
```

## 3. Advanced URDF Features for Humanoid Robots

### 3.1 Using xacro for Complex Models
xacro (XML Macros) allows for parameterization and reusability of URDF models, which is essential for complex humanoid robots with many similar components.

```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_mass" value="10.0" />
  <xacro:property name="arm_mass" value="1.0" />
  <xacro:property name="leg_mass" value="2.0" />

  <!-- Macro for creating a generic link -->
  <xacro:macro name="generic_link" params="name mass geometry_type size material_color">
    <link name="${name}">
      <visual>
        <geometry>
          <xacro:if value="${geometry_type == 'box'}">
            <box size="${size}"/>
          </xacro:if>
          <xacro:if value="${geometry_type == 'cylinder'}">
            <cylinder radius="${size.split(' ')[0]}" length="${size.split(' ')[1]}"/>
          </xacro:if>
          <xacro:if value="${geometry_type == 'sphere'}">
            <sphere radius="${size}"/>
          </xacro:if>
        </geometry>
        <material name="material_${name}">
          <color rgba="${material_color}"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <xacro:if value="${geometry_type == 'box'}">
            <box size="${size}"/>
          </xacro:if>
          <xacro:if value="${geometry_type == 'cylinder'}">
            <cylinder radius="${size.split(' ')[0]}" length="${size.split(' ')[1]}"/>
          </xacro:if>
          <xacro:if value="${geometry_type == 'sphere'}">
            <sphere radius="${size}"/>
          </xacro:if>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for creating a generic joint -->
  <xacro:macro name="generic_joint" params="name type parent child xyz rpy axis limits">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <xacro:if value="${limits != ''}">
        <limit lower="${limits.split(' ')[0]}" upper="${limits.split(' ')[1]}"
               effort="${limits.split(' ')[2]}" velocity="${limits.split(' ')[3]}"/>
      </xacro:if>
    </joint>
  </xacro:macro>

  <!-- Create torso -->
  <xacro:generic_link name="torso" mass="${torso_mass}"
                      geometry_type="box" size="0.3 0.2 0.5"
                      material_color="0.5 0.5 0.5 1.0"/>

  <!-- Create head -->
  <xacro:generic_link name="head" mass="2.0"
                      geometry_type="sphere" size="0.1"
                      material_color="1.0 1.0 1.0 1.0"/>

  <xacro:generic_joint name="neck_joint" type="revolute"
                       parent="torso" child="head"
                       xyz="0 0 0.35" rpy="0 0 0"
                       axis="0 1 0"
                       limits="-0.5 0.5 20.0 1.0"/>

  <!-- Macro for creating an arm -->
  <xacro:macro name="humanoid_arm" params="side">
    <xacro:generic_link name="${side}_upper_arm" mass="${arm_mass}"
                        geometry_type="cylinder" size="0.04 0.3"
                        material_color="0.0 0.0 1.0 1.0"/>

    <xacro:generic_joint name="${side}_shoulder_joint" type="revolute"
                         parent="torso" child="${side}_upper_arm"
                         xyz="0.15 ${'0.1' if side == 'left' else '-0.1'} 0.1" rpy="0 0 0"
                         axis="0 1 0"
                         limits="-1.57 1.57 50.0 1.0"/>

    <xacro:generic_link name="${side}_lower_arm" mass="0.5"
                        geometry_type="cylinder" size="0.03 0.25"
                        material_color="0.0 0.0 1.0 1.0"/>

    <xacro:generic_joint name="${side}_elbow_joint" type="revolute"
                         parent="${side}_upper_arm" child="${side}_lower_arm"
                         xyz="0 0 -0.3" rpy="0 0 0"
                         axis="0 1 0"
                         limits="-1.57 1.57 30.0 1.0"/>
  </xacro:macro>

  <!-- Create both arms -->
  <xacro:humanoid_arm side="left"/>
  <xacro:humanoid_arm side="right"/>

  <!-- Macro for creating a leg -->
  <xacro:macro name="humanoid_leg" params="side">
    <xacro:generic_link name="${side}_hip" mass="1.0"
                        geometry_type="cylinder" size="0.05 0.1"
                        material_color="1.0 0.0 0.0 1.0"/>

    <xacro:generic_joint name="${side}_hip_joint" type="revolute"
                         parent="torso" child="${side}_hip"
                         xyz="-0.1 ${'0.05' if side == 'right' else '-0.05'} -0.25" rpy="0 0 0"
                         axis="1 0 0"
                         limits="-0.5 0.5 100.0 0.5"/>

    <xacro:generic_link name="${side}_thigh" mass="${leg_mass}"
                        geometry_type="cylinder" size="0.06 0.4"
                        material_color="1.0 0.0 0.0 1.0"/>

    <xacro:generic_joint name="${side}_knee_joint" type="revolute"
                         parent="${side}_hip" child="${side}_thigh"
                         xyz="0 0 -0.1" rpy="0 0 0"
                         axis="1 0 0"
                         limits="0 1.57 150.0 0.5"/>

    <xacro:generic_link name="${side}_shin" mass="1.5"
                        geometry_type="cylinder" size="0.05 0.4"
                        material_color="1.0 0.0 0.0 1.0"/>

    <xacro:generic_joint name="${side}_ankle_joint" type="revolute"
                         parent="${side}_thigh" child="${side}_shin"
                         xyz="0 0 -0.4" rpy="0 0 0"
                         axis="1 0 0"
                         limits="-0.5 0.5 80.0 0.5"/>
  </xacro:macro>

  <!-- Create both legs -->
  <xacro:humanoid_leg side="right"/>
  <xacro:humanoid_leg side="left"/>
</robot>
```

### 3.2 Inertial Properties and Dynamics
Proper inertial properties are crucial for realistic simulation of humanoid robots. Each link must have accurate mass, center of mass, and inertia tensor values.

```xml
<!-- Example of detailed inertial properties -->
<link name="torso_with_detailed_inertial">
  <visual>
    <geometry>
      <mesh filename="meshes/torso.stl"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <mesh filename="meshes/torso.stl"/>
    </geometry>
  </collision>
  <inertial>
    <!-- Mass in kg -->
    <mass value="8.5"/>
    <!-- Inertia tensor - calculated based on actual geometry -->
    <inertia ixx="0.15" ixy="0.001" ixz="0.002"
             iyy="0.25" iyz="0.003" izz="0.20"/>
  </inertial>
</link>
```

## 4. URDF Integration with ROS 2 Control Systems

### 4.1 Robot State Publisher
The robot_state_publisher node in ROS 2 uses URDF to publish joint states as transforms, which is essential for visualization and control.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_state_publisher')

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)  # 20 Hz

        # Store joint positions
        self.joint_positions = {}

        self.get_logger().info('Humanoid State Publisher initialized')

    def joint_state_callback(self, msg):
        """Update joint positions from joint state messages"""
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def publish_transforms(self):
        """Publish transforms based on joint positions"""
        # Example: Publish transforms for a simple humanoid
        transforms = []

        # Publish each joint transform
        for joint_name, position in self.joint_positions.items():
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'torso'
            t.child_frame_id = f'{joint_name}_link'

            # Set transform based on joint position
            # This is a simplified example - in practice, you'd use forward kinematics
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            transforms.append(t)

        # Publish all transforms
        for t in transforms:
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    state_publisher = HumanoidStatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 Joint Control with URDF Models
Integrating URDF with joint controllers allows for precise control of humanoid robot movements:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Joint command publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        # Joint positions
        self.current_joint_positions = {}
        self.target_joint_positions = {}

        # Define humanoid joint names
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint', 'left_hip_joint',
            'left_knee_joint', 'right_shoulder_joint', 'right_elbow_joint',
            'right_hip_joint', 'right_knee_joint'
        ]

        # Initialize target positions
        for joint in self.joint_names:
            self.target_joint_positions[joint] = 0.0

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.current_joint_positions[name] = position

    def control_loop(self):
        """Main control loop"""
        # Example: Simple sinusoidal movement pattern
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Update target positions with oscillating pattern
        for i, joint_name in enumerate(self.joint_names):
            # Create different oscillation patterns for different joints
            amplitude = 0.5
            frequency = 0.5
            phase_offset = i * 0.5  # Different phase for each joint

            self.target_joint_positions[joint_name] = amplitude * \
                math.sin(2 * math.pi * frequency * current_time + phase_offset)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.target_joint_positions[joint] for joint in self.joint_names]

        self.joint_cmd_pub.publish(cmd_msg)

    def move_to_pose(self, joint_positions):
        """Move to a specific joint configuration"""
        for joint_name, position in joint_positions.items():
            if joint_name in self.target_joint_positions:
                self.target_joint_positions[joint_name] = position

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Visualization and Debugging URDF Models

### 5.1 RViz Visualization
Proper URDF models can be visualized in RViz for debugging and validation:

```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2 -d `ros2 pkg prefix your_robot_description`/share/your_robot_description/rviz/robot.rviz

# Or launch with robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(find-pkg-share your_robot_description)/urdf/your_robot.urdf'
```

### 5.2 Checking URDF Validity
ROS 2 provides tools to check URDF validity:

```bash
# Check URDF syntax
check_urdf your_robot.urdf

# View URDF model structure
urdf_to_graphiz your_robot.urdf
```

## 6. Practical Example: Complete Humanoid URDF Model

Here's a complete example of a humanoid robot URDF model with all components:

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Torso (base link) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.12 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.006"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.12 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.006"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.1 0.05 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="100.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.07" length="0.45"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.07" length="0.45"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.06"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="150.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="0.05" soft_upper_limit="1.5"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.45"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.45"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.45" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="80.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.1 -0.05 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="100.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.07" length="0.45"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.07" length="0.45"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.06"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="150.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="0.05" soft_upper_limit="1.5"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.45"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.45"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.45" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="80.0" velocity="0.5"/>
    <safety_controller k_position="30.0" k_velocity="500.0"
                      soft_lower_limit="-0.4" soft_upper_limit="0.4"/>
  </joint>
</robot>
```

## 7. Exercises and Practice

Complete the following exercises to reinforce your understanding of URDF for humanoid robots:

1. [Chapter 3 Exercises](./exercises.md) - Practice problems covering URDF creation and humanoid robot modeling
2. [Chapter 3 Solutions](./solutions.md) - Complete implementations and solution guides

## 8. Summary

This chapter covered the essential concepts of URDF for humanoid robot description and control:

- URDF fundamentals and structure for humanoid robots
- Creating complex humanoid models with proper kinematic chains
- Advanced features like xacro for model reusability
- Integration with ROS 2 control systems
- Visualization and debugging techniques

The next chapter will explore implementing robotic nervous system patterns, building upon the URDF knowledge to create distributed control architectures for humanoid robots.

## 9. Further Reading

- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [xacro Documentation](http://wiki.ros.org/xacro)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [Gazebo Robot Simulation](http://gazebosim.org/tutorials?cat=build_robot)

## 10. Links to External Resources

- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [Humanoid Robot Models Repository](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [Robotics Certification Resources](https://www.ros.org/certification/)