# URDF and SDF Robot Description Formats

## Learning Objectives
By the end of this chapter, readers will be able to:
- Understand the differences between URDF and SDF formats for robot description
- Create accurate robot models using URDF with proper joint constraints and physical properties
- Apply Xacro macros to simplify complex robot descriptions
- Validate URDF models using standard tools
- Extend robot models with SDF-specific features for simulation

## Prerequisites
- Completion of Module 1: Foundations of Physical AI & Humanoid Robotics
- Completion of Module 2: Robotic Nervous System
- Chapter 1: Gazebo Simulation Environment Setup
- Understanding of coordinate systems and transformations
- Basic knowledge of robot kinematics

## Introduction

Robot description formats are fundamental to robotics development, serving as blueprints that define a robot's physical structure, kinematic properties, and dynamic characteristics. In the ROS ecosystem, Unified Robot Description Format (URDF) is the standard for representing robot models, while Simulation Description Format (SDF) extends this concept specifically for simulation environments.

This chapter explores both formats in depth, focusing on how to create accurate robot models with proper physical properties that enable realistic simulation and control. We'll examine best practices for humanoid robot modeling, emphasizing the importance of accurate inertial properties, joint constraints, and geometric representations.

## 1. Understanding URDF (Unified Robot Description Format)

### 1.1 URDF Fundamentals

URDF is an XML-based format that describes robot models in terms of links, joints, and their relationships. A typical URDF model consists of:

- **Links**: Rigid bodies with mass, inertia, and geometric properties
- **Joints**: Connections between links with defined motion constraints
- **Materials**: Visual appearance definitions
- **Transmissions**: Actuator interface specifications
- **Gazebo plugins**: Simulation-specific extensions

Here's a basic URDF structure:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Additional links and joints would follow -->
</robot>
```

### 1.2 Links and Their Properties

Links represent rigid bodies in the robot model. Each link should include:

- **Visual**: Defines how the link appears in visualization tools
- **Collision**: Defines the collision geometry for physics simulation
- **Inertial**: Specifies mass properties for dynamic simulation

**Visual Properties:**
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
    <!-- Other options: cylinder, sphere, mesh -->
  </geometry>
  <material name="red_material"/>
</visual>
```

**Collision Properties:**
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

**Inertial Properties:**
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="0.5"/>
  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
</inertial>
```

### 1.3 Joint Types and Constraints

Joints connect links and define their relative motion. URDF supports several joint types:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement allowed
- **floating**: 6DOF motion (rarely used)
- **planar**: Motion constrained to a plane

Example joint definition:
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## 2. Understanding SDF (Simulation Description Format)

### 2.1 SDF Fundamentals

While URDF is primarily used for robot description in ROS, SDF is specifically designed for Gazebo simulation. SDF can include URDF models but also provides additional simulation-specific features:

- **Plugins**: Custom simulation behaviors
- **Sensors**: Detailed sensor configurations
- **Physics parameters**: Advanced physics settings
- **Visual effects**: Special rendering properties

Basic SDF structure for a robot model:
```xml
<sdf version="1.7">
  <model name="simple_robot">
    <pose>0 0 0.5 0 0 0</pose>

    <!-- Links -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>

      <!-- Visual properties -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </visual>

      <!-- Collision properties -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>

      <!-- Inertial properties -->
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.001</iyy>
          <iyz>0.0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Joints would follow -->
  </model>
</sdf>
```

### 2.2 SDF Extensions for Simulation

SDF provides several extensions that are particularly useful for simulation:

**SDF Models with Plugins:**
```xml
<model name="sensor_equipped_robot">
  <!-- Include URDF model -->
  <include>
    <uri>model://robot_model.urdf</uri>
  </include>

  <!-- Add simulation plugins -->
  <plugin name="imu_sensor" filename="libgazebo_ros_imu.so">
    <topic>imu/data</topic>
    <serviceName>imu/service</serviceName>
  </plugin>

  <!-- Custom controller plugin -->
  <plugin name="joint_controller" filename="libcustom_controller.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</model>
```

## 3. Differences Between URDF and SDF

| Aspect | URDF | SDF |
|--------|------|-----|
| **Primary Use** | Robot description in ROS | Simulation in Gazebo |
| **Format** | XML | XML |
| **Geometry Support** | Box, Cylinder, Sphere, Mesh | Same + more advanced shapes |
| **Plugin Support** | Through Gazebo tags | Native plugin support |
| **Sensor Definitions** | Limited | Full sensor support |
| **Physics Extensions** | Basic | Advanced physics options |
| **Visualization** | Basic | Advanced rendering options |

### 3.1 When to Use Each Format

- **Use URDF when:**
  - Defining robot structure for ROS
  - Working with MoveIt! or other ROS-based planning tools
  - Need compatibility with standard ROS tools
  - Describing robot for control purposes

- **Use SDF when:**
  - Creating simulation-specific models
  - Adding sensors and plugins
  - Defining world objects in Gazebo
  - Need advanced simulation features

## 4. Best Practices for URDF with Xacro Macros

### 4.1 Introduction to Xacro

Xacro (XML Macros) extends URDF with features like variables, math expressions, and macros, making complex robot descriptions more manageable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_width" value="0.02" />

  <!-- Define a macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy *joint_origin *geometry">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <xacro:insert_block name="joint_origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <xacro:insert_block name="geometry"/>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link">
    <origin xyz="0.2 0.1 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
    </geometry>
  </xacro:wheel>
</robot>
```

### 4.2 Advanced Xacro Techniques

**Conditional Statements:**
```xml
<xacro:arg name="has_laser" default="false"/>

<xacro:if value="$(arg has_laser)">
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
    </visual>
  </link>
</xacro:if>
```

**Mathematical Expressions:**
```xml
<xacro:property name="link_length" value="0.3" />
<xacro:property name="half_length" value="${link_length / 2.0}" />

<link name="arm_link">
  <visual>
    <origin xyz="0 0 ${half_length}" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.05 ${link_length}"/>
    </geometry>
  </visual>
</link>
```

## 5. Creating Example Humanoid Robot URDF Model

Let's create a simplified humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="mass_body" value="10.0"/>
  <xacro:property name="mass_limb" value="2.0"/>
  <xacro:property name="body_size_x" value="0.2"/>
  <xacro:property name="body_size_y" value="0.15"/>
  <xacro:property name="body_size_z" value="0.4"/>
  <xacro:property name="limb_radius" value="0.05"/>
  <xacro:property name="limb_length" value="0.3"/>

  <!-- Material definitions -->
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link - pelvis/torso -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${body_size_x} ${body_size_y} ${body_size_z}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${body_size_x} ${body_size_y} ${body_size_z}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_body}"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 ${body_size_z/2 + 0.075}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.075"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.075"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="${body_size_x/2 + limb_radius} 0 ${body_size_z/4}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 ${-limb_length/2}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm (mirror of left) -->
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="${-(body_size_x/2 + limb_radius)} 0 ${body_size_z/4}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 ${-limb_length/2}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="${body_size_x/4} 0 ${-body_size_z/2 - limb_radius}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/4}" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 ${-limb_length/2}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="${-body_size_x/4} 0 ${-body_size_z/2 - limb_radius}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/4}" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 ${-limb_length/2}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${limb_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_limb}"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## 6. Validating URDF Models

### 6.1 Using check_urdf Tool

The `check_urdf` tool validates URDF syntax and structure:

```bash
# Install the tool if not available
sudo apt-get install ros-humble-urdfdom-tools

# Validate your URDF file
check_urdf /path/to/your/robot.urdf
```

Expected output includes:
- Parsing results
- Number of links and joints
- Joint types and limits
- Kinematic tree structure

### 6.2 Using urdf_tutorial

Visualize your URDF model:
```bash
# Launch the model visualizer
roslaunch urdf_tutorial display.launch model:=/path/to/your/robot.urdf
```

### 6.3 Common Validation Checks

1. **Mass Properties**: Ensure all links have positive mass values
2. **Inertia Values**: Verify inertia matrices are physically plausible
3. **Joint Limits**: Check that joint limits are reasonable for the robot
4. **Kinematic Chain**: Verify the model has a proper tree structure (no loops)
5. **Collision Geometries**: Ensure collision geometries are properly defined

## 7. Joint Constraints and Physical Properties

### 7.1 Joint Limit Considerations

Proper joint limits are crucial for realistic robot behavior:

```xml
<!-- Realistic shoulder joint limits -->
<joint name="shoulder_pitch" type="revolute">
  <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50.0" velocity="2.0"/>
</joint>

<!-- Elbow joint with physical constraints -->
<joint name="elbow_joint" type="revolute">
  <limit lower="0" upper="${M_PI*0.9}" effort="30.0" velocity="2.5"/>
</joint>
```

### 7.2 Inertial Parameter Estimation

Accurate inertial properties are essential for dynamic simulation:

- **Mass**: Use CAD models or estimate based on material density
- **Center of Mass**: Should be at the geometric center for symmetric objects
- **Inertia Tensor**: For a solid cylinder rotating about its central axis: Izz = ½mr²

### 7.3 Collision vs Visual Geometry

- **Visual Geometry**: Defines how the robot looks (can be complex)
- **Collision Geometry**: Defines physics interactions (should be simpler)

```xml
<link name="complex_visual_link">
  <!-- Detailed visual mesh -->
  <visual>
    <geometry>
      <mesh filename="complex_shape.dae"/>
    </geometry>
  </visual>

  <!-- Simplified collision geometry -->
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.2"/>
    </geometry>
  </collision>
</link>
```

## 8. SDF Extensions for Simulation-Specific Features

### 8.1 Adding Sensors to URDF Models

While URDF doesn't natively support sensors, they can be added through Gazebo tags:

```xml
<link name="sensor_mount">
  <!-- Standard URDF elements -->
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>

  <!-- Gazebo-specific sensor definition -->
  <gazebo reference="sensor_mount">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
</link>
```

### 8.2 Physics Properties in SDF

Advanced physics settings for more realistic simulation:

```xml
<link name="physical_link">
  <!-- Standard elements -->
  <collision name="collision">
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <!-- Surface properties for realistic physics -->
    <surface>
      <friction>
        <ode>
          <mu>0.5</mu>
          <mu2>0.5</mu2>
          <fdir1>0 0 0</fdir1>
          <slip1>0</slip1>
          <slip2>0</slip2>
        </ode>
      </friction>
      <contact>
        <ode>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+13</kp>
          <kd>1.0</kd>
          <max_vel>0.01</max_vel>
          <min_depth>0</min_depth>
        </ode>
      </contact>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
        <threshold>100000</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

## 9. Best Practices for Humanoid Robot Modeling

### 9.1 Proportional Design

Humanoid robots should follow realistic proportions:
- Leg length: ~40-45% of total height
- Torso length: ~30-35% of total height
- Arm length: ~35-40% of total height
- Head size: ~8-10% of total height

### 9.2 Center of Mass Considerations

For stable bipedal locomotion:
- Keep overall center of mass low
- Distribute mass appropriately in limbs
- Consider the effect of actuator placement

### 9.3 Joint Configuration for Locomotion

Key joints for humanoid locomotion:
- 6DOF at feet for balance control
- Hip joints with pitch, roll, and yaw
- Knee joints with appropriate flexion limits
- Ankle joints for fine balance adjustments

## 10. Summary

This chapter covered the fundamentals of robot description formats, focusing on:

- URDF for robot structure definition with links, joints, and properties
- SDF for simulation-specific enhancements and features
- Differences between the two formats and appropriate use cases
- Xacro macros for managing complex robot descriptions
- Proper validation techniques for URDF models
- Joint constraints and physical properties for realistic simulation
- SDF extensions for sensors and advanced physics

With accurate robot models, you can now proceed to configure realistic physics simulations and sensor systems, which will be covered in the next chapter.

## 11. Exercises and Practice

Complete the following exercises to reinforce your understanding of URDF and SDF robot description formats:

1. [Chapter 2 Exercises](./exercises.md) - Practice problems covering URDF modeling, Xacro macros, and SDF extensions
2. [Chapter 2 Solutions](./solutions.md) - Complete implementations and solution guides