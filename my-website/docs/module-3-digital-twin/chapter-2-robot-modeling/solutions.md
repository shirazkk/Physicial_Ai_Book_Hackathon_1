# URDF and SDF Robot Description Formats - Solutions

## Solution 2.1: URDF Fundamentals

**Sample URDF file (`simple_wheeled_robot.urdf`):**

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.2 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.2 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

**Validation command:**
```bash
check_urdf simple_wheeled_robot.urdf
```

**Expected output:** Parsing success, 3 links, 2 joints, proper kinematic tree.

## Solution 2.2: Xacro Macros Implementation

**Xacro version (`simple_wheeled_robot.xacro`):**

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix parent x_pos y_pos z_pos">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_pos} ${y_pos} ${z_pos}" rpy="${M_PI/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Use wheel macros -->
  <xacro:wheel prefix="left" parent="base_link" x_pos="0.2" y_pos="-0.2" z_pos="-0.05"/>
  <xacro:wheel prefix="right" parent="base_link" x_pos="0.2" y_pos="0.2" z_pos="-0.05"/>
</robot>
```

**Comparison:**
- Original: ~60 lines with duplicate wheel definitions
- Xacro: ~50 lines with reusable macro
- Xacro version is more maintainable and less error-prone

## Solution 2.3: Humanoid Robot Modeling

**Enhanced humanoid model with additional joints:**

```xml
<?xml version="1.0"?>
<robot name="enhanced_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="mass_body" value="10.0"/>
  <xacro:property name="mass_limb" value="2.0"/>
  <xacro:property name="mass_arm" value="1.5"/>
  <xacro:property name="body_size_x" value="0.2"/>
  <xacro:property name="body_size_y" value="0.15"/>
  <xacro:property name="body_size_z" value="0.4"/>
  <xacro:property name="limb_radius" value="0.05"/>
  <xacro:property name="limb_length" value="0.3"/>
  <xacro:property name="arm_length" value="0.25"/>

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

  <!-- Left Arm with additional joints -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="${body_size_x/2 + limb_radius} 0 ${body_size_z/4}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>
  </joint>

  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_yaw_link"/>
    <origin xyz="${body_size_x/2 + limb_radius} 0 ${body_size_z/4}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_shoulder_yaw_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_roll" type="revolute">
    <parent link="left_shoulder_yaw_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="${arm_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${arm_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_arm}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="${arm_length}" radius="${limb_radius}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${arm_length}" radius="${limb_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_arm}"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wrist_pitch" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_wrist_link"/>
    <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="5.0" velocity="1.0"/>
  </joint>

  <link name="left_wrist_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Similar joints for right arm (not shown for brevity) -->
  <!-- Left leg with ankle joints -->
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
    <origin xyz="0 0 ${-limb_length}" rpy="0 0 0"/>
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

  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 ${-limb_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="15.0" velocity="1.0"/>
  </joint>

  <joint name="left_ankle_roll" type="revolute">
    <parent link="left_foot"/>
    <child link="left_foot_tip"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="15.0" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="left_foot_tip">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

**Degrees of Freedom:**
- Left arm: 5 DOF (shoulder pitch, yaw, roll; elbow; wrist pitch)
- Right arm: 5 DOF (symmetrical to left)
- Left leg: 4 DOF (hip pitch, knee, ankle pitch, ankle roll)
- Right leg: 4 DOF (symmetrical to left)
- Neck: 1 DOF
- Total: 19 DOF

## Solution 2.4: Inertial Properties Estimation

**Example calculations:**

For torso (box: 0.2×0.15×0.4 m, density 2700 kg/m³):
- Volume = 0.2 × 0.15 × 0.4 = 0.012 m³
- Mass = 0.012 × 2700 = 32.4 kg
- For a box: Ixx = 1/12 × m × (h² + d²) = 1/12 × 32.4 × (0.15² + 0.4²) = 0.47 kg⋅m²
- Iyy = 1/12 × 32.4 × (0.2² + 0.4²) = 0.54 kg⋅m²
- Izz = 1/12 × 32.4 × (0.2² + 0.15²) = 0.18 kg⋅m²

For upper arm (cylinder: r=0.05, l=0.25 m):
- Volume = π × r² × l = 3.14159 × 0.05² × 0.25 = 0.00196 m³
- Mass = 0.00196 × 2700 = 5.29 kg
- For cylinder about central axis: Izz = ½ × m × r² = 0.5 × 5.29 × 0.05² = 0.0066 kg⋅m²
- For cylinder about perpendicular axis: Ixx = Iyy = 1/12 × m × (3r² + l²) = 1/12 × 5.29 × (3×0.05² + 0.25²) = 0.029 kg⋅m²

For thigh (cylinder: r=0.05, l=0.3 m):
- Volume = 3.14159 × 0.05² × 0.3 = 0.00236 m³
- Mass = 0.00236 × 2700 = 6.37 kg
- Izz = 0.5 × 6.37 × 0.05² = 0.0079 kg⋅m²
- Ixx = Iyy = 1/12 × 6.37 × (3×0.05² + 0.3²) = 0.049 kg⋅m²

## Solution 2.5: URDF to SDF Conversion and Enhancement

**SDF conversion with sensors:**

```xml
<sdf version="1.7">
  <model name="humanoid_with_sensors">
    <pose>0 0 0.5 0 0 0</pose>

    <!-- Include the URDF model -->
    <include>
      <uri>model://humanoid.urdf</uri>
    </include>

    <!-- IMU Sensor -->
    <link name="imu_link">
      <pose>0 0 0.1 0 0 0</pose>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.02</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.02</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.02</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <!-- Camera Sensor -->
    <link name="camera_link">
      <pose>0.05 0 0.05 0 0 0</pose>
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <!-- LiDAR Sensor -->
    <link name="lidar_link">
      <pose>0.1 0 0.1 0 0 0</pose>
      <sensor name="lidar" type="ray">
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>true</always_on>
        <update_rate>10</update_rate>
      </sensor>
    </link>

    <!-- Physics properties -->
    <link name="physical_body">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.15 0.4</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.5</mu>
              <mu2>0.5</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+6</kp>
              <kd>1e+3</kd>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
```

## Solution 2.6: Validation and Debugging

**Common URDF errors and fixes:**

1. **Invalid joint limits:**
   ```xml
   <!-- WRONG -->
   <limit lower="2.0" upper="1.0" effort="10" velocity="1.0"/>

   <!-- CORRECT -->
   <limit lower="1.0" upper="2.0" effort="10" velocity="1.0"/>
   ```

2. **Negative mass values:**
   ```xml
   <!-- WRONG -->
   <mass value="-1.0"/>

   <!-- CORRECT -->
   <mass value="1.0"/>
   ```

3. **Non-positive definite inertia matrix:**
   ```xml
   <!-- WRONG -->
   <inertia ixx="-0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>

   <!-- CORRECT -->
   <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
   ```

4. **Kinematic loops:**
   ```xml
   <!-- WRONG - creates a loop -->
   <joint name="a_to_b" type="fixed">
     <parent link="link_a"/>
     <child link="link_b"/>
   </joint>
   <joint name="b_to_a" type="fixed">
     <parent link="link_b"/>
     <child link="link_a"/>
   </joint>

   <!-- CORRECT - tree structure -->
   <joint name="a_to_b" type="fixed">
     <parent link="link_a"/>
     <child link="link_b"/>
   </joint>
   ```

**Validation checklist:**
- [ ] All links have visual, collision, and inertial properties
- [ ] All joints have valid parent-child relationships
- [ ] Joint limits are properly ordered (lower < upper)
- [ ] Mass values are positive
- [ ] Inertia values form positive definite matrix
- [ ] No kinematic loops in the model
- [ ] Proper root link exists

## Solution 2.7: Multi-Link Chain Optimization

**Performance optimization techniques:**

1. **Simplified collision geometries:**
   ```xml
   <!-- Instead of complex mesh -->
   <collision>
     <geometry>
       <mesh filename="complex_robot.dae"/>
     </geometry>
   </collision>

   <!-- Use simplified primitive -->
   <collision>
     <geometry>
       <cylinder radius="0.1" length="0.5"/>
     </geometry>
   </collision>
   ```

2. **Physics parameter adjustment:**
   ```xml
   <physics type='ode'>
     <max_step_size>0.01</max_step_size>  <!-- Larger for better performance -->
     <real_time_update_rate>100.0</real_time_update_rate>  <!-- Lower for better performance -->
     <gravity>0 0 -9.8</gravity>
   </physics>
   ```

3. **Reduced visual detail:**
   ```xml
   <!-- Use simple shapes for visual -->
   <visual>
     <geometry>
       <cylinder radius="0.05" length="0.3"/>
     </geometry>
   </visual>
   ```

**Performance metrics to monitor:**
- Simulation update rate
- CPU usage
- Memory consumption
- Physics solver convergence time

---

## Back to Chapter Contents
Return to [Chapter 2 Content](./content.md) | Continue to [Chapter 3](../chapter-3-sensor-simulation/content.md)