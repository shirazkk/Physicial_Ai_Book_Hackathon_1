# Gazebo Simulation Environment Setup - Solutions

## Solution 1.1: Gazebo Installation and Verification

**Steps:**
1. Installation command: `curl -sSL http://get.gazebosim.org | sh` followed by `sudo apt-get install gz-harmonic`
2. Verification: `gz --version` should return the Gazebo Garden version information
3. Launch: `gz sim` opens the Gazebo GUI
4. Main components include:
   - 3D visualization window showing the simulated world
   - Entity tree displaying world objects
   - Toolbar with simulation controls (play, pause, reset)
   - Menu bar with file and edit options

**Expected outcome:** Successful installation with version information displayed.

## Solution 1.2: Basic World Creation

**Sample world file (`custom_world.sdf`):**

```xml
<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='moon_environment'>
    <!-- Moon-like Physics Configuration (1/6th Earth gravity) -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0.5</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -1.63</gravity>  <!-- Moon gravity: 9.8 / 6 ≈ 1.63 -->
    </physics>

    <!-- Scene Configuration -->
    <scene>
      <ambient>0.3 0.3 0.3 1</ambient>
      <background>0.1 0.1 0.1 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- Lighting -->
    <light name='sun' type='directional'>
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Optional: Add a simple object to observe the reduced gravity -->
    <model name='dropped_ball'>
      <pose>0 0 5 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0004</ixx>
            <iyy>0.0004</iyy>
            <izz>0.0004</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

**Observations:** Objects will fall more slowly due to reduced gravity, and the simulation will run at half real-time speed.

## Solution 1.3: Robot Spawning Practice

**Command Line Method:**
```bash
gz model -f simple_box_robot.sdf -m my_robot
```

**Python Script (`spawn_robot.py`):**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
import sys

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

    def spawn_robot(self, robot_xml, robot_name, x=0.0, y=0.0, z=0.5):
        request = SpawnEntity.Request()
        request.xml = robot_xml
        request.name = robot_name

        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z
        request.initial_pose = initial_pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {robot_name}')
        else:
            self.get_logger().error(f'Failed to spawn {robot_name}')

def main(args=None):
    rclpy.init(args=args)

    # Robot XML definition (same as in content.md)
    robot_xml = '''<sdf version='1.7'>
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
            </inertia>
          </inertial>
        </link>
      </model>
    </sdf>'''

    spawner = RobotSpawner()
    spawner.spawn_robot(robot_xml, 'programmatic_robot', x=2.0, y=0.0, z=0.5)

    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Comparison:**
- Command line: Faster for quick testing, no programming required
- Programmatic: Better for automation, dynamic spawning, integration with ROS 2 nodes

## Solution 1.4: World Customization Challenge

**Sample world file with obstacle course:**

```xml
<?xml version="1.0" ?>
<sdf version='1.7'>
  <world name='humanoid_obstacle_course'>
    <!-- Physics Configuration -->
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
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Ramp -->
    <model name='training_ramp'>
      <pose>5 0 0 0 0.2 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2 1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2 1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Stepping stones -->
    <model name='stepping_stone_1'>
      <pose>8 0 0.05 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.2</radius>
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

    <!-- Obstacles -->
    <model name='wall_obstacle'>
      <pose>11 0 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.2 2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.2 2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Solution 1.5: Physics Validation Experiment

**Method:**
1. Create a simple sphere model dropped from a known height (e.g., 5 meters)
2. Record the simulation time when the sphere hits the ground
3. Use the formula: h = ½gt², so g = 2h/t²
4. Expected result: ~9.8 m/s² for Earth gravity

**Sample experiment code:**
```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
import time

class PhysicsValidator(Node):
    def __init__(self):
        super().__init__('physics_validator')
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.listener_callback,
            10)
        self.start_time = time.time()
        self.object_dropped = False

    def listener_callback(self, msg):
        # Look for our dropped ball in the link states
        for i, name in enumerate(msg.name):
            if 'dropped_ball::link' in name:
                z_pos = msg.pose[i].position.z
                if z_pos <= 0.1 and not self.object_dropped:  # Hit the ground
                    end_time = time.time()
                    fall_time = end_time - self.start_time
                    initial_height = 5.0  # meters

                    # Calculate g = 2h/t^2
                    calculated_g = (2 * initial_height) / (fall_time ** 2)

                    self.get_logger().info(f'Fall time: {fall_time:.2f}s')
                    self.get_logger().info(f'Calculated gravity: {calculated_g:.2f} m/s²')
                    self.get_logger().info(f'Theoretical gravity: 9.8 m/s²')
                    self.get_logger().info(f'Difference: {abs(9.8 - calculated_g):.2f} m/s²')

                    self.object_dropped = True
```

## Solution 1.6: Performance Optimization

**Performance monitoring:**
- Use Gazebo's statistics plugin to monitor frame rate and CPU usage
- Reduce mesh complexity where possible
- Lower physics update rates for less critical simulations
- Minimize the number of active sensors
- Use simpler collision geometries (boxes instead of complex meshes)

**Optimization techniques:**
- Replace complex meshes with primitive shapes for collision detection
- Reduce the number of joints and links in robot models
- Use lower resolution textures
- Implement level-of-detail (LOD) for distant objects

---

## Back to Chapter Contents
Return to [Chapter 1 Content](./content.md) | Continue to [Chapter 2](../chapter-2-robot-modeling/content.md)