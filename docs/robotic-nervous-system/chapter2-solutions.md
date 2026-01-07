# Chapter 2 Solutions: Bridging Python-based AI Agents to Robot Controllers

## Solution for Exercise 1: Basic AI Agent Implementation

### Complete AI Agent Node Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class AIAgentNode(Node):

    def __init__(self):
        super().__init__('ai_agent_node')

        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.laser_data = None
        self.obstacle_threshold = 1.0  # meters

        self.get_logger().info('AI Agent Node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        # Convert to numpy array for easier processing
        ranges = np.array(msg.ranges)
        # Filter out invalid measurements (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.laser_data = {
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def control_loop(self):
        """Main control loop for the AI agent"""
        if self.laser_data is None:
            return

        # Get the minimum distance to any obstacle
        min_distance = self.laser_data['min_distance']

        # Create velocity command
        cmd = Twist()

        if min_distance < self.obstacle_threshold:
            # Obstacle detected - implement avoidance behavior
            # Find the angle of the closest obstacle
            ranges = self.laser_data['ranges']
            angles = np.linspace(
                self.laser_data['angle_min'],
                self.laser_data['angle_max'],
                len(ranges)
            )

            closest_idx = np.argmin(ranges)
            closest_angle = angles[closest_idx]

            # Turn away from the obstacle
            if closest_angle < 0:
                # Obstacle on the left - turn right
                cmd.linear.x = 0.2  # Slow forward motion
                cmd.angular.z = -0.5  # Turn right
            else:
                # Obstacle on the right - turn left
                cmd.linear.x = 0.2  # Slow forward motion
                cmd.angular.z = 0.5  # Turn left
        else:
            # No immediate obstacles - move forward
            cmd.linear.x = 0.5  # Move forward at medium speed
            cmd.angular.z = 0.0  # No turning

        # Publish the command
        self.cmd_vel_pub.publish(cmd)

        # Log the command
        self.get_logger().info(
            f'Velocity: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, '
            f'closest_obstacle={min_distance:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run
1. Save the code as `ai_agent_node.py`
2. Make sure your ROS 2 environment is sourced
3. Run the node: `python3 ai_agent_node.py`
4. Test with a simulated robot that publishes laser scan data and accepts velocity commands

## Solution for Exercise 2: Multi-Sensor Integration

### Complete Multi-Sensor AI Agent Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from collections import deque
import math

class MultiSensorAINode(Node):

    def __init__(self):
        super().__init__('multi_sensor_ai_node')

        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers for multiple sensors
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Data storage with timestamps for synchronization
        self.scan_data = None
        self.odom_data = None
        self.imu_data = None

        # Store recent sensor data for fusion
        self.scan_history = deque(maxlen=10)
        self.odom_history = deque(maxlen=10)
        self.imu_history = deque(maxlen=10)

        # Fused state
        self.fused_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'obstacle_distances': [],
            'min_obstacle_distance': float('inf'),
            'heading': 0.0  # Robot's heading in radians
        }

        self.get_logger().info('Multi-Sensor AI Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        timestamp = self.get_clock().now().nanoseconds
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.scan_data = {
                'timestamp': timestamp,
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }
            # Add to history
            self.scan_history.append(self.scan_data)

    def odom_callback(self, msg):
        """Process odometry data"""
        timestamp = self.get_clock().now().nanoseconds
        self.odom_data = {
            'timestamp': timestamp,
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        }
        # Add to history
        self.odom_history.append(self.odom_data)

    def imu_callback(self, msg):
        """Process IMU data"""
        timestamp = self.get_clock().now().nanoseconds
        self.imu_data = {
            'timestamp': timestamp,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        # Add to history
        self.imu_history.append(self.imu_data)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def sensor_fusion(self):
        """Fuse data from multiple sensors to create comprehensive state"""
        # Get the most recent data from each sensor
        if not all([self.scan_data, self.odom_data, self.imu_data]):
            return

        # Use the most recent timestamp as reference
        ref_timestamp = self.get_clock().now().nanoseconds

        # Fuse position and orientation
        # Prioritize odometry for position, IMU for orientation (more accurate)
        fused_position = self.odom_data['position']

        # Extract orientation from IMU (usually more accurate than odometry)
        imu_orientation = self.imu_data['orientation']
        roll, pitch, yaw = self.quaternion_to_euler(
            imu_orientation['x'], imu_orientation['y'],
            imu_orientation['z'], imu_orientation['w']
        )

        # Fuse velocity information
        linear_vel = self.odom_data['linear_velocity']
        angular_vel = self.imu_data['angular_velocity']  # IMU typically more accurate for angular rates

        # Update fused state
        self.fused_state.update({
            'position': fused_position,
            'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel,
            'heading': yaw,  # Current heading in radians
            'min_obstacle_distance': self.scan_data['min_distance'],
            'obstacle_distances': self.scan_data['ranges'].tolist()
        })

    def control_loop(self):
        """Main control loop with multi-sensor fusion"""
        # Perform sensor fusion
        self.sensor_fusion()

        if self.fused_state['min_obstacle_distance'] == float('inf'):
            return

        # Create velocity command based on fused state
        cmd = Twist()

        min_distance = self.fused_state['min_obstacle_distance']
        obstacle_threshold = 1.0  # meters

        if min_distance < obstacle_threshold:
            # Obstacle detected - implement avoidance behavior
            # Use laser scan data to determine obstacle direction
            if self.scan_data:
                ranges = self.scan_data['ranges']
                angles = np.linspace(
                    self.scan_data['angle_min'],
                    self.scan_data['angle_max'],
                    len(ranges)
                )

                closest_idx = np.argmin(ranges)
                closest_angle = angles[closest_idx]

                # Adjust heading based on obstacle position
                if closest_angle < 0:
                    # Obstacle on the left - turn right
                    cmd.linear.x = max(0.1, min_distance * 0.3)  # Slow forward motion, slower when closer to obstacles
                    cmd.angular.z = -max(0.3, abs(closest_angle) * 0.5)  # Turn right more aggressively when closer to obstacles
                else:
                    # Obstacle on the right - turn left
                    cmd.linear.x = max(0.1, min_distance * 0.3)  # Slow forward motion
                    cmd.angular.z = max(0.3, abs(closest_angle) * 0.5)  # Turn left more aggressively when closer to obstacles
        else:
            # No immediate obstacles - move forward with heading correction
            cmd.linear.x = 0.5  # Move forward at medium speed
            cmd.angular.z = -self.fused_state['heading'] * 0.5  # Correct heading towards desired direction

        # Publish the command
        self.cmd_vel_pub.publish(cmd)

        # Log the command and fused state
        self.get_logger().info(
            f'Fused AI Agent - Vel: lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}, '
            f'Pos:({self.fused_state["position"]["x"]:.2f},{self.fused_state["position"]["y"]:.2f}), '
            f'Heading:{math.degrees(self.fused_state["heading"]):.1f}°, '
            f'Obst:{self.fused_state["min_obstacle_distance"]:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    multi_sensor_node = MultiSensorAINode()

    try:
        rclpy.spin(multi_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        multi_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 3: Behavior-Based Control

### Complete Behavior-Based AI Agent Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum
import numpy as np
import math

class BehaviorState(Enum):
    WANDERING = 1
    OBSTACLE_AVOIDANCE = 2
    GOAL_SEEKING = 3

class BehaviorBasedAINode(Node):

    def __init__(self):
        super().__init__('behavior_based_ai_node')

        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.laser_data = None
        self.current_behavior = BehaviorState.WANDERING
        self.previous_behavior = BehaviorState.WANDERING

        # Behavior-specific parameters
        self.obstacle_threshold = 1.0  # meters
        self.danger_threshold = 0.5    # meters (immediate danger)

        # Goal-seeking parameters
        self.goal_x = 5.0  # Target x-coordinate
        self.goal_y = 5.0  # Target y-coordinate
        self.current_x = 0.0  # Current position (would come from odometry in real implementation)
        self.current_y = 0.0

        # Behavior priorities (higher number = higher priority)
        self.behavior_priorities = {
            BehaviorState.OBSTACLE_AVOIDANCE: 3,
            BehaviorState.GOAL_SEEKING: 2,
            BehaviorState.WANDERING: 1
        }

        self.get_logger().info('Behavior-Based AI Node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.laser_data = {
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def wandering_behavior(self):
        """Wandering behavior - random exploration"""
        cmd = Twist()

        # Random wandering with some forward momentum
        cmd.linear.x = 0.4 + np.random.uniform(-0.1, 0.1)  # Mostly forward with some variation
        cmd.angular.z = np.random.uniform(-0.3, 0.3)       # Random turns

        return cmd

    def obstacle_avoidance_behavior(self):
        """Obstacle avoidance behavior"""
        if not self.laser_data:
            return Twist()  # Return stop command if no data

        cmd = Twist()

        # Find the closest obstacle
        ranges = self.laser_data['ranges']
        angles = np.linspace(
            self.laser_data['angle_min'],
            self.laser_data['angle_max'],
            len(ranges)
        )

        closest_idx = np.argmin(ranges)
        closest_distance = ranges[closest_idx]
        closest_angle = angles[closest_idx]

        # If in immediate danger, prioritize escape
        if closest_distance < self.danger_threshold:
            # Emergency maneuver
            cmd.linear.x = -0.3  # Back up
            cmd.angular.z = 0.8 if closest_angle < 0 else -0.8  # Sharp turn away
        else:
            # Standard avoidance
            if closest_angle < 0:
                # Obstacle on the left - turn right
                cmd.linear.x = max(0.1, closest_distance * 0.2)
                cmd.angular.z = -max(0.3, abs(closest_angle) * 0.5)
            else:
                # Obstacle on the right - turn left
                cmd.linear.x = max(0.1, closest_distance * 0.2)
                cmd.angular.z = max(0.3, abs(closest_angle) * 0.5)

        return cmd

    def goal_seeking_behavior(self):
        """Goal seeking behavior"""
        cmd = Twist()

        # Calculate direction to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)

        # Normalize angle to [-pi, pi]
        if angle_to_goal > math.pi:
            angle_to_goal -= 2 * math.pi
        elif angle_to_goal < -math.pi:
            angle_to_goal += 2 * math.pi

        # Set velocity based on distance to goal
        cmd.linear.x = min(0.8, max(0.2, distance_to_goal * 0.2))
        cmd.angular.z = angle_to_goal * 0.5

        return cmd

    def determine_active_behavior(self):
        """Determine which behavior should be active based on conditions"""
        if not self.laser_data:
            return BehaviorState.WANDERING

        min_distance = self.laser_data['min_distance']

        # Check for immediate danger (highest priority)
        if min_distance < self.danger_threshold:
            return BehaviorState.OBSTACLE_AVOIDANCE

        # Check for obstacle in path (high priority)
        if min_distance < self.obstacle_threshold:
            return BehaviorState.OBSTACLE_AVOIDANCE

        # Check if we have a goal and are trying to reach it
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        if distance_to_goal > 0.5:  # If we're not already at the goal
            return BehaviorState.GOAL_SEEKING

        # Default to wandering
        return BehaviorState.WANDERING

    def control_loop(self):
        """Main control loop with behavior selection"""
        # Determine which behavior should be active
        self.current_behavior = self.determine_active_behavior()

        # Execute the appropriate behavior
        cmd = Twist()

        if self.current_behavior == BehaviorState.WANDERING:
            cmd = self.wandering_behavior()
        elif self.current_behavior == BehaviorState.OBSTACLE_AVOIDANCE:
            cmd = self.obstacle_avoidance_behavior()
        elif self.current_behavior == BehaviorState.GOAL_SEEKING:
            cmd = self.goal_seeking_behavior()

        # Publish the command
        self.cmd_vel_pub.publish(cmd)

        # Log behavior transitions
        if self.current_behavior != self.previous_behavior:
            self.get_logger().info(f'Behavior transition: {self.previous_behavior.name} -> {self.current_behavior.name}')

        # Log the command and current behavior
        self.get_logger().info(
            f'Behavior: {self.current_behavior.name}, '
            f'Velocity: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}'
        )

        self.previous_behavior = self.current_behavior

def main(args=None):
    rclpy.init(args=args)
    behavior_node = BehaviorBasedAINode()

    try:
        rclpy.spin(behavior_node)
    except KeyboardInterrupt:
        pass
    finally:
        behavior_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 4: Path Planning Integration

### Complete Path Planning AI Agent Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import numpy as np
import math
from enum import Enum

class NavigationState(Enum):
    IDLE = 1
    PLANNING = 2
    FOLLOWING_PATH = 3
    OBSTACLE_AVOIDANCE = 4
    REPLANNING = 5

class PathPlanningAINode(Node):

    def __init__(self):
        super().__init__('path_planning_ai_node')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for path visualization
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/planned_path',
            10
        )

        # Publisher for current goal
        self.current_goal_pub = self.create_publisher(
            PoseStamped,
            '/current_goal',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.laser_data = None
        self.current_state = NavigationState.IDLE
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Goal and path planning
        self.goals = []  # List of (x, y) tuples
        self.current_goal_index = 0
        self.path = []   # List of waypoints
        self.current_waypoint_index = 0

        # Navigation parameters
        self.waypoint_tolerance = 0.5  # meters
        self.obstacle_threshold = 1.0  # meters
        self.replan_threshold = 0.3    # meters (if obstacle is closer, replan)

        # Path planning parameters
        self.max_path_length = 10  # maximum number of waypoints in path

        self.get_logger().info('Path Planning AI Node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.laser_data = {
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def set_goals(self, goals_list):
        """Set a list of goals to visit"""
        self.goals = goals_list
        self.current_goal_index = 0
        self.current_state = NavigationState.PLANNING
        self.get_logger().info(f'Set {len(goals_list)} goals: {goals_list}')

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """Simple straight-line path planner with intermediate waypoints"""
        # Calculate distance and direction to goal
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Already at goal
            return []

        # Create intermediate waypoints
        num_waypoints = min(self.max_path_length, max(2, int(distance / 0.5)))
        path = []

        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            wp_x = start_x + dx * t
            wp_y = start_y + dy * t
            path.append((wp_x, wp_y))

        return path

    def check_path_clear(self, path):
        """Check if the path is clear of obstacles"""
        if not self.laser_data or len(path) < 2:
            return True

        # For simplicity, check if any point on the path is too close to an obstacle
        # In a real implementation, this would involve ray casting or occupancy grid checking
        for i in range(len(path) - 1):
            start_point = path[i]
            end_point = path[i + 1]

            # Check midpoint of segment
            mid_x = (start_point[0] + end_point[0]) / 2
            mid_y = (start_point[1] + end_point[1]) / 2

            # Calculate distance from robot to this point
            dist_to_point = math.sqrt((mid_x - self.current_x)**2 + (mid_y - self.current_y)**2)

            # If the point is close to robot and we have laser data, check if it's blocked
            if dist_to_point < 2.0:  # Only check nearby points
                # This is a simplified check - in reality you'd need to transform laser data
                # to global coordinates and check for obstacles at these points
                if self.laser_data['min_distance'] < 0.5:
                    return False

        return True

    def simple_path_planner(self):
        """Execute path planning logic"""
        if not self.goals or self.current_goal_index >= len(self.goals):
            self.current_state = NavigationState.IDLE
            return

        # Get current goal
        goal_x, goal_y = self.goals[self.current_goal_index]

        # Plan path to current goal
        self.path = self.plan_path(self.current_x, self.current_y, goal_x, goal_y)
        self.current_waypoint_index = 0

        if self.path:
            self.current_state = NavigationState.FOLLOWING_PATH
            self.get_logger().info(f'Planned path to goal ({goal_x:.2f}, {goal_y:.2f}) with {len(self.path)} waypoints')

            # Visualize path
            self.visualize_path()
        else:
            self.current_state = NavigationState.IDLE
            self.get_logger().warn('Failed to plan path')

    def visualize_path(self):
        """Publish visualization marker for the planned path"""
        if not self.path:
            return

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planned_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale of the line
        marker.scale.x = 0.05  # Line width

        # Set the color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (opacity)

        # Add points to the line strip
        for x, y in self.path:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0  # Assuming 2D navigation
            marker.points.append(point)

        self.path_marker_pub.publish(marker)

    def navigate_to_waypoint(self):
        """Generate commands to navigate to the current waypoint"""
        if not self.path or self.current_waypoint_index >= len(self.path):
            return Twist()

        # Get current waypoint
        target_x, target_y = self.path[self.current_waypoint_index]

        # Calculate direction to waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_to_waypoint = math.sqrt(dx*dx + dy*dy)

        # Check if we've reached the current waypoint
        if distance_to_waypoint < self.waypoint_tolerance:
            self.current_waypoint_index += 1

            # Check if we've reached the end of the path
            if self.current_waypoint_index >= len(self.path):
                # Check if we've reached the final goal
                goal_x, goal_y = self.goals[self.current_goal_index]
                distance_to_goal = math.sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)

                if distance_to_goal < self.waypoint_tolerance:
                    # Reached current goal, move to next goal if available
                    self.current_goal_index += 1
                    if self.current_goal_index < len(self.goals):
                        self.current_state = NavigationState.PLANNING
                        self.get_logger().info(f'Reached goal {self.current_goal_index}, moving to next goal')
                    else:
                        self.current_state = NavigationState.IDLE
                        self.get_logger().info('Reached all goals!')
                else:
                    # We reached the end of the path but not the goal - replan
                    self.current_state = NavigationState.REPLANNING
            else:
                # Move to next waypoint
                target_x, target_y = self.path[self.current_waypoint_index]

        # Recalculate direction to current waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_to_waypoint = math.sqrt(dx*dx + dy*dy)
        angle_to_waypoint = math.atan2(dy, dx)

        # Normalize angle difference
        angle_diff = angle_to_waypoint - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create velocity command
        cmd = Twist()
        cmd.linear.x = min(0.8, max(0.1, distance_to_waypoint * 0.5))  # Speed based on distance
        cmd.angular.z = max(-1.0, min(1.0, angle_diff * 1.5))  # Proportional control for heading

        return cmd

    def handle_obstacle_during_navigation(self):
        """Handle obstacle detection during path following"""
        if not self.laser_data:
            return Twist()

        # Simple obstacle avoidance while trying to maintain path following
        cmd = Twist()

        # Find the closest obstacle
        ranges = self.laser_data['ranges']
        angles = np.linspace(
            self.laser_data['angle_min'],
            self.laser_data['angle_max'],
            len(ranges)
        )

        closest_idx = np.argmin(ranges)
        closest_distance = ranges[closest_idx]
        closest_angle = angles[closest_idx]

        # If obstacle is very close, prioritize avoidance
        if closest_distance < self.replan_threshold:
            # Emergency avoidance
            cmd.linear.x = -0.2  # Slow reverse
            cmd.angular.z = 0.8 if closest_angle < 0 else -0.8  # Turn away
            self.current_state = NavigationState.REPLANNING
        elif closest_distance < self.obstacle_threshold:
            # Standard avoidance while maintaining heading toward waypoint
            if closest_angle < 0:
                # Obstacle on the left - turn right gently
                cmd.linear.x = 0.3
                cmd.angular.z = -0.5
            else:
                # Obstacle on the right - turn left gently
                cmd.linear.x = 0.3
                cmd.angular.z = 0.5
        else:
            # No immediate obstacles, continue with path following
            cmd = self.navigate_to_waypoint()

        return cmd

    def control_loop(self):
        """Main control loop with path planning integration"""
        # Update robot position (in a real system, this would come from odometry)
        # For simulation purposes, we'll just keep it at origin or update based on commands

        # Execute state-specific behavior
        cmd = Twist()

        if self.current_state == NavigationState.IDLE:
            # Wait for goals to be set
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.current_state == NavigationState.PLANNING:
            self.simple_path_planner()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.current_state == NavigationState.FOLLOWING_PATH:
            if self.laser_data and self.laser_data['min_distance'] < self.obstacle_threshold:
                cmd = self.handle_obstacle_during_navigation()
            else:
                cmd = self.navigate_to_waypoint()

        elif self.current_state == NavigationState.OBSTACLE_AVOIDANCE:
            cmd = self.handle_obstacle_during_navigation()

        elif self.current_state == NavigationState.REPLANNING:
            # Stop temporarily while replanning
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # Trigger replanning in the next cycle
            self.current_state = NavigationState.PLANNING

        # Publish the command
        self.cmd_vel_pub.publish(cmd)

        # Log current state and command
        self.get_logger().info(
            f'State: {self.current_state.name}, '
            f'Velocity: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, '
            f'Goals: {self.current_goal_index}/{len(self.goals) if self.goals else 0}'
        )

def main(args=None):
    rclpy.init(args=args)
    path_planning_node = PathPlanningAINode()

    # Example: Set some goals for the robot to visit
    goals = [(2.0, 2.0), (4.0, 1.0), (5.0, 5.0), (1.0, 4.0)]
    path_planning_node.set_goals(goals)

    try:
        rclpy.spin(path_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        path_planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 5: Safety and Validation System

### Complete Safety Layer Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math

class SafetySystemNode(Node):

    def __init__(self):
        super().__init__('safety_system_node')

        # Subscriber for AI-generated commands
        self.ai_cmd_sub = self.create_subscription(
            Twist,
            '/ai_cmd_vel',
            self.ai_command_callback,
            10
        )

        # Subscriber for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for validated/safe commands
        self.safe_cmd_pub = self.create_publisher(
            Twist,
            '/safe_cmd_vel',
            10
        )

        # Publisher for emergency stop
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20Hz safety check

        # Internal state
        self.ai_command = None
        self.last_ai_command_time = None
        self.laser_data = None

        # Safety parameters
        self.max_linear_velocity = 1.0      # m/s
        self.max_angular_velocity = 1.5     # rad/s
        self.max_linear_acceleration = 2.0  # m/s^2
        self.max_angular_acceleration = 3.0 # rad/s^2
        self.obstacle_distance_threshold = 0.5  # meters
        self.collision_prediction_time = 1.0    # seconds
        self.command_timeout = 0.5              # seconds

        # Previous command for acceleration calculation
        self.prev_command = Twist()
        self.prev_command_time = None

        # Emergency state
        self.emergency_active = False
        self.last_valid_command = Twist()

        self.get_logger().info('Safety System Node initialized')

    def ai_command_callback(self, msg):
        """Receive AI-generated commands"""
        self.ai_command = msg
        self.last_ai_command_time = self.get_clock().now()

        if self.emergency_active:
            self.get_logger().warn('AI command received during emergency stop - will not process until cleared')

    def scan_callback(self, msg):
        """Process laser scan data for safety monitoring"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.laser_data = {
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def validate_velocity_limits(self, cmd):
        """Validate that velocities are within safe limits"""
        validated_cmd = Twist()

        # Linear velocity limits
        validated_cmd.linear.x = max(-self.max_linear_velocity,
                                   min(self.max_linear_velocity, cmd.linear.x))
        validated_cmd.linear.y = max(-self.max_linear_velocity,
                                   min(self.max_linear_velocity, cmd.linear.y))
        validated_cmd.linear.z = max(-self.max_linear_velocity,
                                   min(self.max_linear_velocity, cmd.linear.z))

        # Angular velocity limits
        validated_cmd.angular.x = max(-self.max_angular_velocity,
                                    min(self.max_angular_velocity, cmd.angular.x))
        validated_cmd.angular.y = max(-self.max_angular_velocity,
                                    min(self.max_angular_velocity, cmd.angular.y))
        validated_cmd.angular.z = max(-self.max_angular_velocity,
                                    min(self.max_angular_velocity, cmd.angular.z))

        return validated_cmd

    def validate_acceleration_limits(self, cmd, prev_cmd, dt):
        """Validate that accelerations are within safe limits"""
        if dt <= 0:
            return cmd

        validated_cmd = Twist()

        # Calculate desired accelerations
        linear_acc_x = (cmd.linear.x - prev_cmd.linear.x) / dt
        linear_acc_y = (cmd.linear.y - prev_cmd.linear.y) / dt
        linear_acc_z = (cmd.linear.z - prev_cmd.linear.z) / dt
        angular_acc_x = (cmd.angular.x - prev_cmd.angular.x) / dt
        angular_acc_y = (cmd.angular.y - prev_cmd.angular.y) / dt
        angular_acc_z = (cmd.angular.z - prev_cmd.angular.z) / dt

        # Limit accelerations
        linear_acc_x = max(-self.max_linear_acceleration,
                          min(self.max_linear_acceleration, linear_acc_x))
        linear_acc_y = max(-self.max_linear_acceleration,
                          min(self.max_linear_acceleration, linear_acc_y))
        linear_acc_z = max(-self.max_linear_acceleration,
                          min(self.max_linear_acceleration, linear_acc_z))
        angular_acc_x = max(-self.max_angular_acceleration,
                           min(self.max_angular_acceleration, angular_acc_x))
        angular_acc_y = max(-self.max_angular_acceleration,
                           min(self.max_angular_acceleration, angular_acc_y))
        angular_acc_z = max(-self.max_angular_acceleration,
                           min(self.max_angular_acceleration, angular_acc_z))

        # Apply acceleration limits to get final velocities
        validated_cmd.linear.x = prev_cmd.linear.x + linear_acc_x * dt
        validated_cmd.linear.y = prev_cmd.linear.y + linear_acc_y * dt
        validated_cmd.linear.z = prev_cmd.linear.z + linear_acc_z * dt
        validated_cmd.angular.x = prev_cmd.angular.x + angular_acc_x * dt
        validated_cmd.angular.y = prev_cmd.angular.y + angular_acc_y * dt
        validated_cmd.angular.z = prev_cmd.angular.z + angular_acc_z * dt

        # Also apply velocity limits after acceleration limiting
        validated_cmd = self.validate_velocity_limits(validated_cmd)

        return validated_cmd

    def predict_collision(self, cmd):
        """Predict if the command will lead to a collision"""
        if not self.laser_data:
            return False

        # Simple collision prediction: if the robot would move into space closer than threshold
        # within the prediction time

        # Calculate predicted position based on current command
        predicted_forward_distance = cmd.linear.x * self.collision_prediction_time
        predicted_turn_angle = cmd.angular.z * self.collision_prediction_time

        # Check if moving forward would hit an obstacle
        if predicted_forward_distance > 0 and self.laser_data['min_distance'] < predicted_forward_distance + 0.2:
            return True

        # For simplicity, we'll just check if there are obstacles directly ahead
        # In a real system, this would involve more complex geometric calculations
        ranges = self.laser_data['ranges']
        angles = np.linspace(
            self.laser_data['angle_min'],
            self.laser_data['angle_max'],
            len(ranges)
        )

        # Check the front sector (±30 degrees)
        front_mask = (angles >= -math.pi/6) & (angles <= math.pi/6)
        front_ranges = ranges[front_mask]

        if len(front_ranges) > 0 and np.min(front_ranges) < self.obstacle_distance_threshold:
            if cmd.linear.x > 0:  # Moving forward toward obstacles
                return True

        return False

    def safety_check(self):
        """Main safety monitoring function"""
        current_time = self.get_clock().now()

        # Check for command timeout
        if (self.last_ai_command_time and
            (current_time - self.last_ai_command_time).nanoseconds / 1e9 > self.command_timeout):
            self.get_logger().warn('AI command timeout - stopping robot')
            self.emergency_stop()
            return

        # If emergency is active, only publish stop command
        if self.emergency_active:
            stop_cmd = Twist()
            self.safe_cmd_pub.publish(stop_cmd)
            return

        # Process AI command if available
        if self.ai_command:
            # Get time delta for acceleration calculation
            dt = 0.0
            if self.prev_command_time:
                dt = (current_time - self.prev_command_time).nanoseconds / 1e9

            # Apply safety validations
            safe_cmd = self.ai_command

            # Validate velocity limits
            safe_cmd = self.validate_velocity_limits(safe_cmd)

            # Validate acceleration limits
            if dt > 0:
                safe_cmd = self.validate_acceleration_limits(safe_cmd, self.prev_command, dt)

            # Predict and avoid collisions
            if self.predict_collision(safe_cmd):
                self.get_logger().warn('Collision predicted - reducing velocity/turning')
                # Reduce linear velocity and increase turning to avoid obstacle
                safe_cmd.linear.x *= 0.3  # Reduce forward speed significantly
                # Maintain some angular control to turn away from obstacle
                if safe_cmd.linear.x > 0:  # If moving forward
                    # Add a turn component to avoid obstacle
                    if self.laser_data and len(self.laser_data['ranges']) > 0:
                        # Find the direction with most clearance
                        ranges = self.laser_data['ranges']
                        angles = np.linspace(
                            self.laser_data['angle_min'],
                            self.laser_data['angle_max'],
                            len(ranges)
                        )

                        # Find the angle with maximum range (most clearance)
                        max_range_idx = np.argmax(ranges)
                        max_range_angle = angles[max_range_idx]

                        # Adjust angular velocity to turn toward clearer direction
                        safe_cmd.angular.z = max(-self.max_angular_velocity,
                                               min(self.max_angular_velocity,
                                                   max_range_angle * 0.5))

            # Publish the validated command
            self.safe_cmd_pub.publish(safe_cmd)

            # Update previous command and time
            self.prev_command = safe_cmd
            self.prev_command_time = current_time
            self.last_valid_command = safe_cmd

            # Log safety check results
            self.get_logger().info(
                f'Safe Cmd: linear={safe_cmd.linear.x:.2f}, angular={safe_cmd.angular.z:.2f}, '
                f'Min Obs: {self.laser_data["min_distance"]:.2f}m' if self.laser_data else ''
            )

    def emergency_stop(self):
        """Trigger emergency stop"""
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED')

            # Publish emergency stop signal
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            # Publish stop command
            stop_cmd = Twist()
            self.safe_cmd_pub.publish(stop_cmd)

    def emergency_clear(self):
        """Clear emergency stop"""
        if self.emergency_active:
            self.emergency_active = False
            self.get_logger().info('Emergency stop cleared')

            # Publish emergency clear signal
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_pub.publish(emergency_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetySystemNode()

    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 6: Learning-Based Behavior (Advanced)

### Complete Learning-Based AI Agent Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import json
import os
from datetime import datetime

class LearningAINode(Node):

    def __init__(self):
        super().__init__('learning_ai_node')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.laser_data = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_run_start_time = None

        # Learning parameters
        self.learning_enabled = True
        self.experience_buffer = []  # Store experiences for learning
        self.max_experience_buffer_size = 1000

        # Navigation goal
        self.goal_x = 5.0
        self.goal_y = 5.0

        # Learnable parameters
        self.params = {
            'obstacle_weight': 1.0,      # How much to avoid obstacles
            'goal_weight': 1.0,          # How much to pursue goal
            'speed_factor': 0.5,         # Base speed factor
            'turn_sensitivity': 0.5,     # How sensitive turning is
            'collision_penalty': 5.0,    # Penalty for getting too close to obstacles
            'efficiency_bonus': 0.1      # Bonus for efficient navigation
        }

        # Performance metrics
        self.run_count = 0
        self.success_count = 0
        self.total_time = 0.0
        self.total_collisions = 0
        self.performance_history = []

        # Load saved parameters if available
        self.load_parameters()

        self.get_logger().info(f'Learning AI Node initialized with params: {self.params}')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.laser_data = {
                'ranges': valid_ranges,
                'min_distance': np.min(valid_ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def calculate_reward(self):
        """Calculate reward based on current state and action"""
        if not self.laser_data:
            return 0.0

        # Distance to goal
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        # Reward for getting closer to goal
        goal_reward = -distance_to_goal  # Negative because closer is better

        # Penalty for being too close to obstacles
        min_distance = self.laser_data['min_distance']
        obstacle_penalty = 0.0
        if min_distance < 0.5:
            obstacle_penalty = -self.params['collision_penalty'] / min_distance if min_distance > 0 else -float('inf')

        # Small bonus for moving efficiently toward goal
        efficiency_bonus = 0.0
        if hasattr(self, '_prev_distance_to_goal'):
            if distance_to_goal < self._prev_distance_to_goal:
                efficiency_bonus = self.params['efficiency_bonus']

        # Total reward
        total_reward = goal_reward + obstacle_penalty + efficiency_bonus

        # Store for next iteration
        self._prev_distance_to_goal = distance_to_goal

        return total_reward

    def simple_navigation_policy(self):
        """Basic navigation policy that uses learnable parameters"""
        if not self.laser_data:
            return Twist()

        cmd = Twist()

        # Calculate direction to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)

        # Normalize angle difference
        angle_diff = angle_to_goal - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Get obstacle information
        min_distance = self.laser_data['min_distance']
        ranges = self.laser_data['ranges']
        angles = np.linspace(
            self.laser_data['angle_min'],
            self.laser_data['angle_max'],
            len(ranges)
        )

        # Find direction of most clearance (away from obstacles)
        max_range_idx = np.argmax(ranges)
        max_range_angle = angles[max_range_idx]

        # Combine goal-seeking and obstacle-avoidance behaviors using learned weights
        if min_distance < 1.0:  # Significant obstacle detected
            # Weighted combination of goal direction and obstacle avoidance
            weighted_angle = (self.params['goal_weight'] * angle_diff +
                            self.params['obstacle_weight'] * max_range_angle) / \
                           (self.params['goal_weight'] + self.params['obstacle_weight'])
        else:
            # Mainly follow goal direction
            weighted_angle = angle_diff

        # Set velocities based on learned parameters
        cmd.linear.x = min(1.0, max(0.1, self.params['speed_factor'] * (distance_to_goal * 0.2)))
        cmd.angular.z = max(-1.0, min(1.0, weighted_angle * self.params['turn_sensitivity']))

        # Reduce speed when close to obstacles
        if min_distance < 1.5:
            cmd.linear.x *= (min_distance / 1.5)

        return cmd

    def simple_learning_algorithm(self):
        """Simple parameter learning algorithm"""
        if not self.experience_buffer or len(self.experience_buffer) < 10:
            return  # Not enough experience to learn

        # Calculate average performance over recent experiences
        recent_experiences = self.experience_buffer[-50:]  # Last 50 experiences
        avg_reward = sum(exp['reward'] for exp in recent_experiences) / len(recent_experiences)

        # Performance indicators
        distance_traveled = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
        time_elapsed = (self.get_clock().now() - self.current_run_start_time).nanoseconds / 1e9 if self.current_run_start_time else 0

        # Adjust parameters based on performance
        # This is a very simple learning approach - in practice, you'd use more sophisticated methods
        learning_rate = 0.01

        # If we're making good progress, slightly increase goal weight
        if avg_reward > -2.0 and self.params['goal_weight'] < 2.0:
            self.params['goal_weight'] += learning_rate
        elif avg_reward < -5.0 and self.params['goal_weight'] > 0.5:
            self.params['goal_weight'] -= learning_rate

        # If we're hitting obstacles frequently, increase obstacle avoidance
        if self.laser_data and self.laser_data['min_distance'] < 0.5:
            self.params['obstacle_weight'] = min(3.0, self.params['obstacle_weight'] + learning_rate * 2)
        else:
            self.params['obstacle_weight'] = max(0.5, self.params['obstacle_weight'] - learning_rate)

        # Adjust speed based on obstacle proximity history
        avg_min_dist = np.mean([exp['state']['min_distance'] for exp in recent_experiences if 'min_distance' in exp['state']])
        if avg_min_dist < 0.8:
            self.params['speed_factor'] = max(0.2, self.params['speed_factor'] - learning_rate)
        elif avg_min_dist > 1.2:
            self.params['speed_factor'] = min(1.0, self.params['speed_factor'] + learning_rate)

        self.get_logger().info(f'Learned params: {self.params}')

    def save_experience(self, state, action, reward, next_state):
        """Store experience tuple for learning"""
        experience = {
            'timestamp': self.get_clock().now().nanoseconds,
            'state': state.copy() if state else {},
            'action': (action.linear.x, action.angular.z),
            'reward': reward,
            'next_state': next_state.copy() if next_state else {}
        }

        self.experience_buffer.append(experience)

        # Keep buffer size manageable
        if len(self.experience_buffer) > self.max_experience_buffer_size:
            self.experience_buffer.pop(0)

    def save_parameters(self):
        """Save learned parameters to file"""
        params_file = 'learned_params.json'
        params_data = {
            'params': self.params,
            'run_count': self.run_count,
            'success_count': self.success_count,
            'total_time': self.total_time,
            'total_collisions': self.total_collisions,
            'performance_history': self.performance_history[-100:],  # Keep last 100 runs
            'saved_at': datetime.now().isoformat()
        }

        try:
            with open(params_file, 'w') as f:
                json.dump(params_data, f, indent=2)
            self.get_logger().info(f'Parameters saved to {params_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save parameters: {e}')

    def load_parameters(self):
        """Load learned parameters from file"""
        params_file = 'learned_params.json'

        if os.path.exists(params_file):
            try:
                with open(params_file, 'r') as f:
                    params_data = json.load(f)

                self.params.update(params_data.get('params', {}))
                self.run_count = params_data.get('run_count', 0)
                self.success_count = params_data.get('success_count', 0)
                self.total_time = params_data.get('total_time', 0.0)
                self.total_collisions = params_data.get('total_collisions', 0)
                self.performance_history = params_data.get('performance_history', [])

                self.get_logger().info(f'Parameters loaded from {params_file}: {self.params}')
            except Exception as e:
                self.get_logger().error(f'Failed to load parameters: {e}')
        else:
            self.get_logger().info('No saved parameters found, using defaults')

    def evaluate_performance(self):
        """Evaluate current run performance"""
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        # Check if we reached the goal (within tolerance)
        goal_reached = distance_to_goal < 0.5

        # Calculate time taken
        time_taken = 0.0
        if self.current_run_start_time:
            time_taken = (self.get_clock().now() - self.current_run_start_time).nanoseconds / 1e9

        # Calculate collisions (when robot got too close to obstacles)
        collision_count = 0
        if self.laser_data and self.laser_data['min_distance'] < 0.3:
            collision_count = 1  # Simplified collision detection

        return {
            'success': goal_reached,
            'time_taken': time_taken,
            'collisions': collision_count,
            'distance_to_goal': distance_to_goal
        }

    def control_loop(self):
        """Main control loop with learning capability"""
        # Initialize run if just starting
        if self.current_run_start_time is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.current_run_start_time = self.get_clock().now()

        # Get current state
        current_state = {
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'min_distance': self.laser_data['min_distance'] if self.laser_data else float('inf')
        }

        # Generate command using current policy
        cmd = self.simple_navigation_policy()

        # Calculate reward for current state-action pair
        reward = self.calculate_reward()

        # Store experience for learning
        self.save_experience(current_state, cmd, reward, {
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'min_distance': self.laser_data['min_distance'] if self.laser_data else float('inf')
        })

        # Periodically perform learning update
        if len(self.experience_buffer) % 20 == 0 and self.learning_enabled:
            self.simple_learning_algorithm()

        # Evaluate performance and check for run completion
        perf = self.evaluate_performance()

        if perf['success']:
            # Run completed successfully
            self.run_count += 1
            self.success_count += 1
            self.total_time += perf['time_taken']
            self.total_collisions += perf['collisions']

            avg_time = self.total_time / self.success_count if self.success_count > 0 else 0
            success_rate = self.success_count / self.run_count if self.run_count > 0 else 0

            self.performance_history.append({
                'run': self.run_count,
                'success': True,
                'time': perf['time_taken'],
                'collisions': perf['collisions'],
                'avg_time': avg_time,
                'success_rate': success_rate
            })

            self.get_logger().info(
                f'GOAL REACHED! Run #{self.run_count}, '
                f'Success Rate: {success_rate:.2%}, Avg Time: {avg_time:.2f}s'
            )

            # Save parameters after successful run
            self.save_parameters()

            # Reset for next run
            self.current_run_start_time = None

        elif perf['time_taken'] > 60:  # Timeout after 60 seconds
            # Run timed out
            self.run_count += 1
            self.total_time += perf['time_taken']
            self.total_collisions += perf['collisions']

            self.performance_history.append({
                'run': self.run_count,
                'success': False,
                'time': perf['time_taken'],
                'collisions': perf['collisions'],
                'avg_time': self.total_time / self.run_count if self.run_count > 0 else 0,
                'success_rate': self.success_count / self.run_count if self.run_count > 0 else 0
            })

            self.get_logger().info(f'RUN TIMED OUT (60s). Success rate: {self.success_count}/{self.run_count}')

            # Reset for next run
            self.current_run_start_time = None

        # Publish the command
        self.cmd_vel_pub.publish(cmd)

        # Log current status periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # Every ~1 second
            self.get_logger().info(
                f'Learning AI - Pos:({self.current_x:.2f},{self.current_y:.2f}), '
                f'Goal Dist:{perf["distance_to_goal"]:.2f}m, '
                f'Cmd:(lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}), '
                f'Success Rate:{self.success_count}/{self.run_count}'
            )

def main(args=None):
    rclpy.init(args=args)
    learning_node = LearningAINode()

    try:
        rclpy.spin(learning_node)
    except KeyboardInterrupt:
        # Save parameters before shutting down
        learning_node.save_parameters()
        pass
    finally:
        learning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementation Guide

### For Exercise 1 (Basic AI Agent):
1. Create the AI agent node with laser scan subscription
2. Implement obstacle detection logic using `np.min()` on valid ranges
3. Create simple navigation behaviors based on obstacle distances
4. Test in simulation with a robot that has laser range finder

### For Exercise 2 (Multi-Sensor Integration):
1. Set up multiple subscribers for different sensor types
2. Implement data synchronization using timestamps
3. Create a sensor fusion algorithm to combine sensor inputs
4. Use the fused state for improved navigation decisions

### For Exercise 3 (Behavior-Based Control):
1. Define behavior states using an Enum
2. Implement each behavior as a separate method
3. Create a state machine to manage transitions between behaviors
4. Use priority-based conflict resolution for competing behaviors

### For Exercise 4 (Path Planning Integration):
1. Implement a simple path planner (straight-line with intermediate waypoints)
2. Create a waypoint follower that navigates along the planned path
3. Integrate obstacle avoidance with path following
4. Implement replanning when obstacles block the path

### For Exercise 5 (Safety and Validation):
1. Create a separate safety node that intercepts AI commands
2. Implement velocity and acceleration limits
3. Add collision prediction and avoidance
4. Include emergency stop functionality

### For Exercise 6 (Learning-Based Behavior):
1. Implement a simple learning algorithm that adjusts parameters based on performance
2. Track performance metrics over multiple runs
3. Store learned parameters to persist across sessions
4. Use rewards to guide learning toward better navigation strategies

## Best Practices

1. **Modular Design**: Keep each exercise implementation modular and reusable
2. **Error Handling**: Always check for null data and handle edge cases
3. **Logging**: Use appropriate logging to track robot behavior and debugging information
4. **Parameter Tuning**: Use ROS parameters for easy tuning of behavior
5. **Safety First**: Always implement safety checks, especially when working with real robots
6. **Simulation Testing**: Test extensively in simulation before deploying to real hardware
7. **Performance Monitoring**: Track performance metrics to measure improvement over time

These solutions provide complete implementations for each exercise, demonstrating how to bridge AI agents with robot controllers using ROS 2. Each solution builds on the previous ones, showing progressive complexity from basic sensor processing to learning-based navigation.