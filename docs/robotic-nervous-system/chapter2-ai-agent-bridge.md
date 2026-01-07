# Chapter 2: Bridging Python-based AI Agents to Robot Controllers

## Learning Objectives
By the end of this chapter, readers will be able to:
- Explain the concepts of AI agent interfaces and their integration with ROS 2
- Create Python-based AI agents that interface with robot controllers using rclpy
- Implement sensor data processing for AI decision-making
- Develop control command generation systems
- Apply simulation techniques to test AI-robot integration

## Prerequisites
- Completion of Chapter 1: Understanding ROS 2 Architecture and Communication Patterns
- Basic understanding of AI and machine learning concepts
- Python programming experience
- Understanding of robotics fundamentals from Module 1

## Introduction
The integration of AI agents with robotic systems represents a crucial aspect of modern robotics. This chapter explores how to bridge the gap between high-level AI decision-making and low-level robot control using ROS 2. We'll examine how Python-based AI agents can communicate with robot controllers through ROS 2's communication infrastructure, enabling intelligent behavior in robotic systems.

The bridge between AI and robotics involves multiple layers of abstraction: sensor data processing, decision-making algorithms, and actuator command generation. This chapter will demonstrate how to implement these components using ROS 2's node-based architecture, enabling the creation of intelligent robotic systems.

## 1. AI Agent Interface Concepts

### 1.1 Components of an AI-Agent Bridge
An AI-agent bridge typically consists of several key components:

- **Sensor Interface**: Processes sensor data for the AI agent
- **AI Core**: Implements decision-making algorithms
- **Control Interface**: Translates AI decisions into robot commands
- **Communication Layer**: Handles ROS 2 messaging patterns

```python
# Example: Basic AI Agent Node Structure
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIAgentNode(Node):

    def __init__(self):
        super().__init__('ai_agent_node')

        # Sensor interface
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laser_callback,
            10
        )

        # Control interface
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.sensor_data = None
        self.ai_state = "IDLE"

        # AI processing timer
        self.ai_timer = self.create_timer(0.1, self.ai_processing_loop)

        self.get_logger().info('AI Agent Node initialized')

    def laser_callback(self, msg):
        """Process laser scan data from sensors"""
        self.sensor_data = msg.ranges  # Store sensor readings
        self.get_logger().debug(f'Received laser data with {len(msg.ranges)} readings')

    def ai_processing_loop(self):
        """Main AI decision-making loop"""
        if self.sensor_data is not None:
            # Process sensor data and make decisions
            command = self.make_decision(self.sensor_data)
            if command is not None:
                self.cmd_vel_publisher.publish(command)

    def make_decision(self, sensor_data):
        """AI decision-making logic"""
        # Example: Simple obstacle avoidance
        if sensor_data:
            min_distance = min([d for d in sensor_data if d > 0.1])  # Ignore invalid readings
            cmd = Twist()

            if min_distance < 1.0:  # Obstacle detected within 1 meter
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn right
                self.ai_state = "AVOIDING"
            else:
                cmd.linear.x = 0.5  # Move forward
                cmd.angular.z = 0.0
                self.ai_state = "MOVING"

            return cmd
        return None

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

### 1.2 AI Agent Interface Patterns
Different patterns can be used for AI-robot integration:

- **Reactive Agents**: Respond directly to sensor inputs
- **Deliberative Agents**: Plan actions based on goals and environment state
- **Hybrid Agents**: Combine reactive and deliberative approaches

## 2. rclpy Integration for AI Agents

### 2.1 Setting up AI Agent Nodes
Creating AI agents that integrate with ROS 2 requires careful consideration of the node structure and communication patterns.

```python
# Example: Advanced AI Agent with Multiple Sensors
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import numpy as np

class AdvancedAIAgent(Node):

    def __init__(self):
        super().__init__('advanced_ai_agent')

        # Multiple sensor interfaces
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Control interface
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.laser_data = None
        self.odom_data = None
        self.image_data = None
        self.goal_pose = None

        # Processing timer
        self.process_timer = self.create_timer(0.05, self.process_sensors)  # 20 Hz

        self.get_logger().info('Advanced AI Agent initialized')

    def laser_callback(self, msg):
        self.laser_data = np.array(msg.ranges)
        # Filter out invalid readings
        self.laser_data[self.laser_data == float('inf')] = 3.5  # Max range
        self.laser_data[np.isnan(self.laser_data)] = 3.5

    def odom_callback(self, msg):
        self.odom_data = msg

    def image_callback(self, msg):
        # Convert ROS Image to numpy array (simplified)
        # In practice, you'd use cv_bridge
        self.image_data = msg

    def process_sensors(self):
        """Process all sensor data and make decisions"""
        if self.laser_data is not None and self.odom_data is not None:
            # Example: Goal-oriented navigation with obstacle avoidance
            cmd = self.navigate_with_obstacle_avoidance()
            if cmd is not None:
                self.cmd_pub.publish(cmd)

    def navigate_with_obstacle_avoidance(self):
        """Combine navigation and obstacle avoidance"""
        cmd = Twist()

        # Check for obstacles
        if self.laser_data is not None:
            min_distance = np.min(self.laser_data)

            if min_distance < 0.8:  # Obstacle too close
                # Emergency stop and turn
                cmd.linear.x = 0.0
                cmd.angular.z = 0.8
                return cmd

        # If no immediate obstacles, navigate toward goal
        if self.goal_pose is not None and self.odom_data is not None:
            # Calculate direction to goal (simplified)
            pos = self.odom_data.pose.pose.position
            goal_pos = self.goal_pose.position

            dx = goal_pos.x - pos.x
            dy = goal_pos.y - pos.y

            distance_to_goal = np.sqrt(dx*dx + dy*dy)

            if distance_to_goal > 0.2:  # Not at goal
                cmd.linear.x = min(0.5, distance_to_goal * 0.5)  # Proportional to distance
                cmd.angular.z = np.arctan2(dy, dx) * 0.5  # Proportional to angle
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            # Default behavior: move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        return cmd
```

### 2.2 Handling Asynchronous AI Processing
AI processing often requires more time than sensor processing, so it's important to handle this properly:

```python
import asyncio
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class AsyncAIAgent(Node):

    def __init__(self):
        super().__init__('async_ai_agent')

        # Sensor interface
        qos_profile = QoSProfile(depth=10)
        self.sensor_sub = self.create_subscription(
            LaserScan, '/scan', self.sensor_callback, qos_profile)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store sensor data for processing
        self.latest_sensor_data = None

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.2, self.process_with_ai)  # 5 Hz for AI

        # Async processing queue
        self.ai_queue = asyncio.Queue()

        self.get_logger().info('Async AI Agent initialized')

    def sensor_callback(self, msg):
        """Store latest sensor data"""
        self.latest_sensor_data = msg

    def process_with_ai(self):
        """Process sensor data with AI and send commands"""
        if self.latest_sensor_data is not None:
            # Perform AI processing
            command = self.ai_decision_process(self.latest_sensor_data)
            if command is not None:
                self.cmd_pub.publish(command)

    def ai_decision_process(self, sensor_data):
        """AI decision-making with more complex logic"""
        # Example: More sophisticated AI algorithm
        ranges = np.array(sensor_data.ranges)

        # Detect closest obstacle in front (forward 90 degrees)
        front_ranges = ranges[270:450]  # Assuming 720-point scan
        front_ranges = front_ranges[np.isfinite(front_ranges)]  # Remove inf values

        if len(front_ranges) > 0:
            min_front = np.min(front_ranges)
        else:
            min_front = float('inf')

        cmd = Twist()

        if min_front < 0.8:
            # Obstacle detected - turn away
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6
        elif min_front < 1.5:
            # Safe distance - move forward slowly
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
        else:
            # Clear path - move forward normally
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        return cmd
```

## 3. Sensor Data Processing

### 3.1 Processing Different Sensor Types
Different sensors provide different types of information that need to be processed appropriately:

```python
from sensor_msgs.msg import LaserScan, PointCloud2, Imu, JointState
from geometry_msgs.msg import Vector3Stamped

class MultiSensorProcessor(Node):

    def __init__(self):
        super().__init__('multi_sensor_processor')

        # Multiple sensor subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Processed data storage
        self.sensor_fusion_data = {
            'position': None,
            'orientation': None,
            'velocity': None,
            'obstacles': None
        }

        self.get_logger().info('Multi-sensor processor initialized')

    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        ranges = np.array(msg.ranges)
        # Filter invalid readings
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0.1)]

        if len(valid_ranges) > 0:
            # Detect obstacles in different directions
            n = len(ranges)
            front_idx = slice(n//2 - n//8, n//2 + n//8)  # Front 1/4 of scan
            left_idx = slice(n//4 - n//16, n//4 + n//16)   # Left region
            right_idx = slice(3*n//4 - n//16, 3*n//4 + n//16)  # Right region

            front_min = np.min(ranges[front_idx]) if np.any(np.isfinite(ranges[front_idx])) else float('inf')
            left_min = np.min(ranges[left_idx]) if np.any(np.isfinite(ranges[left_idx])) else float('inf')
            right_min = np.min(ranges[right_idx]) if np.any(np.isfinite(ranges[right_idx])) else float('inf')

            self.sensor_fusion_data['obstacles'] = {
                'front': front_min,
                'left': left_min,
                'right': right_min
            }

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        # Extract orientation (quaternion to euler)
        import math
        orientation_q = msg.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.sensor_fusion_data['orientation'] = yaw
        self.sensor_fusion_data['angular_velocity'] = msg.angular_velocity.z

    def joint_callback(self, msg):
        """Process joint state data for position and velocity"""
        # Example: Process wheel encoder data if available
        if 'wheel_left_joint' in msg.name and 'wheel_right_joint' in msg.name:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')

            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]

            self.sensor_fusion_data['wheel_positions'] = (left_pos, right_pos)
```

### 3.2 Sensor Fusion Techniques
Combining data from multiple sensors improves the AI agent's understanding of the environment:

```python
class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # All sensor subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Fused state publisher
        self.state_pub = self.create_publisher(String, '/fused_state', 10)

        # Internal state
        self.robot_state = {
            'position': (0, 0),
            'orientation': 0,
            'linear_velocity': 0,
            'angular_velocity': 0,
            'obstacle_distances': [],
            'confidence': 1.0  # Overall confidence in state estimate
        }

        # State update timer
        self.state_timer = self.create_timer(0.05, self.publish_fused_state)  # 20 Hz

    def laser_callback(self, msg):
        """Process laser data for obstacle detection"""
        ranges = np.array(msg.ranges)
        # Filter and store obstacle distances
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0.1)]
        self.robot_state['obstacle_distances'] = valid_ranges if len(valid_ranges) > 0 else [float('inf')]

    def odom_callback(self, msg):
        """Update position and velocity from odometry"""
        pos = msg.pose.pose.position
        self.robot_state['position'] = (pos.x, pos.y)

        vel = msg.twist.twist
        self.robot_state['linear_velocity'] = np.sqrt(vel.linear.x**2 + vel.linear.y**2)
        self.robot_state['angular_velocity'] = vel.angular.z

    def imu_callback(self, msg):
        """Update orientation from IMU"""
        # Convert quaternion to yaw
        import math
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_state['orientation'] = math.atan2(siny_cosp, cosy_cosp)

    def publish_fused_state(self):
        """Publish fused state for AI agent"""
        # Create a comprehensive state message
        state_msg = String()
        state_dict = {
            'position': self.robot_state['position'],
            'orientation': self.robot_state['orientation'],
            'linear_velocity': self.robot_state['linear_velocity'],
            'angular_velocity': self.robot_state['angular_velocity'],
            'min_obstacle_distance': min(self.robot_state['obstacle_distances']) if self.robot_state['obstacle_distances'] else float('inf'),
            'confidence': self.robot_state['confidence']
        }

        import json
        state_msg.data = json.dumps(state_dict)
        self.state_pub.publish(state_msg)
```

## 4. Control Command Generation

### 4.1 Translating AI Decisions to Robot Commands
The bridge between AI decisions and robot control requires careful mapping of abstract decisions to specific robot commands:

```python
from geometry_msgs.msg import Twist, Pose, Point
from std_msgs.msg import Float64
from builtin_interfaces.msg import Duration

class ControlCommandGenerator(Node):

    def __init__(self):
        super().__init__('control_command_generator')

        # Subscribe to AI decisions
        self.ai_decision_sub = self.create_subscription(
            String, '/ai_decision', self.ai_decision_callback, 10)

        # Robot control publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pubs = []  # For joint controllers

        # Robot parameters
        self.robot_params = {
            'max_linear_vel': 1.0,
            'max_angular_vel': 1.0,
            'max_joint_vel': 2.0
        }

        self.get_logger().info('Control command generator initialized')

    def ai_decision_callback(self, msg):
        """Process AI decision and generate control commands"""
        try:
            decision = eval(msg.data)  # In practice, use json.loads for safety
            command = self.generate_command(decision)
            if command is not None:
                self.cmd_vel_pub.publish(command)
        except Exception as e:
            self.get_logger().error(f'Error processing AI decision: {e}')

    def generate_command(self, decision):
        """Generate robot command from AI decision"""
        cmd = Twist()

        if decision['type'] == 'move':
            # Translate movement decision to velocity command
            cmd.linear.x = self.clamp_value(
                decision.get('linear_speed', 0.0),
                -self.robot_params['max_linear_vel'],
                self.robot_params['max_linear_vel']
            )
            cmd.angular.z = self.clamp_value(
                decision.get('angular_speed', 0.0),
                -self.robot_params['max_angular_vel'],
                self.robot_params['max_angular_vel']
            )

        elif decision['type'] == 'navigate':
            # Navigate to specific point
            cmd = self.generate_navigation_command(decision['target'])

        elif decision['type'] == 'avoid':
            # Obstacle avoidance maneuver
            cmd = self.generate_avoidance_command(decision.get('obstacle_direction', 'front'))

        return cmd

    def generate_navigation_command(self, target):
        """Generate navigation command to reach target"""
        # This would typically involve path planning
        # For simplicity, we'll create a direct movement command
        cmd = Twist()

        # Calculate direction to target (simplified)
        # In practice, you'd get current position from odometry
        dx = target['x'] - 0.0  # Current x assumed to be 0
        dy = target['y'] - 0.0  # Current y assumed to be 0

        distance = np.sqrt(dx*dx + dy*dy)

        if distance > 0.1:  # Not at target
            cmd.linear.x = min(0.5, distance * 0.5)  # Proportional to distance
            cmd.angular.z = np.arctan2(dy, dx) * 0.5  # Proportional to angle
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def generate_avoidance_command(self, direction):
        """Generate obstacle avoidance command"""
        cmd = Twist()

        if direction == 'front':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right
        elif direction == 'left':
            cmd.linear.x = 0.3
            cmd.angular.z = -0.3  # Turn right
        elif direction == 'right':
            cmd.linear.x = 0.3
            cmd.angular.z = 0.3   # Turn left
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def clamp_value(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))
```

### 4.2 Safety and Validation
Implementing safety checks is crucial when bridging AI agents to physical robots:

```python
class SafeControlBridge(Node):

    def __init__(self):
        super().__init__('safe_control_bridge')

        # Subscriptions
        self.ai_cmd_sub = self.create_subscription(Twist, '/ai_cmd', self.ai_command_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        # Publications
        self.safety_filtered_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety parameters
        self.safety_params = {
            'max_linear_vel': 0.5,
            'max_angular_vel': 0.5,
            'max_linear_acc': 1.0,
            'max_angular_acc': 1.0,
            'emergency_stop': False
        }

        # Previous command for acceleration limiting
        self.prev_cmd = Twist()
        self.prev_time = self.get_clock().now()

        self.get_logger().info('Safe control bridge initialized')

    def ai_command_callback(self, msg):
        """Process AI command with safety validation"""
        if self.safety_params['emergency_stop']:
            # Publish zero command if emergency stop is active
            safe_cmd = Twist()
        else:
            # Apply safety filters
            safe_cmd = self.apply_safety_filters(msg)

        # Publish the safe command
        self.safety_filtered_cmd_pub.publish(safe_cmd)
        self.prev_cmd = safe_cmd
        self.prev_time = self.get_clock().now()

    def apply_safety_filters(self, cmd):
        """Apply safety filters to AI command"""
        safe_cmd = Twist()

        # Limit velocities
        safe_cmd.linear.x = self.clamp_value(
            cmd.linear.x,
            -self.safety_params['max_linear_vel'],
            self.safety_params['max_linear_vel']
        )
        safe_cmd.angular.z = self.clamp_value(
            cmd.angular.z,
            -self.safety_params['max_angular_vel'],
            self.safety_params['max_angular_vel']
        )

        # Limit accelerations (simplified approach)
        current_time = self.get_clock().now()
        time_diff = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        if time_diff > 0:
            # Calculate required acceleration
            lin_acc = abs(safe_cmd.linear.x - self.prev_cmd.linear.x) / time_diff
            ang_acc = abs(safe_cmd.angular.z - self.prev_cmd.angular.z) / time_diff

            # Limit if acceleration is too high
            if lin_acc > self.safety_params['max_linear_acc']:
                safe_cmd.linear.x = self.prev_cmd.linear.x + np.sign(safe_cmd.linear.x - self.prev_cmd.linear.x) * self.safety_params['max_linear_acc'] * time_diff

            if ang_acc > self.safety_params['max_angular_acc']:
                safe_cmd.angular.z = self.prev_cmd.angular.z + np.sign(safe_cmd.angular.z - self.prev_cmd.angular.z) * self.safety_params['max_angular_acc'] * time_diff

        return safe_cmd

    def emergency_stop_callback(self, msg):
        """Handle emergency stop command"""
        self.safety_params['emergency_stop'] = msg.data
        if msg.data:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().info('Emergency stop deactivated')

    def clamp_value(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))
```

## 5. Practical Example: AI-Controlled Robot Navigation

Let's create a complete example that demonstrates AI-robot integration:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
import math

class AINavigator(Node):

    def __init__(self):
        super().__init__('ai_navigator')

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publications
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.scan_data = None
        self.position = (0.0, 0.0)
        self.orientation = 0.0
        self.target = (5.0, 5.0)  # Navigate to (5,5)

        # Processing timer
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info(f'AI Navigator initialized, navigating to {self.target}')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Filter invalid readings
        self.scan_data[self.scan_data == float('inf')] = 3.5
        self.scan_data[np.isnan(self.scan_data)] = 3.5

    def odom_callback(self, msg):
        """Update position from odometry"""
        pos = msg.pose.pose.position
        self.position = (pos.x, pos.y)

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)

    def navigation_loop(self):
        """Main navigation decision loop"""
        if self.scan_data is None:
            return

        # Calculate distance to target
        dx = self.target[0] - self.position[0]
        dy = self.target[1] - self.position[1]
        distance_to_target = math.sqrt(dx*dx + dy*dy)

        # Check for obstacles
        if self.scan_data.size > 0:
            min_obstacle_dist = np.min(self.scan_data)
        else:
            min_obstacle_dist = float('inf')

        cmd = Twist()

        if distance_to_target < 0.5:
            # Reached target
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Target reached!')
        elif min_obstacle_dist < 0.8:
            # Obstacle avoidance
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8  # Turn to avoid
        else:
            # Navigate toward target
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.orientation

            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Proportional control for orientation
            cmd.angular.z = max(-0.5, min(0.5, angle_diff * 1.0))

            # Move forward if roughly aligned with target
            if abs(angle_diff) < 0.5:
                cmd.linear.x = 0.5
            else:
                cmd.linear.x = 0.1  # Move slowly while turning

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    ai_navigator = AINavigator()

    try:
        rclpy.spin(ai_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        ai_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```


## 6. Exercises and Practice

Complete the following exercises to reinforce your understanding of bridging AI agents with robot controllers:

1. [Chapter 2 Exercises](./chapter2-exercises.md) - Hands-on problems covering AI agent implementation and integration
2. [Chapter 2 Solutions](./chapter2-solutions.md) - Complete implementations and solution guides

## 7. Summary

This chapter covered the essential concepts of bridging Python-based AI agents to robot controllers:

- AI agent interface patterns and structures
- rclpy integration for AI-robot communication
- Sensor data processing and fusion techniques
- Control command generation from AI decisions
- Safety considerations for AI-robot systems

The next chapter will explore URDF for humanoid robot description and control, building upon the communication and integration concepts learned here.

## 8. Further Reading

- [ROS 2 Navigation2 Documentation](https://navigation.ros.org/)
- [Behavior Trees in Robotics](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [ROS 2 Control Documentation](https://control.ros.org/)

## 9. Links to External Resources

- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [AI in Robotics Research Papers](https://arxiv.org/list/cs.RO/recent)
- [Robot Operating System Tutorials](https://docs.ros.org/en/humble/Tutorials.html)