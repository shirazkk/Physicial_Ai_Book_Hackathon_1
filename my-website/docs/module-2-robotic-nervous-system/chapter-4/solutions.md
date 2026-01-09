#  Solutions: Implementing Robotic Nervous System Patterns

## Solution for Exercise 1: Basic Reflex System Implementation

### Complete Reflex System Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class BasicReflexSystem(Node):
    """
    Basic reflex system that immediately responds to obstacles.
    """

    def __init__(self):
        super().__init__('basic_reflex_system')

        # Subscribe to laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        # Publish emergency stop commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Emergency status publisher
        self.emergency_pub = self.create_publisher(Bool, '/emergency_status', 10)

        # High-frequency reflex timer (200 Hz for immediate response)
        self.reflex_timer = self.create_timer(0.005, self.reflex_control)

        # Internal state
        self.laser_data = None
        self.safety_threshold = 0.3  # meters
        self.emergency_active = False

        self.get_logger().info('Basic Reflex System initialized')

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = np.array(msg.ranges)
        # Filter invalid readings
        self.laser_data[self.laser_data == float('inf')] = 3.5
        self.laser_data[np.isnan(self.laser_data)] = 3.5

    def reflex_control(self):
        """Main reflex control loop with immediate response"""
        if self.laser_data is None:
            return

        # Find minimum distance to any obstacle
        min_distance = np.min(self.laser_data) if self.laser_data.size > 0 else float('inf')

        # Check for immediate danger
        if min_distance < self.safety_threshold:
            if not self.emergency_active:
                self.emergency_active = True
                self.get_logger().warn(f'EMERGENCY REFLEX: Obstacle detected at {min_distance:.2f}m! STOPPING!')

                # Publish emergency status
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_pub.publish(emergency_msg)

            # Immediate stop command
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
        else:
            # No immediate danger, but may need to slow down for approach
            if self.emergency_active:
                self.emergency_active = False
                self.get_logger().info('REFLEX: Emergency cleared')

                # Publish normal status
                normal_msg = Bool()
                normal_msg.data = False
                self.emergency_pub.publish(normal_msg)

def main(args=None):
    rclpy.init(args=args)
    reflex_system = BasicReflexSystem()

    try:
        rclpy.spin(reflex_system)
    except KeyboardInterrupt:
        pass
    finally:
        reflex_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run
1. Save the code as `basic_reflex_system.py`
2. Make sure your ROS 2 environment is sourced
3. Run the node: `python3 basic_reflex_system.py`
4. Test with obstacles placed closer than 0.3 meters

## Solution for Exercise 2: Hierarchical Control Architecture

### Complete Hierarchical Control System
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus
import json
import math

class HighLevelPlanner(Node):
    """
    High-level planning node - cortical level.
    """
    def __init__(self):
        super().__init__('high_level_planner')

        # Publishers and subscribers
        self.task_pub = self.create_publisher(String, '/mid_level_tasks', 10)
        self.status_pub = self.create_publisher(String, '/high_level_status', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/navigation_goal', self.goal_callback, 10)

        # Timer for high-level planning
        self.planning_timer = self.create_timer(0.5, self.planning_loop)  # 2 Hz

        # Internal state
        self.current_goal = None
        self.plan = []
        self.execution_status = "IDLE"

        self.get_logger().info('High Level Planner initialized')

    def goal_callback(self, msg):
        """Receive navigation goals"""
        self.current_goal = msg
        self.execution_status = "PLANNING"
        self.get_logger().info(f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def planning_loop(self):
        """High-level planning and goal management"""
        if self.execution_status == "PLANNING" and self.current_goal:
            # Generate a simple plan (in real implementation, this would be actual path planning)
            plan = [
                {"type": "navigate", "target": [self.current_goal.pose.position.x, self.current_goal.pose.position.y]},
                {"type": "wait", "duration": 2.0},
                {"type": "return", "target": [0.0, 0.0]}
            ]

            self.plan = plan
            self.execution_status = "EXECUTING"

            # Send first task to mid-level
            if self.plan:
                task_msg = String()
                task_msg.data = json.dumps(self.plan[0])
                self.task_pub.publish(task_msg)
                self.get_logger().info(f'Sent task to mid-level: {self.plan[0]}')

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            "status": self.execution_status,
            "current_goal": [self.current_goal.pose.position.x, self.current_goal.pose.position.y] if self.current_goal else None,
            "plan_length": len(self.plan),
            "tasks_completed": 0
        })
        self.status_pub.publish(status_msg)

class MidLevelTaskManager(Node):
    """
    Mid-level task execution - midbrain level.
    """
    def __init__(self):
        super().__init__('mid_level_task_manager')

        # Publishers and subscribers
        self.action_pub = self.create_publisher(String, '/low_level_actions', 10)
        self.task_sub = self.create_subscription(String, '/mid_level_tasks', self.task_callback, 10)
        self.status_pub = self.create_publisher(String, '/mid_level_status', 10)

        # Timer for task management
        self.task_timer = self.create_timer(0.1, self.task_execution_loop)  # 10 Hz

        # Internal state
        self.current_task = None
        self.task_queue = []
        self.execution_status = "IDLE"

        self.get_logger().info('Mid Level Task Manager initialized')

    def task_callback(self, msg):
        """Receive tasks from high-level planner"""
        try:
            task_data = json.loads(msg.data)
            self.task_queue.append(task_data)
            self.get_logger().info(f'Received task: {task_data}')
        except Exception as e:
            self.get_logger().error(f'Error parsing task: {e}')

    def task_execution_loop(self):
        """Execute tasks and break them into low-level actions"""
        if not self.task_queue and not self.current_task:
            return

        if not self.current_task and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            self.execution_status = "EXECUTING"

        if self.current_task:
            # Break down complex tasks into simple actions
            action = self.task_to_action(self.current_task)

            if action:
                action_msg = String()
                action_msg.data = json.dumps(action)
                self.action_pub.publish(action_msg)

                self.get_logger().info(f'Sent action to low-level: {action}')

                # Check if task is completed
                if self.is_task_completed(self.current_task):
                    self.current_task = None
                    self.execution_status = "IDLE"
                    self.get_logger().info('Task completed')

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            "status": self.execution_status,
            "current_task": self.current_task,
            "queue_length": len(self.task_queue)
        })
        self.status_pub.publish(status_msg)

    def task_to_action(self, task):
        """Convert task to low-level action"""
        task_type = task.get("type", "")

        if task_type == "navigate":
            target = task.get("target", [0, 0])
            return {
                "type": "move_to",
                "x": target[0],
                "y": target[1],
                "speed": 0.5
            }
        elif task_type == "wait":
            duration = task.get("duration", 1.0)
            return {
                "type": "wait",
                "duration": duration
            }
        elif task_type == "return":
            target = task.get("target", [0, 0])
            return {
                "type": "move_to",
                "x": target[0],
                "y": target[1],
                "speed": 0.3
            }

        return None

    def is_task_completed(self, task):
        """Check if task is completed (simplified)"""
        # In a real implementation, this would check actual robot position vs target
        return True  # For this example, assume immediate completion

class LowLevelController(Node):
    """
    Low-level motor control - spinal level.
    """
    def __init__(self):
        super().__init__('low_level_controller')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_sub = self.create_subscription(String, '/low_level_actions', self.action_callback, 10)
        self.status_pub = self.create_publisher(String, '/low_level_status', 10)

        # Timer for low-level control
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        # Internal state
        self.current_action = None
        self.action_start_time = None
        self.execution_status = "IDLE"

        self.get_logger().info('Low Level Controller initialized')

    def action_callback(self, msg):
        """Receive low-level actions from mid-level"""
        try:
            action_data = json.loads(msg.data)
            self.current_action = action_data
            self.action_start_time = self.get_clock().now()
            self.execution_status = "EXECUTING"
            self.get_logger().info(f'Received action: {action_data}')
        except Exception as e:
            self.get_logger().error(f'Error parsing action: {e}')

    def control_loop(self):
        """Execute low-level motor commands"""
        if not self.current_action:
            # Send stop command if no action
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return

        action_type = self.current_action.get("type", "")
        cmd = Twist()

        if action_type == "move_to":
            # Simplified navigation - in reality, this would involve PID control, path following, etc.
            cmd.linear.x = self.current_action.get("speed", 0.5)
            cmd.angular.z = 0.0  # For simplicity, assume direct movement
        elif action_type == "wait":
            # During wait, send zero velocity
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action_type == "rotate":
            cmd.angular.z = self.current_action.get("angular_speed", 0.5)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Check if action is completed
        if self.is_action_completed():
            self.current_action = None
            self.execution_status = "IDLE"
            self.get_logger().info('Action completed')

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            "status": self.execution_status,
            "current_action": self.current_action
        })
        self.status_pub.publish(status_msg)

    def is_action_completed(self):
        """Check if current action is completed"""
        if not self.current_action or not self.action_start_time:
            return True

        action_type = self.current_action.get("type", "")
        if action_type == "wait":
            duration = self.current_action.get("duration", 1.0)
            elapsed = (self.get_clock().now() - self.action_start_time).nanoseconds / 1e9
            return elapsed >= duration

        # For movement actions, this would check actual position vs target
        # For this example, assume a fixed duration
        return False

def main(args=None):
    rclpy.init(args=args)

    # Create all nodes
    high_level = HighLevelPlanner()
    mid_level = MidLevelTaskManager()
    low_level = LowLevelController()

    # Create executor to handle multiple nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(high_level)
    executor.add_node(mid_level)
    executor.add_node(low_level)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        high_level.destroy_node()
        mid_level.destroy_node()
        low_level.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 3: Sensorimotor Integration

### Complete Sensorimotor Integration System
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
import numpy as np
import math
from collections import deque

class SensorimotorIntegration(Node):
    """
    System that integrates multiple sensors to generate coordinated motor outputs.
    """

    def __init__(self):
        super().__init__('sensorimotor_integration')

        # Multiple sensor subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Motor command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Integration timer
        self.integration_timer = self.create_timer(0.02, self.integration_loop)  # 50 Hz

        # Internal sensor buffers
        self.laser_buffer = deque(maxlen=5)
        self.imu_buffer = deque(maxlen=5)
        self.odom_buffer = deque(maxlen=5)

        # Sensor data storage
        self.sensors = {
            'laser': None,
            'imu': None,
            'odom': None
        }

        # Integration parameters
        self.integration_weights = {
            'obstacle_avoidance': 0.4,
            'balance_control': 0.3,
            'navigation': 0.3
        }

        # Sensor validation thresholds
        self.validation_thresholds = {
            'laser_min_range': 0.1,
            'laser_max_range': 3.5,
            'imu_timeout': 1.0  # seconds
        }

        self.get_logger().info('Sensorimotor Integration System initialized')

    def laser_callback(self, msg):
        """Process laser scan data"""
        # Validate laser data
        ranges = np.array(msg.ranges)

        # Filter invalid readings
        ranges[ranges < self.validation_thresholds['laser_min_range']] = self.validation_thresholds['laser_max_range']
        ranges[ranges > self.validation_thresholds['laser_max_range']] = self.validation_thresholds['laser_max_range']
        ranges[np.isnan(ranges)] = self.validation_thresholds['laser_max_range']

        self.sensors['laser'] = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Add to buffer
        self.laser_buffer.append(self.sensors['laser'])

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Extract orientation from quaternion
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.sensors['imu'] = {
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': yaw},  # Simplified
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Add to buffer
        self.imu_buffer.append(self.sensors['imu'])

    def odom_callback(self, msg):
        """Process odometry data for navigation"""
        self.sensors['odom'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'velocity': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Add to buffer
        self.odom_buffer.append(self.sensors['odom'])

    def integration_loop(self):
        """Main sensorimotor integration loop"""
        if not all(self.sensors.values()):
            return

        # Process each sensor modality
        obstacle_cmd = self.process_obstacle_avoidance()
        balance_cmd = self.process_balance_control()
        navigation_cmd = self.process_navigation()

        # Weighted integration of commands
        final_cmd = Twist()

        # Combine linear components
        final_cmd.linear.x = (
            self.integration_weights['obstacle_avoidance'] * obstacle_cmd['linear_x'] +
            self.integration_weights['balance_control'] * balance_cmd['linear_x'] +
            self.integration_weights['navigation'] * navigation_cmd['linear_x']
        )

        # Combine angular components
        final_cmd.angular.z = (
            self.integration_weights['obstacle_avoidance'] * obstacle_cmd['angular_z'] +
            self.integration_weights['balance_control'] * balance_cmd['angular_z'] +
            self.integration_weights['navigation'] * navigation_cmd['angular_z']
        )

        # Apply safety limits
        final_cmd.linear.x = max(-1.0, min(1.0, final_cmd.linear.x))
        final_cmd.angular.z = max(-1.0, min(1.0, final_cmd.angular.z))

        # Publish integrated command
        self.cmd_vel_pub.publish(final_cmd)

        self.get_logger().debug(f'Integrated command: linear={final_cmd.linear.x:.2f}, angular={final_cmd.angular.z:.2f}')

    def process_obstacle_avoidance(self):
        """Process laser data for obstacle avoidance"""
        if not self.sensors['laser']:
            return {'linear_x': 0.0, 'angular_z': 0.0}

        laser_data = self.sensors['laser']['ranges']
        min_distance = np.min(laser_data) if laser_data.size > 0 else float('inf')

        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        if min_distance < 0.3:  # Emergency avoidance
            cmd['linear_x'] = -0.3  # Back up
            # Turn away from closest obstacle
            closest_idx = np.argmin(laser_data)
            cmd['angular_z'] = 0.8 if closest_idx < len(laser_data) / 2 else -0.8
        elif min_distance < 0.8:  # Normal avoidance
            cmd['linear_x'] = max(0.1, min_distance * 0.4)  # Slow down proportionally
            # Gentle turn away from closest obstacle
            closest_idx = np.argmin(laser_data)
            cmd['angular_z'] = 0.4 if closest_idx < len(laser_data) / 2 else -0.4
        else:  # Clear path
            cmd['linear_x'] = 0.6  # Normal forward speed
            cmd['angular_z'] = 0.0  # No turn

        return cmd

    def process_balance_control(self):
        """Process IMU data for balance control"""
        if not self.sensors['imu']:
            return {'linear_x': 0.0, 'angular_z': 0.0}

        # Extract orientation data
        yaw = self.sensors['imu']['orientation']['yaw']

        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        # Simple balance correction based on orientation
        # In a real system, this would be more sophisticated
        if abs(yaw) > 0.2:  # Significant orientation error
            cmd['angular_z'] = -yaw * 1.5  # Correct by turning opposite to error
        else:
            cmd['angular_z'] = -yaw * 0.8  # Gentle correction

        return cmd

    def process_navigation(self):
        """Process odometry for navigation"""
        if not self.sensors['odom']:
            return {'linear_x': 0.0, 'angular_z': 0.0}

        # Extract velocity data
        linear_vel = self.sensors['odom']['velocity']['linear']['x']
        angular_vel = self.sensors['odom']['velocity']['angular']['z']

        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        # For this example, maintain current velocity as baseline
        # In a real system, this would involve goal-seeking behavior
        cmd['linear_x'] = 0.3  # Default forward motion
        cmd['angular_z'] = 0.0  # Default no turn

        return cmd

def main(args=None):
    rclpy.init(args=args)
    sensorimotor_system = SensorimotorIntegration()

    try:
        rclpy.spin(sensorimotor_system)
    except KeyboardInterrupt:
        pass
    finally:
        sensorimotor_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 4: Adaptive Control System

### Complete Adaptive Control System
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
import math
from collections import deque
import json
import os
import pickle

class AdaptiveControlSystem(Node):
    """
    Adaptive control system that learns and improves behavior over time.
    """

    def __init__(self):
        super().__init__('adaptive_control_system')

        # Sensor subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Motor command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Learning and adaptation timer
        self.adaptation_timer = self.create_timer(0.1, self.adaptation_loop)  # 10 Hz

        # Internal state
        self.sensors = {
            'laser': None,
            'odom': None
        }

        # Learning components
        self.experience_buffer = deque(maxlen=500)  # Store experiences
        self.performance_history = deque(maxlen=100)  # Track performance
        self.learning_params = {
            'learning_rate': 0.01,
            'exploration_rate': 0.1,
            'discount_factor': 0.9,
            'performance_threshold': 0.7
        }

        # Adaptive parameters
        self.adaptive_params = {
            'obstacle_threshold': 0.8,
            'approach_speed_factor': 0.5,
            'turn_sensitivity': 0.8,
            'safety_margin': 0.3
        }

        # Learned behaviors (state-action mappings)
        self.learnt_behaviors = {}

        # Performance tracking
        self.total_distance_traveled = 0.0
        self.collision_count = 0
        self.safety_violation_count = 0

        # Load saved learning if available
        self.load_learning()

        self.get_logger().info('Adaptive Control System initialized')

    def laser_callback(self, msg):
        """Process laser scan data"""
        ranges = np.array(msg.ranges)
        # Filter invalid readings
        ranges[ranges == float('inf')] = 3.5
        ranges[np.isnan(ranges)] = 3.5
        ranges[ranges < 0.1] = 3.5  # Remove very close invalid readings

        self.sensors['laser'] = {
            'ranges': ranges,
            'min_distance': float(np.min(ranges)) if ranges.size > 0 else float('inf'),
            'front_clear': float(np.min(ranges[len(ranges)//2-10:len(ranges)//2+10])) if ranges.size > 20 else float('inf'),
            'left_clear': float(np.min(ranges[:len(ranges)//4])) if ranges.size > 0 else float('inf'),
            'right_clear': float(np.min(ranges[3*len(ranges)//4:])) if ranges.size > 0 else float('inf'),
            'timestamp': self.get_clock().now().nanoseconds
        }

    def odom_callback(self, msg):
        """Process odometry data"""
        self.sensors['odom'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            },
            'velocity': {
                'linear': msg.twist.twist.linear.x,
                'angular': msg.twist.twist.angular.z
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

    def adaptation_loop(self):
        """Main adaptation loop"""
        if not all(self.sensors.values()):
            return

        # Get current state representation
        current_state = self.get_state_representation()

        # Select action based on current state and learning
        action = self.select_adaptive_action(current_state)

        # Execute action
        self.execute_action(action)

        # Evaluate performance
        performance = self.evaluate_performance()

        # Store experience for learning
        experience = {
            'state': current_state,
            'action': action,
            'performance': performance,
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.experience_buffer.append(experience)
        self.performance_history.append(performance)

        # Update learning periodically
        if len(self.experience_buffer) % 10 == 0:
            self.update_learning()

        # Log performance metrics periodically
        if len(self.performance_history) % 50 == 0:
            avg_performance = np.mean(list(self.performance_history))
            self.get_logger().info(f'Performance: avg={avg_performance:.3f}, collisions={self.collision_count}')

    def get_state_representation(self):
        """Create a discrete state representation from sensor data"""
        if not self.sensors['laser']:
            return "unknown"

        laser = self.sensors['laser']

        # Discretize sensor readings
        min_dist_category = self.categorize_distance(laser['min_distance'])
        front_clear_category = self.categorize_distance(laser['front_clear'])
        approach_state = "approaching" if laser['min_distance'] < self.adaptive_params['obstacle_threshold'] else "clear"

        # Create state key
        state_key = f"{min_dist_category}_{front_clear_category}_{approach_state}"
        return state_key

    def categorize_distance(self, distance):
        """Categorize distance into discrete bins"""
        if distance < 0.3:
            return "very_close"
        elif distance < 0.6:
            return "close"
        elif distance < 1.0:
            return "medium"
        else:
            return "far"

    def select_adaptive_action(self, state):
        """Select action based on state and learned knowledge"""
        # Check if we have learned behavior for this state
        if state in self.learnt_behaviors:
            # Use learned behavior with some exploration
            if np.random.random() < self.learning_params['exploration_rate']:
                # Explore: random action
                return self.generate_exploratory_action()
            else:
                # Exploit: use learned behavior
                return self.learnt_behaviors[state]
        else:
            # No learned behavior, use default adaptive controller
            return self.adaptive_behavior()

    def generate_exploratory_action(self):
        """Generate a random exploratory action"""
        cmd = Twist()
        cmd.linear.x = np.random.uniform(0.1, 0.8)  # Forward speed between 0.1 and 0.8
        cmd.angular.z = np.random.uniform(-0.8, 0.8)  # Turn rate between -0.8 and 0.8
        return cmd

    def adaptive_behavior(self):
        """Default adaptive behavior based on current sensor readings"""
        if not self.sensors['laser']:
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            return cmd

        laser = self.sensors['laser']
        cmd = Twist()

        # Adaptive obstacle avoidance
        if laser['min_distance'] < self.adaptive_params['safety_margin']:
            # Emergency stop - too close to obstacle
            cmd.linear.x = -0.3  # Back up slowly
            # Turn away from closest obstacle
            ranges = laser['ranges']
            closest_idx = np.argmin(ranges)
            cmd.angular.z = 0.6 if closest_idx < len(ranges) / 2 else -0.6
        elif laser['min_distance'] < self.adaptive_params['obstacle_threshold']:
            # Normal obstacle avoidance
            cmd.linear.x = max(0.1, laser['min_distance'] * self.adaptive_params['approach_speed_factor'])

            # Turn away from closest obstacle with adaptive sensitivity
            ranges = laser['ranges']
            closest_idx = np.argmin(ranges)
            turn_direction = 0.6 if closest_idx < len(ranges) / 2 else -0.6
            cmd.angular.z = turn_direction * self.adaptive_params['turn_sensitivity']
        else:
            # Clear path, move forward
            cmd.linear.x = 0.6
            cmd.angular.z = 0.0

        return cmd

    def execute_action(self, action):
        """Execute the selected action"""
        self.cmd_vel_pub.publish(action)

    def evaluate_performance(self):
        """Evaluate the performance of the current behavior"""
        if not all(self.sensors.values()):
            return 0.0

        laser = self.sensors['laser']
        velocity = self.sensors['odom']['velocity']['linear']

        # Performance metric combining safety and progress
        safety_score = 1.0 if laser['min_distance'] > 0.5 else laser['min_distance'] / 0.5
        progress_score = max(0.0, velocity) if velocity > 0 else 0.0  # Only reward forward movement

        # Combine scores with weights
        performance = 0.6 * progress_score + 0.4 * safety_score
        return min(1.0, performance)  # Normalize to [0, 1]

    def update_learning(self):
        """Update learned behaviors based on experiences"""
        if len(self.experience_buffer) < 20:
            return

        # Get recent experiences
        recent_experiences = list(self.experience_buffer)[-20:]

        for exp in recent_experiences:
            state = exp['state']
            action = exp['action']
            performance = exp['performance']

            if performance > self.learning_params['performance_threshold']:
                # Good performance - reinforce this behavior
                if state not in self.learnt_behaviors:
                    self.learnt_behaviors[state] = action
                else:
                    # Update towards successful action
                    current_action = self.learnt_behaviors[state]
                    # Gradually adjust towards successful action
                    new_linear = (1 - self.learning_params['learning_rate']) * current_action.linear.x + \
                                self.learning_params['learning_rate'] * action.linear.x
                    new_angular = (1 - self.learning_params['learning_rate']) * current_action.angular.z + \
                                 self.learning_params['learning_rate'] * action.angular.z

                    new_action = Twist()
                    new_action.linear.x = new_linear
                    new_action.angular.z = new_angular
                    self.learnt_behaviors[state] = new_action

        # Adjust exploration rate based on performance
        if self.performance_history:
            recent_avg = np.mean(list(self.performance_history)[-20:])
            if recent_avg > 0.8:
                # High performance, reduce exploration
                self.learning_params['exploration_rate'] = max(0.05,
                    self.learning_params['exploration_rate'] * 0.95)
            elif recent_avg < 0.5:
                # Low performance, increase exploration
                self.learning_params['exploration_rate'] = min(0.3,
                    self.learning_params['exploration_rate'] * 1.05)

        # Adapt parameters based on experience
        self.adapt_parameters()

    def adapt_parameters(self):
        """Adapt control parameters based on performance"""
        if not self.performance_history:
            return

        recent_performance = np.mean(list(self.performance_history)[-20:]) if len(self.performance_history) >= 20 else 0.0

        # Adapt obstacle threshold based on collision frequency
        if self.collision_count > 0 and len(self.performance_history) > 50:
            collision_rate = self.collision_count / len(self.performance_history)
            if collision_rate > 0.05:  # Too many collisions
                # Increase safety margin
                self.adaptive_params['obstacle_threshold'] = min(1.5,
                    self.adaptive_params['obstacle_threshold'] * 1.05)
                self.adaptive_params['safety_margin'] = min(0.5,
                    self.adaptive_params['safety_margin'] * 1.1)
            elif recent_performance > 0.8 and collision_rate < 0.01:  # Good performance, few collisions
                # Can be more aggressive
                self.adaptive_params['obstacle_threshold'] = max(0.5,
                    self.adaptive_params['obstacle_threshold'] * 0.98)
                self.adaptive_params['safety_margin'] = max(0.15,
                    self.adaptive_params['safety_margin'] * 0.98)

        # Adapt turn sensitivity based on navigation success
        if recent_performance > 0.7:
            self.adaptive_params['turn_sensitivity'] = min(1.0,
                self.adaptive_params['turn_sensitivity'] * 1.01)
        else:
            self.adaptive_params['turn_sensitivity'] = max(0.3,
                self.adaptive_params['turn_sensitivity'] * 0.99)

    def save_learning(self):
        """Save learned behaviors and parameters to file"""
        learning_data = {
            'learnt_behaviors': self.learnt_behaviors,
            'adaptive_params': self.adaptive_params,
            'learning_params': self.learning_params,
            'experience_count': len(self.experience_buffer),
            'performance_stats': {
                'total_distance': self.total_distance_traveled,
                'collision_count': self.collision_count,
                'safety_violations': self.safety_violation_count
            }
        }

        try:
            with open('adaptive_control_learning.pkl', 'wb') as f:
                pickle.dump(learning_data, f)
            self.get_logger().info('Learning data saved successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to save learning data: {e}')

    def load_learning(self):
        """Load learned behaviors and parameters from file"""
        if os.path.exists('adaptive_control_learning.pkl'):
            try:
                with open('adaptive_control_learning.pkl', 'rb') as f:
                    learning_data = pickle.load(f)

                self.learnt_behaviors = learning_data.get('learnt_behaviors', {})
                self.adaptive_params = learning_data.get('adaptive_params', self.adaptive_params)
                self.learning_params = learning_data.get('learning_params', self.learning_params)

                stats = learning_data.get('performance_stats', {})
                self.total_distance_traveled = stats.get('total_distance', 0.0)
                self.collision_count = stats.get('collision_count', 0)
                self.safety_violation_count = stats.get('safety_violations', 0)

                self.get_logger().info(f'Learning data loaded: {learning_data.get("experience_count", 0)} experiences')
            except Exception as e:
                self.get_logger().error(f'Failed to load learning data: {e}')

    def destroy_node(self):
        """Save learning before shutdown"""
        self.save_learning()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    adaptive_system = AdaptiveControlSystem()

    try:
        rclpy.spin(adaptive_system)
    except KeyboardInterrupt:
        pass
    finally:
        adaptive_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 5: Coordination Manager

### Complete Coordination Manager
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import json
from enum import Enum
from collections import deque

class ControlLevel(Enum):
    REFLEX = 1
    SPINAL = 2
    MIDBRAIN = 3
    CORTICAL = 4

class CoordinationManager(Node):
    """
    Manages coordination between different control levels and resolves conflicts.
    """

    def __init__(self):
        super().__init__('coordination_manager')

        # Subscriptions for status from all levels
        self.high_status_sub = self.create_subscription(String, '/high_level_status', self.high_status_callback, 10)
        self.mid_status_sub = self.create_subscription(String, '/mid_level_status', self.mid_status_callback, 10)
        self.low_status_sub = self.create_subscription(String, '/low_level_status', self.low_status_callback, 10)
        self.reflex_status_sub = self.create_subscription(String, '/reflex_status', self.reflex_status_callback, 10)

        # Subscriptions for commands from all levels
        self.high_cmd_sub = self.create_subscription(String, '/high_level_commands', self.high_cmd_callback, 10)
        self.mid_cmd_sub = self.create_subscription(String, '/mid_level_commands', self.mid_cmd_callback, 10)
        self.low_cmd_sub = self.create_subscription(String, '/low_level_commands', self.low_cmd_callback, 10)
        self.reflex_cmd_sub = self.create_subscription(String, '/reflex_commands', self.reflex_cmd_callback, 10)

        # Emergency override subscription
        self.emergency_sub = self.create_subscription(Bool, '/emergency_override', self.emergency_callback, 10)

        # Motor command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Coordination timer
        self.coordination_timer = self.create_timer(0.01, self.coordination_loop)  # 100 Hz

        # Status tracking
        self.level_status = {
            ControlLevel.REFLEX: None,
            ControlLevel.SPINAL: None,
            ControlLevel.MIDBRAIN: None,
            ControlLevel.CORTICAL: None
        }

        # Command queues for each level
        self.command_queues = {
            ControlLevel.REFLEX: deque(maxlen=10),
            ControlLevel.SPINAL: deque(maxlen=10),
            ControlLevel.MIDBRAIN: deque(maxlen=10),
            ControlLevel.CORTICAL: deque(maxlen=10)
        }

        # Priority mapping
        self.level_priority = {
            ControlLevel.REFLEX: 100,      # Highest priority
            ControlLevel.SPINAL: 80,
            ControlLevel.MIDBRAIN: 60,
            ControlLevel.CORTICAL: 40      # Lowest priority
        }

        # Active commands
        self.active_commands = {}
        self.command_timestamps = {}

        # Emergency state
        self.emergency_override = False
        self.emergency_reason = ""
        self.emergency_start_time = None

        # Conflict tracking
        self.conflict_history = deque(maxlen=50)

        self.get_logger().info('Coordination Manager initialized')

    def high_status_callback(self, msg):
        try:
            self.level_status[ControlLevel.CORTICAL] = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid high-level status format: {e}')

    def mid_status_callback(self, msg):
        try:
            self.level_status[ControlLevel.MIDBRAIN] = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid mid-level status format: {e}')

    def low_status_callback(self, msg):
        try:
            self.level_status[ControlLevel.SPINAL] = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid low-level status format: {e}')

    def reflex_status_callback(self, msg):
        try:
            self.level_status[ControlLevel.REFLEX] = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid reflex status format: {e}')

    def high_cmd_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            cmd_data['level'] = ControlLevel.CORTICAL
            cmd_data['timestamp'] = self.get_clock().now().nanoseconds
            self.command_queues[ControlLevel.CORTICAL].append(cmd_data)
        except Exception as e:
            self.get_logger().warn(f'Invalid high-level command format: {e}')

    def mid_cmd_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            cmd_data['level'] = ControlLevel.MIDBRAIN
            cmd_data['timestamp'] = self.get_clock().now().nanoseconds
            self.command_queues[ControlLevel.MIDBRAIN].append(cmd_data)
        except Exception as e:
            self.get_logger().warn(f'Invalid mid-level command format: {e}')

    def low_cmd_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            cmd_data['level'] = ControlLevel.SPINAL
            cmd_data['timestamp'] = self.get_clock().now().nanoseconds
            self.command_queues[ControlLevel.SPINAL].append(cmd_data)
        except Exception as e:
            self.get_logger().warn(f'Invalid low-level command format: {e}')

    def reflex_cmd_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            cmd_data['level'] = ControlLevel.REFLEX
            cmd_data['timestamp'] = self.get_clock().now().nanoseconds
            self.command_queues[ControlLevel.REFLEX].append(cmd_data)
        except Exception as e:
            self.get_logger().warn(f'Invalid reflex command format: {e}')

    def emergency_callback(self, msg):
        """Handle emergency override"""
        self.emergency_override = msg.data
        if self.emergency_override:
            self.emergency_reason = "External Emergency Override"
            self.emergency_start_time = self.get_clock().now()
            self.get_logger().fatal('EMERGENCY OVERRIDE ACTIVATED')

            # Send emergency stop
            self.send_emergency_stop()
        else:
            self.emergency_reason = ""
            self.get_logger().info('EMERGENCY OVERRIDE CLEARED')

    def coordination_loop(self):
        """Main coordination loop"""
        if self.emergency_override:
            # Emergency has highest priority, no coordination needed
            return

        # Check for conflicts between levels
        conflicts = self.detect_conflicts()

        if conflicts:
            # Log conflicts
            for conflict in conflicts:
                self.conflict_history.append(conflict)
                self.get_logger().warn(f'Conflict detected: {conflict}')

            # Resolve conflicts based on priority
            self.resolve_conflicts(conflicts)

        # Select and execute the highest priority command
        self.execute_highest_priority_command()

        # Monitor system health
        self.monitor_system_health()

    def detect_conflicts(self):
        """Detect conflicts between different control levels"""
        conflicts = []

        # Check for command conflicts (different levels issuing conflicting commands)
        active_levels = []
        for level, queue in self.command_queues.items():
            if queue:
                active_levels.append(level)

        # If multiple levels have commands, check for conflicts
        if len(active_levels) > 1:
            # For this example, we'll consider any simultaneous commands as potential conflicts
            for level in active_levels:
                conflicts.append({
                    'type': 'simultaneous_commands',
                    'levels': [l.name for l in active_levels],
                    'active_level': level.name,
                    'timestamp': self.get_clock().now().nanoseconds
                })

        # Check for status inconsistencies
        active_statuses = [level for level, status in self.level_status.items() if status]
        if len(active_statuses) > 1:
            # Check if statuses indicate conflicting states
            for level in active_statuses:
                status = self.level_status[level]
                # Example: if one level indicates "STOP" while another indicates "MOVE"
                if status and isinstance(status, dict):
                    command_status = status.get('command_status', '')
                    if command_status == 'STOP' and any(
                        self.level_status[other_level] and
                        self.level_status[other_level].get('command_status', '') == 'MOVE'
                        for other_level in active_statuses if other_level != level
                    ):
                        conflicts.append({
                            'type': 'command_conflict',
                            'levels': [level.name, next(l.name for l in active_statuses
                                                      if l != level and self.level_status[l].get('command_status', '') == 'MOVE')],
                            'description': f'{level.name} wants STOP but others want MOVE',
                            'timestamp': self.get_clock().now().nanoseconds
                        })

        return conflicts

    def resolve_conflicts(self, conflicts):
        """Resolve detected conflicts based on priority"""
        for conflict in conflicts:
            if conflict['type'] == 'simultaneous_commands':
                # Resolve by priority - only execute highest priority command
                active_levels = [ControlLevel[name] for name in conflict['levels']]
                highest_priority_level = max(active_levels, key=lambda l: self.level_priority[l])

                # Clear commands from lower priority levels
                for level in active_levels:
                    if level != highest_priority_level and self.command_queues[level]:
                        self.command_queues[level].clear()
                        self.get_logger().info(f'Cleared commands from {level.name} due to priority conflict')

            elif conflict['type'] == 'command_conflict':
                # Specific resolution for command conflicts
                self.get_logger().info(f'Resolving command conflict: {conflict["description"]}')

    def execute_highest_priority_command(self):
        """Execute the command from the highest priority level that has a command"""
        # Find the highest priority level with a command in its queue
        available_levels = [level for level, queue in self.command_queues.items() if queue]

        if not available_levels:
            # No commands available, send stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return

        # Find the highest priority level
        highest_priority_level = max(available_levels, key=lambda l: self.level_priority[l])

        # Get the command from the highest priority level
        if self.command_queues[highest_priority_level]:
            cmd_data = self.command_queues[highest_priority_level][0]  # Peek at first command
            self.command_queues[highest_priority_level].popleft()  # Remove it

            # Convert command data to Twist message
            cmd = Twist()
            cmd.linear.x = cmd_data.get('linear_x', 0.0)
            cmd.linear.y = cmd_data.get('linear_y', 0.0)
            cmd.linear.z = cmd_data.get('linear_z', 0.0)
            cmd.angular.x = cmd_data.get('angular_x', 0.0)
            cmd.angular.y = cmd_data.get('angular_y', 0.0)
            cmd.angular.z = cmd_data.get('angular_z', 0.0)

            # Apply safety limits
            cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))
            cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

            # Publish the command
            self.cmd_vel_pub.publish(cmd)

            # Log execution
            self.get_logger().debug(f'Executing command from {highest_priority_level.name}: '
                                  f'linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')

    def send_emergency_stop(self):
        """Send emergency stop command"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Emergency stop command published')

    def monitor_system_health(self):
        """Monitor the health of the coordination system"""
        # Check for stale commands
        current_time = self.get_clock().now().nanoseconds
        timeout_ns = 1e9  # 1 second timeout

        for level, queue in self.command_queues.items():
            if queue:
                oldest_cmd_time = queue[0].get('timestamp', current_time)
                if (current_time - oldest_cmd_time) > timeout_ns:
                    # Clear stale commands
                    queue.clear()
                    self.get_logger().warn(f'Cleared stale commands from {level.name}')

        # Check for status staleness
        for level, status in self.level_status.items():
            if status and isinstance(status, dict):
                status_time = status.get('timestamp', current_time)
                if (current_time - status_time) > timeout_ns:
                    self.level_status[level] = None
                    self.get_logger().warn(f'Cleared stale status from {level.name}')

def main(args=None):
    rclpy.init(args=args)
    coordination_manager = CoordinationManager()

    try:
        rclpy.spin(coordination_manager)
    except KeyboardInterrupt:
        pass
    finally:
        coordination_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 6: Bio-Inspired Neural Network (Advanced)

### Complete Bio-Inspired Neural Network System
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from collections import defaultdict
import random

class NeuralNode:
    """
    A single neural node in the bio-inspired network.
    """
    def __init__(self, node_id, node_type="interneuron", activation_threshold=0.5):
        self.node_id = node_id
        self.node_type = node_type  # sensory, interneuron, motor
        self.activation_threshold = activation_threshold
        self.current_activation = 0.0
        self.connections = {}  # {target_node_id: weight}
        self.input_sum = 0.0
        self.last_update_time = 0
        self.activation_history = []
        self.max_history = 100

    def add_connection(self, target_node_id, weight):
        """Add a weighted connection to another node"""
        self.connections[target_node_id] = weight

    def process_input(self, input_value, time_step):
        """Process input and update activation"""
        self.input_sum += input_value

        # Apply activation function (sigmoid)
        self.current_activation = 1 / (1 + math.exp(-self.input_sum))

        # Store in history
        self.activation_history.append((time_step, self.current_activation))
        if len(self.activation_history) > self.max_history:
            self.activation_history.pop(0)

        # Decay input sum over time
        self.input_sum *= 0.9  # 10% decay per time step

        return self.current_activation

    def get_output(self):
        """Get output if activation exceeds threshold"""
        return self.current_activation if self.current_activation > self.activation_threshold else 0.0

class NeuralNetworkController(Node):
    """
    Bio-inspired neural network controller for robotic control.
    """

    def __init__(self):
        super().__init__('neural_network_controller')

        # Sensor subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Motor command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Neural network timer
        self.network_timer = self.create_timer(0.02, self.neural_processing_loop)  # 50 Hz

        # Initialize neural network
        self.initialize_neural_network()

        # Sensor data storage
        self.sensors = {
            'laser': None,
            'imu': None
        }

        # Learning parameters
        self.learning_rate = 0.01
        self.plasticity_enabled = True

        self.get_logger().info('Neural Network Controller initialized')

    def initialize_neural_network(self):
        """Initialize the neural network with a bio-inspired structure"""
        self.neural_nodes = {}

        # Create sensory neurons for laser inputs (one per laser sector)
        laser_sectors = 8  # Divide laser scan into 8 sectors
        for i in range(laser_sectors):
            node_id = f"sensory_laser_{i}"
            self.neural_nodes[node_id] = NeuralNode(node_id, "sensory", activation_threshold=0.3)

        # Create sensory neurons for IMU
        self.neural_nodes['sensory_imu_balance'] = NeuralNode('sensory_imu_balance', "sensory", activation_threshold=0.2)

        # Create interneurons for processing
        interneuron_count = 12
        for i in range(interneuron_count):
            node_id = f"interneuron_{i}"
            self.neural_nodes[node_id] = NeuralNode(node_id, "interneuron", activation_threshold=0.4)

        # Create motor neurons
        self.neural_nodes['motor_linear'] = NeuralNode('motor_linear', "motor", activation_threshold=0.3)
        self.neural_nodes['motor_angular'] = NeuralNode('motor_angular', "motor", activation_threshold=0.3)

        # Create initial connections (random weights)
        self.create_initial_connections()

        self.get_logger().info(f'Neural network initialized with {len(self.neural_nodes)} nodes')

    def create_initial_connections(self):
        """Create initial connections between neurons"""
        # Connect sensory neurons to interneurons
        for i in range(8):  # Laser sensory neurons
            sensory_id = f"sensory_laser_{i}"
            for j in range(6):  # Connect to first 6 interneurons
                interneuron_id = f"interneuron_{j}"
                weight = random.uniform(-0.5, 0.5)  # Random initial weight
                self.neural_nodes[sensory_id].add_connection(interneuron_id, weight)

        # Connect IMU sensory to interneurons
        for j in range(6, 12):  # Connect to last 6 interneurons
            interneuron_id = f"interneuron_{j}"
            weight = random.uniform(-0.3, 0.3)
            self.neural_nodes['sensory_imu_balance'].add_connection(interneuron_id, weight)

        # Connect interneurons to each other (recurrent connections)
        for i in range(12):
            interneuron_id = f"interneuron_{i}"
            # Connect to other interneurons
            for j in range(12):
                if i != j:  # No self-connections
                    target_id = f"interneuron_{j}"
                    weight = random.uniform(-0.2, 0.2)
                    # Only create some connections to avoid full connectivity
                    if random.random() < 0.3:  # 30% connectivity
                        self.neural_nodes[interneuron_id].add_connection(target_id, weight)

        # Connect interneurons to motor neurons
        for i in range(6):  # First 6 interneurons to linear motor
            interneuron_id = f"interneuron_{i}"
            self.neural_nodes[interneuron_id].add_connection('motor_linear', random.uniform(-0.5, 0.5))

        for i in range(6, 12):  # Last 6 interneurons to angular motor
            interneuron_id = f"interneuron_{i}"
            self.neural_nodes[interneuron_id].add_connection('motor_angular', random.uniform(-0.5, 0.5))

    def laser_callback(self, msg):
        """Process laser scan data and activate sensory neurons"""
        ranges = np.array(msg.ranges)
        # Filter invalid readings
        ranges[ranges == float('inf')] = 3.5
        ranges[np.isnan(ranges)] = 3.5

        # Divide laser scan into 8 sectors
        sector_size = len(ranges) // 8
        for i in range(8):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size if i < 7 else len(ranges)  # Handle remainder
            sector_ranges = ranges[start_idx:end_idx]

            # Calculate average distance in this sector (inverse for obstacle proximity)
            avg_distance = np.mean(sector_ranges) if len(sector_ranges) > 0 else 3.5
            # Closer obstacles result in higher activation
            activation = max(0.0, (3.5 - avg_distance) / 3.5)  # Normalize to [0, 1]

            node_id = f"sensory_laser_{i}"
            if node_id in self.neural_nodes:
                self.neural_nodes[node_id].process_input(activation, self.get_clock().now().nanoseconds)

    def imu_callback(self, msg):
        """Process IMU data and activate balance sensory neuron"""
        # Extract orientation for balance information
        q = msg.orientation
        # Simple roll/pitch extraction (simplified)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        pitch = math.atan2(siny_cosp, cosy_cosp)

        # Activation based on tilt magnitude
        tilt_magnitude = abs(roll) + abs(pitch)
        activation = min(1.0, tilt_magnitude)  # Cap at 1.0

        node_id = 'sensory_imu_balance'
        if node_id in self.neural_nodes:
            self.neural_nodes[node_id].process_input(activation, self.get_clock().now().nanoseconds)

    def neural_processing_loop(self):
        """Main neural network processing loop"""
        if not all(self.sensors.values()):
            return

        current_time = self.get_clock().now().nanoseconds

        # Propagate activations through the network
        self.propagate_activations(current_time)

        # Get motor outputs
        linear_output = self.neural_nodes['motor_linear'].get_output()
        angular_output = self.neural_nodes['motor_angular'].get_output()

        # Convert to motor command
        cmd = Twist()
        cmd.linear.x = linear_output * 0.8  # Scale to reasonable speed
        cmd.angular.z = angular_output * 1.0  # Scale to reasonable turning rate

        # Apply safety limits
        cmd.linear.x = max(-0.8, min(0.8, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Apply learning if enabled
        if self.plasticity_enabled:
            self.apply_hebbian_learning()

        self.get_logger().debug(f'Neural output - linear: {cmd.linear.x:.3f}, angular: {cmd.angular.z:.3f}')

    def propagate_activations(self, time_step):
        """Propagate activations through the network"""
        # This is a simplified propagation - in a real implementation,
        # you might want to use a more sophisticated method

        # Process in layers: sensory -> interneurons -> motor
        # First, collect outputs from sensory neurons
        sensory_outputs = {}
        for node_id, node in self.neural_nodes.items():
            if node.node_type == "sensory":
                sensory_outputs[node_id] = node.get_output()

        # Then update interneurons based on sensory input and connections
        interneuron_updates = {}
        for node_id, node in self.neural_nodes.items():
            if node.node_type == "interneuron":
                total_input = 0.0

                # Sum inputs from sensory neurons
                for src_id, output in sensory_outputs.items():
                    if src_id in node.connections:
                        total_input += output * node.connections[src_id]

                # Sum inputs from other interneurons
                for other_id, other_node in self.neural_nodes.items():
                    if (other_node.node_type == "interneuron" and
                        other_id in node.connections and
                        other_id in interneuron_updates):
                        total_input += interneuron_updates[other_id] * node.connections[other_id]

                # Update this interneuron
                node.process_input(total_input, time_step)
                interneuron_updates[node_id] = node.get_output()

        # Finally, update motor neurons
        for node_id, node in self.neural_nodes.items():
            if node.node_type == "motor":
                total_input = 0.0

                # Sum inputs from interneurons
                for src_id, output in interneuron_updates.items():
                    if src_id in node.connections:
                        total_input += output * node.connections[src_id]

                node.process_input(total_input, time_step)

    def apply_hebbian_learning(self):
        """Apply Hebbian learning rule: "neurons that fire together, wire together" """
        # Simplified Hebbian learning
        for node_id, node in self.neural_nodes.items():
            for target_id in node.connections:
                if target_id in self.neural_nodes:
                    target_node = self.neural_nodes[target_id]

                    # Get current activations
                    pre_activation = node.get_output()
                    post_activation = target_node.get_output()

                    # Apply Hebbian rule: weight change proportional to product of activations
                    weight_change = self.learning_rate * pre_activation * post_activation
                    node.connections[target_id] += weight_change

                    # Constrain weights to reasonable range
                    node.connections[target_id] = max(-1.0, min(1.0, node.connections[target_id]))

    def get_network_state(self):
        """Get the current state of the neural network"""
        state = {
            'node_count': len(self.neural_nodes),
            'sensory_nodes': [],
            'interneurons': [],
            'motor_nodes': [],
            'total_connections': 0
        }

        for node_id, node in self.neural_nodes.items():
            state[f'{node.node_type}_nodes'].append({
                'id': node_id,
                'activation': node.current_activation,
                'connections': len(node.connections)
            })
            state['total_connections'] += len(node.connections)

        return state

def main(args=None):
    rclpy.init(args=args)
    neural_controller = NeuralNetworkController()

    try:
        rclpy.spin(neural_controller)
    except KeyboardInterrupt:
        pass
    finally:
        neural_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementation Guide

### For Exercise 1 (Basic Reflex System):
1. Create a high-frequency timer for immediate response
2. Implement obstacle detection with safety threshold
3. Create command override mechanism
4. Test with various obstacle scenarios

### For Exercise 2 (Hierarchical Control):
1. Design three-level architecture with clear interfaces
2. Implement different timer frequencies for each level
3. Create communication protocols between levels
4. Implement conflict resolution mechanisms

### For Exercise 3 (Sensorimotor Integration):
1. Set up multiple sensor subscriptions
2. Implement data synchronization mechanisms
3. Create state estimation from fused data
4. Generate appropriate motor commands

### For Exercise 4 (Adaptive Control):
1. Implement basic behavior with adjustable parameters
2. Create performance evaluation function
3. Store and analyze experience data
4. Implement learning algorithm

### For Exercise 5 (Coordination Manager):
1. Design conflict detection mechanisms
2. Implement priority resolution
3. Create status monitoring system
4. Test with various conflict scenarios

### For Exercise 6 (Bio-Inspired Neural Network):
1. Design network architecture with appropriate neuron types
2. Implement activation propagation
3. Create learning mechanisms
4. Test with complex behavioral tasks

## Best Practices

1. **Modularity**: Keep each control level modular and independently testable
2. **Safety**: Always implement safety checks and emergency overrides
3. **Performance**: Use appropriate timer frequencies for each control level
4. **Communication**: Implement clear and efficient communication between levels
5. **Monitoring**: Include status reporting and health monitoring
6. **Adaptation**: Implement learning with appropriate exploration vs exploitation balance
7. **Scalability**: Design systems that can scale to more complex networks
8. **Documentation**: Document all parameters and design decisions

These solutions provide complete implementations for each exercise, demonstrating how to create bio-inspired robotic nervous system patterns using ROS 2. Each solution builds on the previous ones, showing progressive complexity from basic reflexes to sophisticated neural network control.