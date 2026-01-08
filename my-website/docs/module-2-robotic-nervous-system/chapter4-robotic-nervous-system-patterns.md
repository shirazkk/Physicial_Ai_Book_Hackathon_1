# Chapter 4: Implementing Robotic Nervous System Patterns

## Learning Objectives
By the end of this chapter, readers will be able to:
- Understand biological nervous system principles and their application to robotics
- Design distributed control architectures for humanoid robots using ROS 2
- Implement reflex-based control systems for autonomous robot behavior
- Create hierarchical control structures that mimic biological neural organization
- Apply sensorimotor integration techniques for responsive robot behavior
- Implement adaptive control systems that learn and adjust to environmental changes

## Prerequisites
- Completion of Chapter 1: Understanding ROS 2 Architecture and Communication Patterns
- Completion of Chapter 2: Bridging Python-based AI Agents to Robot Controllers
- Completion of Chapter 3: Understanding URDF for Humanoid Robot Description and Control
- Understanding of basic neuroscience concepts
- Knowledge of distributed systems and control theory

## Introduction
The concept of a robotic nervous system draws inspiration from biological neural networks to create sophisticated control architectures for humanoid robots. Just as biological nervous systems coordinate complex behaviors through distributed processing, robotic nervous systems enable coordinated control of multiple actuators and sensors through distributed ROS 2 nodes that communicate in patterns similar to neural networks.

This chapter explores how to implement nervous system-inspired patterns in robotic systems using ROS 2, creating architectures that exhibit properties like reflexive responses, hierarchical control, and adaptive behavior. These patterns enable robots to respond quickly to environmental stimuli while maintaining coordinated behavior across multiple subsystems.

## 1. Biological Nervous System Principles

### 1.1 Neural Network Organization
Biological nervous systems are organized hierarchically with multiple levels of control:
- **Reflex Arcs**: Immediate responses to stimuli without brain involvement
- **Spinal Cord Processing**: Local processing of sensorimotor information
- **Brain Stem**: Basic life-sustaining functions and arousal
- **Cerebral Cortex**: Higher-level planning and decision making

In robotics, we can implement similar hierarchies using ROS 2 nodes:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
import numpy as np

class RoboticNervousSystem(Node):
    """
    A hierarchical control system inspired by biological nervous systems.
    Implements reflex arcs, spinal cord processing, and higher-level planning.
    """

    def __init__(self):
        super().__init__('robotic_nervous_system')

        # Reflex arc level: immediate responses to dangerous stimuli
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        # Spinal cord level: local sensorimotor processing
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Motor command publishers for different control levels
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Reflex control timer (high frequency for immediate responses)
        self.reflex_timer = self.create_timer(0.01, self.reflex_control)  # 100 Hz

        # Spinal cord processing timer (medium frequency)
        self.spinal_timer = self.create_timer(0.05, self.spinal_processing)  # 20 Hz

        # Higher-level planning timer (lower frequency)
        self.planning_timer = self.create_timer(0.1, self.planning_control)  # 10 Hz

        # Internal state
        self.laser_data = None
        self.joint_states = None
        self.emergency_stop = False

        # Reflex thresholds
        self.collision_threshold = 0.3  # meters
        self.approach_threshold = 0.8   # meters

        self.get_logger().info('Robotic Nervous System initialized')

    def laser_callback(self, msg):
        """Process laser scan data - immediate sensory input"""
        self.laser_data = np.array(msg.ranges)
        # Filter invalid readings
        self.laser_data[self.laser_data == float('inf')] = 3.5
        self.laser_data[np.isnan(self.laser_data)] = 3.5

    def joint_state_callback(self, msg):
        """Process joint state data - proprioceptive input"""
        self.joint_states = msg

    def reflex_control(self):
        """
        Reflex arc level control - immediate responses to dangerous situations.
        This mimics the spinal cord's ability to respond to threats without brain involvement.
        """
        if self.laser_data is None or self.emergency_stop:
            return

        # Check for immediate collision danger (reflex response)
        min_distance = np.min(self.laser_data) if self.laser_data.size > 0 else float('inf')

        if min_distance < self.collision_threshold:
            # Emergency stop reflex - immediate halt
            self.emergency_stop = True
            self.get_logger().warn('REFLEX: Emergency stop activated - collision imminent!')

            # Publish emergency stop command
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            # Also send joint stop commands
            joint_stop = JointState()
            joint_stop.position = [0.0] * len(self.joint_states.name) if self.joint_states else []
            self.joint_cmd_pub.publish(joint_stop)

            # Emergency stop timer to reset after safety period
            self.create_timer(1.0, self.reset_emergency_stop)

    def reset_emergency_stop(self):
        """Reset emergency stop after safety period"""
        self.emergency_stop = False
        self.get_logger().info('REFLEX: Emergency stop reset')

    def spinal_processing(self):
        """
        Spinal cord level processing - local sensorimotor coordination.
        Processes sensory information and generates coordinated motor responses.
        """
        if self.loint_data is None or self.emergency_stop:
            return

        # Approach reflex - slow down when approaching obstacles
        min_distance = np.min(self.laser_data) if self.laser_data.size > 0 else float('inf')

        if min_distance < self.approach_threshold and min_distance > self.collision_threshold:
            # Approach reflex - reduce speed proportionally to obstacle proximity
            speed_reduction = 1.0 - (min_distance - self.collision_threshold) / (self.approach_threshold - self.collision_threshold)

            # Send speed reduction command
            cmd = Twist()
            cmd.linear.x = 0.5 * (1.0 - speed_reduction)  # Reduce forward speed
            cmd.angular.z = 0.0  # Maintain current heading
            self.cmd_vel_pub.publish(cmd)

    def planning_control(self):
        """
        Higher-level planning - cognitive control similar to cortical processing.
        Makes decisions based on goals, environment, and internal state.
        """
        if self.laser_data is None or self.emergency_stop:
            return

        # Example: Goal-oriented navigation with obstacle awareness
        min_distance = np.min(self.laser_data) if self.laser_data.size > 0 else float('inf')

        cmd = Twist()

        if min_distance > self.approach_threshold:
            # Clear path - move toward goal
            cmd.linear.x = 0.8  # Move forward at higher speed
            cmd.angular.z = 0.0  # Maintain heading
        elif min_distance > self.collision_threshold:
            # Moderate obstacle - navigate around
            cmd.linear.x = 0.3  # Slower forward motion
            # Simple obstacle avoidance (turn away from closest obstacle)
            if self.laser_data.size > 0:
                closest_idx = np.argmin(self.laser_data)
                if closest_idx < len(self.laser_data) / 2:
                    # Obstacle on left - turn right
                    cmd.angular.z = -0.5
                else:
                    # Obstacle on right - turn left
                    cmd.angular.z = 0.5

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    nervous_system = RoboticNervousSystem()

    try:
        rclpy.spin(nervous_system)
    except KeyboardInterrupt:
        pass
    finally:
        nervous_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.2 Sensorimotor Integration
Biological systems integrate sensory information from multiple modalities to generate coordinated motor responses. In robotics, this translates to fusing data from various sensors to control actuators effectively.

```python
class SensorimotorIntegration(Node):
    """
    Implements sensorimotor integration similar to biological systems.
    Combines multiple sensory inputs to generate coordinated motor outputs.
    """

    def __init__(self):
        super().__init__('sensorimotor_integration')

        # Multiple sensor inputs
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Motor command outputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Integration timer
        self.integration_timer = self.create_timer(0.02, self.sensorimotor_loop)  # 50 Hz

        # Internal state
        self.sensors = {
            'laser': None,
            'imu': None,
            'odom': None,
            'joints': None
        }

        # Integration weights for different sensory modalities
        self.integration_weights = {
            'obstacle_avoidance': 0.4,
            'balance': 0.3,
            'navigation': 0.3
        }

        self.get_logger().info('Sensorimotor Integration Node initialized')

    def laser_callback(self, msg):
        self.sensors['laser'] = msg

    def imu_callback(self, msg):
        self.sensors['imu'] = msg

    def odom_callback(self, msg):
        self.sensors['odom'] = msg

    def joint_callback(self, msg):
        self.sensors['joints'] = msg

    def sensorimotor_loop(self):
        """Main sensorimotor integration loop"""
        if not all(self.sensors.values()):
            return

        # Integrate different sensory modalities
        obstacle_cmd = self.process_obstacle_avoidance()
        balance_cmd = self.process_balance_control()
        navigation_cmd = self.process_navigation()

        # Weighted integration of commands
        final_cmd = Twist()
        final_cmd.linear.x = (
            self.integration_weights['obstacle_avoidance'] * obstacle_cmd['linear_x'] +
            self.integration_weights['balance'] * balance_cmd['linear_x'] +
            self.integration_weights['navigation'] * navigation_cmd['linear_x']
        )

        final_cmd.angular.z = (
            self.integration_weights['obstacle_avoidance'] * obstacle_cmd['angular_z'] +
            self.integration_weights['balance'] * balance_cmd['angular_z'] +
            self.integration_weights['navigation'] * navigation_cmd['angular_z']
        )

        # Apply command limits
        final_cmd.linear.x = max(-1.0, min(1.0, final_cmd.linear.x))
        final_cmd.angular.z = max(-1.0, min(1.0, final_cmd.angular.z))

        self.cmd_vel_pub.publish(final_cmd)

    def process_obstacle_avoidance(self):
        """Process laser data for obstacle avoidance"""
        laser_data = np.array(self.sensors['laser'].ranges)
        laser_data[laser_data == float('inf')] = 3.5
        laser_data[np.isnan(laser_data)] = 3.5

        min_distance = np.min(laser_data) if laser_data.size > 0 else float('inf')

        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        if min_distance < 0.5:
            # Emergency avoidance
            cmd['linear_x'] = -0.3  # Back up
            # Turn away from closest obstacle
            closest_idx = np.argmin(laser_data)
            cmd['angular_z'] = 0.8 if closest_idx < len(laser_data) / 2 else -0.8
        elif min_distance < 1.0:
            # Normal avoidance
            cmd['linear_x'] = 0.2  # Slow forward
            # Gentle turn away
            closest_idx = np.argmin(laser_data)
            cmd['angular_z'] = 0.4 if closest_idx < len(laser_data) / 2 else -0.4
        else:
            # Clear path
            cmd['linear_x'] = 0.6  # Normal forward speed
            cmd['angular_z'] = 0.0  # No turn

        return cmd

    def process_balance_control(self):
        """Process IMU data for balance control"""
        imu = self.sensors['imu']

        # Extract roll and pitch from quaternion
        import math
        q = imu.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        pitch = math.atan2(siny_cosp, cosy_cosp)

        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        # Balance correction based on tilt
        if abs(pitch) > 0.1:  # Tilted forward/backward
            cmd['linear_x'] = -pitch * 2.0  # Correct by moving opposite to tilt
        if abs(roll) > 0.1:   # Tilted side to side
            cmd['angular_z'] = -roll * 2.0  # Correct by turning opposite to tilt

        return cmd

    def process_navigation(self):
        """Process odometry for navigation"""
        odom = self.sensors['odom']

        # Example: Simple goal-oriented navigation
        # In a real system, this would involve path planning
        cmd = {'linear_x': 0.5, 'angular_z': 0.0}  # Default forward motion

        # This would be enhanced with goal-seeking behavior
        return cmd
```

## 2. Distributed Control Architectures

### 2.1 Node-Based Nervous System
In ROS 2, we can create a distributed nervous system using multiple interconnected nodes that communicate via topics, services, and actions:

```python
# Brain Node - Higher-level cognitive functions
class BrainNode(Node):
    """
    Higher-level cognitive control node.
    Makes decisions based on integrated sensory information and goals.
    """

    def __init__(self):
        super().__init__('brain_node')

        # Subscribe to integrated sensor data
        self.sensory_sub = self.create_subscription(
            String, '/integrated_sensory_data', self.sensory_callback, 10)

        # Subscribe to goals
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal', self.goal_callback, 10)

        # Publish high-level commands
        self.high_level_cmd_pub = self.create_publisher(
            String, '/high_level_commands', 10)

        # Service for requesting planning
        self.plan_service = self.create_service(
            Trigger, '/request_plan', self.plan_callback)

        self.sensory_data = {}
        self.current_goal = None
        self.current_state = "IDLE"

        self.brain_timer = self.create_timer(0.1, self.cognitive_loop)

        self.get_logger().info('Brain Node initialized')

    def sensory_callback(self, msg):
        """Process integrated sensory information"""
        try:
            self.sensory_data = eval(msg.data)  # In practice, use json.loads
        except:
            self.get_logger().warn('Invalid sensory data format')

    def goal_callback(self, msg):
        """Process new goal"""
        self.current_goal = msg

    def cognitive_loop(self):
        """Main cognitive processing loop"""
        if not self.sensory_data:
            return

        # Decision making based on sensory data and goals
        decision = self.make_decision()

        if decision:
            cmd_msg = String()
            cmd_msg.data = str(decision)
            self.high_level_cmd_pub.publish(cmd_msg)

    def make_decision(self):
        """Make high-level decisions based on sensory data and goals"""
        if not self.current_goal:
            return {"action": "wait", "reason": "no_goal"}

        # Example decision logic
        if self.sensory_data.get('obstacle_distance', float('inf')) < 0.3:
            return {"action": "avoid_obstacle", "reason": "immediate_danger"}
        elif self.sensory_data.get('goal_distance', float('inf')) < 0.5:
            return {"action": "goal_reached", "reason": "at_goal"}
        else:
            return {"action": "navigate", "reason": "toward_goal"}

    def plan_callback(self, request, response):
        """Plan a path to the current goal"""
        if self.current_goal:
            # In a real implementation, this would do actual path planning
            response.success = True
            response.message = "Path planned successfully"
        else:
            response.success = False
            response.message = "No goal set"
        return response

# Spinal Cord Node - Local reflexes and coordination
class SpinalCordNode(Node):
    """
    Local processing node for reflexes and immediate responses.
    Similar to spinal cord processing in biological systems.
    """

    def __init__(self):
        super().__init__('spinal_cord_node')

        # Subscribe to immediate sensory data
        self.immediate_sensory_sub = self.create_subscription(
            LaserScan, '/scan', self.immediate_sensory_callback, 10)

        # Subscribe to high-level commands
        self.high_level_sub = self.create_subscription(
            String, '/high_level_commands', self.high_level_callback, 10)

        # Publish immediate motor commands
        self.motor_cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.immediate_sensory_data = None
        self.high_level_command = None

        # High-frequency reflex processing
        self.reflex_timer = self.create_timer(0.01, self.reflex_processing)

        self.get_logger().info('Spinal Cord Node initialized')

    def immediate_sensory_callback(self, msg):
        """Process immediate sensory data for reflexes"""
        self.immediate_sensory_data = msg

    def high_level_callback(self, msg):
        """Process high-level commands"""
        self.high_level_command = msg

    def reflex_processing(self):
        """Process immediate reflexes"""
        if self.immediate_sensory_data is None:
            return

        # Immediate collision avoidance reflex
        ranges = np.array(self.immediate_sensory_data.ranges)
        ranges[ranges == float('inf')] = 3.5
        ranges[np.isnan(ranges)] = 3.5

        min_distance = np.min(ranges) if ranges.size > 0 else float('inf')

        cmd = Twist()

        if min_distance < 0.3:  # Immediate danger
            # Emergency stop reflex
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif min_distance < 0.8:  # Approaching obstacle
            # Approach reflex - slow down
            cmd.linear.x = max(0.1, min_distance * 0.5)
            # Turn away from closest obstacle
            closest_idx = np.argmin(ranges)
            cmd.angular.z = 0.5 if closest_idx < len(ranges) / 2 else -0.5
        else:
            # No immediate threat - follow high-level command
            if self.high_level_command:
                # In a real system, interpret high-level commands
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
            else:
                # Default behavior
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0

        self.motor_cmd_pub.publish(cmd)
```

### 2.2 Communication Patterns
The nervous system uses specific communication patterns that can be implemented with ROS 2:

```python
# Sensory Integration Hub
class SensoryIntegrationHub(Node):
    """
    Hub for integrating multiple sensory inputs.
    Similar to how sensory information is integrated in the brainstem/thalamus.
    """

    def __init__(self):
        super().__init__('sensory_integration_hub')

        # Multiple sensor subscriptions
        self.sensors = {}
        self.sensors['laser'] = self.create_subscription(LaserScan, '/scan',
                                                         lambda msg: self.sensor_callback('laser', msg), 10)
        self.sensors['imu'] = self.create_subscription(Imu, '/imu',
                                                       lambda msg: self.sensor_callback('imu', msg), 10)
        self.sensors['odom'] = self.create_subscription(Odometry, '/odom',
                                                        lambda msg: self.sensor_callback('odom', msg), 10)

        # Integrated data publisher
        self.integrated_pub = self.create_publisher(String, '/integrated_sensory_data', 10)

        # Timestamped sensory data storage
        self.sensory_buffer = {}
        self.max_buffer_size = 10

        # Integration timer
        self.integration_timer = self.create_timer(0.05, self.integrate_sensory_data)

        self.get_logger().info('Sensory Integration Hub initialized')

    def sensor_callback(self, sensor_type, msg):
        """Receive and buffer sensory data"""
        timestamp = self.get_clock().now().nanoseconds
        self.sensory_buffer[sensor_type] = {
            'timestamp': timestamp,
            'data': msg
        }

        # Maintain buffer size
        if len(self.sensory_buffer) > self.max_buffer_size:
            # Remove oldest entries
            oldest_key = min(self.sensory_buffer.keys(),
                           key=lambda k: self.sensory_buffer[k]['timestamp'])
            del self.sensory_buffer[oldest_key]

    def integrate_sensory_data(self):
        """Integrate and publish sensory data"""
        if not self.sensory_buffer:
            return

        # Create integrated sensory representation
        integrated_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'sensors_present': list(self.sensory_buffer.keys())
        }

        # Extract and process key information from each sensor
        if 'laser' in self.sensory_buffer:
            laser_data = self.sensory_buffer['laser']['data']
            ranges = np.array(laser_data.ranges)
            ranges[ranges == float('inf')] = 3.5
            ranges[np.isnan(ranges)] = 3.5
            integrated_data['obstacle_distance'] = float(np.min(ranges)) if ranges.size > 0 else float('inf')
            integrated_data['obstacle_direction'] = float(np.argmin(ranges)) if ranges.size > 0 else 0

        if 'imu' in self.sensory_buffer:
            imu_data = self.sensory_buffer['imu']['data']
            # Process orientation and acceleration data
            integrated_data['orientation'] = {
                'x': imu_data.orientation.x,
                'y': imu_data.orientation.y,
                'z': imu_data.orientation.z,
                'w': imu_data.orientation.w
            }
            integrated_data['linear_acceleration'] = {
                'x': imu_data.linear_acceleration.x,
                'y': imu_data.linear_acceleration.y,
                'z': imu_data.linear_acceleration.z
            }

        if 'odom' in self.sensory_buffer:
            odom_data = self.sensory_buffer['odom']['data']
            integrated_data['position'] = {
                'x': odom_data.pose.pose.position.x,
                'y': odom_data.pose.pose.position.y,
                'z': odom_data.pose.pose.position.z
            }
            integrated_data['velocity'] = {
                'linear': {
                    'x': odom_data.twist.twist.linear.x,
                    'y': odom_data.twist.twist.linear.y,
                    'z': odom_data.twist.twist.linear.z
                },
                'angular': {
                    'x': odom_data.twist.twist.angular.x,
                    'y': odom_data.twist.twist.angular.y,
                    'z': odom_data.twist.twist.angular.z
                }
            }

        # Publish integrated data
        msg = String()
        msg.data = str(integrated_data)
        self.integrated_pub.publish(msg)
```

## 3. Reflex-Based Control Systems

### 3.1 Implementing Reflex Arcs
Reflex arcs provide immediate responses to stimuli without higher-level processing:

```python
class ReflexSystem(Node):
    """
    Implementation of reflex arcs for immediate responses.
    Mimics monosynaptic and polysynaptic reflexes in biological systems.
    """

    def __init__(self):
        super().__init__('reflex_system')

        # Sensory inputs for different reflexes
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.contact_sub = self.create_subscription(Bool, '/contact_sensor', self.contact_callback, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/force_torque', self.force_callback, 10)

        # Motor command outputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Reflex timers (very high frequency for immediate responses)
        self.reflex_timer = self.create_timer(0.005, self.process_reflexes)  # 200 Hz

        # Reflex state
        self.sensory_inputs = {
            'laser': None,
            'contact': None,
            'force': None
        }

        # Reflex thresholds and parameters
        self.reflex_params = {
            'collision_distance': 0.2,      # meters
            'contact_threshold': True,      # contact detected
            'force_threshold': 50.0         # Newtons
        }

        self.active_reflexes = set()

        self.get_logger().info('Reflex System initialized')

    def laser_callback(self, msg):
        """Process laser data for collision reflex"""
        self.sensory_inputs['laser'] = msg

    def contact_callback(self, msg):
        """Process contact sensor data"""
        self.sensory_inputs['contact'] = msg

    def force_callback(self, msg):
        """Process force/torque data"""
        self.sensory_inputs['force'] = msg

    def process_reflexes(self):
        """Process all active reflexes at high frequency"""
        if not all(self.sensory_inputs.values()):
            return

        # Check and execute reflexes
        if self.check_collision_reflex():
            self.execute_collision_reflex()

        if self.check_contact_reflex():
            self.execute_contact_reflex()

        if self.check_force_reflex():
            self.execute_force_reflex()

    def check_collision_reflex(self):
        """Check if collision reflex should be triggered"""
        if self.sensory_inputs['laser'] is None:
            return False

        ranges = np.array(self.sensory_inputs['laser'].ranges)
        ranges[ranges == float('inf')] = 3.5
        ranges[np.isnan(ranges)] = 3.5

        min_distance = np.min(ranges) if ranges.size > 0 else float('inf')

        return min_distance < self.reflex_params['collision_distance']

    def execute_collision_reflex(self):
        """Execute collision avoidance reflex"""
        if 'collision' in self.active_reflexes:
            return  # Already executing

        self.active_reflexes.add('collision')
        self.get_logger().warn('COLLISION REFLEX: Executing emergency avoidance')

        # Immediate stop command
        cmd = Twist()
        cmd.linear.x = -0.3  # Move backward quickly
        cmd.angular.z = 0.5  # Turn to avoid
        self.cmd_vel_pub.publish(cmd)

        # Schedule reflex reset
        self.create_timer(0.5, lambda: self.reset_reflex('collision'))

    def check_contact_reflex(self):
        """Check if contact reflex should be triggered"""
        if self.sensory_inputs['contact'] is None:
            return False

        return self.sensory_inputs['contact'].data == self.reflex_params['contact_threshold']

    def execute_contact_reflex(self):
        """Execute contact response reflex"""
        if 'contact' in self.active_reflexes:
            return

        self.active_reflexes.add('contact')
        self.get_logger().warn('CONTACT REFLEX: Withdrawing from contact')

        # Withdraw from contact
        cmd = Twist()
        cmd.linear.x = -0.2  # Move away
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        self.create_timer(0.3, lambda: self.reset_reflex('contact'))

    def check_force_reflex(self):
        """Check if force reflex should be triggered"""
        if self.sensory_inputs['force'] is None:
            return False

        force_magnitude = (
            self.sensory_inputs['force'].wrench.force.x**2 +
            self.sensory_inputs['force'].wrench.force.y**2 +
            self.sensory_inputs['force'].wrench.force.z**2
        )**0.5

        return force_magnitude > self.reflex_params['force_threshold']

    def execute_force_reflex(self):
        """Execute force response reflex"""
        if 'force' in self.active_reflexes:
            return

        self.active_reflexes.add('force')
        self.get_logger().warn('FORCE REFLEX: Reducing applied force')

        # Reduce joint efforts
        joint_cmd = JointState()
        # In a real system, this would command specific joint adjustments
        self.joint_cmd_pub.publish(joint_cmd)

        self.create_timer(0.2, lambda: self.reset_reflex('force'))

    def reset_reflex(self, reflex_type):
        """Reset a specific reflex"""
        if reflex_type in self.active_reflexes:
            self.active_reflexes.remove(reflex_type)
            self.get_logger().info(f'{reflex_type.upper()} REFLEX: Reset')

class AdaptiveReflexSystem(ReflexSystem):
    """
    Extended reflex system with adaptive capabilities.
    Reflux responses can adapt based on experience and context.
    """

    def __init__(self):
        super().__init__()

        # Add adaptation timer
        self.adaptation_timer = self.create_timer(1.0, self.adapt_reflexes)  # 1 Hz adaptation

        # Reflex adaptation parameters
        self.reflex_history = {
            'collision': {'success_count': 0, 'failure_count': 0, 'avg_time': 0.0},
            'contact': {'success_count': 0, 'failure_count': 0, 'avg_time': 0.0},
            'force': {'success_count': 0, 'failure_count': 0, 'avg_time': 0.0}
        }

        self.adaptation_learning_rate = 0.1

    def adapt_reflexes(self):
        """Adapt reflex parameters based on performance history"""
        for reflex_type, history in self.reflex_history.items():
            if history['success_count'] + history['failure_count'] > 0:
                success_rate = history['success_count'] / (history['success_count'] + history['failure_count'])

                # Adjust thresholds based on success rate
                if success_rate < 0.7:  # Too many failures, make more conservative
                    if reflex_type == 'collision':
                        self.reflex_params['collision_distance'] *= 1.1  # Increase threshold
                    elif reflex_type == 'force':
                        self.reflex_params['force_threshold'] *= 0.9  # Decrease threshold
                elif success_rate > 0.95:  # Very successful, could be more aggressive
                    if reflex_type == 'collision':
                        self.reflex_params['collision_distance'] *= 0.95  # Decrease threshold
                    elif reflex_type == 'force':
                        self.reflex_params['force_threshold'] *= 1.05  # Increase threshold

                # Keep parameters within reasonable bounds
                self.reflex_params['collision_distance'] = max(0.1, min(1.0, self.reflex_params['collision_distance']))
                self.reflex_params['force_threshold'] = max(10.0, min(100.0, self.reflex_params['force_threshold']))
```

## 4. Hierarchical Control Structures

### 4.1 Multi-Level Control Hierarchy
Implementing a control hierarchy similar to biological nervous systems:

```python
class HierarchicalController(Node):
    """
    Hierarchical control system with multiple levels of abstraction.
    Mimics the organization from spinal cord to cerebral cortex.
    """

    def __init__(self):
        super().__init__('hierarchical_controller')

        # Communication with different levels
        self.high_level_sub = self.create_subscription(String, '/high_level_goals', self.high_level_callback, 10)
        self.mid_level_sub = self.create_subscription(String, '/mid_level_tasks', self.mid_level_callback, 10)
        self.low_level_sub = self.create_subscription(String, '/low_level_commands', self.low_level_callback, 10)

        # Command publishers for different levels
        self.high_level_pub = self.create_publisher(String, '/high_level_status', 10)
        self.mid_level_pub = self.create_publisher(String, '/mid_level_status', 10)
        self.low_level_pub = self.create_publisher(String, '/low_level_status', 10)

        # Motor command publisher
        self.motor_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Level-specific timers
        self.high_level_timer = self.create_timer(0.5, self.high_level_processing)   # 2 Hz
        self.mid_level_timer = self.create_timer(0.1, self.mid_level_processing)     # 10 Hz
        self.low_level_timer = self.create_timer(0.02, self.low_level_processing)    # 50 Hz

        # Internal state for each level
        self.high_level_state = {
            'current_goal': None,
            'plan': [],
            'execution_status': 'IDLE'
        }

        self.mid_level_state = {
            'current_task': None,
            'subtasks': [],
            'progress': 0.0
        }

        self.low_level_state = {
            'current_action': None,
            'motor_commands': [],
            'execution_time': 0.0
        }

        self.get_logger().info('Hierarchical Controller initialized')

    def high_level_callback(self, msg):
        """Receive high-level goals and plans"""
        try:
            goal_data = eval(msg.data)  # In practice, use json.loads
            self.high_level_state['current_goal'] = goal_data
            self.high_level_state['execution_status'] = 'PLANNING'
        except:
            self.get_logger().warn('Invalid high-level command format')

    def mid_level_callback(self, msg):
        """Receive mid-level tasks"""
        try:
            task_data = eval(msg.data)
            self.mid_level_state['current_task'] = task_data
        except:
            self.get_logger().warn('Invalid mid-level command format')

    def low_level_callback(self, msg):
        """Receive low-level commands"""
        try:
            cmd_data = eval(msg.data)
            self.low_level_state['current_action'] = cmd_data
        except:
            self.get_logger().warn('Invalid low-level command format')

    def high_level_processing(self):
        """High-level cognitive processing (goal planning and reasoning)"""
        if self.high_level_state['execution_status'] == 'PLANNING':
            if self.high_level_state['current_goal']:
                # Generate plan for the goal
                plan = self.generate_plan(self.high_level_state['current_goal'])
                self.high_level_state['plan'] = plan
                self.high_level_state['execution_status'] = 'EXECUTING'

                # Send first task to mid-level
                if plan:
                    task_msg = String()
                    task_msg.data = str(plan[0])
                    self.mid_level_pub.publish(task_msg)

        elif self.high_level_state['execution_status'] == 'EXECUTING':
            # Monitor progress and adjust plan if needed
            if not self.check_execution_progress():
                # Plan adjustment needed
                self.adjust_plan()

        # Publish status
        status_msg = String()
        status_msg.data = str({
            'status': self.high_level_state['execution_status'],
            'current_goal': self.high_level_state['current_goal'],
            'plan_progress': len(self.high_level_state['plan']) - len([t for t in self.high_level_state['plan'] if t not in self.mid_level_state['subtasks']])
        })
        self.high_level_pub.publish(status_msg)

    def mid_level_processing(self):
        """Mid-level task execution and coordination"""
        if self.mid_level_state['current_task']:
            task = self.mid_level_state['current_task']

            # Execute task or delegate to low-level
            if self.is_primitive_task(task):
                # Send to low-level for execution
                cmd_msg = String()
                cmd_msg.data = str(task)
                self.low_level_pub.publish(cmd_msg)
            else:
                # Break down complex task into subtasks
                subtasks = self.decompose_task(task)
                self.mid_level_state['subtasks'] = subtasks
                self.execute_next_subtask()

        # Publish status
        status_msg = String()
        status_msg.data = str({
            'current_task': self.mid_level_state['current_task'],
            'subtasks_remaining': len(self.mid_level_state['subtasks']),
            'progress': self.mid_level_state['progress']
        })
        self.mid_level_pub.publish(status_msg)

    def low_level_processing(self):
        """Low-level motor control and execution"""
        if self.low_level_state['current_action']:
            action = self.low_level_state['current_action']

            # Convert action to motor commands
            motor_cmd = self.action_to_motor_command(action)
            self.motor_cmd_pub.publish(motor_cmd)

            # Update execution time
            self.low_level_state['execution_time'] += 0.02  # Timer period

        # Publish status
        status_msg = String()
        status_msg.data = str({
            'current_action': self.low_level_state['current_action'],
            'execution_time': self.low_level_state['execution_time']
        })
        self.low_level_pub.publish(status_msg)

    def generate_plan(self, goal):
        """Generate a plan to achieve the given goal"""
        # In a real implementation, this would use path planning, task planning, etc.
        # For example: navigate to goal, perform action, return
        return [
            {'type': 'navigate', 'target': goal.get('location', (0, 0))},
            {'type': 'perform_action', 'action': goal.get('action', 'wait')},
            {'type': 'return', 'target': goal.get('return_location', (0, 0))}
        ]

    def is_primitive_task(self, task):
        """Check if task is primitive (can be executed directly)"""
        primitive_types = ['move', 'turn', 'stop', 'grip', 'release']
        return task.get('type', '') in primitive_types

    def decompose_task(self, task):
        """Decompose a complex task into primitive subtasks"""
        task_type = task.get('type', '')

        if task_type == 'navigate':
            # Complex navigation breaks down into move and turn primitives
            return [
                {'type': 'turn', 'angle': task.get('heading', 0)},
                {'type': 'move', 'distance': task.get('distance', 1.0)}
            ]
        elif task_type == 'grasp_object':
            # Grasping breaks down into approach, align, grip
            return [
                {'type': 'navigate', 'target': task.get('approach_pose', (0, 0, 0))},
                {'type': 'align_gripper', 'target': task.get('grasp_pose', (0, 0, 0))},
                {'type': 'grip', 'force': task.get('grip_force', 10.0)}
            ]

        return [task]  # If no decomposition known, return as-is

    def action_to_motor_command(self, action):
        """Convert an action to motor commands"""
        cmd = Twist()

        action_type = action.get('type', '')
        if action_type == 'move':
            cmd.linear.x = action.get('speed', 0.5)
        elif action_type == 'turn':
            cmd.angular.z = action.get('angular_speed', 0.5)
        elif action_type == 'stop':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def check_execution_progress(self):
        """Check if high-level goal execution is progressing"""
        # In a real implementation, this would check actual progress toward goal
        # For now, return True to continue
        return True

    def adjust_plan(self):
        """Adjust the current plan based on execution feedback"""
        self.get_logger().info('Adjusting plan based on execution feedback')
        # Implementation would adjust plan based on actual vs expected progress
```

### 4.2 Coordination Between Levels
Ensuring proper coordination between different control levels:

```python
class CoordinationManager(Node):
    """
    Manages coordination between different levels of the control hierarchy.
    Ensures that higher-level goals are properly decomposed and executed
    while lower-level reflexes can override when necessary.
    """

    def __init__(self):
        super().__init__('coordination_manager')

        # Subscriptions for status from all levels
        self.high_status_sub = self.create_subscription(String, '/high_level_status', self.high_status_callback, 10)
        self.mid_status_sub = self.create_subscription(String, '/mid_level_status', self.mid_status_callback, 10)
        self.low_status_sub = self.create_subscription(String, '/low_level_status', self.low_status_callback, 10)

        # Emergency override subscription
        self.emergency_sub = self.create_subscription(Bool, '/emergency_override', self.emergency_callback, 10)

        # Command publications to all levels
        self.high_cmd_pub = self.create_publisher(String, '/high_level_commands', 10)
        self.mid_cmd_pub = self.create_publisher(String, '/mid_level_commands', 10)
        self.low_cmd_pub = self.create_publisher(String, '/low_level_commands', 10)

        # Coordination timer
        self.coordination_timer = self.create_timer(0.05, self.coordination_loop)

        # Status tracking
        self.level_status = {
            'high': None,
            'mid': None,
            'low': None
        }

        self.emergency_override = False
        self.emergency_priority = 10  # Highest priority

        self.get_logger().info('Coordination Manager initialized')

    def high_status_callback(self, msg):
        try:
            self.level_status['high'] = eval(msg.data)
        except:
            self.get_logger().warn('Invalid high-level status format')

    def mid_status_callback(self, msg):
        try:
            self.level_status['mid'] = eval(msg.data)
        except:
            self.get_logger().warn('Invalid mid-level status format')

    def low_status_callback(self, msg):
        try:
            self.level_status['low'] = eval(msg.data)
        except:
            self.get_logger().warn('Invalid low-level status format')

    def emergency_callback(self, msg):
        """Handle emergency override commands"""
        self.emergency_override = msg.data
        if self.emergency_override:
            self.get_logger().fatal('EMERGENCY OVERRIDE ACTIVATED')
            # Send emergency stop to all levels
            self.send_emergency_stop()

    def coordination_loop(self):
        """Main coordination loop"""
        if self.emergency_override:
            return  # Emergency override has highest priority

        # Check for conflicts between levels
        conflicts = self.detect_conflicts()

        if conflicts:
            # Resolve conflicts based on priority and context
            self.resolve_conflicts(conflicts)

        # Monitor execution progress and adjust coordination as needed
        self.monitor_progress()

    def detect_conflicts(self):
        """Detect conflicts between different control levels"""
        conflicts = []

        # Example: Check if low-level is executing while high-level wants to replan
        if (self.level_status['high'] and
            self.level_status['high'].get('execution_status') == 'PLANNING' and
            self.level_status['low'] and
            self.level_status['low'].get('current_action')):
            conflicts.append({
                'type': 'execution_conflict',
                'levels': ['high', 'low'],
                'description': 'High-level planning while low-level executing'
            })

        # Add more conflict detection as needed
        return conflicts

    def resolve_conflicts(self, conflicts):
        """Resolve detected conflicts between control levels"""
        for conflict in conflicts:
            if conflict['type'] == 'execution_conflict':
                # For execution conflicts, typically pause low-level execution during replanning
                pause_cmd = String()
                pause_cmd.data = str({'command': 'pause_execution'})
                self.low_cmd_pub.publish(pause_cmd)

    def monitor_progress(self):
        """Monitor execution progress across all levels"""
        # Check if execution is proceeding as expected
        # This might involve comparing expected vs actual progress
        pass

    def send_emergency_stop(self):
        """Send emergency stop command to all levels"""
        stop_cmd = String()
        stop_cmd.data = str({'command': 'emergency_stop'})

        self.high_cmd_pub.publish(stop_cmd)
        self.mid_cmd_pub.publish(stop_cmd)
        self.low_cmd_pub.publish(stop_cmd)
```

## 5. Adaptive Control Systems

### 5.1 Learning-Based Adaptation
Implementing systems that can learn and adapt their behavior:

```python
import pickle
import os
from collections import deque

class AdaptiveController(Node):
    """
    Adaptive controller that learns from experience and adjusts behavior.
    Implements basic learning mechanisms similar to habituation and adaptation in biological systems.
    """

    def __init__(self):
        super().__init__('adaptive_controller')

        # Sensor and command interfaces
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Learning timer
        self.learning_timer = self.create_timer(0.1, self.learning_loop)

        # Internal state
        self.sensors = {
            'laser': None,
            'odom': None,
            'imu': None
        }

        # Learning components
        self.experience_buffer = deque(maxlen=1000)  # Store recent experiences
        self.performance_history = deque(maxlen=100)  # Track performance over time
        self.adaptation_params = {
            'learning_rate': 0.01,
            'exploration_rate': 0.1,
            'adaptation_threshold': 0.8
        }

        # Learned behaviors
        self.learnt_behaviors = {}  # Maps situations to behaviors
        self.behavior_preferences = {}  # Tracks which behaviors work best

        # Load any saved learning
        self.load_learning()

        self.get_logger().info('Adaptive Controller initialized')

    def laser_callback(self, msg):
        self.sensors['laser'] = msg

    def odom_callback(self, msg):
        self.sensors['odom'] = msg

    def imu_callback(self, msg):
        self.sensors['imu'] = msg

    def learning_loop(self):
        """Main learning and adaptation loop"""
        if not all(self.sensors.values()):
            return

        # Get current state
        current_state = self.get_current_state()

        # Select action based on current state and learned knowledge
        action = self.select_action(current_state)

        # Execute action
        self.execute_action(action)

        # Evaluate performance and store experience
        performance = self.evaluate_performance()
        experience = {
            'state': current_state,
            'action': action,
            'performance': performance,
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.experience_buffer.append(experience)
        self.performance_history.append(performance)

        # Periodically update learned behaviors
        if len(self.experience_buffer) % 10 == 0:  # Update every 10 experiences
            self.update_learning()

    def get_current_state(self):
        """Extract relevant features from sensor data to represent current state"""
        state = {}

        if self.sensors['laser']:
            laser_data = np.array(self.sensors['laser'].ranges)
            laser_data[laser_data == float('inf')] = 3.5
            laser_data[np.isnan(laser_data)] = 3.5

            # Extract key features
            state['min_obstacle_distance'] = float(np.min(laser_data)) if laser_data.size > 0 else float('inf')
            state['front_clear'] = float(np.min(laser_data[len(laser_data)//2-10:len(laser_data)//2+10])) if laser_data.size > 20 else float('inf')
            state['left_clear'] = float(np.min(laser_data[:len(laser_data)//4])) if laser_data.size > 0 else float('inf')
            state['right_clear'] = float(np.min(laser_data[3*len(laser_data)//4:])) if laser_data.size > 0 else float('inf')

        if self.sensors['odom']:
            odom = self.sensors['odom']
            state['linear_velocity'] = odom.twist.twist.linear.x
            state['angular_velocity'] = odom.twist.twist.angular.z

        if self.sensors['imu']:
            imu = self.sensors['imu']
            # Extract orientation features
            import math
            q = imu.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            state['yaw'] = yaw

        return state

    def select_action(self, state):
        """Select action based on state and learned behaviors"""
        # First, check if we have a learned behavior for this state
        state_key = self.discretize_state(state)

        if state_key in self.learnt_behaviors:
            # Use learned behavior
            return self.learnt_behaviors[state_key]
        else:
            # Use exploration with some random behavior
            if np.random.random() < self.adaptation_params['exploration_rate']:
                # Random exploration
                return self.generate_random_action()
            else:
                # Default behavior
                return self.default_behavior(state)

    def generate_random_action(self):
        """Generate a random exploratory action"""
        cmd = Twist()
        cmd.linear.x = np.random.uniform(0.1, 0.8)  # Forward speed
        cmd.angular.z = np.random.uniform(-0.5, 0.5)  # Turn rate
        return cmd

    def default_behavior(self, state):
        """Default behavior when no learned behavior exists"""
        cmd = Twist()

        # Simple obstacle avoidance by default
        if state.get('min_obstacle_distance', float('inf')) < 0.5:
            cmd.linear.x = 0.0  # Stop
            if state.get('left_clear', 0) > state.get('right_clear', 0):
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right
        else:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0  # No turn

        return cmd

    def execute_action(self, action):
        """Execute the selected action"""
        self.cmd_vel_pub.publish(action)

    def evaluate_performance(self):
        """Evaluate the performance of recent actions"""
        # Calculate performance based on progress toward goals
        # For simplicity, we'll use a basic metric based on movement and obstacle avoidance

        if not self.sensors['laser'] or not self.sensors['odom']:
            return 0.0

        laser_data = np.array(self.sensors['laser'].ranges)
        laser_data[laser_data == float('inf')] = 3.5
        laser_data[np.isnan(laser_data)] = 3.5

        min_distance = np.min(laser_data) if laser_data.size > 0 else float('inf')
        linear_vel = self.sensors['odom'].twist.twist.linear.x

        # Performance metric: combination of progress and safety
        safety_factor = 1.0 if min_distance > 0.8 else min_distance / 0.8
        progress_factor = abs(linear_vel) if linear_vel > 0 else 0  # Only reward forward movement

        performance = 0.7 * progress_factor + 0.3 * safety_factor
        return min(1.0, performance)  # Normalize to [0, 1]

    def update_learning(self):
        """Update learned behaviors based on experience"""
        if len(self.performance_history) < 10:
            return

        # Calculate average recent performance
        recent_performance = np.mean(list(self.performance_history)[-10:])

        if recent_performance > self.adaptation_params['adaptation_threshold']:
            # Good performance - reinforce successful patterns
            self.reinforce_successful_behaviors()
        else:
            # Poor performance - explore new behaviors
            self.explore_new_behaviors()

    def reinforce_successful_behaviors(self):
        """Reinforce behaviors that led to good performance"""
        # Look at recent experiences with high performance
        recent_experiences = list(self.experience_buffer)[-20:]

        for exp in recent_experiences:
            if exp['performance'] > 0.8:  # High performance threshold
                state_key = self.discretize_state(exp['state'])
                action = exp['action']

                # Update behavior preference
                if state_key not in self.behavior_preferences:
                    self.behavior_preferences[state_key] = {}

                action_key = self.action_to_key(action)
                if action_key not in self.behavior_preferences[state_key]:
                    self.behavior_preferences[state_key][action_key] = 0

                # Increase preference for this action in this state
                self.behavior_preferences[state_key][action_key] += self.adaptation_params['learning_rate']

    def explore_new_behaviors(self):
        """Encourage exploration of new behaviors when performance is poor"""
        # Increase exploration rate temporarily
        self.adaptation_params['exploration_rate'] = min(0.5, self.adaptation_params['exploration_rate'] * 1.1)

    def discretize_state(self, state):
        """Convert continuous state to discrete representation for learning"""
        # Simplified discretization - in practice, this would be more sophisticated
        if not state:
            return "unknown"

        min_dist = state.get('min_obstacle_distance', float('inf'))
        front_clear = state.get('front_clear', float('inf'))

        # Create discrete state representation
        dist_category = "close" if min_dist < 0.5 else "medium" if min_dist < 1.0 else "far"
        front_category = "clear" if front_clear > 1.0 else "obstructed"

        return f"{dist_category}_{front_category}"

    def action_to_key(self, action):
        """Convert action to a hashable key for learning"""
        return (round(action.linear.x, 2), round(action.angular.z, 2))

    def save_learning(self):
        """Save learned behaviors to file"""
        learning_data = {
            'learnt_behaviors': self.learnt_behaviors,
            'behavior_preferences': self.behavior_preferences,
            'adaptation_params': self.adaptation_params,
            'experience_count': len(self.experience_buffer)
        }

        try:
            with open('adaptive_controller_learning.pkl', 'wb') as f:
                pickle.dump(learning_data, f)
            self.get_logger().info('Learning data saved successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to save learning data: {e}')

    def load_learning(self):
        """Load learned behaviors from file"""
        if os.path.exists('adaptive_controller_learning.pkl'):
            try:
                with open('adaptive_controller_learning.pkl', 'rb') as f:
                    learning_data = pickle.load(f)

                self.learnt_behaviors = learning_data.get('learnt_behaviors', {})
                self.behavior_preferences = learning_data.get('behavior_preferences', {})
                self.adaptation_params = learning_data.get('adaptation_params', self.adaptation_params)

                self.get_logger().info(f'Learning data loaded: {learning_data.get("experience_count", 0)} experiences')
            except Exception as e:
                self.get_logger().error(f'Failed to load learning data: {e}')

    def destroy_node(self):
        """Save learning before shutdown"""
        self.save_learning()
        super().destroy_node()
```

## 6. Practical Example: Complete Robotic Nervous System

Here's a complete example that integrates all the concepts:

```python
#!/usr/bin/env python3
"""
Complete Robotic Nervous System Example
This example demonstrates a complete implementation of a robotic nervous system
with reflexes, hierarchical control, and adaptive behavior.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Odometry, JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Time
import numpy as np
import math
from enum import Enum
from collections import deque

class ControlLevel(Enum):
    REFLEX = 1      # Immediate responses
    SPINAL = 2      # Local coordination
    MIDBRAIN = 3    # Basic behavioral patterns
    CORTICAL = 4    # High-level planning

class RoboticNervousSystemCore(Node):
    """
    Core nervous system that coordinates all levels of control.
    """

    def __init__(self):
        super().__init__('robotic_nervous_system_core')

        # All sensor inputs
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Motor outputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Control level timers with different frequencies
        self.reflex_timer = self.create_timer(0.005, self.reflex_level_control)    # 200 Hz
        self.spinal_timer = self.create_timer(0.02, self.spinal_level_control)     # 50 Hz
        self.midbrain_timer = self.create_timer(0.1, self.midbrain_level_control)  # 10 Hz
        self.cortical_timer = self.create_timer(0.5, self.cortical_level_control)  # 2 Hz

        # Internal state
        self.sensors = {
            'laser': None,
            'imu': None,
            'odom': None,
            'joints': None
        }

        # Control priorities and states
        self.control_priority = {
            ControlLevel.REFLEX: 100,
            ControlLevel.SPINAL: 80,
            ControlLevel.MIDBRAIN: 60,
            ControlLevel.CORTICAL: 40
        }

        self.active_controls = {}  # Track which control level is active
        self.control_commands = {}  # Store commands from each level

        # Emergency state
        self.emergency_active = False
        self.emergency_reason = ""

        # Learning and adaptation
        self.adaptation_enabled = True
        self.performance_history = deque(maxlen=50)

        self.get_logger().info('Robotic Nervous System Core initialized')

    def laser_callback(self, msg):
        self.sensors['laser'] = msg

    def imu_callback(self, msg):
        self.sensors['imu'] = msg

    def odom_callback(self, msg):
        self.sensors['odom'] = msg

    def joint_callback(self, msg):
        self.sensors['joints'] = msg

    def reflex_level_control(self):
        """Highest priority - immediate reflex responses"""
        if not self.sensors['laser']:
            return

        # Collision avoidance reflex
        laser_data = np.array(self.sensors['laser'].ranges)
        laser_data[laser_data == float('inf')] = 3.5
        laser_data[np.isnan(laser_data)] = 3.5

        min_distance = np.min(laser_data) if laser_data.size > 0 else float('inf')

        if min_distance < 0.2:  # Immediate collision danger
            cmd = Twist()
            cmd.linear.x = -0.4  # Rapid reverse
            cmd.angular.z = 0.8 if np.argmin(laser_data) < len(laser_data) / 2 else -0.8
            self.control_commands[ControlLevel.REFLEX] = cmd
            self.active_controls[ControlLevel.REFLEX] = self.get_clock().now().nanoseconds
            return

        # Approach reflex
        if min_distance < 0.5 and min_distance > 0.2:
            cmd = Twist()
            cmd.linear.x = max(0.1, min_distance - 0.3)  # Slow approach
            # Gentle turn away from closest obstacle
            closest_idx = np.argmin(laser_data)
            cmd.angular.z = 0.3 if closest_idx < len(laser_data) / 2 else -0.3
            self.control_commands[ControlLevel.REFLEX] = cmd
            self.active_controls[ControlLevel.REFLEX] = self.get_clock().now().nanoseconds
            return

        # If no reflex action needed, clear reflex command
        if ControlLevel.REFLEX in self.control_commands:
            del self.control_commands[ControlLevel.REFLEX]

    def spinal_level_control(self):
        """Local coordination and balance control"""
        if not all([self.sensors['imu'], self.sensors['odom']]):
            return

        cmd = Twist()

        # Balance control based on IMU
        imu = self.sensors['imu']
        q = imu.orientation
        # Convert quaternion to roll/pitch
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        pitch = math.atan2(siny_cosp, cosy_cosp)

        # Correct for tilt
        cmd.angular.z = -pitch * 2.0  # Turn to correct pitch
        # cmd.linear.x adjusted based on roll if needed for more complex balance

        # If no high-priority reflex is active, use spinal control
        if ControlLevel.REFLEX not in self.active_controls or \
           (self.get_clock().now().nanoseconds - self.active_controls.get(ControlLevel.REFLEX, 0)) > 100000000:  # 0.1 sec
            self.control_commands[ControlLevel.SPINAL] = cmd
            self.active_controls[ControlLevel.SPINAL] = self.get_clock().now().nanoseconds

    def midbrain_level_control(self):
        """Basic behavioral patterns and responses"""
        if not all([self.sensors['laser'], self.sensors['odom']]):
            return

        cmd = Twist()

        # Simple wall following behavior
        laser_data = np.array(self.sensors['laser'].ranges)
        laser_data[laser_data == float('inf')] = 3.5
        laser_data[np.isnan(laser_data)] = 3.5

        # Check for walls on left/right
        left_section = laser_data[:len(laser_data)//4]
        right_section = laser_data[3*len(laser_data)//4:]

        left_distance = np.min(left_section) if left_section.size > 0 else float('inf')
        right_distance = np.min(right_section) if right_section.size > 0 else float('inf')

        # Wall following: maintain distance from wall
        target_distance = 0.8
        kp = 1.0  # Proportional gain

        if left_distance < target_distance * 1.5:  # Wall detected on left
            # Turn right to maintain distance
            cmd.angular.z = -kp * (target_distance - left_distance)
            cmd.linear.x = 0.4  # Forward motion
        elif right_distance < target_distance * 1.5:  # Wall on right
            # Turn left to maintain distance
            cmd.angular.z = kp * (target_distance - right_distance)
            cmd.linear.x = 0.4  # Forward motion
        else:
            # No wall detected, continue straight
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        # Only activate if lower levels aren't handling critical situations
        if not any(level in self.active_controls and
                  self.get_clock().now().nanoseconds - self.active_controls[level] < 50000000  # 0.05 sec
                  for level in [ControlLevel.REFLEX, ControlLevel.SPINAL]):
            self.control_commands[ControlLevel.MIDBRAIN] = cmd
            self.active_controls[ControlLevel.MIDBRAIN] = self.get_clock().now().nanoseconds

    def cortical_level_control(self):
        """High-level goal-oriented behavior"""
        # For this example, implement simple exploration behavior
        # In a real system, this would handle complex tasks and planning

        cmd = Twist()

        # Random walk with bias toward open areas
        if self.sensors['laser']:
            laser_data = np.array(self.sensors['laser'].ranges)
            laser_data[laser_data == float('inf')] = 3.5
            laser_data[np.isnan(laser_data)] = 3.5

            # Find the direction with maximum clearance
            max_idx = np.argmax(laser_data)
            angle_to_max = (max_idx / len(laser_data)) * 2 * math.pi - math.pi  # Convert to angle

            # Move toward the clearest direction with some randomness
            cmd.linear.x = 0.6  # Forward speed
            cmd.angular.z = angle_to_max * 0.5 + np.random.uniform(-0.2, 0.2)  # Turn toward clear area

        # Only activate if lower levels aren't handling critical situations
        if not any(level in self.active_controls and
                  self.get_clock().now().nanoseconds - self.active_controls[level] < 100000000  # 0.1 sec
                  for level in [ControlLevel.REFLEX, ControlLevel.SPINAL, ControlLevel.MIDBRAIN]):
            self.control_commands[ControlLevel.CORTICAL] = cmd
            self.active_controls[ControlLevel.CORTICAL] = self.get_clock().now().nanoseconds

    def execute_highest_priority_command(self):
        """Execute the command from the highest priority active control level"""
        if not self.control_commands:
            return

        # Find the highest priority active control
        active_levels = [level for level in ControlLevel
                        if level in self.control_commands]

        if not active_levels:
            return

        # Sort by priority (highest first)
        active_levels.sort(key=lambda x: self.control_priority[x], reverse=True)

        # Execute command from highest priority level
        highest_level = active_levels[0]
        cmd = self.control_commands[highest_level]

        # Apply safety limits
        cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

        self.cmd_vel_pub.publish(cmd)

        # Log which level is currently controlling
        self.get_logger().debug(f'Control level {highest_level.name} active')

def main(args=None):
    rclpy.init(args=args)
    nervous_system = RoboticNervousSystemCore()

    try:
        # Run the main control loop
        while rclpy.ok():
            rclpy.spin_once(nervous_system, timeout_sec=0.001)
            # Execute the highest priority command
            nervous_system.execute_highest_priority_command()
    except KeyboardInterrupt:
        pass
    finally:
        nervous_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. Exercises and Practice

Complete the following exercises to reinforce your understanding of robotic nervous system patterns:

1. [Chapter 4 Exercises](./chapter4-exercises.md) - Practice problems covering nervous system pattern implementation
2. [Chapter 4 Solutions](./chapter4-solutions.md) - Complete implementations and solution guides

## 8. Summary

This chapter covered the implementation of robotic nervous system patterns inspired by biological neural networks:

- Biological nervous system principles and their robotic applications
- Distributed control architectures using ROS 2 nodes
- Reflex-based control systems for immediate responses
- Hierarchical control structures with proper coordination
- Adaptive control systems that learn and adjust behavior

The next chapter would typically cover advanced topics in humanoid robotics, building upon the nervous system patterns to create more sophisticated behaviors and capabilities.

## 9. Further Reading

- [Biological Neural Networks and Robotics](https://www.sciencedirect.com/topics/engineering/biological-neural-network)
- [ROS 2 Control Framework](https://control.ros.org/)
- [Bio-inspired Robotics](https://ieeexplore.ieee.org/document/9123456)
- [Distributed Control Systems](https://link.springer.com/book/10.1007/978-3-030-12442-9)

## 10. Links to External Resources

- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [Biologically-Inspired Robotics Research](https://www.ieee-ras.org/)
- [Neural Network Control Systems](https://www.scholarpedia.org/article/Neural_networks)