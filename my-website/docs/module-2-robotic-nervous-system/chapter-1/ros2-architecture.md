# Understanding ROS 2 Architecture and Communication Patterns

## Learning Objectives
By the end of this chapter, readers will be able to:
- Explain the fundamental architecture of ROS 2 and its core components
- Create and run basic ROS 2 nodes using Python and rclpy
- Implement publisher-subscriber communication patterns
- Use service client-server communication for request-response interactions
- Apply action client-server patterns for long-running operations with feedback

## Prerequisites
- Basic Python programming knowledge
- Completion of Module 1: Foundations of Physical AI & Humanoid Robotics
- Understanding of distributed systems concepts (helpful but not required)

## Introduction
The Robot Operating System 2 (ROS 2) provides the middleware infrastructure that enables communication between different components of robotic systems. Understanding ROS 2 architecture is fundamental to working with any ROS 2-based robotic system. This chapter covers the core concepts of ROS 2 architecture, including nodes, topics, services, and actions, which form the backbone of distributed robotic systems.

ROS 2 represents a significant evolution from ROS 1, addressing critical requirements for production systems including real-time performance, security, and support for multiple operating systems. The architecture is built on DDS (Data Distribution Service) which provides a robust communication infrastructure for distributed systems.

## 1. ROS 2 Architecture Fundamentals

### 1.1 Nodes
A node is an independent process that performs computation. Nodes are the fundamental building blocks of ROS 2 programs. Each node can perform specific functions such as sensor data processing, control algorithm execution, or user interface management.

Key characteristics of nodes:
- Each node runs in its own process
- Nodes communicate with other nodes using topics, services, and actions
- Nodes can be organized hierarchically using namespaces
- Nodes must be uniquely named within a ROS domain

```python
# Example: Creating a basic ROS 2 node
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code goes here
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.2 Topics and Publishing/Subscription
Topics enable asynchronous communication between nodes through a publish/subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This pattern supports one-to-many communication.

Key characteristics of topics:
- Asynchronous communication
- One publisher can have many subscribers
- Many publishers can send to the same topic
- Message types must be consistent across publisher/subscriber pairs

```python
# Publisher example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# Subscriber example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### 1.3 Services
Services provide synchronous request/response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response.

Key characteristics of services:
- Synchronous communication
- Request/response pattern
- One-to-one communication between client and server
- Useful for operations that have a clear start and end

```python
# Service server example
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

# Service client example
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

### 1.4 Actions
Actions provide asynchronous request/response communication with feedback for long-running operations. They include three parts: goal, feedback, and result.

Key characteristics of actions:
- Asynchronous communication
- Goal, feedback, and result phases
- Support for cancellation and preemption
- Ideal for long-running operations with progress updates

```python
# Action server example
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

# Action client example
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')
```

## 2. Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior between nodes. They control reliability, durability, liveliness, and other aspects of message delivery.

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Example: Configuring QoS for a publisher
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

## 3. Namespaces and Parameter Management

Namespaces provide a way to organize nodes hierarchically, while parameters allow runtime configuration.

```python
class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters
        self.declare_parameter('param_name', 'default_value')

        # Get parameter value
        param_value = self.get_parameter('param_name').value

        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'param_name':
                self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)
```

## 4. Practical Example: Simple Robot Controller

Let's create a complete example that demonstrates multiple ROS 2 concepts:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

class SimpleRobotController(Node):

    def __init__(self):
        super().__init__('simple_robot_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Subscribers
        self.sensor_sub = self.create_subscription(
            Float64, '/sensor_data', self.sensor_callback, 10)

        # Services
        self.emergency_stop_srv = self.create_service(
            SetBool, '/emergency_stop', self.emergency_stop_callback)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.robot_enabled = True
        self.sensor_value = 0.0

        self.get_logger().info('Simple Robot Controller initialized')

    def sensor_callback(self, msg):
        self.sensor_value = msg.data

    def emergency_stop_callback(self, request, response):
        if request.data:
            self.robot_enabled = False
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        else:
            self.robot_enabled = True
            self.get_logger().info('Robot re-enabled')

        response.success = True
        response.message = f'Robot enabled status: {self.robot_enabled}'
        return response

    def control_loop(self):
        if not self.robot_enabled:
            # Send stop command
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # Simple control logic based on sensor value
        cmd = Twist()
        cmd.linear.x = min(1.0, max(-1.0, self.sensor_value * 0.1))
        cmd.angular.z = min(1.0, max(-1.0, self.sensor_value * 0.05))

        self.cmd_vel_pub.publish(cmd)

        # Publish status
        status_msg = String()
        status_msg.data = f'Sensor: {self.sensor_value:.2f}, Enabled: {self.robot_enabled}'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleRobotController()

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

## 5. Exercise: Create a Basic ROS 2 Publisher-Subscriber System

### Objective
Create a simple publisher-subscriber system that demonstrates the basic communication pattern in ROS 2.

### Instructions
1. Create a publisher node that publishes a counter value every second
2. Create a subscriber node that receives and logs the counter value
3. Run both nodes and observe the communication
4. Experiment with different QoS settings to see their effect

### Expected Outcome
Two nodes communicating via a topic, with the subscriber successfully receiving messages from the publisher.

## 6. Summary

This chapter introduced the fundamental architecture of ROS 2, including:
- Nodes as the basic computational units
- Topics for asynchronous publish/subscribe communication
- Services for synchronous request/response communication
- Actions for long-running operations with feedback
- QoS settings for fine-tuning communication behavior
- Namespaces and parameters for organization and configuration

Understanding these concepts is essential for building distributed robotic systems with ROS 2. The next chapter will build upon these foundations to show how to bridge AI agents to robot controllers.

## 7. Exercises and Practice

Complete the following exercises to reinforce your understanding of ROS 2 architecture:

1. [Chapter 1 Exercises](./exercises.md) - Practice problems covering ROS 2 communication patterns
2. [Chapter 1 Solutions](./solutions.md) - Complete implementations and solution guides

## 8. Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Papers](https://index.ros.org/doc/ros2/About-Ros2-design/)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings/)

## 9. Links to External Resources

- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Design Principles](https://design.ros2.org/)