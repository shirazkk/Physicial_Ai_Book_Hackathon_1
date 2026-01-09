# Solutions: ROS 2 Architecture and Communication Patterns

## Solution for Exercise 1: Basic Publisher-Subscriber

### Complete Publisher Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperaturePublisher(Node):

    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float64, '/temperature', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Temperature publisher started')

    def timer_callback(self):
        msg = Float64()
        # Generate random temperature between 18 and 25 degrees Celsius
        msg.data = random.uniform(18.0, 25.0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = TemperaturePublisher()

    try:
        rclpy.spin(temperature_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Complete Subscriber Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureSubscriber(Node):

    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            '/temperature',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Temperature subscriber started')

    def listener_callback(self, msg):
        temperature = msg.data
        if 20.0 <= temperature <= 24.0:
            status = "Comfortable temperature range"
        else:
            status = "Temperature outside comfortable range"

        self.get_logger().info(f'Received temperature: {temperature:.2f}°C - {status}')

def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber = TemperatureSubscriber()

    try:
        rclpy.spin(temperature_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run
1. Save the publisher code as `temperature_publisher.py`
2. Save the subscriber code as `temperature_subscriber.py`
3. Make sure your ROS 2 environment is sourced
4. Run the publisher: `python3 temperature_publisher.py`
5. In another terminal, run the subscriber: `python3 temperature_subscriber.py`

## Solution for Exercise 2: Service-Based Calculator

### Service Definition
First, create a service definition file `Calculator.srv` in a `srv` directory:

```
# Request
float64 a
float64 b
string operation  # add, subtract, multiply, divide
---
# Response
float64 result
bool success
string message
```

### Service Server Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from your_package.srv import Calculator  # Replace with your actual package name

class CalculatorService(Node):

    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(Calculator, 'calculate', self.calculate_callback)

    def calculate_callback(self, request, response):
        a = request.a
        b = request.b
        operation = request.operation

        if operation == 'add':
            response.result = a + b
        elif operation == 'subtract':
            response.result = a - b
        elif operation == 'multiply':
            response.result = a * b
        elif operation == 'divide':
            if b == 0:
                response.success = False
                response.message = 'Error: Division by zero'
                return response
            response.result = a / b
        else:
            response.success = False
            response.message = f'Error: Unknown operation {operation}'
            return response

        response.success = True
        response.message = f'Success: {a} {operation} {b} = {response.result}'
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    calculator_service = CalculatorService()

    try:
        rclpy.spin(calculator_service)
    except KeyboardInterrupt:
        pass
    finally:
        calculator_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Code
```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from your_package.srv import Calculator  # Replace with your actual package name

class CalculatorClient(Node):

    def __init__(self):
        super().__init__('calculator_client')
        self.cli = self.create_client(Calculator, 'calculate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b, operation):
        request = Calculator.Request()
        request.a = a
        request.b = b
        request.operation = operation
        future = self.cli.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    calculator_client = CalculatorClient()

    if len(sys.argv) != 4:
        print('Usage: python3 calculator_client.py <num1> <num2> <operation>')
        print('Operations: add, subtract, multiply, divide')
        return

    a = float(sys.argv[1])
    b = float(sys.argv[2])
    operation = sys.argv[3]

    future = calculator_client.send_request(a, b, operation)
    rclpy.spin_until_future_complete(calculator_client, future)

    try:
        response = future.result()
        if response.success:
            print(f'Result: {response.message}')
        else:
            print(f'Error: {response.message}')
    except Exception as e:
        print(f'Service call failed: {e}')

    calculator_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 3: Action-Based Navigation

### Action Definition
Create an action definition file `Navigate.action` in an `action` directory:

```
# Goal
float64 target_x
float64 target_y
---
# Result
float64 final_x
float64 final_y
bool success
string message
---
# Feedback
float64 current_x
float64 current_y
float64 distance_remaining
int32 percentage_complete
```

### Action Server Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package.action import Navigate  # Replace with your actual package name
import time

class NavigateActionServer(Node):

    def __init__(self):
        super().__init__('navigate_action_server')
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')

        # Simulate navigation
        start_x, start_y = 0.0, 0.0  # Starting position
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        # Calculate distance
        total_distance = ((target_x - start_x)**2 + (target_y - start_y)**2)**0.5
        current_x, current_y = start_x, start_y

        # Navigation simulation
        step_size = 0.1  # Move 0.1 units per step
        steps = int(total_distance / step_size)

        feedback_msg = Navigate.Feedback()

        for i in range(steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Navigate.Result()
                result.success = False
                result.message = 'Goal canceled'
                return result

            # Calculate current position
            ratio = i / steps if steps > 0 else 1.0
            current_x = start_x + (target_x - start_x) * ratio
            current_y = start_y + (target_y - start_y) * ratio

            # Calculate remaining distance
            distance_remaining = ((target_x - current_x)**2 + (target_y - current_y)**2)**0.5
            percentage_complete = int(ratio * 100)

            # Publish feedback
            feedback_msg.current_x = current_x
            feedback_msg.current_y = current_y
            feedback_msg.distance_remaining = distance_remaining
            feedback_msg.percentage_complete = percentage_complete

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {percentage_complete}% - Distance remaining: {distance_remaining:.2f}')

            # Simulate movement time
            time.sleep(0.1)

        # Complete the goal
        goal_handle.succeed()
        result = Navigate.Result()
        result.final_x = current_x
        result.final_y = current_y
        result.success = True
        result.message = f'Navigation completed successfully to ({current_x:.2f}, {current_y:.2f})'

        self.get_logger().info(result.message)
        return result

def main(args=None):
    rclpy.init(args=args)
    navigate_action_server = NavigateActionServer()

    try:
        rclpy.spin(navigate_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package.action import Navigate  # Replace with your actual package name
import time

class NavigateActionClient(Node):

    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(
            self,
            Navigate,
            'navigate')

    def send_goal(self, target_x, target_y):
        goal_msg = Navigate.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self._action_client.wait_for_server()

        # Send the goal
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
        self.get_logger().info(
            f'Current position: ({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'Distance remaining: {feedback.distance_remaining:.2f}, '
            f'Progress: {feedback.percentage_complete}%')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateActionClient()

    # Send a goal to navigate to position (5.0, 3.0)
    action_client.send_goal(5.0, 3.0)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 4: QoS Configuration Challenge

### QoS Example Code
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')

        # QoS profile with RELIABLE reliability and KEEP_LAST history
        qos_profile_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.publisher_reliable = self.create_publisher(Float64, 'temperature_reliable', qos_profile_reliable)

        # QoS profile with BEST_EFFORT reliability and KEEP_ALL history
        qos_profile_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_ALL
        )

        self.publisher_best_effort = self.create_publisher(Float64, 'temperature_best_effort', qos_profile_best_effort)

        timer_period = 1.0  # seconds
        self.timer_reliable = self.create_timer(timer_period, self.timer_callback_reliable)
        self.timer_best_effort = self.create_timer(timer_period + 0.5, self.timer_callback_best_effort)

        self.counter = 0

    def timer_callback_reliable(self):
        msg = Float64()
        msg.data = 20.0 + (self.counter % 5)  # Vary temperature slightly
        self.publisher_reliable.publish(msg)
        self.get_logger().info(f'Published (RELIABLE): {msg.data:.2f}')
        self.counter += 1

    def timer_callback_best_effort(self):
        msg = Float64()
        msg.data = 25.0 + (self.counter % 5)  # Vary temperature slightly
        self.publisher_best_effort.publish(msg)
        self.get_logger().info(f'Published (BEST_EFFORT): {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()

    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Solution for Exercise 5: Parameter-Based Configuration

### Parameter-Based Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('message_content', 'Hello from parameter node!')
        self.declare_parameter('node_name', 'parameter_node')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.message_content = self.get_parameter('message_content').value
        self.node_name = self.get_parameter('node_name').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'parameter_topic', 10)

        # Create timer based on parameter
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Parameter node initialized with rate: {self.publish_rate}Hz')

    def timer_callback(self):
        msg = String()
        msg.data = f'[{self.node_name}] {self.message_content} (Rate: {self.publish_rate}Hz)'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_rate':
                self.publish_rate = param.value
                # Update timer with new rate
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
                self.get_logger().info(f'Publish rate updated to: {self.publish_rate}Hz')
            elif param.name == 'message_content':
                self.message_content = param.value
                self.get_logger().info(f'Message content updated to: {self.message_content}')
            elif param.name == 'node_name':
                self.node_name = param.value
                self.get_logger().info(f'Node name updated to: {self.node_name}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()

    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Test Parameters
```bash
# Run the node
ros2 run your_package parameter_node

# In another terminal, change parameters
ros2 param set /parameter_node publish_rate 2.0
ros2 param set /parameter_node message_content "New parameter value!"
```

## Implementation Guide

### For Exercise 1:
1. Make sure to properly initialize rclpy with `rclpy.init()`
2. Use `rclpy.spin()` to keep nodes running
3. Always properly clean up with `destroy_node()` and `rclpy.shutdown()`
4. Use appropriate QoS settings for your use case

### For Exercise 2:
1. Define custom service interfaces in `.srv` files
2. Use `create_service()` to create service servers
3. Use `create_client()` to create service clients
4. Handle edge cases like division by zero
5. Properly handle asynchronous service calls

### For Exercise 3:
1. Define custom action interfaces in `.action` files
2. Use `ActionServer` for action servers
3. Use `ActionClient` for action clients
4. Implement proper feedback publishing during execution
5. Handle goal cancellation and preemption

### For Exercise 4:
1. Import QoS classes from `rclpy.qos`
2. Create `QoSProfile` with desired settings
3. Apply profiles to publishers and subscribers
4. Understand the trade-offs between different QoS settings

### For Exercise 5:
1. Use `declare_parameter()` to declare parameters
2. Use `get_parameter()` to retrieve parameter values
3. Implement parameter callbacks with `add_on_set_parameters_callback()`
4. Test parameter changes using ROS 2 command line tools

## Best Practices

1. **Error Handling**: Always implement proper error handling for services and actions
2. **Resource Management**: Properly clean up resources when nodes are destroyed
3. **Logging**: Use appropriate log levels (info, warn, error) for different messages
4. **Documentation**: Comment your code to explain the purpose of different sections
5. **Testing**: Test your nodes in isolation before integrating them into larger systems
6. **QoS Selection**: Choose appropriate QoS settings based on your application requirements
7. **Parameter Validation**: Validate parameter values to prevent unexpected behavior

## Common Pitfalls to Avoid

1. **Forgetting to initialize rclpy**: Always call `rclpy.init()` before creating nodes
2. **Not handling node destruction**: Always properly clean up nodes and shutdown rclpy
3. **Using inappropriate QoS settings**: Consider your application's requirements for reliability and performance
4. **Not handling service/client timeouts**: Implement appropriate timeout handling for services
5. **Ignoring parameter validation**: Validate parameters to prevent invalid configurations
6. **Not using proper message types**: Use appropriate standard message types from std_msgs or geometry_msgs when possible

These solutions provide complete implementations for each exercise, demonstrating best practices for ROS 2 development. Use these as references when implementing your own solutions, and feel free to modify them to explore different approaches or add additional functionality.