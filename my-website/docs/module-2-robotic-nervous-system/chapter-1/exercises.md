# Exercises: ROS 2 Architecture and Communication Patterns

## Exercise 1: Basic Publisher-Subscriber

### Objective
Create a simple publisher-subscriber system to understand the basic communication pattern in ROS 2.

### Prerequisites
- Basic Python knowledge
- ROS 2 Humble Hawksbill installed
- Understanding of ROS 2 nodes and topics

### Problem Statement
Create two nodes: a publisher that sends temperature readings and a subscriber that receives and processes these readings.

### Instructions
1. Create a publisher node that publishes temperature values (Float64 messages) to the topic `/temperature`
2. The publisher should send a new temperature value every 2 seconds
3. The temperature values should simulate realistic room temperatures (between 18°C and 25°C)
4. Create a subscriber node that listens to the `/temperature` topic
5. The subscriber should log received temperature values and indicate if they are within a comfortable range (&gt;20°C and &lt;24°C)
6. Run both nodes and observe the communication

### Expected Outcome
Two nodes communicating via the `/temperature` topic, with the subscriber successfully receiving and processing temperature values.

### Hints
- Use `std_msgs.msg.Float64` for temperature messages
- Use `self.create_publisher()` and `self.create_subscription()` methods
- Remember to initialize rclpy and spin the nodes

### Solution Approach
1. Create a node class inheriting from `rclpy.node.Node`
2. Initialize a publisher in the constructor
3. Create a timer to publish messages periodically
4. For the subscriber, create a callback function to handle incoming messages
5. Run both nodes using `rclpy.spin()`

## Exercise 2: Service-Based Calculator

### Objective
Implement a calculator service that can perform basic arithmetic operations.

### Prerequisites
- Understanding of ROS 2 services
- Knowledge of creating service servers and clients

### Problem Statement
Create a service that accepts two numbers and an operation type (add, subtract, multiply, divide) and returns the result.

### Instructions
1. Define a custom service interface with fields for two numbers, operation type, and result
2. Create a service server that implements the calculator functionality
3. Create a service client that sends requests to the server
4. Test the service with various operations
5. Handle division by zero appropriately

### Expected Outcome
A working service system that can perform arithmetic operations on demand.

### Hints
- Create a `.srv` file in a `srv` directory
- Use `ros2 run` to test the service client
- Handle edge cases like division by zero

### Solution Approach
1. Define the service interface in a `.srv` file
2. Create a service server node with the callback function
3. Create a service client node that sends requests
4. Test with various inputs and verify correct results

## Exercise 3: Action-Based Navigation

### Objective
Implement a navigation action that simulates moving a robot to a target position.

### Prerequisites
- Understanding of ROS 2 actions
- Knowledge of action servers and clients

### Problem Statement
Create an action that accepts a target position (x, y coordinates) and simulates navigating to that position, providing feedback on progress.

### Instructions
1. Define a custom action interface for navigation
2. Create an action server that simulates navigation to the target
3. The server should provide feedback on current progress (percentage complete)
4. Create an action client that sends navigation goals
5. The client should handle feedback and final results
6. Test with various target positions

### Expected Outcome
A working action system that simulates robot navigation with progress feedback.

### Hints
- Use `action_msgs` for action interfaces
- An action interface has three parts: Goal, Feedback, Result
- Implement proper feedback publishing during execution

### Solution Approach
1. Define the action interface in a `.action` file
2. Create an action server with execution callback
3. Implement feedback publishing during navigation simulation
4. Create an action client that sends goals and monitors progress
5. Test with different target positions

## Exercise 4: QoS Configuration Challenge

### Objective
Experiment with different Quality of Service settings and observe their effects.

### Prerequisites
- Understanding of ROS 2 QoS profiles
- Working publisher-subscriber system

### Problem Statement
Modify your publisher-subscriber system from Exercise 1 to experiment with different QoS settings and observe the effects on communication.

### Instructions
1. Modify your temperature publisher to use RELIABLE reliability
2. Change to BEST_EFFORT reliability and observe differences
3. Experiment with different history policies (KEEP_ALL, KEEP_LAST)
4. Test with different depth settings for message queues
5. Document the differences in behavior

### Expected Outcome
Understanding of how QoS settings affect message delivery and communication behavior.

### Hints
- Import QoS profiles from `rclpy.qos`
- Use `QoSProfile` to customize settings
- Observe behavior changes when modifying QoS parameters

### Solution Approach
1. Import necessary QoS classes
2. Create different QoS profiles
3. Apply profiles to publishers and subscribers
4. Run experiments and compare results
5. Document observations

## Exercise 5: Parameter-Based Configuration

### Objective
Create a node that uses parameters for runtime configuration.

### Prerequisites
- Understanding of ROS 2 parameters
- Working knowledge of node creation

### Problem Statement
Create a node that adjusts its behavior based on runtime parameters.

### Instructions
1. Create a node that publishes messages at a configurable rate
2. Use a parameter for the publication frequency (Hz)
3. Allow the parameter to be changed at runtime
4. Add a callback to handle parameter changes
5. Test changing the parameter while the node is running

### Expected Outcome
A node that responds to parameter changes without restarting.

### Hints
- Use `declare_parameter()` to declare parameters
- Use `add_on_set_parameters_callback()` for parameter change handling
- Use `ros2 param` command to change parameters at runtime

### Solution Approach
1. Declare parameters in the node constructor
2. Use parameters to control node behavior
3. Implement parameter callback to handle changes
4. Test parameter changes using ROS 2 command line tools

## Assessment Criteria

### For all exercises:
- Code follows ROS 2 best practices
- Proper error handling is implemented
- Code is well-documented with comments
- Nodes are properly initialized and cleaned up
- Appropriate message types are used

### Exercise 1:
- Publisher sends messages at regular intervals
- Subscriber receives and processes messages correctly
- Temperature values are within expected range

### Exercise 2:
- Service responds correctly to requests
- All arithmetic operations work properly
- Edge cases (like division by zero) are handled

### Exercise 3:
- Action provides appropriate feedback during execution
- Result is returned when navigation completes
- Client properly handles feedback and results

### Exercise 4:
- Different QoS settings are properly applied
- Effects of different settings are observed and documented

### Exercise 5:
- Parameters are properly declared and used
- Runtime parameter changes are handled correctly
- Parameter callback functions work as expected

## Extension Activities

1. **Advanced Exercise**: Combine multiple communication patterns (topics, services, actions) in a single application
2. **Challenge Exercise**: Implement a distributed system with multiple nodes coordinating via different communication methods
3. **Research Exercise**: Investigate the performance implications of different QoS settings in various network conditions

## Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings/)
- [ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/)
- [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/)