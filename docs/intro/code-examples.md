---
sidebar_position: 8
title: Code Examples for Physical AI
---

# Code Examples for Physical AI

This page provides practical code examples that demonstrate core Physical AI concepts using Python and PyBullet. These examples illustrate the principles discussed in the introduction and provide a foundation for more advanced implementations.

## Basic Physical AI Concepts in Code

### 1. Robot State Representation

In Physical AI, robots are represented by their state in the physical world:

```python
import numpy as np

class RobotState:
    def __init__(self, position, orientation, joint_angles, velocities):
        """
        Represents the state of a robot in the physical world
        """
        self.position = np.array(position)  # 3D position [x, y, z]
        self.orientation = np.array(orientation)  # 4D quaternion [w, x, y, z]
        self.joint_angles = np.array(joint_angles)  # Joint positions
        self.velocities = np.array(velocities)  # Joint velocities

    def update_position(self, new_position):
        """Update the robot's position in the environment"""
        self.position = np.array(new_position)

    def get_configuration_space(self):
        """Return the robot's configuration in joint space"""
        return self.joint_angles

    def get_task_space(self):
        """Return the robot's position in Cartesian space"""
        return self.position
```

### 2. Sensor Data Processing

Physical AI systems must process sensor data to understand their environment:

```python
import numpy as np

class SensorProcessor:
    def __init__(self):
        self.sensor_noise = 0.01  # Standard deviation of sensor noise

    def process_lidar_data(self, raw_data):
        """
        Process raw LIDAR data to detect obstacles
        """
        # Apply noise model to simulate real-world sensor noise
        noisy_data = raw_data + np.random.normal(0, self.sensor_noise, raw_data.shape)

        # Filter out distances beyond maximum range
        valid_distances = noisy_data[noisy_data < 10.0]  # 10m max range

        # Identify potential obstacles
        obstacles = []
        for i, distance in enumerate(valid_distances):
            if distance < 0.5:  # Obstacle within 50cm
                obstacles.append({
                    'angle': i * (2 * np.pi / len(raw_data)),
                    'distance': distance
                })

        return obstacles

    def process_camera_data(self, image):
        """
        Process camera image to detect objects
        """
        # Simple edge detection (in practice, you'd use more sophisticated methods)
        edges = self.detect_edges(image)

        # Identify objects based on edge patterns
        objects = self.identify_objects_from_edges(edges)

        return objects

    def detect_edges(self, image):
        """Simple edge detection"""
        # In practice, you'd use more sophisticated techniques like Canny edge detection
        return np.gradient(image)

    def identify_objects_from_edges(self, edges):
        """Identify objects based on edge patterns"""
        # Simplified object identification
        objects = []
        # In practice, you'd use computer vision techniques like contour detection
        return objects
```

### 3. Control Systems for Physical AI

Implementing control systems that operate in real-time with physical constraints:

```python
import numpy as np
import time

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        """
        PID controller for robot control
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        dt: Time step
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.previous_error = 0
        self.integral = 0

    def compute(self, target, current):
        """
        Compute control output based on target and current values
        """
        error = target - current

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d_term = self.kd * derivative

        # Store error for next iteration
        self.previous_error = error

        # Return control output
        return p_term + i_term + d_term

class RobotController:
    def __init__(self):
        # Initialize PID controllers for different joints
        self.joint_controllers = {
            'joint_1': PIDController(kp=2.0, ki=0.1, kd=0.01),
            'joint_2': PIDController(kp=2.0, ki=0.1, kd=0.01),
            'joint_3': PIDController(kp=2.0, ki=0.1, kd=0.01)
        }

    def control_step(self, target_positions, current_positions):
        """
        Compute control commands for all joints
        """
        control_outputs = {}

        for joint_name, target_pos in target_positions.items():
            current_pos = current_positions.get(joint_name, 0)
            controller = self.joint_controllers[joint_name]

            control_output = controller.compute(target_pos, current_pos)
            control_outputs[joint_name] = control_output

        return control_outputs
```

### 4. Simulation-First Approach with PyBullet

Implementing the simulation-first methodology:

```python
import pybullet as p
import pybullet_data
import numpy as np

class SimulationEnvironment:
    def __init__(self, use_gui=True):
        """
        Initialize PyBullet simulation environment
        """
        if use_gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        # Set gravity
        p.setGravity(0, 0, -9.81)

        # Load plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")

        # Robot ID placeholder
        self.robot_id = None

    def load_robot(self, urdf_path, start_position=[0, 0, 1]):
        """
        Load robot into the simulation
        """
        self.robot_id = p.loadURDF(urdf_path, start_position)
        return self.robot_id

    def get_robot_state(self):
        """
        Get current state of the robot
        """
        if self.robot_id is None:
            return None

        # Get base position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)

        # Get joint states
        joint_states = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointState(self.robot_id, i)
            joint_states.append({
                'position': joint_info[0],
                'velocity': joint_info[1],
                'force': joint_info[3]
            })

        return {
            'position': pos,
            'orientation': orn,
            'joint_states': joint_states
        }

    def apply_control_commands(self, joint_commands):
        """
        Apply control commands to robot joints
        """
        for joint_idx, command in enumerate(joint_commands):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=command
            )

    def step_simulation(self):
        """
        Step the simulation forward
        """
        p.stepSimulation()

    def disconnect(self):
        """
        Disconnect from physics server
        """
        p.disconnect(self.physics_client)

# Example usage of simulation environment
def example_simulation_usage():
    """
    Example of how to use the simulation environment
    """
    # Create simulation environment
    sim_env = SimulationEnvironment(use_gui=True)

    # Load a simple robot (using KUKA iiwa as example)
    robot_id = sim_env.load_robot("kuka_iiwa/model.urdf")

    # Run simulation for a while
    for i in range(1000):  # 1000 steps
        # Get current robot state
        state = sim_env.get_robot_state()

        # Simple control: move to a target position
        target_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        sim_env.apply_control_commands(target_positions)

        # Step simulation
        sim_env.step_simulation()

        # Small delay to visualize
        if i % 100 == 0:  # Print every 100 steps
            print(f"Step {i}: Robot position = {state['position'] if state else 'N/A'}")

    # Clean up
    sim_env.disconnect()
```

### 5. Embodied Intelligence in Practice

Implementing concepts of embodied intelligence:

```python
import numpy as np

class EmbodiedAgent:
    def __init__(self, body_properties, environment_properties):
        """
        An agent that embodies intelligence through its interaction with the environment
        """
        self.body = body_properties  # Properties of the agent's body
        self.environment = environment_properties  # Properties of the environment
        self.state = np.zeros(10)  # Internal state representation
        self.sensors = []
        self.actuators = []

    def perceive(self, environment_state):
        """
        Agent perceives the environment through its sensors
        """
        # In embodied cognition, perception is active and guided by motor intentions
        sensor_data = self.process_environment_state(environment_state)

        # Update internal state based on perception
        self.update_internal_state(sensor_data)

        return sensor_data

    def act(self, sensor_data):
        """
        Agent acts on the environment through its actuators
        """
        # Decision making based on sensor data and internal state
        motor_commands = self.decision_process(sensor_data)

        # Execute actions through actuators
        self.execute_motor_commands(motor_commands)

        return motor_commands

    def process_environment_state(self, env_state):
        """
        Process environmental information through the agent's body
        """
        # The agent's body affects what can be perceived
        # For example, sensor positions determine what can be sensed
        processed_data = {}

        for sensor in self.sensors:
            # Each sensor provides data based on its position and the environment
            sensor_data = sensor.sense(env_state, self.body.position)
            processed_data[sensor.name] = sensor_data

        return processed_data

    def decision_process(self, sensor_data):
        """
        Decision process that considers the agent's embodiment
        """
        # In embodied cognition, decision making is influenced by the body
        # The agent's physical constraints affect what actions are possible

        # Simple example: if obstacle detected in front, move around it
        if 'front_sensor' in sensor_data and sensor_data['front_sensor'] < 0.5:
            # Body configuration affects how to move around obstacle
            if self.body.wheel_base > 0.3:  # Wheeled robot
                return {'left_motor': -0.5, 'right_motor': 0.5}  # Turn right
            else:  # Legged robot
                return {'left_leg': 0.1, 'right_leg': -0.1}  # Step aside

        return {}  # No action needed

    def update_internal_state(self, sensor_data):
        """
        Update internal state based on sensory input
        """
        # In embodied cognition, internal state is shaped by interaction
        # with the environment through the body
        pass

    def execute_motor_commands(self, commands):
        """
        Execute motor commands through actuators
        """
        for actuator_name, command in commands.items():
            actuator = next((a for a in self.actuators if a.name == actuator_name), None)
            if actuator:
                actuator.execute(command)

# Example: Creating an embodied agent
def create_simple_embodied_agent():
    """
    Create a simple embodied agent to demonstrate the concept
    """
    body_props = {
        'position': [0, 0, 0],
        'wheel_base': 0.3,  # Distance between wheels
        'mass': 1.0,
        'size': [0.2, 0.2, 0.1]  # width, length, height
    }

    env_props = {
        'size': [10, 10],  # 10x10 environment
        'obstacles': [{'position': [2, 2], 'size': [0.5, 0.5]}]
    }

    agent = EmbodiedAgent(body_props, env_props)

    # Add simple sensors and actuators
    class SimpleSensor:
        def __init__(self, name, position):
            self.name = name
            self.position = position

        def sense(self, env_state, agent_pos):
            # Simple distance measurement
            obstacle_pos = env_props['obstacles'][0]['position']
            distance = np.linalg.norm(np.array(obstacle_pos) - np.array(agent_pos))
            return distance

    class SimpleActuator:
        def __init__(self, name):
            self.name = name

        def execute(self, command):
            # Execute the command (in simulation, this would affect the physics)
            pass

    agent.sensors = [SimpleSensor('front_sensor', [0.1, 0, 0])]
    agent.actuators = [SimpleActuator('left_motor'), SimpleActuator('right_motor')]

    return agent
```

## Key Takeaways

1. **Embodiment Matters**: In Physical AI, the body is not just a vessel but an integral part of the cognitive system
2. **Real-time Processing**: Physical systems must operate within real-time constraints
3. **Uncertainty Management**: Physical systems must handle sensor noise and environmental uncertainty
4. **Simulation-First**: Develop and test in simulation before real-world deployment
5. **Safety-Critical**: Physical AI systems must prioritize safety in all operations

These code examples provide a foundation for understanding how Physical AI concepts translate into practical implementations. Each example demonstrates a key principle while maintaining the simulation-first approach that is central to safe and effective Physical AI development.

## Next Steps

- Experiment with the provided PyBullet examples in your own environment
- Modify the code examples to explore different scenarios
- Apply these concepts to the four-module structure outlined in the book
- Consider how these implementations would need to be adapted for real-world deployment