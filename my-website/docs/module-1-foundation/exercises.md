# Module 1: Exercises and Implementation Guides

## Overview
This document contains exercises and implementation guides for Module 1: Foundations of Physical AI & Humanoid Robotics. These exercises are designed to reinforce the concepts covered in the four chapters of this module:
1. Mathematical Foundations
2. Kinematics and Dynamics
3. Sensing and Perception
4. Embodied Intelligence

## Chapter 1: Mathematical Foundations - Exercises

### Exercise 1.1: Vector Operations in Robotics
Implement functions to perform common vector operations used in robotics.

```python
import numpy as np

def normalize_vector(vector):
    """
    Normalize a vector to unit length

    Args:
        vector: numpy array representing a 3D vector

    Returns:
        normalized vector
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm

def vector_projection(v1, v2):
    """
    Calculate the projection of v1 onto v2

    Args:
        v1, v2: numpy arrays representing 3D vectors

    Returns:
        projection of v1 onto v2
    """
    v2_norm = normalize_vector(v2)
    scalar_proj = np.dot(v1, v2_norm)
    return scalar_proj * v2_norm

def cross_product_matrix(vector):
    """
    Create the skew-symmetric matrix for cross product operation
    Such that cross(a,b) = [a]_× b

    Args:
        vector: numpy array representing a 3D vector

    Returns:
        3x3 skew-symmetric matrix
    """
    x, y, z = vector
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])

# Test the functions
v1 = np.array([1, 2, 3])
v2 = np.array([4, 5, 6])

normalized_v1 = normalize_vector(v1)
projection = vector_projection(v1, v2)
cross_matrix = cross_product_matrix(v1)

print(f"Original vector v1: {v1}")
print(f"Normalized v1: {normalized_v1}")
print(f"Projection of v1 onto v2: {projection}")
print(f"Cross product matrix of v1:\n{cross_matrix}")
```

### Exercise 1.2: Transformation Matrices
Implement functions for creating and using transformation matrices in robotics.

```python
def rotation_matrix_x(angle):
    """Rotation matrix around X-axis"""
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotation_matrix_y(angle):
    """Rotation matrix around Y-axis"""
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotation_matrix_z(angle):
    """Rotation matrix around Z-axis"""
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

def homogeneous_transform(rotation_matrix, translation_vector):
    """Create a 4x4 homogeneous transformation matrix"""
    T = np.eye(4)
    T[0:3, 0:3] = rotation_matrix
    T[0:3, 3] = translation_vector
    return T

def transform_point(point, transformation_matrix):
    """
    Transform a 3D point using a 4x4 homogeneous transformation matrix
    """
    # Convert point to homogeneous coordinates
    homogeneous_point = np.append(point, 1)

    # Apply transformation
    transformed_homogeneous = transformation_matrix @ homogeneous_point

    # Convert back to 3D coordinates
    transformed_point = transformed_homogeneous[:3]

    return transformed_point

# Example: Create a transformation and apply it
angle = np.pi / 4  # 45 degrees
R_z = rotation_matrix_z(angle)
translation = np.array([1, 2, 3])
T = homogeneous_transform(R_z, translation)

point = np.array([1, 0, 0])
transformed_point = transform_point(point, T)

print(f"Original point: {point}")
print(f"Transformation matrix:\n{T}")
print(f"Transformed point: {transformed_point}")
```

### Exercise 1.3: Probability and Statistics for Sensor Fusion
Implement functions for handling uncertainty in sensor data.

```python
def gaussian_pdf(x, mean, std_dev):
    """Calculate probability density for Gaussian distribution"""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = -0.5 * ((x - mean) / std_dev) ** 2
    return coefficient * np.exp(exponent)

def bayes_update(prior, likelihood, evidence):
    """Apply Bayes' theorem to update probability"""
    posterior = (likelihood * prior) / evidence
    return posterior

def weighted_average_fusion(measurements, uncertainties):
    """
    Fuse multiple sensor measurements using weighted average
    measurements: list of measured values
    uncertainties: list of uncertainty values (standard deviations)
    """
    # Calculate weights (inverse of variance)
    weights = [1.0 / (unc**2) for unc in uncertainties]

    # Calculate weighted sum
    weighted_sum = sum(m * w for m, w in zip(measurements, weights))
    total_weight = sum(weights)

    # Calculate fused estimate
    fused_estimate = weighted_sum / total_weight

    # Calculate fused uncertainty
    fused_uncertainty = np.sqrt(1.0 / total_weight)

    return fused_estimate, fused_uncertainty

# Example: Sensor fusion
measurements = [10.2, 9.8, 10.1]
uncertainties = [0.5, 0.8, 0.3]

fused_result, fused_unc = weighted_average_fusion(measurements, uncertainties)

print(f"Measurements: {measurements}")
print(f"Uncertainties: {uncertainties}")
print(f"Fused result: {fused_result:.3f} ± {fused_unc:.3f}")

# Example: Bayes' theorem
prior_prob = 0.3  # Prior probability
sensor_likelihood = 0.8  # P(evidence|hypothsis)
total_evidence = 0.5  # P(evidence)
posterior_prob = bayes_update(prior_prob, sensor_likelihood, total_evidence)

print(f"Prior: {prior_prob:.3f}, Likelihood: {sensor_likelihood:.3f}")
print(f"Posterior: {posterior_prob:.3f}")
```

## Chapter 2: Kinematics and Dynamics - Exercises

### Exercise 2.1: Forward Kinematics
Implement forward kinematics for a simple robotic arm.

```python
def dh_transform(a, alpha, d, theta):
    """
    Denavit-Hartenberg transformation matrix
    a: link length
    alpha: link twist
    d: link offset
    theta: joint angle
    """
    T = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics_planar_2dof(theta1, theta2, l1, l2):
    """
    Forward kinematics for 2-DOF planar manipulator
    """
    # Link 1 transformation
    T1 = dh_transform(l1, 0, 0, theta1)

    # Link 2 transformation relative to link 1
    T2 = dh_transform(l2, 0, 0, theta2)

    # Total transformation from base to end-effector
    T_total = T1 @ T2

    # Extract end-effector position
    x = T_total[0, 3]
    y = T_total[1, 3]

    return np.array([x, y])

# Example: Calculate end-effector position
theta1 = np.pi/4  # 45 degrees
theta2 = np.pi/6  # 30 degrees
l1 = 1.0  # Link 1 length
l2 = 0.8  # Link 2 length

end_effector_pos = forward_kinematics_planar_2dof(theta1, theta2, l1, l2)
print(f"End-effector position: ({end_effector_pos[0]:.3f}, {end_effector_pos[1]:.3f})")
```

### Exercise 2.2: Inverse Kinematics
Implement inverse kinematics for a simple robotic arm.

```python
def inverse_kinematics_planar_2dof(x, y, l1, l2):
    """
    Inverse kinematics for 2-DOF planar manipulator
    """
    # Check if position is reachable
    r = np.sqrt(x**2 + y**2)
    if r > l1 + l2:
        print("Position is outside workspace")
        return None

    if r < abs(l1 - l2):
        print("Position is inside workspace but unreachable")
        return None

    # Calculate theta2
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # Calculate theta1
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return np.array([theta1, theta2])

# Example: Find joint angles for desired position
desired_pos = np.array([1.2, 0.8])
angles = inverse_kinematics_planar_2dof(desired_pos[0], desired_pos[1], l1, l2)

if angles is not None:
    print(f"Required joint angles: [{np.degrees(angles[0]):.2f}°, {np.degrees(angles[1]):.2f}°]")

    # Verify with forward kinematics
    verify_pos = forward_kinematics_planar_2dof(angles[0], angles[1], l1, l2)
    print(f"Verification - Forward kinematics result: ({verify_pos[0]:.3f}, {verify_pos[1]:.3f})")
    print(f"Desired position: ({desired_pos[0]}, {desired_pos[1]})")
```

### Exercise 2.3: Robot Dynamics
Implement basic dynamic calculations for a robotic system.

```python
def simple_pendulum_dynamics(theta, theta_dot, mass, length, gravity=9.81):
    """
    Calculate dynamics for a simple pendulum
    """
    # Equation of motion for simple pendulum: θ_ddot = -(g/l)*sin(θ)
    theta_ddot = -(gravity / length) * np.sin(theta)

    # Calculate kinetic and potential energy
    kinetic_energy = 0.5 * mass * (length * theta_dot)**2
    potential_energy = mass * gravity * length * (1 - np.cos(theta))

    return theta_ddot, kinetic_energy, potential_energy

# Example: Calculate pendulum dynamics
mass = 1.0  # kg
length = 1.0  # m
theta = np.pi/6  # 30 degrees
theta_dot = 0.5  # rad/s

theta_ddot, ke, pe = simple_pendulum_dynamics(theta, theta_dot, mass, length)
mechanical_energy = ke + pe

print(f"Pendulum dynamics:")
print(f"Angle: {np.degrees(theta):.2f}°, Angular velocity: {theta_dot:.3f} rad/s")
print(f"Angular acceleration: {theta_ddot:.3f} rad/s²")
print(f"Kinetic energy: {ke:.3f} J")
print(f"Potential energy: {pe:.3f} J")
print(f"Mechanical energy: {mechanical_energy:.3f} J")
```

## Chapter 3: Sensing and Perception - Exercises

### Exercise 3.1: Sensor Models
Implement basic sensor models for different types of sensors.

```python
class RangeSensor:
    def __init__(self, max_range=10.0, min_range=0.1, accuracy=0.01, fov=30):
        """
        Range sensor simulator (e.g., ultrasonic, IR, LiDAR)
        """
        self.max_range = max_range
        self.min_range = min_range
        self.accuracy = accuracy  # measurement accuracy
        self.fov = fov  # field of view in degrees

    def measure_distance(self, true_distance, add_noise=True):
        """Measure distance with sensor limitations and noise"""
        # Check if within range
        if true_distance > self.max_range:
            return float('inf')  # Out of range
        elif true_distance < self.min_range:
            return self.min_range  # Too close

        if add_noise:
            noise = np.random.normal(0, self.accuracy)
            measured = true_distance + noise
            # Ensure within bounds
            measured = max(self.min_range, min(self.max_range, measured))
            return measured
        return true_distance

    def detect_object(self, true_distance, threshold=None):
        """Detect if an object is within range"""
        if threshold is None:
            threshold = self.max_range
        measured_dist = self.measure_distance(true_distance)
        return measured_dist < threshold and measured_dist != float('inf')

# Example: Range sensor usage
range_sensor = RangeSensor(max_range=5.0, min_range=0.05, accuracy=0.02)

# Test measurements at different distances
test_distances = [0.5, 1.0, 2.0, 4.0, 6.0, 0.02]
for dist in test_distances:
    measured = range_sensor.measure_distance(dist)
    detected = range_sensor.detect_object(dist)
    print(f"True: {dist:.2f}m -> Measured: {measured:.2f}m, Detected: {detected}")
```

### Exercise 3.2: Kalman Filter Implementation
Implement a simple Kalman filter for state estimation.

```python
class SimpleKalmanFilter:
    def __init__(self, initial_state, initial_uncertainty, process_noise, measurement_noise):
        """
        Simple Kalman filter for 1D position tracking
        """
        self.x = initial_state  # State (position)
        self.P = initial_uncertainty  # Uncertainty
        self.Q = process_noise  # Process noise
        self.R = measurement_noise  # Measurement noise

    def predict(self, dt, control_input=0):
        """
        Prediction step
        """
        # For constant velocity model: x = x + v*dt
        # We assume velocity is part of state or control
        self.x = self.x + control_input * dt
        self.P = self.P + self.Q

    def update(self, measurement):
        """
        Update step
        """
        # Calculate Kalman gain
        S = self.P + self.R
        K = self.P / S

        # Update state estimate
        innovation = measurement - self.x
        self.x = self.x + K * innovation

        # Update uncertainty
        self.P = (1 - K) * self.P

# Example: Track a moving object
kf = SimpleKalmanFilter(initial_state=0.0, initial_uncertainty=10.0,
                       process_noise=0.1, measurement_noise=1.0)

# Simulate measurements
true_positions = []
measurements = []
estimates = []
times = []

dt = 0.1
for t in np.arange(0, 5, dt):
    # True position (with some motion)
    true_pos = 0.1 * t**2  # Accelerating motion
    true_positions.append(true_pos)

    # Noisy measurement
    measured_pos = true_pos + np.random.normal(0, 0.5)
    measurements.append(measured_pos)

    # Kalman filter update
    kf.predict(dt, control_input=0.2*t)  # Approximate velocity
    kf.update(measured_pos)
    estimates.append(kf.x)
    times.append(t)

print(f"Kalman filter example completed with {len(times)} steps")
print(f"Final estimate: {kf.x:.3f}, Final measurement: {measurements[-1]:.3f}")
```

## Chapter 4: Embodied Intelligence - Exercises

### Exercise 4.1: Embodied Agent Simulation
Implement a simple embodied agent that interacts with its environment.

```python
class EmbodiedAgent:
    """
    Simple embodied agent demonstrating basic principles
    """
    def __init__(self, position=np.array([0.0, 0.0]), mass=1.0):
        self.position = position
        self.velocity = np.array([0.0, 0.0])
        self.mass = mass
        self.energy = 100.0  # Energy level

    def sense_environment(self, environment):
        """
        Sense the environment
        """
        sensor_data = {
            'distance_to_goal': np.linalg.norm(environment.goal - self.position),
            'obstacle_proximity': self._check_obstacles(environment),
            'energy_level': self.energy
        }
        return sensor_data

    def _check_obstacles(self, environment):
        """
        Check for obstacles in the environment
        """
        min_distance = float('inf')
        for obstacle in environment.obstacles:
            distance = np.linalg.norm(self.position - obstacle['position'])
            if distance < min_distance:
                min_distance = distance
        return min_distance

    def act(self, sensor_data, environment):
        """
        Act based on sensor data
        """
        # Simple navigation behavior
        direction_to_goal = environment.goal - self.position
        distance_to_goal = np.linalg.norm(direction_to_goal)

        if distance_to_goal < 0.1:  # Reached goal
            return np.array([0.0, 0.0])

        # Normalize direction
        if distance_to_goal > 0:
            direction_to_goal = direction_to_goal / distance_to_goal

        # Simple obstacle avoidance
        if sensor_data['obstacle_proximity'] < 1.0:
            # Move perpendicular to obstacle
            obstacle_direction = self.position - environment.obstacles[0]['position']
            obstacle_direction = obstacle_direction / np.linalg.norm(obstacle_direction)
            avoidance = np.array([-obstacle_direction[1], obstacle_direction[0]])
            direction_to_goal = 0.7 * direction_to_goal + 0.3 * avoidance

        # Calculate required force (based on energy constraint)
        desired_velocity = direction_to_goal * min(2.0, self.energy / 50.0)  # Slower when low energy
        force = (desired_velocity - self.velocity) * self.mass

        # Consume energy based on action
        energy_cost = np.linalg.norm(force) * 0.1
        self.energy = max(0, self.energy - energy_cost)

        return force

    def update(self, force, dt=0.1):
        """
        Update agent state based on applied force
        """
        # Apply force: F = ma => a = F/m
        acceleration = force / self.mass

        # Update velocity and position
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

class SimpleEnvironment:
    def __init__(self):
        self.goal = np.array([10.0, 10.0])
        self.obstacles = [
            {'position': np.array([5.0, 5.0]), 'radius': 1.0}
        ]

# Example: Run embodied agent simulation
env = SimpleEnvironment()
agent = EmbodiedAgent(position=np.array([0.0, 0.0]))

print("Starting embodied agent simulation...")
print(f"Goal: {env.goal}, Starting position: {agent.position}")

# Run simulation
for step in range(100):
    sensor_data = agent.sense_environment(env)
    force = agent.act(sensor_data, env)
    agent.update(force, dt=0.1)

    # Check if goal reached
    if np.linalg.norm(agent.position - env.goal) < 0.5:
        print(f"Goal reached at step {step}! Final position: {agent.position}")
        break

    # Print status periodically
    if step % 20 == 0:
        print(f"Step {step}: Position={agent.position}, Energy={agent.energy:.1f}")

print(f"Final position: {agent.position}, Energy: {agent.energy:.1f}")
```

### Exercise 4.2: Morphological Computation
Demonstrate how physical properties can perform computation.

```python
class MorphologicalComputer:
    """
    System that uses physical properties for computation
    """
    def __init__(self, material_type='elastic'):
        self.material_type = material_type
        self.state = 0
        self.memory = []  # Short-term memory through physical state

    def process_signal(self, input_signal):
        """
        Process signal using material properties
        """
        if self.material_type == 'elastic':
            # Elastic material stores and releases energy - acts like a filter
            processed = input_signal * 0.7 + self.state * 0.3  # Some memory
            self.state = processed
            self.memory.append(processed)

            # Keep only last 5 values in memory
            if len(self.memory) > 5:
                self.memory.pop(0)

            # Return smoothed signal based on recent history
            return sum(self.memory) / len(self.memory)

        elif self.material_type == 'viscous':
            # Viscous material dampens signals - acts like a low-pass filter
            filtered = self.state * 0.8 + input_signal * 0.2
            self.state = filtered
            return filtered

        elif self.material_type == 'adaptive':
            # Adaptive material changes properties based on input
            if abs(input_signal) > 1.0:
                # High input makes material stiffer
                processed = input_signal * 0.9
            else:
                # Low input allows more compliance
                processed = input_signal * 0.5 + self.state * 0.5

            self.state = processed
            return processed

# Example: Compare different material computations
materials = ['elastic', 'viscous', 'adaptive']
input_signals = [1.0, -0.5, 2.0, 0.3, -1.2, 0.8]

for material in materials:
    computer = MorphologicalComputer(material)
    print(f"\n{material.capitalize()} material processing:")

    for i, signal in enumerate(input_signals):
        output = computer.process_signal(signal)
        print(f"  Input: {signal:5.2f} -> Output: {output:5.2f}")
```

## Implementation Guide

### Setting Up Your Development Environment

1. **Install Required Libraries**:
```bash
pip install numpy matplotlib scipy pybullet
```

2. **Verify Installation**:
```python
import numpy as np
import matplotlib.pyplot as plt
print("Environment ready!")
```

### Running the Exercises

1. Copy the code for each exercise into a Python file
2. Run the code to see the results
3. Modify parameters to understand how different values affect the results
4. Try to extend the examples with your own variations

### Extending the Exercises

1. **Mathematical Foundations**: Try implementing quaternions for rotation representation
2. **Kinematics**: Extend the 2-DOF example to 3-DOF or more complex manipulators
3. **Sensing**: Add more sophisticated sensor models (camera, IMU)
4. **Embodied Intelligence**: Create more complex environments with multiple goals

### Troubleshooting Tips

- Ensure NumPy is properly installed for all mathematical operations
- For PyBullet exercises, install with: `pip install pybullet`
- If getting errors with matrix operations, check dimensions carefully
- Use `np.linalg.norm()` to compute vector magnitudes safely