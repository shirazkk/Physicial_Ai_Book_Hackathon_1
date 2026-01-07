# Chapter 1: Mathematical Foundations for Physical AI & Robotics

## Learning Objectives
By the end of this chapter, readers will be able to:
- Apply linear algebra concepts to robotics problems including transformations and rotations
- Use calculus and differential equations to model robotic dynamics
- Apply probability and statistics for sensor fusion in robotic perception
- Implement mathematical concepts using PyBullet simulation examples
- Perform matrix transformations relevant to robotics applications

## Prerequisites
- Basic understanding of linear algebra (vectors, matrices)
- Fundamental calculus knowledge
- Basic Python programming skills
- Familiarity with mathematical notation

## Introduction

Mathematics forms the foundation of all robotics and Physical AI systems. From describing the position and orientation of robotic components to modeling their motion and interactions with the environment, mathematical concepts are essential tools for any roboticist. This chapter covers the core mathematical foundations needed for understanding and implementing robotic systems, with a focus on practical applications in Physical AI.

## 1. Linear Algebra Applications in Robotics

Linear algebra is fundamental to robotics, providing the mathematical tools needed to describe positions, orientations, transformations, and movements of robotic systems.

### 1.1 Vectors in Robotics

Vectors are used extensively in robotics to represent positions, velocities, forces, and other directional quantities.

**Position Vectors**: Represent the location of points in space relative to a reference frame.

**Example: Position Vector in 3D Space**
```python
import numpy as np

# Position of a point in 3D space [x, y, z]
position = np.array([1.0, 2.0, 3.0])
print(f"Position vector: {position}")
```

**Velocity and Force Vectors**: Represent directional quantities with magnitude.

```python
# Velocity vector [vx, vy, vz]
velocity = np.array([0.5, -1.2, 0.3])
print(f"Velocity vector: {velocity}")

# Force vector [fx, fy, fz]
force = np.array([10.0, 5.0, -2.0])
print(f"Force vector: {force}")
```

### 1.2 Matrix Operations for Transformations

Matrices are used to represent transformations such as rotations, translations, and scaling in robotic systems.

**Rotation Matrices**: 3x3 matrices that represent rotations in 3D space.

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

# Example: Rotate 45 degrees around Z-axis
angle = np.pi / 4  # 45 degrees in radians
R_z = rotation_matrix_z(angle)
print(f"Rotation matrix around Z-axis by 45°:\n{R_z}")
```

**Homogeneous Transformation Matrices**: 4x4 matrices that combine rotation and translation.

```python
def homogeneous_transform(rotation_matrix, translation_vector):
    """Create a 4x4 homogeneous transformation matrix"""
    T = np.eye(4)
    T[0:3, 0:3] = rotation_matrix
    T[0:3, 3] = translation_vector
    return T

# Example: Combine rotation and translation
translation = np.array([1.0, 2.0, 3.0])
T = homogeneous_transform(R_z, translation)
print(f"Homogeneous transformation matrix:\n{T}")
```

### 1.3 Vector Operations in Robotics

**Dot Product**: Used for calculating angles between vectors and projections.

```python
def vector_angle(v1, v2):
    """Calculate angle between two vectors in radians"""
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return np.arccos(np.clip(cos_angle, -1.0, 1.0))

# Example: Calculate angle between two vectors
v1 = np.array([1, 0, 0])
v2 = np.array([1, 1, 0])
angle = vector_angle(v1, v2)
print(f"Angle between vectors: {np.degrees(angle):.2f}°")
```

**Cross Product**: Used for calculating torques, angular velocities, and normal vectors.

```python
def cross_product(v1, v2):
    """Calculate cross product of two vectors"""
    return np.cross(v1, v2)

# Example: Calculate cross product
torque = cross_product(position, force)
print(f"Torque vector: {torque}")
```

## 2. Calculus and Differential Equations for Dynamics

Calculus is essential for understanding the dynamics of robotic systems, including motion, velocity, acceleration, and control.

### 2.1 Derivatives in Robotics

**Velocity as Derivative of Position**: v = dp/dt

```python
def numerical_derivative(positions, time_steps):
    """Calculate numerical derivative (velocity) from position data"""
    velocities = np.diff(positions) / np.diff(time_steps)
    return velocities

# Example: Calculate velocity from position data
time = np.linspace(0, 10, 100)
position_data = np.sin(time)  # Example position function
velocity_data = numerical_derivative(position_data, time)
print(f"Calculated velocity from position data")
```

**Acceleration as Derivative of Velocity**: a = dv/dt = d²p/dt²

```python
def numerical_second_derivative(positions, time_steps):
    """Calculate numerical second derivative (acceleration) from position data"""
    velocities = numerical_derivative(positions, time_steps)
    # Need to adjust time steps for velocity calculation
    time_vel = time_steps[1:]  # Adjusted time vector for velocities
    accelerations = numerical_derivative(velocities, time_vel)
    return accelerations

acceleration_data = numerical_second_derivative(position_data, time)
print(f"Calculated acceleration from position data")
```

### 2.2 Differential Equations for Dynamic Systems

Robot dynamics are often described by differential equations. The general form for a dynamic system is:

M(q)q̈ + C(q, q̇)q̇ + G(q) = τ

Where:
- M(q) is the mass matrix
- C(q, q̇) contains Coriolis and centrifugal terms
- G(q) contains gravitational terms
- τ is the vector of applied torques
- q, q̇, q̈ are joint positions, velocities, and accelerations

```python
def simple_mass_spring_damper(state, t, m, k, c, F_ext):
    """
    Simple mass-spring-damper system: m*ẍ + c*ẋ + k*x = F_ext
    state = [position, velocity]
    """
    x, v = state
    dxdt = v
    dvdt = (F_ext - c*v - k*x) / m
    return [dxdt, dvdt]

# Example: Simulate a simple dynamic system
from scipy.integrate import odeint

def simulate_mass_spring_damper():
    # System parameters
    m = 1.0  # mass
    k = 2.0  # spring constant
    c = 0.5  # damping coefficient
    F_ext = 1.0  # external force

    # Initial conditions: [position, velocity]
    state0 = [1.0, 0.0]

    # Time points
    t = np.linspace(0, 10, 100)

    # Solve ODE
    solution = odeint(lambda state, t: simple_mass_spring_damper(state, t, m, k, c, F_ext),
                      state0, t)

    positions = solution[:, 0]
    velocities = solution[:, 1]

    return t, positions, velocities

# Simulate and plot
time, pos, vel = simulate_mass_spring_damper()
print(f"Simulated mass-spring-damper system for {len(time)} time steps")
```

## 3. Probability and Statistics for Sensor Fusion

Robots operate in uncertain environments and must process noisy sensor data. Probability and statistics provide the tools for handling uncertainty and fusing information from multiple sensors.

### 3.1 Probability Concepts

**Bayes' Theorem**: Fundamental for updating beliefs based on new evidence.

P(A|B) = P(B|A) * P(A) / P(B)

```python
def bayes_update(prior, likelihood, evidence):
    """Apply Bayes' theorem to update probability"""
    posterior = (likelihood * prior) / evidence
    return posterior

# Example: Robot localization with sensor update
prior_prob = 0.3  # Prior probability of being in location A
sensor_likelihood = 0.8  # P(sensor reading | location A)
total_evidence = 0.5  # P(sensor reading)
posterior_prob = bayes_update(prior_prob, sensor_likelihood, total_evidence)
print(f"Updated probability after sensor reading: {posterior_prob:.3f}")
```

### 3.2 Gaussian Distributions

Many sensor measurements follow Gaussian (normal) distributions.

```python
def gaussian_pdf(x, mean, std_dev):
    """Calculate probability density for Gaussian distribution"""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = -0.5 * ((x - mean) / std_dev) ** 2
    return coefficient * np.exp(exponent)

# Example: Sensor measurement with uncertainty
measurement = 5.0
uncertainty = 0.5  # standard deviation
true_value = 4.8
probability = gaussian_pdf(true_value, measurement, uncertainty)
print(f"Probability of true value given measurement: {probability:.3f}")
```

### 3.3 Covariance Matrices

Represent uncertainty in multi-dimensional measurements.

```python
def create_covariance_matrix(uncertainties):
    """Create diagonal covariance matrix from individual uncertainties"""
    return np.diag(np.array(uncertainties) ** 2)

# Example: 3D position uncertainty
position_uncertainties = [0.1, 0.2, 0.15]  # [x, y, z] uncertainties
covariance = create_covariance_matrix(position_uncertainties)
print(f"Position covariance matrix:\n{covariance}")
```

## 4. PyBullet Examples for Mathematical Concepts

Let's implement some of these mathematical concepts using PyBullet simulation environment.

```python
import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_pybullet_environment():
    """Set up PyBullet for mathematical concept demonstrations"""
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    # Set gravity
    p.setGravity(0, 0, -9.81)

    # Load plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    return physicsClient

def demonstrate_transformations():
    """Demonstrate coordinate transformations using PyBullet"""
    # Set up environment
    physicsClient = setup_pybullet_environment()

    # Create a simple object (sphere)
    sphereStartPos = [0, 0, 1]
    sphereStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    sphereId = p.loadURDF("sphere2.urdf", sphereStartPos, sphereStartOrientation)

    # Apply transformations
    # Move the sphere using mathematical transformations
    new_position = [2, 1, 1.5]  # New position vector
    new_orientation = p.getQuaternionFromEuler([0, 0, np.pi/4])  # 45 degree rotation around Z

    # Update the object's position and orientation
    p.resetBasePositionAndOrientation(sphereId, new_position, new_orientation)

    # Run simulation briefly to visualize
    for i in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()

# Note: This example would run in an environment with PyBullet installed
print("PyBullet transformation example defined - requires PyBullet installation to run")
```

## 5. Mathematical Foundations Exercises

### Exercise 1: Vector Transformations
Create a function that transforms a point from one coordinate frame to another using a transformation matrix.

```python
def transform_point(point, transformation_matrix):
    """
    Transform a 3D point using a 4x4 homogeneous transformation matrix
    point: [x, y, z] - 3D point
    transformation_matrix: 4x4 matrix
    Returns: transformed 3D point [x', y', z']
    """
    # Convert point to homogeneous coordinates
    homogeneous_point = np.append(point, 1)

    # Apply transformation
    transformed_homogeneous = transformation_matrix @ homogeneous_point

    # Convert back to 3D coordinates
    transformed_point = transformed_homogeneous[:3]

    return transformed_point

# Test the function
test_point = np.array([1, 0, 0])
test_transform = np.array([
    [0, -1, 0, 2],
    [1,  0, 0, 1],
    [0,  0, 1, 0],
    [0,  0, 0, 1]
])

result = transform_point(test_point, test_transform)
print(f"Transformed point: {result}")
```

### Exercise 2: Forward Kinematics Preparation
Prepare the mathematical foundation for forward kinematics calculations.

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

# Example DH parameters for a simple link
a = 1.0      # link length
alpha = 0    # link twist
d = 0        # link offset
theta = np.pi/4  # joint angle (45 degrees)

T_dh = dh_transform(a, alpha, d, theta)
print(f"DH transformation matrix:\n{T_dh}")
```

## 6. Summary

This chapter established the mathematical foundations necessary for understanding and implementing robotic systems. We covered:

1. **Linear Algebra**: Vectors and matrices for representing positions, orientations, and transformations in robotics
2. **Calculus**: Derivatives and differential equations for modeling dynamics and motion
3. **Probability**: Statistical methods for handling uncertainty and sensor fusion
4. **Practical Implementation**: Examples using Python and preparation for PyBullet simulation

These mathematical tools form the basis for all subsequent concepts in robotics, from kinematics and dynamics to control and perception. Mastery of these concepts is essential for working with Physical AI and humanoid robotics systems.

## 7. Implementation Guide

To implement the mathematical concepts covered in this chapter:

1. Practice vector and matrix operations using NumPy
2. Implement transformation functions for coordinate frame changes
3. Work with differential equation solvers for dynamic system simulation
4. Apply probability concepts to sensor data processing
5. Use PyBullet to visualize mathematical transformations in 3D space

The exercises provided offer hands-on practice with these fundamental concepts, preparing readers for more advanced topics in robotics and Physical AI.