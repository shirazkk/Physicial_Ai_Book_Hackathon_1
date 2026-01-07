# Chapter 2: Kinematics and Dynamics for Physical AI & Robotics

## Learning Objectives
By the end of this chapter, readers will be able to:
- Calculate forward kinematics for robotic manipulators using transformation matrices
- Solve inverse kinematics problems for simple robotic systems
- Model and analyze dynamic behavior of robotic systems
- Apply Newton-Euler and Lagrangian methods for dynamic analysis
- Implement kinematic and dynamic models using PyBullet simulation
- Predict and control robot motion based on kinematic and dynamic principles

## Prerequisites
- Understanding of linear algebra concepts (covered in Chapter 1)
- Basic calculus knowledge (covered in Chapter 1)
- Fundamental physics concepts (forces, torques, motion)
- Basic Python programming skills

## Introduction

Kinematics and dynamics form the core of robot motion understanding and control. Kinematics deals with the geometry of motion without considering forces, while dynamics analyzes motion considering the forces and torques that cause it. This chapter covers both forward and inverse kinematics, as well as dynamic modeling and analysis of robotic systems.

## 1. Forward Kinematics

Forward kinematics determines the position and orientation of the end-effector given the joint angles of a robotic manipulator.

### 1.1 Denavit-Hartenberg (DH) Convention

The DH convention is a systematic method for assigning coordinate frames to robotic links.

**DH Parameters**:
- aᵢ: link length (distance along xᵢ from zᵢ to zᵢ₊₁)
- αᵢ: link twist (angle from zᵢ to zᵢ₊₁ about xᵢ)
- dᵢ: link offset (distance along zᵢ from xᵢ₋₁ to xᵢ)
- θᵢ: joint angle (angle from xᵢ₋₁ to xᵢ about zᵢ)

```python
import numpy as np

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

# Example: 2-DOF planar manipulator
def forward_kinematics_2dof(theta1, theta2, l1, l2):
    """
    Forward kinematics for 2-DOF planar manipulator
    theta1, theta2: joint angles
    l1, l2: link lengths
    Returns: end-effector position [x, y]
    """
    # Calculate transformation matrices
    T1 = dh_transform(l1, 0, 0, theta1)
    T2 = dh_transform(l2, 0, 0, theta2)

    # Total transformation from base to end-effector
    T_total = T1 @ T2

    # Extract end-effector position
    x = T_total[0, 3]
    y = T_total[1, 3]
    z = T_total[2, 3]

    return np.array([x, y, z])

# Example calculation
theta1 = np.pi/4  # 45 degrees
theta2 = np.pi/6  # 30 degrees
l1 = 1.0
l2 = 0.8

end_effector_pos = forward_kinematics_2dof(theta1, theta2, l1, l2)
print(f"End-effector position: {end_effector_pos}")
```

### 1.2 Homogeneous Transformation Matrices

Using homogeneous transformation matrices to represent position and orientation.

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

# Example: Create transformation for a simple case
angle = np.pi / 3  # 60 degrees
R = rotation_matrix_z(angle)
t = np.array([2.0, 1.0, 0.5])
T = homogeneous_transform(R, t)
print(f"Homogeneous transformation matrix:\n{T}")
```

## 2. Inverse Kinematics

Inverse kinematics determines the joint angles required to achieve a desired end-effector position and orientation.

### 2.1 Analytical Solutions

For simple manipulators, analytical solutions may be possible.

```python
def inverse_kinematics_2dof(x, y, l1, l2):
    """
    Inverse kinematics for 2-DOF planar manipulator
    x, y: desired end-effector position
    l1, l2: link lengths
    Returns: joint angles [theta1, theta2] or None if no solution
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
angles = inverse_kinematics_2dof(desired_pos[0], desired_pos[1], l1, l2)
if angles is not None:
    print(f"Required joint angles: {np.degrees(angles)} degrees")

    # Verify with forward kinematics
    verify_pos = forward_kinematics_2dof(angles[0], angles[1], l1, l2)
    print(f"Verification - Forward kinematics result: {verify_pos[:2]}")
    print(f"Desired position: {desired_pos}")
```

### 2.2 Numerical Solutions

For complex manipulators, numerical methods are often necessary.

```python
def jacobian_2dof(theta1, theta2, l1, l2):
    """
    Jacobian matrix for 2-DOF planar manipulator
    The Jacobian relates joint velocities to end-effector velocities
    """
    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1 + theta2), -l2*np.sin(theta1 + theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1 + theta2), l2*np.cos(theta1 + theta2)]
    ])
    return J

def numerical_inverse_kinematics(desired_pos, initial_angles, l1, l2, max_iterations=100, tolerance=1e-6):
    """
    Numerical inverse kinematics using Jacobian transpose method
    """
    angles = initial_angles.copy()

    for i in range(max_iterations):
        # Calculate current position
        current_pos = forward_kinematics_2dof(angles[0], angles[1], l1, l2)[:2]

        # Calculate error
        error = desired_pos - current_pos

        # Check if we're close enough
        if np.linalg.norm(error) < tolerance:
            print(f"Converged after {i+1} iterations")
            return angles

        # Calculate Jacobian
        J = jacobian_2dof(angles[0], angles[1], l1, l2)

        # Update angles using Jacobian transpose
        angles += 0.01 * J.T @ error  # Small step size for stability

    print(f"Did not converge after {max_iterations} iterations")
    return angles

# Example: Numerical inverse kinematics
initial_guess = np.array([0.5, 0.5])  # Initial guess for joint angles
numerical_angles = numerical_inverse_kinematics(desired_pos, initial_guess, l1, l2)
print(f"Numerical solution - Joint angles: {np.degrees(numerical_angles)} degrees")
```

## 3. Robot Dynamics

Robot dynamics deals with the forces and torques that cause motion in robotic systems.

### 3.1 Newton-Euler Formulation

The Newton-Euler method applies Newton's laws to each link of the manipulator.

**Newton's equation**: F = ma (for translational motion)
**Euler's equation**: τ = Iα (for rotational motion)

```python
def newton_euler_1dof(theta, theta_dot, theta_ddot, mass, length, gravity=9.81):
    """
    Simple dynamics for 1-DOF pendulum using Newton-Euler formulation
    """
    # For a simple pendulum: τ = I*θ_ddot + m*g*l*sin(θ)
    # Moment of inertia for point mass at end of rod: I = m*l²
    inertia = mass * length**2

    # External torque needed to achieve desired acceleration
    required_torque = inertia * theta_ddot + mass * gravity * length * np.sin(theta)

    return required_torque

# Example: Calculate torque for 1-DOF pendulum
mass = 1.0  # kg
length = 1.0  # m
theta = np.pi/6  # 30 degrees
theta_dot = 0.5  # rad/s
theta_ddot = 0.2  # rad/s²

torque = newton_euler_1dof(theta, theta_dot, theta_ddot, mass, length)
print(f"Required torque: {torque:.3f} N·m")
```

### 3.2 Lagrangian Formulation

The Lagrangian method uses energy principles to derive equations of motion.

**Lagrangian**: L = T - V (Kinetic Energy - Potential Energy)
**Euler-Lagrange Equation**: d/dt(∂L/∂q̇) - ∂L/∂q = τ

```python
def lagrangian_1dof_pendulum(theta, theta_dot, mass, length, gravity=9.81):
    """
    Calculate Lagrangian for 1-DOF pendulum
    """
    # Kinetic energy: T = 1/2 * I * θ_dot²
    # For pendulum: I = m*l²
    kinetic_energy = 0.5 * mass * length**2 * theta_dot**2

    # Potential energy: V = m*g*h
    # For pendulum: h = l*(1 - cos(θ)) (with reference at bottom)
    potential_energy = mass * gravity * length * (1 - np.cos(theta))

    # Lagrangian
    lagrangian = kinetic_energy - potential_energy

    return lagrangian, kinetic_energy, potential_energy

def equations_of_motion_1dof_pendulum(theta, theta_dot, mass, length, gravity=9.81):
    """
    Derive equations of motion for 1-DOF pendulum using Lagrangian method
    Returns: θ_ddot (angular acceleration)
    """
    # The equation of motion for a simple pendulum is:
    # θ_ddot = -(g/l)*sin(θ)
    theta_ddot = -(gravity / length) * np.sin(theta)

    return theta_ddot

# Example: Calculate Lagrangian and equation of motion
lag, T, V = lagrangian_1dof_pendulum(theta, theta_dot, mass, length)
theta_ddot_calc = equations_of_motion_1dof_pendulum(theta, theta_dot, mass, length)

print(f"Lagrangian: {lag:.3f} J")
print(f"Kinetic Energy: {T:.3f} J")
print(f"Potential Energy: {V:.3f} J")
print(f"Angular acceleration: {theta_ddot_calc:.3f} rad/s²")
```

### 3.3 Dynamic Model for Multi-DOF Manipulator

For multi-DOF manipulators, the dynamic equation is:

M(q)q̈ + C(q, q̇)q̇ + G(q) = τ

Where:
- M(q) is the mass matrix
- C(q, q̇) contains Coriolis and centrifugal terms
- G(q) contains gravitational terms
- τ is the vector of applied torques

```python
def mass_matrix_2dof(theta1, theta2, m1, m2, l1, l2):
    """
    Mass matrix for 2-DOF planar manipulator
    """
    # Simplified mass matrix calculation
    # In a real implementation, this would involve more complex calculations
    # based on the specific manipulator geometry and mass distribution

    # Calculate intermediate values
    c2 = np.cos(theta2)

    # Mass matrix elements (simplified for demonstration)
    M11 = m1*l1**2 + m2*(l1**2 + l2**2 + 2*l1*l2*c2)
    M12 = m2*(l2**2 + l1*l2*c2)
    M21 = M12
    M22 = m2*l2**2

    M = np.array([[M11, M12],
                  [M21, M22]])

    return M

def coriolis_matrix_2dof(theta1, theta2, theta1_dot, theta2_dot, m2, l1, l2):
    """
    Coriolis matrix for 2-DOF planar manipulator
    """
    # Simplified Coriolis matrix calculation
    s2 = np.sin(theta2)

    C11 = -2*m2*l1*l2*theta2_dot*s2
    C12 = -m2*l1*l2*theta2_dot*s2
    C21 = m2*l1*l2*theta1_dot*s2
    C22 = 0

    C = np.array([[C11, C12],
                  [C21, C22]])

    return C

def gravity_vector_2dof(theta1, theta2, m1, m2, l1, l2, g=9.81):
    """
    Gravity vector for 2-DOF planar manipulator
    """
    g1 = (m1 + m2)*g*l1*np.cos(theta1) + m2*g*l2*np.cos(theta1 + theta2)
    g2 = m2*g*l2*np.cos(theta1 + theta2)

    return np.array([g1, g2])

# Example: Calculate dynamic components
m1, m2 = 1.0, 0.8  # masses
theta1, theta2 = np.pi/4, np.pi/6
theta1_dot, theta2_dot = 0.5, 0.3

M = mass_matrix_2dof(theta1, theta2, m1, m2, l1, l2)
C = coriolis_matrix_2dof(theta1, theta2, theta1_dot, theta2_dot, m2, l1, l2)
G = gravity_vector_2dof(theta1, theta2, m1, m2, l1, l2)

print(f"Mass matrix:\n{M}")
print(f"Coriolis matrix:\n{C}")
print(f"Gravity vector: {G}")
```

## 4. Control Theory Fundamentals

Control theory provides the mathematical framework for controlling robotic motion.

### 4.1 PID Control

Proportional-Integral-Derivative control is widely used in robotics.

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # Store current error for next iteration
        self.prev_error = error

        # Calculate output
        output = p_term + i_term + d_term
        return output

# Example: PID control for position tracking
pid = PIDController(kp=10.0, ki=0.5, kd=2.0, dt=0.01)

# Simulate a simple control scenario
target_position = 1.0
current_position = 0.0
dt = 0.01

for t in np.arange(0, 2, dt):  # 2 seconds of simulation
    error = target_position - current_position

    # Apply control (simplified - in reality this would affect system dynamics)
    control_effort = pid.update(error)

    # Update position based on control effort (simplified model)
    current_position += control_effort * dt

    if t % 0.5 < dt:  # Print every 0.5 seconds
        print(f"Time: {t:.2f}s, Position: {current_position:.3f}, Error: {error:.3f}")

print(f"Final position: {current_position:.3f}, Target: {target_position}")
```

## 5. PyBullet Examples for Kinematics and Dynamics

Let's implement some kinematic and dynamic examples using PyBullet.

```python
import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_kinematics_demo():
    """Set up PyBullet environment for kinematics demonstration"""
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)

    # Set gravity
    p.setGravity(0, 0, -9.81)

    # Load plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    return physicsClient

def demo_inverse_kinematics():
    """Demonstrate inverse kinematics using PyBullet's built-in IK solver"""
    # Set up environment
    physicsClient = setup_kinematics_demo()

    # Load a simple robot (KUKA LBR iiwa)
    robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

    # Define target position for end-effector
    target_position = [0.5, 0.2, 0.3]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Solve inverse kinematics
    joint_angles = p.calculateInverseKinematics(
        robotId,
        endEffectorLinkIndex=6,  # iiwa has 7 joints, index 6 is the end effector
        targetPosition=target_position,
        targetOrientation=target_orientation
    )

    # Apply the joint angles to the robot
    for i in range(len(joint_angles)):
        p.resetJointState(robotId, i, joint_angles[i])

    # Run simulation briefly to visualize
    for i in range(300):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()

# Note: This example would run in an environment with PyBullet installed
print("PyBullet inverse kinematics demo defined - requires PyBullet installation to run")
```

## 6. Kinematics and Dynamics Exercises

### Exercise 1: Forward Kinematics Implementation
Implement forward kinematics for a 3-DOF planar manipulator.

```python
def forward_kinematics_3dof(theta1, theta2, theta3, l1, l2, l3):
    """
    Forward kinematics for 3-DOF planar manipulator
    Calculate end-effector position given joint angles
    """
    # Calculate end-effector position using trigonometry
    x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2) + l3*np.cos(theta1 + theta2 + theta3)
    y = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2) + l3*np.sin(theta1 + theta2 + theta3)

    return np.array([x, y])

# Test the function
l1, l2, l3 = 1.0, 0.8, 0.6
theta1, theta2, theta3 = np.pi/4, np.pi/6, np.pi/8

end_pos = forward_kinematics_3dof(theta1, theta2, theta3, l1, l2, l3)
print(f"3-DOF end-effector position: {end_pos}")
```

### Exercise 2: Simple Dynamic Simulation
Create a simple dynamic simulation for a single joint.

```python
def simulate_single_joint_dynamics(torque, initial_position, initial_velocity,
                                  mass, length, gravity=9.81, dt=0.01, duration=2.0):
    """
    Simulate the dynamics of a single joint pendulum
    """
    # Moment of inertia for point mass at end of rod
    inertia = mass * length**2

    # Initialize state
    theta = initial_position
    theta_dot = initial_velocity

    # Time vector
    time_steps = np.arange(0, duration, dt)
    positions = []
    velocities = []

    for t in time_steps:
        # Calculate angular acceleration using equation of motion
        # For pendulum: I*θ_ddot = τ - m*g*l*sin(θ)
        theta_ddot = (torque - mass * gravity * length * np.sin(theta)) / inertia

        # Update state using Euler integration
        theta_dot += theta_ddot * dt
        theta += theta_dot * dt

        # Store values
        positions.append(theta)
        velocities.append(theta_dot)

    return time_steps, np.array(positions), np.array(velocities)

# Example simulation
time_vals, pos_vals, vel_vals = simulate_single_joint_dynamics(
    torque=0.5,  # N·m
    initial_position=np.pi/6,  # 30 degrees
    initial_velocity=0.1,  # rad/s
    mass=1.0,
    length=1.0,
    dt=0.01,
    duration=5.0
)

print(f"Simulated {len(time_vals)} time steps")
print(f"Final position: {pos_vals[-1]:.3f} rad, Final velocity: {vel_vals[-1]:.3f} rad/s")
```

## 7. Summary

This chapter covered the fundamental concepts of kinematics and dynamics for robotic systems:

1. **Forward Kinematics**: Determining end-effector position from joint angles using DH parameters and transformation matrices
2. **Inverse Kinematics**: Finding joint angles for desired end-effector positions using analytical and numerical methods
3. **Robot Dynamics**: Understanding forces and torques that cause motion using Newton-Euler and Lagrangian formulations
4. **Control Theory**: Basic control methods like PID for managing robot motion
5. **Practical Implementation**: Using PyBullet for kinematic and dynamic simulation

These concepts are essential for understanding how robots move and how to control their motion effectively. The mathematical models developed in this chapter form the foundation for more advanced topics in robotics and Physical AI.

## 8. Implementation Guide

To implement the kinematic and dynamic concepts covered in this chapter:

1. Practice deriving DH parameters for different manipulator configurations
2. Implement forward and inverse kinematics solvers for simple manipulators
3. Work with dynamic models to simulate robot motion
4. Apply control theory concepts to stabilize robot movements
5. Use PyBullet to visualize and validate kinematic and dynamic calculations
6. Experiment with different control parameters to understand their effects

The exercises provided offer hands-on practice with these fundamental concepts, preparing readers for more advanced topics in robotics and Physical AI.