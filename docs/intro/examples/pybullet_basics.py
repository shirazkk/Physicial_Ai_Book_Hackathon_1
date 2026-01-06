"""
Basic PyBullet Example for Physical AI & Humanoid Robotics Textbook

This example demonstrates fundamental PyBullet concepts for simulation-first robotics development.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def setup_basic_simulation():
    """
    Set up a basic PyBullet simulation environment
    """
    # Connect to physics server
    physics_client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    # Set gravity
    p.setGravity(0, 0, -9.81)

    # Load plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    return physics_client, plane_id


def create_simple_robot():
    """
    Create a simple robot in the simulation
    This is a simplified example - real robots would use URDF files
    """
    # Create a simple robot with a base and a few links
    # For this example, we'll create a simple arm-like structure

    # Start creating multi-body
    robot_start_pos = [0, 0, 1]

    # Create base link
    base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    base_id = p.createMultiBody(
        baseMass=1,
        baseInertialFramePosition=[0, 0, 0],
        baseVisualShapeIndex=base_visual,
        baseCollisionShapeIndex=base_collision,
        basePosition=robot_start_pos
    )

    # Add a simple joint/limb
    link_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.2])
    link_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.2])

    joint_type = p.JOINT_REVOLUTE
    joint_axis = [0, 0, 1]  # Rotate around Z-axis
    parent_pos = [0, 0, 0.1]  # Position in parent frame
    child_pos = [0, 0, -0.1]  # Position in child frame

    p.createMultiBody(
        baseMass=0.5,
        baseInertialFramePosition=[0, 0, 0],
        baseVisualShapeIndex=link_visual,
        baseCollisionShapeIndex=link_collision,
        basePosition=[0, 0, 0.8]  # Position of the second body
    )

    # Create joint between base and link
    p.createConstraint(
        parentBodyUniqueId=base_id,
        parentLinkIndex=-1,
        childBodyUniqueId=2,  # The second body we just created
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 1],
        parentFramePosition=parent_pos,
        childFramePosition=child_pos
    )

    return base_id


def run_simulation_demo():
    """
    Run a simple simulation demonstrating basic physics
    """
    print("Setting up simulation...")
    physics_client, plane_id = setup_basic_simulation()

    print("Creating robot...")
    robot_id = create_simple_robot()

    # Set up some objects to interact with
    # Create a small box to move around
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    box_id = p.createMultiBody(
        baseMass=0.1,
        baseInertialFramePosition=[0, 0, 0],
        baseVisualShapeIndex=box_visual,
        baseCollisionShapeIndex=box_collision,
        basePosition=[0.5, 0, 0.1]
    )

    print("Simulation running. Press Ctrl+C to exit.")

    # Run simulation for a while
    for i in range(300):  # Run for 300 steps
        p.stepSimulation()

        # Occasionally apply a small force to the box
        if i % 50 == 0:
            p.applyExternalForce(
                objectUniqueId=box_id,
                linkIndex=-1,
                forceObj=[10, 0, 0],  # Apply force in x direction
                posObj=[0.5, 0, 0.1],  # Position where force is applied
                flags=p.WORLD_FRAME
            )

        time.sleep(1./240.)  # Slow down simulation to real-time

    # Disconnect from physics server
    p.disconnect()
    print("Simulation completed.")


def demonstrate_inverse_kinematics():
    """
    Demonstrate inverse kinematics - a key concept in robotics
    """
    print("Setting up IK demonstration...")

    # Connect to physics server
    physics_client = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load a simple robot (using KUKA iiwa as example)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

    # Get the number of joints
    num_joints = p.getNumJoints(robot_id)
    print(f"Robot has {num_joints} joints")

    # Define a target position for the end effector
    target_pos = [0.5, 0.2, 0.3]

    # Calculate inverse kinematics
    joint_angles = p.calculateInverseKinematics(
        robot_id,
        endEffectorLinkIndex=num_joints-1,  # Last link is usually the end effector
        targetPosition=target_pos
    )

    print(f"Target position: {target_pos}")
    print(f"Required joint angles: {joint_angles[:7]}")  # First 7 joints for this robot

    # Apply the joint angles
    for i, angle in enumerate(joint_angles):
        if i < num_joints:  # Only set angles for existing joints
            p.resetJointState(robot_id, i, angle)

    # Run simulation to visualize
    for i in range(300):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()
    print("IK demonstration completed.")


def domain_randomization_example():
    """
    Example of domain randomization - a key technique for sim-to-real transfer
    """
    print("Demonstrating domain randomization...")

    # Different physics parameters to randomize
    gravity_options = [-9.5, -9.81, -10.0]  # Different gravity values
    friction_options = [0.5, 0.7, 1.0, 1.5]  # Different friction values
    mass_variance = [0.8, 1.0, 1.2]  # Mass multipliers

    # For demonstration, we'll just show the concept:
    print("Domain randomization parameters:")
    print(f"Gravity range: {min(gravity_options)} to {max(gravity_options)} m/s^2")
    print(f"Friction range: {min(friction_options)} to {max(friction_options)}")
    print(f"Mass variance: {min(mass_variance)}x to {max(mass_variance)}x")

    # In a real implementation, you would:
    # 1. Randomly select parameters from these ranges
    # 2. Apply them to the simulation
    # 3. Train your robot controller
    # 4. Repeat with different random parameters

    print("This technique helps robots learn to be robust to parameter variations")


if __name__ == "__main__":
    print("PyBullet Simulation Examples for Physical AI & Humanoid Robotics")
    print("=" * 65)

    print("\n1. Basic Simulation Demo:")
    run_simulation_demo()

    print("\n2. Domain Randomization Example:")
    domain_randomization_example()

    print("\n3. Inverse Kinematics Demo (uncomment to run):")
    # demonstrate_inverse_kinematics()  # Uncomment to run this demo

    print("\nExamples completed. These demonstrate core concepts for sim-to-real transfer.")