# Gazebo Simulation Environment Setup - Exercises

## Exercise 1.1: Gazebo Installation and Verification
**Objective**: Install Gazebo Garden and verify the installation works correctly.

1. Follow the installation instructions to install Gazebo Garden on your system
2. Verify the installation by running `gz --version`
3. Launch the Gazebo GUI using `gz sim`
4. Take a screenshot of the Gazebo interface and describe the main components you observe

**Difficulty**: Beginner

## Exercise 1.2: Basic World Creation
**Objective**: Create a custom world file with modified physics parameters.

1. Create a new world file called `custom_world.sdf` based on the example in the chapter
2. Modify the gravity parameter to simulate a moon-like environment (1/6th of Earth's gravity)
3. Change the real_time_factor to 0.5 to run the simulation at half speed
4. Test your world file by launching Gazebo with your custom world: `gz sim custom_world.sdf`
5. Document the changes you observe compared to the default world

**Difficulty**: Intermediate

## Exercise 1.3: Robot Spawning Practice
**Objective**: Practice spawning a robot model programmatically and via command line.

1. Create a simple SDF robot model (similar to the simple_box_robot example)
2. Spawn the robot using the command line: `gz model -f your_robot.sdf -m robot_name`
3. Create a Python script that uses the ROS 2 SpawnEntity service to spawn the same robot
4. Compare the advantages and disadvantages of each approach

**Difficulty**: Intermediate

## Exercise 1.4: World Customization Challenge
**Objective**: Design and implement a custom testing environment.

1. Create a world file that includes:
   - Modified physics parameters suitable for humanoid robot testing
   - At least 3 different objects/models (e.g., ground plane, ramp, obstacle)
   - Custom lighting configuration
   - A simple obstacle course for navigation testing
2. Test your world in Gazebo
3. Document any challenges you faced during implementation

**Difficulty**: Advanced

## Exercise 1.5: Physics Validation Experiment
**Objective**: Validate the physics simulation by conducting controlled experiments.

1. Create a simple experiment to measure gravitational acceleration in your simulation:
   - Drop an object from a known height
   - Measure the time it takes to reach the ground
   - Calculate the effective gravitational acceleration
2. Compare your calculated value with the expected value (9.8 m/s²)
3. Discuss any discrepancies and possible reasons for them

**Difficulty**: Advanced

## Exercise 1.6: Performance Optimization
**Objective**: Analyze and optimize simulation performance.

1. Create a complex world with multiple objects and robots
2. Monitor simulation performance using Gazebo's statistics
3. Identify potential performance bottlenecks
4. Apply optimization techniques (reduce polygon count, adjust physics parameters)
5. Measure and compare performance before and after optimization

**Difficulty**: Advanced

---

## Solutions Reference
Solutions to these exercises can be found in [Chapter 1 Solutions](./solutions.md).