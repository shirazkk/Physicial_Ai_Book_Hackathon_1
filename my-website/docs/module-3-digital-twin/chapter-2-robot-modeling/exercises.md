# URDF and SDF Robot Description Formats - Exercises

## Exercise 2.1: URDF Fundamentals
**Objective**: Create a simple robot model with basic links and joints.

1. Create a URDF file for a simple wheeled robot with:
   - One base link (box shape)
   - Two wheel links (cylinder shapes)
   - Two continuous joints connecting wheels to base
2. Validate your URDF using the `check_urdf` tool
3. Visualize the robot using RViz or Gazebo
4. Document any issues encountered during validation

**Difficulty**: Beginner

## Exercise 2.2: Xacro Macros Implementation
**Objective**: Implement a complex robot using Xacro macros to reduce redundancy.

1. Take the simple robot from Exercise 2.1 and convert it to use Xacro
2. Create a macro for wheels that can be reused for multiple wheels
3. Add parameters for wheel radius, width, and position
4. Create a second robot model using the same wheel macro but with different parameters
5. Compare the code complexity between the original and Xacro versions

**Difficulty**: Intermediate

## Exercise 2.3: Humanoid Robot Modeling
**Objective**: Create a complete humanoid robot model with proper kinematic chain.

1. Extend the example humanoid model from the chapter with:
   - Additional joints for shoulder yaw and roll
   - Wrist joints for more dexterity
   - Feet with ankle joints
2. Ensure proper joint limits that reflect human-like movement ranges
3. Calculate and verify the total degrees of freedom
4. Validate the model and test its kinematic structure

**Difficulty**: Advanced

## Exercise 2.4: Inertial Properties Estimation
**Objective**: Calculate realistic inertial properties for robot links.

1. Choose 3 links from your humanoid robot (torso, upper arm, thigh)
2. Estimate their mass based on approximate dimensions and material density (assume aluminum: 2700 kg/m³)
3. Calculate their inertia tensors using standard geometric formulas
4. Validate that your inertia values are physically plausible
5. Test the robot in Gazebo simulation to observe the effect of inertial properties

**Difficulty**: Intermediate

## Exercise 2.5: URDF to SDF Conversion and Enhancement
**Objective**: Convert a URDF model to SDF and add simulation-specific features.

1. Take your humanoid robot URDF and convert it to SDF format
2. Add at least 3 different sensor definitions (IMU, camera, LiDAR)
3. Include realistic noise models for the sensors
4. Add physics properties like friction and contact parameters
5. Compare the simulation behavior between the original URDF and enhanced SDF

**Difficulty**: Advanced

## Exercise 2.6: Validation and Debugging
**Objective**: Validate and debug common URDF issues.

1. Create a URDF file with intentional errors (incorrect joint limits, invalid inertia values, etc.)
2. Use validation tools to identify the issues
3. Correct the errors systematically
4. Document the common error types and how to fix them
5. Create a validation checklist for future robot models

**Difficulty**: Intermediate

## Exercise 2.7: Multi-Link Chain Optimization
**Objective**: Optimize a complex robot chain for simulation performance.

1. Create a robot with at least 10 links and joints
2. Profile the simulation performance in Gazebo
3. Optimize the model by:
   - Simplifying collision geometries
   - Adjusting physics parameters
   - Reducing unnecessary visual details
4. Measure the performance improvement
5. Document the trade-offs between accuracy and performance

**Difficulty**: Advanced

---

## Solutions Reference
Solutions to these exercises can be found in [Chapter 2 Solutions](./solutions.md).