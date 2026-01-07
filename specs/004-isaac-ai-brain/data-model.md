# Data Model: Module 3 - AI-Robot Brain with NVIDIA Isaac Ecosystem

## Overview
This document defines the key data structures and entities for Module 3, focusing on the educational content and simulation components that will be taught to students learning the NVIDIA Isaac ecosystem. This model addresses all functional requirements (FR-001 through FR-013) specified in the feature specification.

## Core Entities

### 1. IsaacSimEnvironment
- **Description**: Virtual simulation platform for creating photorealistic robotic environments (addresses FR-001)
- **Attributes**:
  - id: String (unique identifier)
  - name: String (environment name)
  - description: String (environment description)
  - usdPath: String (path to USD asset file)
  - physicsEngine: String (physics engine type)
  - lightingConfig: Object (lighting and material properties)
  - sensorConfigs: Array[SensorConfig] (sensor configurations)
  - humanoidConfig: Object (humanoid-specific configuration for FR-011)
- **Relationships**:
  - Contains multiple RobotAssets
  - Contains multiple SceneObjects
  - Associated with multiple SyntheticDatasets
  - Used in EducationalExercises (addresses FR-004, FR-009)

### 2. RobotAsset
- **Description**: Robot model within the Isaac Sim environment (addresses FR-011 for humanoid configs)
- **Attributes**:
  - id: String (unique identifier)
  - name: String (robot name)
  - urdfPath: String (path to URDF file)
  - kinematicChain: Array[String] (joint hierarchy)
  - sensorMounts: Array[SensorMount] (sensor mounting points)
  - physicalProperties: Object (mass, dimensions, etc.)
  - humanoidProperties: Object (complex kinematics for FR-011)
  - robotType: String (humanoid, wheeled, etc.)
- **Relationships**:
  - Belongs to one IsaacSimEnvironment
  - Generates multiple SyntheticDatasets
  - Used in multiple IsaacROSPipelines

### 3. SensorConfig
- **Description**: Configuration for sensors in the simulation (addresses FR-001)
- **Attributes**:
  - id: String (unique identifier)
  - sensorType: String (camera, lidar, IMU, etc.)
  - position: Vector3 (position relative to robot)
  - orientation: Quaternion (orientation relative to robot)
  - parameters: Object (sensor-specific parameters)
- **Relationships**:
  - Belongs to one IsaacSimEnvironment
  - Used by multiple RobotAssets

### 4. SyntheticDataset
- **Description**: Dataset generated through synthetic data generation in Isaac Sim with domain randomization (addresses FR-002)
- **Attributes**:
  - id: String (unique identifier)
  - name: String (dataset name)
  - description: String (dataset description)
  - outputPath: String (path to generated dataset)
  - dataType: String (images, point clouds, sensor data, etc.)
  - sampleCount: Integer (number of samples generated)
  - domainRandomizationApplied: Boolean (whether randomization was applied for FR-002)
  - photorealisticQuality: Float (quality metric for FR-002)
  - labels: Object (ground truth labels for training)
  - simToRealGapMetrics: Object (metrics for FR-012)
- **Relationships**:
  - Generated from one IsaacSimEnvironment
  - Generated using one RobotAsset
  - Consumed by IsaacROSPipelines for training

### 5. IsaacROSPipeline
- **Description**: GPU-accelerated perception pipeline using Isaac ROS for VSLAM, object detection, and tracking (addresses FR-003)
- **Attributes**:
  - id: String (unique identifier)
  - name: String (pipeline name)
  - description: String (pipeline description)
  - nodeGraph: Object (graph of connected ROS nodes)
  - gpuRequirements: Object (GPU memory and compute requirements for FR-007)
  - cpuRequirements: Object (CPU requirements for FR-008 comparison)
  - inputTopics: Array[String] (input ROS topics)
  - outputTopics: Array[String] (output ROS topics)
  - performanceMetrics: Object (real-time performance data)
  - perceptionModules: Array[String] (VSLAM, object detection, tracking for FR-003)
  - cudaEnabled: Boolean (indicates GPU acceleration for FR-007)
- **Relationships**:
  - Processes SyntheticDatasets
  - Connects to Nav2Planner (addresses FR-013)
  - Uses RobotAsset configurations

### 6. Nav2Planner
- **Description**: Navigation planner using Nav2 for path planning and humanoid movement with Isaac ROS integration (addresses FR-013)
- **Attributes**:
  - id: String (unique identifier)
  - name: String (planner name)
  - description: String (planner description)
  - globalPlanner: String (global path planning algorithm)
  - localPlanner: String (local path planning algorithm)
  - controller: String (trajectory controller type)
  - behaviorTree: Object (behavior tree configuration)
  - humanoidSpecificParams: Object (parameters for humanoid robots for FR-011)
  - perceptionInputSource: String (source of perception data from Isaac ROS for FR-013)
- **Relationships**:
  - Receives input from IsaacROSPipeline (addresses FR-013)
  - Controls RobotAsset movement
  - Operates within IsaacSimEnvironment

### 7. EducationalExercise
- **Description**: Educational exercise for students learning Isaac ecosystem with practical examples (addresses FR-004, FR-009, FR-010)
- **Attributes**:
  - id: String (unique identifier)
  - title: String (exercise title)
  - description: String (exercise description)
  - difficulty: Enum (beginner, intermediate, advanced for FR-010)
  - duration: Integer (estimated completion time in minutes)
  - objectives: Array[String] (learning objectives)
  - prerequisites: Array[String] (required knowledge from Modules 1&2 for FR-010)
  - environmentRequirements: Object (Isaac ecosystem version, hardware, etc.)
  - steps: Array[ExerciseStep] (sequence of steps)
  - expectedOutcome: String (what students should achieve)
  - isaacSimExamples: Array[String] (practical examples for FR-004, FR-009)
  - certificationLinks: Array[String] (links for FR-005)
  - gpuAlgorithmExamples: Array[String] (code examples for FR-007)
  - cpuVsGpuContent: Object (optional advanced content for FR-008)
  - simulationRealityTransfer: Object (techniques for FR-012)
- **Relationships**:
  - Associated with IsaacSimEnvironment
  - May use IsaacROSPipeline
  - May use Nav2Planner

### 8. ExerciseStep
- **Description**: Individual step within an educational exercise
- **Attributes**:
  - id: String (unique identifier)
  - stepNumber: Integer (order of execution)
  - title: String (step title)
  - description: String (detailed step instructions)
  - codeExample: String (sample code for the step for FR-007)
  - validationCriteria: String (how to verify success)
  - isaacPackageUsed: String (specific Isaac ROS package for FR-007)
- **Relationships**:
  - Belongs to one EducationalExercise

### 9. SimulationRealityTransferGuide
- **Description**: Guide for transferring AI models from simulation to reality (addresses FR-012)
- **Attributes**:
  - id: String (unique identifier)
  - title: String (guide title)
  - domainRandomizationTechniques: Array[String] (techniques for FR-012)
  - systemIdentificationMethods: Array[String] (methods for FR-012)
  - validationStrategies: Array[String] (validation for FR-012)
  - realWorldDeploymentGuidance: Object (deployment guidance for FR-012)
- **Relationships**:
  - Associated with multiple EducationalExercises

## Relationships Summary

- IsaacSimEnvironment "contains" multiple RobotAssets and SceneObjects
- RobotAsset "uses" multiple SensorConfigs
- IsaacSimEnvironment "generates" multiple SyntheticDatasets (with domain randomization for FR-002)
- IsaacROSPipeline "consumes" SyntheticDatasets
- IsaacROSPipeline "connects to" Nav2Planner (for FR-013)
- EducationalExercise "utilizes" IsaacSimEnvironment, IsaacROSPipeline, and Nav2Planner
- EducationalExercise "contains" multiple ExerciseSteps
- EducationalExercise "includes" SimulationRealityTransferGuide (for FR-012)

## Validation Rules

1. **IsaacSimEnvironment** must have valid USD file path (addresses FR-001)
2. **RobotAsset** must have valid URDF file path and proper kinematic chain (addresses FR-011)
3. **SyntheticDataset** must have consistent labeling with ground truth (addresses FR-002)
4. **IsaacROSPipeline** must meet real-time performance requirements (addresses FR-003)
5. **EducationalExercise** must have appropriate difficulty level for target audience (addresses FR-010)
6. **SensorConfig** must be compatible with the target RobotAsset mounting points (addresses FR-001)
7. **SyntheticDataset** must have domainRandomizationApplied flag set appropriately (addresses FR-002)
8. **IsaacROSPipeline** must specify GPU requirements (addresses FR-007)
9. **Nav2Planner** must reference IsaacROS perception data source (addresses FR-013)
10. **RobotAsset** must have humanoidProperties for humanoid robots (addresses FR-011)