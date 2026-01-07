# Data Model for Module 2: Robotic Nervous System with ROS 2

## Overview

This document defines the key data structures and concepts that will be covered in Module 2 of the Physical AI & Humanoid Robotics textbook, focusing on the robotic nervous system using ROS 2 as the core middleware.

## Core ROS 2 Entities

### 1. Node
**Definition**: An independent process that communicates with other nodes using topics, services, and actions.

**Attributes**:
- `node_name`: String identifier for the node
- `namespace`: String for organizing nodes hierarchically
- `parameters`: Dictionary of configurable values
- `publishers`: List of topic publishers
- `subscribers`: List of topic subscribers
- `services`: List of service servers/clients
- `actions`: List of action servers/clients

**Relationships**:
- Nodes communicate with other nodes through topics, services, and actions
- Nodes can be organized hierarchically using namespaces

### 2. Topic
**Definition**: Asynchronous communication channel for publishing/subscribing to data streams.

**Attributes**:
- `topic_name`: String identifier for the topic
- `message_type`: Type definition for messages
- `qos_profile`: Quality of Service settings
- `publishers_count`: Number of publishers
- `subscribers_count`: Number of subscribers

**Relationships**:
- One publisher can have many subscribers (one-to-many)
- Topics are independent of nodes but connect them

### 3. Service
**Definition**: Synchronous request/response communication pattern for specific operations.

**Attributes**:
- `service_name`: String identifier for the service
- `service_type`: Type definition for request/response
- `server`: The service server node
- `clients`: List of service client nodes

**Relationships**:
- One server can serve multiple clients
- Service communication is synchronous

### 4. Action
**Definition**: Asynchronous request/response pattern with feedback for long-running operations.

**Attributes**:
- `action_name`: String identifier for the action
- `action_type`: Type definition for goal/feedback/result
- `server`: The action server node
- `clients`: List of action client nodes
- `goals`: List of active goals

**Relationships**:
- One server can handle multiple clients
- Action communication includes goal, feedback, and result phases

### 5. Message
**Definition**: Data structure used for communication between nodes.

**Attributes**:
- `message_type`: Type identifier (e.g., std_msgs/String)
- `fields`: Dictionary of field names and types
- `timestamp`: When the message was created
- `header`: Optional metadata (frame_id, timestamp)

**Relationships**:
- Messages are published to topics
- Messages are sent in service requests/responses
- Messages are used in action goals/feedback/results

## Robot Description Entities

### 6. URDF Model
**Definition**: Unified Robot Description Format for defining robot structure, geometry, and kinematics.

**Attributes**:
- `robot_name`: Name of the robot
- `links`: List of rigid body components
- `joints`: List of connections between links
- `materials`: List of visual materials
- `gazebo_extensions`: Simulation-specific extensions

**Relationships**:
- Links are connected by joints
- Links contain visual and collision properties
- Joints define kinematic relationships

### 7. Link
**Definition**: Rigid body component of a robot.

**Attributes**:
- `link_name`: Unique identifier for the link
- `visual`: Visual properties (geometry, material)
- `collision`: Collision properties (geometry)
- `inertial`: Mass, center of mass, and inertia
- `parent_joint`: Joint that connects to parent link

**Relationships**:
- Links are connected to other links through joints
- Each link has one parent joint (except root)

### 8. Joint
**Definition**: Connection between two links defining their kinematic relationship.

**Attributes**:
- `joint_name`: Unique identifier for the joint
- `joint_type`: Type (revolute, prismatic, fixed, etc.)
- `parent_link`: Link that is the parent in the kinematic chain
- `child_link`: Link that is the child in the kinematic chain
- `origin`: Transform from parent to child
- `limits`: Joint limits (for revolute/prismatic joints)

**Relationships**:
- Joints connect parent and child links
- Joints define degrees of freedom between links

## AI Agent Integration Entities

### 9. AI Agent Interface
**Definition**: Structure for connecting AI agents to ROS 2 systems.

**Attributes**:
- `agent_type`: Type of AI agent (behavior tree, state machine, neural network)
- `input_topics`: List of topics the agent subscribes to
- `output_topics`: List of topics the agent publishes to
- `services_used`: List of services the agent calls
- `actions_used`: List of actions the agent uses

**Relationships**:
- AI agents connect to ROS 2 system through various communication patterns
- AI agents process sensor data and send control commands

### 10. Control Command
**Definition**: Commands sent from AI agents to robot controllers.

**Attributes**:
- `command_type`: Type of command (velocity, position, torque)
- `target_joint`: Joint to control (for joint commands)
- `target_frame`: Frame for motion commands
- `command_value`: The actual command value
- `timestamp`: When the command was issued
- `sequence_id`: Command sequence identifier

**Relationships**:
- Commands are published to specific topics or sent via services/actions
- Commands are processed by robot controllers

## Simulation Entities

### 11. Simulation Model
**Definition**: Extension of URDF for simulation-specific properties.

**Attributes**:
- `gazebo_plugins`: List of plugins to load
- `physics_properties`: Mass, friction, damping
- `sensors`: List of simulated sensors
- `actuators`: List of simulated actuators
- `material_properties`: Visual and physical properties

**Relationships**:
- Simulation models extend URDF models
- Simulation models connect to Gazebo simulation environment

### 12. Sensor Model
**Definition**: Simulation of real-world sensors in the virtual environment.

**Attributes**:
- `sensor_type`: Type of sensor (camera, IMU, LIDAR, etc.)
- `frame_id`: Reference frame for sensor data
- `update_rate`: Rate at which sensor publishes data
- `noise_model`: Noise characteristics
- `range_min/max`: Measurement range limits
- `fov`: Field of view (for cameras)

**Relationships**:
- Sensor models are attached to links in URDF
- Sensor models publish data to ROS topics

## Nervous System Pattern Entities

### 13. Distributed Control Pattern
**Definition**: Pattern mimicking biological nervous system organization.

**Attributes**:
- `pattern_type`: Type of pattern (reflex, hierarchical, distributed)
- `sensors`: List of sensor inputs
- `actuators`: List of actuator outputs
- `processing_nodes`: List of nodes performing processing
- `response_time`: Expected response time
- `reliability`: Expected reliability of the pattern

**Relationships**:
- Distributed control patterns span multiple nodes
- Patterns connect sensor inputs to actuator outputs
- Patterns may include intermediate processing nodes

## Validation Rules

### Node Validation
- Node names must follow ROS naming conventions (alphanumeric, underscores, no leading underscores)
- Each node must have a unique name within its namespace
- Nodes must properly clean up resources on shutdown

### Topic Validation
- Topic names must follow ROS naming conventions
- Message types must be properly defined and available
- Publishers and subscribers must use compatible message types

### Service Validation
- Service names must follow ROS naming conventions
- Request and response types must be properly defined
- Services must handle errors gracefully

### URDF Validation
- Robot names must be unique
- All joint parent/child relationships must be valid
- Joint limits must be physically reasonable
- Link masses must be positive
- URDF must form a valid kinematic tree

## State Transitions

### Node Lifecycle States
- Unconfigured → Inactive → Active → Inactive → Unconfigured
- Each state has specific allowed operations
- Transitions must be properly managed

### Action Goal States
- Pending → Active → (Succeeded/Aborted/Canceled)
- Feedback is sent while active
- Results are sent upon completion

## Relationships Summary

1. **Communication Layer**: Nodes communicate via topics, services, and actions
2. **Robot Description Layer**: Links connected by joints form URDF models
3. **Integration Layer**: AI agents interface with ROS 2 through communication patterns
4. **Simulation Layer**: Simulation models extend URDF with Gazebo-specific properties
5. **Pattern Layer**: Distributed control patterns organize nodes into biological-inspired systems

This data model provides the conceptual framework for Module 2 content, ensuring consistency in terminology and concepts across all four chapters.