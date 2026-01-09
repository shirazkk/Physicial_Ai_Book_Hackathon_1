# Data Model: Module 3 - The Digital Twin (Gazebo & Unity)

## Key Entities and System Concepts

### Entity 1: Digital Twin Framework
**Description**: The overarching system that connects physical robots with their virtual counterparts
**Attributes**:
- TwinID: Unique identifier for each digital twin instance
- PhysicalRobotModel: Reference to the real robot specifications
- VirtualRobotModel: Reference to the simulation model
- SynchronizationRate: Frequency of data exchange between physical and virtual
- FidelityLevel: Level of detail and accuracy in the digital twin

**Relationships**:
- Contains multiple RobotModels
- Connected to SimulationEnvironments
- Interfaces with SensorSimulations

### Entity 2: Simulation Environment
**Description**: Virtual world where robot simulation occurs
**Attributes**:
- EnvironmentID: Unique identifier for the environment
- WorldDescription: SDF file defining the environment geometry
- PhysicsEngine: Type of physics engine (ODE, Bullet, DART)
- GravitySettings: Gravitational parameters
- LightingConditions: Environmental lighting configuration

**Relationships**:
- Contains multiple RobotModels
- Connected to DigitalTwinFramework
- Associated with SensorSimulations

### Entity 3: Robot Model
**Description**: Representation of a robot in both URDF and SDF formats
**Attributes**:
- RobotID: Unique identifier for the robot model
- URDFPath: Path to the URDF file
- SDFPath: Path to the SDF file (if different from URDF)
- JointConfigurations: Array of joint limits and properties
- LinkProperties: Mass, inertia, and visual properties for each link

**Relationships**:
- Belongs to SimulationEnvironment
- Connected to SensorSimulations
- Part of DigitalTwinFramework

### Entity 4: Sensor Simulation
**Description**: Virtual sensors that replicate real-world sensor behavior
**Attributes**:
- SensorID: Unique identifier for the sensor
- SensorType: Type of sensor (LiDAR, DepthCamera, IMU, etc.)
- NoiseParameters: Configuration for sensor noise and inaccuracies
- UpdateRate: Frequency at which the sensor produces data
- FieldOfView: Angular field of view for vision-based sensors

**Relationships**:
- Attached to RobotModel
- Connected to SimulationEnvironment
- Interfaces with ROSMessageTypes

### Entity 5: ROS Message Types
**Description**: Standardized message formats for communication between components
**Attributes**:
- MessageType: Name of the ROS message type
- Fields: List of data fields in the message
- Frequency: Typical publishing frequency
- Publishers: List of nodes that publish this message type
- Subscribers: List of nodes that subscribe to this message type

**Relationships**:
- Connected to SensorSimulations
- Used by DigitalTwinFramework

## System Relationships and Interactions

### Digital Twin System Flow
1. Physical robot collects sensor data
2. Data transmitted to DigitalTwinFramework
3. Virtual robot in SimulationEnvironment updated
4. SensorSimulations generate virtual sensor data
5. Data published via ROSMessageTypes
6. Unity visualization updated based on simulation data

### Data Validation Rules
- Each RobotModel must have valid URDF/SDF syntax
- SimulationEnvironment must have consistent physics parameters
- SensorSimulation update rates must be within realistic bounds
- DigitalTwinFramework synchronization rate must support real-time operation

## State Models

### Robot Model State Machine
- **Idle**: Robot model loaded but not simulated
- **Initializing**: Physics properties being calculated
- **Active**: Robot is being simulated with physics
- **Paused**: Simulation temporarily stopped
- **Error**: Error condition requiring intervention

### Sensor Simulation State Machine
- **Off**: Sensor not active
- **Calibrating**: Noise parameters being set
- **Active**: Producing sensor data
- **Overloaded**: Sensor data saturated
- **Fault**: Sensor malfunction simulation

## Integration Points

### Gazebo-Unity Bridge
**Interface**: ROS TCP Connection
**Data Flow**: Robot poses, sensor data, control commands
**Frequency**: Configurable based on application needs
**Validation**: Data integrity and timing consistency checks

### ROS Integration Layer
**Interface**: Standard ROS message types
**Data Flow**: Bidirectional communication between all components
**Frequency**: Depends on individual component requirements
**Validation**: Message format and content validation

## Data Persistence and Versioning

### Model Versioning
- Each RobotModel has version number
- SimulationEnvironment has revision history
- DigitalTwinFramework configurations are versioned
- Change logs maintained for all modifications

### Configuration Management
- Default parameter sets for different robot types
- Environment presets for common scenarios
- Sensor configuration templates
- Validation profiles for different use cases