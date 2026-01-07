# Chapter 3: Sensing and Perception for Physical AI & Robotics

## Learning Objectives
By the end of this chapter, readers will be able to:
- Identify different types of sensors used in robotics and their characteristics
- Apply sensor fusion techniques to combine information from multiple sensors
- Implement state estimation and filtering methods for robotic perception
- Design effective perception systems for robotic applications
- Use probabilistic methods for handling uncertainty in sensor data
- Implement perception examples using PyBullet simulation

## Prerequisites
- Understanding of probability and statistics (covered in Chapter 1)
- Basic knowledge of linear algebra (covered in Chapter 1)
- Fundamental concepts of robotics and coordinate systems
- Basic Python programming skills

## Introduction

Sensing and perception are critical components of robotic systems, enabling robots to understand and interact with their environment. Robots rely on various sensors to gather information about their state and surroundings, then process this information to make informed decisions. This chapter covers sensor types, characteristics, fusion techniques, and state estimation methods essential for building effective robotic perception systems.

## 1. Sensor Types and Characteristics

Robots use various sensors to perceive their environment and internal state. Each sensor type has specific characteristics that make it suitable for particular applications.

### 1.1 Proprioceptive Sensors

Proprioceptive sensors measure the internal state of the robot.

#### Encoders
Encoders measure joint positions and velocities.

```python
import numpy as np
import matplotlib.pyplot as plt

class Encoder:
    def __init__(self, resolution=1000, noise_level=0.01):
        """
        Encoder simulator
        resolution: counts per revolution
        noise_level: standard deviation of noise as fraction of signal
        """
        self.resolution = resolution
        self.noise_level = noise_level
        self.counts_per_rev = resolution
        self.position = 0
        self.velocity = 0

    def read_position(self, true_position, add_noise=True):
        """Read encoder position with optional noise"""
        # Convert position to encoder counts
        counts = int(true_position * self.counts_per_rev / (2 * np.pi))

        # Add noise if requested
        if add_noise:
            noise = np.random.normal(0, self.noise_level * self.counts_per_rev)
            counts += noise

        # Convert back to position
        measured_position = counts * 2 * np.pi / self.counts_per_rev
        return measured_position

    def read_velocity(self, true_velocity, add_noise=True):
        """Read encoder velocity with optional noise"""
        if add_noise:
            noise = np.random.normal(0, self.noise_level * abs(true_velocity))
            return true_velocity + noise
        return true_velocity

# Example: Simulate encoder readings
encoder = Encoder(resolution=4096, noise_level=0.005)

# Simulate a rotating joint
time = np.linspace(0, 2, 100)
true_positions = np.sin(time * np.pi)  # Sinusoidal motion
true_velocities = np.pi * np.cos(time * np.pi)  # Derivative

measured_positions = [encoder.read_position(pos) for pos in true_positions]
measured_velocities = [encoder.read_velocity(vel) for vel in true_velocities]

print(f"Encoder resolution: {encoder.resolution} counts/rev")
print(f"Sample measurements - True pos: {true_positions[10]:.3f}, Measured: {measured_positions[10]:.3f}")
```

#### Inertial Measurement Units (IMUs)
IMUs measure acceleration, angular velocity, and sometimes magnetic field.

```python
class IMU:
    def __init__(self, accel_noise=0.01, gyro_noise=0.001, mag_noise=0.01):
        """
        IMU simulator
        """
        self.accel_noise = accel_noise  # m/s²
        self.gyro_noise = gyro_noise    # rad/s
        self.mag_noise = mag_noise      # arbitrary units
        self.gravity = 9.81  # m/s²

    def read_accelerometer(self, true_acceleration, add_noise=True):
        """Read accelerometer data with noise"""
        if add_noise:
            noise = np.random.normal(0, self.accel_noise, size=true_acceleration.shape)
            return true_acceleration + noise
        return true_acceleration

    def read_gyroscope(self, true_angular_velocity, add_noise=True):
        """Read gyroscope data with noise"""
        if add_noise:
            noise = np.random.normal(0, self.gyro_noise, size=true_angular_velocity.shape)
            return true_angular_velocity + noise
        return true_angular_velocity

    def read_magnetometer(self, true_magnetic_field, add_noise=True):
        """Read magnetometer data with noise"""
        if add_noise:
            noise = np.random.normal(0, self.mag_noise, size=true_magnetic_field.shape)
            return true_magnetic_field + noise
        return true_magnetic_field

# Example: Simulate IMU readings
imu = IMU(accel_noise=0.02, gyro_noise=0.002, mag_noise=0.02)

# Simulate robot movement
true_accel = np.array([0.5, 0.2, 9.81])  # x, y, z acceleration (including gravity)
true_ang_vel = np.array([0.1, -0.05, 0.02])  # Angular velocities
true_mag_field = np.array([0.2, 0.1, 0.5])  # Magnetic field components

measured_accel = imu.read_accelerometer(true_accel)
measured_gyro = imu.read_gyroscope(true_ang_vel)
measured_mag = imu.read_magnetometer(true_mag_field)

print(f"Accelerometer - True: {true_accel}, Measured: {measured_accel}")
print(f"Gyroscope - True: {true_ang_vel}, Measured: {measured_gyro}")
```

### 1.2 Exteroceptive Sensors

Exteroceptive sensors measure properties of the external environment.

#### Range Sensors
Range sensors measure distances to objects in the environment.

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

# Example: Simulate range sensor readings
range_sensor = RangeSensor(max_range=5.0, min_range=0.05, accuracy=0.02)

# Simulate measurements at different distances
distances = [0.5, 1.0, 2.0, 4.0, 6.0, 0.02]
for dist in distances:
    measured = range_sensor.measure_distance(dist)
    detected = range_sensor.detect_object(dist)
    print(f"True: {dist:.2f}m -> Measured: {measured:.2f}m, Detected: {detected}")
```

#### Cameras
Cameras provide rich visual information about the environment.

```python
class Camera:
    def __init__(self, width=640, height=480, fov_h=60, fov_v=45, focal_length=500):
        """
        Camera simulator
        """
        self.width = width
        self.height = height
        self.fov_h = fov_h  # horizontal field of view in degrees
        self.fov_v = fov_v  # vertical field of view in degrees
        self.focal_length = focal_length
        self.cx = width / 2  # principal point x
        self.cy = height / 2  # principal point y

    def world_to_pixel(self, world_point, camera_pose):
        """
        Convert 3D world point to 2D pixel coordinates
        world_point: [x, y, z] in world coordinates
        camera_pose: [x, y, z, roll, pitch, yaw] pose of camera
        """
        # This is a simplified version - in practice, this involves
        # transforming to camera frame and applying projection
        x_w, y_w, z_w = world_point

        # For simplicity, assume camera is at origin looking along -z axis
        # Transform point to camera frame (simplified)
        x_cam = x_w - camera_pose[0]
        y_cam = y_w - camera_pose[1]
        z_cam = z_w - camera_pose[2]

        # Project to image plane
        if z_cam > 0:  # Point in front of camera
            x_pix = self.focal_length * x_cam / z_cam + self.cx
            y_pix = self.focal_length * y_cam / z_cam + self.cy
            return [x_pix, y_pix]
        else:
            return None  # Point behind camera

    def get_depth_at_pixel(self, pixel_coords, depth_map):
        """
        Get depth value at specific pixel coordinates
        """
        x, y = int(pixel_coords[0]), int(pixel_coords[1])
        if 0 <= x < self.width and 0 <= y < self.height:
            return depth_map[y, x]
        return None

# Example: Simulate camera projection
camera = Camera(width=320, height=240, fov_h=60, fov_v=45)

# Simulate a point in the world
world_point = [1.0, 0.5, 2.0]  # x, y, z in meters
camera_pose = [0, 0, 0, 0, 0, 0]  # x, y, z, roll, pitch, yaw

pixel_coords = camera.world_to_pixel(world_point, camera_pose)
if pixel_coords:
    print(f"World point {world_point} -> Pixel coordinates {pixel_coords}")
else:
    print(f"World point {world_point} not visible to camera")
```

## 2. Sensor Fusion Techniques

Sensor fusion combines information from multiple sensors to improve perception accuracy and robustness.

### 2.1 Weighted Average Fusion

Simple fusion technique for combining redundant sensor measurements.

```python
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

# Example: Fuse measurements from two sensors
sensor1_measurement = 10.2
sensor1_uncertainty = 0.5

sensor2_measurement = 9.8
sensor2_uncertainty = 0.8

measurements = [sensor1_measurement, sensor2_measurement]
uncertainties = [sensor1_uncertainty, sensor2_uncertainty]

fused_result, fused_unc = weighted_average_fusion(measurements, uncertainties)

print(f"Sensor 1: {sensor1_measurement} ± {sensor1_uncertainty}")
print(f"Sensor 2: {sensor2_measurement} ± {sensor2_uncertainty}")
print(f"Fused result: {fused_result:.3f} ± {fused_unc:.3f}")
```

### 2.2 Covariance Intersection

Method for fusing estimates when cross-correlations are unknown.

```python
def covariance_intersection(mean1, cov1, mean2, cov2):
    """
    Fuse two estimates using covariance intersection
    Handles unknown correlations between estimates
    """
    # For scalar case (simple version)
    if np.isscalar(cov1) and np.isscalar(cov2):
        # Calculate weights
        w1 = cov2 / (cov1 + cov2)
        w2 = cov1 / (cov1 + cov2)

        # Fused estimate
        fused_mean = w1 * mean1 + w2 * mean2

        # Fused covariance
        fused_cov = 1.0 / (1.0/cov1 + 1.0/cov2)

        return fused_mean, fused_cov

    # For vector/matrix case (simplified implementation)
    # In practice, this would involve matrix inversions
    else:
        # Calculate weights based on determinant of covariances
        det1_inv = 1.0 / np.linalg.det(cov1) if np.isscalar(np.linalg.det(cov1)) else 1.0
        det2_inv = 1.0 / np.linalg.det(cov2) if np.isscalar(np.linalg.det(cov2)) else 1.0

        omega = det1_inv / (det1_inv + det2_inv)

        # Fused estimate
        fused_mean = omega * mean1 + (1 - omega) * mean2

        # Fused covariance
        fused_cov = np.linalg.inv(omega * np.linalg.inv(cov1) + (1 - omega) * np.linalg.inv(cov2))

        return fused_mean, fused_cov

# Example: Covariance intersection for scalar values
mean1, cov1 = 10.2, 0.25  # Variance = 0.25, Std dev = 0.5
mean2, cov2 = 9.8, 0.64   # Variance = 0.64, Std dev = 0.8

fused_mean_ci, fused_cov_ci = covariance_intersection(mean1, cov1, mean2, cov2)

print(f"Covariance Intersection:")
print(f"Estimate 1: {mean1} ± {np.sqrt(cov1):.3f}")
print(f"Estimate 2: {mean2} ± {np.sqrt(cov2):.3f}")
print(f"Fused estimate: {fused_mean_ci:.3f} ± {np.sqrt(fused_cov_ci):.3f}")
```

## 3. State Estimation and Filtering Methods

State estimation is crucial for maintaining accurate knowledge of robot state despite noisy sensor data.

### 3.1 Kalman Filter

The Kalman filter is an optimal estimator for linear systems with Gaussian noise.

```python
class KalmanFilter:
    def __init__(self, dim_x, dim_z, dim_u=0):
        """
        Kalman Filter implementation
        dim_x: dimension of state vector
        dim_z: dimension of measurement vector
        dim_u: dimension of control input vector
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        # State vector: [position, velocity]
        self.x = np.zeros((dim_x, 1))

        # State covariance matrix
        self.P = np.eye(dim_x) * 1000

        # Process noise covariance
        self.Q = np.eye(dim_x)

        # Measurement noise covariance
        self.R = np.eye(dim_z)

        # State transition matrix
        self.F = np.eye(dim_x)

        # Measurement function matrix
        self.H = np.zeros((dim_z, dim_x))

        # Control transition matrix
        self.B = np.zeros((dim_x, dim_u)) if dim_u > 0 else 0

    def predict(self, u=None):
        """
        Predict next state
        u: control input
        """
        # State prediction: x = F*x + B*u
        if u is not None:
            self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        else:
            self.x = np.dot(self.F, self.x)

        # Covariance prediction: P = F*P*F^T + Q
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        """
        Update state estimate with measurement
        z: measurement vector
        """
        # Innovation: y = z - H*x
        y = z - np.dot(self.H, self.x)

        # Innovation covariance: S = H*P*H^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # Kalman gain: K = P*H^T*S^(-1)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # State update: x = x + K*y
        self.x = self.x + np.dot(K, y)

        # Covariance update: P = (I - K*H)*P
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

# Example: 1D position tracking with Kalman filter
kf = KalmanFilter(dim_x=2, dim_z=1)  # Position and velocity, 1D measurement

# Initialize state: [position, velocity]
kf.x = np.array([[0.0], [0.0]])

# State transition matrix (constant velocity model)
dt = 0.1  # time step
kf.F = np.array([[1, dt],
                 [0, 1]])

# Measurement function (only position is measured)
kf.H = np.array([[1, 0]])

# Process noise (how much we expect the model to be wrong)
kf.Q = np.array([[0.01, 0],
                 [0, 0.1]])

# Measurement noise (sensor noise)
kf.R = np.array([[0.1]])

# Simulate measurements and track
true_positions = []
measurements = []
estimates = []
times = []

for t in np.arange(0, 10, dt):
    # Simulate true motion (simple acceleration)
    true_pos = 0.1 * t**2  # Quadratic motion
    true_vel = 0.2 * t

    # Add noise to measurement
    measured_pos = true_pos + np.random.normal(0, 0.1)

    # Predict
    kf.predict()

    # Update with measurement
    kf.update(np.array([[measured_pos]]))

    # Store results
    true_positions.append(true_pos)
    measurements.append(measured_pos)
    estimates.append(kf.x[0, 0])
    times.append(t)

print(f"Kalman filter example completed with {len(times)} time steps")
print(f"Final estimate: Position={kf.x[0,0]:.3f}, Velocity={kf.x[1,0]:.3f}")
```

### 3.2 Extended Kalman Filter (EKF)

For nonlinear systems, the Extended Kalman Filter linearizes around the current estimate.

```python
class ExtendedKalmanFilter:
    def __init__(self, dim_x, dim_z, dim_u=0):
        """
        Extended Kalman Filter for nonlinear systems
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = np.zeros((dim_x, 1))  # State
        self.P = np.eye(dim_x) * 1000  # Covariance
        self.Q = np.eye(dim_x)         # Process noise
        self.R = np.eye(dim_z)         # Measurement noise

    def predict_and_jacobian(self, fx, FJacobian, u=None):
        """
        Predict step with Jacobian of nonlinear function
        """
        # Update state: x = fx(x, u)
        self.x = fx(self.x, u)

        # Get Jacobian of state transition function
        F = FJacobian(self.x, u)

        # Update covariance: P = F*P*F^T + Q
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update_and_jacobian(self, hx, HJacobian, z):
        """
        Update step with Jacobian of measurement function
        """
        # Get measurement prediction
        z_pred = hx(self.x)

        # Get Jacobian of measurement function
        H = HJacobian(self.x)

        # Innovation: y = z - z_pred
        y = z - z_pred

        # Innovation covariance: S = H*P*H^T + R
        S = np.dot(np.dot(H, self.P), H.T) + self.R

        # Kalman gain: K = P*H^T*S^(-1)
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # State update: x = x + K*y
        self.x = self.x + np.dot(K, y)

        # Covariance update: P = (I - K*H)*P
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, H)), self.P)

# Example: Nonlinear tracking with EKF (tracking in polar coordinates)
def fx_radar(state, dt):
    """Nonlinear state transition function for radar tracking"""
    x, y, vx, vy = state.flatten()

    # Update position based on velocity
    new_x = x + vx * dt
    new_y = y + vy * dt
    new_vx = vx  # Assume constant velocity model
    new_vy = vy

    return np.array([[new_x], [new_y], [new_vx], [new_vy]])

def FJacobian_radar(state, dt):
    """Jacobian of state transition function"""
    return np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def hx_radar(state):
    """Measurement function: convert cartesian to polar"""
    x, y, _, _ = state.flatten()

    # Convert to range and bearing
    rng = np.sqrt(x**2 + y**2)
    bearing = np.arctan2(y, x)

    return np.array([[rng], [bearing]])

def HJacobian_radar(state):
    """Jacobian of measurement function"""
    x, y, _, _ = state.flatten()
    r = np.sqrt(x**2 + y**2)

    return np.array([
        [x/r, y/r, 0, 0],           # dr/dx, dr/dy, dr/dvx, dr/dvy
        [-y/(r**2), x/(r**2), 0, 0] # db/dx, db/dy, db/dvx, db/dvy
    ])

# Initialize EKF for radar tracking
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)  # x, y, vx, vy -> range, bearing

# Initial state (position and velocity)
ekf.x = np.array([[1000.0], [0.0], [0.0], [100.0]])  # Moving north at 100 m/s

# Covariances
ekf.Q = np.eye(4) * 0.1  # Process noise
ekf.R = np.array([[5**2, 0], [0, 0.01**2]])  # Measurement noise (range and bearing)

print(f"Extended Kalman Filter initialized for radar tracking")
print(f"Initial state: Position=({ekf.x[0,0]:.1f}, {ekf.x[1,0]:.1f}), "
      f"Velocity=({ekf.x[2,0]:.1f}, {ekf.x[3,0]:.1f})")
```

### 3.3 Particle Filter

For highly nonlinear systems with non-Gaussian noise, particle filters can be more appropriate.

```python
class ParticleFilter:
    def __init__(self, num_particles, dim_state):
        """
        Particle Filter implementation
        """
        self.num_particles = num_particles
        self.dim_state = dim_state

        # Initialize particles randomly
        self.particles = np.random.randn(num_particles, dim_state)
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, process_noise_std):
        """
        Predict step: propagate particles through motion model
        """
        # Add random noise to each particle
        noise = np.random.normal(0, process_noise_std, self.particles.shape)
        self.particles += noise

    def update(self, measurement, measurement_std, measurement_model):
        """
        Update step: weight particles based on measurement likelihood
        """
        # Calculate likelihood of measurement for each particle
        for i, particle in enumerate(self.particles):
            predicted_measurement = measurement_model(particle)

            # Calculate likelihood (assuming Gaussian noise)
            likelihood = np.exp(-0.5 * ((measurement - predicted_measurement) / measurement_std)**2)

            # Update weight
            self.weights[i] *= likelihood

        # Normalize weights
        self.weights += 1.e-300  # Avoid division by zero
        self.weights /= np.sum(self.weights)

    def resample(self):
        """
        Resample particles based on weights
        """
        # Systematic resampling
        indices = []
        cumulative_sum = np.cumsum(self.weights)
        start = np.random.random() / self.num_particles

        i, j = 0, 0
        while i < self.num_particles:
            if start < cumulative_sum[j]:
                indices.append(j)
                start += 1.0 / self.num_particles
                i += 1
            else:
                j += 1

        # Resample particles
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        """
        Calculate state estimate as weighted average of particles
        """
        return np.average(self.particles, weights=self.weights, axis=0)

# Example: Simple 1D position tracking with particle filter
def measurement_model_1d(particle_state):
    """Simple measurement model: just return position"""
    return particle_state[0]  # Assume first element is position

pf = ParticleFilter(num_particles=1000, dim_state=2)  # Position and velocity

# Set initial particles around an estimated position
pf.particles[:, 0] = np.random.normal(10.0, 2.0, pf.num_particles)  # Position
pf.particles[:, 1] = np.random.normal(1.0, 0.5, pf.num_particles)   # Velocity

# Simulate tracking
true_position = 10.0
measurements = [9.8, 10.2, 10.1, 9.9, 10.3, 10.0, 9.7, 10.4]

for measurement in measurements:
    # Prediction step
    pf.predict(process_noise_std=0.1)

    # Update step
    pf.update(measurement, measurement_std=0.5, measurement_model=measurement_model_1d)

    # Resample if effective sample size is low
    neff = 1.0 / np.sum(pf.weights**2)
    if neff < pf.num_particles / 2:
        pf.resample()

    # Get estimate
    estimate = pf.estimate()
    print(f"Measurement: {measurement:.2f}, Estimate: {estimate[0]:.2f}")

final_estimate = pf.estimate()
print(f"Final particle filter estimate: Position={final_estimate[0]:.3f}, Velocity={final_estimate[1]:.3f}")
```

## 4. PyBullet Examples for Perception

Let's implement some perception examples using PyBullet.

```python
import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_perception_demo():
    """Set up PyBullet environment for perception demonstration"""
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)

    # Set gravity
    p.setGravity(0, 0, -9.81)

    # Load plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # Add some objects for the robot to perceive
    boxId = p.loadURDF("cube.urdf", [2, 0, 0.5], useFixedBase=False)
    sphereId = p.loadURDF("sphere2.urdf", [-1, 1, 0.5], useFixedBase=False)

    return physicsClient, boxId, sphereId

def demo_ray_casting_perception():
    """
    Demonstrate ray casting for distance sensing
    This simulates how a robot might use LIDAR or similar sensors
    """
    # Set up environment
    physicsClient, boxId, sphereId = setup_perception_demo()

    # Define robot position and multiple ray directions (simulating LIDAR)
    robot_pos = [0, 0, 1]
    ray_directions = []

    # Create rays in a circular pattern around the robot
    for angle in np.linspace(0, 2*np.pi, 36):  # 36 rays
        direction = [np.cos(angle), np.sin(angle), 0]
        ray_directions.append(direction)

    # Perform ray casting
    ray_starts = [robot_pos for _ in ray_directions]
    ray_ends = [[robot_pos[0] + 10*d[0], robot_pos[1] + 10*d[1], robot_pos[2] + 10*d[2]]
                for d in ray_directions]

    results = p.rayTestBatch(ray_starts, ray_ends)

    # Process results
    distances = []
    for i, result in enumerate(results):
        hit_fraction = result[2]  # Fraction of ray length where hit occurred
        if hit_fraction == 1.0:
            # No hit, maximum range
            distances.append(10.0)  # Maximum range
        else:
            # Calculate actual distance
            distance = hit_fraction * 10.0
            distances.append(distance)

    print(f"Ray casting completed: {len(distances)} distance measurements")
    print(f"Min distance: {min(distances):.2f}m, Max distance: {max(distances):.2f}m")

    # Run simulation briefly to visualize
    for i in range(300):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()

    return distances

# Note: This example would run in an environment with PyBullet installed
print("PyBullet perception demo defined - requires PyBullet installation to run")
```

## 5. Sensor Fusion Exercise

### Exercise 1: Multi-Sensor Position Estimation
Implement a system that fuses GPS, IMU, and wheel encoder data for position estimation.

```python
class MultiSensorFusion:
    def __init__(self):
        """
        Multi-sensor fusion system combining GPS, IMU, and wheel encoders
        """
        # Initialize Kalman filter for position and velocity
        self.kf = KalmanFilter(dim_x=4, dim_z=3)  # [x, y, vx, vy] state, [x_gps, y_gps, theta_imu] measurement

        # Initial state: [x, y, vx, vy]
        self.kf.x = np.array([[0.0], [0.0], [0.0], [0.0]])

        # State transition model (constant velocity)
        dt = 0.1
        self.kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Measurement function: we can measure x, y position and heading
        self.kf.H = np.array([
            [1, 0, 0, 0],  # Measure x position
            [0, 1, 0, 0],  # Measure y position
            [0, 0, 0, 0]   # We'll handle heading separately
        ])

        # Process noise
        self.kf.Q = np.array([
            [0.01, 0, 0, 0],
            [0, 0.01, 0, 0],
            [0, 0, 0.1, 0],
            [0, 0, 0, 0.1]
        ])

        # Measurement noise
        self.kf.R = np.array([
            [1.0, 0, 0],    # GPS position noise
            [0, 1.0, 0],    # GPS position noise
            [0, 0, 0.01]    # IMU heading noise
        ])

        self.dt = dt
        self.time = 0

    def update_with_sensors(self, gps_pos, imu_yaw, wheel_odom_delta):
        """
        Update the state estimate using multiple sensor inputs
        """
        # Prediction step
        self.kf.predict()
        self.time += self.dt

        # Create measurement vector
        # For this example, we'll use GPS position and IMU heading
        measurement = np.array([[gps_pos[0]], [gps_pos[1]], [imu_yaw]])

        # Update with measurement
        self.kf.update(measurement)

        return self.kf.x.flatten()

# Example: Simulate multi-sensor fusion
fusion_system = MultiSensorFusion()

# Simulate sensor data
for step in range(50):
    # Simulate true motion
    true_x = 0.1 * step * np.cos(step * 0.1)  # Spiral motion
    true_y = 0.1 * step * np.sin(step * 0.1)
    true_heading = step * 0.05  # Gradually changing heading

    # Simulate noisy sensor readings
    gps_pos = [true_x + np.random.normal(0, 0.5), true_y + np.random.normal(0, 0.5)]
    imu_yaw = true_heading + np.random.normal(0, 0.01)
    wheel_odom_delta = [0.1, 0.01]  # Simulated wheel encoder data

    # Update fusion system
    state_estimate = fusion_system.update_with_sensors(gps_pos, imu_yaw, wheel_odom_delta)

    if step % 10 == 0:  # Print every 10 steps
        print(f"Step {step}: Estimated pos=({state_estimate[0]:.2f}, {state_estimate[1]:.2f}), "
              f"True pos=({true_x:.2f}, {true_y:.2f})")

final_state = fusion_system.kf.x.flatten()
print(f"Final fused state: x={final_state[0]:.3f}, y={final_state[1]:.3f}, "
      f"vx={final_state[2]:.3f}, vy={final_state[3]:.3f}")
```

## 6. Summary

This chapter covered the essential aspects of sensing and perception in robotics:

1. **Sensor Types**: Understanding proprioceptive (encoders, IMUs) and exteroceptive (range sensors, cameras) sensors
2. **Sensor Characteristics**: Learning about accuracy, precision, range, field of view, and noise characteristics
3. **Sensor Fusion**: Combining information from multiple sensors to improve perception
4. **State Estimation**: Using filtering techniques (Kalman, Extended Kalman, Particle) to maintain accurate state estimates
5. **Practical Implementation**: Using PyBullet for perception simulation

Effective sensing and perception are fundamental to Physical AI systems, enabling robots to understand their environment and make intelligent decisions. The fusion of multiple sensor modalities is crucial for robust operation in real-world conditions.

## 7. Implementation Guide

To implement the sensing and perception concepts covered in this chapter:

1. Experiment with different sensor models and their characteristics
2. Implement sensor fusion algorithms to combine multiple sensor inputs
3. Practice state estimation using various filtering techniques
4. Use PyBullet to simulate sensor data and test perception algorithms
5. Apply probabilistic methods to handle uncertainty in sensor data
6. Design perception systems tailored to specific robotic applications

The exercises provided offer hands-on practice with these fundamental concepts, preparing readers for more advanced topics in robotics and Physical AI.