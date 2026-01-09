# Physics Simulation and Sensor Simulation - Solutions

## Solution 3.1: Physics Parameter Tuning

**Physics configuration for stable humanoid simulation:**

```xml
<physics type='ode'>
  <max_step_size>0.001</max_step_size>  <!-- Small step for stability -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>  <!-- More iterations for stability -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>1e-5</cfm>  <!-- Small CFM for tight constraints -->
      <erp>0.8</erp>   <!-- High ERP for good contact resolution -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Humanoid model with stable feet:**
```xml
<link name="foot_link">
  <visual>
    <geometry>
      <box size="0.15 0.08 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.15 0.08 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
  </inertial>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>    <!-- High friction for stability -->
        <mu2>0.8</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <soft_cfm>0.0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1e+6</kp>   <!-- Stiff contact -->
        <kd>1e+3</kd>
      </ode>
    </contact>
  </surface>
</link>
```

**Trade-offs:**
- Stability vs. Performance: More iterations = more stable but slower
- Accuracy vs. Speed: Smaller time steps = more accurate but computationally expensive

## Solution 3.2: LiDAR Sensor Implementation

**Complete LiDAR configuration:**
```xml
<sensor name="front_lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2cm noise -->
    </noise>
  </ray>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>false</visualize>
</sensor>
```

**Testing different surface materials:**
- Smooth surfaces (metal, glass): Clean reflections
- Rough surfaces (grass, gravel): More scattering
- Absorptive materials (carpet): Reduced range returns

## Solution 3.3: Depth Camera Simulation

**Realistic depth camera configuration:**
```xml
<sensor name="rgb_depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>5.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm at 1m -->
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <point_cloud>
    <output>sensor</output>
    <point_cloud_min_dist>0.1</point_cloud_min_dist>
    <point_cloud_max_dist>5.0</point_cloud_max_dist>
  </point_cloud>
</sensor>
```

**Distance-dependent noise function:**
```python
def distance_dependent_noise(distance, base_noise=0.005):
    """Model noise that increases with distance"""
    return base_noise * (1 + distance * 0.1)  # Noise increases with distance
```

## Solution 3.4: IMU Sensor Calibration and Noise Modeling

**Advanced IMU configuration:**
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.2e-3</stddev>  <!-- 1.2 mrad/s -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>5.0e-4</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.2e-3</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>5.0e-4</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.2e-3</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>5.0e-4</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>8.3e-4</stddev>  <!-- 830 μg -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1.7e-3</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>8.3e-4</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1.7e-3</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>8.3e-4</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1.7e-3</bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          <dynamic_bias_stddev>1.0e-5</dynamic_bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

**IMU bias drift simulation:**
```python
import numpy as np

class IMUBiasDrift:
    def __init__(self):
        self.accel_bias = np.random.normal(0, 1.7e-3, 3)  # Initial bias
        self.gyro_bias = np.random.normal(0, 5.0e-4, 3)
        self.time_constant = 300  # Correlation time in seconds

    def update_bias(self, dt):
        """Update bias with random walk"""
        # Random walk for bias
        self.accel_bias += np.random.normal(0, 1.0e-5, 3) * dt
        self.gyro_bias += np.random.normal(0, 1.0e-6, 3) * dt
        return self.accel_bias, self.gyro_bias
```

## Solution 3.5: Sensor Fusion Algorithm Implementation

**Extended Kalman Filter for position fusion:**
```python
import numpy as np
from scipy.linalg import inv

class PositionEKF:
    def __init__(self):
        # State: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Initial covariance
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])  # Process noise
        self.R_lidar = 0.02**2   # LiDAR measurement noise (2cm)
        self.R_camera = 0.01**2  # Camera measurement noise (1cm)

    def predict(self, dt):
        """Motion model prediction"""
        # State transition matrix for constant velocity model
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # Process noise matrix
        G = np.array([
            [0.5*dt**2, 0, 0],
            [0, 0.5*dt**2, 0],
            [0, 0, 0.5*dt**2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt]
        ])

        # Predict state and covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ np.diag([0.1, 0.1, 0.1]) @ G.T

    def update_lidar(self, z):
        """Update with LiDAR position measurement [x, y, z]"""
        # Observation matrix for position
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        # Innovation
        y = z - H @ self.x
        S = H @ self.P @ H.T + np.eye(3) * self.R_lidar
        K = self.P @ H.T @ inv(S)

        # Update state and covariance
        self.x = self.x + K @ y
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P

    def update_camera(self, z):
        """Update with camera position measurement [x, y, z]"""
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        y = z - H @ self.x
        S = H @ self.P @ H.T + np.eye(3) * self.R_camera
        K = self.P @ H.T @ inv(S)

        self.x = self.x + K @ y
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P
```

**Complementary filter for IMU-accelerometer fusion:**
```python
class IMUComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.dt = 0.01

    def update(self, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z):
        # Integrate gyroscope
        self.pitch += gyro_x * self.dt
        self.roll += gyro_y * self.dt
        self.yaw += gyro_z * self.dt

        # Calculate angles from accelerometer
        pitch_acc = np.arctan2(acc_y, np.sqrt(acc_x**2 + acc_z**2))
        roll_acc = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2))

        # Complementary filter
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc

        return self.pitch, self.roll, self.yaw
```

## Solution 3.6: Sensor Validation and Testing

**Ground truth comparison:**
```python
def validate_lidar_accuracy(true_positions, measured_ranges, angles):
    """Validate LiDAR accuracy against known positions"""
    errors = []

    for i, (true_pos, measured_range, angle) in enumerate(zip(true_positions, measured_ranges, angles)):
        # Calculate expected range to ground truth
        expected_range = np.linalg.norm(true_pos)

        # Calculate error
        error = abs(measured_range - expected_range)
        errors.append(error)

    # Calculate statistics
    rmse = np.sqrt(np.mean(np.square(errors)))
    mae = np.mean(np.abs(errors))
    std_err = np.std(errors)

    print(f"LIDAR Validation Results:")
    print(f"  RMSE: {rmse:.4f} m")
    print(f"  MAE: {mae:.4f} m")
    print(f"  Std Dev: {std_err:.4f} m")
    print(f"  Bias: {np.mean(errors):.4f} m")

    return rmse, mae, std_err
```

**Statistical analysis of sensor noise:**
```python
def analyze_sensor_noise(data, sample_rate=100):
    """Analyze noise characteristics of sensor data"""
    from scipy import signal
    from scipy.stats import kurtosis, skew

    # Time domain analysis
    mean_val = np.mean(data)
    std_val = np.std(data)
    variance = np.var(data)

    # Frequency domain analysis (Power Spectral Density)
    freqs, psd = signal.welch(data, fs=sample_rate, nperseg=len(data)//10)

    # Statistical tests
    kurt = kurtosis(data)
    skewness = skew(data)

    print(f"Statistical Analysis:")
    print(f"  Mean: {mean_val:.6f}")
    print(f"  Std Dev: {std_val:.6f}")
    print(f"  Variance: {variance:.6f}")
    print(f"  Kurtosis: {kurt:.4f} (Gaussian = 0)")
    print(f"  Skewness: {skewness:.4f} (Symmetric = 0)")

    return mean_val, std_val, variance, freqs, psd
```

## Solution 3.7: Performance Optimization

**Point cloud downsampling:**
```python
def voxel_grid_filter(points, voxel_size=0.01):
    """Downsample point cloud using voxel grid filter"""
    if len(points) == 0:
        return points

    # Convert to voxel coordinates
    voxel_coords = np.floor(points / voxel_size).astype(int)

    # Group points by voxel
    voxel_dict = {}
    for i, coord in enumerate(voxel_coords):
        coord_tuple = tuple(coord)
        if coord_tuple not in voxel_dict:
            voxel_dict[coord_tuple] = []
        voxel_dict[coord_tuple].append(i)

    # Take centroid of each voxel
    downsampled_indices = [indices[0] for indices in voxel_dict.values()]
    return points[downsampled_indices]

def adaptive_downsampling(points, target_count=1000):
    """Adaptively downsample to target point count"""
    if len(points) <= target_count:
        return points

    step = len(points) // target_count
    return points[::step]
```

**Multi-threaded sensor processing:**
```python
import threading
import queue
import time

class MultiThreadedSensorFusion:
    def __init__(self):
        self.lidar_queue = queue.Queue(maxsize=10)
        self.camera_queue = queue.Queue(maxsize=10)
        self.imu_queue = queue.Queue(maxsize=10)
        self.fused_output = queue.Queue(maxsize=10)
        self.running = True

    def lidar_processor(self):
        """Process LiDAR data in separate thread"""
        while self.running:
            try:
                lidar_data = self.lidar_queue.get(timeout=0.1)
                # Process LiDAR data
                processed = self._process_lidar(lidar_data)
                # Put on fusion queue
                self.fused_output.put(('lidar', processed))
            except queue.Empty:
                continue

    def camera_processor(self):
        """Process camera data in separate thread"""
        while self.running:
            try:
                camera_data = self.camera_queue.get(timeout=0.1)
                processed = self._process_camera(camera_data)
                self.fused_output.put(('camera', processed))
            except queue.Empty:
                continue

    def start_processing(self):
        """Start all processing threads"""
        threading.Thread(target=self.lidar_processor, daemon=True).start()
        threading.Thread(target=self.camera_processor, daemon=True).start()
```

## Solution 3.8: Environmental Effects on Sensors

**Lighting effects on camera:**
```xml
<sensor name="adaptive_camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <!-- Add exposure compensation for lighting -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
</sensor>
```

**Weather effects on LiDAR:**
```python
def simulate_weather_effects(base_range, weather_condition="clear"):
    """Simulate weather effects on LiDAR range measurements"""
    if weather_condition == "clear":
        return base_range
    elif weather_condition == "fog":
        # Fog reduces effective range
        attenuation = 0.1  # 10% loss per meter
        return base_range * np.exp(-attenuation * base_range)
    elif weather_condition == "rain":
        # Rain causes additional noise
        noise_multiplier = 1.5
        return base_range + np.random.normal(0, 0.02 * noise_multiplier)
    elif weather_condition == "snow":
        # Snow can cause false returns
        false_return_prob = 0.05
        if np.random.random() < false_return_prob:
            return base_range * 0.5  # Early return
        return base_range
    else:
        return base_range
```

**Electromagnetic interference on IMU:**
```python
def add_em_interference(raw_imu_data, interference_level=0.0):
    """Add electromagnetic interference to IMU readings"""
    if interference_level > 0:
        # Add correlated noise that mimics EM interference
        interference_signal = np.random.normal(0, interference_level, size=raw_imu_data.shape)
        # Add some frequency components typical of EM interference
        t = np.arange(len(interference_signal))
        em_noise = 0.1 * interference_level * np.sin(2 * np.pi * 60 * t * 0.01)  # 60Hz line noise
        return raw_imu_data + interference_signal + em_noise
    return raw_imu_data
```

---

## Back to Chapter Contents
Return to [Chapter 3 Content](./content.md) | Continue to [Chapter 4](../chapter-4-unity-integration/content.md)