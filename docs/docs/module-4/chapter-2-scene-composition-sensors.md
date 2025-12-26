---
id: chapter-2-scene-composition-sensors
title: "Chapter 2: Scene Composition and Sensors"
sidebar_label: "2. Scenes & Sensors"
sidebar_position: 3
---

# Chapter 2: Isaac Sim Scene Composition and Sensors

## Chapter Goal

By the end of this chapter, you will be able to **master USD-based scene composition in Isaac Sim and configure RTX-accelerated sensors** with physically-based rendering for photorealistic synthetic data.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 2.1 | Create Isaac Sim scenes using USD layers and references |
| 2.2 | Import URDF robots into Isaac Sim and configure articulation physics |
| 2.3 | Configure RGB and depth cameras with RTX rendering |
| 2.4 | Configure LiDAR sensors with RTX ray-tracing |
| 2.5 | Configure IMU sensors with realistic noise parameters |

---

## From SDF to USD: Composition Paradigms

In Module 3, you used SDF's `<include>` tags for composition. USD provides a more powerful model.

### USD Composition Operations

```
USD Composition Stack (strongest to weakest):
1. Local opinions        ← Edits in the current layer
2. Sublayers            ← Layers stacked on top
3. References           ← Include other USD files
4. Payloads             ← Lazy-loaded references
5. Variants             ← Switchable alternatives
6. Inherits             ← Class-like inheritance
7. Specializes          ← Derived types
```

### Practical Composition Example

```python
# Python USD composition in Isaac Sim
from pxr import Usd, UsdGeom, Sdf

# Create new stage
stage = Usd.Stage.CreateNew("warehouse_scene.usd")

# Set up scene
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Add reference to robot
robot_prim = stage.DefinePrim("/World/Robot")
robot_prim.GetReferences().AddReference("./robot.usd")

# Add reference to environment
warehouse_prim = stage.DefinePrim("/World/Warehouse")
warehouse_prim.GetReferences().AddReference("./warehouse_env.usd")

# Override robot position
xform = UsdGeom.Xformable(robot_prim)
xform.AddTranslateOp().Set((5.0, 3.0, 0.0))

stage.Save()
```

---

## URDF Import {#urdf-import}

### Import Process

Isaac Sim can import your Module 2 URDF files:

1. **File → Import → URDF**
2. **Select URDF file**
3. **Configure import settings**:
   - Fix base link: Yes (for manipulation), No (for mobile)
   - Self collision: Enable
   - Create physics: Enable
   - Joint drive type: Position/Velocity

### Import Settings

```python
# Programmatic URDF import
from omni.isaac.urdf import _urdf

urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False  # Mobile robot
import_config.import_inertia_tensor = True
import_config.self_collision = True
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

# Import the robot
result = urdf_interface.parse_urdf(
    "robot.urdf",
    import_config
)
robot_path = urdf_interface.import_robot(
    "/World/Robot",
    result,
    import_config
)
```

### Verifying Import

After import, verify:

1. **Joint limits match URDF**: Property panel → Articulation → Joint limits
2. **Mass/inertia correct**: Property panel → Rigid Body → Mass
3. **Collision geometry**: Viewport → Show → Physics Collision

---

## RTX Camera Configuration {#camera}

### Creating a Camera

```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

# Create camera sensor
camera = Camera(
    prim_path="/World/Robot/camera_link/camera",
    resolution=(640, 480),
    frequency=30,
    translation=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (w,x,y,z)
)

# Initialize
camera.initialize()
```

### Camera Parameters

Matching Intel RealSense D435 from Module 3:

```python
# Configure camera to match hardware specs
camera_config = {
    "resolution": (1920, 1080),  # Or (640, 480) for performance
    "frequency": 30,
    "focal_length": 1.93,  # mm, from datasheet
    "horizontal_aperture": 3.6,  # mm (sensor width)
    "vertical_aperture": 2.4,  # mm (sensor height)
    "clipping_range": (0.1, 10.0),  # meters
}

# Resulting FOV calculation:
# HFOV = 2 * atan(horizontal_aperture / (2 * focal_length))
# HFOV = 2 * atan(3.6 / (2 * 1.93)) = 86.4° (matches D435 ~87°)
```

### Depth Camera Configuration

```python
from omni.isaac.sensor import Camera

# RGB-D camera (like RealSense)
rgbd_camera = Camera(
    prim_path="/World/Robot/camera_link/rgbd_camera",
    resolution=(640, 480),
    frequency=30
)

# Enable depth output
rgbd_camera.add_distance_to_camera_output()
rgbd_camera.add_distance_to_image_plane_output()

# Get depth data
depth_image = rgbd_camera.get_depth_image()
```

### Camera Noise

RTX cameras support physically-based noise:

```python
# Add post-processing effects for realism
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Motion blur (from robot motion)
# Enabled in render settings

# Lens distortion
# Configurable in camera intrinsics

# Exposure/gain noise
# Add in Replicator randomization (Chapter 5)
```

---

## RTX LiDAR Configuration {#lidar}

### RTX LiDAR Advantages

| Aspect | Gazebo LiDAR | Isaac RTX LiDAR |
|--------|--------------|-----------------|
| Method | CPU ray-casting | GPU ray-tracing |
| Speed | Limited rays | Millions of rays |
| Accuracy | Geometry only | Material properties |
| Multi-path | Not modeled | Partial support |

### Configuration

```python
from omni.isaac.sensor import LidarRtx

# Create RTX LiDAR (Velodyne VLP-16 equivalent)
lidar = LidarRtx(
    prim_path="/World/Robot/lidar_link/lidar"
)

# Configure to match VLP-16 specifications
lidar.set_fov([360.0, 30.0])  # Horizontal, Vertical FOV
lidar.set_resolution([0.4, 2.0])  # Horizontal, Vertical resolution (degrees)
lidar.set_valid_range([0.5, 100.0])  # Min, Max range (meters)
lidar.set_rotation_frequency(10.0)  # Hz

lidar.initialize()
```

### LiDAR Noise Configuration

```python
# Enable noise model
lidar_config = {
    "min_range": 0.5,
    "max_range": 100.0,
    "noise_type": "gaussian",
    "noise_stddev": 0.02,  # 2cm range noise (from M3 datasheet)
    "dropout_rate": 0.01,  # 1% dropout
}
```

### Getting LiDAR Data

```python
# Get point cloud
point_cloud = lidar.get_current_frame()

# Data format: (N, 3) array of XYZ points
# In robot's lidar_link frame

# Get as ROS message
from omni.isaac.ros2_bridge import LaserScanPublisher
lidar_publisher = LaserScanPublisher(
    lidar_prim_path="/World/Robot/lidar_link/lidar",
    topic_name="/scan",
    frame_id="lidar_link"
)
```

---

## IMU Configuration {#imu}

### Creating IMU Sensor

```python
from omni.isaac.sensor import Imu

# Create IMU sensor
imu = Imu(
    prim_path="/World/Robot/imu_link/imu",
    translation=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0])
)

imu.initialize()
```

### IMU Noise Parameters

From Module 1 MPU-6050 analysis:

```python
# Configure IMU noise to match MEMS specifications
imu_config = {
    # Accelerometer
    "accel_noise_density": 0.0004,  # g/√Hz (400 µg/√Hz)
    "accel_bias_instability": 0.00004,  # g (40 µg)

    # Gyroscope
    "gyro_noise_density": 0.005,  # °/s/√Hz
    "gyro_bias_instability": 5.0,  # °/hr

    # Sample rate
    "frequency": 100.0,  # Hz
}

# At 100 Hz:
# Accel noise = 0.0004 * sqrt(100) * 9.81 = 0.039 m/s² (matches M3)
# Gyro noise = 0.005 * sqrt(100) * π/180 = 0.00087 rad/s (matches M3)
```

### Getting IMU Data

```python
# Get IMU reading
imu_data = imu.get_current_frame()

# Returns:
# - linear_acceleration: (3,) array [m/s²]
# - angular_velocity: (3,) array [rad/s]
# - orientation: (4,) quaternion (if available)

# Publish to ROS
from omni.isaac.ros2_bridge import ImuPublisher
imu_publisher = ImuPublisher(
    imu_prim_path="/World/Robot/imu_link/imu",
    topic_name="/imu/data",
    frame_id="imu_link"
)
```

---

## Ground Truth Extraction {#ground-truth}

Isaac Sim can provide perfect ground truth for validation:

### Pose Ground Truth

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage

# Get exact robot pose (no noise)
stage = get_current_stage()
robot_prim = stage.GetPrimAtPath("/World/Robot")

# Get transform
from pxr import UsdGeom
xformable = UsdGeom.Xformable(robot_prim)
transform = xformable.ComputeLocalToWorldTransform(0)

# Extract position and orientation
# This is ground truth - use for VSLAM evaluation (Chapter 4)
```

### Sensor Ground Truth

```python
# Get perfect depth (no noise)
camera.add_distance_to_camera_output()
perfect_depth = camera.get_depth_image()  # Before noise is applied

# Compare to noisy output for validation
```

### Object Annotations

```python
# Get semantic labels for synthetic data
from omni.isaac.synthetic_utils import SyntheticDataHelper

sd_helper = SyntheticDataHelper()

# Get instance segmentation
instance_seg = sd_helper.get_instance_segmentation()

# Get bounding boxes
bboxes = sd_helper.get_bounding_boxes_2d()

# Get class labels
labels = sd_helper.get_semantic_segmentation()
```

---

## Complete Sensor Suite Example {#complete-example}

```python
"""Configure complete sensor suite for warehouse robot."""

import numpy as np
from omni.isaac.core import World
from omni.isaac.sensor import Camera, LidarRtx, Imu
from omni.isaac.ros2_bridge import (
    ImagePublisher, LaserScanPublisher, ImuPublisher
)

class SensorSuite:
    def __init__(self, robot_path: str):
        self.robot_path = robot_path

    def setup_sensors(self):
        # RGB-D Camera (RealSense D435)
        self.camera = Camera(
            prim_path=f"{self.robot_path}/camera_link/camera",
            resolution=(640, 480),
            frequency=30
        )
        self.camera.add_distance_to_camera_output()
        self.camera.initialize()

        # LiDAR (VLP-16)
        self.lidar = LidarRtx(
            prim_path=f"{self.robot_path}/lidar_link/lidar"
        )
        self.lidar.set_fov([360.0, 30.0])
        self.lidar.set_resolution([0.4, 2.0])
        self.lidar.set_valid_range([0.5, 100.0])
        self.lidar.set_rotation_frequency(10.0)
        self.lidar.initialize()

        # IMU (MPU-6050)
        self.imu = Imu(
            prim_path=f"{self.robot_path}/imu_link/imu"
        )
        self.imu.initialize()

    def setup_ros_publishers(self):
        # Image publisher
        self.image_pub = ImagePublisher(
            camera_prim_path=f"{self.robot_path}/camera_link/camera",
            topic_name="/camera/image_raw",
            frame_id="camera_link"
        )

        # Depth publisher
        self.depth_pub = ImagePublisher(
            camera_prim_path=f"{self.robot_path}/camera_link/camera",
            topic_name="/camera/depth/image_raw",
            frame_id="camera_link",
            image_type="depth"
        )

        # LiDAR publisher
        self.lidar_pub = LaserScanPublisher(
            lidar_prim_path=f"{self.robot_path}/lidar_link/lidar",
            topic_name="/scan",
            frame_id="lidar_link"
        )

        # IMU publisher
        self.imu_pub = ImuPublisher(
            imu_prim_path=f"{self.robot_path}/imu_link/imu",
            topic_name="/imu/data",
            frame_id="imu_link"
        )

# Usage
sensor_suite = SensorSuite("/World/Robot")
sensor_suite.setup_sensors()
sensor_suite.setup_ros_publishers()
```

---

## Summary

This chapter covered Isaac Sim scene composition and sensor configuration:

1. **USD composition** provides layers, references, and variants for modular scenes—more powerful than SDF includes.

2. **URDF import** brings Module 2 robots into Isaac with articulation physics.

3. **RTX cameras** use physically-based rendering; configure focal length and aperture to match real camera specs.

4. **RTX LiDAR** uses GPU ray-tracing for faster, more accurate ranging than Gazebo.

5. **IMU sensors** configure with noise parameters from Module 1 datasheets.

6. **Ground truth** enables validation and synthetic data generation.

---

## Reality Gap Callout

Even with RTX, gaps remain:

| Sensor | RTX Improvement | Remaining Gap |
|--------|----------------|---------------|
| Camera | Photorealistic materials | Some lighting, no lens dust |
| LiDAR | Better ray-tracing | Multi-path simplified |
| IMU | Configurable noise | Non-Gaussian outliers |
| Depth | Better accuracy | Edge artifacts differ |

Use domain randomization (Chapter 5) to address remaining gaps.

---

## Self-Assessment Questions

1. **USD Composition**: You have a warehouse.usd and three different robot.usd files. How would you create a scene that can switch between robots without duplicating the warehouse?

2. **URDF Import**: Your imported robot floats 10cm above the ground. What import setting likely caused this?

3. **Camera Matching**: Your real camera has 90° HFOV. The Isaac camera shows 60°. What parameters need adjustment?

4. **LiDAR Comparison**: Compare the LiDAR configuration here to your Module 3 Gazebo config. What's different?

5. **Ground Truth Use**: You're evaluating a depth estimation neural network. How would you use Isaac's ground truth depth for validation?

---

## What's Next

In [Chapter 3: Isaac ROS Integration](/module-4/chapter-3-isaac-ros-integration), you'll connect Isaac Sim to ROS 2 using Isaac ROS packages for GPU-accelerated perception pipelines.
