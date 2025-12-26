---
id: chapter-2-robot-setup
title: "Chapter 2: Humanoid Robot Setup"
sidebar_label: "2. Robot Setup"
sidebar_position: 3
---

# Chapter 2: Humanoid Robot Setup in Isaac Sim

## Chapter Goal

By the end of this chapter, you will have **configured a complete humanoid robot in Isaac Sim** with mobile base, dual arms, grippers, and full sensor suite, ready for VLA-controlled operation.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 2.1 | Create a humanoid URDF with mobile base and dual arms |
| 2.2 | Import and configure the robot in Isaac Sim |
| 2.3 | Set up camera, LiDAR, and IMU sensors |
| 2.4 | Configure Isaac ROS perception pipelines |
| 2.5 | Verify sensor data flow and robot control |

---

## Robot Design Overview {#overview}

### Humanoid Configuration

```
                    ┌─────────┐
                    │  Head   │  ← Stereo cameras + RGB-D
                    │ (Cameras)│
                    └────┬────┘
                         │
              ┌──────────┴──────────┐
              │                     │
         ┌────┴────┐          ┌────┴────┐
         │  Left   │          │  Right  │
         │   Arm   │          │   Arm   │
         │ (7-DOF) │          │ (7-DOF) │
         └────┬────┘          └────┬────┘
              │                    │
         ┌────┴────┐          ┌────┴────┐
         │ Gripper │          │ Gripper │
         │ (2-DOF) │          │ (2-DOF) │
         └─────────┘          └─────────┘

                    ┌─────────┐
                    │  Torso  │  ← IMU
                    │         │
                    └────┬────┘
                         │
                    ┌────┴────┐
                    │ Mobile  │  ← LiDAR, Wheel encoders
                    │  Base   │
                    │(Diff/Omni)│
                    └─────────┘
```

### Specifications

| Component | Configuration |
|-----------|--------------|
| Mobile base | Differential drive or holonomic |
| Arms | 2x 7-DOF manipulators |
| Grippers | 2x parallel jaw (2-DOF each) |
| Head | Pan-tilt (2-DOF) |
| Cameras | Stereo + RGB-D |
| LiDAR | 2D or 3D |
| IMU | 6-DOF |

---

## URDF Construction {#urdf}

### Base Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Include component xacros -->
  <xacro:include filename="$(find humanoid_description)/urdf/mobile_base.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/torso.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/arm.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/gripper.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/head.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/sensors.xacro"/>

  <!-- Base link -->
  <link name="base_footprint"/>

  <!-- Mobile base -->
  <xacro:mobile_base parent="base_footprint"/>

  <!-- Torso -->
  <xacro:torso parent="base_link"/>

  <!-- Arms -->
  <xacro:arm prefix="left" parent="torso_link" reflect="1"/>
  <xacro:arm prefix="right" parent="torso_link" reflect="-1"/>

  <!-- Grippers -->
  <xacro:gripper prefix="left" parent="left_arm_link_7"/>
  <xacro:gripper prefix="right" parent="right_arm_link_7"/>

  <!-- Head with cameras -->
  <xacro:head parent="torso_link"/>

  <!-- Sensors -->
  <xacro:lidar parent="base_link"/>
  <xacro:imu parent="torso_link"/>

</robot>
```

### Mobile Base Xacro

```xml
<!-- mobile_base.xacro -->
<xacro:macro name="mobile_base" params="parent">

  <!-- Base link -->
  <joint name="base_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="30.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.8"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right wheel (similar) -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.25 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <link name="right_wheel">
    <!-- Similar to left wheel -->
  </link>

  <!-- Caster wheel -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
  </link>

</xacro:macro>
```

### 7-DOF Arm Xacro

```xml
<!-- arm.xacro -->
<xacro:macro name="arm" params="prefix parent reflect">

  <!-- Shoulder pitch -->
  <joint name="${prefix}_arm_joint_1" type="revolute">
    <parent link="${parent}"/>
    <child link="${prefix}_arm_link_1"/>
    <origin xyz="0 ${reflect * 0.2} 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="${prefix}_arm_link_1">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Shoulder roll -->
  <joint name="${prefix}_arm_joint_2" type="revolute">
    <parent link="${prefix}_arm_link_1"/>
    <child link="${prefix}_arm_link_2"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="2.0"/>
  </joint>

  <!-- Continue for joints 3-7... -->
  <!-- Joint 3: Shoulder yaw -->
  <!-- Joint 4: Elbow pitch -->
  <!-- Joint 5: Wrist roll -->
  <!-- Joint 6: Wrist pitch -->
  <!-- Joint 7: Wrist yaw -->

  <!-- End effector link -->
  <joint name="${prefix}_arm_ee_joint" type="fixed">
    <parent link="${prefix}_arm_link_7"/>
    <child link="${prefix}_arm_ee_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="${prefix}_arm_ee_link"/>

</xacro:macro>
```

### Sensor Xacros

```xml
<!-- sensors.xacro -->

<!-- RGB-D Camera -->
<xacro:macro name="rgbd_camera" params="prefix parent">
  <joint name="${prefix}_camera_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${prefix}_camera_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <link name="${prefix}_camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
    </visual>
  </link>

  <!-- Optical frame (Z forward, X right, Y down) -->
  <joint name="${prefix}_camera_optical_joint" type="fixed">
    <parent link="${prefix}_camera_link"/>
    <child link="${prefix}_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <link name="${prefix}_camera_optical_frame"/>

  <!-- Gazebo/Isaac sensor plugin defined separately -->
</xacro:macro>

<!-- LiDAR -->
<xacro:macro name="lidar" params="parent">
  <joint name="lidar_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- IMU -->
<xacro:macro name="imu" params="parent">
  <joint name="imu_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>
</xacro:macro>
```

---

## Isaac Sim Import {#isaac-import}

### USD Scene Setup

```python
#!/usr/bin/env python3
"""Set up humanoid robot in Isaac Sim."""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.sensor import Camera, LidarRtx, IMUSensor
import omni.isaac.core.utils.prims as prim_utils


def setup_humanoid_scene():
    """Create Isaac Sim scene with humanoid robot."""

    # Create world
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_default_ground_plane()

    # Import robot from URDF
    # Note: URDF should be converted to USD first using URDF Importer
    robot_usd_path = "/path/to/humanoid.usd"
    robot_prim_path = "/World/humanoid"

    add_reference_to_stage(robot_usd_path, robot_prim_path)

    # Create robot object
    robot = world.scene.add(Robot(
        prim_path=robot_prim_path,
        name="humanoid",
        position=[0, 0, 0.1],
        orientation=[1, 0, 0, 0]
    ))

    # Add environment objects
    add_table(world)
    add_objects(world)

    return world, robot


def add_table(world):
    """Add table to scene."""
    from omni.isaac.core.prims import GeometryPrim

    table = GeometryPrim(
        prim_path="/World/table",
        name="table",
        position=[0.8, 0, 0],
        scale=[1.0, 0.6, 0.8]
    )

    # Load table mesh
    prim_utils.create_prim(
        "/World/table",
        "Cube",
        position=[0.8, 0, 0.4],
        scale=[0.6, 0.4, 0.01]  # Table top
    )


def add_objects(world):
    """Add manipulable objects."""
    objects = [
        {"name": "red_mug", "pos": [0.7, 0.1, 0.85], "color": [0.8, 0.1, 0.1]},
        {"name": "blue_box", "pos": [0.8, -0.1, 0.85], "color": [0.1, 0.1, 0.8]},
        {"name": "green_bottle", "pos": [0.9, 0.0, 0.85], "color": [0.1, 0.8, 0.1]},
    ]

    for obj in objects:
        prim_utils.create_prim(
            f"/World/{obj['name']}",
            "Cylinder",
            position=obj['pos'],
            scale=[0.04, 0.04, 0.1]
        )
        # Apply material with color


if __name__ == "__main__":
    world, robot = setup_humanoid_scene()
    world.reset()

    # Run simulation
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()
```

### URDF to USD Conversion

```python
from omni.isaac.urdf import _urdf

def convert_urdf_to_usd():
    """Convert humanoid URDF to USD format."""

    urdf_interface = _urdf.acquire_urdf_interface()

    # Configure import settings
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.density = 1000.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION

    # Convert
    result = urdf_interface.parse_urdf(
        "/path/to/humanoid.urdf",
        import_config
    )

    urdf_interface.import_robot(
        "/path/to/humanoid.urdf",
        result,
        import_config,
        "/World/humanoid"
    )
```

---

## Sensor Configuration {#sensors}

### Camera Setup

```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

def setup_cameras(robot_prim_path: str):
    """Configure cameras on humanoid head."""

    # RGB-D Camera (head mounted)
    head_camera = Camera(
        prim_path=f"{robot_prim_path}/head_link/rgbd_camera",
        frequency=30,
        resolution=(640, 480)
    )

    # Configure camera properties
    head_camera.set_focal_length(1.93)  # mm
    head_camera.set_focus_distance(0.5)  # m
    head_camera.set_horizontal_aperture(2.65)

    # Enable depth
    head_camera.add_depth_to_frame()

    # Stereo cameras for VSLAM
    left_stereo = Camera(
        prim_path=f"{robot_prim_path}/head_link/left_camera",
        frequency=30,
        resolution=(640, 480)
    )

    right_stereo = Camera(
        prim_path=f"{robot_prim_path}/head_link/right_camera",
        frequency=30,
        resolution=(640, 480)
    )

    return head_camera, left_stereo, right_stereo


def create_ros2_camera_publishers(cameras):
    """Create ROS 2 publishers for camera data."""
    import omni.isaac.ros2_bridge as ros2_bridge

    # RGB publisher
    rgb_pub = ros2_bridge.create_camera_publisher(
        cameras[0],
        topic="/camera/rgb",
        frame_id="head_camera_optical_frame"
    )

    # Depth publisher
    depth_pub = ros2_bridge.create_depth_publisher(
        cameras[0],
        topic="/camera/depth",
        frame_id="head_camera_optical_frame"
    )

    # Stereo publishers
    left_pub = ros2_bridge.create_camera_publisher(
        cameras[1],
        topic="/camera/left/image",
        frame_id="left_camera_optical_frame"
    )

    right_pub = ros2_bridge.create_camera_publisher(
        cameras[2],
        topic="/camera/right/image",
        frame_id="right_camera_optical_frame"
    )

    return [rgb_pub, depth_pub, left_pub, right_pub]
```

### LiDAR Setup

```python
from omni.isaac.sensor import LidarRtx

def setup_lidar(robot_prim_path: str):
    """Configure LiDAR sensor."""

    lidar = LidarRtx(
        prim_path=f"{robot_prim_path}/base_link/lidar",
        name="base_lidar",
        rotation_rate=20.0,  # Hz
        high_lod=True,
        horizontal_fov=360.0,
        horizontal_resolution=0.4,
        vertical_fov=30.0,
        vertical_resolution=4.0,
        min_range=0.1,
        max_range=100.0
    )

    return lidar


def create_ros2_lidar_publisher(lidar):
    """Create ROS 2 publisher for LiDAR data."""
    import omni.isaac.ros2_bridge as ros2_bridge

    scan_pub = ros2_bridge.create_laser_scan_publisher(
        lidar,
        topic="/scan",
        frame_id="lidar_link"
    )

    pointcloud_pub = ros2_bridge.create_point_cloud_publisher(
        lidar,
        topic="/points",
        frame_id="lidar_link"
    )

    return scan_pub, pointcloud_pub
```

### IMU Setup

```python
from omni.isaac.sensor import IMUSensor

def setup_imu(robot_prim_path: str):
    """Configure IMU sensor."""

    imu = IMUSensor(
        prim_path=f"{robot_prim_path}/torso_link/imu",
        name="torso_imu",
        frequency=200,  # Hz
        linear_acceleration_filter_size=10,
        angular_velocity_filter_size=10,
        orientation_filter_size=10
    )

    return imu


def create_ros2_imu_publisher(imu):
    """Create ROS 2 publisher for IMU data."""
    import omni.isaac.ros2_bridge as ros2_bridge

    imu_pub = ros2_bridge.create_imu_publisher(
        imu,
        topic="/imu",
        frame_id="imu_link"
    )

    return imu_pub
```

---

## Isaac ROS Integration {#isaac-ros}

### Perception Pipeline Launch

```python
# launch/perception_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Visual SLAM
        ComposableNodeContainer(
            name='visual_slam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam_node',
                    parameters=[{
                        'enable_imu_fusion': True,
                        'gyro_noise_density': 0.00016,
                        'gyro_random_walk': 0.000022,
                        'accel_noise_density': 0.0017,
                        'accel_random_walk': 0.00019,
                        'calibration_frequency': 200.0,
                    }],
                    remappings=[
                        ('stereo_camera/left/image', '/camera/left/image'),
                        ('stereo_camera/right/image', '/camera/right/image'),
                        ('visual_slam/imu', '/imu'),
                    ]
                ),
            ],
        ),

        # Object Detection
        ComposableNodeContainer(
            name='detection_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Image encoding for detection
                ComposableNode(
                    package='isaac_ros_dnn_image_encoder',
                    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                    name='dnn_image_encoder',
                    parameters=[{
                        'input_image_width': 640,
                        'input_image_height': 480,
                        'network_image_width': 640,
                        'network_image_height': 480,
                    }],
                    remappings=[
                        ('image', '/camera/rgb'),
                    ]
                ),

                # Detection inference
                ComposableNode(
                    package='isaac_ros_detectnet',
                    plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
                    name='detectnet_decoder',
                    parameters=[{
                        'label_list': ['mug', 'box', 'bottle', 'book'],
                        'confidence_threshold': 0.5,
                    }]
                ),
            ],
        ),
    ])
```

### Robot Control Interface

```python
#!/usr/bin/env python3
"""ROS 2 control interface for humanoid in Isaac Sim."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class HumanoidController(Node):
    """Control interface for humanoid robot."""

    def __init__(self):
        super().__init__('humanoid_controller')

        # Joint names
        self.arm_joints = {
            'left': [f'left_arm_joint_{i}' for i in range(1, 8)],
            'right': [f'right_arm_joint_{i}' for i in range(1, 8)]
        }
        self.gripper_joints = {
            'left': ['left_gripper_joint_1', 'left_gripper_joint_2'],
            'right': ['right_gripper_joint_1', 'right_gripper_joint_2']
        }

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.left_arm_pub = self.create_publisher(
            JointTrajectory, '/left_arm_controller/joint_trajectory', 10
        )
        self.right_arm_pub = self.create_publisher(
            JointTrajectory, '/right_arm_controller/joint_trajectory', 10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.current_joint_state = None

    def joint_state_callback(self, msg: JointState):
        """Store current joint states."""
        self.current_joint_state = msg

    def move_base(self, linear_x: float, angular_z: float):
        """Send velocity command to mobile base."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def move_arm(
        self,
        arm: str,
        positions: list,
        duration: float = 2.0
    ):
        """Send trajectory to arm."""
        if arm not in ['left', 'right']:
            raise ValueError("arm must be 'left' or 'right'")

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints[arm]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        traj.points = [point]

        if arm == 'left':
            self.left_arm_pub.publish(traj)
        else:
            self.right_arm_pub.publish(traj)

    def get_arm_positions(self, arm: str) -> list:
        """Get current arm joint positions."""
        if self.current_joint_state is None:
            return None

        joint_names = self.arm_joints[arm]
        positions = []

        for name in joint_names:
            if name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(name)
                positions.append(self.current_joint_state.position[idx])

        return positions if len(positions) == 7 else None
```

---

## Verification Tests {#verification}

### Sensor Data Flow Test

```python
#!/usr/bin/env python3
"""Verify sensor data is flowing correctly."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, JointState
from nav_msgs.msg import Odometry


class SensorVerificationNode(Node):
    """Verify all sensor topics are publishing."""

    def __init__(self):
        super().__init__('sensor_verification')

        self.received = {
            'rgb': False,
            'depth': False,
            'scan': False,
            'imu': False,
            'joint_states': False,
            'odom': False,
        }

        # Subscribers
        self.create_subscription(Image, '/camera/rgb', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Verification timer
        self.create_timer(1.0, self.verify_callback)

    def rgb_callback(self, msg): self.received['rgb'] = True
    def depth_callback(self, msg): self.received['depth'] = True
    def scan_callback(self, msg): self.received['scan'] = True
    def imu_callback(self, msg): self.received['imu'] = True
    def joint_callback(self, msg): self.received['joint_states'] = True
    def odom_callback(self, msg): self.received['odom'] = True

    def verify_callback(self):
        """Report sensor status."""
        all_received = all(self.received.values())

        self.get_logger().info("Sensor Status:")
        for sensor, received in self.received.items():
            status = "OK" if received else "MISSING"
            self.get_logger().info(f"  {sensor}: {status}")

        if all_received:
            self.get_logger().info("All sensors operational!")
        else:
            missing = [s for s, r in self.received.items() if not r]
            self.get_logger().warn(f"Missing sensors: {missing}")


def main():
    rclpy.init()
    node = SensorVerificationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Robot Control Test

```python
#!/usr/bin/env python3
"""Test robot control interfaces."""

import rclpy
from rclpy.node import Node
import time


class ControlTestNode(Node):
    """Test robot control functionality."""

    def __init__(self):
        super().__init__('control_test')

        self.controller = HumanoidController()
        self.test_timer = self.create_timer(2.0, self.run_tests)
        self.test_phase = 0

    def run_tests(self):
        """Run through control tests."""
        if self.test_phase == 0:
            self.get_logger().info("Test 1: Base forward motion")
            self.controller.move_base(0.1, 0.0)
            self.test_phase = 1

        elif self.test_phase == 1:
            self.get_logger().info("Test 2: Base rotation")
            self.controller.move_base(0.0, 0.3)
            self.test_phase = 2

        elif self.test_phase == 2:
            self.get_logger().info("Test 3: Stop base")
            self.controller.move_base(0.0, 0.0)
            self.test_phase = 3

        elif self.test_phase == 3:
            self.get_logger().info("Test 4: Left arm home position")
            home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.controller.move_arm('left', home)
            self.test_phase = 4

        elif self.test_phase == 4:
            self.get_logger().info("Test 5: Left arm extended")
            extended = [0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0]
            self.controller.move_arm('left', extended)
            self.test_phase = 5

        elif self.test_phase == 5:
            self.get_logger().info("All tests complete!")
            self.test_timer.cancel()
```

---

## Summary

This chapter covered humanoid robot setup in Isaac Sim:

1. **URDF construction** with mobile base, dual 7-DOF arms, grippers, and sensors.

2. **Isaac Sim import** using URDF-to-USD conversion and scene configuration.

3. **Sensor setup** including RGB-D cameras, stereo cameras, LiDAR, and IMU.

4. **Isaac ROS integration** for perception pipelines and robot control.

5. **Verification tests** confirm sensor data flow and control interfaces.

---

## Setup Checklist

Before proceeding, verify:

- [ ] Humanoid URDF builds without errors
- [ ] Robot imports correctly in Isaac Sim
- [ ] All sensors publishing data at expected rates
- [ ] Joint state feedback received
- [ ] Base velocity commands move the robot
- [ ] Arm trajectory commands execute smoothly

---

## What's Next

In [Chapter 3: Navigation System](/capstone/chapter-3-navigation), you'll implement VSLAM-based navigation with obstacle avoidance using the configured sensors.
