---
id: chapter-4-simulated-actuators
title: "Chapter 4: Simulated Actuators"
sidebar_label: "4. Simulated Actuators"
sidebar_position: 5
---

# Chapter 4: Simulated Actuators and Control Interfaces

## Chapter Goal

By the end of this chapter, you will be able to **configure simulated actuators with realistic response characteristics including delay, saturation, friction, and backlash**, and implement control interfaces through ROS 2.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Spawn a robot from URDF in Gazebo with physics properties correctly configured |
| 4.2 | Configure joint controllers (position, velocity, effort) with appropriate PID gains |
| 4.3 | Add actuator delay simulating motor time constants |
| 4.4 | Configure joint friction and damping that affects motion under load |
| 4.5 | Implement differential drive controller for mobile robots |
| 4.6 | Implement joint trajectory controller for manipulators |

---

## Why Actuator Modeling Matters

Consider this scenario: You tune a PID controller in simulation and achieve perfect tracking. On real hardware, the arm oscillates and overshoots.

What happened? The simulated actuator was too ideal:
- **No delay**: Real motors have electrical and mechanical time constants
- **No saturation**: Real motors have torque/speed limits
- **No friction**: Real joints have static and dynamic friction

This chapter makes simulated actuators behave more like real hardware.

---

## Spawning Robots from URDF {#spawn-urdf}

### URDF to Gazebo

Your URDF from Module 2 needs additional elements for Gazebo:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Standard URDF elements -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <!-- ros2_control for joint control -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

### Spawning in Gazebo

```python
# launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws/src/my_robot/urdf/robot.urdf'
    )

    return LaunchDescription([
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', urdf_path,
                '-name', 'my_robot',
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        ),
    ])
```

### Verifying Physics Properties

After spawning, verify in Gazebo GUI:
1. Select model → Right-click → View → Inertia
2. Check that inertia ellipsoids match expected mass distribution
3. Verify collision geometry covers visual geometry

---

## Joint Controllers {#joint-controllers}

### Control Modes

| Mode | Command | Use Case |
|------|---------|----------|
| Position | Target angle (rad) | Precise positioning |
| Velocity | Target speed (rad/s) | Continuous rotation |
| Effort | Torque (Nm) | Direct force control |

### ros2_control Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - wrist_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # PID gains (critical for realistic behavior)
    gains:
      shoulder_joint:
        p: 100.0
        i: 0.0
        d: 10.0
      elbow_joint:
        p: 80.0
        i: 0.0
        d: 8.0
      wrist_joint:
        p: 50.0
        i: 0.0
        d: 5.0
```

### PID Tuning Methodology

**Physical Grounding**: PID gains depend on joint inertia from Module 1:

```
Approximate starting gains:
  P ≈ inertia × (desired_bandwidth)²
  D ≈ 2 × sqrt(P × inertia) × damping_ratio
  I = 0 initially (add if steady-state error)

Example for joint with inertia = 0.1 kg·m², bandwidth = 10 rad/s:
  P = 0.1 × 10² = 10
  D = 2 × sqrt(10 × 0.1) × 0.7 = 1.4
```

### Tuning Process

```bash
# 1. Start with low gains
P=10, I=0, D=0

# 2. Increase P until oscillation begins
# 3. Back off P by 50%
# 4. Add D to reduce overshoot
# 5. Add small I if steady-state error exists

# Monitor response
ros2 topic echo /joint_states
```

---

## Modeling Actuator Dynamics {#actuator-dynamics}

### Time Constant (Delay)

Real actuators don't respond instantly. Model the lag:

```xml
<!-- In URDF gazebo extension -->
<transmission name="shoulder_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_motor">
    <mechanicalReduction>100</mechanicalReduction>
  </actuator>
</transmission>

<!-- In Gazebo plugin configuration -->
<plugin name="gz_ros2_control" filename="libgz_ros2_control-system.so">
  <parameters>
    <joint name="shoulder_joint">
      <motor_model>
        <time_constant>0.05</time_constant>  <!-- 50ms response -->
      </motor_model>
    </joint>
  </parameters>
</plugin>
```

**Physical Grounding**: Motor time constant from Module 1:

| Motor Type | Typical Time Constant |
|------------|----------------------|
| Small servo | 20-50 ms |
| Industrial servo | 5-20 ms |
| Geared DC motor | 50-200 ms |
| Stepper | Near-instant (but discrete) |

### Effort Saturation

Real motors have torque limits:

```yaml
# In ros2_control config
arm_controller:
  ros__parameters:
    joints:
      - shoulder_joint

    # Effort limits
    constraints:
      shoulder_joint:
        min_effort: -10.0  # Nm
        max_effort: 10.0   # Nm
```

In URDF:
```xml
<joint name="shoulder_joint" type="revolute">
  <limit
    lower="-3.14"
    upper="3.14"
    velocity="2.0"      <!-- rad/s max -->
    effort="10.0"       <!-- Nm max torque -->
  />
</joint>
```

### Velocity Saturation

```yaml
arm_controller:
  ros__parameters:
    constraints:
      shoulder_joint:
        max_velocity: 2.0  # rad/s
```

---

## Friction and Damping {#friction}

### Joint Dynamics in URDF

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" velocity="2.0" effort="10.0"/>

  <!-- Friction and damping -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Friction Types

| Type | URDF Element | Physical Meaning |
|------|--------------|------------------|
| Viscous damping | `damping` | Resistance proportional to velocity |
| Coulomb friction | `friction` | Constant resistance to motion |

**Physical Grounding**:

```
Joint torque required = τ_command - damping × velocity - friction × sign(velocity)
```

### Configuring Realistic Friction

From Module 1 actuator analysis:

```xml
<!-- High-quality ball bearing joint -->
<dynamics damping="0.1" friction="0.01"/>

<!-- Gearbox with moderate friction -->
<dynamics damping="0.5" friction="0.2"/>

<!-- Worn or low-quality joint -->
<dynamics damping="1.0" friction="0.5"/>
```

---

## Differential Drive Controller {#diff-drive}

For mobile robots with two drive wheels:

### URDF Configuration

```xml
<ros2_control name="DiffDriveSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Controller Configuration

```yaml
# config/diff_drive.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    # Wheel joints
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    # Robot geometry
    wheel_separation: 0.4  # meters between wheels
    wheel_radius: 0.1      # wheel radius in meters

    # Velocity limits (from M1 actuator specs)
    linear.x.max_velocity: 1.0      # m/s
    linear.x.min_velocity: -0.5     # m/s
    angular.z.max_velocity: 2.0     # rad/s
    angular.z.min_velocity: -2.0    # rad/s

    # Acceleration limits
    linear.x.max_acceleration: 0.5  # m/s²
    angular.z.max_acceleration: 1.0 # rad/s²

    # Odometry
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
```

### Teleoperation Test

```bash
# Terminal 1: Run simulation with controller
ros2 launch my_robot gazebo.launch.py

# Terminal 2: Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Monitor odometry
ros2 topic echo /diff_drive_controller/odom
```

---

## Joint Trajectory Controller {#trajectory-controller}

For robot arms requiring smooth motion:

### Configuration

```yaml
arm_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - wrist_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # Trajectory constraints
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0  # 0 means no time constraint

      shoulder_joint:
        trajectory: 0.1  # position tolerance during motion
        goal: 0.01       # position tolerance at goal

      elbow_joint:
        trajectory: 0.1
        goal: 0.01

      wrist_joint:
        trajectory: 0.05
        goal: 0.005
```

### Sending Trajectory Commands

```python
#!/usr/bin/env python3
"""Send joint trajectory to arm controller."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory'
        )

    def send_trajectory(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        # Point 1: Home position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)

        # Point 2: Intermediate
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.3, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)

        # Point 3: Goal
        point3 = JointTrajectoryPoint()
        point3.positions = [1.0, 0.6, 0.2]
        point3.time_from_start = Duration(sec=4, nanosec=0)

        goal.trajectory.points = [point1, point2, point3]

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)

        return future


def main():
    rclpy.init()
    client = TrajectoryClient()
    future = client.send_trajectory()
    rclpy.spin_until_future_complete(client, future)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Complete Mobile Manipulator Example {#complete-example}

```yaml
# config/mobile_manipulator.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: position_controllers/GripperActionController

# Diff drive for base
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.5
    wheel_radius: 0.1
    linear.x.max_velocity: 1.0
    angular.z.max_velocity: 2.0

# Trajectory controller for arm
arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]

# Gripper controller
gripper_controller:
  ros__parameters:
    joint: gripper_joint
    max_effort: 50.0
    stall_velocity_threshold: 0.001
```

---

## Summary

This chapter covered configuring simulated actuators:

1. **URDF spawning** requires inertial properties and Gazebo-specific extensions.

2. **Joint controllers** (position, velocity, effort) need PID tuning based on joint inertia.

3. **Actuator delay** models motor time constants—critical for realistic control behavior.

4. **Friction and damping** affect motion under load; configure from actuator specifications.

5. **Diff drive controller** translates cmd_vel to wheel velocities for mobile robots.

6. **Trajectory controller** enables smooth multi-joint motion for manipulators.

---

## Reality Gap Callout

Simulated actuators differ from real actuators:

| Aspect | Simulation | Reality |
|--------|------------|---------|
| Electrical dynamics | Ignored | Motor inductance affects response |
| Gear backlash | Not modeled | Causes position uncertainty |
| Thermal effects | Ignored | Motors derate when hot |
| Wear | Static | Parameters change over time |

---

## Self-Assessment Questions

1. **PID Tuning**: Your simulated arm joint oscillates with P=100, I=0, D=0. What would you adjust first?

2. **Time Constant**: Your real motor has a 100ms time constant but simulation responds instantly. What parameter would you add?

3. **Saturation**: Your controller commands 20 Nm but the motor is rated for 10 Nm. What happens in simulation? On real hardware?

4. **Diff Drive**: Your robot curves left when commanded to go straight. What calibration parameter is likely wrong?

5. **Trajectory Execution**: A trajectory succeeds in simulation but fails on hardware with "goal tolerance violated". What might cause this?

---

## What's Next

In [Chapter 5: Unity Integration](/module-3/chapter-5-unity-integration), you'll learn to use Unity as an alternative simulation platform with ROS 2 integration—useful for photorealistic rendering and perception training.
