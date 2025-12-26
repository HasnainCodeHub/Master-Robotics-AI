---
id: chapter-4-manipulation
title: "Chapter 4: Manipulation System"
sidebar_label: "4. Manipulation"
sidebar_position: 5
---

# Chapter 4: Manipulation System Implementation

## Chapter Goal

By the end of this chapter, you will have **implemented a complete manipulation system** using MoveIt 2 for motion planning, with pick and place capabilities for both arms and coordinated bimanual operations.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Configure MoveIt 2 for dual-arm humanoid |
| 4.2 | Implement pick and place action servers |
| 4.3 | Integrate grasp planning with perception |
| 4.4 | Implement coordinated bimanual manipulation |
| 4.5 | Handle manipulation failures with recovery |

---

## MoveIt 2 Configuration {#moveit-config}

### SRDF Definition

```xml
<!-- humanoid.srdf -->
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- Planning groups -->
  <group name="left_arm">
    <chain base_link="torso_link" tip_link="left_arm_ee_link"/>
  </group>

  <group name="right_arm">
    <chain base_link="torso_link" tip_link="right_arm_ee_link"/>
  </group>

  <group name="left_gripper">
    <joint name="left_gripper_joint_1"/>
    <joint name="left_gripper_joint_2"/>
  </group>

  <group name="right_gripper">
    <joint name="right_gripper_joint_1"/>
    <joint name="right_gripper_joint_2"/>
  </group>

  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>

  <!-- End effectors -->
  <end_effector name="left_gripper" parent_link="left_arm_ee_link"
                group="left_gripper" parent_group="left_arm"/>
  <end_effector name="right_gripper" parent_link="right_arm_ee_link"
                group="right_gripper" parent_group="right_arm"/>

  <!-- Disabled collisions (adjacent links) -->
  <disable_collisions link1="torso_link" link2="left_arm_link_1" reason="Adjacent"/>
  <disable_collisions link1="left_arm_link_1" link2="left_arm_link_2" reason="Adjacent"/>
  <!-- ... more collision pairs ... -->

  <!-- Named poses -->
  <group_state name="home" group="left_arm">
    <joint name="left_arm_joint_1" value="0"/>
    <joint name="left_arm_joint_2" value="0"/>
    <joint name="left_arm_joint_3" value="0"/>
    <joint name="left_arm_joint_4" value="-1.57"/>
    <joint name="left_arm_joint_5" value="0"/>
    <joint name="left_arm_joint_6" value="1.57"/>
    <joint name="left_arm_joint_7" value="0"/>
  </group_state>

  <group_state name="home" group="right_arm">
    <joint name="right_arm_joint_1" value="0"/>
    <joint name="right_arm_joint_2" value="0"/>
    <joint name="right_arm_joint_3" value="0"/>
    <joint name="right_arm_joint_4" value="-1.57"/>
    <joint name="right_arm_joint_5" value="0"/>
    <joint name="right_arm_joint_6" value="1.57"/>
    <joint name="right_arm_joint_7" value="0"/>
  </group_state>

  <group_state name="open" group="left_gripper">
    <joint name="left_gripper_joint_1" value="0.04"/>
    <joint name="left_gripper_joint_2" value="0.04"/>
  </group_state>

  <group_state name="closed" group="left_gripper">
    <joint name="left_gripper_joint_1" value="0"/>
    <joint name="left_gripper_joint_2" value="0"/>
  </group_state>
</robot>
```

### MoveIt Configuration Files

```yaml
# config/moveit/kinematics.yaml

left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05

right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05

both_arms:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
```

```yaml
# config/moveit/ompl_planning.yaml

planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0

  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0

  PRMkConfigDefault:
    type: geometric::PRM
    max_nearest_neighbors: 10

left_arm:
  default_planner_config: RRTConnectkConfigDefault
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
  projection_evaluator: joints(left_arm_joint_1,left_arm_joint_2)
  longest_valid_segment_fraction: 0.005

right_arm:
  default_planner_config: RRTConnectkConfigDefault
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
  projection_evaluator: joints(right_arm_joint_1,right_arm_joint_2)
  longest_valid_segment_fraction: 0.005
```

```yaml
# config/moveit/joint_limits.yaml

joint_limits:
  left_arm_joint_1:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 3.0

  left_arm_joint_2:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 3.0

  # ... similar for other joints ...

  left_gripper_joint_1:
    has_velocity_limits: true
    max_velocity: 0.5
    has_acceleration_limits: true
    max_acceleration: 1.0
```

---

## MoveIt 2 Interface {#moveit-interface}

### Arm Controller

```python
#!/usr/bin/env python3
"""MoveIt 2 interface for humanoid arms."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
import numpy as np


class ArmController(Node):
    """Control humanoid arms via MoveIt 2."""

    def __init__(self):
        super().__init__('arm_controller')

        # Action clients
        self.move_group_clients = {
            'left_arm': ActionClient(self, MoveGroup, '/move_action'),
            'right_arm': ActionClient(self, MoveGroup, '/move_action'),
        }

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )

        # IK service
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik'
        )

        # Gripper publishers
        self.gripper_pubs = {
            'left': self.create_publisher(
                Float64MultiArray, '/left_gripper_controller/commands', 10
            ),
            'right': self.create_publisher(
                Float64MultiArray, '/right_gripper_controller/commands', 10
            ),
        }

        self.get_logger().info('Arm controller initialized')

    async def move_to_pose(
        self,
        arm: str,
        target_pose: Pose,
        velocity_scaling: float = 0.5,
        acceleration_scaling: float = 0.5
    ) -> bool:
        """Move arm to target pose."""
        self.get_logger().info(f'Moving {arm} to pose')

        # Create goal
        goal = MoveGroup.Goal()
        goal.request.group_name = arm
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = acceleration_scaling

        # Set target
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'torso_link'
        pose_stamped.pose = target_pose

        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = f'{arm}_ee_link'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Bounding volume (sphere around target)
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(target_pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = f'{arm}_ee_link'
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # Add to goal constraints
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints.append(constraints)

        # Send goal
        client = self.move_group_clients[arm]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup server not available')
            return False

        result = await client.send_goal_async(goal)

        if not result.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        final_result = await result.get_result_async()
        success = final_result.result.error_code.val == 1  # SUCCESS

        if success:
            self.get_logger().info(f'{arm} reached target')
        else:
            self.get_logger().warn(f'{arm} failed: {final_result.result.error_code}')

        return success

    async def move_to_named_pose(self, arm: str, pose_name: str) -> bool:
        """Move to named pose (e.g., 'home')."""
        goal = MoveGroup.Goal()
        goal.request.group_name = arm
        goal.request.start_state.is_diff = True

        # Named target
        goal.request.goal_constraints = []  # Clear pose constraints
        # Named targets handled by planner

        client = self.move_group_clients[arm]
        result = await client.send_goal_async(goal)

        return result.accepted

    def open_gripper(self, side: str):
        """Open gripper."""
        msg = Float64MultiArray()
        msg.data = [0.04, 0.04]  # Open position
        self.gripper_pubs[side].publish(msg)
        self.get_logger().info(f'{side} gripper opened')

    def close_gripper(self, side: str, width: float = 0.0):
        """Close gripper to specified width."""
        msg = Float64MultiArray()
        msg.data = [width / 2, width / 2]
        self.gripper_pubs[side].publish(msg)
        self.get_logger().info(f'{side} gripper closed to {width}m')

    async def compute_ik(self, arm: str, pose: Pose) -> list:
        """Compute inverse kinematics for pose."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = arm
        request.ik_request.pose_stamped.header.frame_id = 'torso_link'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.timeout.sec = 1

        future = self.ik_client.call_async(request)
        response = await future

        if response.error_code.val == 1:  # SUCCESS
            return list(response.solution.joint_state.position)
        else:
            return None
```

---

## Pick and Place Actions {#pick-place}

### Pick Action Server

```python
#!/usr/bin/env python3
"""Pick action server for humanoid manipulation."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from humanoid_msgs.action import Pick
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import time


class PickActionServer(Node):
    """Execute pick operations."""

    def __init__(self, arm_controller: ArmController):
        super().__init__('pick_action_server')

        self.arm = arm_controller

        # Action server
        self.action_server = ActionServer(
            self,
            Pick,
            'humanoid/pick',
            execute_callback=self.execute_pick
        )

        # Parameters
        self.approach_distance = 0.1  # 10cm above object
        self.lift_height = 0.15       # 15cm lift after grasp

        self.get_logger().info('Pick action server ready')

    async def execute_pick(self, goal_handle):
        """Execute pick sequence."""
        target = goal_handle.request.target_pose
        arm = goal_handle.request.arm  # 'left' or 'right'
        object_width = goal_handle.request.object_width

        self.get_logger().info(f'Picking object with {arm} arm')

        feedback = Pick.Feedback()

        # Phase 1: Open gripper
        feedback.phase = 'opening_gripper'
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        self.arm.open_gripper(arm)
        time.sleep(0.5)

        # Phase 2: Move to approach pose
        feedback.phase = 'approaching'
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        approach_pose = self.compute_approach_pose(target)
        success = await self.arm.move_to_pose(arm, approach_pose)

        if not success:
            self.get_logger().error('Failed to reach approach pose')
            goal_handle.abort()
            return Pick.Result(success=False, message='Approach failed')

        # Phase 3: Move to grasp pose
        feedback.phase = 'reaching'
        feedback.progress = 0.4
        goal_handle.publish_feedback(feedback)

        grasp_pose = self.compute_grasp_pose(target)
        success = await self.arm.move_to_pose(
            arm, grasp_pose,
            velocity_scaling=0.3  # Slower for precision
        )

        if not success:
            self.get_logger().error('Failed to reach grasp pose')
            goal_handle.abort()
            return Pick.Result(success=False, message='Grasp approach failed')

        # Phase 4: Close gripper
        feedback.phase = 'grasping'
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        self.arm.close_gripper(arm, object_width)
        time.sleep(0.5)

        # Phase 5: Lift
        feedback.phase = 'lifting'
        feedback.progress = 0.8
        goal_handle.publish_feedback(feedback)

        lift_pose = self.compute_lift_pose(grasp_pose)
        success = await self.arm.move_to_pose(arm, lift_pose)

        if not success:
            self.get_logger().warn('Lift failed, attempting recovery')
            # Recovery: try to lift anyway
            lift_pose.position.z += 0.05
            success = await self.arm.move_to_pose(arm, lift_pose)

        # Complete
        feedback.phase = 'complete'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        if success:
            goal_handle.succeed()
            return Pick.Result(success=True, message='Pick successful')
        else:
            goal_handle.abort()
            return Pick.Result(success=False, message='Lift failed')

    def compute_approach_pose(self, target: Pose) -> Pose:
        """Compute approach pose above target."""
        approach = Pose()
        approach.position.x = target.position.x
        approach.position.y = target.position.y
        approach.position.z = target.position.z + self.approach_distance

        # Top-down grasp orientation
        approach.orientation.x = 0.707
        approach.orientation.y = 0.707
        approach.orientation.z = 0.0
        approach.orientation.w = 0.0

        return approach

    def compute_grasp_pose(self, target: Pose) -> Pose:
        """Compute grasp pose at target."""
        grasp = Pose()
        grasp.position = target.position

        # Same orientation as approach
        grasp.orientation.x = 0.707
        grasp.orientation.y = 0.707
        grasp.orientation.z = 0.0
        grasp.orientation.w = 0.0

        return grasp

    def compute_lift_pose(self, grasp: Pose) -> Pose:
        """Compute lift pose above grasp."""
        lift = Pose()
        lift.position.x = grasp.position.x
        lift.position.y = grasp.position.y
        lift.position.z = grasp.position.z + self.lift_height
        lift.orientation = grasp.orientation

        return lift
```

### Place Action Server

```python
#!/usr/bin/env python3
"""Place action server for humanoid manipulation."""

from humanoid_msgs.action import Place


class PlaceActionServer(Node):
    """Execute place operations."""

    def __init__(self, arm_controller: ArmController):
        super().__init__('place_action_server')

        self.arm = arm_controller

        self.action_server = ActionServer(
            self,
            Place,
            'humanoid/place',
            execute_callback=self.execute_place
        )

        self.approach_distance = 0.1
        self.place_offset = 0.02  # Small offset above surface

        self.get_logger().info('Place action server ready')

    async def execute_place(self, goal_handle):
        """Execute place sequence."""
        target = goal_handle.request.target_pose
        arm = goal_handle.request.arm

        self.get_logger().info(f'Placing object with {arm} arm')

        feedback = Place.Feedback()

        # Phase 1: Move to approach pose
        feedback.phase = 'approaching'
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        approach_pose = Pose()
        approach_pose.position.x = target.position.x
        approach_pose.position.y = target.position.y
        approach_pose.position.z = target.position.z + self.approach_distance
        approach_pose.orientation = target.orientation

        success = await self.arm.move_to_pose(arm, approach_pose)

        if not success:
            goal_handle.abort()
            return Place.Result(success=False, message='Approach failed')

        # Phase 2: Move to place pose
        feedback.phase = 'placing'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        place_pose = Pose()
        place_pose.position.x = target.position.x
        place_pose.position.y = target.position.y
        place_pose.position.z = target.position.z + self.place_offset
        place_pose.orientation = target.orientation

        success = await self.arm.move_to_pose(
            arm, place_pose,
            velocity_scaling=0.3
        )

        if not success:
            goal_handle.abort()
            return Place.Result(success=False, message='Place motion failed')

        # Phase 3: Open gripper
        feedback.phase = 'releasing'
        feedback.progress = 0.7
        goal_handle.publish_feedback(feedback)

        self.arm.open_gripper(arm)
        time.sleep(0.5)

        # Phase 4: Retreat
        feedback.phase = 'retreating'
        feedback.progress = 0.9
        goal_handle.publish_feedback(feedback)

        retreat_pose = Pose()
        retreat_pose.position.x = place_pose.position.x
        retreat_pose.position.y = place_pose.position.y
        retreat_pose.position.z = place_pose.position.z + self.approach_distance
        retreat_pose.orientation = place_pose.orientation

        await self.arm.move_to_pose(arm, retreat_pose)

        # Complete
        feedback.phase = 'complete'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return Place.Result(success=True, message='Place successful')
```

---

## Grasp Planning {#grasp-planning}

### Grasp Pose Generation

```python
#!/usr/bin/env python3
"""Generate grasp poses for detected objects."""

import numpy as np
from geometry_msgs.msg import Pose
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class GraspPose:
    """A candidate grasp pose with metadata."""
    pose: Pose
    approach_direction: np.ndarray
    grasp_width: float
    score: float


class GraspPlanner:
    """Generate and rank grasp poses."""

    def __init__(self):
        # Gripper parameters
        self.gripper_depth = 0.08  # How far fingers extend
        self.max_gripper_width = 0.08
        self.min_gripper_width = 0.0

    def generate_grasps(
        self,
        object_pose: Pose,
        object_dimensions: List[float],
        object_class: str
    ) -> List[GraspPose]:
        """Generate candidate grasp poses for object."""
        grasps = []

        # Object center
        center = np.array([
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z
        ])

        # Object size
        width, depth, height = object_dimensions

        # Generate grasps based on object type
        if object_class in ['mug', 'cup', 'bottle']:
            grasps.extend(self.generate_cylindrical_grasps(
                center, max(width, depth), height
            ))
        elif object_class in ['box', 'book']:
            grasps.extend(self.generate_box_grasps(
                center, width, depth, height
            ))
        else:
            # Generic grasp generation
            grasps.extend(self.generate_generic_grasps(
                center, max(width, depth, height)
            ))

        # Sort by score
        grasps.sort(key=lambda g: g.score, reverse=True)

        return grasps

    def generate_cylindrical_grasps(
        self,
        center: np.ndarray,
        diameter: float,
        height: float
    ) -> List[GraspPose]:
        """Generate grasps for cylindrical objects."""
        grasps = []

        # Top-down grasp (best for mugs/bottles)
        if diameter < self.max_gripper_width:
            grasp = self.create_top_grasp(center, diameter)
            grasp.score = 0.9  # High score for top grasp
            grasps.append(grasp)

        # Side grasps at different angles
        for angle in np.linspace(0, np.pi, 4):
            grasp = self.create_side_grasp(center, diameter, height, angle)
            grasp.score = 0.7
            grasps.append(grasp)

        return grasps

    def generate_box_grasps(
        self,
        center: np.ndarray,
        width: float,
        depth: float,
        height: float
    ) -> List[GraspPose]:
        """Generate grasps for box-shaped objects."""
        grasps = []

        # Grasp along shorter dimension
        if width < depth and width < self.max_gripper_width:
            # Grasp across width
            grasp = self.create_side_grasp(center, width, height, 0)
            grasp.score = 0.8
            grasps.append(grasp)

            grasp = self.create_side_grasp(center, width, height, np.pi)
            grasp.score = 0.8
            grasps.append(grasp)

        if depth < width and depth < self.max_gripper_width:
            # Grasp across depth
            grasp = self.create_side_grasp(center, depth, height, np.pi/2)
            grasp.score = 0.8
            grasps.append(grasp)

        # Top grasp if thin enough
        if min(width, depth) < self.max_gripper_width:
            grasp = self.create_top_grasp(center, min(width, depth))
            grasp.score = 0.6  # Lower score for boxes
            grasps.append(grasp)

        return grasps

    def create_top_grasp(
        self,
        center: np.ndarray,
        width: float
    ) -> GraspPose:
        """Create top-down grasp pose."""
        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]

        # Top-down orientation (gripper pointing down)
        pose.orientation.x = 0.707
        pose.orientation.y = 0.707
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return GraspPose(
            pose=pose,
            approach_direction=np.array([0, 0, -1]),
            grasp_width=width + 0.01,  # Small margin
            score=0.0
        )

    def create_side_grasp(
        self,
        center: np.ndarray,
        width: float,
        height: float,
        angle: float
    ) -> GraspPose:
        """Create side grasp pose at given angle."""
        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]

        # Horizontal grasp orientation
        # Quaternion for rotation around Z then pointing horizontally
        qw = np.cos(angle / 2)
        qz = np.sin(angle / 2)

        pose.orientation.x = 0.5
        pose.orientation.y = 0.5
        pose.orientation.z = 0.5 * qz / max(0.001, np.sqrt(1 - qw*qw)) if qw < 0.99 else 0
        pose.orientation.w = 0.5

        approach = np.array([np.cos(angle), np.sin(angle), 0])

        return GraspPose(
            pose=pose,
            approach_direction=approach,
            grasp_width=width + 0.01,
            score=0.0
        )

    def generate_generic_grasps(
        self,
        center: np.ndarray,
        size: float
    ) -> List[GraspPose]:
        """Generate generic grasps for unknown objects."""
        grasps = []

        # Top grasp if small enough
        if size < self.max_gripper_width:
            grasp = self.create_top_grasp(center, size)
            grasp.score = 0.5
            grasps.append(grasp)

        return grasps
```

---

## Bimanual Coordination {#bimanual}

### Coordinated Dual-Arm Operations

```python
#!/usr/bin/env python3
"""Coordinated bimanual manipulation."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from typing import Tuple


class BimanualController(Node):
    """Coordinate both arms for complex tasks."""

    def __init__(self, arm_controller: ArmController):
        super().__init__('bimanual_controller')

        self.arm = arm_controller

        # Coordination parameters
        self.sync_tolerance = 0.1  # Max allowed desync

    async def handoff(
        self,
        from_arm: str,
        to_arm: str,
        object_width: float
    ) -> bool:
        """Hand object from one arm to the other."""
        self.get_logger().info(f'Handoff from {from_arm} to {to_arm}')

        # Calculate handoff position (center of workspace)
        handoff_pose = Pose()
        handoff_pose.position.x = 0.5
        handoff_pose.position.y = 0.0
        handoff_pose.position.z = 0.6

        # Receiving arm orientation (facing giving arm)
        handoff_pose.orientation.x = 0.5
        handoff_pose.orientation.y = 0.5
        handoff_pose.orientation.z = 0.5
        handoff_pose.orientation.w = 0.5

        # Move giving arm to handoff position
        success = await self.arm.move_to_pose(from_arm, handoff_pose)
        if not success:
            return False

        # Open receiving gripper
        self.arm.open_gripper(to_arm)

        # Move receiving arm to handoff position
        receive_pose = Pose()
        receive_pose.position.x = handoff_pose.position.x
        receive_pose.position.y = handoff_pose.position.y
        receive_pose.position.z = handoff_pose.position.z

        # Opposite orientation
        receive_pose.orientation.x = -0.5
        receive_pose.orientation.y = 0.5
        receive_pose.orientation.z = 0.5
        receive_pose.orientation.w = 0.5

        success = await self.arm.move_to_pose(to_arm, receive_pose)
        if not success:
            return False

        # Close receiving gripper
        self.arm.close_gripper(to_arm, object_width)

        # Wait for grip
        await asyncio.sleep(0.5)

        # Open giving gripper
        self.arm.open_gripper(from_arm)

        # Retreat giving arm
        retreat_pose = Pose()
        retreat_pose.position.x = handoff_pose.position.x
        retreat_pose.position.y = 0.3 if from_arm == 'left' else -0.3
        retreat_pose.position.z = handoff_pose.position.z
        retreat_pose.orientation = handoff_pose.orientation

        await self.arm.move_to_pose(from_arm, retreat_pose)

        self.get_logger().info('Handoff complete')
        return True

    async def synchronized_move(
        self,
        left_pose: Pose,
        right_pose: Pose
    ) -> bool:
        """Move both arms simultaneously."""
        self.get_logger().info('Synchronized bimanual move')

        # Compute trajectories
        left_ik = await self.arm.compute_ik('left_arm', left_pose)
        right_ik = await self.arm.compute_ik('right_arm', right_pose)

        if left_ik is None or right_ik is None:
            self.get_logger().error('IK failed for synchronized move')
            return False

        # TODO: Implement synchronized trajectory execution
        # This would require custom trajectory controller that
        # synchronizes both arms

        # For now, move sequentially
        left_success = await self.arm.move_to_pose('left_arm', left_pose)
        right_success = await self.arm.move_to_pose('right_arm', right_pose)

        return left_success and right_success

    async def carry_large_object(
        self,
        object_pose: Pose,
        target_pose: Pose,
        object_width: float
    ) -> bool:
        """Carry large object with both hands."""
        self.get_logger().info('Bimanual carry operation')

        # Calculate grasp positions on either side of object
        half_width = object_width / 2 + 0.05  # Grasp margin

        left_grasp = Pose()
        left_grasp.position.x = object_pose.position.x
        left_grasp.position.y = object_pose.position.y + half_width
        left_grasp.position.z = object_pose.position.z
        # Inward-facing orientation
        left_grasp.orientation.x = 0.0
        left_grasp.orientation.y = 0.707
        left_grasp.orientation.z = 0.0
        left_grasp.orientation.w = 0.707

        right_grasp = Pose()
        right_grasp.position.x = object_pose.position.x
        right_grasp.position.y = object_pose.position.y - half_width
        right_grasp.position.z = object_pose.position.z
        right_grasp.orientation.x = 0.0
        right_grasp.orientation.y = 0.707
        right_grasp.orientation.z = 0.0
        right_grasp.orientation.w = -0.707

        # Open both grippers
        self.arm.open_gripper('left')
        self.arm.open_gripper('right')

        # Move to grasp positions
        success = await self.synchronized_move(left_grasp, right_grasp)
        if not success:
            return False

        # Close grippers
        self.arm.close_gripper('left')
        self.arm.close_gripper('right')
        await asyncio.sleep(0.5)

        # Lift and carry to target
        left_target = Pose()
        left_target.position.x = target_pose.position.x
        left_target.position.y = target_pose.position.y + half_width
        left_target.position.z = target_pose.position.z
        left_target.orientation = left_grasp.orientation

        right_target = Pose()
        right_target.position.x = target_pose.position.x
        right_target.position.y = target_pose.position.y - half_width
        right_target.position.z = target_pose.position.z
        right_target.orientation = right_grasp.orientation

        success = await self.synchronized_move(left_target, right_target)
        if not success:
            return False

        # Release
        self.arm.open_gripper('left')
        self.arm.open_gripper('right')

        return True
```

---

## Summary

This chapter covered manipulation system implementation:

1. **MoveIt 2 configuration** with SRDF, kinematics, and motion planning parameters.

2. **Arm controller** providing high-level motion commands and gripper control.

3. **Pick and place actions** with approach, grasp, lift, and retreat phases.

4. **Grasp planning** generating candidate poses for different object shapes.

5. **Bimanual coordination** for handoffs and large object manipulation.

---

## Manipulation Checklist

Before proceeding, verify:

- [ ] MoveIt 2 plans paths for both arms
- [ ] Gripper open/close commands work
- [ ] Pick action successfully grasps objects
- [ ] Place action successfully releases objects
- [ ] Grasp planning generates valid poses
- [ ] Arms don't collide with each other or environment

---

## What's Next

In [Chapter 5: Integration & Testing](/capstone/chapter-5-integration), you'll bring all systems together for end-to-end VLA-controlled operation.
