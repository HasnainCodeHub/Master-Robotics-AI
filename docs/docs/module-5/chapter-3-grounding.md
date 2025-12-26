---
id: chapter-3-grounding
title: "Chapter 3: Grounding"
sidebar_label: "3. Grounding"
sidebar_position: 4
---

# Chapter 3: Grounding and Symbol-to-Action Mapping

## Chapter Goal

By the end of this chapter, you will be able to **implement the grounding layer that maps LLM-generated symbolic task plans to executable ROS 2 actions**, with explicit scene verification and coordinate frame resolution.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Explain the symbol grounding problem |
| 3.2 | Implement object reference resolution |
| 3.3 | Implement spatial reference resolution |
| 3.4 | Implement action mapping to ROS 2 action goals |
| 3.5 | Implement grounding verification |
| 3.6 | Handle grounding failures |

---

## The Symbol Grounding Problem {#problem}

### Why Can't Robots Just Execute Plans?

Consider the command: "Pick up the red mug"

The LLM produces:
```json
{"action_type": "pick", "target": "red_mug"}
```

But the robot needs:
```
target_pose:
  position: {x: 1.23, y: 0.45, z: 0.82}
  orientation: {x: 0, y: 0, z: 0, w: 1}
approach_direction: {x: 0, y: 0, z: -1}
```

**Symbol grounding** connects abstract symbols ("red_mug") to physical quantities (pose, reachability, grasp points).

### Grounding Pipeline

```
Symbolic Plan ──► Object Resolution ──► Spatial Resolution ──► Action Mapping
     │                    │                     │                    │
"red_mug"        Detection + ID        Pose estimation        ROS 2 Action
                                                                  Goal
```

---

## Object Reference Resolution {#object-resolution}

### From Name to Detection

```python
#!/usr/bin/env python3
"""Object reference resolution using perception."""

from dataclasses import dataclass
from typing import Optional, List
import numpy as np


@dataclass
class DetectedObject:
    class_name: str
    instance_id: int
    confidence: float
    bbox_2d: tuple  # (x, y, width, height)
    position_3d: np.ndarray  # (x, y, z) in robot frame
    color: Optional[str] = None


class ObjectResolver:
    def __init__(self):
        self.detections: List[DetectedObject] = []

    def update_detections(self, detection_msg):
        """Update from perception pipeline."""
        self.detections = []
        for det in detection_msg.detections:
            obj = DetectedObject(
                class_name=det.class_name,
                instance_id=det.id,
                confidence=det.confidence,
                bbox_2d=(det.bbox.x, det.bbox.y, det.bbox.w, det.bbox.h),
                position_3d=np.array([det.pose.x, det.pose.y, det.pose.z]),
                color=det.color if hasattr(det, 'color') else None
            )
            self.detections.append(obj)

    def resolve(self, reference: str) -> Optional[DetectedObject]:
        """
        Resolve textual reference to detected object.

        Args:
            reference: Object description (e.g., "red_mug", "the box")

        Returns:
            DetectedObject if found and unambiguous, None otherwise
        """
        # Parse reference
        tokens = reference.lower().replace('_', ' ').split()

        # Extract color and class
        colors = {'red', 'blue', 'green', 'yellow', 'black', 'white'}
        ref_color = None
        ref_class = None

        for token in tokens:
            if token in colors:
                ref_color = token
            elif token not in {'the', 'a', 'an'}:
                ref_class = token

        # Find matching objects
        candidates = []
        for det in self.detections:
            # Class match
            if ref_class and ref_class not in det.class_name.lower():
                continue

            # Color match (if specified)
            if ref_color and det.color != ref_color:
                continue

            candidates.append(det)

        # Handle results
        if len(candidates) == 0:
            return None  # Not found
        elif len(candidates) == 1:
            return candidates[0]  # Unambiguous
        else:
            # Ambiguous - return None, caller should request clarification
            return None

    def get_ambiguous_candidates(self, reference: str) -> List[DetectedObject]:
        """Get all candidates for ambiguous reference."""
        # Same logic as resolve, but return all matches
        # Used for generating clarification requests
        pass
```

### Handling Ambiguity

```python
def resolve_with_disambiguation(
    self,
    reference: str,
    context: dict
) -> tuple[Optional[DetectedObject], Optional[str]]:
    """
    Resolve reference, returning clarification request if ambiguous.

    Returns:
        (object, None) if resolved
        (None, clarification_question) if ambiguous
        (None, None) if not found
    """
    candidates = self.find_candidates(reference)

    if len(candidates) == 0:
        return None, None

    if len(candidates) == 1:
        return candidates[0], None

    # Ambiguous - generate clarification
    descriptions = [self.describe_object(c) for c in candidates]
    question = f"I see multiple objects matching '{reference}': {', '.join(descriptions)}. Which one do you mean?"

    return None, question
```

---

## Spatial Reference Resolution {#spatial-resolution}

### Location Descriptions to Coordinates

```python
class SpatialResolver:
    def __init__(self):
        self.known_locations = {}  # name -> pose
        self.scene_objects = []

    def register_location(self, name: str, pose: np.ndarray):
        """Register named location."""
        self.known_locations[name.lower()] = pose

    def resolve_location(
        self,
        description: str,
        reference_object: Optional[DetectedObject] = None
    ) -> Optional[np.ndarray]:
        """
        Resolve location description to coordinates.

        Examples:
            "on the shelf" -> shelf surface position
            "next to the keyboard" -> position adjacent to keyboard
            "home position" -> registered home pose
        """
        desc_lower = description.lower()

        # Check registered locations
        for name, pose in self.known_locations.items():
            if name in desc_lower:
                return pose

        # Parse spatial relations
        if "on" in desc_lower or "on top of" in desc_lower:
            return self.resolve_on_surface(desc_lower)
        elif "next to" in desc_lower or "beside" in desc_lower:
            return self.resolve_adjacent(desc_lower, reference_object)
        elif "in front of" in desc_lower:
            return self.resolve_relative(desc_lower, "front")
        elif "behind" in desc_lower:
            return self.resolve_relative(desc_lower, "behind")
        elif "left of" in desc_lower:
            return self.resolve_relative(desc_lower, "left")
        elif "right of" in desc_lower:
            return self.resolve_relative(desc_lower, "right")

        return None

    def resolve_on_surface(self, description: str) -> Optional[np.ndarray]:
        """Resolve 'on the X' to surface position."""
        # Find surface object (table, shelf, etc.)
        surfaces = ['table', 'shelf', 'desk', 'counter']
        for surface in surfaces:
            if surface in description:
                for obj in self.scene_objects:
                    if surface in obj.class_name.lower():
                        # Return position on top of surface
                        pos = obj.position_3d.copy()
                        pos[2] += 0.1  # Above surface
                        return pos
        return None
```

---

## Action Mapping {#action-mapping}

### From Symbolic Action to ROS 2 Goal

```python
#!/usr/bin/env python3
"""Map symbolic actions to ROS 2 action goals."""

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from nav2_msgs.action import NavigateToPose


class ActionMapper:
    def __init__(self, node):
        self.node = node

        # Action clients
        self.nav_client = ActionClient(
            node, NavigateToPose, '/navigate_to_pose'
        )
        self.moveit_client = ActionClient(
            node, MoveGroup, '/move_action'
        )

    def map_action(
        self,
        action: dict,
        grounded_target: Optional[DetectedObject],
        grounded_position: Optional[np.ndarray]
    ) -> tuple[str, Any]:
        """
        Map symbolic action to ROS 2 action goal.

        Returns:
            (action_name, goal_msg)
        """
        action_type = action['action_type']

        if action_type == 'navigate_to':
            return self.map_navigate(action, grounded_position)

        elif action_type == 'pick':
            return self.map_pick(action, grounded_target)

        elif action_type == 'place':
            return self.map_place(action, grounded_position)

        elif action_type == 'open_gripper':
            return self.map_gripper(open=True)

        elif action_type == 'close_gripper':
            force = action.get('parameters', {}).get('force', 50.0)
            return self.map_gripper(open=False, force=force)

        else:
            raise ValueError(f"Unknown action type: {action_type}")

    def map_navigate(
        self,
        action: dict,
        position: np.ndarray
    ) -> tuple[str, NavigateToPose.Goal]:
        """Map navigate action to Nav2 goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(position[0])
        goal.pose.pose.position.y = float(position[1])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        return 'navigate_to_pose', goal

    def map_pick(
        self,
        action: dict,
        target: DetectedObject
    ) -> tuple[str, MoveGroup.Goal]:
        """Map pick action to MoveIt goal."""
        goal = MoveGroup.Goal()

        # Set target pose (pre-grasp position above object)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(target.position_3d[0])
        target_pose.pose.position.y = float(target.position_3d[1])
        target_pose.pose.position.z = float(target.position_3d[2] + 0.1)

        # Top-down grasp orientation
        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.y = 0.707
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        # Configure MoveIt goal
        goal.request.group_name = 'arm'
        goal.request.goal_constraints = [
            self.create_pose_constraint(target_pose)
        ]

        return 'move_group', goal
```

---

## Grounding Verification {#verification}

### Precondition Checking

```python
class GroundingVerifier:
    def __init__(self, node, object_resolver, spatial_resolver):
        self.node = node
        self.object_resolver = object_resolver
        self.spatial_resolver = spatial_resolver

    def verify_action(
        self,
        action: dict,
        grounded_target: Optional[DetectedObject],
        grounded_position: Optional[np.ndarray]
    ) -> tuple[bool, str]:
        """
        Verify action preconditions are met.

        Returns:
            (success, error_message)
        """
        action_type = action['action_type']

        if action_type == 'pick':
            return self.verify_pick(action, grounded_target)

        elif action_type == 'place':
            return self.verify_place(action, grounded_position)

        elif action_type == 'navigate_to':
            return self.verify_navigation(action, grounded_position)

        return True, ""

    def verify_pick(
        self,
        action: dict,
        target: Optional[DetectedObject]
    ) -> tuple[bool, str]:
        """Verify pick preconditions."""

        # Target must be resolved
        if target is None:
            return False, "Target object not found"

        # Target must be visible (recent detection)
        if not self.is_currently_visible(target):
            return False, "Target not currently visible"

        # Target must be reachable
        if not self.is_reachable(target.position_3d):
            return False, "Target position not reachable"

        # Gripper must be empty
        if self.is_gripper_holding():
            return False, "Gripper is not empty"

        return True, ""

    def is_reachable(self, position: np.ndarray) -> bool:
        """Check if position is within robot workspace."""
        # Simple workspace check
        x, y, z = position
        return (0.2 <= x <= 0.8 and
                -0.5 <= y <= 0.5 and
                0.0 <= z <= 1.0)

    def is_currently_visible(self, target: DetectedObject) -> bool:
        """Check if target was detected recently."""
        current_detections = self.object_resolver.detections
        for det in current_detections:
            if det.instance_id == target.instance_id:
                return True
        return False
```

---

## Handling Grounding Failures {#failures}

### Failure Recovery Strategies

```python
class GroundingFailureHandler:
    def __init__(self, node):
        self.node = node
        self.clarification_pub = node.create_publisher(
            String, '/vla/clarification_request', 10
        )

    def handle_failure(
        self,
        action: dict,
        failure_reason: str,
        context: dict
    ) -> Optional[dict]:
        """
        Handle grounding failure with appropriate recovery.

        Returns:
            Recovery action or None if unrecoverable
        """
        if "not found" in failure_reason.lower():
            return self.handle_not_found(action, context)

        elif "ambiguous" in failure_reason.lower():
            return self.handle_ambiguous(action, context)

        elif "not reachable" in failure_reason.lower():
            return self.handle_unreachable(action, context)

        elif "not visible" in failure_reason.lower():
            return self.handle_not_visible(action, context)

        return None

    def handle_not_found(self, action: dict, context: dict) -> dict:
        """Object not found - request human assistance."""
        target = action.get('target', 'the object')

        self.request_clarification(
            f"I cannot find {target}. Can you show me where it is?"
        )

        return {
            "action": "wait_for_assistance",
            "timeout": 30.0,
            "then_retry": True
        }

    def handle_ambiguous(self, action: dict, context: dict) -> dict:
        """Multiple candidates - request clarification."""
        candidates = context.get('candidates', [])
        descriptions = [f"{c.color} {c.class_name}" for c in candidates]

        self.request_clarification(
            f"I see multiple objects: {', '.join(descriptions)}. "
            "Which one should I pick?"
        )

        return {
            "action": "wait_for_clarification",
            "timeout": 30.0
        }

    def handle_unreachable(self, action: dict, context: dict) -> dict:
        """Position not reachable - navigate closer."""
        target_pos = context.get('target_position')

        return {
            "action": "navigate_closer",
            "target": target_pos,
            "then_retry": True
        }

    def request_clarification(self, message: str):
        """Publish clarification request (for speech synthesis)."""
        msg = String()
        msg.data = message
        self.clarification_pub.publish(msg)
        self.node.get_logger().info(f"Requesting clarification: {message}")
```

---

## Complete Grounding Node {#complete-node}

```python
#!/usr/bin/env python3
"""Complete grounding node connecting LLM planner to robot execution."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class GroundingNode(Node):
    def __init__(self):
        super().__init__('grounding')

        # Components
        self.object_resolver = ObjectResolver()
        self.spatial_resolver = SpatialResolver()
        self.action_mapper = ActionMapper(self)
        self.verifier = GroundingVerifier(
            self, self.object_resolver, self.spatial_resolver
        )
        self.failure_handler = GroundingFailureHandler(self)

        # Subscriptions
        self.plan_sub = self.create_subscription(
            String, '/task_planner/plan', self.plan_callback, 10
        )
        self.detection_sub = self.create_subscription(
            DetectionArray, '/detections', self.detection_callback, 10
        )

        # Publishers
        self.action_pub = self.create_publisher(
            String, '/grounding/action', 10
        )

        self.get_logger().info('Grounding node ready')

    def plan_callback(self, msg: String):
        """Process task plan and ground each action."""
        plan = json.loads(msg.data)

        for action in plan.get('actions', []):
            grounded = self.ground_action(action)

            if grounded is None:
                self.get_logger().error(f"Failed to ground action: {action}")
                continue

            # Publish grounded action
            action_msg = String()
            action_msg.data = json.dumps(grounded)
            self.action_pub.publish(action_msg)

    def ground_action(self, action: dict) -> Optional[dict]:
        """Ground single action to executable form."""
        # Resolve object reference
        target = None
        if action.get('target'):
            target = self.object_resolver.resolve(action['target'])

        # Resolve spatial reference
        position = None
        if action.get('parameters', {}).get('position'):
            pos_ref = action['parameters']['position']
            if isinstance(pos_ref, str):
                position = self.spatial_resolver.resolve_location(pos_ref)
            else:
                position = np.array(pos_ref)

        # Verify preconditions
        success, error = self.verifier.verify_action(action, target, position)

        if not success:
            recovery = self.failure_handler.handle_failure(
                action, error, {'target': target, 'position': position}
            )
            return recovery

        # Map to ROS 2 action
        action_name, goal = self.action_mapper.map_action(
            action, target, position
        )

        return {
            'action_name': action_name,
            'goal': self.goal_to_dict(goal),
            'original_action': action
        }
```

---

## Summary

This chapter covered grounding—connecting symbols to the physical world:

1. **Symbol grounding** maps abstract references ("red_mug") to physical quantities (pose).

2. **Object resolution** uses perception to find and identify referenced objects.

3. **Spatial resolution** converts location descriptions to coordinates.

4. **Action mapping** translates symbolic actions to ROS 2 action goals.

5. **Verification** checks preconditions before execution.

6. **Failure handling** requests clarification or triggers recovery.

---

## Self-Assessment Questions

1. **Grounding Challenge**: The LLM says "pick up the cup" but perception detects "mug" not "cup". How would you handle this synonym problem?

2. **Spatial Ambiguity**: "Put it on the table" when there are two tables. What disambiguation strategy would you use?

3. **Verification Design**: What preconditions would you verify for a "place" action?

4. **Failure Recovery**: The target object disappears during approach (someone moved it). What should the system do?

5. **Coordinate Frames**: Your perception gives positions in camera_link frame but navigation needs map frame. What's needed for grounding?

---

## What's Next

In [Chapter 4: Vision-Language Models](/module-5/chapter-4-vision-language-models), you'll integrate VLMs for scene understanding and flexible object identification.
