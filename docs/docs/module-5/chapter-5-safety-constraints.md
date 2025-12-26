---
id: chapter-5-safety-constraints
title: "Chapter 5: Safety Constraints"
sidebar_label: "5. Safety Constraints"
sidebar_position: 6
---

# Chapter 5: Safety Constraints for AI-Generated Commands

## Chapter Goal

By the end of this chapter, you will be able to **implement a safety constraint layer that validates AI-generated commands before execution**, preventing dangerous actions while providing informative feedback for recovery.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 5.1 | Design a safety constraint architecture for VLA systems |
| 5.2 | Implement workspace boundary checking |
| 5.3 | Implement velocity and acceleration limits |
| 5.4 | Implement collision checking with perception |
| 5.5 | Implement force limits and contact safety |
| 5.6 | Design informative safety rejection responses |

---

## Safety Philosophy {#philosophy}

### The Safety Override Principle

**Critical Principle**: Safety constraints MUST override AI outputs.

```
AI says: "Move at 2 m/s"  →  Safety: "Max is 0.5 m/s"  →  REJECT
AI says: "Path is clear"  →  Safety: "Object detected"  →  REJECT
AI says: "Pick 50kg box"  →  Safety: "Max is 5 kg"     →  REJECT
```

The robot may fail to act, but it **must not act unsafely**.

### Defense in Depth

```
                    Command Input
                         │
                         ▼
              ┌──────────────────────┐
              │  Layer 1: LLM Prompt │  ← Soft constraints in prompt
              │  Constraints         │
              └──────────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │  Layer 2: Schema     │  ← Parameter validation
              │  Validation          │
              └──────────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │  Layer 3: Safety     │  ← Physical constraint checking
              │  Filter              │
              └──────────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │  Layer 4: Runtime    │  ← Real-time monitoring
              │  Monitoring          │
              └──────────────────────┘
                         │
                         ▼
                   Safe Execution
```

Each layer catches different failure modes. **Never rely on a single layer.**

---

## Safety Constraint Architecture {#architecture}

### SafetyFilter Base Class

```python
#!/usr/bin/env python3
"""Safety constraint system for VLA pipeline."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
import numpy as np


class SafetyLevel(Enum):
    """Severity of safety violation."""
    INFO = 0        # Informational, no action needed
    WARNING = 1     # Concerning but allowed
    VIOLATION = 2   # Not allowed, reject command
    CRITICAL = 3    # Emergency, stop all motion


@dataclass
class SafetyViolation:
    """Details of a safety constraint violation."""
    constraint_name: str
    level: SafetyLevel
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    suggested_fix: Optional[str] = None


@dataclass
class SafetyCheckResult:
    """Result of safety constraint checking."""
    is_safe: bool
    violations: List[SafetyViolation] = field(default_factory=list)

    def add_violation(self, violation: SafetyViolation):
        self.violations.append(violation)
        if violation.level >= SafetyLevel.VIOLATION:
            self.is_safe = False


class SafetyConstraint(ABC):
    """Base class for safety constraints."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Constraint name for logging."""
        pass

    @abstractmethod
    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        """
        Check if action violates this constraint.

        Args:
            action: The proposed action
            context: Current robot/environment state

        Returns:
            List of violations (empty if safe)
        """
        pass
```

### Safety Filter Node

```python
class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('safety_filter')

        # Load constraints
        self.constraints: List[SafetyConstraint] = [
            WorkspaceBoundaryConstraint(),
            VelocityLimitConstraint(),
            AccelerationLimitConstraint(),
            CollisionConstraint(),
            ForceLimitConstraint(),
            JointLimitConstraint(),
        ]

        # Context providers
        self.robot_state = None
        self.scene_objects = []

        # Subscribers
        self.action_sub = self.create_subscription(
            String, '/grounding/action', self.action_callback, 10
        )
        self.robot_state_sub = self.create_subscription(
            JointState, '/joint_states', self.robot_state_callback, 10
        )

        # Publishers
        self.safe_action_pub = self.create_publisher(
            String, '/safety/approved_action', 10
        )
        self.rejection_pub = self.create_publisher(
            String, '/safety/rejection', 10
        )

        self.get_logger().info('Safety filter ready')

    def action_callback(self, msg: String):
        """Check action against all safety constraints."""
        action = json.loads(msg.data)
        context = self.build_context()

        result = SafetyCheckResult(is_safe=True)

        # Check all constraints
        for constraint in self.constraints:
            violations = constraint.check(action, context)
            for v in violations:
                result.add_violation(v)
                self.get_logger().warn(
                    f'Safety: {constraint.name} - {v.message}'
                )

        if result.is_safe:
            # Publish approved action
            self.safe_action_pub.publish(msg)
            self.get_logger().info('Action approved by safety filter')
        else:
            # Publish rejection with details
            rejection = {
                'original_action': action,
                'violations': [
                    {
                        'constraint': v.constraint_name,
                        'level': v.level.name,
                        'message': v.message,
                        'suggested_fix': v.suggested_fix
                    }
                    for v in result.violations
                    if v.level >= SafetyLevel.VIOLATION
                ]
            }
            rejection_msg = String()
            rejection_msg.data = json.dumps(rejection)
            self.rejection_pub.publish(rejection_msg)
```

---

## Workspace Boundary Checking {#workspace}

### Cartesian Workspace Limits

```python
class WorkspaceBoundaryConstraint(SafetyConstraint):
    """Ensure positions are within robot workspace."""

    def __init__(self):
        # Define workspace as axis-aligned bounding box
        # Values should come from robot URDF/configuration
        self.workspace_min = np.array([0.1, -0.6, 0.0])   # x, y, z min
        self.workspace_max = np.array([0.9, 0.6, 1.2])    # x, y, z max

        # Danger zones (e.g., near electronics, human workspace)
        self.danger_zones = [
            {'center': np.array([0.5, 0.5, 0.8]), 'radius': 0.1},  # Example
        ]

    @property
    def name(self) -> str:
        return "workspace_boundary"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Extract target position from action
        position = self.extract_position(action)
        if position is None:
            return violations  # No position to check

        position = np.array(position)

        # Check bounding box
        if np.any(position < self.workspace_min):
            axis = ['x', 'y', 'z'][np.argmin(position - self.workspace_min)]
            violations.append(SafetyViolation(
                constraint_name=self.name,
                level=SafetyLevel.VIOLATION,
                message=f"Position {axis}={position[['x','y','z'].index(axis)]:.2f} "
                        f"below minimum {self.workspace_min[['x','y','z'].index(axis)]:.2f}",
                details={'position': position.tolist(), 'limit': 'min'},
                suggested_fix=f"Increase {axis} to at least {self.workspace_min[['x','y','z'].index(axis)]:.2f}"
            ))

        if np.any(position > self.workspace_max):
            axis = ['x', 'y', 'z'][np.argmax(position - self.workspace_max)]
            violations.append(SafetyViolation(
                constraint_name=self.name,
                level=SafetyLevel.VIOLATION,
                message=f"Position {axis}={position[['x','y','z'].index(axis)]:.2f} "
                        f"above maximum {self.workspace_max[['x','y','z'].index(axis)]:.2f}",
                details={'position': position.tolist(), 'limit': 'max'},
                suggested_fix=f"Decrease {axis} to at most {self.workspace_max[['x','y','z'].index(axis)]:.2f}"
            ))

        # Check danger zones
        for i, zone in enumerate(self.danger_zones):
            distance = np.linalg.norm(position - zone['center'])
            if distance < zone['radius']:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"Position inside danger zone {i}",
                    details={'zone': i, 'distance': distance},
                    suggested_fix="Move target position away from restricted area"
                ))

        return violations

    def extract_position(self, action: dict) -> Optional[List[float]]:
        """Extract target position from various action formats."""
        # From goal pose
        if 'goal' in action and 'pose' in action['goal']:
            pos = action['goal']['pose'].get('position')
            if pos:
                return [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]

        # From parameters
        params = action.get('parameters', {})
        if 'position' in params:
            return params['position']

        return None
```

### Joint Limit Checking

```python
class JointLimitConstraint(SafetyConstraint):
    """Ensure joint positions are within limits."""

    def __init__(self):
        # Load from URDF or configuration
        self.joint_limits = {
            'joint1': {'min': -3.14, 'max': 3.14},
            'joint2': {'min': -2.0, 'max': 2.0},
            'joint3': {'min': -2.5, 'max': 2.5},
            'joint4': {'min': -3.14, 'max': 3.14},
            'joint5': {'min': -2.0, 'max': 2.0},
            'joint6': {'min': -3.14, 'max': 3.14},
        }

    @property
    def name(self) -> str:
        return "joint_limits"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Check if action specifies joint targets
        joint_targets = action.get('joint_positions', {})

        for joint_name, target in joint_targets.items():
            if joint_name not in self.joint_limits:
                continue

            limits = self.joint_limits[joint_name]
            if target < limits['min']:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"{joint_name} target {target:.2f} below minimum {limits['min']:.2f}",
                    suggested_fix=f"Set {joint_name} to at least {limits['min']:.2f}"
                ))
            elif target > limits['max']:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"{joint_name} target {target:.2f} above maximum {limits['max']:.2f}",
                    suggested_fix=f"Set {joint_name} to at most {limits['max']:.2f}"
                ))

        return violations
```

---

## Velocity and Acceleration Limits {#velocity}

### Motion Limit Checking

```python
class VelocityLimitConstraint(SafetyConstraint):
    """Ensure velocities are within safe limits."""

    def __init__(self):
        # Cartesian velocity limits (m/s)
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0  # rad/s

        # Joint velocity limits (rad/s)
        self.max_joint_velocity = {
            'joint1': 2.0,
            'joint2': 2.0,
            'joint3': 2.0,
            'joint4': 3.0,
            'joint5': 3.0,
            'joint6': 3.0,
        }

        # Reduced limits when near humans
        self.human_proximity_factor = 0.5

    @property
    def name(self) -> str:
        return "velocity_limits"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Check requested speed
        speed = action.get('parameters', {}).get('speed')
        if speed is not None:
            # Adjust limit if human nearby
            effective_limit = self.max_linear_velocity
            if context.get('human_nearby', False):
                effective_limit *= self.human_proximity_factor

            if speed > effective_limit:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"Requested speed {speed:.2f} m/s exceeds "
                            f"limit {effective_limit:.2f} m/s",
                    details={
                        'requested': speed,
                        'limit': effective_limit,
                        'human_nearby': context.get('human_nearby', False)
                    },
                    suggested_fix=f"Reduce speed to {effective_limit:.2f} m/s or less"
                ))

        return violations


class AccelerationLimitConstraint(SafetyConstraint):
    """Ensure accelerations are within safe limits."""

    def __init__(self):
        self.max_linear_acceleration = 1.0   # m/s²
        self.max_angular_acceleration = 2.0  # rad/s²

    @property
    def name(self) -> str:
        return "acceleration_limits"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Check acceleration parameter if specified
        accel = action.get('parameters', {}).get('acceleration')
        if accel is not None and accel > self.max_linear_acceleration:
            violations.append(SafetyViolation(
                constraint_name=self.name,
                level=SafetyLevel.VIOLATION,
                message=f"Requested acceleration {accel:.2f} m/s² exceeds "
                        f"limit {self.max_linear_acceleration:.2f} m/s²",
                suggested_fix=f"Reduce acceleration to {self.max_linear_acceleration:.2f} m/s² or less"
            ))

        return violations
```

---

## Collision Checking {#collision}

### Perception-Based Collision Avoidance

```python
class CollisionConstraint(SafetyConstraint):
    """Check for potential collisions using perception data."""

    def __init__(self):
        self.safety_margin = 0.05  # 5cm clearance
        self.robot_collision_spheres = [
            # Simplified robot model as spheres
            {'link': 'base_link', 'radius': 0.15, 'offset': [0, 0, 0.1]},
            {'link': 'link1', 'radius': 0.08, 'offset': [0, 0, 0]},
            {'link': 'link2', 'radius': 0.08, 'offset': [0, 0, 0]},
            {'link': 'ee_link', 'radius': 0.05, 'offset': [0, 0, 0]},
        ]

    @property
    def name(self) -> str:
        return "collision"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Get scene objects from perception
        scene_objects = context.get('scene_objects', [])
        target_position = self.extract_target_position(action)

        if target_position is None:
            return violations

        # Check distance to each detected object
        for obj in scene_objects:
            obj_pos = np.array(obj['position'])
            obj_radius = obj.get('radius', 0.1)  # Estimated object size

            distance = np.linalg.norm(target_position - obj_pos)
            min_clearance = self.safety_margin + obj_radius

            if distance < min_clearance:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"Target position too close to {obj.get('class', 'object')} "
                            f"(distance: {distance:.2f}m, required: {min_clearance:.2f}m)",
                    details={
                        'object': obj.get('class', 'unknown'),
                        'distance': distance,
                        'required_clearance': min_clearance
                    },
                    suggested_fix="Adjust target position away from obstacle"
                ))

        # Check for collision along path (simplified)
        current_position = context.get('ee_position')
        if current_position is not None:
            path_collisions = self.check_path_collision(
                np.array(current_position),
                target_position,
                scene_objects
            )
            violations.extend(path_collisions)

        return violations

    def check_path_collision(
        self,
        start: np.ndarray,
        end: np.ndarray,
        obstacles: List[dict]
    ) -> List[SafetyViolation]:
        """Check for collisions along straight-line path."""
        violations = []

        # Sample points along path
        num_samples = 10
        for i in range(num_samples):
            t = i / (num_samples - 1)
            point = start + t * (end - start)

            for obj in obstacles:
                obj_pos = np.array(obj['position'])
                obj_radius = obj.get('radius', 0.1)
                distance = np.linalg.norm(point - obj_pos)

                if distance < (self.safety_margin + obj_radius):
                    violations.append(SafetyViolation(
                        constraint_name=self.name,
                        level=SafetyLevel.VIOLATION,
                        message=f"Path collision with {obj.get('class', 'object')} "
                                f"at t={t:.1f}",
                        details={'path_fraction': t, 'object': obj.get('class')},
                        suggested_fix="Plan path around obstacle"
                    ))
                    break  # One path collision is enough

        return violations

    def extract_target_position(self, action: dict) -> Optional[np.ndarray]:
        """Extract target position from action."""
        params = action.get('parameters', {})
        if 'position' in params:
            return np.array(params['position'])

        goal = action.get('goal', {})
        if 'pose' in goal:
            pos = goal['pose'].get('position', {})
            return np.array([pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)])

        return None
```

---

## Force Limits and Contact Safety {#force}

### Force-Limited Operations

```python
class ForceLimitConstraint(SafetyConstraint):
    """Ensure force commands are within safe limits."""

    def __init__(self):
        # Maximum forces by context
        self.max_grip_force_default = 50.0  # N
        self.max_grip_force_fragile = 10.0  # N
        self.max_contact_force = 20.0       # N

        # Fragile object classes
        self.fragile_classes = {'glass', 'ceramic', 'egg', 'bulb', 'phone'}

    @property
    def name(self) -> str:
        return "force_limits"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        # Check grip force
        action_type = action.get('action_type')
        params = action.get('parameters', {})

        if action_type == 'close_gripper':
            requested_force = params.get('force', self.max_grip_force_default)
            target_object = context.get('held_object_class')

            # Determine appropriate limit
            if target_object in self.fragile_classes:
                max_force = self.max_grip_force_fragile
            else:
                max_force = self.max_grip_force_default

            if requested_force > max_force:
                violations.append(SafetyViolation(
                    constraint_name=self.name,
                    level=SafetyLevel.VIOLATION,
                    message=f"Grip force {requested_force:.1f}N exceeds "
                            f"limit {max_force:.1f}N for {target_object or 'object'}",
                    details={
                        'requested': requested_force,
                        'limit': max_force,
                        'object_class': target_object
                    },
                    suggested_fix=f"Reduce grip force to {max_force:.1f}N or less"
                ))

        return violations
```

### Payload Checking

```python
class PayloadConstraint(SafetyConstraint):
    """Ensure payload is within robot capacity."""

    def __init__(self):
        self.max_payload_kg = 5.0

        # Known object weights (from perception or database)
        self.known_weights = {
            'mug': 0.3,
            'bottle': 0.5,
            'book': 0.8,
            'laptop': 2.5,
            'box': 3.0,  # Default box weight
        }

    @property
    def name(self) -> str:
        return "payload"

    def check(
        self,
        action: dict,
        context: dict
    ) -> List[SafetyViolation]:
        violations = []

        if action.get('action_type') != 'pick':
            return violations

        target = action.get('target', '')

        # Estimate weight
        estimated_weight = self.estimate_weight(target, context)

        if estimated_weight > self.max_payload_kg:
            violations.append(SafetyViolation(
                constraint_name=self.name,
                level=SafetyLevel.VIOLATION,
                message=f"Estimated payload {estimated_weight:.1f}kg exceeds "
                        f"limit {self.max_payload_kg:.1f}kg",
                details={
                    'estimated_weight': estimated_weight,
                    'limit': self.max_payload_kg,
                    'target': target
                },
                suggested_fix="Cannot pick this object - too heavy"
            ))

        return violations

    def estimate_weight(self, target: str, context: dict) -> float:
        """Estimate object weight from class and perception."""
        # Check known weights
        for known_class, weight in self.known_weights.items():
            if known_class in target.lower():
                return weight

        # Default to unknown weight warning
        return 1.0  # Conservative default
```

---

## Safety Rejection Responses {#responses}

### Informative Rejection Messages

```python
class SafetyResponseGenerator:
    """Generate informative responses for safety rejections."""

    def generate_response(
        self,
        original_command: str,
        violations: List[SafetyViolation]
    ) -> str:
        """Generate user-friendly rejection message."""
        if not violations:
            return "Command approved."

        # Group by severity
        critical = [v for v in violations if v.level == SafetyLevel.CRITICAL]
        blocking = [v for v in violations if v.level == SafetyLevel.VIOLATION]

        if critical:
            return self.format_critical(critical)

        if blocking:
            return self.format_blocking(blocking)

        return "Command has warnings but can proceed."

    def format_critical(self, violations: List[SafetyViolation]) -> str:
        """Format critical safety message."""
        messages = [v.message for v in violations]
        return f"EMERGENCY STOP: {'; '.join(messages)}"

    def format_blocking(self, violations: List[SafetyViolation]) -> str:
        """Format blocking violation message."""
        response_parts = ["I cannot execute this command safely:"]

        for v in violations[:3]:  # Limit to 3 most important
            response_parts.append(f"  - {v.message}")
            if v.suggested_fix:
                response_parts.append(f"    Suggestion: {v.suggested_fix}")

        if len(violations) > 3:
            response_parts.append(f"  - ...and {len(violations) - 3} more issues")

        return "\n".join(response_parts)
```

### Speech Synthesis Integration

```python
class SafetySpeechNode(Node):
    """Announce safety rejections via speech."""

    def __init__(self):
        super().__init__('safety_speech')

        self.rejection_sub = self.create_subscription(
            String, '/safety/rejection', self.rejection_callback, 10
        )

        self.speech_pub = self.create_publisher(
            String, '/tts/input', 10
        )

        self.response_generator = SafetyResponseGenerator()

    def rejection_callback(self, msg: String):
        """Convert rejection to speech."""
        rejection = json.loads(msg.data)

        violations = [
            SafetyViolation(
                constraint_name=v['constraint'],
                level=SafetyLevel[v['level']],
                message=v['message'],
                suggested_fix=v.get('suggested_fix')
            )
            for v in rejection.get('violations', [])
        ]

        speech_text = self.response_generator.generate_response(
            rejection.get('original_command', ''),
            violations
        )

        speech_msg = String()
        speech_msg.data = speech_text
        self.speech_pub.publish(speech_msg)

        self.get_logger().info(f'Safety announcement: {speech_text}')
```

---

## Runtime Safety Monitoring {#monitoring}

### Continuous Safety Watch

```python
class RuntimeSafetyMonitor(Node):
    """Continuous monitoring during execution."""

    def __init__(self):
        super().__init__('runtime_safety')

        # State
        self.is_executing = False
        self.current_action = None

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.wrench_sub = self.create_subscription(
            WrenchStamped, '/ft_sensor', self.wrench_callback, 10
        )

        # Emergency stop publisher
        self.estop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )

        # Limits
        self.force_threshold = 50.0  # N
        self.torque_threshold = 10.0  # Nm

        # Timer for periodic checks
        self.create_timer(0.01, self.monitor_callback)  # 100 Hz

        self.get_logger().info('Runtime safety monitor active')

    def wrench_callback(self, msg: WrenchStamped):
        """Check force/torque limits."""
        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])
        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

        force_magnitude = np.linalg.norm(force)
        torque_magnitude = np.linalg.norm(torque)

        if force_magnitude > self.force_threshold:
            self.trigger_emergency_stop(
                f"Force limit exceeded: {force_magnitude:.1f}N > {self.force_threshold:.1f}N"
            )

        if torque_magnitude > self.torque_threshold:
            self.trigger_emergency_stop(
                f"Torque limit exceeded: {torque_magnitude:.1f}Nm > {self.torque_threshold:.1f}Nm"
            )

    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop."""
        self.get_logger().error(f'EMERGENCY STOP: {reason}')

        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

        # Log incident
        self.log_safety_incident(reason)

    def log_safety_incident(self, reason: str):
        """Log safety incident for analysis."""
        import time
        incident = {
            'timestamp': time.time(),
            'reason': reason,
            'action': self.current_action,
            'robot_state': 'captured_state_here'
        }
        self.get_logger().error(f'Safety incident: {json.dumps(incident)}')
```

---

## Summary

This chapter covered safety constraints for AI-generated robot commands:

1. **Safety philosophy** requires constraints to override AI outputs—refuse rather than act unsafely.

2. **Defense in depth** uses multiple layers: prompt constraints, schema validation, safety filter, runtime monitoring.

3. **Workspace checking** validates positions against bounds and danger zones.

4. **Velocity/acceleration limits** prevent dangerous motion speeds, with dynamic adjustment for human proximity.

5. **Collision checking** uses perception to verify clearance along planned paths.

6. **Force limits** prevent damage to objects and robot, with context-aware thresholds.

7. **Runtime monitoring** provides continuous safety supervision during execution.

---

## Safety Callout

**This safety layer is CRITICAL but not SUFFICIENT:**

- It assumes perception is accurate (but VLM can hallucinate)
- It assumes workspace is calibrated (but calibration drifts)
- It assumes force sensors work (but sensors fail)

**For real deployment, add:**
- Redundant sensors
- Hardware e-stop
- Human supervision
- Regular calibration verification

---

## Self-Assessment Questions

1. **Defense in Depth**: The LLM prompt says "max speed 0.5 m/s" but the safety filter also checks. Why both?

2. **False Positives**: Your collision check rejects a valid command because perception falsely detected an obstacle. How do you balance safety vs. functionality?

3. **Human Proximity**: The robot is 2 meters from a human. Should velocity limits be reduced? At what distance?

4. **Payload Estimation**: You don't know the weight of an unknown object. Should you attempt to pick it? What's the safe approach?

5. **Emergency Stop**: The force sensor reads 60N during a pick operation. Should you immediately stop, or could this be normal contact?

---

## What's Next

In [Chapter 6: Failure Handling](/module-5/chapter-6-failure-handling), you'll implement robust failure detection and recovery strategies for when commands fail despite passing safety checks.
