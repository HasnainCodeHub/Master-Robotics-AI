---
id: chapter-6-failure-handling
title: "Chapter 6: Failure Handling"
sidebar_label: "6. Failure Handling"
sidebar_position: 7
---

# Chapter 6: Failure Handling and Recovery Strategies

## Chapter Goal

By the end of this chapter, you will be able to **implement comprehensive failure detection and recovery strategies for VLA systems**, including graceful degradation, human-in-the-loop recovery, and robust error reporting.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 6.1 | Classify failure modes in VLA pipelines |
| 6.2 | Implement failure detection at each pipeline stage |
| 6.3 | Design recovery strategies for common failures |
| 6.4 | Implement human-in-the-loop recovery |
| 6.5 | Design graceful degradation modes |
| 6.6 | Implement comprehensive error logging and analysis |

---

## VLA Failure Taxonomy {#taxonomy}

### Pipeline Stage Failures

```
┌─────────────────────────────────────────────────────────────────────┐
│                        VLA FAILURE MODES                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Speech ──► LLM ──► Grounding ──► Safety ──► Execution ──► Monitor  │
│    │         │          │           │            │           │      │
│    ▼         ▼          ▼           ▼            ▼           ▼      │
│  - Noise   - Timeout  - Not found - Rejected  - Collision  - Drift  │
│  - Silence - Invalid  - Ambiguous              - Timeout   - Stuck  │
│  - Wrong   - Refusal  - Out of                 - Hardware           │
│    text               reach                     failure             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Failure Classification

| Category | Examples | Severity | Recovery |
|----------|----------|----------|----------|
| Transient | Network timeout, brief occlusion | Low | Retry |
| Recoverable | Ambiguous reference, unreachable | Medium | Clarification |
| Blocking | Object not found, safety rejection | High | Human help |
| Critical | Hardware fault, collision | Critical | Stop, escalate |

---

## Failure Detection {#detection}

### Pipeline Health Monitor

```python
#!/usr/bin/env python3
"""VLA pipeline health monitoring and failure detection."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable
from enum import Enum
import time
import json


class FailureType(Enum):
    """Types of failures in VLA pipeline."""
    SPEECH_TIMEOUT = "speech_timeout"
    SPEECH_NOISE = "speech_noise"
    SPEECH_EMPTY = "speech_empty"
    LLM_TIMEOUT = "llm_timeout"
    LLM_INVALID = "llm_invalid"
    LLM_REFUSAL = "llm_refusal"
    GROUNDING_NOT_FOUND = "grounding_not_found"
    GROUNDING_AMBIGUOUS = "grounding_ambiguous"
    GROUNDING_UNREACHABLE = "grounding_unreachable"
    SAFETY_REJECTED = "safety_rejected"
    EXECUTION_TIMEOUT = "execution_timeout"
    EXECUTION_COLLISION = "execution_collision"
    EXECUTION_HARDWARE = "execution_hardware"
    MONITORING_DRIFT = "monitoring_drift"


@dataclass
class Failure:
    """Details of a detected failure."""
    failure_type: FailureType
    stage: str
    message: str
    timestamp: float = field(default_factory=time.time)
    context: Dict = field(default_factory=dict)
    recoverable: bool = True
    retry_count: int = 0


class FailureDetector:
    """Detect failures at each pipeline stage."""

    def __init__(self):
        self.stage_timeouts = {
            'speech': 30.0,    # 30s to speak
            'llm': 10.0,       # 10s for LLM response
            'grounding': 5.0,  # 5s for grounding
            'execution': 60.0, # 60s for action execution
        }

        self.stage_start_times: Dict[str, float] = {}
        self.pending_failures: List[Failure] = []

    def start_stage(self, stage: str):
        """Mark stage as started for timeout detection."""
        self.stage_start_times[stage] = time.time()

    def check_timeout(self, stage: str) -> Optional[Failure]:
        """Check if stage has timed out."""
        if stage not in self.stage_start_times:
            return None

        elapsed = time.time() - self.stage_start_times[stage]
        timeout = self.stage_timeouts.get(stage, 30.0)

        if elapsed > timeout:
            return Failure(
                failure_type=FailureType[f"{stage.upper()}_TIMEOUT"],
                stage=stage,
                message=f"{stage} timed out after {elapsed:.1f}s (limit: {timeout}s)",
                context={'elapsed': elapsed, 'timeout': timeout}
            )

        return None

    def detect_speech_failure(self, result: dict) -> Optional[Failure]:
        """Detect speech recognition failures."""
        text = result.get('text', '').strip()

        if not text:
            return Failure(
                failure_type=FailureType.SPEECH_EMPTY,
                stage='speech',
                message="No speech detected",
                context=result
            )

        confidence = result.get('confidence', 1.0)
        if confidence < 0.5:
            return Failure(
                failure_type=FailureType.SPEECH_NOISE,
                stage='speech',
                message=f"Low confidence transcription ({confidence:.2f})",
                context={'text': text, 'confidence': confidence}
            )

        return None

    def detect_llm_failure(self, response: dict) -> Optional[Failure]:
        """Detect LLM planning failures."""
        if 'error' in response:
            return Failure(
                failure_type=FailureType.LLM_INVALID,
                stage='llm',
                message=f"LLM error: {response['error']}",
                context=response
            )

        if response.get('clarification_needed'):
            return Failure(
                failure_type=FailureType.LLM_REFUSAL,
                stage='llm',
                message=f"LLM needs clarification: {response['clarification_needed']}",
                context=response,
                recoverable=True
            )

        actions = response.get('actions', [])
        if not actions:
            return Failure(
                failure_type=FailureType.LLM_INVALID,
                stage='llm',
                message="LLM produced no actions",
                context=response
            )

        return None

    def detect_grounding_failure(self, result: dict) -> Optional[Failure]:
        """Detect grounding failures."""
        if result.get('action') == 'wait_for_assistance':
            return Failure(
                failure_type=FailureType.GROUNDING_NOT_FOUND,
                stage='grounding',
                message="Target object not found",
                context=result,
                recoverable=True
            )

        if result.get('action') == 'wait_for_clarification':
            return Failure(
                failure_type=FailureType.GROUNDING_AMBIGUOUS,
                stage='grounding',
                message="Ambiguous object reference",
                context=result,
                recoverable=True
            )

        if result.get('action') == 'navigate_closer':
            return Failure(
                failure_type=FailureType.GROUNDING_UNREACHABLE,
                stage='grounding',
                message="Target position not reachable from current location",
                context=result,
                recoverable=True
            )

        return None
```

### Stage-Specific Monitors

```python
class SpeechMonitor:
    """Monitor speech recognition stage."""

    def __init__(self):
        self.last_speech_time = None
        self.consecutive_failures = 0
        self.noise_level_history = []

    def on_speech_result(self, result: dict) -> Optional[Failure]:
        """Process speech result and detect failures."""
        self.last_speech_time = time.time()

        # Check for repeated failures
        if result.get('confidence', 1.0) < 0.3:
            self.consecutive_failures += 1
            if self.consecutive_failures >= 3:
                return Failure(
                    failure_type=FailureType.SPEECH_NOISE,
                    stage='speech',
                    message="Repeated low-confidence transcriptions - "
                            "possible microphone or noise issue",
                    context={
                        'consecutive_failures': self.consecutive_failures
                    }
                )
        else:
            self.consecutive_failures = 0

        return None


class ExecutionMonitor:
    """Monitor action execution stage."""

    def __init__(self):
        self.action_start_time = None
        self.expected_duration = None
        self.position_history = []

    def on_action_start(self, action: dict, expected_duration: float):
        """Record action start."""
        self.action_start_time = time.time()
        self.expected_duration = expected_duration
        self.position_history = []

    def on_position_update(self, position: list):
        """Track position for stuck detection."""
        self.position_history.append({
            'position': position,
            'time': time.time()
        })

        # Keep only recent history
        cutoff = time.time() - 5.0
        self.position_history = [
            p for p in self.position_history
            if p['time'] > cutoff
        ]

    def check_stuck(self) -> Optional[Failure]:
        """Detect if robot is stuck."""
        if len(self.position_history) < 10:
            return None

        # Check if position hasn't changed
        positions = [p['position'] for p in self.position_history]
        pos_array = np.array(positions)

        movement = np.max(pos_array, axis=0) - np.min(pos_array, axis=0)
        max_movement = np.max(movement)

        if max_movement < 0.001:  # Less than 1mm movement in 5 seconds
            return Failure(
                failure_type=FailureType.MONITORING_DRIFT,
                stage='execution',
                message="Robot appears stuck - no movement detected",
                context={
                    'max_movement': max_movement,
                    'duration': 5.0
                }
            )

        return None
```

---

## Recovery Strategies {#recovery}

### Recovery Strategy Registry

```python
class RecoveryStrategy:
    """Base class for recovery strategies."""

    @property
    def applicable_failures(self) -> List[FailureType]:
        """Which failure types this strategy handles."""
        raise NotImplementedError

    def can_recover(self, failure: Failure, context: dict) -> bool:
        """Check if recovery is possible."""
        return (
            failure.failure_type in self.applicable_failures and
            failure.retry_count < 3
        )

    def recover(self, failure: Failure, context: dict) -> dict:
        """
        Attempt recovery.

        Returns:
            Recovery action dict or None if unrecoverable
        """
        raise NotImplementedError


class RetryStrategy(RecoveryStrategy):
    """Simple retry for transient failures."""

    @property
    def applicable_failures(self) -> List[FailureType]:
        return [
            FailureType.SPEECH_TIMEOUT,
            FailureType.LLM_TIMEOUT,
            FailureType.EXECUTION_TIMEOUT,
        ]

    def recover(self, failure: Failure, context: dict) -> dict:
        failure.retry_count += 1
        return {
            'action': 'retry',
            'stage': failure.stage,
            'delay': 1.0 * failure.retry_count,  # Backoff
            'message': f"Retrying {failure.stage} (attempt {failure.retry_count})"
        }


class ClarificationStrategy(RecoveryStrategy):
    """Request clarification from user."""

    @property
    def applicable_failures(self) -> List[FailureType]:
        return [
            FailureType.GROUNDING_AMBIGUOUS,
            FailureType.LLM_REFUSAL,
            FailureType.SPEECH_NOISE,
        ]

    def recover(self, failure: Failure, context: dict) -> dict:
        # Generate appropriate question based on failure
        if failure.failure_type == FailureType.GROUNDING_AMBIGUOUS:
            candidates = failure.context.get('candidates', [])
            question = self.format_disambiguation_question(candidates)
        elif failure.failure_type == FailureType.SPEECH_NOISE:
            question = "I didn't catch that clearly. Could you repeat your command?"
        else:
            question = "I need more information. Could you clarify what you'd like me to do?"

        return {
            'action': 'request_clarification',
            'question': question,
            'timeout': 30.0,
            'failure_context': failure.context
        }

    def format_disambiguation_question(self, candidates: list) -> str:
        """Format question for ambiguous objects."""
        if not candidates:
            return "Which object do you mean?"

        descriptions = []
        for i, c in enumerate(candidates, 1):
            desc = f"{i}. {c.get('color', '')} {c.get('class', 'object')}"
            if c.get('location'):
                desc += f" ({c['location']})"
            descriptions.append(desc)

        return f"I see multiple objects: {', '.join(descriptions)}. Which one?"


class NavigateCloserStrategy(RecoveryStrategy):
    """Navigate closer to unreachable target."""

    @property
    def applicable_failures(self) -> List[FailureType]:
        return [FailureType.GROUNDING_UNREACHABLE]

    def recover(self, failure: Failure, context: dict) -> dict:
        target_pos = failure.context.get('target_position')

        if target_pos is None:
            return {
                'action': 'request_assistance',
                'message': "I cannot reach that location and don't know where to go."
            }

        # Calculate approach position
        approach_pos = self.calculate_approach_position(
            target_pos,
            context.get('robot_position')
        )

        return {
            'action': 'navigate_to',
            'position': approach_pos,
            'then_retry': True,
            'message': "Moving closer to reach the target."
        }

    def calculate_approach_position(
        self,
        target: list,
        current: list
    ) -> list:
        """Calculate position closer to target."""
        target = np.array(target)
        current = np.array(current) if current else np.zeros(3)

        # Move 80% of the way toward target
        direction = target - current
        approach = current + 0.8 * direction

        return approach.tolist()


class AssistanceStrategy(RecoveryStrategy):
    """Request human assistance for unrecoverable failures."""

    @property
    def applicable_failures(self) -> List[FailureType]:
        return [
            FailureType.GROUNDING_NOT_FOUND,
            FailureType.EXECUTION_COLLISION,
            FailureType.EXECUTION_HARDWARE,
        ]

    def can_recover(self, failure: Failure, context: dict) -> bool:
        # Always escalate hardware failures
        if failure.failure_type == FailureType.EXECUTION_HARDWARE:
            return True
        # Escalate after retries exhausted
        return failure.retry_count >= 2

    def recover(self, failure: Failure, context: dict) -> dict:
        return {
            'action': 'request_assistance',
            'message': self.format_assistance_message(failure),
            'severity': 'high' if failure.failure_type == FailureType.EXECUTION_HARDWARE else 'medium',
            'requires_acknowledgment': True
        }

    def format_assistance_message(self, failure: Failure) -> str:
        """Format message for human assistance."""
        if failure.failure_type == FailureType.GROUNDING_NOT_FOUND:
            target = failure.context.get('target', 'the object')
            return f"I cannot find {target}. Can you show me where it is?"

        if failure.failure_type == FailureType.EXECUTION_COLLISION:
            return "I detected a collision and stopped. Please check if it's safe to continue."

        if failure.failure_type == FailureType.EXECUTION_HARDWARE:
            return "I'm experiencing a hardware issue and need assistance."

        return "I need help to continue."
```

### Recovery Orchestrator

```python
class RecoveryOrchestrator:
    """Coordinate recovery across strategies."""

    def __init__(self):
        self.strategies = [
            RetryStrategy(),
            ClarificationStrategy(),
            NavigateCloserStrategy(),
            AssistanceStrategy(),
        ]

        self.failure_history: List[Failure] = []
        self.max_history = 100

    def handle_failure(
        self,
        failure: Failure,
        context: dict
    ) -> dict:
        """Find and execute appropriate recovery."""
        self.failure_history.append(failure)
        if len(self.failure_history) > self.max_history:
            self.failure_history.pop(0)

        # Find applicable strategy
        for strategy in self.strategies:
            if strategy.can_recover(failure, context):
                recovery = strategy.recover(failure, context)
                recovery['strategy'] = strategy.__class__.__name__
                return recovery

        # No recovery possible
        return {
            'action': 'abort',
            'message': f"Unrecoverable failure: {failure.message}",
            'failure': failure
        }

    def get_failure_statistics(self) -> dict:
        """Get statistics on recent failures."""
        if not self.failure_history:
            return {'total': 0}

        by_type = {}
        for f in self.failure_history:
            type_name = f.failure_type.value
            by_type[type_name] = by_type.get(type_name, 0) + 1

        return {
            'total': len(self.failure_history),
            'by_type': by_type,
            'most_common': max(by_type, key=by_type.get) if by_type else None
        }
```

---

## Human-in-the-Loop Recovery {#human-loop}

### Human Interaction Manager

```python
class HumanInteractionManager(Node):
    """Manage human-in-the-loop recovery interactions."""

    def __init__(self):
        super().__init__('human_interaction')

        # State
        self.pending_requests: Dict[str, dict] = {}
        self.interaction_timeout = 60.0  # seconds

        # Publishers
        self.speech_pub = self.create_publisher(
            String, '/tts/input', 10
        )
        self.display_pub = self.create_publisher(
            String, '/ui/display', 10
        )

        # Subscribers
        self.speech_input_sub = self.create_subscription(
            String, '/speech/text', self.speech_response_callback, 10
        )
        self.button_sub = self.create_subscription(
            String, '/ui/button', self.button_callback, 10
        )

        # Response callback
        self.response_callbacks: Dict[str, Callable] = {}

        self.get_logger().info('Human interaction manager ready')

    def request_clarification(
        self,
        request_id: str,
        question: str,
        options: Optional[List[str]] = None,
        callback: Optional[Callable] = None
    ):
        """Request clarification from human."""
        self.pending_requests[request_id] = {
            'question': question,
            'options': options,
            'timestamp': time.time(),
            'type': 'clarification'
        }

        if callback:
            self.response_callbacks[request_id] = callback

        # Speak question
        self.speak(question)

        # Display options if available
        if options:
            self.display_options(question, options)

        # Set timeout
        self.create_timer(
            self.interaction_timeout,
            lambda: self.handle_timeout(request_id)
        )

    def request_assistance(
        self,
        request_id: str,
        message: str,
        severity: str = 'medium',
        callback: Optional[Callable] = None
    ):
        """Request human assistance."""
        self.pending_requests[request_id] = {
            'message': message,
            'severity': severity,
            'timestamp': time.time(),
            'type': 'assistance'
        }

        if callback:
            self.response_callbacks[request_id] = callback

        # Alert based on severity
        if severity == 'high':
            self.speak(f"Attention! {message}")
            self.display_alert(message, 'error')
        else:
            self.speak(message)
            self.display_alert(message, 'warning')

    def speech_response_callback(self, msg: String):
        """Handle speech response from human."""
        response_text = msg.data.strip().lower()

        # Match to pending requests
        for request_id, request in list(self.pending_requests.items()):
            if request['type'] == 'clarification':
                # Try to match response to options
                matched = self.match_response(
                    response_text,
                    request.get('options', [])
                )

                if matched is not None:
                    self.resolve_request(request_id, matched)
                    return

        # No match - might be new command
        self.get_logger().info(f'Unmatched speech: {response_text}')

    def match_response(
        self,
        response: str,
        options: List[str]
    ) -> Optional[str]:
        """Match response to available options."""
        # Direct match
        for opt in options:
            if opt.lower() in response or response in opt.lower():
                return opt

        # Number match ("the first one", "number 2")
        numbers = ['first', 'second', 'third', '1', '2', '3', 'one', 'two', 'three']
        for i, num in enumerate(numbers):
            if num in response and i % 3 < len(options):
                return options[i % 3]

        return None

    def resolve_request(self, request_id: str, response: Any):
        """Resolve pending request with response."""
        if request_id in self.pending_requests:
            del self.pending_requests[request_id]

        if request_id in self.response_callbacks:
            callback = self.response_callbacks.pop(request_id)
            callback(response)

    def handle_timeout(self, request_id: str):
        """Handle interaction timeout."""
        if request_id not in self.pending_requests:
            return  # Already resolved

        request = self.pending_requests.pop(request_id)
        self.get_logger().warn(f'Interaction timeout: {request_id}')

        if request_id in self.response_callbacks:
            callback = self.response_callbacks.pop(request_id)
            callback(None)  # Indicate timeout

    def speak(self, text: str):
        """Send text to speech synthesis."""
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)

    def display_options(self, question: str, options: List[str]):
        """Display options on UI."""
        display = {
            'type': 'options',
            'question': question,
            'options': options
        }
        msg = String()
        msg.data = json.dumps(display)
        self.display_pub.publish(msg)

    def display_alert(self, message: str, level: str):
        """Display alert on UI."""
        alert = {
            'type': 'alert',
            'message': message,
            'level': level
        }
        msg = String()
        msg.data = json.dumps(alert)
        self.display_pub.publish(msg)
```

---

## Graceful Degradation {#degradation}

### Degradation Modes

```python
class DegradationMode(Enum):
    """Operating modes with reduced capability."""
    FULL = "full"                    # All systems operational
    NO_SPEECH = "no_speech"          # Speech unavailable, text only
    NO_VLM = "no_vlm"                # VLM unavailable, detector only
    NO_NAVIGATION = "no_navigation"  # Navigation unavailable, arm only
    SAFETY_ONLY = "safety_only"      # Only safe, simple commands
    MANUAL = "manual"                # Human teleoperation only


class DegradationManager:
    """Manage graceful degradation of VLA capabilities."""

    def __init__(self):
        self.current_mode = DegradationMode.FULL
        self.component_status = {
            'speech': True,
            'llm': True,
            'vlm': True,
            'grounding': True,
            'navigation': True,
            'manipulation': True,
        }

        self.mode_capabilities = {
            DegradationMode.FULL: {'all'},
            DegradationMode.NO_SPEECH: {'text_commands', 'execution'},
            DegradationMode.NO_VLM: {'speech', 'llm', 'detector_grounding'},
            DegradationMode.NO_NAVIGATION: {'speech', 'llm', 'manipulation'},
            DegradationMode.SAFETY_ONLY: {'simple_commands'},
            DegradationMode.MANUAL: {'teleoperation'},
        }

    def report_component_failure(self, component: str):
        """Report component failure and adjust mode."""
        self.component_status[component] = False
        self.update_mode()

    def report_component_recovery(self, component: str):
        """Report component recovery."""
        self.component_status[component] = True
        self.update_mode()

    def update_mode(self):
        """Update degradation mode based on component status."""
        if all(self.component_status.values()):
            self.current_mode = DegradationMode.FULL
        elif not self.component_status['speech']:
            self.current_mode = DegradationMode.NO_SPEECH
        elif not self.component_status['vlm']:
            self.current_mode = DegradationMode.NO_VLM
        elif not self.component_status['navigation']:
            self.current_mode = DegradationMode.NO_NAVIGATION
        elif not self.component_status['llm']:
            self.current_mode = DegradationMode.SAFETY_ONLY
        else:
            self.current_mode = DegradationMode.MANUAL

    def can_execute(self, command_type: str) -> bool:
        """Check if command type can be executed in current mode."""
        capabilities = self.mode_capabilities[self.current_mode]
        return 'all' in capabilities or command_type in capabilities

    def get_alternative(self, command_type: str) -> Optional[str]:
        """Suggest alternative for unavailable command."""
        alternatives = {
            'navigate_to': "Navigation unavailable. I can only manipulate objects within reach.",
            'vlm_query': "Scene understanding unavailable. Please specify object by name and color.",
            'voice_command': "Voice unavailable. Please type your command.",
        }
        return alternatives.get(command_type)
```

### Fallback Handlers

```python
class FallbackHandler:
    """Handle operations when primary method fails."""

    def __init__(self, degradation_manager: DegradationManager):
        self.degradation = degradation_manager

    def resolve_object(
        self,
        reference: str,
        use_vlm: bool = True
    ) -> Optional[dict]:
        """Resolve object reference with fallbacks."""
        # Try VLM first if available
        if use_vlm and self.degradation.component_status['vlm']:
            result = self.vlm_resolve(reference)
            if result:
                return result

        # Fall back to detector
        if self.degradation.component_status['grounding']:
            result = self.detector_resolve(reference)
            if result:
                return result

        # Fall back to known locations
        result = self.known_location_resolve(reference)
        if result:
            return result

        return None

    def vlm_resolve(self, reference: str) -> Optional[dict]:
        """Resolve using VLM."""
        # Implementation would call VLM service
        pass

    def detector_resolve(self, reference: str) -> Optional[dict]:
        """Resolve using object detector."""
        # Implementation would call detector service
        pass

    def known_location_resolve(self, reference: str) -> Optional[dict]:
        """Resolve using known location database."""
        known_locations = {
            'home': [0.5, 0.0, 0.5],
            'shelf': [0.8, 0.3, 0.6],
            'table': [0.6, 0.0, 0.3],
        }

        for name, position in known_locations.items():
            if name in reference.lower():
                return {
                    'method': 'known_location',
                    'position': position,
                    'confidence': 0.9
                }

        return None
```

---

## Error Logging and Analysis {#logging}

### Comprehensive Error Logger

```python
class VLAErrorLogger:
    """Log errors for analysis and improvement."""

    def __init__(self, log_dir: str = '/var/log/vla'):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.session_id = self.generate_session_id()
        self.session_log = []

    def generate_session_id(self) -> str:
        """Generate unique session ID."""
        import uuid
        return f"vla_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"

    def log_failure(
        self,
        failure: Failure,
        context: dict,
        recovery_action: Optional[dict] = None
    ):
        """Log failure with full context."""
        entry = {
            'timestamp': datetime.now().isoformat(),
            'session_id': self.session_id,
            'failure': {
                'type': failure.failure_type.value,
                'stage': failure.stage,
                'message': failure.message,
                'recoverable': failure.recoverable,
                'retry_count': failure.retry_count,
                'context': failure.context
            },
            'system_context': {
                'robot_state': context.get('robot_state'),
                'scene_objects': context.get('scene_objects'),
                'recent_commands': context.get('recent_commands', [])[-5:]
            },
            'recovery': recovery_action
        }

        self.session_log.append(entry)
        self.write_log_entry(entry)

    def write_log_entry(self, entry: dict):
        """Write entry to log file."""
        log_file = self.log_dir / f"{self.session_id}.jsonl"
        with open(log_file, 'a') as f:
            f.write(json.dumps(entry) + '\n')

    def log_success(self, command: str, execution_time: float):
        """Log successful command execution."""
        entry = {
            'timestamp': datetime.now().isoformat(),
            'session_id': self.session_id,
            'type': 'success',
            'command': command,
            'execution_time': execution_time
        }
        self.session_log.append(entry)
        self.write_log_entry(entry)

    def get_session_summary(self) -> dict:
        """Get summary of current session."""
        successes = [e for e in self.session_log if e.get('type') == 'success']
        failures = [e for e in self.session_log if 'failure' in e]

        return {
            'session_id': self.session_id,
            'total_commands': len(successes) + len(failures),
            'successes': len(successes),
            'failures': len(failures),
            'success_rate': len(successes) / max(1, len(successes) + len(failures)),
            'failure_types': self.count_failure_types(failures),
            'avg_execution_time': np.mean([s['execution_time'] for s in successes]) if successes else 0
        }

    def count_failure_types(self, failures: list) -> dict:
        """Count failures by type."""
        counts = {}
        for f in failures:
            ftype = f['failure']['type']
            counts[ftype] = counts.get(ftype, 0) + 1
        return counts
```

### Analytics Dashboard Data

```python
class VLAAnalytics:
    """Analyze VLA system performance over time."""

    def __init__(self, log_dir: str = '/var/log/vla'):
        self.log_dir = Path(log_dir)

    def load_logs(self, days: int = 7) -> List[dict]:
        """Load logs from recent days."""
        entries = []
        cutoff = datetime.now() - timedelta(days=days)

        for log_file in self.log_dir.glob('*.jsonl'):
            with open(log_file) as f:
                for line in f:
                    entry = json.loads(line)
                    entry_time = datetime.fromisoformat(entry['timestamp'])
                    if entry_time > cutoff:
                        entries.append(entry)

        return entries

    def compute_metrics(self, entries: List[dict]) -> dict:
        """Compute performance metrics."""
        successes = [e for e in entries if e.get('type') == 'success']
        failures = [e for e in entries if 'failure' in e]

        return {
            'period_start': min(e['timestamp'] for e in entries) if entries else None,
            'period_end': max(e['timestamp'] for e in entries) if entries else None,
            'total_interactions': len(entries),
            'success_rate': len(successes) / max(1, len(entries)),
            'mean_time_to_failure': self.compute_mttf(entries),
            'most_common_failures': self.top_failures(failures, 5),
            'recovery_success_rate': self.recovery_rate(failures),
            'hourly_distribution': self.hourly_distribution(entries)
        }

    def compute_mttf(self, entries: List[dict]) -> float:
        """Compute mean time to failure."""
        # Time between consecutive failures
        failure_times = sorted([
            datetime.fromisoformat(e['timestamp'])
            for e in entries if 'failure' in e
        ])

        if len(failure_times) < 2:
            return float('inf')

        intervals = [
            (failure_times[i+1] - failure_times[i]).total_seconds()
            for i in range(len(failure_times) - 1)
        ]

        return np.mean(intervals)

    def top_failures(self, failures: List[dict], n: int) -> List[dict]:
        """Get most common failure types."""
        counts = {}
        for f in failures:
            ftype = f['failure']['type']
            counts[ftype] = counts.get(ftype, 0) + 1

        sorted_types = sorted(counts.items(), key=lambda x: -x[1])
        return [{'type': t, 'count': c} for t, c in sorted_types[:n]]

    def recovery_rate(self, failures: List[dict]) -> float:
        """Compute rate of successful recovery."""
        recoverable = [f for f in failures if f['failure'].get('recoverable', False)]
        recovered = [f for f in recoverable if f.get('recovery', {}).get('success', False)]

        return len(recovered) / max(1, len(recoverable))

    def hourly_distribution(self, entries: List[dict]) -> dict:
        """Get distribution of interactions by hour."""
        hours = {}
        for e in entries:
            hour = datetime.fromisoformat(e['timestamp']).hour
            hours[hour] = hours.get(hour, 0) + 1
        return hours
```

---

## Summary

This chapter covered failure handling for VLA systems:

1. **Failure taxonomy** classifies failures by stage, severity, and recoverability.

2. **Failure detection** monitors each pipeline stage for timeouts, invalid outputs, and anomalies.

3. **Recovery strategies** provide appropriate responses: retry, clarification, navigation, or human assistance.

4. **Human-in-the-loop** recovery enables disambiguation and assistance through speech and UI.

5. **Graceful degradation** maintains partial functionality when components fail.

6. **Error logging** captures comprehensive data for analysis and improvement.

---

## Self-Assessment Questions

1. **Failure Classification**: The VLM says "object on the left" but it's actually center-left, causing grounding to fail. What failure type is this? How would you recover?

2. **Retry vs. Escalate**: LLM times out for the third time. Should you retry again or escalate to human? What factors determine this?

3. **Degradation Decision**: VLM is unavailable but the user says "pick up the thing next to the keyboard." Can you proceed? How?

4. **Human Interaction**: The robot asks for clarification but the human doesn't respond for 30 seconds. What should happen?

5. **Log Analysis**: Your failure logs show 40% of failures are "grounding_ambiguous." What system improvement would you prioritize?

---

## Module 5 Complete

Congratulations! You've completed Module 5: Vision-Language-Action Systems.

You can now:
- Integrate speech recognition for voice commands
- Design LLM prompts for structured task planning
- Implement grounding to connect symbols to physical actions
- Use VLMs for flexible scene understanding
- Implement safety constraints that override AI outputs
- Handle failures gracefully with recovery and degradation

In the **Capstone: Integrated Humanoid System**, you'll apply all these capabilities to build a complete humanoid robot system that accepts natural language commands and executes them safely in the physical world.
