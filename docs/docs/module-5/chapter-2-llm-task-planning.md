---
id: chapter-2-llm-task-planning
title: "Chapter 2: LLM Task Planning"
sidebar_label: "2. LLM Task Planning"
sidebar_position: 3
---

# Chapter 2: LLM-Based Task Planning

## Chapter Goal

By the end of this chapter, you will be able to **design LLM prompts that translate natural language commands into structured robot task plans**, with emphasis on prompt engineering discipline, structured outputs, and understanding LLM limitations.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 2.1 | Explain LLM strengths and limitations for robot task planning |
| 2.2 | Design prompts producing structured task plans with explicit schemas |
| 2.3 | Implement few-shot prompting with task decomposition examples |
| 2.4 | Validate LLM outputs against schema |
| 2.5 | Implement caching for repeated commands |
| 2.6 | Design prompts including robot capability constraints |

---

## LLMs for Task Planning: Strengths and Limitations

### What LLMs Do Well

| Capability | Example |
|------------|---------|
| Semantic understanding | "grab the thing next to the keyboard" → object reference |
| Command decomposition | "clean the table" → [pick up items, wipe surface, return items] |
| Context handling | Remembering "it" refers to previous object |
| Variation handling | "pick up", "grab", "get" → same action |

### What LLMs Do Poorly

| Limitation | Example Failure |
|------------|-----------------|
| **Physical reasoning** | Plans path through solid wall |
| **Spatial awareness** | "Move left" without knowing robot orientation |
| **Constraint satisfaction** | Commands exceed joint limits |
| **Hallucination** | Invents objects not in scene |
| **Consistency** | Different plans for identical commands |

**Critical Insight**: LLMs are semantic parsers, not physical reasoners. Physical constraints must be explicit in prompts and validated after generation.

---

## Structured Output Design {#structured-output}

### Task Plan Schema

```json
{
  "command_id": "string",
  "original_text": "string",
  "actions": [
    {
      "action_type": "string",
      "target": "string | null",
      "parameters": {
        "position": [x, y, z] | null,
        "orientation": [x, y, z, w] | null,
        "speed": "number | null",
        "force": "number | null"
      },
      "preconditions": ["string"],
      "effects": ["string"]
    }
  ],
  "success_criteria": "string",
  "abort_conditions": ["string"]
}
```

### Valid Action Types

```python
VALID_ACTIONS = {
    "navigate_to": {
        "required_params": ["position"],
        "optional_params": ["speed", "orientation"]
    },
    "pick": {
        "required_params": ["target"],
        "optional_params": ["approach_direction"]
    },
    "place": {
        "required_params": ["position"],
        "optional_params": ["orientation"]
    },
    "open_gripper": {
        "required_params": [],
        "optional_params": []
    },
    "close_gripper": {
        "required_params": [],
        "optional_params": ["force"]
    },
    "wait": {
        "required_params": ["duration"],
        "optional_params": []
    }
}
```

---

## Prompt Engineering {#prompt-engineering}

### System Prompt

```python
SYSTEM_PROMPT = """You are a robot task planner. Convert natural language commands into structured action plans.

ROBOT CAPABILITIES:
- Navigate to positions within workspace (0,0,0) to (5,5,2) meters
- Maximum speed: 0.5 m/s
- Pick objects up to 2 kg
- Gripper opening: 0-10 cm
- Cannot climb stairs or open doors

AVAILABLE ACTIONS:
- navigate_to(position): Move base to position
- pick(target): Grasp target object
- place(position): Release held object at position
- open_gripper(): Open gripper fully
- close_gripper(force): Close gripper with specified force
- wait(duration): Wait for specified seconds

OUTPUT FORMAT:
Respond ONLY with valid JSON matching this schema:
{
  "actions": [
    {
      "action_type": "string",
      "target": "string or null",
      "parameters": {},
      "preconditions": [],
      "effects": []
    }
  ]
}

CONSTRAINTS:
- All positions must be within workspace bounds
- Never exceed maximum speed
- Never attempt to pick objects heavier than 2 kg
- If command is ambiguous, include "clarification_needed" field
- If command is impossible, return empty actions with "error" field
"""
```

### Few-Shot Examples

```python
FEW_SHOT_EXAMPLES = [
    {
        "user": "Pick up the red mug",
        "assistant": """{
  "actions": [
    {
      "action_type": "navigate_to",
      "target": null,
      "parameters": {"position": "NEAR_red_mug"},
      "preconditions": ["red_mug_visible", "path_clear"],
      "effects": ["robot_near_red_mug"]
    },
    {
      "action_type": "open_gripper",
      "target": null,
      "parameters": {},
      "preconditions": ["robot_near_red_mug"],
      "effects": ["gripper_open"]
    },
    {
      "action_type": "pick",
      "target": "red_mug",
      "parameters": {},
      "preconditions": ["gripper_open", "red_mug_reachable"],
      "effects": ["holding_red_mug"]
    }
  ],
  "success_criteria": "holding_red_mug"
}"""
    },
    {
        "user": "Put the mug on the shelf",
        "assistant": """{
  "actions": [
    {
      "action_type": "navigate_to",
      "target": null,
      "parameters": {"position": "NEAR_shelf"},
      "preconditions": ["holding_object", "path_clear"],
      "effects": ["robot_near_shelf"]
    },
    {
      "action_type": "place",
      "target": null,
      "parameters": {"position": "ON_shelf"},
      "preconditions": ["robot_near_shelf", "shelf_has_space"],
      "effects": ["object_on_shelf"]
    },
    {
      "action_type": "open_gripper",
      "target": null,
      "parameters": {},
      "preconditions": ["object_on_shelf"],
      "effects": ["gripper_empty"]
    }
  ],
  "success_criteria": "object_on_shelf AND gripper_empty"
}"""
    },
    {
        "user": "Move at 10 m/s to the door",
        "assistant": """{
  "actions": [],
  "error": "Requested speed (10 m/s) exceeds maximum (0.5 m/s)",
  "suggestion": "Command with speed <= 0.5 m/s"
}"""
    }
]
```

---

## LLM Integration {#llm-integration}

### Task Planner Node

```python
#!/usr/bin/env python3
"""LLM-based task planning node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from openai import OpenAI


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')

        # LLM client (OpenAI, local, or other)
        self.client = OpenAI()

        # Schema for validation
        self.action_schema = self.load_schema()

        # Cache for repeated commands
        self.plan_cache = {}

        # Subscribers and publishers
        self.text_sub = self.create_subscription(
            String, '/speech/text', self.text_callback, 10
        )
        self.plan_pub = self.create_publisher(
            String, '/task_planner/plan', 10
        )

        self.get_logger().info('Task planner ready')

    def text_callback(self, msg: String):
        """Process transcribed speech command."""
        command = msg.data.strip()

        # Check cache first
        if command in self.plan_cache:
            self.get_logger().info(f'Using cached plan for: "{command}"')
            plan = self.plan_cache[command]
        else:
            # Generate new plan
            plan = self.generate_plan(command)

            # Validate
            if self.validate_plan(plan):
                self.plan_cache[command] = plan
            else:
                self.get_logger().error('Generated plan failed validation')
                return

        # Publish
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)

    def generate_plan(self, command: str) -> dict:
        """Generate task plan using LLM."""
        import time
        start = time.time()

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        # Add few-shot examples
        for example in FEW_SHOT_EXAMPLES:
            messages.append({"role": "user", "content": example["user"]})
            messages.append({"role": "assistant", "content": example["assistant"]})

        # Add current command
        messages.append({"role": "user", "content": command})

        response = self.client.chat.completions.create(
            model="gpt-4",  # Or local model
            messages=messages,
            temperature=0,  # Deterministic
            response_format={"type": "json_object"}
        )

        latency = (time.time() - start) * 1000
        self.get_logger().info(f'LLM inference: {latency:.0f}ms')

        try:
            plan = json.loads(response.choices[0].message.content)
            plan['original_command'] = command
            plan['inference_latency_ms'] = latency
            return plan
        except json.JSONDecodeError:
            return {"error": "Failed to parse LLM response", "actions": []}

    def validate_plan(self, plan: dict) -> bool:
        """Validate plan against schema and constraints."""
        if "error" in plan:
            return True  # Error responses are valid

        if "actions" not in plan:
            self.get_logger().error('Plan missing "actions" field')
            return False

        for action in plan["actions"]:
            # Check action type
            if action.get("action_type") not in VALID_ACTIONS:
                self.get_logger().error(
                    f'Invalid action type: {action.get("action_type")}'
                )
                return False

            # Check required parameters
            action_spec = VALID_ACTIONS[action["action_type"]]
            params = action.get("parameters", {})
            for req_param in action_spec["required_params"]:
                if req_param not in params and action.get("target") is None:
                    self.get_logger().error(
                        f'Missing required param: {req_param}'
                    )
                    return False

        return True
```

---

## Output Validation {#validation}

### Schema Validation with Pydantic

```python
from pydantic import BaseModel, validator
from typing import List, Optional, Dict, Any

class ActionParameters(BaseModel):
    position: Optional[List[float]] = None
    orientation: Optional[List[float]] = None
    speed: Optional[float] = None
    force: Optional[float] = None

    @validator('position')
    def validate_position(cls, v):
        if v is not None and isinstance(v, list) and len(v) == 3:
            x, y, z = v
            if not (0 <= x <= 5 and 0 <= y <= 5 and 0 <= z <= 2):
                raise ValueError('Position outside workspace')
        return v

    @validator('speed')
    def validate_speed(cls, v):
        if v is not None and v > 0.5:
            raise ValueError(f'Speed {v} exceeds maximum 0.5 m/s')
        return v

class Action(BaseModel):
    action_type: str
    target: Optional[str] = None
    parameters: ActionParameters = ActionParameters()
    preconditions: List[str] = []
    effects: List[str] = []

class TaskPlan(BaseModel):
    actions: List[Action]
    success_criteria: Optional[str] = None
    abort_conditions: List[str] = []
    error: Optional[str] = None
    clarification_needed: Optional[str] = None

def validate_task_plan(plan_dict: dict) -> TaskPlan:
    """Validate and parse task plan."""
    return TaskPlan(**plan_dict)
```

### Handling Invalid Responses

```python
def safe_generate_plan(self, command: str, max_retries: int = 2) -> dict:
    """Generate plan with retry on validation failure."""
    for attempt in range(max_retries + 1):
        plan = self.generate_plan(command)

        try:
            validated = validate_task_plan(plan)
            return validated.dict()
        except Exception as e:
            self.get_logger().warn(
                f'Validation failed (attempt {attempt + 1}): {e}'
            )

            if attempt < max_retries:
                # Regenerate with error feedback
                continue

    return {"error": "Failed to generate valid plan", "actions": []}
```

---

## Caching Strategy {#caching}

### Semantic Caching

```python
from sentence_transformers import SentenceTransformer
import numpy as np

class SemanticCache:
    def __init__(self, threshold: float = 0.95):
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')
        self.cache = {}  # embedding -> plan
        self.embeddings = []
        self.threshold = threshold

    def get(self, command: str) -> Optional[dict]:
        """Get cached plan for semantically similar command."""
        embedding = self.encoder.encode(command)

        for i, cached_emb in enumerate(self.embeddings):
            similarity = np.dot(embedding, cached_emb) / (
                np.linalg.norm(embedding) * np.linalg.norm(cached_emb)
            )
            if similarity > self.threshold:
                return list(self.cache.values())[i]

        return None

    def put(self, command: str, plan: dict):
        """Cache plan with command embedding."""
        embedding = self.encoder.encode(command)
        self.embeddings.append(embedding)
        self.cache[command] = plan
```

---

## Summary

This chapter covered LLM-based task planning:

1. **LLM strengths**: Semantic understanding, command decomposition, variation handling.

2. **LLM limitations**: No physical reasoning, hallucination, inconsistency—constraints must be explicit.

3. **Structured outputs** with JSON schemas enable downstream validation.

4. **Prompt engineering** includes capability constraints, few-shot examples, and output format.

5. **Validation** rejects invalid plans before execution.

6. **Caching** reduces latency for repeated commands.

---

## Safety Callout

**LLMs can generate dangerous commands:**
- "Move at maximum speed through the warehouse"
- "Pick up the 50 kg crate"
- "Navigate through the glass door"

The safety filter (Chapter 5) is the final defense, but prompts should include constraints to reduce dangerous outputs.

---

## Self-Assessment Questions

1. **Limitation Awareness**: An LLM plans a path that goes through a wall. Why did this happen? How would you prevent it?

2. **Schema Design**: Design an action schema for a robot that can also open doors. What new action types and parameters?

3. **Few-Shot Design**: Write a few-shot example for handling an impossible command: "Fly to the ceiling"

4. **Validation**: Your validator accepts all plans because it only checks action types. What additional validations are critical?

5. **Caching Trade-off**: "Pick up the red mug" and "Pick up the blue mug" are semantically similar. Should they share a cache entry? Why or why not?

---

## What's Next

In [Chapter 3: Grounding](/module-5/chapter-3-grounding), you'll implement the layer that maps these symbolic task plans to executable ROS 2 actions with real-world verification.
