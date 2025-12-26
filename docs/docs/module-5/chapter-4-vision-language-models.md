---
id: chapter-4-vision-language-models
title: "Chapter 4: Vision-Language Models"
sidebar_label: "4. Vision-Language Models"
sidebar_position: 5
---

# Chapter 4: Vision-Language Models for Scene Understanding

## Chapter Goal

By the end of this chapter, you will be able to **integrate vision-language models (VLMs) for scene understanding**, enabling the robot to answer questions about its camera view and ground visual references in natural language commands.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Explain VLM architecture and capabilities for robotics |
| 4.2 | Deploy a VLM for robot camera image understanding |
| 4.3 | Design prompts that extract robotics-relevant information |
| 4.4 | Implement VLM-based object identification for grounding |
| 4.5 | Evaluate VLM reliability and latency |
| 4.6 | Integrate VLM with the grounding layer |

---

## VLMs vs Object Detection {#comparison}

### Complementary Capabilities

| Capability | Object Detection | VLM |
|------------|------------------|-----|
| Fixed classes | Yes | No (open vocabulary) |
| Spatial relations | No | Yes |
| Object descriptions | No | Yes |
| Speed | Fast (10-50ms) | Slow (1-10s) |
| Reliability | High (trained) | Variable (hallucination) |
| Arbitrary queries | No | Yes |

**Best Practice**: Use both. Object detection for reliable, fast detection. VLM for flexible queries and disambiguation.

### When to Use VLM

```
"Pick up the red mug" → Object detection (has "mug" class)
"Pick up the thing next to the keyboard" → VLM (spatial relation)
"Pick up what I'm pointing at" → VLM (requires understanding)
"Pick up the broken one" → VLM (attribute not in detector)
```

---

## VLM Architecture Overview {#architecture}

### How VLMs Work

```
Image ──► Vision Encoder ──► Image Embeddings
                                    │
                                    ▼
Text Query ──► Text Encoder ──► Fusion ──► LLM Decoder ──► Response
```

**Vision Encoder**: Processes image to feature vectors (e.g., CLIP, ViT)

**Fusion**: Combines image and text representations

**LLM Decoder**: Generates text response conditioned on image and query

### Common VLMs

| Model | Access | Latency | Quality |
|-------|--------|---------|---------|
| GPT-4V | API | 2-5s | Best |
| LLaVA | Local | 1-3s | Good |
| BLIP-2 | Local | 1-2s | Good |
| MiniGPT-4 | Local | 1-3s | Moderate |

---

## VLM Deployment {#deployment}

### Using LLaVA (Local)

```python
#!/usr/bin/env python3
"""Local VLM deployment with LLaVA."""

import torch
from transformers import AutoProcessor, LlavaForConditionalGeneration
from PIL import Image
import numpy as np
import time


class VLMNode:
    def __init__(self, model_name: str = "llava-hf/llava-1.5-7b-hf"):
        # Load model
        self.processor = AutoProcessor.from_pretrained(model_name)
        self.model = LlavaForConditionalGeneration.from_pretrained(
            model_name,
            torch_dtype=torch.float16,
            device_map="auto"
        )

    def query(self, image: np.ndarray, question: str) -> tuple[str, float]:
        """
        Query VLM about image.

        Args:
            image: RGB image as numpy array
            question: Natural language question

        Returns:
            (answer, latency_ms)
        """
        start = time.time()

        # Convert to PIL
        pil_image = Image.fromarray(image)

        # Format prompt
        prompt = f"USER: <image>\n{question}\nASSISTANT:"

        # Process
        inputs = self.processor(
            text=prompt,
            images=pil_image,
            return_tensors="pt"
        ).to(self.model.device)

        # Generate
        outputs = self.model.generate(
            **inputs,
            max_new_tokens=256,
            do_sample=False  # Deterministic
        )

        # Decode
        response = self.processor.decode(outputs[0], skip_special_tokens=True)
        answer = response.split("ASSISTANT:")[-1].strip()

        latency = (time.time() - start) * 1000
        return answer, latency
```

### Using GPT-4V (API)

```python
import base64
from openai import OpenAI
import numpy as np
import cv2
import time


class GPT4VNode:
    def __init__(self):
        self.client = OpenAI()

    def query(self, image: np.ndarray, question: str) -> tuple[str, float]:
        """Query GPT-4V about image."""
        start = time.time()

        # Encode image to base64
        _, buffer = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        base64_image = base64.b64encode(buffer).decode('utf-8')

        response = self.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": question},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=300
        )

        answer = response.choices[0].message.content
        latency = (time.time() - start) * 1000

        return answer, latency
```

---

## Prompt Engineering for Robotics {#prompts}

### Scene Understanding Prompts

```python
SCENE_PROMPTS = {
    "object_list": """List all objects visible in this image.
Format your response as a JSON array of objects with fields:
- name: object name
- color: primary color
- location: general location (left, right, center, etc.)
Example: [{"name": "mug", "color": "red", "location": "center"}]""",

    "object_location": """Where is the {object} in this image?
Describe its location relative to other objects.
If not visible, respond with "NOT_FOUND".""",

    "spatial_relation": """Is there {relation} {object} in this image?
Examples: "a cup on the table", "something next to the keyboard"
Respond with YES or NO, then briefly explain.""",

    "reference_resolution": """The user said "{reference}".
Looking at this image, which specific object are they referring to?
Describe it precisely (color, location, distinguishing features).
If ambiguous or unclear, list all possibilities.""",
}
```

### Object Identification Prompts

```python
def create_identification_prompt(reference: str) -> str:
    return f"""You are helping a robot identify objects.

The user wants the robot to interact with: "{reference}"

Looking at this image:
1. Can you identify what the user is referring to?
2. If yes, describe its exact location (which part of the image: top-left, center, etc.)
3. If there are multiple possibilities, list them all
4. If the object is not visible, say "NOT_FOUND"

Response format:
FOUND: [yes/no/ambiguous]
LOCATION: [description or N/A]
CANDIDATES: [list if ambiguous, empty otherwise]
CONFIDENCE: [high/medium/low]"""
```

---

## Integration with Grounding {#grounding-integration}

### VLM-Enhanced Object Resolution

```python
class VLMEnhancedResolver:
    def __init__(self, vlm, object_detector):
        self.vlm = vlm
        self.detector = object_detector

    def resolve(
        self,
        reference: str,
        image: np.ndarray
    ) -> Optional[DetectedObject]:
        """
        Resolve reference using VLM + detector.

        Strategy:
        1. Try detector first (fast, reliable)
        2. If ambiguous/not found, use VLM
        3. Match VLM response to detections
        """
        # Try detector first
        detections = self.detector.detect(image)
        detector_match = self.match_reference(reference, detections)

        if detector_match and detector_match['confidence'] > 0.8:
            return detector_match['object']

        # Use VLM for disambiguation
        vlm_response, latency = self.vlm.query(
            image,
            f"Where is {reference} in this image? "
            "Describe its exact position and appearance."
        )

        # Match VLM description to detections
        for det in detections:
            if self.vlm_matches_detection(vlm_response, det, image):
                return det

        return None

    def vlm_matches_detection(
        self,
        vlm_description: str,
        detection: DetectedObject,
        image: np.ndarray
    ) -> bool:
        """Check if VLM description matches detection."""
        # Get detection location description
        img_h, img_w = image.shape[:2]
        det_x = detection.bbox_2d[0] + detection.bbox_2d[2] / 2
        det_y = detection.bbox_2d[1] + detection.bbox_2d[3] / 2

        location_terms = []
        if det_x < img_w / 3:
            location_terms.append("left")
        elif det_x > 2 * img_w / 3:
            location_terms.append("right")
        else:
            location_terms.append("center")

        if det_y < img_h / 3:
            location_terms.append("top")
        elif det_y > 2 * img_h / 3:
            location_terms.append("bottom")

        # Check if VLM description mentions these locations
        vlm_lower = vlm_description.lower()
        return any(term in vlm_lower for term in location_terms)
```

---

## VLM Limitations {#limitations}

### Hallucination Examples

| Query | VLM Response | Reality |
|-------|--------------|---------|
| "How many mugs?" | "There are 3 mugs" | Actually 2 |
| "Is there a cat?" | "Yes, on the left" | No cat present |
| "What color is the cup?" | "Blue" | Actually white |

### Reliability Evaluation

```python
def evaluate_vlm_reliability(vlm, test_set):
    """Evaluate VLM accuracy on robotics queries."""
    results = {
        "object_presence": [],
        "counting": [],
        "color_accuracy": [],
        "spatial_accuracy": [],
    }

    for image, ground_truth in test_set:
        # Object presence
        for obj in ground_truth["objects"]:
            response, _ = vlm.query(image, f"Is there a {obj['name']} in this image?")
            correct = "yes" in response.lower()
            results["object_presence"].append(correct)

        # Counting
        for category, count in ground_truth["counts"].items():
            response, _ = vlm.query(image, f"How many {category}s are there?")
            # Parse number from response
            predicted = extract_number(response)
            results["counting"].append(predicted == count)

    return {k: np.mean(v) for k, v in results.items()}
```

Expected results:

| Task | Typical Accuracy |
|------|-----------------|
| Object presence | 80-90% |
| Counting (≤3) | 70-85% |
| Counting (>3) | 40-60% |
| Color | 75-90% |
| Spatial relations | 60-80% |

---

## Latency Considerations {#latency}

### VLM Latency in VLA Pipeline

```
Speech (200ms) + LLM (500ms) + VLM (2000ms) + Grounding (50ms) = 2750ms
                                    ↑
                              Dominates!
```

### Latency Mitigation

```python
class CachedVLM:
    def __init__(self, vlm):
        self.vlm = vlm
        self.scene_cache = {}  # image_hash -> scene description
        self.cache_timeout = 5.0  # seconds

    def query(self, image: np.ndarray, question: str) -> tuple[str, float]:
        """Query with scene caching."""
        # For general scene queries, use cache
        if self.is_scene_query(question):
            scene = self.get_or_update_scene(image)
            # Answer from cached scene description
            return self.answer_from_scene(scene, question), 0

        # For specific queries, call VLM
        return self.vlm.query(image, question)

    def get_or_update_scene(self, image: np.ndarray) -> dict:
        """Get cached scene or update."""
        image_hash = self.hash_image(image)

        if image_hash in self.scene_cache:
            cached = self.scene_cache[image_hash]
            if time.time() - cached['time'] < self.cache_timeout:
                return cached['scene']

        # Generate new scene description
        scene_response, _ = self.vlm.query(
            image,
            "Describe all objects in this image with their colors and positions."
        )

        self.scene_cache[image_hash] = {
            'scene': self.parse_scene(scene_response),
            'time': time.time()
        }

        return self.scene_cache[image_hash]['scene']
```

---

## Summary

This chapter covered VLMs for robot scene understanding:

1. **VLM capabilities** complement object detection with flexible queries and spatial reasoning.

2. **Deployment options** include local (LLaVA) and API (GPT-4V) with different latency/quality tradeoffs.

3. **Prompt engineering** extracts robotics-relevant information (objects, locations, relations).

4. **Integration with grounding** uses VLM to resolve ambiguous references.

5. **Limitations** include hallucination and counting errors—VLMs are not reliable for safety-critical perception.

6. **Latency management** via caching helps with real-time constraints.

---

## Safety Callout

**Never trust VLM for safety-critical decisions:**
- VLM: "The path is clear" → Might hallucinate
- VLM: "No humans in frame" → Could miss person

Use traditional perception (LiDAR, depth) for safety. VLM assists with understanding, not safety verification.

---

## Self-Assessment Questions

1. **VLM vs Detection**: When would you use VLM instead of object detection? When detection instead of VLM?

2. **Hallucination Risk**: You ask "Is there a person?" and VLM says "No". The robot proceeds and hits a person. What went wrong? How would you prevent this?

3. **Latency Trade-off**: VLM adds 2 seconds to response time. When is this acceptable? When unacceptable?

4. **Prompt Design**: Design a prompt to identify "the thing the user is holding" from a camera image.

5. **Integration**: Your grounding layer resolved to the wrong object because VLM said it was "on the left" when it was actually center-left. How would you improve matching accuracy?

---

## What's Next

In [Chapter 5: Safety Constraints](/module-5/chapter-5-safety-constraints), you'll implement the critical safety layer that prevents dangerous AI-generated commands from reaching robot actuators.
