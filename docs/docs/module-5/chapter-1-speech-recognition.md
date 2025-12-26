---
id: chapter-1-speech-recognition
title: "Chapter 1: Speech Recognition"
sidebar_label: "1. Speech Recognition"
sidebar_position: 2
---

# Chapter 1: Speech Recognition for Robot Command

## Chapter Goal

By the end of this chapter, you will be able to **integrate Whisper for voice command input**, understanding its architecture, deployment options, latency characteristics, and robustness to acoustic noise in robotic environments.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 1.1 | Explain Whisper's architecture conceptually |
| 1.2 | Deploy Whisper with appropriate model size selection |
| 1.3 | Implement an audio capture pipeline with voice activity detection |
| 1.4 | Configure Whisper for robot command recognition |
| 1.5 | Evaluate robustness to environmental noise |

---

## Why Voice Commands for Robots?

Voice is natural for humans but challenging for robots:

| Input Method | Pros | Cons |
|--------------|------|------|
| Keyboard/GUI | Precise, reliable | Requires hands, screen |
| Joystick | Real-time control | Limited vocabulary |
| **Voice** | Hands-free, natural | Noise, ambiguity, latency |

Voice enables commands like "pick up the red mug" that would require complex menus otherwise.

---

## Whisper Architecture Overview {#architecture}

Whisper is OpenAI's speech-to-text transformer model.

### Processing Pipeline

```
Audio Waveform ──► Mel Spectrogram ──► Encoder ──► Decoder ──► Text
      │                  │                │            │
   16kHz            Log-mel         Transformer    Autoregressive
   mono             features         layers         token generation
```

### Key Components

**Mel Spectrogram**: Converts audio to frequency-time representation matching human hearing.

**Encoder**: Processes the spectrogram, outputs context vectors.

**Decoder**: Generates text tokens autoregressively.

### Model Sizes

| Model | Parameters | VRAM | Speed | WER |
|-------|-----------|------|-------|-----|
| tiny | 39M | ~1 GB | Fastest | Higher |
| base | 74M | ~1 GB | Fast | Good |
| small | 244M | ~2 GB | Moderate | Better |
| medium | 769M | ~5 GB | Slower | Best |
| large | 1550M | ~10 GB | Slowest | Best |

**Recommendation**: Start with `base` for real-time robot commands; use `small` if accuracy insufficient.

---

## Deployment {#deployment}

### Local Inference with faster-whisper

```python
#!/usr/bin/env python3
"""Whisper deployment for robot commands."""

from faster_whisper import WhisperModel
import numpy as np
import time

class WhisperASR:
    def __init__(self, model_size: str = "base"):
        # Use faster-whisper for optimized inference
        self.model = WhisperModel(
            model_size,
            device="cuda",
            compute_type="float16"  # Half precision for speed
        )
        self.sample_rate = 16000

    def transcribe(self, audio: np.ndarray) -> tuple:
        """
        Transcribe audio to text.

        Args:
            audio: Float32 numpy array of audio samples at 16kHz

        Returns:
            (text, latency_ms)
        """
        start = time.time()

        segments, info = self.model.transcribe(
            audio,
            language="en",
            task="transcribe",
            beam_size=1,  # Greedy for speed
            vad_filter=True,  # Filter silence
        )

        text = " ".join([seg.text for seg in segments])
        latency_ms = (time.time() - start) * 1000

        return text.strip(), latency_ms

# Usage
asr = WhisperASR(model_size="base")
text, latency = asr.transcribe(audio_samples)
print(f"Transcribed: '{text}' in {latency:.0f}ms")
```

### Latency Measurements

| Model | 2s Audio | 5s Audio | 10s Audio |
|-------|----------|----------|-----------|
| tiny | 50ms | 80ms | 150ms |
| base | 100ms | 200ms | 400ms |
| small | 250ms | 500ms | 1000ms |
| medium | 600ms | 1200ms | 2500ms |

**Physical Grounding**: For responsive robot interaction, target < 500ms total speech-to-response. With base model at ~200ms, you have ~300ms for planning and execution.

---

## Audio Capture Pipeline {#audio-pipeline}

### ROS 2 Audio Capture Node

```python
#!/usr/bin/env python3
"""Audio capture node for robot voice commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
from collections import deque
import threading


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Parameters
        self.sample_rate = 16000
        self.chunk_duration = 0.1  # 100ms chunks
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)

        # Buffer for audio
        self.audio_buffer = deque(maxlen=int(10 * self.sample_rate))  # 10s max

        # VAD state
        self.is_speaking = False
        self.silence_threshold = 0.01
        self.speech_frames = []

        # Publishers
        self.text_pub = self.create_publisher(String, '/speech/text', 10)

        # Whisper
        from faster_whisper import WhisperModel
        self.whisper = WhisperModel("base", device="cuda")

        # Start audio stream
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.float32,
            blocksize=self.chunk_samples,
            callback=self.audio_callback
        )
        self.stream.start()

        self.get_logger().info('Audio capture started')

    def audio_callback(self, indata, frames, time_info, status):
        """Called for each audio chunk."""
        audio = indata[:, 0]  # Mono

        # Simple energy-based VAD
        energy = np.sqrt(np.mean(audio ** 2))

        if energy > self.silence_threshold:
            if not self.is_speaking:
                self.get_logger().info('Speech started')
                self.is_speaking = True
                self.speech_frames = []

            self.speech_frames.append(audio.copy())

        elif self.is_speaking:
            # Silence after speech - transcribe
            self.is_speaking = False
            self.get_logger().info('Speech ended, transcribing...')

            # Concatenate speech frames
            audio_data = np.concatenate(self.speech_frames)

            # Transcribe in separate thread
            threading.Thread(
                target=self.transcribe_and_publish,
                args=(audio_data,)
            ).start()

    def transcribe_and_publish(self, audio: np.ndarray):
        """Transcribe audio and publish result."""
        segments, _ = self.whisper.transcribe(audio, language="en")
        text = " ".join([seg.text for seg in segments]).strip()

        if text:
            msg = String()
            msg.data = text
            self.text_pub.publish(msg)
            self.get_logger().info(f'Transcribed: "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Voice Activity Detection (VAD)

The simple energy-based VAD above works for quiet environments. For noisy robot environments:

```python
# WebRTC VAD for better performance
import webrtcvad

class RobustVAD:
    def __init__(self, aggressiveness: int = 2):
        # 0: least aggressive, 3: most aggressive
        self.vad = webrtcvad.Vad(aggressiveness)
        self.sample_rate = 16000
        self.frame_duration_ms = 30  # 10, 20, or 30 ms

    def is_speech(self, audio_frame: bytes) -> bool:
        """Check if audio frame contains speech."""
        return self.vad.is_speech(audio_frame, self.sample_rate)
```

---

## Prompt Conditioning {#conditioning}

### Domain-Specific Recognition

Whisper can be conditioned on expected vocabulary:

```python
def transcribe_robot_command(self, audio: np.ndarray) -> str:
    """Transcribe with robot command conditioning."""
    segments, _ = self.whisper.transcribe(
        audio,
        language="en",
        task="transcribe",
        initial_prompt=(
            "Robot commands: pick up, put down, move to, "
            "navigate to, stop, go, grab, release, "
            "shelf, table, mug, cup, box, "
            "red, blue, green, left, right"
        )
    )
    return " ".join([seg.text for seg in segments]).strip()
```

### Vocabulary Boosting

| Without Conditioning | With Conditioning |
|---------------------|-------------------|
| "Pickup the read mug" | "Pick up the red mug" |
| "Navigate to shelf too" | "Navigate to shelf two" |
| "Stop their" | "Stop there" |

---

## Noise Robustness {#noise}

### Robot Environmental Noise

| Noise Source | Frequency Range | Impact on ASR |
|--------------|-----------------|---------------|
| DC motors | 100-500 Hz | Moderate |
| Servo motors | 1-5 kHz | High |
| HVAC | 50-200 Hz | Low |
| Background speech | 100-3000 Hz | High |

### Mitigation Strategies

**Directional Microphone**:
```python
# Use USB directional mic pointed at user
# Reduces off-axis noise by 10-20 dB
```

**Noise Gate**:
```python
def apply_noise_gate(audio: np.ndarray, threshold: float = 0.02) -> np.ndarray:
    """Zero samples below threshold."""
    return np.where(np.abs(audio) < threshold, 0, audio)
```

**Spectral Subtraction** (for stationary noise):
```python
import scipy.signal as signal

def spectral_subtract(audio: np.ndarray, noise_profile: np.ndarray) -> np.ndarray:
    """Subtract estimated noise spectrum."""
    # Compute spectrograms
    f, t, Zxx = signal.stft(audio)
    f_n, t_n, Zxx_n = signal.stft(noise_profile)

    # Subtract noise magnitude (keep phase)
    noise_mag = np.mean(np.abs(Zxx_n), axis=1, keepdims=True)
    cleaned_mag = np.maximum(np.abs(Zxx) - noise_mag, 0)
    cleaned = cleaned_mag * np.exp(1j * np.angle(Zxx))

    # Inverse STFT
    _, cleaned_audio = signal.istft(cleaned)
    return cleaned_audio
```

### Noise Robustness Testing

```python
def evaluate_noise_robustness(asr, test_commands, noise_levels):
    """Evaluate ASR accuracy at different noise levels."""
    results = {}

    for snr_db in noise_levels:
        correct = 0
        for command, audio in test_commands:
            # Add noise at specified SNR
            noisy_audio = add_noise(audio, snr_db)
            transcribed = asr.transcribe(noisy_audio)

            if transcribed.lower() == command.lower():
                correct += 1

        accuracy = correct / len(test_commands)
        results[snr_db] = accuracy
        print(f"SNR {snr_db} dB: {accuracy:.1%} accuracy")

    return results
```

Expected results:

| SNR | Accuracy |
|-----|----------|
| Clean | 95%+ |
| 20 dB | 90%+ |
| 10 dB | 75-85% |
| 5 dB | 50-70% |

---

## Summary

This chapter covered speech recognition for robot commands:

1. **Whisper architecture** converts audio to text via mel spectrograms and transformer encoder-decoder.

2. **Model size selection** trades accuracy for latency; `base` is good for real-time robot commands.

3. **Audio capture pipeline** with VAD segments speech for transcription.

4. **Prompt conditioning** improves accuracy on robot-specific vocabulary.

5. **Noise robustness** requires mitigation strategies for real robot environments.

---

## Self-Assessment Questions

1. **Model Selection**: Your robot needs < 300ms speech-to-text latency. Which Whisper model size would you choose?

2. **Latency Budget**: Speech capture takes 2s (user speaking). Whisper takes 200ms. LLM will take 500ms. Is total latency acceptable for interactive command?

3. **VAD Tuning**: Your robot's motors create continuous 50 Hz hum. How would you adjust VAD to avoid false speech detection?

4. **Noise Impact**: The robot is in a warehouse with forklifts. SNR is approximately 10 dB. What accuracy can you expect? What would help?

5. **Prompt Conditioning**: Design an initial prompt for a warehouse robot that picks items from shelves. What vocabulary would you include?

---

## What's Next

In [Chapter 2: LLM Task Planning](/module-5/chapter-2-llm-task-planning), you'll design prompts that translate natural language commands into structured robot task plans.
