---
id: hardware-requirements
title: "Appendix B: Hardware Requirements"
sidebar_label: "B. Hardware Requirements"
sidebar_position: 3
---

# Appendix B: Hardware Requirements

This appendix details the hardware requirements for running the course materials, from minimal configurations for learning to recommended setups for production work.

---

## Quick Reference {#quick-reference}

| Configuration | GPU | RAM | Storage | Use Case |
|--------------|-----|-----|---------|----------|
| Minimum | GTX 1660 (6GB) | 16 GB | 50 GB SSD | Basic ROS 2, simple Gazebo |
| Recommended | RTX 3070 (8GB) | 32 GB | 100 GB NVMe | Full course, Isaac Sim basics |
| Production | RTX 4080+ (16GB) | 64 GB | 200 GB NVMe | Isaac Sim + VLA + development |

---

## Detailed Requirements by Module {#by-module}

### Module 1: Physical AI Foundations

**Hardware**: Minimal requirements
- Any modern CPU
- 8 GB RAM
- No GPU required (theory-focused)

### Module 2: ROS 2 Fundamentals

**Hardware**: Light requirements
- 4+ CPU cores
- 16 GB RAM
- No dedicated GPU required

**Software**:
- Ubuntu 22.04 LTS
- ROS 2 Humble

### Module 3: Simulation & Digital Twin

**Hardware**: Moderate requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4 cores | 8+ cores |
| RAM | 16 GB | 32 GB |
| GPU | GTX 1660 (6GB) | RTX 3060 (12GB) |
| Storage | 50 GB SSD | 100 GB NVMe |

**Why GPU Matters**:
- Gazebo rendering: 1-2 GB VRAM
- Physics simulation: CPU + basic GPU
- Multiple robot simulation: More VRAM for textures

### Module 4: NVIDIA Isaac Ecosystem

**Hardware**: Significant requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 8 cores | 12+ cores |
| RAM | 32 GB | 64 GB |
| GPU | RTX 3070 (8GB) | RTX 4080 (16GB) |
| Storage | 100 GB NVMe | 200 GB NVMe |

**Critical Notes**:
- Isaac Sim requires NVIDIA GPU with RTX (ray tracing)
- GeForce 10-series and older NOT supported
- AMD GPUs NOT supported for Isaac Sim

**VRAM Usage**:
| Task | Typical VRAM |
|------|-------------|
| Isaac Sim idle | 4 GB |
| Simple scene | 6-8 GB |
| Complex scene + sensors | 10-14 GB |
| Isaac ROS perception | +2-4 GB |

### Module 5: Vision-Language-Action

**Hardware**: Heavy requirements for local models

| Component | Minimum (API) | Recommended (Local) |
|-----------|--------------|-------------------|
| CPU | 8 cores | 12+ cores |
| RAM | 32 GB | 64 GB |
| GPU | RTX 3070 (8GB) | RTX 4090 (24GB) |
| Storage | 100 GB NVMe | 300 GB NVMe |

**GPU Memory for Local Models**:
| Model | VRAM Required |
|-------|--------------|
| Whisper base | 1 GB |
| Whisper small | 2 GB |
| LLaVA 7B | 8-10 GB |
| LLaVA 13B | 14-16 GB |
| Local LLM 7B | 6-8 GB |

**API Alternative**: Use cloud APIs (OpenAI, Anthropic) to reduce local requirements.

### Capstone: Integrated System

**Hardware**: Combines all requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 12 cores | 16+ cores |
| RAM | 64 GB | 128 GB |
| GPU | RTX 3090 (24GB) | RTX 4090 (24GB) |
| Storage | 200 GB NVMe | 500 GB NVMe |

**Running Everything Simultaneously**:
- Isaac Sim: 8-12 GB VRAM
- Isaac ROS perception: 2-4 GB VRAM
- VLA models: 4-8 GB VRAM
- Total: 14-24 GB VRAM recommended

---

## GPU Selection Guide {#gpu-guide}

### NVIDIA GeForce (Consumer)

| GPU | VRAM | Isaac Sim | VLA Local | Recommendation |
|-----|------|-----------|-----------|----------------|
| GTX 1660 | 6 GB | No | Limited | Module 2-3 only |
| RTX 3060 | 12 GB | Basic | Whisper | Learning |
| RTX 3070 | 8 GB | Yes | Limited VLM | Recommended minimum |
| RTX 3080 | 10 GB | Yes | LLaVA 7B | Good balance |
| RTX 3090 | 24 GB | Yes | All models | Excellent |
| RTX 4070 | 12 GB | Yes | LLaVA 7B | Good |
| RTX 4080 | 16 GB | Yes | LLaVA 13B | Very good |
| RTX 4090 | 24 GB | Yes | All models | Best consumer |

### NVIDIA Workstation

| GPU | VRAM | Notes |
|-----|------|-------|
| RTX A4000 | 16 GB | Professional features |
| RTX A5000 | 24 GB | ECC memory |
| RTX A6000 | 48 GB | Maximum headroom |

### Cloud Alternatives

| Provider | Instance Type | GPU | Cost/hr |
|----------|--------------|-----|---------|
| AWS | g5.xlarge | A10G (24GB) | ~$1.00 |
| GCP | n1-standard-8 + T4 | T4 (16GB) | ~$0.70 |
| Lambda Labs | GPU Cloud | RTX 3090 | ~$0.50 |
| RunPod | Community | RTX 3090 | ~$0.40 |

---

## Storage Requirements {#storage}

### Software Installation Sizes

| Software | Installation Size |
|----------|------------------|
| Ubuntu 22.04 | 10 GB |
| ROS 2 Humble | 5 GB |
| Gazebo Harmonic | 3 GB |
| Isaac Sim | 30-50 GB |
| Isaac ROS | 10 GB |
| NVIDIA drivers | 2 GB |
| ML models | 5-30 GB |
| Course materials | 5 GB |

### Recommended Partitioning

```
Total: 200 GB NVMe SSD
├── / (root): 50 GB
├── /home: 100 GB
│   ├── ros2_ws: 20 GB
│   ├── isaac-sim: 50 GB
│   └── models: 30 GB
└── swap: 32 GB (match RAM for hibernation)
```

### Storage Performance

| Type | Read Speed | Adequate For |
|------|------------|--------------|
| HDD | 100 MB/s | Not recommended |
| SATA SSD | 500 MB/s | Basic use |
| NVMe SSD | 3000+ MB/s | Recommended |

**Note**: Isaac Sim asset loading benefits significantly from NVMe speeds.

---

## Memory (RAM) Considerations {#memory}

### Usage Breakdown

| Application | Typical RAM |
|-------------|------------|
| Ubuntu desktop | 2 GB |
| ROS 2 nodes | 1-4 GB |
| Gazebo | 4-8 GB |
| Isaac Sim | 16-32 GB |
| Browser (docs) | 2-4 GB |
| IDE | 2-4 GB |

### Configuration Recommendations

| Scenario | RAM | Notes |
|----------|-----|-------|
| Learning | 16 GB | Tight, close other apps |
| Development | 32 GB | Comfortable |
| Full stack | 64 GB | Isaac Sim + ROS + IDE |
| Production | 128 GB | Headroom for large scenes |

### Swap Configuration

For systems with limited RAM:

```bash
# Check current swap
swapon --show

# Create 16GB swap file if needed
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## Recommended Configurations {#configurations}

### Budget Build (~$1,500 USD)

**For**: Learning Modules 1-4, light VLA

| Component | Specification |
|-----------|--------------|
| CPU | AMD Ryzen 5 5600X (6 cores) |
| GPU | NVIDIA RTX 3060 12GB |
| RAM | 32 GB DDR4-3200 |
| Storage | 500 GB NVMe SSD |
| OS | Ubuntu 22.04 |

### Mid-Range Build (~$2,500 USD)

**For**: Full course, comfortable development

| Component | Specification |
|-----------|--------------|
| CPU | AMD Ryzen 7 5800X (8 cores) |
| GPU | NVIDIA RTX 3080 10GB |
| RAM | 64 GB DDR4-3600 |
| Storage | 1 TB NVMe SSD |
| OS | Ubuntu 22.04 |

### High-End Build (~$4,000+ USD)

**For**: Production development, multiple simulations

| Component | Specification |
|-----------|--------------|
| CPU | AMD Ryzen 9 5950X (16 cores) |
| GPU | NVIDIA RTX 4090 24GB |
| RAM | 128 GB DDR4-3600 |
| Storage | 2 TB NVMe SSD |
| OS | Ubuntu 22.04 |

---

## Laptop Considerations {#laptop}

### Requirements for Mobile Development

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 8 cores | 12 cores |
| GPU | RTX 3070 Laptop | RTX 4080 Laptop |
| RAM | 32 GB | 64 GB |
| Storage | 512 GB NVMe | 1 TB NVMe |

### Laptop Limitations

| Issue | Impact | Mitigation |
|-------|--------|------------|
| Thermal throttling | Reduced performance | Cooling pad, power mode |
| VRAM (usually less) | Scene limitations | Optimize assets |
| Power consumption | Battery drain | AC power required |
| Display (1080p typical) | Less screen space | External monitor |

### Recommended Laptops

- ASUS ROG Zephyrus
- Lenovo Legion Pro
- MSI GE Series
- Razer Blade 15/17

---

## Verification Commands {#verification}

### Check GPU

```bash
# NVIDIA driver and CUDA
nvidia-smi

# GPU memory
nvidia-smi --query-gpu=memory.total --format=csv
```

### Check CPU and RAM

```bash
# CPU info
lscpu | grep "Model name\|CPU(s)\|Thread"

# Memory
free -h
```

### Check Storage

```bash
# Disk space
df -h

# Storage speed (rough test)
dd if=/dev/zero of=/tmp/test bs=1M count=1024 conv=fdatasync
```

### Verify Isaac Sim Compatibility

```bash
# Check RTX support
nvidia-smi -q | grep "CUDA Version"

# Should show CUDA 11.x or higher for Isaac Sim
```

---

## Summary

For the full course experience:

1. **GPU**: RTX 3070 minimum, RTX 4080+ recommended
2. **RAM**: 32 GB minimum, 64 GB recommended
3. **Storage**: 100 GB NVMe minimum, 200 GB recommended
4. **Cloud**: Viable alternative for heavy workloads

Start with what you have—many modules work with modest hardware. Upgrade or use cloud resources for Module 4-5 and Capstone if needed.
