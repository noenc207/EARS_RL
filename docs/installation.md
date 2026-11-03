# EARS-RL Installation & Environment Setup Guide

## 1. Prerequisites

- Python 3.11+ (tested on Python 3.11.9 and 3.12).
- Node.js v20+ / v24+ (for MCP tooling: Context7, Filesystem).
- Git 2.40+.

## 2. Standard Development Installation

Clone the repository and install in editable development mode:

```bash
cd D:/EARS-RL
pip install -e ".[dev]"
```

Verify installation:

```bash
ears-rl --version
ears-rl doctor
```

## 3. Optional PyTorch & CUDA Setup

For neural network training and policy inference:

```bash
pip install torch>=2.1.0 --index-url https://download.pytorch.org/whl/cu121
```

## 4. Optional NVIDIA Isaac Lab Integration

NVIDIA Isaac Lab requires an Isaac Sim environment. Follow the official NVIDIA installation protocol:
https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

Verify simulator detection:

```bash
ears-rl sim check
```
